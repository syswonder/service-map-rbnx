#!/usr/bin/env python3
"""Real ROS supervisor, scan gate, verified initialization and live checkpoints.

Kinematic Ranger Mini lidar replay. Truth is used only for controlled initial
seeds and evaluation; automatic recovery uses the real static matcher/filter.
Run in an isolated container/domain. No physical robot commands are published.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import urllib.request
from types import SimpleNamespace as NS

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src'))
from replay import Replay
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from mapping_rbnx import engines,lifecycle,localizers,map_ops
from mapping_rbnx.slam_checkpoints import digest,inspect_graph


class Stream(Replay):
    def __init__(self,meta,first,rate=1):
        super().__init__(NS(backend='slam_toolbox',fault='none',fault_at=30.,odom_reset=False),meta,[])
        self.lock=threading.Lock()
        self.current=first
        self.route=[];self.route_index=0
        self.route_done=threading.Event();self.route_done.set()
        self.anchor=(0.,0.,0.)
        self.drift=0.
        self.odom_bias=(0.,0.)
        self.noise_sigma=0.
        self.drop_fraction=0.
        import numpy as np
        self.rng=np.random.default_rng(1)
        self.stop_event=threading.Event()
        self.history=[]
        self.map_tf_count=0
        self.map_tf_sources=[]
        self.rate=rate
        self.thread=threading.Thread(target=self.loop,daemon=True)

    def check_authority(self,msg):
        if any(t.header.frame_id=='map' and t.child_frame_id=='odom' for t in msg.transforms):
            self.map_tf_count+=1
            # This Humble build's Executor drops MessageInfo. Check the live
            # DDS endpoints at each actual map->odom message; do not pretend
            # endpoint discovery is per-message publisher provenance.
            endpoints=[e for e in self.get_publishers_info_by_topic('/tf') if e.node_name!=self.get_name()]
            gids=[bytes(e.endpoint_gid).hex() for e in endpoints]
            self.authorities.update(gids)
            self.map_tf_sources.append({'wall':time.monotonic(),'gids':gids})
            for endpoint in endpoints:
                self.authority_nodes.add(endpoint.node_namespace.rstrip('/')+'/'+endpoint.node_name)

    def hold(self,frame,reset=False):
        with self.lock:
            self.route=[];self.current=frame;self.route_done.set()
            if reset:self.anchor=tuple(frame['truth'])

    def drive(self,frames):
        with self.lock:
            self.route=frames;self.route_index=0;self.route_done.clear()

    def loop(self):
        index=0
        while not self.stop_event.is_set():
            started=time.monotonic()
            with self.lock:
                if self.route:
                    self.current=self.route[self.route_index]
                    self.route_index+=1
                    if self.route_index==len(self.route):self.route=[];self.route_done.set()
                frame={**self.current,'t':index/10}
                if self.noise_sigma or self.drop_fraction:
                    import numpy as np
                    ranges=np.asarray([r if r is not None else math.inf for r in frame['ranges']])
                    ranges+=self.rng.normal(0.,self.noise_sigma,len(ranges))
                    ranges[self.rng.random(len(ranges))<self.drop_fraction]=math.inf
                    frame['ranges']=[float(r) if math.isfinite(r) and r>.1 else None for r in ranges]
                x,y,yaw=frame['truth'];ax,ay,ah=self.anchor
                dx,dy=x-ax,y-ay
                odom=(math.cos(ah)*dx+math.sin(ah)*dy,-math.sin(ah)*dx+math.cos(ah)*dy,yaw-ah)
                # Inject the same odometry error into /odom and TF. Sensor
                # ranges and evaluation truth remain untouched.
                actual=frame['truth']
                published_odom=[odom[0]+self.odom_bias[0],odom[1]+self.odom_bias[1],odom[2]+self.drift]
                self.frames.append({**frame,'truth':published_odom})
                self.publish(index)
                self.frames[index]['truth']=actual
                self.history.append({'index':index,'t':index/10,'truth':actual,'odom':published_odom,'wall':started})
            index+=1
            self.stop_event.wait(max(0.,.1/self.rate-(time.monotonic()-started)))


def wait(condition,timeout=30,detail='condition'):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if condition():return
        time.sleep(.05)
    raise RuntimeError(f'timeout: {detail}; localization={map_ops._LOCALIZATION.snapshot()}')


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--datasets',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--rate',type=float,default=1.)
    p.add_argument('--stress-passes',type=int,default=0)
    p.add_argument('--fault-test',action='store_true')
    p.add_argument('--recovery',choices=['candidate','filter'],default='candidate')
    p.add_argument('--calibration-test',action='store_true')
    p.add_argument('--http-test',action='store_true')
    args=p.parse_args()
    args.out.mkdir(parents=True,exist_ok=False)
    os.environ.update(MAPPING_ALGO='slam_toolbox',MAPPING_MAPS_DIR=str(args.out/'maps'),
        MAPPING_CHECKPOINT_DIR=str(args.out/'checkpoints'),MAPPING_CHECKPOINT_AREA_M2='5',
        MAPPING_CHECKPOINT_MIN_INTERVAL_S='3',MAPPING_CHECKPOINT_KEEP='30',
        MAPPING_LAUNCH_DIR=str(ROOT/'launch'),MAPPING_USE_SIM_TIME='true',
        MAPPING_SCAN_TOPIC='/scan',MAPPING_LOCALIZER='amcl')
    map_ops.MAPS_DIR=lifecycle.MAPS_DIR=str(args.out/'maps')
    Path(map_ops.MAPS_DIR).mkdir()
    Path('/tmp/slam_toolbox_resolved.yaml').write_text('scan_topic: /scan\nbase_frame: base_link\nodom_frame: odom\nuse_sim_time: true\nmap_mode: mapping\n')
    meta=json.loads((args.datasets/'checkpoint_a/dataset.json').read_text())
    route_a=[json.loads(s) for s in (args.datasets/'checkpoint_a/scans.jsonl').read_text().splitlines()]
    route_b=[json.loads(s) for s in (args.datasets/'checkpoint_b/scans.jsonl').read_text().splitlines()]
    rclpy.init()
    localizers.configure({'localizer':'amcl','scan_topic':'/scan','base_frame':'base_link','odom_frame':'odom','use_sim_time':True})
    ops=engines.engine_for('slam_toolbox')
    ops.configure(scan_topic='/scan',base_frame='base_link',odom_frame='odom',map_frame='map',use_sim_time=True)
    node=map_ops._get_node()
    node.set_parameters([Parameter('use_sim_time',value=True)])
    stream=Stream(meta,route_a[0],args.rate)
    ex=MultiThreadedExecutor(num_threads=2);ex.add_node(stream)
    spinning=threading.Thread(target=ex.spin,daemon=True);spinning.start();stream.thread.start()
    log=(args.out/'supervisor.log').open('w')
    audit_log=(args.out/'tf-message-provenance.jsonl').open('w')
    audit=subprocess.Popen([str(ROOT/'rbnx-build/tf-audit/tf_audit')],stdout=audit_log,start_new_session=True)
    supervisor=subprocess.Popen(['bash',str(ROOT/'scripts/start_engine.sh')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    events=[]
    if args.http_test:
        from mapping_rbnx import webui
        webui.MAPS_DIR=map_ops.MAPS_DIR
        os.environ['MAPPING_WEBUI_PORT']='8091'
        webui.set_sensor_topics(scan='/scan')
        webui.maybe_start()
    def http(path,body=None):
        request=urllib.request.Request('http://127.0.0.1:8091'+path,
            data=json.dumps(body).encode() if body is not None else None,
            headers={'Content-Type':'application/json'})
        with urllib.request.urlopen(request,timeout=30) as response:
            return json.load(response)
    output={'wall_minus_monotonic':time.time()-time.monotonic(),
        'limitations':['MuJoCo kinematic sensor replay; not wheel dynamics or physical robot',
        'Controlled seeds used for wrong/correct-seed tests; checkpoint restore confirms an algorithm-generated candidate'],
        'rate':args.rate,'events':events,'passed':False}
    def event(name,**data):
        record={'event':name,'wall':time.monotonic(),'frame_index':len(stream.frames),**data}
        events.append(record);print(json.dumps(record),flush=True)
    try:
        lifecycle.set_state('','mapping',bump=True)
        session=map_ops._slam_session();session.begin(metadata={'test':'fresh'})
        wait(lambda:stream.last_result is not None,20,'fresh mapping pose')
        event('fresh_started')
        if args.http_test:
            state=http('/api/state')
            assert state['engine']=='slam_toolbox' and state['mapping_session']['supported']
        stream.drive(route_a)
        wait(stream.route_done.is_set,60/args.rate+20,'route A')
        event('route_a_finished',known_area_m2=session.status()['known_area_m2'])
        wait(lambda:session.status()['last_checkpoint'] is not None,20,'automatic area checkpoint')
        saved=map_ops.save_map_impl('base-a','unified mapping experiment')
        assert saved['ok'],saved
        base=Path(map_ops.MAPS_DIR)/'base-a'
        original={name:digest(base/name) for name in engines.SlamToolboxOps.graph_files}
        base_nodes={json.loads(s)['id'] for s in (base/'nodes.jsonl').read_text().splitlines()}
        event('base_saved',graph=json.loads((base/'graph.json').read_text()))
        # An arbitrary new map pose with a new odometry origin, as after
        # restarting the robot. This move is outside the mapping task.
        session.suspend();ops.stop_engine()
        stream.hold(route_b[0],reset=True)
        tf_before=stream.map_tf_count
        rejected=map_ops.start_mapping_impl('base-a',True,0.,0.,0.)
        assert rejected['ok'],rejected
        wait(lambda:map_ops._LOCALIZATION.snapshot()['phase']=='failed',30,'wrong seed rejection')
        assert any(text in map_ops._LOCALIZATION.snapshot()['detail'] for text in
            ('rejected by live laser','departed from verified odometry')),map_ops._LOCALIZATION.snapshot()
        assert stream.map_tf_count==tf_before,'unverified initial pose escaped onto public TF'
        event('wrong_seed_rejected',detail=map_ops._LOCALIZATION.snapshot()['detail'])
        if args.calibration_test:
            import shutil
            bad_source=Path(map_ops.MAPS_DIR)/'bad-sensor'
            shutil.copytree(base,bad_source)
            sensor=json.loads((bad_source/'sensor.json').read_text())
            sensor['translation'][0]+=.2
            (bad_source/'sensor.json').write_text(json.dumps(sensor))
            response=map_ops.start_mapping_impl('bad-sensor',True,1.,0.,0.);assert response['ok'],response
            wait(lambda:map_ops._LOCALIZATION.snapshot()['phase']=='failed',40,'calibration rejection')
            assert 'calibration' in map_ops._LOCALIZATION.snapshot()['detail'],map_ops._LOCALIZATION.snapshot()
            event('calibration_mismatch_rejected',detail=map_ops._LOCALIZATION.snapshot()['detail'])
        resumed=map_ops.start_mapping_impl('base-a',True,1.,0.,0.)
        assert resumed['ok'],resumed
        wait(lambda:map_ops._LOCALIZATION.snapshot()['phase'] in {'starting','failed'},30,'verified seed')
        assert map_ops._LOCALIZATION.snapshot()['phase']=='starting',map_ops._LOCALIZATION.snapshot()
        # Move DURING engine startup. The scan gate must propagate the seed
        # using odom at the first held scan, not start at the stale (1,0,0).
        handover_start=len(stream.frames)
        stream.drive(route_b)
        wait(lambda:map_ops._LOCALIZATION.snapshot()['phase'] in {'ready','failed'},60,'moving handover')
        assert map_ops._LOCALIZATION.snapshot()['phase']=='ready',map_ops._LOCALIZATION.snapshot()
        resumed_start=len(stream.frames)
        event('moving_handover_ready',start_index=handover_start)
        wait(stream.route_done.is_set,60/args.rate+20,'route B')
        wait(lambda:session.status()['last_checkpoint'] is not None,20,'continued checkpoint')
        end=len(stream.frames)
        errors=[]
        for index,result in list(stream.results.items()):
            if resumed_start<=index<end:
                truth=stream.frames[index]['truth'];pose=result['estimate']
                errors.append(math.hypot(pose[0]-truth[0],pose[1]-truth[1]))
        assert errors and max(errors)<.25,errors
        continued=map_ops.save_map_impl('continued-b','verified moving handover')
        assert continued['ok'],continued
        nodes={json.loads(s)['id'] for s in (Path(map_ops.MAPS_DIR)/'continued-b/nodes.jsonl').read_text().splitlines()}
        assert base_nodes.issubset(nodes)
        assert all(digest(base/name)==value for name,value in original.items())
        event('continuation_verified',position_rmse_m=math.sqrt(sum(e*e for e in errors)/len(errors)),
              max_error_m=max(errors),baseline_nodes=len(base_nodes),continued_nodes=len(nodes))
        # Switching to localization must preserve the whole NEW graph, not
        # revert to base-a or silently discard the B-room observations.
        switched=map_ops.switch_mode_impl('localization');assert switched['ok'],switched
        wait(lambda:map_ops._LOCALIZATION.snapshot()['phase'] in {'ready','failed'},60,'switch to localization')
        assert map_ops._LOCALIZATION.snapshot()['phase']=='ready',map_ops._LOCALIZATION.snapshot()
        current=map_ops._SLAM_REENTRY['source']
        loc_nodes={json.loads(s)['id'] for s in Path(current,'nodes.jsonl').read_text().splitlines()}
        assert nodes.issubset(loc_nodes)
        from mapping_rbnx.grid_relay import Selection
        assert Selection(map_ops.MAPS_DIR).get(lifecycle.current()) is None, 'runtime localization must use the new live grid'
        if args.http_test:
            wait(lambda:http('/api/state').get('navigation_grid',{}).get('revision')=='live',10,'runtime navigation grid')
            event('http_runtime_grid_verified')
        event('localization_preserved_graph',nodes=len(loc_nodes))
        saved_readonly=map_ops.save_map_impl('readonly-copy','full runtime graph while localizing')
        assert saved_readonly['ok'],saved_readonly
        import yaml
        copied=Path(map_ops.MAPS_DIR)/'readonly-copy'
        assert math.isclose(yaml.safe_load((copied/'occupancy.yaml').read_text())['resolution'],
                            yaml.safe_load(Path(current,'occupancy.yaml').read_text())['resolution'],rel_tol=1e-6)
        assert digest(copied/'posegraph.data')==digest(Path(current,'posegraph.data'))
        event('readonly_saved_without_graph_loss')
        switched=map_ops.switch_mode_impl('mapping');assert switched['ok'],switched
        wait(lambda:map_ops._LOCALIZATION.snapshot()['phase'] in {'ready','failed'},60,'switch back to continued mapping')
        assert map_ops._LOCALIZATION.snapshot()['phase']=='ready',map_ops._LOCALIZATION.snapshot()
        event('mapping_mode_roundtrip_ready')
        for number in range(args.stress_passes):
            stream.drive(route_b)
            wait(stream.route_done.is_set,60/args.rate+20,'stress route')
            before=session.status()['last_checkpoint']
            wait(lambda:session.status()['state']=='mapping',30,'checkpoint idle')
            response=map_ops.checkpoint_impl();assert response['ok'],response
            wait(lambda:session.status()['last_checkpoint'] is not None and session.status()['last_checkpoint']!=before,30,'stress checkpoint')
            cp=session.status()['last_checkpoint']
            event('stress_checkpoint',number=number,nodes=cp['graph']['nodes'],serialize_ms=cp['metadata']['serialize_ms'])
        checkpoint=session.store.list()[0]
        directory,manifest=session.store.validate(checkpoint['session_id'],checkpoint['checkpoint_id'])
        # Corrupt a COPY; an integrity rejection must not stop a healthy engine.
        import shutil,uuid
        corrupted_id=uuid.uuid4().hex
        corrupted=session.store.path(checkpoint['session_id'],corrupted_id)
        shutil.copytree(directory,corrupted)
        corrupt_manifest=json.loads((corrupted/'manifest.json').read_text())
        corrupt_manifest['checkpoint_id']=corrupted_id
        (corrupted/'manifest.json').write_text(json.dumps(corrupt_manifest))
        (corrupted/'posegraph.data').write_bytes(b'broken')
        state_before=lifecycle.current()
        bad=map_ops.restore_checkpoint_impl(checkpoint['session_id'],corrupted_id)
        assert not bad['ok'] and lifecycle.current()==state_before,bad
        shutil.rmtree(corrupted)
        event('corrupt_checkpoint_rejected',detail=bad['detail'])
        if args.http_test:
            assert http('/api/checkpoints')['checkpoints']
            restored=http('/api/checkpoints/restore',{'session_id':checkpoint['session_id'],'checkpoint_id':checkpoint['checkpoint_id']})
        else:
            restored=map_ops.restore_checkpoint_impl(checkpoint['session_id'],checkpoint['checkpoint_id'])
        assert restored['ok'],restored
        wait(lambda:map_ops._LOCALIZATION.snapshot()['phase'] in {'ready','failed','choosing'},90,'automatic checkpoint initialization')
        state=map_ops._LOCALIZATION.snapshot()
        if state['phase']=='choosing':
            # Keep strict confidence gates. Exercise the real AMCL fallback
            # with motion instead of treating "choosing" as a failed restore.
            event('restore_ambiguous',localization=state)
            if args.recovery=='candidate':
                # Simulate the existing UI's operator confirmation of one of
                # the actual computed candidates. No truth pose is supplied.
                candidate=state['candidates'][0]
                if args.http_test:
                    chosen=http('/api/pose_estimate',{'x':candidate['x'],'y':candidate['y'],'theta':candidate['theta'],
                        'run_id':state['run_id'],'map_id':state['map_id'],'generation':state['generation']})
                else:
                    chosen=map_ops.pose_estimate_impl(candidate['x'],candidate['y'],candidate['theta'],run_id=state['run_id'])
                assert chosen['ok'],chosen
                event('checkpoint_candidate_confirmed',candidate=candidate)
                wait(lambda:map_ops._LOCALIZATION.snapshot()['phase'] in {'ready','failed'},60,'candidate verification')
            else:
                wait(lambda:map_ops._LOCALIZATION.snapshot()['filter_state']=='running',30,'AMCL startup')
                stream.drive(route_b)
                wait(lambda:map_ops._LOCALIZATION.snapshot()['phase'] in {'ready','failed'},180,'AMCL recovery')
            state=map_ops._LOCALIZATION.snapshot()
        assert state['phase']=='ready',state
        event('checkpoint_restored',recovery=args.recovery,localization=state)
        if args.fault_test:
            # Stop waypoint progression, then inject an odometry yaw reset.
            # The robot's laser geometry and physical pose stay unchanged.
            stream.hold(stream.current)
            wait(lambda:session.status()['last_checkpoint'] is not None,30,'restored checkpoint')
            latest=session.status()['last_checkpoint']['checkpoint_id']
            stream.drift=1.0
            wait(lambda:session.status()['state']=='suspect',20,'odometry jump guard')
            assert session.status()['last_checkpoint']['checkpoint_id']==latest
            assert not map_ops.checkpoint_impl()['ok']
            event('odometry_jump_withdrawn',detail=session.status()['error'])
        output['checkpoints']=session.store.list()
        output['native_pose_samples']=len(stream.results)
        output['map_odom_authority_nodes']=sorted(stream.authority_nodes)
        assert stream.authority_nodes <= {'/slam_scan_gate','/slam_toolbox'},stream.authority_nodes
        # The C++ observer records actual rmw publisher_gid per message.
        # A source must never reappear after another source takes over.
        audit_log.flush()
        rows=[json.loads(s) for s in (args.out/'tf-message-provenance.jsonl').read_text().splitlines()]
        segments=[]
        for row in rows:
            if not segments or segments[-1]!=row['gid']:segments.append(row['gid'])
        output['actual_tf_source_epochs']=len(segments)
        output['actual_tf_messages']=len(rows)
        assert rows and len(segments)==len(set(segments)), 'interleaved TF publishers'
        output['passed']=True
    finally:
        output['last_localization']=map_ops._LOCALIZATION.snapshot()
        output['last_session']=map_ops.mapping_status_impl()
        map_ops._LOCALIZATION.cancel()
        if map_ops._SLAM_SESSION:map_ops._SLAM_SESSION.close()
        try:os.killpg(supervisor.pid,signal.SIGTERM)
        except ProcessLookupError:pass
        try:supervisor.wait(timeout=15)
        except subprocess.TimeoutExpired:os.killpg(supervisor.pid,signal.SIGKILL);supervisor.wait()
        stream.stop_event.set();stream.thread.join(3)
        audit.terminate();audit.wait(timeout=10);audit_log.close()
        ex.shutdown(timeout_sec=3)
        stream.destroy_node()
        (args.out/'result.json').write_text(json.dumps(output,indent=2)+'\n')
        (args.out/'native-poses.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in stream.results.values()))
        (args.out/'frames.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in stream.history))
        (args.out/'map-tf-publishers.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in stream.map_tf_sources))
        log.close()
    print(json.dumps({'passed':output['passed'],'out':str(args.out)},indent=2))


if __name__=='__main__':main()
