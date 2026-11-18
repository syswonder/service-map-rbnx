#!/usr/bin/env python3
"""Paired replay benchmark: identical routes with/without moving snapshots."""
import argparse
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time

from integration import ROOT,Stream,wait
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from mapping_rbnx import engines,lifecycle,localizers,map_ops


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--datasets',type=Path,required=True)
    p.add_argument('--base',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--checkpoints',action='store_true')
    p.add_argument('--passes',type=int,default=12)
    p.add_argument('--rate',type=float,default=2.)
    p.add_argument('--drift',type=float,default=0.,help='Injected odometry yaw drift, radians per simulated second')
    p.add_argument('--position-drift',type=float,default=0.,help='Injected x odometry drift in metres per simulated second')
    p.add_argument('--noise-sigma',type=float,default=0.)
    p.add_argument('--drop-fraction',type=float,default=0.)
    p.add_argument('--seed',type=int,default=1)
    p.add_argument('--moving-handover',action='store_true',help='Drive as soon as the verified pose enters engine handover')
    args=p.parse_args();args.out.mkdir(parents=True,exist_ok=False)
    os.environ.update(MAPPING_ALGO='slam_toolbox',MAPPING_MAPS_DIR=str(args.out/'maps'),
        MAPPING_CHECKPOINT_DIR=str(args.out/'checkpoints'),MAPPING_CHECKPOINT_AREA_M2='1000000',
        MAPPING_CHECKPOINT_MIN_INTERVAL_S='1',MAPPING_CHECKPOINT_MAX_INTERVAL_S='1000000',
        MAPPING_CHECKPOINT_KEEP='20',MAPPING_LAUNCH_DIR=str(ROOT/'launch'),MAPPING_USE_SIM_TIME='true')
    map_ops.MAPS_DIR=lifecycle.MAPS_DIR=str(args.out/'maps')
    Path(map_ops.MAPS_DIR).mkdir();shutil.copytree(args.base,Path(map_ops.MAPS_DIR)/'base')
    Path('/tmp/slam_toolbox_resolved.yaml').write_text('scan_topic: /scan\nbase_frame: base_link\nodom_frame: odom\nuse_sim_time: true\nmap_mode: localization\n')
    meta=json.loads((args.datasets/'checkpoint_b/dataset.json').read_text())
    route=[json.loads(s) for s in (args.datasets/'checkpoint_b/scans.jsonl').read_text().splitlines()]
    rclpy.init()
    localizers.configure({'localizer':'amcl','scan_topic':'/scan','base_frame':'base_link','odom_frame':'odom','use_sim_time':True})
    ops=engines.engine_for('slam_toolbox');ops.configure(scan_topic='/scan',base_frame='base_link',odom_frame='odom',map_frame='map',use_sim_time=True)
    map_ops._get_node().set_parameters([Parameter('use_sim_time',value=True)])
    stream=Stream(meta,route[0],args.rate);stream.anchor=tuple(route[0]['truth'])
    import numpy as np
    stream.rng=np.random.default_rng(args.seed);stream.noise_sigma=args.noise_sigma;stream.drop_fraction=args.drop_fraction
    executor=MultiThreadedExecutor(num_threads=2);executor.add_node(stream)
    spinning=threading.Thread(target=executor.spin,daemon=True);spinning.start();stream.thread.start()
    log=(args.out/'supervisor.log').open('w')
    supervisor=subprocess.Popen(['bash',str(ROOT/'scripts/start_engine.sh')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    output={'passed':False,'snapshots_enabled':args.checkpoints,'passes':args.passes,'rate':args.rate,'drift':args.drift,
        'position_drift':args.position_drift,'noise_sigma':args.noise_sigma,'drop_fraction':args.drop_fraction,'seed':args.seed,
        'wall_minus_monotonic':time.time()-time.monotonic(),'events':[]}
    try:
        lifecycle.set_state('base','localization',bump=False)
        time.sleep(2)
        response=map_ops.start_mapping_impl('base',True,1.,0.,0.);assert response['ok'],response
        moving_first=None
        if args.moving_handover:
            wait(lambda:map_ops._LOCALIZATION.snapshot()['phase'] in {'starting','failed'},30,'moving handover phase')
            assert map_ops._LOCALIZATION.snapshot()['phase']=='starting'
            moving_first=len(stream.frames)
            stream.drive(route)
        wait(lambda:map_ops._LOCALIZATION.snapshot()['phase'] in {'ready','failed'},60,'benchmark initialization')
        assert map_ops._LOCALIZATION.snapshot()['phase']=='ready',map_ops._LOCALIZATION.snapshot()
        session=map_ops._slam_session()
        wait(lambda:session.status()['last_checkpoint'] is not None,30,'initial checkpoint')
        first_index=moving_first if moving_first is not None else len(stream.frames)
        # Start each repeated pass at the previous pass's endpoint (nearly
        # the start), avoiding a teleport at route boundaries.
        for number in range(args.passes):
            if number or not args.moving_handover:stream.drive(route)
            if args.checkpoints:
                time.sleep(.5)
                response=map_ops.checkpoint_impl();assert response['ok'],response
            while not stream.route_done.wait(.1):
                stream.drift=args.drift*(len(stream.frames)-first_index)/10.
                stream.odom_bias=(args.position_drift*(len(stream.frames)-first_index)/10.,0.)
                if session.status()['state']=='suspect':raise RuntimeError(session.status()['error'])
            item={'pass':number,'frame_index':len(stream.frames),'session':session.status()['state']}
            last=session.status()['last_checkpoint']
            if last:item.update(nodes=last['graph']['nodes'],serialize_ms=last['metadata']['serialize_ms'])
            output['events'].append(item);print(json.dumps(item),flush=True)
        end_index=len(stream.frames)
        wait(lambda:session.status()['state']=='mapping',30,'checkpoint completion')
        final=map_ops.save_map_impl('final','checkpoint benchmark');assert final['ok'],final
        errors=[];latencies=[]
        for index,result in list(stream.results.items()):
            if first_index<=index<end_index:
                truth=stream.frames[index]['truth'];pose=result['estimate']
                errors.append(math.hypot(pose[0]-truth[0],pose[1]-truth[1]));latencies.append(result['latency_ms'])
        import numpy as np
        output.update(passed=bool(errors and max(errors)<.25),first_index=first_index,end_index=end_index,
            moving_handover=args.moving_handover,
            native_samples=len(errors),input_scans=end_index-first_index,
            rmse_m=math.sqrt(sum(e*e for e in errors)/len(errors)),max_error_m=max(errors),
            latency_median_ms=float(np.median(latencies)),latency_p95_ms=float(np.percentile(latencies,95)),
            latency_max_ms=max(latencies),final_graph=json.loads((Path(map_ops.MAPS_DIR)/'final/graph.json').read_text()),
            checkpoints=session.store.list(),map_odom_authority_nodes=sorted(stream.authority_nodes))
        assert output['passed'],output
    finally:
        output['last_localization']=map_ops._LOCALIZATION.snapshot()
        output['last_session']=map_ops.mapping_status_impl()
        map_ops._LOCALIZATION.cancel()
        if map_ops._SLAM_SESSION:map_ops._SLAM_SESSION.close()
        try:os.killpg(supervisor.pid,signal.SIGTERM)
        except ProcessLookupError:pass
        try:supervisor.wait(timeout=15)
        except subprocess.TimeoutExpired:os.killpg(supervisor.pid,signal.SIGKILL);supervisor.wait()
        stream.stop_event.set();stream.thread.join(3);executor.shutdown(timeout_sec=3);stream.destroy_node()
        (args.out/'result.json').write_text(json.dumps(output,indent=2)+'\n')
        (args.out/'native-poses.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in stream.results.values()))
        (args.out/'frames.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in stream.history))
        log.close()
    print(json.dumps({k:v for k,v in output.items() if k not in {'events','checkpoints','last_localization','last_session'}},indent=2))


if __name__=='__main__':main()
