#!/usr/bin/env python3
"""Real RTAB-Map scan/odom replay and mode services; no SLAM-specific state."""
import argparse
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import threading
import time
from unittest.mock import patch
from integration import ROOT,Stream,wait
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from mapping_rbnx import map_ops,lifecycle


def main():
    p=argparse.ArgumentParser();p.add_argument('--datasets',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    args=p.parse_args();args.out.mkdir(parents=True,exist_ok=False)
    os.environ['MAPPING_ALGO']='rtabmap'
    meta=json.loads((args.datasets/'checkpoint_a/dataset.json').read_text())
    frames=[json.loads(s) for s in (args.datasets/'checkpoint_a/scans.jsonl').read_text().splitlines()]
    rclpy.init();stream=Stream(meta,frames[0]);executor=MultiThreadedExecutor(num_threads=2);executor.add_node(stream)
    # RTAB-Map's existing default scan subscriber requests reliable delivery.
    # Match it in the test publisher; do not change the production profile.
    stream.destroy_publisher(stream.scan_pub)
    stream.scan_pub=stream.create_publisher(LaserScan,'/scan',QoSProfile(depth=20))
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start();stream.thread.start()
    node=map_ops._get_node();node.set_parameters([Parameter('use_sim_time',value=True)])
    maps=[];topics=[]
    subscriptions=[]
    for topic in ('/map','/grid_map','/rtabmap/grid_map'):
        def receive(msg,topic=topic):maps.append(msg);topics.append(topic)
        subscriptions.append(node.create_subscription(OccupancyGrid,topic,receive,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    db=args.out/'live.db';log=(args.out/'engine.log').open('w')
    process=subprocess.Popen(['ros2','launch',str(ROOT/'launch/rtabmap_2d.launch.py'),
        'scan_topic:=/scan','odom_topic:=/odom','use_sim_time:=true','enable_viz:=false',
        'base_frame:=base_link','odom_frame:=odom',f'database_path:={db}','map_mode:=mapping','reset_map:=true'],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    result={'passed':False}
    try:
        lifecycle.set_state('','mapping',bump=True)
        stream.drive(frames);wait(stream.route_done.is_set,45,'RTAB-Map route')
        wait(lambda:bool(maps and any(v>=0 for v in maps[-1].data)),15,'RTAB-Map occupancy')
        # Any diversion into the new SLAM checkpoint/localizer controller is
        # an assertion failure. These are real ROS mode-service calls.
        with patch.object(map_ops,'_slam_session',side_effect=AssertionError('RTAB-Map touched SLAM state')):
            loc=map_ops.switch_mode_impl('localization');assert loc['ok'],loc
            mapping=map_ops.switch_mode_impl('mapping');assert mapping['ok'],mapping
            assert not map_ops.checkpoint_impl()['ok']
        result.update(passed=True,map_messages=len(maps),width=maps[-1].info.width,height=maps[-1].info.height,
            occupancy_topics=sorted(set(topics)),
            known_cells=sum(v>=0 for v in maps[-1].data),mode_localization=loc,mode_mapping=mapping,
            slam_session_created=map_ops._SLAM_SESSION is not None)
    finally:
        try:os.killpg(process.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:process.wait(timeout=15)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        if db.exists():
            con=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
            result['sqlite_check']=con.execute('PRAGMA quick_check').fetchone()[0]
            result['nodes']=con.execute('SELECT COUNT(*) FROM Node').fetchone()[0];con.close()
            result['passed']=bool(result['passed'] and result['sqlite_check']=='ok' and result['nodes']>0)
        stream.stop_event.set();stream.thread.join(2);executor.shutdown(timeout_sec=3);stream.destroy_node();log.close()
        (args.out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));assert result['passed'],result


if __name__=='__main__':main()
