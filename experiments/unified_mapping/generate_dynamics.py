#!/usr/bin/env python3
"""Generate Ranger wheel/steering dynamics + ray-cast lidar for repeatable tests.

This is a waypoint controller using simulator truth, not a Nav2 acceptance
test. The base pose is never teleported after spawn; every motion is produced
by wheel and steering actuators and mj_step.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import mujoco
import numpy as np


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--ranger',type=Path,required=True)
    parser.add_argument('--scene',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=65.)
    args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=False)
    sys.path.insert(0,str(args.ranger))
    from sim.native.controller import RangerPiperController
    from sim.native.sensors import NativeSensorSuite
    model=mujoco.MjModel.from_xml_path(str(args.scene/'scene/scene.xml'))
    data=mujoco.MjData(model)
    controller=RangerPiperController(model,data)
    joint=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,'ranger_base_freejoint')
    address=model.jnt_qposadr[joint]
    data.qpos[address:address+2]=[1.,0.]
    mujoco.mj_forward(model,data)
    robot=json.loads((args.ranger/'assets/robots/ranger_mini_v3_piper/robot.json').read_text())
    config=robot['sensors'];config['lidar']['raysPerScan']=1
    sensors=NativeSensorSuite(model,data,config)
    for _ in range(round(1/model.opt.timestep)):
        controller.command({'type':'cmd_vel'})
        controller.step();mujoco.mj_step(model,data)
    waypoints=[(1.,0.),(6.,0.),(6.,.45),(1.,.45),(1.,0.)]
    target=1;frames=[];commands=[];started=data.time;next_scan=data.time
    while data.time-started<args.seconds:
        x,y=data.qpos[address:address+2]
        w,qx,qy,qz=data.qpos[address+3:address+7]
        yaw=math.atan2(2*(w*qz+qx*qy),1-2*(qy*qy+qz*qz))
        tx,ty=waypoints[target];dx,dy=tx-x,ty-y
        if math.hypot(dx,dy)<.08 and target<len(waypoints)-1:
            target+=1;tx,ty=waypoints[target];dx,dy=tx-x,ty-y
        vx,vy=np.clip([.9*dx,.9*dy],-.35,.35)
        if target==len(waypoints)-1 and math.hypot(dx,dy)<.04:vx=vy=0.
        controller.command({'type':'cmd_vel','linearX':math.cos(yaw)*vx+math.sin(yaw)*vy,
            'linearY':-math.sin(yaw)*vx+math.cos(yaw)*vy,'angularZ':float(np.clip(-1.2*yaw,-.3,.3))})
        controller.step();mujoco.mj_step(model,data)
        if data.time>=next_scan:
            scan,_=sensors.lidar()
            x,y=data.qpos[address:address+2]
            w,qx,qy,qz=data.qpos[address+3:address+7]
            yaw=math.atan2(2*(w*qz+qx*qy),1-2*(qy*qy+qz*qz))
            frames.append({'t':len(frames)/10,'truth':[float(x),float(y),yaw],
                'ranges':[float(v) if np.isfinite(v) else None for v in scan['ranges']],
                'base_z':float(data.qpos[address+2])})
            commands.append({'t':len(frames)/10,'vx':float(vx),'vy':float(vy),'target':target})
            next_scan+=.1
    path=args.out/'scans.jsonl'
    path.write_text(''.join(json.dumps(f)+'\n' for f in frames))
    meta=json.loads((args.scene/'dataset.json').read_text())
    meta.update(route='wheel_dynamics',seconds=len(frames)/10,scan_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        limitations=['Actual MuJoCo wheel/steering dynamics and ray casting','Waypoint control uses simulator truth, not Nav2',
        'Replay odometry derives from simulator pose; optional drift is injected by benchmark'],mujoco_version=mujoco.__version__)
    (args.out/'dataset.json').write_text(json.dumps(meta,indent=2)+'\n')
    (args.out/'commands.json').write_text(json.dumps(commands))
    print(json.dumps({'frames':len(frames),'final_waypoint':target,'first_pose':frames[0]['truth'],
        'last_pose':frames[-1]['truth'],'max_x':max(f['truth'][0] for f in frames),'base_z_range':[min(f['base_z'] for f in frames),max(f['base_z'] for f in frames)]},indent=2))
    sensors.close()


if __name__=='__main__':main()
