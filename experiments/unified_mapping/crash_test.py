#!/usr/bin/env python3
"""SIGKILL a real checkpoint writer between the two graph-file writes."""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src'))
from mapping_rbnx.slam_checkpoints import CheckpointStore,atomic_json


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--source',type=Path);parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--writer',default='');args=parser.parse_args()
    store=CheckpointStore(args.out)
    if args.writer:
        def capture(directory):
            with (directory/'posegraph.posegraph').open('wb') as f:
                f.write(b'partial-write');f.flush();os.fsync(f.fileno())
            (args.out/'writer-ready').write_text('first file durable; second absent')
            time.sleep(60)
        store.create(args.writer,capture,metadata={},reason='interrupted')
        return
    manifest=json.loads((args.source/'manifest.json').read_text())
    sid,cid=manifest['session_id'],manifest['checkpoint_id']
    destination=store.path(sid,cid);destination.parent.mkdir(parents=True,exist_ok=True)
    shutil.copytree(args.source,destination)
    atomic_json(destination.parent/'latest.json',{'checkpoint_id':cid})
    process=subprocess.Popen([sys.executable,__file__,'--out',str(args.out),'--writer',sid])
    deadline=time.monotonic()+10
    while not (args.out/'writer-ready').exists():
        if time.monotonic()>deadline:process.kill();raise RuntimeError('writer did not reach fault point')
        time.sleep(.01)
    os.kill(process.pid,signal.SIGKILL);process.wait(timeout=3)
    reloaded=CheckpointStore(args.out)
    reloaded.validate(sid,cid)
    assert len(reloaded.list())==1
    assert json.loads((destination.parent/'latest.json').read_text())['checkpoint_id']==cid
    result={'passed':True,'writer_exit_code':process.returncode,'visible_checkpoints':len(reloaded.list()),
            'incomplete_staging_dirs':len(list(destination.parent.glob('.*.staging'))),'baseline_checkpoint_id':cid}
    (args.out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':main()
