#!/usr/bin/env python3
"""Render measured replay evidence; no synthetic trajectories or metric values."""
import argparse
import base64
import html
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import yaml


def read(path):
    return json.loads(path.read_text())


def lines(path):
    return [json.loads(s) for s in path.read_text().splitlines()]


def uri(path):
    return 'data:image/png;base64,'+base64.b64encode(path.read_bytes()).decode()


def map_view(ax, directory, old_ids=None):
    cfg=yaml.safe_load((directory/'occupancy.yaml').read_text())
    im=np.asarray(Image.open(directory/'occupancy.pgm'))
    x,y,_=cfg['origin'];r=cfg['resolution']
    ax.imshow(im,cmap='gray',vmin=0,vmax=255,extent=[x,x+im.shape[1]*r,y,y+im.shape[0]*r],origin='upper')
    nodes=lines(directory/'nodes.jsonl')
    a=np.array([n['pose'][:2] for n in nodes if old_ids is None or n['id'] in old_ids])
    b=np.array([n['pose'][:2] for n in nodes if old_ids is not None and n['id'] not in old_ids])
    if len(a):ax.scatter(a[:,0],a[:,1],s=14,c='#009e91',label='Retained base nodes',zorder=3)
    if len(b):ax.scatter(b[:,0],b[:,1],s=9,c='#d87c1c',label='Added nodes',zorder=4)
    ax.set_aspect('equal');ax.set_xlabel('Map x (m)');ax.set_ylabel('Map y (m)')


def errors(directory,result):
    frames={f['index']:f for f in lines(directory/'frames.jsonl')}
    pairs=[]
    for p in lines(directory/'native-poses.jsonl'):
        i=p['index']
        if result['first_index']<=i<result['end_index']:
            f=frames[i];pairs.append((f['t'],math.dist(f['truth'][:2],p['estimate'][:2]),p,f))
    return pairs


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--evidence',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    root=args.evidence;accept=root/'acceptance-final2'
    a=read(accept/'result.json');b=read(root/'baseline-final/result.json');c=read(root/'checkpoints-final/result.json')
    n=read(root/'noise-final/result.json');rt=read(root/'rtabmap-final/result.json');cr=read(root/'crash-writer/result.json')
    large=read(root/'large-moving-final/result.json')
    assert all(r['passed'] for r in (a,b,c,n,rt,cr,large))
    for result,directory in ((a,accept),(c,root/'checkpoints-final')):
        result['checkpoints']=[read(directory/'checkpoints'/q['session_id']/q['checkpoint_id']/'manifest.json') for q in result['checkpoints']]
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    old={q['id'] for q in lines(accept/'maps/base-a/nodes.jsonl')}
    latest=max(a['checkpoints'],key=lambda q:q['created_at'])
    latest_dir=accept/'checkpoints'/latest['session_id']/latest['checkpoint_id']
    stages=[(accept/'maps/base-a','Fresh mapping'),(accept/'maps/continued-b','Resume while moving'),(latest_dir,'Recovered checkpoint')]
    fig,axs=plt.subplots(1,3,figsize=(16,5.4),constrained_layout=True)
    for ax,(directory,title) in zip(axs,stages):
        g=read(directory/'graph.json');map_view(ax,directory,old)
        ax.set_title(f"{title}\n{g['nodes']} nodes / {g['components']} component; {g['known_area_m2']:.1f} m² known")
    axs[1].legend(loc='lower right',fontsize=8)
    fig.savefig(args.out/'mapping-stages.png',dpi=160);plt.close(fig)
    fig,axs=plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
    colors=['#267f9f','#d87c1c'];labels=['Without periodic snapshots','With moving snapshots']
    for result,directory,color,label in zip((b,c),(root/'baseline-final',root/'checkpoints-final'),colors,labels):
        e=errors(directory,result);t=np.array([v[0] for v in e]);t-=t[0]
        axs[0,0].plot(t,[v[1]*100 for v in e],color=color,alpha=.75,lw=.9,label=label)
        axs[0,1].plot(t,[v[2]['latency_ms'] for v in e],color=color,alpha=.6,lw=.7,label=label)
    axs[0,0].set(title='Fixed map-frame position error',xlabel='Simulated route time (s)',ylabel='Error (cm)')
    axs[0,1].set(title='Scan publication to native pose response',xlabel='Simulated route time (s)',ylabel='Latency (ms)')
    axs[0,0].legend(fontsize=8)
    checkpoints=sorted(c['checkpoints'],key=lambda q:q['graph']['nodes'])
    axs[1,0].plot([q['graph']['nodes'] for q in checkpoints],[q['metadata']['serialize_ms'] for q in checkpoints],'o-',color=colors[1],label='Serialize service round trip')
    axs[1,0].plot([q['graph']['nodes'] for q in checkpoints],[q['total_ms'] for q in checkpoints],'o--',color=colors[0],label='Complete verified checkpoint')
    axs[1,0].set(title='Checkpoint cost as the graph grows',xlabel='Nodes in checkpoint',ylabel='Wall time (ms)');axs[1,0].legend(fontsize=8)
    for i,result in enumerate((b,c)):
        rate=result['native_samples']/result['input_scans']*100
        axs[1,1].bar(i,rate,width=.5,color=colors[i])
        axs[1,1].text(i,rate+.5,f"{result['native_samples']} / {result['input_scans']}",ha='center')
    axs[1,1].set_xticks([0,1],['Without periodic snapshots','With moving snapshots'])
    axs[1,1].set(title='Paired replay: native output sample counts',ylabel='Native pose samples per 100 input scans',ylim=(0,33))
    fig.savefig(args.out/'checkpoint-performance.png',dpi=160);plt.close(fig)
    e=errors(root/'noise-final',n)
    fig,axs=plt.subplots(1,2,figsize=(12,4.5),constrained_layout=True)
    truth=np.array([v[3]['truth'][:2] for v in e]);pose=np.array([v[2]['estimate'][:2] for v in e])
    axs[0].plot(truth[:,0],truth[:,1],c='#009e91',lw=3,label='Simulator trajectory')
    axs[0].plot(pose[:,0],pose[:,1],'.-',c='#d87c1c',ms=3,lw=.8,label='SLAM estimate')
    axs[0].axis('equal');axs[0].set(xlabel='Map x (m)',ylabel='Map y (m)',title='Wheel-dynamics replay with injected sensor/odom errors');axs[0].legend(fontsize=8)
    axs[1].plot([v[0]-e[0][0] for v in e],[v[1]*100 for v in e],c='#267f9f')
    axs[1].set(xlabel='Simulated time (s)',ylabel='Position error (cm)',title=f"Seed 23: RMSE {n['rmse_m']*100:.2f} cm / max {n['max_error_m']*100:.2f} cm")
    fig.savefig(args.out/'wheel-dynamics-noise.png',dpi=160);plt.close(fig)
    # Embed every actual acceptance snapshot for an offline slider.
    snapshots=[]
    for q in sorted(a['checkpoints'],key=lambda q:q['created_at']):
        directory=accept/'checkpoints'/q['session_id']/q['checkpoint_id']
        snapshots.append({'image':uri(directory/'occupancy.png'),'nodes':q['graph']['nodes'],
            'known':q['graph']['known_area_m2'],'reason':q['reason'],'serialize_ms':q['metadata']['serialize_ms'],
            'session':q['session_id'][:8],'id':q['checkpoint_id'][:8]})
    titles={
        'fresh_started':'从零建图启动','base_saved':'基础图已保存','wrong_seed_rejected':'1 米错误初值被拒绝',
        'calibration_mismatch_rejected':'20 厘米外参差异被拒绝','moving_handover_ready':'移动中续建接管完成',
        'continuation_verified':'续建图与旧节点验证通过','http_runtime_grid_verified':'导航栅格与完整运行图一致',
        'localization_preserved_graph':'切换定位保留完整新图','mapping_mode_roundtrip_ready':'切回建图验证通过',
        'readonly_saved_without_graph_loss':'定位模式另存保留完整图与分辨率',
        'stress_checkpoint':'持续建图检查点完成','corrupt_checkpoint_rejected':'损坏检查点被拒绝',
        'checkpoint_candidate_confirmed':'确认算法生成的候选点','checkpoint_restored':'检查点恢复完成',
        'odometry_jump_withdrawn':'里程计突跳后撤下建图'}
    timeline=''.join(f"<li><span>{v['wall']-a['events'][0]['wall']:.1f} s</span>{html.escape(titles[v['event']])}</li>" for v in a['events'] if v['event'] in titles)
    rows=''.join(f"<tr><td>{label}</td><td>{r['input_scans']}</td><td>{r['final_graph']['nodes']}</td><td>{r['rmse_m']*100:.3f}</td><td>{r['latency_p95_ms']:.1f}</td><td>{r['native_samples']}</td></tr>" for label,r in [('无周期检查点（2×）',b),('移动中保存检查点（2×）',c),('车轮动力学＋噪声 seed23（1×）',n),('千节点图移动接管（1×）',large)])
    cpmax=max(checkpoints,key=lambda q:q['graph']['nodes'])
    summary={'acceptance_passed':a['passed'],'actual_tf_source_epochs':a['actual_tf_source_epochs'],'actual_tf_messages':a['actual_tf_messages'],
        'baseline':{k:b[k] for k in ('input_scans','native_samples','rmse_m','latency_p95_ms','final_graph')},
        'checkpoints':{k:c[k] for k in ('input_scans','native_samples','rmse_m','latency_p95_ms','final_graph')},
        'largest_checkpoint':{'nodes':cpmax['graph']['nodes'],'serialize_ms':cpmax['metadata']['serialize_ms'],'total_ms':cpmax['total_ms'],
            'bytes':sum(f['bytes'] for f in cpmax['files'].values())},
        'noise_seed23':{k:n[k] for k in ('input_scans','native_samples','rmse_m','max_error_m','latency_p95_ms')},
        'large_moving_handover':{k:large[k] for k in ('moving_handover','rmse_m','max_error_m','latency_p95_ms','final_graph')},
        'rtabmap':rt,'crash':cr}
    (args.out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>slam_toolbox 统一建图实测</title>
<style>body{margin:0;background:#f3f5f6;color:#18313d;font:16px/1.7 system-ui,sans-serif}main{max-width:1180px;margin:auto;padding:36px 22px}h1{font-size:34px;line-height:1.3}h2{margin-top:36px}p{max-width:1000px}.tag{color:#087d70;font-weight:700}.card{background:white;border-radius:14px;padding:24px;margin:20px 0;box-shadow:0 2px 14px #1231}.figure{width:100%;height:auto}table{width:100%;border-collapse:collapse;font-size:14px}td,th{border-bottom:1px solid #dbe4e8;text-align:left;padding:12px 8px}li{margin:8px 0}li span{font-variant-numeric:tabular-nums;display:inline-block;width:90px;color:#617680}input[type=range]{width:100%}#map{width:100%;height:400px;object-fit:contain;image-rendering:pixelated;background:#e6ebed;border-radius:8px}.note{color:#617680;font-size:14px}.links a{margin-right:20px;color:#087d70}</style><main>
<div class="tag">RANGER MINI · ROS 2 HUMBLE · SLAM_TOOLBOX 2.6.10 · 2026-11-07</div><h1>从零建图、续建与检查点：同一条经过验证的流程</h1>
<p>已实现并通过本次仿真回放验收：复用初始定位、移动中装载旧位姿图、验证后接管 TF、新覆盖面积/定时检查点、完整图恢复与异常撤下。基础地图始终保留。局部位姿图更新保留设计，未作为可用功能开放。</p>
<div class="card"><h2 style="margin-top:0">实际保存的地图与位姿图</h2><img class="figure" src="MAPSTAGES"><p class="note">同一固定旧图坐标。绿色是基础节点，橙色是新节点；检查点栅格由保存的扫描与图冷加载重建。</p></div>
<div class="card"><h2 style="margin-top:0">检查点时间轴</h2><img id="map" alt="实际检查点预览"><input id="slider" type="range" min="0" max="MAX" value="0"><div id="caption"></div><p class="note">拖动查看实际成功提交的快照。恢复产生新会话；新会话仍从所选旧图继续。</p></div>
<div class="card"><h2 style="margin-top:0">同输入配对性能</h2><table><thead><tr><th>实验</th><th>输入帧</th><th>末图节点</th><th>位置 RMSE / cm</th><th>响应 P95 / ms</th><th>原生位姿样本</th></tr></thead><tbody>ROWS</tbody></table><img class="figure" src="PERFORMANCE"><p class="note">配对两组均为同路线 12 次、2 倍速回放，各 3600 输入帧；动力学和大图接管各自以 1 倍速运行一遍。原生位姿并非每个激光输入都会生成。Serialize 曲线是服务往返时间，包含调度等待，不能当作精确图锁时间；完整检查点还包含后台冷加载、渲染和持久化。</p></div>
<div class="card"><h2 style="margin-top:0">车轮动力学、激光噪声、丢帧和里程计漂移</h2><img class="figure" src="DYNAMICS"><p>激光噪声 σ=1 cm、随机丢失 2% 测距、航向漂移 0.0005 rad/s、位置漂移 0.001 m/s。轨迹来自 MuJoCo 车轮与转向执行器，建图使用离线 ROS 回放；路线控制使用仿真真值，里程计从轨迹生成并加入漂移。这不是 Nav2 闭环或实车验收。</p></div>
<div class="card"><h2 style="margin-top:0">最终端到端验收事件</h2><ol>TIMELINE</ol><p>TF_AUDIT</p><p>写进程在第一图文件 fsync 后被 SIGKILL，已有有效检查点和 latest 保持可用。RTAB-Map 实际运行回归得到 SQLITE_NODES 个节点，SQLite quick_check=ok，建图/定位两次模式切换通过，未创建 slam 会话。</p></div>
<h2>已知边界</h2><p>自动全局定位在双房间场景存在歧义，恢复测试确认了算法实际给出的候选点；保持原置信门槛。另行尝试的 AMCL 全局散布未在 180 秒内稳定收敛，未计作全自动恢复通过。检查点仍是原版完整图序列化，图大时有短暂锁等待；没有证明超大图硬实时。保守异常阈值可能把正确的大闭环也标为待审查；慢漂移不能保证全部拦截。</p><p>完整 Python 回归 244 项通过、2 项上游既有失败（交付基线日志有相同失败）；前端语法和原有定位交互测试通过。未做实车、设备真实掉电、长走廊或动态人群验收；Jetson 未验证。历史会话需按磁盘容量清理。</p>
<p class="links"><a href="../source/docs/UNIFIED-MAPPING.zh-CN.md">具体实现说明</a><a href="../source/experiments/unified_mapping/README.zh-CN.md">复现实验</a><a href="summary.json">精确指标 JSON</a></p></main><script>const snapshots=SNAPSHOTS;const slider=document.querySelector('#slider');function show(){const q=snapshots[Number(slider.value)];document.querySelector('#map').src=q.image;document.querySelector('#caption').textContent=`${Number(slider.value)+1}/${snapshots.length} · 会话 ${q.session} · 检查点 ${q.id} · ${q.nodes} 节点 · 已知 ${q.known.toFixed(2)} m² · ${q.reason} · 序列化往返 ${q.serialize_ms.toFixed(1)} ms`;}slider.addEventListener('input',show);show();</script></html>'''
    replace={'MAPSTAGES':uri(args.out/'mapping-stages.png'),'PERFORMANCE':uri(args.out/'checkpoint-performance.png'),
        'DYNAMICS':uri(args.out/'wheel-dynamics-noise.png'),'MAX':str(len(snapshots)-1),'ROWS':rows,'TIMELINE':timeline,
        'TF_AUDIT':f"独立观察器记录 {a['actual_tf_messages']} 条实际 map→odom 消息，跨 {a['actual_tf_source_epochs']} 个顺序接管的发布者生命周期，没有来源交错。另从约千节点的完整图在移动中恢复并续建，末图 {large['final_graph']['nodes']} 节点，位置 RMSE {large['rmse_m']*100:.2f} cm。",
        'SQLITE_NODES':str(rt['nodes']),'SNAPSHOTS':json.dumps(snapshots)}
    for key,value in replace.items():page=page.replace(key,value)
    cost=f"最大实测检查点为 {cpmax['graph']['nodes']} 节点、{sum(f['bytes'] for f in cpmax['files'].values())/1024**2:.1f} MiB；序列化服务往返 {cpmax['metadata']['serialize_ms']:.0f} ms，完整提交 {cpmax['total_ms']:.0f} ms。按同等大小保留 20 份约占 1.3 GiB，历史会话另计。"
    page=page.replace('<h2>已知边界</h2>','<p>'+cost+'</p><h2>已知边界</h2>')
    (args.out/'index.html').write_text(page)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
