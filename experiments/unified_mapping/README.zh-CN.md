# 可复现实验

这些脚本是集成实验工具，不在生产启动时运行。它们发布模拟激光、里程计、TF 和时钟；不发布 cmd_vel、不连接实车。使用独立无网络容器及 ROS domain，不能与正在运行的机器人共用 ROS 网络。

## 已准备的 miniPC 环境

- `LAB_DIR=/home/bunny/map-maintenance`：已有 Ranger Mini 仿真源码、两房间场景、10 Hz 激光数据、生成的 Robonix 接口。
- `LAB_IMAGE=robonix-map-maintenance-lab:20261026`：已有 Humble 2.6.10 测试镜像；不要求重新下载镜像。
- `RESULTS_DIR`：新建输出目录；实验会写很多完整图，建议预留数 GB。

先在同一镜像内 source Humble，编译 `scripts/build_slam_graph_tool.sh`。完整验收还要编译 test-only 消息来源观察器：

```bash
cmake -S experiments/unified_mapping/tf_audit -B rbnx-build/tf-audit
cmake --build rbnx-build/tf-audit -j2
```

使用辅助脚本，保留镜像内 ROS 的 PYTHONPATH，指定 `/usr/bin/python3`，不要使用镜像内另一个 `/usr/local/bin/python3`。镜像原 ENTRYPOINT 是 sleep，因此容器命令需显式覆盖。容器通过 loopback 单播发现，允许同一个隔离容器内多个 ROS 进程通信。

```bash
export LAB_DIR=/home/bunny/map-maintenance
export RESULTS_DIR=/tmp/my-unified-mapping-results
bash experiments/unified_mapping/run_lab.sh integration.py \
  --datasets /lab/data --out /results/acceptance \
  --stress-passes 1 --calibration-test --http-test --fault-test

# 相同输入顺序运行，避免竞争 CPU 影响对照结果。
bash experiments/unified_mapping/run_lab.sh benchmark.py \
  --datasets /lab/data --base /results/acceptance/maps/continued-b \
  --out /results/baseline --passes 12 --rate 2
bash experiments/unified_mapping/run_lab.sh benchmark.py \
  --datasets /lab/data --base /results/acceptance/maps/continued-b \
  --out /results/checkpoints --passes 12 --rate 2 --checkpoints

bash experiments/unified_mapping/run_lab.sh rtabmap_smoke.py \
  --datasets /lab/data --out /results/rtabmap
```

`integration.py` 覆盖真正的 supervisor、建图进程切换、原生 Serialize/DeserializePoseGraph、验证后 TF 放行、网页 HTTP、完整图冷加载、候选确认恢复与异常保护。候选点由真实静态算法产生，脚本扮演操作者确认第一候选；错误与正确初始 seed 测试使用受控真值，评估使用固定旧图坐标，不做每轮轨迹重新对齐。C++ `tf_audit` 使用每条消息的 rmw publisher_gid 验证不同发布者没有交错，不能用 DDS 端点发现列表替代该测量。

`benchmark.py` 对相同轨迹重复 12 次，2 倍速向 ROS 发布共 3600 帧（原始模拟轨迹时间 360 s），每轮行进中手动请求一次检查点。它刻意持续加入节点让图增长，测试保存开销，不把重复圈数当作新覆盖面积。latency_ms 从扫描发布到相同时间戳的原生 `/pose` 回调，是端到端本机响应延迟；不是误差、优化耗时或图锁时长。

## 车轮动力学与噪声

`generate_dynamics.py` 使用现有 Ranger Mini wheel/steering 控制器和 mj_step，不在运行中传送机器人 base；由 raycast 生成扫描。场景与初始建图数据必须一致。该脚本需要 lab 中的 MuJoCo Python 环境，而不是 ROS 系统 Python。示例：

```bash
python experiments/unified_mapping/generate_dynamics.py \
  --ranger /home/bunny/map-maintenance/ranger \
  --scene /home/bunny/map-maintenance/data/checkpoint_b \
  --out /tmp/dynamics-data/checkpoint_b --seconds 65
```

将数据挂载到容器的 `/results/dynamics-data/checkpoint_b` 后运行：

```bash
bash experiments/unified_mapping/run_lab.sh benchmark.py \
  --datasets /results/dynamics-data \
  --base /results/acceptance/maps/continued-b \
  --out /results/noise-23 --passes 1 --rate 1 --checkpoints \
  --drift .0005 --position-drift .001 \
  --noise-sigma .01 --drop-fraction .02 --seed 23
```

这是使用仿真真值控制的轨迹生成加离线 ROS 回放，不是 Nav2 闭环，也不是编码器生成里程计或真实机器人测试。seed 7/17/23 的噪声回放与无噪回放分别记录在交付报告；早期探索与最终代码结果分开标记。

## 写入中断与单元回归

`crash_test.py --source <完整检查点目录> --out <新目录>` 会复制有效检查点，启动真实写子进程，在第一图文件 fsync 后、第二图文件不存在时 SIGKILL；验证旧 latest 和旧图仍有效、staging 不出现在列表中。这模拟进程崩溃，不模拟设备掉电。

`test_slam_checkpoints.py` 包含面积累计、原子事务、损坏/路径验证、后台非阻塞、保留策略、失败不推进水位、异常停建、旧节点变形、服务客户端清理和 zombie 进程组处理等回归。

完整测试在已 source ROS 和生成接口的环境运行：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q -p no:cacheprovider
node --check src/mapping_rbnx/webui_static/app.js
node test_webui_localization.js
```

上游快照基线有两个相同的既有失败（test_package_manifests 的候选契约、test_rtabmap_profiles 的只读挂载断言）；本次新增功能不修改这两个不相关问题。交付包含基线与最终原始日志，不能把完整测试描述为全绿。
