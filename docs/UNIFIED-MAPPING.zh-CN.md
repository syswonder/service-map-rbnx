# slam_toolbox 统一建图与检查点

本实现以 service-map-rbnx `070248daab98695f30d54c1f7e349fc6a6b48c41` 为基础，实验环境为 x86 miniPC、ROS 2 Humble、slam_toolbox 2.6.10。RTAB-Map 不进入本会话控制器，其 launch、参数、数据库与模式服务保持原路径。测试报告由 `experiments/unified_mapping/` 的真实 ROS 回放产生。

精确指标、回归结果与实验边界见 [最终实测](UNIFIED-MAPPING-RESULTS.zh-CN.md)。默认滚动检查点是完整图快照，本次约千节点时单份约 67 MiB，应按实际图规模配置保留数量。

## 1. 从零与续建使用同一条建图路径

两者都启动原版 `async_slam_toolbox_node`、同一个扫描入口和同一个检查点管理器。差别只在启动前是否装载旧图：

```text
从零：空图 ───────────────────────────────────────┐
续建：旧图校验 → 初始定位/候选确认 → 3 帧激光验证 ─┤
                                                 ↓
              统一建图引擎 → 面积/定时检查点 → 正式另存
```

续建会复用已有的静态匹配、粒子滤波和候选点确认状态机，不另写一套初始定位。人工选择的候选同样需要激光验证。编辑后栅格与原始栅格都参与验证，避免只在编辑后像素上吻合、在实际旧图上不吻合。

原版反序列化会替换当前图和 dataset；它不是把一个图合并进另一个图。因此续建必须在新引擎收到第一帧激光之前完成装载。新扫描门 `scripts/slam_scan_gate.py` 先缓存激光，再以 `START_AT_GIVEN_POSE` 装载图。

初始位姿绑定验证帧的 odom 时间戳。切换进程期间机器人可以移动：用 SE(2) 里程计增量把位姿传递到第一帧缓存扫描，装载后先只投递这一帧，等待引擎回传相同时间戳的 `/pose`，再按 50 Hz 排空队列。最多缓存 200 帧，超过则拒绝启动。这样不会因瞬时投递大量缓存而丢掉决定初始坐标的首帧。

候选建图引擎的 TF 先发布在 `/slam_toolbox/internal_tf`；门节点把正常 odom TF 转入该总线，并扣住 map→odom。应用检查实际引擎输出与验证位姿的里程计预测（10 cm / 3°）、旧节点保留、旧图变形后，才调用 release_frame 放行公共 TF。初始化失败关闭整个候选进程组，不能在错误坐标上继续累积图。

初始化阶段首帧移动超过 1 m 会拒绝接管。开始续建时应位于旧图有稳定可辨识结构的区域；自动定位有歧义时由候选确认完成，不能通过降低阈值强行放行。

## 2. 检查点保存的是什么

默认策略为新增已知覆盖 20 m²、最短间隔 15 s；即使没有扩展到新区域，最长 60 s 也保存一次，避免重复经过旧区域新加的图节点长期没有备份。每次续建成功还会请求 session_start 检查点。手动按钮只入队，不阻塞 HTTP 等待完整保存。

面积统计读取原始 `/slam_toolbox/map`，按固定世界坐标网格累计已知单元的并集。unknown padding 和画布边框扩张不计入面积；整数单元的 origin 平移不会重复计数。它是新观测覆盖的近似触发指标，并非建筑实测面积；地图优化导致的覆盖边缘变化也可能触发。重复路线依靠定时策略兜底。

每份检查点有以下文件：

| 文件 | 用途 |
|---|---|
| `posegraph.posegraph`、`posegraph.data` | 原版完整图、扫描与传感器数据，可继续建图 |
| `occupancy.pgm/yaml/png` | 从上述保存数据冷加载、重建得到的栅格和缩略图 |
| `graph.json`、`nodes.jsonl` | 节点、边、连通性与位姿审计 |
| `sensor.json` | 激光几何、frame、base→laser 外参 |
| `manifest.json` | 会话/检查点/父版本、覆盖面积、引擎参数与版本、各文件大小/SHA256、保存耗时 |

单后台线程调用原版 `serialize_map`，随后独立本地进程冷加载、检查非空/单连通/有限位姿、重建栅格、验证旧节点变形、计算哈希与 fsync。写入隐藏 staging 目录，完整成功后同文件系统 rename，最后原子更新 latest 指针。失败不改变上一个有效检查点，也不推进面积保存水位。每会话滚动保留 20 份；历史会话不自动删除，需要按容量清理。

保存不会调用 pause、停止扫描或给底盘发送任何运动指令。原版 2.6.10 序列化在图互斥锁内完成，复杂度随整图增长；后台化消除了应用等待和渲染占用图锁，但没有消除原生序列化的锁。这不是增量文件格式、fork 快照或硬实时保证。`serialize_ms` 是客户端观察到的服务往返耗时，包含等调度/锁时间，不能直接当作精确锁占用时间。

## 3. 异常保护与恢复

运行时检查原始里程计突跳、非有限值、时间倒退，以及原生位姿相对里程计预测的大修正。默认里程计速度上限 2 m/s、角速度上限 3 rad/s；位姿创新量阈值 0.5 m / 15°。旧节点相对本次会话基础图发生大于 0.5 m / 15° 的变形，也不允许提交检查点或正式地图。

大修正可能是正确闭环，也可能是错误闭环。实现采取保守策略：标记 suspect，停止当前建图引擎，撤销可用建图状态，保留此前有效检查点，由操作者检查后恢复。它不会自行选择“哪一份地图一定正确”，也不保证拦住所有缓慢漂移或所有错误闭环。检查点冷校验通过只表示结构和已配置约束通过，不等同真实环境正确。

恢复先验证 manifest、大小、哈希和可冷加载性；失败时不停止当前可用会话。有效恢复会启动一个新会话，保留原检查点不变，记录 restored_from，然后走与普通续建相同的初始定位和扫描门流程。恢复的是指定版本，不会试图用坏图反向修正好图。

保存后切换定位时，先捕获当前完整图作为只读运行副本，不能偷偷退回最初基础图。再切回建图从这个副本继续。这里特别处理了 lifecycle 与 grid_relay 的身份：运行副本不冒用原基础地图 id，避免定位图已更新、导航栅格仍是旧地图。

## 4. 使用方式

在已有 slam_toolbox 部署中重新构建包：

```bash
# 原有 x86 Docker 构建入口会自动编译图工具。
bash scripts/build.sh
# 原生环境：先 source 与运行节点一致的 ROS/slam_toolbox overlay。
source /opt/ros/humble/setup.bash
bash scripts/build_slam_graph_tool.sh
```

新 C++ 工具需要 cmake、编译器、Eigen、Boost serialization/filesystem/system/thread、TBB 及 slam_toolbox 开发文件。必须与运行引擎使用相同 Karto/Boost ABI；已验证 Humble 2.6.10 x86，Jetson/其他 ROS 发行版需要另行编译验证。不能拷贝 x86 二进制给 Jetson 使用。

部署配置示例（沿用现有配置入口）：

```yaml
algo: slam_toolbox
map_mode: mapping
mapping_base_map_id: ""   # 空：从零；填写已有地图 id：先定位再续建
checkpoint_area_m2: 20
checkpoint_min_interval_s: 15
checkpoint_max_interval_s: 60
checkpoint_keep: 20
```

`mapping_base_map_id` 只对 slam_toolbox 生效，不能与 reset_map=true 同用。已有 map_id 的旧有用途保持原含义；不要仅靠 map_id 猜测要续建。原生运行未指定目录时，检查点保存在 maps 同级的 `slam-checkpoints/`。Docker 启动器未显式指定时使用已有包挂载内的 `/mapping/slam-checkpoints`；显式设置 `MAPPING_CHECKPOINT_DIR` 时会创建并挂载该外部目录。若地图保存在提供方缓存之外，建议同时明确检查点的持久存储路径。最长期限不能小于最短期限。

网页增加“从零建图”“从选中地图继续建图”、检查点面积/状态、缩略图、手动检查点和恢复入口。现有加载接口以 mode=mapping 加载 slam 地图也走这套流程。恢复期间如果出现候选点，使用已有候选确认交互。

| HTTP | 含义 |
|---|---|
| POST `/api/mapping/start` `{}` | 从零开始新会话 |
| POST `/api/mapping/start` `{"base_map_id":"id"}` | 自动定位后续建；接受的请求仍需查询最终状态 |
| GET `/api/mapping/status` | 当前会话、面积进度、保存/异常状态与最后检查点 |
| POST `/api/checkpoints/create` `{}` | 入队手动检查点 |
| GET `/api/checkpoints` | 列表及元数据 |
| POST `/api/checkpoints/restore` `{"session_id":"…","checkpoint_id":"…"}` | 验证并进入恢复定位 |
| GET `/api/checkpoints/preview.png?session_id=…&checkpoint_id=…` | 对应快照的缩略图 |

写接口要求 application/json。初始位姿仍可按现有接口传 has_initial_pose/x/y/theta；它不会绕过激光验证。正式“保存地图”仍要求新的地图 id，不覆盖基础地图。

## 5. 实验与边界

实验脚本在 `experiments/unified_mapping/README.zh-CN.md`。测试包括实际 supervisor/ROS 服务、移动中装载、错误 1 m 初值、20 cm 激光外参变化、图身份与导航栅格、候选点恢复、损坏文件、原始里程计突跳、写进程 SIGKILL、同路线配对性能、MuJoCo 车轮动力学生成的带噪回放和独立 RTAB-Map 运行回归。

简单双房间场景的自动全局定位存在歧义；保持生产置信阈值时，恢复会进入候选确认。实验确认真实算法给出的候选后完成恢复，没有注入真值完成恢复。另行尝试的 AMCL 全局散布在 180 s 内未稳定收敛，因此不把它算作全自动恢复通过。

车轮动力学数据用真实 wheel/steering 控制和 MuJoCo mj_step 生成，扫描用场景 raycast；路线控制器使用模拟真值，ROS 建图测试消费离线数据，里程计由轨迹生成并注入漂移。它不是 Nav2 闭环自主导航，不是底盘编码器全链路，也不是实车可靠性证明。长走廊、动态人群、真实反光/玻璃、超大图、磁盘掉电和满盘的系统级试验仍需部署环境补充。SIGKILL 已验证，真实断电没有验证。

## 6. 局部更新：保留方案，未开放删图按钮

区域内的“节点中心”不等于“观测只影响该区域”。区外节点也会看到区内障碍，删掉区内节点可能仍保留旧障碍；删掉所有看到障碍的节点又可能破坏区域外地图或使图断开。

建议独立的受控引擎 overlay：先在候选副本生成预览，计算扫描射线影响、待删除节点/边、剩余连通分量、边界锚点和区域外覆盖损失；允许安全的支路/房间完整节点裁剪，拒绝断图和无稳定边界。随后从可信边界定位、采集新扫描、冷重载验收后发布。

若需要精确到区域内的旧障碍替换，则应退休区域内旧测量，同时保留区域外观测，并重估受影响的匹配/闭环约束。掩码必须同时作用于匹配、栅格重建和冷加载，不能只修改输出像素或把射线一律设为 inf。上游 experimental lifelong 展示底层节点删除，但不是可直接使用的按区域删除服务。这部分本轮未实现，以免把未验证的删图机制混入稳定续建流程。

## 7. 主要实现位置与上游依据

- `map_ops.py`：统一会话入口、已有定位状态机接管、模式切换、正式保存。
- `slam_scan_gate.py`、`slam_motion.py`：扫描时序、里程计传递、私有 TF 接管。
- `slam_checkpoints.py`：覆盖、调度、事务、哈希、冷校验与保留策略。
- `slam_guard.py`：运行时里程计/位姿保护与传感器元数据。
- `native/slam_graph_tool/`：使用同版 Karto 加载图和扫描、审计与栅格重建。
- webui 与 config：入口和状态；RTAB-Map 的 engine/launch/config 路径保持隔离。

上游依据：[2.6.10 保存/反序列化实现](https://github.com/SteveMacenski/slam_toolbox/blob/2.6.10/src/slam_toolbox_common.cpp)、[DeserializePoseGraph 定义](https://github.com/SteveMacenski/slam_toolbox/blob/2.6.10/srv/DeserializePoseGraph.srv)、[Karto Mapper](https://github.com/SteveMacenski/slam_toolbox/blob/2.6.10/lib/karto_sdk/src/Mapper.cpp)、[experimental lifelong](https://github.com/SteveMacenski/slam_toolbox/blob/2.6.10/src/experimental/slam_toolbox_lifelong.cpp)。本方案以本次固定版本的源码与实际实验为依据，不假定其他版本序列化兼容。

## House187 最新包重启增量实验（2026-11-08）

本轮先执行 `rbnx update`，逐项确认上游 main：Map `070248daab98`、Nav2 `0fbf1c78ecb3`、Explore `a2f5eefb7526`、Robonix `e66e4abebd9c`。Map 在上游基线上叠加本功能；Explore 只增加预算边界修复。实际源码和镜像版本均随实验导出，未提交或推送上游。

### 发现并修复的问题

1. **二维扫描的平面假设。** 早期实验中车体爬上家具，底盘高度从约 0.310m 升至 0.438m，随后出现扫描匹配偏移和里程计异常保护。`slam_planarity.py` 和 `slam_scan_gate.py` 现在按扫描时间戳读取完整 odom→laser TF，拒收超过 3° 倾斜或相对首次有效扫描超过 4cm 高度变化的观测。等待晚到 TF，保留原始时间戳；恢复水平后继续。该门控仅在 slam_toolbox 映射节点前使用。
2. **外侧墙体被栅格边界裁切。** Karto 的尺寸计算用 round 得到宽高，最大端点可能落在索引 width/height。`native/slam_graph_tool/padded_grid.h` 给重建栅格保留边界余量，保持占据阈值及最少穿越次数，不添加没有观测支持的墙体。四面墙端点原生回归覆盖这一问题。
3. **旧导出的增量初始化。** `_LocalizationBackend.prepare` 仅对 slam_toolbox 的 mapping 目标，在临时目录冷加载图并生成完整跟踪栅格。原始地图与检查点不改写；显式栅格编辑仍作为用户定位表面。普通 localization 和 RTAB 路径不启用该处理。
4. **部分地图的真实候选在粗搜索阶段丢失。** 仅对 slam_toolbox 增量初始化，把进入精匹配的候选预算由 24 扩为 256；评分、自动接受阈值、实时激光验证和交接限制保持原规则。实测原本缺失的正确远处位置成为第 5 个候选，按用户授权确认后完成交接。普通定位默认仍为 24。
5. **Explore 的预算边界。** 当最后一段导航在全局预算结束时超时，先检查任务 deadline，再累计导航失败，避免把正常预算结束误报为第 5 次导航失败。

### 实验与指标

使用指定 `scenesmith_house_187`、真实 MuJoCo 物理、模拟底盘/激光、Robonix Atlas/Executor/Soma、Map、Nav2 与 Explore。补接前置 RGB-D 生成深度障碍，避免 MID360 近处低矮障碍盲区；修正仿真深度图的光轴 Z 约定以及相机 3cm 横向外参。没有修改场景几何、家具或机器人碰撞模型。

最终对照保留同一份 89 节点基础图。初始 Explore 预算 300s，技能在约 133s 主动完成；增量预算 600s，技能在约 162s 主动完成。**没有跑满 300/600 秒，也没有宣称完成 600 秒耐久测试。** 剩余前沿为技能已尝试或无法到达的区域。

原始基础图导出已知面积 26.015m²；相同图按修复后的重建口径为 26.2675m²。最终图 30.3925m²，真正新增 4.125m²（约 15.7%）；补回旧边界的 0.2525m² 不计入增量。已知自由面积 24.455→28.060m²。节点 89→109，连通分量保持 1，所有旧节点保留且最大位移/转角为 0。

重启位置位于首次栅格已覆盖、但第一次轨迹没有到达的远处房间，距起点约 6.91m。它在新仿真创建前确定；odom 在每次仿真重启后归零。Map 仅接收基础地图 ID，未接收真实位姿。ground truth 仅用于评估、出生点审查及用户授权的实际候选确认。

固定首次 world→map 关系评估，原生增量位姿 RMSE 约 1.58cm；不可在重启后重新拟合轨迹来隐藏整体偏移。完整高频 TF+odom 误差、样本数和边界见交付 `summary.json`。

### 检查点开销与验证

本轮配置为新增已知覆盖 10m²、最短 10s、最长 30s 的周期保护。Native serialize_map 使用引擎图互斥锁获取一致文件对；后续冷加载、栅格、SHA256、fsync 和原子发布在后台执行，不发停车命令。不是零暂停，也不是与图规模无关的 O(1) 操作。

最终两个会话共 11 个检查点，全部文件校验与逐个冷加载通过。原生序列化 14.1–25.8ms，含后台处理的总时间 46.0–83.7ms；4 次保存窗口中观测到机器人运动。这些数字仅代表本轮约百节点图，不能外推到大型地图。

### 实际边界

- Map Python 回归 250 passed，2 个既有失败：候选点契约声明和 Docker 清单只读挂载断言。Explore 67 passed，另有原生端点回归和 ROS 扫描门控集成测试通过。
- 本轮未进行完整 RTAB 仿真验收；RTAB 路径隔离回归通过，新增条件均限制在 slam_toolbox 增量初始化。
- 仍记录到与长凳的接触，模型最大穿透约 0.335mm，不能称为无碰撞导航验收。
- 使用理想模拟位姿增量里程计；没有实车、编码器误差、多随机种子或任意出生位置可靠性保证。前置 RGB-D 已启用，未运行语义识别。
- 主要增量来自更合适初始视角，延长预算本身不会保证新增区域；实际初始化与后续探索面积增量分别记录。
- 局部更新/删除图节点仍属于单独方案；不能把擦除栅格等同于删除内部图节点。

Karto 行为依据及对照源码：[Karto 网格实现](https://github.com/SteveMacenski/slam_toolbox/blob/humble/lib/karto_sdk/include/karto_sdk/Karto.h)、[给定位姿恢复与最近节点匹配](https://github.com/SteveMacenski/slam_toolbox/blob/humble/lib/karto_sdk/src/Mapper.cpp)。本轮结论以运行数据、文件哈希和回归结果为准。

补充实测：已在独立会话调用 `/api/checkpoints/restore`。从最终 109 节点图恢复到 90 节点检查点，接入首帧后为 91 节点、1 个连通分量；原检查点、基础图和最终图文件 SHA256 均未变化。该验证回滚内部图，不只是替换 PGM。

倾斜门控需要上游提供完整 odom→laser 姿态；若硬件里程计强制只发布 yaw，必须补充 IMU 姿态或上游水平化扫描，不能凭二维里程计检测真实倾角。
