# Spark 语音巡检与取放课程包

操作主文档：工作区 `docs/课程包运行指令与功能讲解.md`（按软件演示、单项功能、条件满足后的整机运行分节）。

`spark_course_mission` 把 `voice_fms` 提供的中文语音、室内巡检导航、D435 颜色检测与 uArm SwiftPro 取放编排成一条任务。`voice_fms` 是独立 catkin 工作区；本包通过 ROS 话题接入它，不复制云端账号配置。包内节点默认不会启动底盘；实体机械臂默认关闭。

## 组成

| 节点 | 输入 | 输出/作用 |
|---|---|---|
| `voice_fms/voice_03` | 麦克风、云端 ASR | `/voice/stt` 中文识别文本；讯飞和百度入口择一运行 |
| `voice_fms/voice_04` | `/voice/stt` | 自然语言对话回复 `/voice/tts`，不输出机器人动作指令 |
| `voice_fms/voice_05` | `/voice/tts` | 本机 `ekho` 播报；`voice_06` 的百度 TTS 脚本也订阅此话题，现场只选一个 TTS 节点 |
| `spark_voice_nlu`、`voice_07`（仅记录） | 手动移动意图或关键词 | 两个新增包本次不接入课程演示；其运动节点直接发布 `/cmd_vel` |
| `voice_intent.py` | `/voice/stt` | 白名单命令 `/mission/command` |
| `color_inspector.py` | D435 `/camera/color/image_raw`、当前站点 | `/mission/inspection_result`；7 帧中至少 5 帧一致 |
| `mission_manager.py` | 命令、检测、里程计和传感器状态 | `move_base` 与取放 action、任务状态/播报 |
| `swiftpro_adapter.py` | `/mission/pick_place` | mock 记录动作；实体模式通过 `swiftpro_move`、`swiftpro_set_pump` 服务等待固件应答与位置/吸泵回读，拒绝 `E20`～`E25` 和到位误差超限 |
| `status_speaker.py` | `/mission/announcement` | 将任务状态桥接到 `/voice/tts`，由选定的 TTS 节点播报 |

`/mission/status` 显示当前状态。`COMPLETE` 表示流程结束；SwiftPro 驱动的状态消息主要是命令回显，不是吸附传感器反馈。取物成功只能结合前后图像及现场观察判断。实体模式在抓取后进入 `WAIT_TRANSPORT_CONFIRM`，由现场人员确认机械臂已收拢才继续行驶。

取物动作先在运输净空高度移到目标 XY，再垂直下降、开泵，沿原 XY 垂直抬到接近高度和运输净空高度，最后水平移到运输姿态；投放先在运输高度水平移到投放 XY，再逐级下降，关泵后竖直抬高并水平回到运输姿态。每段仍须现场确认可达、与障碍保持间隙。收到取消时不再自动发送未经验证的收回路径，而是保持当前关节姿态与吸泵状态，等待现场接管；若开泵命令的回执丢失，保守标记为可能携物并禁止任务重启。

## 编译与无硬件验证

```bash
ssh spark@192.168.3.57
cd ~/spark_noetic
source /opt/ros/noetic/setup.bash
source devel/setup.bash
cmake -S /home/spark/spark_noetic/src -B /home/spark/spark_noetic/build \
  -DCMAKE_PREFIX_PATH="/home/spark/spark_noetic/devel;/opt/ros/noetic" \
  -DCATKIN_DEVEL_PREFIX=/home/spark/spark_noetic/devel \
  -DCATKIN_BLACKLIST_PACKAGES=multi_nav
cmake --build /home/spark/spark_noetic/build \
  --target spark_course_mission_generate_messages swiftpro_write_node -- -j2
source devel/setup.bash
python3 -m pytest src/spark_course_mission/test -q
roslaunch --nodes spark_course_mission mission.launch arm_mode:=mock
rostest spark_course_mission default_launch.test
rostest spark_course_mission mission_mock.test
rostest spark_course_mission arm_mock.test
rostest spark_course_mission voice_intent.test
rostest spark_course_mission voice_fms_bridge.test
rostest spark_course_mission waypoint_capture.test
```

构建时暂时排除既有 `multi_nav`，因为其 `CMakeLists.txt` 写的是 `scripts/multinav.py`，实际文件为 `scripts/multiNav.py`。此包不依赖 `multi_nav`。上面 `--nodes` 只解析启动文件；普通 `roslaunch` 会启动相应节点。

`mission_mock.test` 只启动本包节点及测试进程中的假 `move_base`/机械臂 action server，并发布合成图像和里程计；`arm_mock.test` 则启动**真实的本包 SwiftPro 适配节点**，验证 mock action 的取物、放置与携物状态；`voice_intent.test` 向真实意图节点发布模拟中文识别文本，验证白名单输出；`voice_fms_bridge.test` 验证任务播报转发至 `/voice/tts`；`waypoint_capture.test` 用合成 AMCL 位姿验证采点服务。`default_launch.test` 还检查默认入口实际启动，并拒绝未配置站点的 START；六项测试均不打开底盘、相机、雷达或机械臂，也不采集麦克风或调用云端服务。`config/mission_mock.yaml` 和 `config/arm_mock.yaml` 的坐标仅供测试，不能用于实物。

## 接入独立语音工作区

在运行语音节点的终端，先把两个工作区叠加，确认 `voice_03`、`voice_04`、`voice_05` 可由 `rospack` 找到：

```bash
source /opt/ros/noetic/setup.bash
source ~/spark_noetic/devel/setup.bash
source ~/spark_noetic/voice_fms/devel/setup.bash
rospack find voice_03
rospack find voice_04
rospack find voice_05
```

`mission.launch enable_voice:=true` **仅启动本包的意图解析与播报桥接**。本次课程演示仍从 `voice_03` 的 `xfyun_asr_ros.py`、`fms_asr.py` 中选择一个 `/voice/stt` 发布者；`voice_04/xf_nlu.launch` 可负责自由文本对话，`voice_05/local_tts_test.py` 或 `voice_06/baidu_tts.py` 择一播报。用户决定新增 `spark_voice_nlu` 和 `voice_07` **先只记录，不接入课程演示**：其现有意图仅支持手动移动，运动节点还会直发 `/cmd_vel`。本包任务命令只来自 `/voice/stt` 白名单；自由文本回复不经 ROS 话题触发底盘或机械臂。详见工作区 `docs/voice_fms环境与接口.md`。

默认 `mission.yaml` 中 `drop_pose: []`、`home_pose: []` 是未配置占位符，仍会被预检拒绝；默认未标定矩阵 `matrix: []`、`valid: false` 也不可用于实体抓取。`arm_mode:=real` 启动时可能使机械臂关节上电，只有在现场检查、监护与有效标定完成后才启用。

## 实体运行前检查

1. 从现有可用地图中确定与当前室内场地对应的 YAML 路径，验证 `/map`、`/amcl_pose` 和 `map→base_footprint` TF；在 RViz 设定初始位姿。
2. 在 `config/mission.yaml` 填写至少两个实际巡检点、取物许可、投放点与起点位姿。每个位姿为 `[x,y,z,qx,qy,qz,qw]`，所有值须有限，`z/qx/qy` 须接近 0，四元数范数须接近 1；`pickup_enabled` 必须是 YAML 布尔值，且当前只允许一个取物站，因为机械臂仍共用一套相机映射标定。无效配置会在 PRECHECK 阶段拒绝。现场核对后才把 `valid_waypoints` 改为 `true`。地图候选和历史 A/B 点的错误格式见 `docs/地图候选与现场坐标核对.md`。
3. 检查 `/dev/sparkBase`、`/dev/ydlidar`、D435 视频话题、`/scan` 与 `/odom`。本包默认 `camera_type_tel:=d435`、`lidar_type_tel:=ydlidar_g6`；2026-10-12 独立驱动测试已确认硬件为 G6，12 秒获得 88 帧 `/scan`，但重启后 `ttyUSB` 序号与旧记录互换，应始终使用 udev 设备名。实体 PRECHECK 还要求最新扫描有坐标系、至少 60 束有限距离且占总束数至少 20%，并能从扫描坐标系变换到 `base_footprint`；AMCL 须在 2 秒内更新，满足与采点服务相同的平面姿态和协方差门槛。PRECHECK 会查询 ROS Master：`/cmd_vel` 只允许导航 nodelet manager 发布，速度平滑输入只允许 `move_base` 与本任务节点发布；存在 `voice_nav` 等竞争发布者或查询失败时拒绝开始任务。这些软件门槛不代替现场检查雷达覆盖、定位稳定性与避障效果。
4. 用适合吸盘的轻小平顶物体标定 `config/hsv.yaml` 的颜色范围与 ROI。默认蓝色阈值只是起点；须实测光照下误检和漏检。
5. 连接 uArm SwiftPro，确认稳定设备名 `/dev/uarm`（已有 udev 规则会跟随 `ttyACM0/1` 重枚举）；在底盘静止且有人监护时重新标定 `config/arm_calibration.yaml` 的像素到机械臂映射、**有效像素区域 `pixel_roi.u/v`**、工作空间 `bounds_mm.x/y/z` 与**径向 `bounds_mm.radius`**、取放高度、**收拢运输姿态 `transport_mm`** 与投放姿态。径向范围须在官方标称 50～320 mm 内，实体目标超出像素区域或径向边界会拒绝，但径向范围内也可能因关节几何限制被固件拒绝。旧 `~/thefile.txt` 不直接沿用。`calibration_mode` 必须为 `field` 且 `valid: true`，实体适配节点才会启动；`swiftpro_move` 和 `swiftpro_set_pump` 服务必须在线，实体动作等待固件应答，并用 `P2200` 编码器角度经 `M2221` 换算的末端坐标判定到位；`P2220` 仅为规划器内部坐标，不作为独立到位证据。位置偏差或编码器查询失败后锁定后续移动/开泵，须检查并重启驱动，仍允许尝试关泵。课程 launch 对 SwiftPro 写节点启用 `service_only`，关闭旧位置/吸泵话题入口，避免并发控制。写节点的 `feedrate_mm_min` 现可配置，默认 `3000`（50 mm/s），有效范围 1～6000 mm/min；课程 launch 默认传入已验证的 `600`（10 mm/s）。2026-10-19 已以此速度完成固定点自动吸取、短距搬运和放下，用户确认稳定分离、全程无异常；通用驱动默认 `3000` 尚未实体验证。
   2026-10-19 手持轻触时 `/dev/ttyACM0` 曾瞬时消失，内核记录同一 Arduino USB 多次断开重连；在静止收臂后虽能只读设备名、位置和泵 V0，但随后又连续重枚举；现场确认线缆有余量、插头稳固，自动休眠累计时间为 0，故障点尚未隔离。随后保持单一串口连接完成固定点自动取放，过程中 USB 设备编号未变；历史故障点仍未完全隔离。默认通用标定仍禁用；已实测的局部配置见文末，仅覆盖限定像素区域。
6. 语音实体模式须先消除扬声器回灌：TTS 播放期间暂停 ASR 或启用经现场验证的回声抑制；只选一个 TTS 节点。保持 `enable_robot:=false`，播放四类命令和含命令的 NLU 回复，确认 `/mission/command` 不因机器自己的声音出现 START/RETURN，再允许语音触发实体任务。

### 静止定位诊断入口

`localization_probe.launch` 只组合机器人描述、`test_map.yaml` 和 AMCL；默认 `start_base:=false start_lidar:=false`，不开串口，也不启动 `move_base`、相机或机械臂。只读检查：

```bash
roslaunch --nodes spark_course_mission localization_probe.launch start_base:=true start_lidar:=true
```

在现场人员确认底盘空闲后，先确认没有其他节点占用 `/dev/sparkBase`、`/dev/ydlidar` 或发布 `/cmd_vel`。实体静止联测使用**独立 ROS Master**；在一个终端启动 `ROS_MASTER_URI=http://127.0.0.1:11324 roscore -p 11324`，另一个终端显式 `source /opt/ros/noetic/setup.bash`、`source ~/spark_noetic/devel/setup.bash`、`export ROS_MASTER_URI=http://127.0.0.1:11324`，再运行：

```bash
roslaunch spark_course_mission localization_probe.launch start_base:=true start_lidar:=true
```

该入口不发布速度命令。核查 `/scan`、`/odom`、`/map`、`/amcl_pose` 和 `map→base_footprint`；在 RViz 设置真实初始位姿，并对照雷达与 `test_map.yaml` 的障碍轮廓，不可仅凭 `/amcl_pose` 存在就认为定位成功。结束时正常停止本次 launch 和独立 Master，不触碰现场其他 ROS 进程。已进行 G6、底盘里程计、地图与 TF 的实体联测及受监护小幅原地转动；2026-10-19 受监护小角度转动与约8.6cm直行的雷达/里程计测量相符，现场确认动作和停止正常；静止规划返回候选路径，但存在跨观测间隔朝向变化尚未解释，尚未发送自主导航Action。详见 `docs/底盘动态定位验证_20261019.md`。

### 现场采集候选导航坐标

地图和 AMCL 已验证、机器人停稳后，另开终端启动只读采点节点：

```bash
source /opt/ros/noetic/setup.bash
source ~/spark_noetic/devel/setup.bash
rosrun spark_course_mission waypoint_capture.py
```

在同一 ROS Master 的另一终端，站在目标点并确认定位稳定后调用服务。例如先记录一个普通巡检点，再记录唯一取物点、投放点和起点：

```bash
rosservice call /mission/capture_waypoint "kind: 'station'
station_id: 'patrol_a'
pickup_enabled: false"

rosservice call /mission/capture_waypoint "kind: 'station'
station_id: 'pickup_b'
pickup_enabled: true"

rosservice call /mission/capture_waypoint "kind: 'drop'
station_id: ''
pickup_enabled: false"

rosservice call /mission/capture_waypoint "kind: 'home'
station_id: ''
pickup_enabled: false"
```

服务读取最新 `/amcl_pose`，只接受 `map` 坐标系、1 秒内时间戳、平面单位四元数，以及 XY 标准差不超过 0.25 m、航向标准差不超过 0.35 rad 的定位样本。阈值可用节点私有参数 `~max_amcl_age_s`、`~max_xy_sigma_m`、`~max_yaw_sigma_rad` 调整；全零协方差视为未初始化并拒绝。成功响应的 `yaml_snippet` 只是候选片段：须与 RViz、障碍物、底盘转向空间和机械臂工作区核对，再人工写入 `config/mission.yaml`。采点节点不会写文件、发送导航目标、发布速度或启用 `valid_waypoints`。

2026-10-08 已识别 SwiftPro 的 `/dev/ttyACM0`，查询设备名与电源状态成功。2026-10-19 现场在清空扫掠区后单独发送 `M17`，前后读回坐标不变且现场无异常；紧凑姿态的一次 `F600`、Z 轴目标增加 5 mm 命令被固件以 `E22 unreachable` 拒绝，三次位置读回不变；随后在已清空的空位用带应答服务完成上移 5 mm 和原路返回，读回均到位、现场确认平稳。该空位单步不代表取放路径可达；这属于早期单步记录；随后局部实体 PICK/PLACE 已于同日通过现场确认，见文末。`swiftpro_write_node` 启动时先查询 `P2234`，只有外接电源 `V1` 才会发送带应答的 `M17`；移动和开泵服务也先复查供电，关泵服务仍可尝试释放。没有收到有效供电状态时拒绝后续动作；须在底盘静止、现场有人监护且标定完成后才启动实体模式。未完成上述检查时保持 `enable_robot:=false`、`arm_mode:=mock`。

2026-10-19 初期只用官方查询命令读到 SwiftPro 末端 `(78.30,-49.92,31.26) mm`、吸泵停止和电源连接；该时点未发送程序驱动的机械臂动作。D435 首次复核时又朝向天花板，现场重新固定镜头、移动蓝块后，完整露出的顶面在 20/20 帧中被检出，中心约 `(311,178)`、深度约 607 mm。**此处为当日早期画面，不是后续实测标定点；10 月 12 日旧坐标不能直接用于取物。后续局部映射及有效区域见文末。** 详情见 `docs/course-demo-verification.md`。

## 启动与停止

确认地图、站点、机械臂和标定后，才使用实体模式：

```bash
roslaunch spark_course_mission mission.launch \
  enable_robot:=true arm_mode:=real enable_voice:=true \
  arm_port:=/dev/uarm arm_feedrate_mm_min:=600 \
  map_file:=/现场已验证地图/地图.yaml
```

`mission.launch` 的 `enable_robot` 和 `enable_voice` 默认都是 `false`，`arm_mode` 默认 `mock`。语音白名单：**开始巡检、停止任务、返回起点、查询状态**。`enable_voice:=true` 不会替你启动 `voice_fms`；无麦克风时，可在检查完成后用 ROS 话题发送高层命令：

```bash
rostopic pub -1 /mission/command std_msgs/String "data: 'START'"
rostopic pub -1 /mission/command std_msgs/String "data: 'STOP'"
```

实体模式抓取完成且 `/mission/status` 显示 `WAIT_TRANSPORT_CONFIRM` 时，**现场确认机械臂实际处于配置的收拢运输姿态**，再执行：

```bash
rosservice call /mission/confirm_arm_stowed
```

确认服务会复查底盘静止状态；未确认前不会下发投放点导航目标。停止命令取消导航和未完成的机械臂 action，并通过现有速度平滑输入通道反复发送零速、等待里程计归零；若吸泵状态未知或可能仍持物，阻止新任务并提示人工接管。现场物理急停和人员监护不能由软件命令替代。2026-11-03 实测：运动中拔掉底盘 USB，轮子会保持最后 PWM 继续转；串口断开时软件无法发零速；插回后驱动恢复并在约 1 秒内归零。请勿同时运行旧 `spark_voice/voice_nav.launch` 或 `spark_carry_object` 搬运入口，它们可能与本包竞争 `/cmd_vel`；PRECHECK 仅在任务开始时检查发布者，不能仲裁运行途中新增的控制节点。语音工作区同一时刻也只保留一个 ASR 发布者。

## 取物站点自转对准（ALIGN）

取物站（`pickup_enabled: true`）在目标像素超出标定 `pixel_roi` 内缩窗口，或持续未检出目标时，用 move_base 原地旋转（30° 搜索、10° 微调，累计最多 360°）把目标转回可抓窗口，再进入 PICK。窗口内的小偏差仍由机械臂 x/y 映射补偿，底盘不转；仅垂直方向超出窗口时不做旋转，直接进入下一站。窗口内但映射超出标定 x/y/radius 时，不再直接 PICK：按适配器同一 `_check_target` 判据朝最近可抓像素列比例微调（从上一旋转学习 px/deg，首步 2°）；旋转后的 `align_settle_s` 内丢弃仍含运动帧的跟踪判决；无可行列则跳过本站。

配置见 `config/mission.yaml` 的 `align` 段；窗口 ROI 从 `arm_calibration_file` 的 `pixel_roi` 自动读取（launch 同时加载到 `/spark_mission_manager/alignment_calibration`）。旋转只用 move_base，不直接发布 `/cmd_vel`；STOP 会取消旋转并发布零速。该行为已通过单元测试和 mock ROS 测试；2026-11-03 实机取物（目标已在可抓区）通过，窗口边缘的比例对准尚未实机触发。

## 课程演示记录

记录地图文件、巡检点、HSV 与标定参数、实际串口；保留 `/mission/status`、`/mission/inspection_result`、`/move_base/status`、`/odom` 与机械臂动作日志。演示视频应清楚展示语音启动、至少两个巡检点、目标稳定识别、吸取、投放和“停止任务”中断。将软件模拟与实体实测分别标注；没有吸附传感反馈时，不把“命令已发送”写成“确定抓取成功”。


## 2026-10-19 局部实体取放标定

已用 F600 完成固定点自动取放和持物收回，现场均确认稳定、无碰撞。B/C/A2 三个完整可见点拟合像素映射，独立 D 点平面误差1.69mm。部分遮挡的 A 点已剔除。课程原生颜色节点输出稳定检测后，实体 PICK Action 已自动吸起并收回蓝块；随后 PLACE Action 完成指定位置释放、抬离及收臂。现场确认两项动作全程无异常。

实测配置保存在 `config/arm_calibration_20261019_local.yaml`。只接受蓝色顶面**中心** `u=290..306, v=258..272`，同一固定相机装配、640×480图像、同一蓝块顶面高度与平坦地面；开始时机械臂须位于已核对的 `(180,0,100) mm` 运输姿态。这个区域仅厘米量级，当前不代表任意位置抓取、导航停车后可直接取物或持物导航通过。默认 `arm_calibration.yaml` 仍无效，避免把局部标定当成通用参数。

后续课程集成可显式选择这份实测标定（先检查物体、相机和起始臂姿态）：

```bash
roslaunch spark_course_mission mission.launch \
  arm_mode:=real arm_port:=/dev/uarm arm_feedrate_mm_min:=600 \
  arm_calibration_file:="$(rospack find spark_course_mission)/config/arm_calibration_20261019_local.yaml"
```

该入口会给机械臂关节上电。`mission.yaml` 的导航点仍未启用，完整任务须完成动态定位、站点与真人语音验收。不要与本次隔离测试写节点同时打开同一串口。详细证据、局部有效区域和复现方法见 `docs/机械臂实体取放验证_20261019.md`。

## 2026-11-02 二维标定入口（诊断）

`calibrate_grid.launch` 在已验证的 2026-11-02 20 点直线附近增加小范围横向偏移，采集二维网格，拟合 2×3 像素→机械臂矩阵，并输出 `valid:false` 的候选 YAML。它不替代取放高度、z 边界、投放点和运输姿态的现场测量。

```bash
roslaunch spark_course_mission calibrate_grid.launch
# 另一终端，在预览窗口把标定贴片对准绿框后：
rosservice call /spark_calibration_grid/start
```

- 默认网格：先到旧直线的已知安全点 `(180,200,-110) mm`，再围绕 `x=180+5i, y=200-10i` 的 5 个基点做 `±10 mm` 横向偏移（共 16 点，含 warm-up）；可用 `~grid_points` 覆盖。默认安全包络为 x 170–285、y 0–210、z -120–40 mm。
- 启动条件：扫掠区清空、吸盘无负载、D435 能看到标定贴片、有人监护；`swiftpro_write_node service_only:=true`、`F600`，不打开吸泵。
- 驱动新增 `position_tolerance_mm`，默认 1.5 mm；`calibrate_grid.launch` 单独设为 2.5 mm。`mission.launch` 不设置该参数，课程取放仍使用 1.5 mm 门槛；2.5 mm 只用于标定采样，候选仍 `valid:false`。机器人已备份原驱动源码并重建成功，14 项测试与 launch 解析通过。
- 每个点由 `/swiftpro_move` 返回编码器位置并核对 ≤1.5 mm；检测失败、点位跳变或运动失败时停止，不发送未经验证的收回路径，机械臂停在当前点由现场接管。
- 预备位到旧直线安全点的长移动按 ≤60 mm 分段调用 `swiftpro_move`，避免驱动单条命令 15 s 超时；每段仍做编码器核对。
- 输出 `config/arm_calibration_grid_candidate.yaml` 与 `..._samples.json`；候选保持 `valid:false`、`calibration_mode:uncalibrated`，`z` 边界、`heights_mm`、`drop_mm`、`transport_mm` 均未填写，不能直接用于实体取放。
- 预览窗口需要机器人桌面；从 SSH 运行需设置可用的 `DISPLAY`。若输出文件已存在，脚本拒绝覆盖，请通过 `output_yaml` / `samples_json` 换新文件名。
- 纯逻辑测试：`PYTHONPATH=src python3 -m pytest test/test_calibration_grid.py test/test_calibration_grid_launch.py -q`（14 项）。
- 已部署到机器人并重建；完整课程 pytest 164 passed/1 skipped；尚未现场采集。

## 2026-11-02 历史标定（已失效）

相机在该次标定后调整过，此标定已失效，配置已设为 `valid: false`，不得用于当前实体取放。以下数据仅保留历史记录：矩阵/ROI 来自当日 16 点 2D 网格（run3，均值 1.92 mm、最大 4.82 mm）；pick `-45.75 mm`、approach `-15.75 mm`、transport `100.0 mm`；运输姿态 `(240, 30, 100)`、投放点 `(250, 70, -45.75)`。

- 受监护验证：下降 → 开泵（state=1）→ 抬升 → 运输姿态 → 投放点下降 → 关泵（state=0）→ 回 home；现场图确认吸住、运输、放下。
- 半径下限：run3 的 `235.8 mm` 是旧直线采样的径向范围，不覆盖实际目标（实测 `217.4 mm`）；依据本次实体证据与独立点验证放宽到 `200.0 mm`，上限保持 `281.1 mm`。理由与验证点写在 YAML 的 `field_record`。
- 独立点验证：机械臂到预测点 `(205.9, 69.8, z=20)`，摄像头确认吸盘正对蓝块。
- 这份历史配置及其 ROI 不可作为当前实体入口的参数。当前候选为 `config/arm_calibration_20261103_candidate.yaml`，保持禁用；须验证当前二维工作区域、取放高度和路径后才能启用。
- 尚未通过实体 `mission` 的 PICK/PLACE Action 走完整任务；当前验证为直接服务调用与现场图确认。

## 2026-11-03 当前装配有效标定（实测）

`config/arm_calibration_field_20261103_measured.yaml`：`valid: true`、`calibration_mode: field`。矩阵/ROI 来自当日 16 点二维网格（rank 3、均值 2.07 mm、最大 5.06 mm）；pick `-44.72 mm`（吸盘接触实测）、approach `-14.72 mm`、transport `100.0 mm`；运输姿态 `(240, 30, 100)`、投放点 `(250, 70, -44.72)`。像素 ROI 为 u[101.8,326.6]、v[221.97,329.75]；实体入口使用 `inspection_roi_x0:=102 inspection_roi_y0:=222 inspection_roi_x1:=327 inspection_roi_y1:=330`。

- 受监护验证：开泵（state=1）→ 抬升 → 运输姿态 → 投放点下降 → 关泵（state=0）→ 回 home；画面确认吸住、运输、放下。
- 独立点：蓝块像素 `(312,255)` → 预测臂 `(240.9,31.5)`、半径 242.9，吸盘正好接触蓝块顶面。
- 使用：实体入口显式加载本文件与上述 ROI；`mission.yaml` 仍 `valid_waypoints:false`，须先启用站点并完成预检。
- 11-02 的 measured 文件和 field candidate 均已 `valid:false`，不要启用。

### 2026-11-03 整机验收（取放完成，返航未完成）

- `test/mission_align_test.yaml`（仅测试文件启用站点）：patrol_b/c → bench_blue 检测 `(258.5,262.5)` 判可抓 → PICK SUCCEEDED → 现场确认 → NAV_DROP（采集投放点 `(0.420,-3.188,105.9°)`）→ PLACE SUCCEEDED，蓝块落在投放点（相机/现场确认）。
- 返航未完成：投放点距墙约 0.57 m，`robot_radius 0.35` 与代价图内切膨胀使该位姿在局部规划中无合法轨迹（`min_vel_x: 0` 不能倒车）；move_base 报 oscillating / no valid control。人工推挪约 1 cm、teleop 未连接、home 区被动态障碍占用，按现场决定结束。
- ALIGN 同时增加可抓性判据（`grasp_verdict`，复用适配器 `_check_target`）、比例微调（学习 px/deg、2° 探针）与 `align_settle_s` 稳定窗口；单元测试与 mock ROS 测试通过，窗口边缘路径尚未实机触发。
- 复测：投放点/起点/patrol_a 与墙保持 ≥0.8 m 或调整足迹/膨胀参数后，补做返航闭环与窗口边缘 ALIGN。

## 取物后的视觉复核

PICK 成功后任务重新检测：目标若仍在取物前像素附近（`pick_verify_motion_px`，默认 25 px）→ `FAILED`；明显位移（随机械臂抬到运输姿态）→ 视为已取走继续。该检查只区分“目标是否离开原位”，不是吸附传感器；`COMMANDS_SENT_UNVERIFIED` 与画面/现场确认仍需执行。

## 语音→原 move2grasp 桥接

`scripts/voice_move2grasp_bridge.py` 把课程白名单 `/mission/command` 翻译成原 `move2grasp` 的 `/grasp` 话题：`START`→`1`，`STOP`/`RETURN`→`0`，`STATUS`/其他不动作。它不导航，也不直接动机械臂；识别、吸泵和释放仍由原 `grasp_d435.py` 负责。

```bash
# 先验证映射，不发 /grasp：
rosrun spark_course_mission voice_move2grasp_bridge.py _dry_run:=true
rostopic pub -1 /mission/command std_msgs/String "data: 'START'"
rostopic echo -n1 /spark_voice_move2grasp/status

# 真实模式：_dry_run:=false，并确保原 GraspObject 和 D435 已运行。
```

当前真实取放使用的是 2D 候选矩阵补丁版 `grasp_d435.py`；桥接本身不修改原包。导航 (`move.py`/`move_base`) 未包含在此桥接中。

### 课程导航参数与动作期限

`navigation_config` 默认加载 `config/navigation_course.yaml`：直行上限0.20m/s、转动上限0.50rad/s、关闭自动旋转恢复、振荡超时30s，两张代价地图半径0.35m。参数需结合实际装配与场地验收，不能仅凭参数加载成功就运行完整任务。F600实测PICK约44秒、PLACE约47秒，任务 `arm_timeout_s` 设90秒，保留超时取消。
