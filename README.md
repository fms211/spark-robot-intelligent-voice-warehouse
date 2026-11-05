# 基于 Spark 机器人的智能语音仓储场景开发

# Spark 语音巡检与取放课程设计（交付源码）

本目录是课程设计的交付源码，主体是 ROS 1 Noetic catkin 功能包 `spark_course_mission`，另含课程演示复用的旧 `move2grasp` 启动文件参考。功能包把中文语音指令、室内巡检导航、D435 蓝色目标检测与 uArm SwiftPro 吸盘取放编排成一条完整任务：

```text
语音/命令 START
  → PRECHECK
  → 逐站巡逻（NAV_PATROL → INSPECT）
  → 取物站检测蓝块（INSPECT；偏差时 ALIGN 原地对准）
  → PICK（机械臂取物 + 视觉复核）
  → 现场确认（WAIT_TRANSPORT_CONFIRM）
  → NAV_DROP → PLACE
  → RETURN_HOME → COMPLETE
异常/STOP → SAFE_STOP → FAILED / CANCELED
```

安全默认值：不启动底盘、机械臂为 mock、不启动语音；默认 `mission.yaml` 的 `valid_waypoints: false`、默认机械臂标定 `valid: false`，因此默认入口会拒绝 START。不要为了“先跑起来”修改这些布尔值或填入占位位姿。

## 1. 功能包介绍

### 1.1 分层设计

- `src/spark_course_mission/`：与 ROS 解耦的纯 Python 逻辑，pytest 可直接测试。
- `scripts/`：薄 ROS 节点，只做话题/服务/Action 接线，把事件转发给上面的逻辑。
- `launch/`、`config/`、`msg/`、`action/`、`srv/`、`test/`：启动入口、参数、接口与测试。

| 逻辑模块 | 职责 |
|---|---|
| `mission.py` | 任务状态机：预检、导航、巡检、对准、取放、现场确认、停止与超时 |
| `arm.py` | 标定校验、像素→机械臂映射、x/y/z/半径边界检查、PICK/PLACE 步序规划、`grasp_verdict` 可抓性判定 |
| `detection.py` | HSV 轮廓提取与 7 帧稳定跟踪（至少 5 帧一致才输出） |
| `alignment.py` | 取物对准窗口的构建与判定 |
| `navigation.py` / `waypoint_capture.py` | 站点配置/位姿校验；AMCL 采样校验与候选 YAML 输出 |
| `intent.py` / `voice_vad.py` / `voice_grasp.py` | 中文命令白名单、语音时序适配、旧语音取放桥接逻辑 |
| `calibration_grid.py` / `legacy_calibration.py` | 二维网格标定与旧 `move2grasp` 标定转换 |

| ROS 节点 | 作用 |
|---|---|
| `mission_manager.py` | 唯一任务决策者；调用 `move_base` 与 `/mission/pick_place`；发布状态/播报/当前站点；执行 PRECHECK 安全门槛 |
| `color_inspector.py` | 仅在站点激活时检测；输出 `/mission/inspection_result`；换站时重置跟踪器 |
| `swiftpro_adapter.py` | `PickPlace` action → `swiftpro_move` / `swiftpro_set_pump`；mock 记录动作，real 校验收标定、服务与固件应答 |
| `voice_intent.py` | `/voice/stt` → 白名单 `/mission/command`（开始巡检、停止任务、返回起点、查询状态） |
| `status_speaker.py` | `/mission/announcement` → `/voice/tts` |
| `waypoint_capture.py` | 只读采点，返回候选 YAML，不写配置、不导航 |
| `calibration_position.py` / `calibration_grid_run.py` / `*.sh` | 受监护的机械臂标定入口 |
| `import_move2grasp_calibration.py` / `voice_move2grasp_bridge.py` / `course_voice_asr.py` | 旧标定导入、旧语音取放桥接、语音 ASR 辅助入口 |

### 1.2 主要 ROS 接口

| 接口 | 类型 | 说明 |
|---|---|---|
| `/mission/command` | `std_msgs/String` | `START` / `STOP` / `RETURN` / `STATUS` |
| `/mission/status` | `std_msgs/String` | 当前状态（latched） |
| `/mission/announcement` | `std_msgs/String` | 任务播报，转发到 `/voice/tts` |
| `/mission/inspection_station` | `std_msgs/String` | 当前检测站点（latched，空串表示不在巡检） |
| `/mission/inspection_result` | `InspectionResult` | 稳定检测结果（站点、found、中心像素、面积、置信度） |
| `/mission/pick_place` | `PickPlace` action | `PICK=1` / `PLACE=2`；结果含 `success`、`carrying`、`code`、`detail` |
| `/mission/confirm_arm_stowed` | `std_srvs/Trigger` | 携物行驶前的现场确认 |
| `/mission/capture_waypoint` | `CaptureWaypoint` | 只读采点，返回 YAML 片段 |
| `/voice/stt`、`/voice/tts` | `std_msgs/String` | 由独立 `voice_fms` 工作区提供 |
| `/move_base` | `MoveBase` action | 外部导航栈 |
| `/odom`、`/scan`、`/amcl_pose`、`/map`、`/camera/color/image_raw` | 传感器/导航消息 | 预检与任务输入 |

### 1.3 现有状态（截至 2026-11-03）

- **有效标定**：`config/arm_calibration_field_20261103_measured.yaml`（`valid: true`、`calibration_mode: field`）。16 点二维网格 rank 3、均值 2.07 mm、最大 5.06 mm；`pick -44.72`、`approach -14.72`、`transport 100`；投放姿态 `(250,70,-44.72)`、运输姿态 `(240,30,100)`。
- **测试**：机器人上 `pytest` 206 passed / 1 skipped（2026-11-03）；8 项 rostest 见 §3.3。
- **现场验收**：2026-11-03 用测试配置跑通整机取放：patrol_b/c 巡检 → `bench_blue` 检测蓝块 `(258.5,262.5)` 判可抓 → PICK 成功 → 现场确认 → 导航到现场采集的投放点 `(0.420,-3.188,105.9°)` → PLACE 成功，蓝块落在投放区（相机与现场确认）。
- **未完成**：最后返航。投放点距墙约 0.57 m，`robot_radius 0.35` 加代价图膨胀使该位姿在局部规划中无合法轨迹（`min_vel_x: 0` 不能倒车），move_base 振荡超时；详见 `../progress.md`。
- **默认仍是安全模板**：`config/mission.yaml` 的 `valid_waypoints: false`、空站点；`config/arm_calibration.yaml` `valid: false`；`mission.launch` 默认 `enable_robot:=false`、`arm_mode:=mock`、`enable_voice:=false`。

## 2. 目录层级

```text
交付源码/
├── README.md                          # 本文件：交付说明与使用入口
├── move2grasp/
│   └── launch/move2grasp.launch       # 课程演示复用的旧启动文件（参考，不与本包并行运行）
└── spark_course_mission/              # ROS 1 Noetic 功能包
    ├── CMakeLists.txt
    ├── package.xml
    ├── setup.py                       # catkin_python_setup，安装 src/ 下的纯逻辑包
    ├── README.md                      # 包级详细说明与现场记录（含安全/标定历史）
    ├── launch/
    │   ├── mission.launch             # 主入口：底盘/相机/雷达/AMCL/move_base/任务节点
    │   ├── warehouse.launch           # 仓储演示包装：mission.launch + 语音桥接
    │   ├── localization_probe.launch  # 只读地图+AMCL 检查，底盘/雷达需显式开启
    │   ├── calibrate_grid.launch      # 受监护二维网格标定（不启动底盘/泵）
    │   └── calibrate_warehouse.launch # 复用旧仓储标定流程的受监护入口
    ├── config/
    │   ├── mission.yaml               # 现场站点（默认禁用/空）
    │   ├── mission_mock.yaml          # 合成测试站点（不得用于现场）
    │   ├── hsv.yaml                   # 蓝色 HSV、ROI、最小面积、跟踪阈值
    │   ├── navigation_course.yaml     # 课程 DWA：0.20 m/s、0.50 rad/s、半径 0.35
    │   ├── arm_calibration.yaml       # 标定模板（valid:false）
    │   ├── arm_mock.yaml              # mock 标定（仅测试）
    │   ├── arm_calibration_field_20261103_measured.yaml  # 当前有效标定
    │   ├── arm_calibration_grid_20261103_2d.yaml         # 网格候选（valid:false）
    │   └── arm_calibration_*_2026*.yaml                  # 历史候选，全部 valid:false
    ├── src/spark_course_mission/      # 纯 Python 逻辑（无 ROS 依赖）
    │   ├── mission.py  arm.py  detection.py  alignment.py
    │   ├── navigation.py  waypoint_capture.py  intent.py
    │   ├── calibration_grid.py  legacy_calibration.py
    │   └── voice_grasp.py  voice_vad.py  __init__.py
    ├── scripts/                       # ROS 节点与标定脚本
    │   ├── mission_manager.py  color_inspector.py  swiftpro_adapter.py
    │   ├── voice_intent.py  status_speaker.py  waypoint_capture.py
    │   ├── calibration_position.py  calibration_grid_run.py
    │   ├── import_move2grasp_calibration.py  voice_move2grasp_bridge.py
    │   ├── course_voice_asr.py
    │   └── calibration_prompt.sh  calibration_confirm.sh
    ├── msg/InspectionResult.msg
    ├── action/PickPlace.action
    ├── srv/CaptureWaypoint.srv
    └── test/                          # pytest 纯逻辑测试 + rostest 集成测试
        ├── test_mission_state.py  test_swiftpro_adapter.py  test_alignment.py  ...
        ├── mission_mock.test  mission_fault.test  arm_mock.test
        ├── default_launch.test  warehouse_launch.test
        ├── voice_intent.test  voice_fms_bridge.test  waypoint_capture.test
        └── mission_align_test.yaml    # 整机验收用的测试配置（仅测试文件启用站点）
```

## 3. 如何使用

### 3.1 环境

- 机器人：`spark@192.168.3.57`，工作区 `/home/spark/spark_noetic`，包路径 `~/spark_noetic/src/spark_course_mission`。
- 语音：独立 catkin 工作区 `/home/spark/spark_noetic/voice_fms`（ASR/TTS 由它提供）。
- 系统：ROS 1 Noetic、Python 3.8。本仓库所在开发机若只有 ROS 2，编译与 ROS 测试需在机器人上执行。

### 3.2 编译

```bash
ssh spark@192.168.3.57
cd ~/spark_noetic
source /opt/ros/noetic/setup.bash
source ~/spark_noetic/devel/setup.bash

# 工作区根目录的 catkin_make 已知会失败，显式 configure/build；暂时排除 multi_nav
cmake -S /home/spark/spark_noetic/src -B /home/spark/spark_noetic/build \
  -DCMAKE_PREFIX_PATH="/home/spark/spark_noetic/devel;/opt/ros/noetic" \
  -DCATKIN_DEVEL_PREFIX=/home/spark/spark_noetic/devel \
  -DCATKIN_BLACKLIST_PACKAGES=multi_nav
cmake --build /home/spark/spark_noetic/build \
  --target spark_course_mission_generate_messages swiftpro_write_node -- -j2
source /home/spark/spark_noetic/devel/setup.bash
```

### 3.3 无硬件验证

```bash
# 纯逻辑单元测试（需先生成消息）
python3 -m pytest src/spark_course_mission/test -q

# 只解析 launch，不启动节点
roslaunch --nodes spark_course_mission mission.launch arm_mode:=mock

# ROS 集成测试（隔离 Master、假 action、合成图像/里程计；不打开底盘/相机/雷达/机械臂/麦克风）
rostest spark_course_mission default_launch.test
rostest spark_course_mission warehouse_launch.test
rostest spark_course_mission mission_mock.test
rostest spark_course_mission mission_fault.test
rostest spark_course_mission arm_mock.test
rostest spark_course_mission voice_intent.test
rostest spark_course_mission voice_fms_bridge.test
rostest spark_course_mission waypoint_capture.test
```

`mission_mock.test` 驱动完整合成任务；普通 `roslaunch mission.launch` 不提供假导航，且默认配置会拒绝 START。

### 3.4 软件演示（安全默认值）

```bash
# 不启动底盘/相机/雷达，机械臂 mock，不启动语音；START 会被预检拒绝
roslaunch spark_course_mission mission.launch

# 仓储入口：在 mission.launch 基础上启动语音意图/播报桥接，仍不打开硬件
roslaunch spark_course_mission warehouse.launch

# 只读地图 + AMCL 检查；底盘与雷达默认关闭
roslaunch spark_course_mission localization_probe.launch start_base:=true start_lidar:=true
```

### 3.5 整机运行（现场，需监护）

运行前至少确认：地图与 AMCL 已收敛（XY σ≤0.25 m、yaw σ≤0.35 rad，2 s 内更新）、`map→base_footprint` TF 正常、`/scan` 健康、站点位姿是现场采集并核对过的、取物站唯一、标定为 `field`+`valid:true`、检测 ROI 与 `pixel_roi` 一致、`/dev/uarm` 与 D435 正常、已消除 TTS 回灌，且没有并行运行 `voice_nav`、旧 `move2grasp`、`spark_carry_object` 等竞争 `/cmd_vel` 或机械臂串口的节点。详细清单见 `../课程包运行指令与功能讲解.md`。

以 2026-11-03 标定为例（`mission_config` 必须替换为经现场核对的站点文件；`test/mission_align_test.yaml` 只是测试配置）：

```bash
roslaunch spark_course_mission mission.launch \
  enable_robot:=true arm_mode:=real enable_voice:=false enable_rviz:=false \
  arm_calibration_file:=/home/spark/spark_noetic/src/spark_course_mission/config/arm_calibration_field_20261103_measured.yaml \
  mission_config:=/home/spark/spark_noetic/src/spark_course_mission/test/mission_align_test.yaml \
  inspection_roi_x0:=102 inspection_roi_y0:=222 \
  inspection_roi_x1:=327 inspection_roi_y1:=330 \
  map_file:=/home/spark/spark_noetic/src/spark_app/spark_slam/scripts/test_map.yaml
```

正式现场任务应先把采集并核对的站点、投放点、起点写入 `config/mission.yaml`，经预检/现场确认后才把 `valid_waypoints` 置为 `true`。`valid_waypoints`、标定 `valid` 与站点位姿都不是“跑通开关”，而是现场安全门槛。

### 3.6 用户操作流程

1. **摆放目标**：把蓝色目标块放到取物站（`pickup_enabled: true` 的站点）相机视野内、标定 `pixel_roi` 覆盖的区域内。
2. **启动任务**：语音说“开始巡检”（需 `voice_fms` 的 ASR/TTS 在线，且已消除回灌），或手动注入：
   ```bash
   rostopic pub -1 /mission/command std_msgs/String "data: 'START'"
   ```
3. **观察状态**：`/mission/status` 显示 `NAV_PATROL → INSPECT → …`；`/mission/inspection_station` 显示当前站点；`/mission/announcement` 给出中文播报。
4. **现场确认**：PICK 成功后任务进入 `WAIT_TRANSPORT_CONFIRM`。先目视确认蓝块已被吸住、机械臂已收拢在运输姿态、与周围有间隙，再执行：
   ```bash
   rosservice call /mission/confirm_arm_stowed
   ```
   未确认不会导航到投放点；超时会安全停止。
5. **完成核对**：`COMPLETE` 后核对蓝块是否落在投放区。没有吸泵传感器，`COMMANDS_SENT_UNVERIFIED` 只表示命令与位姿被确认，物理抓取仍以现场/画面为准。
6. **停止**：语音说“停止任务”，或 `rostopic pub -1 /mission/command std_msgs/String "data: 'STOP'"`。紧急情况必须使用物理断电/急停：实测运动中拔掉底盘 USB，轮子会保持最后 PWM 继续转，串口断开时软件无法发零速。

### 3.7 站点采集与机械臂标定

站点采集（只读，返回候选 YAML，不写配置）：

```bash
rosrun spark_course_mission waypoint_capture.py
rosservice call /mission/capture_waypoint "kind: 'station'
station_id: 'patrol_a'
pickup_enabled: false"
# 投放点/起点用 kind: 'drop' / 'home'，station_id 留空
```

机械臂二维标定（受监护；生成候选默认 `valid:false`，高度/投放/运输姿态和独立点验证通过后才可置 `valid:true`）：

```bash
roslaunch spark_course_mission calibrate_grid.launch \
  port:=/dev/uarm feedrate_mm_min:=600
# 旧仓储标定流程参考：
roslaunch spark_course_mission calibrate_warehouse.launch \
  camera_type_tel:=d435 arm_port:=/dev/uarm
```

### 3.8 常用指令速查

```bash
rostopic pub -1 /mission/command std_msgs/String "data: 'START'"
rostopic pub -1 /mission/command std_msgs/String "data: 'STOP'"
rostopic pub -1 /mission/command std_msgs/String "data: 'RETURN'"
rostopic pub -1 /mission/command std_msgs/String "data: 'STATUS'"

rostopic echo /mission/status
rostopic echo /mission/inspection_result
rostopic echo /move_base/status
rosservice call /mission/confirm_arm_stowed
```

## 4. 安全边界与配置门槛

- 安全默认值：`enable_robot=false`、`arm_mode=mock`、`enable_voice=false`；`mission.yaml` 的 `valid_waypoints:false`、空站点；默认标定 `valid:false`。
- 不要翻转 `valid_waypoints`/标定 `valid`，也不要插入占位位姿来“让流程先走”。这些门槛对应未经现场核对的场地与标定状态。
- 实体机械臂要求 `calibration_mode: field`、`valid: true`、完整的 2×3 矩阵、`pixel_roi`、x/y/z/半径边界、递增的 pick/approach/transport 高度、投放/运输姿态，以及 `swiftpro_move`/`swiftpro_set_pump` 服务在线；实体 launch 使用 `service_only:=true` 关闭旧话题控制入口。
- PRECHECK 要求 move_base 与 PickPlace action、至少两个站点且恰好一个取物站、平面归一化位姿、新鲜图像/里程计/扫描/TF、AMCL 协方差门槛，以及严格的 `/cmd_vel` 发布者归属检查；存在 `voice_nav` 等竞争发布者时拒绝 START。
- STOP 会取消导航与机械臂 action、通过速度平滑输入发送零速并等待里程计归零；机械臂取消不会自动收回或释放吸泵，未知泵状态会阻止任务重启。
- 不要与旧 `move2grasp`、`spark_carry_object`、`spark_voice/voice_nav.launch` 并行运行：它们竞争 `/cmd_vel` 和/或机械臂串口。

## 5. 已知限制（2026-11-03）

- 整机验收的**返航段未完成**：投放点距墙 0.57 m，0.35 m 机器人半径叠加代价图内切膨胀后，该位姿在 DWA 中无合法轨迹且不能倒车。重新采集投放点/起点/patrol_a 时建议与墙保持 ≥0.8 m，或按实际装配调整足迹/膨胀参数后复测。
- `patrol_a` 的采集位姿与采集的 home 位姿都落在墙体的代价图膨胀区（代价 99），本次测试配置因此省略 patrol_a、home 暂用 patrol_c。
- ALIGN 的“窗口内但不可抓”比例对准已在单元测试覆盖，但 2026-11-03 实机取物时目标已在可抓区，该路径尚未实机触发。
- 重启 AMCL 后用 `/initialpose` 重新定位时使用紧协方差（如 0.0025，σ≈0.05 m）；方差 0.25（σ≈0.5 m）在该朝向会漂到错误盆地。
- 语音实体模式必须先消除扬声器回灌，只保留一个 TTS 节点；ASR/TTS 由独立 `voice_fms` 提供。

## 6. 相关文档

工作区根目录（本目录上一级）：

- `课程包运行指令与功能讲解.md`：主操作文档（现场状态、命令、限制、故障排查）。
- `课程设计完成度审计.md`：逐项需求证据与未验证项。
- `progress.md`、`task_plan.md`：按日期的进展与计划记录。
- `2026-11-02-取物对准设计.md`、`2026-11-02-取物对准实现计划.md`、`ALIGN实机测试记录_20261102.md`：ALIGN 设计与现场记录。
- `底盘异常诊断进展_20261103.md`、`机械臂实体取放验证_20261019.md`、`定位与底盘响应验证_20261026.md`：底盘/机械臂/定位证据。
- `交付源码/spark_course_mission/README.md`：包级详细说明、安全细节与标定历史。

> 说明：包内 README 中提到的 `docs/...` 在本工作区对应工作区根目录下的同名 Markdown 文件。
