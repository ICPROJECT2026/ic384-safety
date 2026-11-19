# 安全识别系统

本仓库是从 IC384 项目单独整理出来的 ROS2 安全识别模块，可以整体复制到 Jetson。功能为人员检测、深度测距、底盘停车、人员离开自动恢复及故障复位。ROS2 模块不依赖原 IC384 项目的 Python 代码。

GitHub 上传方法见 [上传说明](GITHUB_UPLOAD.md)。本包保留源码、测试和 YOLO11n 权重，排除了本机编译产物、缓存及原项目其他模块。

附带 [Mac 相机接入与检查](mac_camera/README.md)：使用另行准备的 Astra Pro Mac 工具包读取深度，不依赖 ROS2。该工具包不包含在本仓库内，运行时通过 `--kit` 指定其位置。该入口目前仅用于深度诊断，尚未接通人员识别或底盘控制；完整人员识别仍使用下文的 ROS2 版本。

## 文件说明

```text
ic384-safety/
├── README.md                         使用说明
├── models/                           已下载的 YOLO11n 权重及来源记录
├── mac_camera/                       Mac 深度接入程序、检查入口与 6 项测试
└── ros2_ws/src/ic384_safety/
    ├── ic384_safety/
    │   ├── __init__.py
    │   ├── person_safety.py           相机输入、人员检测、深度测距
    │   ├── person_safety_core.py      距离阈值、滞回与数据时效判断
    │   └── safety.py                  底盘停车、自动恢复与故障复位
    ├── launch/person_safety.launch.py 启动入口
    ├── test/test_person_safety.py     41 项本地逻辑测试
    ├── resource/ic384_safety          ROS2 包标记
    ├── package.xml                   ROS2 依赖说明
    ├── setup.py                      安装与程序入口
    └── setup.cfg                     ROS2 安装配置
```

预训练模型权重已放在 `models/yolo11n.pt`，无需自行训练。相机驱动、ROS2 与 Jetson 推理环境仍需另外准备。当前启动配置只使用相机，不要求超声波、急停回路或机械臂收拢等外部反馈；本模块只控制底盘停车，不控制机械臂停止。

## 与机器人连接的接口

| 接口 | 类型 | 用途 |
|---|---|---|
| `/cmd_vel_requested` | `geometry_msgs/Twist` | 导航或遥控发来的期望速度 |
| `/cmd_vel_safe` | `geometry_msgs/Twist` | 经过安全判断的速度，交给底盘驱动 |
| `/safety/input/person_clear` | `std_msgs/Bool` | 人员节点输出的通行条件 |
| `/safety/input/estop_ok` | `std_msgs/Bool` | 可选硬件模式：真实急停回路反馈 |
| `/safety/input/obstacle_clear` | `std_msgs/Bool` | 可选硬件模式：障碍物传感器反馈 |
| `/safety/input/arm_stowed` | `std_msgs/Bool` | 可选硬件模式：机械臂收拢状态反馈 |
| `/safety/status` | `std_msgs/String` | 安全监督状态 |
| `/safety/person_status` | `std_msgs/String` | 人员检测与深度判断状态 |
| `/safety/reset` | `std_srvs/Trigger` 服务 | 首次启动/其他故障人工复位；人员离开无需复位 |

当前速度控制适用于差速底盘，只使用 `linear.x` 和 `angular.z`。默认 `person_safety.launch.py` 只要求相机人员输入，不订阅表中列出的三路可选硬件反馈；相机输入缺失或异常仍会停车。

## 运行说明

下文命令均从本仓库根目录开始执行。先加载目标机器已安装的 ROS2 环境；具体发行版按 Jetson 实际安装情况选择。


### 已有硬件记录

目标相机为 **ORBBEC ASTRA PRO RGB-D**，数量 **1 台**。当前配置只使用这一台 RGB-D 相机，不接入超声波或其他外部硬件反馈，也不使用额外 RGB 相机。运行主机为 **NVIDIA Jetson**；具体型号、JetPack 与 ROS2 发行版尚未确认。项目已有资料记录的标称深度范围为 0.6–8 m、深度分辨率为 640×480 / 30 fps，尚未由实机验证。

人员节点的深度有效范围初始设为 `depth_min:=0.6 depth_max:=8.0`，可在启动时调整。0.6 m 以内不具备可靠测距保证；过滤无效值并不能证明近处无人。当前没有其他传感器补充该相机的近距离盲区。相机驱动的话题名、单位及 RGB/深度对齐仍需实机核对，不能由型号推断已经对齐。

新增 `person_safety` 节点：本地 YOLO 检测画面中的所有人员，用对齐深度图估计每个人框内最近的有效光轴深度，输出 `/safety/input/person_clear`。人员距离不大于 `stop_distance` 时禁止行驶，达到 `release_distance` 后才解除该视觉输入的阻塞。默认分别为 **1.0 m / 1.5 m**，仅供离线或驱动断开的验证使用，尚未按机器人速度、外形、安装位置、延迟及制动距离确定。

距离是相机光轴深度，不是人与机器人外壳的净距离。框内前景物体也可能提前触发停车。算法不做人体表面重建；局部深度缺失可能留下背景深度，人员漏检也无法由此逻辑排除。视野外区域没有人员检测覆盖。

### 准备与启动

在安装 ROS2、相机驱动以及上述包依赖的目标机器上，给运行 ROS2 节点的同一 Python 环境安装 `ultralytics`（`python3 -m pip install -r requirements.txt`）。本文件夹已提供包含 `person` 类的 `models/yolo11n.pt` 预训练权重；节点只接受已有文件，不自动下载模型。接口使用 [Ultralytics 官方预测文档](https://docs.ultralytics.com/modes/predict/) 中的 `predict` 与 `boxes.xyxy`。

先构建并加载工作空间，再启动相机驱动。确认 RGB 和深度已经像素对齐、分辨率一致、采集时间同步，且消息都标记同一个 RGB 光学坐标系后，才允许设置 `aligned_depth_confirmed:=true`。不能只修改 frame_id 来绕过检查。

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch ic384_safety person_safety.launch.py \
  model_path:="$(pwd)/../models/yolo11n.pt" \
  aligned_depth_confirmed:=true \
  rgb_topic:=/camera/rgb/image_raw \
  depth_topic:=/camera/aligned_depth/image_raw \
  stop_distance:=1.0 release_distance:=1.5
```

上面的权重路径适用于本文件夹结构；按实机驱动替换话题名称。仿真还需要 `use_sim_time:=true` 和有效 `/clock`。16UC1 默认按毫米转换（`depth_scale:=0.001`），32FC1 必须已是米。参数在节点启动时读取，修改后重新启动。

此启动文件同时启动人员检测与安全监督节点，并设置 `require_person_clear:=true`、`require_hardware_feedback:=false`，默认只使用相机。**只运行这一套安全节点，不要同时启动原项目的安全节点**，避免多个发布者争用速度出口。

### 停车与恢复

相机断流、时间戳冻结/过期/不同步、对齐未确认、模型不可用、推理出错或结果过期、整图有效深度不足、任一人员框深度有效率不足 50% 或不足 9 个有效点时，人员输入为 false 或停止更新。安全监督节点在 false 到达时输出零速度；人员心跳失联超过默认 0.3 s 也输出零速度。默认启用 `auto_resume_person`，人员输入恢复为新鲜的 true 后自动解除人员阻塞，不需要为这一路调用复位。有效深度范围默认 0.6–8 m。只有新帧的新推理结果能发布 true，不用旧结果续心跳。

默认启动文件设置 `require_hardware_feedback:=false`，只要求 `/safety/input/person_clear`，无需另外发布 `estop_ok`、`obstacle_clear`、`arm_stowed`，也无需伪造这些反馈。人员检测不等于通用障碍物检测。以后若接入硬件，可在启动时设置 `require_hardware_feedback:=true`，届时三路真实反馈都必须持续存在。底盘驱动只能接收 `/cmd_vel_safe`，否则软件无法让机器人停下；当前改动只控制底盘，不停止机械臂。

人员距离不大于 1.0 m 时停车，退到不小于 1.5 m（或画面中不再检测到人员）且相机数据正常后，自动解除人员停车。必须收到解除阻塞之后的新速度指令才会行驶，停车前和停车期间的指令均丢弃。导航或遥控若持续发送速度，恢复条件满足后的下一条新指令即可驱动底盘，无需人工复位。

此次自动恢复只适用于人员输入。当前仅相机模式下，系统首次启动、无效速度及正常行驶时的速度指令超时仍保留人工复位。若以后启用硬件反馈模式，急停/障碍物/机械臂互锁故障同样保留人工复位。人员阻塞期间若发生这些故障，人员离开也不能解除它们。以下复位命令仅用于首次启动或这些故障恢复后，不能自动定时调用：

```bash
ros2 service call /safety/reset std_srvs/srv/Trigger '{}'
ros2 topic echo /safety/person_status
```

`/safety/person_status` 包含 clear、reason、nearest_optical_depth_m。相机/模型必须持续快于反馈超时，且推理结束时原图仍须在 0.3 s 内；速度不足时系统保持停车，不延长旧检测结果的有效期。

### 本地验证

```bash
python3 -m unittest discover -s ros2_ws/src/ic384_safety/test -v
```

测试不依赖 ROS2、相机或模型，使用合成深度和消息替身验证距离、滞回、异常、帧时效、人员输入超时、人员离开自动恢复、旧命令丢弃、仅相机模式和其他故障锁定。它们不验证真实检测召回率或制动效果。上机仍需在驱动断开时验证数据接口，再测实际停车延迟和距离；本功能是原型，不能替代独立硬件急停和驱动看门狗。

