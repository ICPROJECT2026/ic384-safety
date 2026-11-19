# Mac 相机接入与检查

这部分使用另行准备的 `AstraPro_Mac_Depth_Kit_v1.0` 工具包，让本机 Python 读取真实深度帧。相机工具包不包含在本仓库内。它是相机接入检查工具，尚未接通 YOLO 或机器人控制。

## 为什么需要单独接入

现有安全识别程序接收 ROS2 发布的彩色图和已对齐深度图。新驱动提供 OpenNI 接口及深度查看器，并不会自动发布这些 ROS2 图像。

驱动实查为 Intel x86_64，而本机 Python 为 Apple Silicon ARM64，不能直接在同一进程加载。因此用一个独立的 Intel 采集程序读取深度，再把原始毫米深度传给本机 Python。它不改动原驱动或查看器，也不安装系统驱动。

## 文件

| 文件 | 用途 |
|---|---|
| `depth_reader.cpp` | 用现有 OpenNI 接口读取真实深度，明确关闭镜像，输出带帧号和时间戳的数据 |
| `check_depth.py` | 编译采集程序、读取数据、检查帧时效和有效深度，显示距离诊断 |
| `test_depth_bridge.py` | 数据格式、截断、时间戳、无效深度及距离边界的 6 项测试 |
| `bin/astra_depth_reader` | 运行脚本时生成的 Intel 采集程序，不随本仓库分发 |

相机工具包的 `Driver`、`Source/Include` 目录须完整保留。本仓库独立使用时，按下面示例用 `--kit` 指向工具包。脚本默认路径仍兼容原 IC384 项目中位于本仓库同级的 `drivers/AstraPro_Mac_Depth_Kit_v1.0`。不会把 Intel 动态库加载到 ARM64 Python 中。

## 连接后的检查步骤

1. 连接 ASTRA PRO，退出 `Astra Depth Viewer.app` 以及其他占用相机的深度采集程序。
2. 在终端进入本仓库根目录，执行：

```bash
python3 mac_camera/check_depth.py \
  --kit "/absolute/path/to/AstraPro_Mac_Depth_Kit_v1.0" \
  --frames 10 --timeout 10
```

本检查仅依赖 Python 标准库、Apple 编译工具和已有驱动。运行 Intel 程序需要 Rosetta。不会自动安装这些组件。

成功后逐帧显示 JSON，其中 `nearest_depth_m` 是整张深度图中最近的有效距离，`depth_valid` 表示当前帧的时效与有效像素比例是否满足检查条件。默认只接受 0.6–8 m 的有效深度，至少 50% 像素有效，帧接收时年龄不大于 0.3 s。

距离标记沿用 1 m / 1.5 m：

- 不大于 1 m：`DEPTH_WITHIN_STOP_DISTANCE`。
- 不小于 1.5 m：`DEPTH_BEYOND_RELEASE_DISTANCE`。
- 二者之间：`DEPTH_BETWEEN_THRESHOLDS`。
- 过期、未来时间或有效深度不足：`UNKNOWN` 并报错退出。

**这些是整图深度诊断，不是人员距离或机器人运动许可。** 输出的 `motion_allowed` 固定为 false，`person_distance_verified` 固定为 false，也不会发布速度命令。前景家具等物体可能比人更近，不能把整图最近深度直接当成人员距离。

自定义工具包位置：

```bash
python3 mac_camera/check_depth.py \
  --kit "/absolute/path/to/AstraPro_Mac_Depth_Kit_v1.0" --frames 10
```

只编译并检查 Intel 程序可启动，不打开相机：

```bash
python3 mac_camera/check_depth.py \
  --kit "/absolute/path/to/AstraPro_Mac_Depth_Kit_v1.0" --build-only
```

## 还需要完成什么

要让 Mac 运行完整的人员识别演示，还需准备 YOLO 运行环境、接入这台相机的彩色流，并验证彩色图与深度图的标定、对齐、镜像方向和时间同步。当前查看器只验证深度流，不能据此认定 RGB 与深度已对齐。在这些条件满足前，原 ROS2 程序的 `aligned_depth_confirmed` 应保持 false。

## 本次验证记录

2026-11-10：本机编译成功，并通过 Rosetta 启动 Intel 采集程序的帮助入口；6 项接入测试和原有 41 项 ROS2 逻辑测试通过。尝试实机读取时未能打开设备，用户随后确认为相机暂未连接，因此尚未验证该新入口的真实帧接收、帧时效或 RGB 对齐。原查看器在此前项目记录中有深度读取成功记录，不代表新入口已经实测成功。

测试命令：

```bash
python3 -m unittest discover -s mac_camera -p 'test_*.py' -v
```
