# 人员检测模型权重

- 文件：`yolo11n.pt`
- 类型：Ultralytics YOLO11n，COCO 预训练目标检测模型，包含 `person` 类。
- 官方说明：https://docs.ultralytics.com/models/yolo11/
- 下载来源：https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt
- 获取日期：2026-11-10
- 许可：以 Ultralytics 官方 AGPL-3.0 / Enterprise 许可说明为准。
- `SHA256SUMS` 记录本次下载文件的 SHA-256，供复制后检查一致性；不是独立官方签名。

已检查归档完整性和 person 类别元数据；尚未加载模型进行推理。当前 Mac 缺少运行依赖，相机深度数据接入也尚未验证。权重本身不能替代驱动、ROS2 或推理环境。

在仓库根目录下进入工作空间后，启动参数使用：

```bash
cd ros2_ws
ros2 launch ic384_safety person_safety.launch.py \
  model_path:="$(pwd)/../models/yolo11n.pt" \
  aligned_depth_confirmed:=true
```

仅在实际确认彩色与深度图已经对齐后，才能设置上述对齐参数。完整构建和话题配置见上层 README.md。
