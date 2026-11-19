# 上传到 GitHub

本目录可直接作为仓库根目录，建议仓库名使用 `ic384-safety`。

## 网页上传

1. 先解压 `ic384-safety-github.zip`，进入解压得到的 `ic384-safety` 文件夹。
2. 在 GitHub 新建仓库，进入文件上传页面。
3. 上传本文件夹内的文件和子文件夹，让 `README.md`、`models/`、`mac_camera/`、`ros2_ws/` 位于仓库根目录。`.gitignore` 是隐藏文件，也需要上传。
4. 提交上传内容。

压缩包用于传输；要在 GitHub 浏览源码，应上传解压后的内容。

## 使用 Git 上传

在 GitHub 建好一个空仓库后，从本目录运行以下命令。将地址中的 `YOUR_USERNAME` 换成你的 GitHub 用户名；仓库名不同也要一起替换。

```bash
git init
git add .
git commit -m "Initial import of IC384 safety module"
git branch -M main
git remote add origin https://github.com/YOUR_USERNAME/ic384-safety.git
git push -u origin main
```

## 包内内容

- `ros2_ws/src/ic384_safety/`：人员检测、距离判断、安全监督、启动配置和 41 项逻辑测试。
- `models/`：YOLO11n 权重、原有来源记录与模型校验值。
- `mac_camera/`：Mac 深度诊断源码及 6 项测试；相机工具包需另备，运行时用 `--kit` 指定位置。
- `requirements.txt`：人员检测的 Python 依赖；ROS2 依赖见 `package.xml`。
- `.gitignore`：忽略缓存、虚拟环境、ROS2 构建目录和 Mac 编译产物等。
- `SHA256SUMS`：包内文件校验清单，清单本身不参与计算。

本次打包保留原有程序源码，不包含原项目其他业务模块、Mac 驱动工具包及已编译采集程序。模型来源及原有许可说明见 `models/README.md`；原 ROS2 包的许可声明保留在 `package.xml` 和 `setup.py`。

测试命令、硬件准备、启动参数与当前验证范围见 [README](README.md)。
