# usb_cam [![ROS 2 CI](https://github.com/ros-drivers/usb_cam/actions/workflows/build_test.yml/badge.svg)](https://github.com/ros-drivers/usb_cam/actions/workflows/build_test.yml)

## 针对 V4L USB 摄像头的 ROS 2 驱动
此包基于 V4L 设备，而非仅仅 UVC。

关于 ROS 1 的文档，请参见 [ROS wiki](http://ros.org/wiki/usb_cam)。

## 支持的 ROS 2 发行版与平台

所有官方支持的 Linux 发行版及对应的 ROS 2 版本均受支持。如果在这些平台上遇到问题，请创建 issue。

Windows：待定/未经测试/未验证  
MacOS：待定/未经测试/未验证

对于 MacOS 或 Windows —— 如果你愿意尝试并使其工作，请创建 issue 记录你的努力。如果成功我们会将其添加到此处的说明中！

## 快速开始

假设你已安装了受支持的 ROS 2 发行版，运行下面命令安装二进制发行包：

```shell
sudo apt-get install ros-<ros2-distro>-usb-cam
```

截至目前，此包应可在所有活跃的 ROS 2 发行版中通过二进制安装。

如果无法安装二进制包，请按照下面的源码编译说明。

## 从源码构建

将源码克隆/下载到你的工作区：

```shell
cd /path/to/colcon_ws/src
git clone https://github.com/ros-drivers/usb_cam.git
```

或者在仓库的 GitHub 页面上点击绿色的 “Download zip” 按钮。

下载完成并确保已 source 你的 ROS 2 下层后，安装依赖：

```shell
cd /path/to/colcon_ws
rosdep install --from-paths src --ignore-src -y
```

接着你应该具备了编译 `usb_cam` 包的所有必要依赖：

```shell
cd /path/to/colcon_ws
colcon build
source /path/to/colcon_ws/install/setup.bash
```

请在成功构建后 source 新构建的包。

一旦 source 完成，你应该可以通过下述三种方式之一运行该包。

## 运行

`usb_cam_node` 可以使用默认设置运行，也可以通过命令行或参数文件设置特定参数。

我们在 `usb_cam/config/params.yaml` 中提供了一个“默认”参数文件供入门使用。你可以根据需要修改此文件。

同时提供了一个 launch 文件，可启动 `usb_cam_node_exe` 可执行文件和一个显示图像话题的附加节点。

下面展示了以三种不同方式启动节点的命令：

**注意：以下命令只需运行其中一个即可启动节点**

```shell
# run the executable with default settings (without params file)
ros2 run usb_cam usb_cam_node_exe

# run the executable while passing in parameters via a yaml file
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file /path/to/colcon_ws/src/usb_cam/config/params.yaml
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file `ros2 pkg prefix usb_cam`/share/usb_cam/config/params_2.yaml

# launch the usb_cam executable that loads parameters from the same `usb_cam/config/params.yaml` file as above
# along with an additional image viewer node
ros2 launch usb_cam camera.launch.py
```

## 启动多个 usb_cam

要同时启动多个节点，只需为每个节点重映射命名空间：

```shell
ros2 run usb_cam usb_cam_node_exe --ros-args --remap __ns:=/usb_cam_0 --params-file /path/to/usb_cam/config/params_0.yaml
ros2 run usb_cam usb_cam_node_exe --ros-args --remap __ns:=/usb_cam_1 --params-file /path/to/usb_cam/config/params_1.yaml
```

## 支持的格式

### 设备支持的格式

要查看连接设备支持的格式，运行 `usb_cam_node` 并观察控制台输出。

示例输出：

```log
This devices supproted formats:
    Motion-JPEG: 1280 x 720 (30 Hz)
    Motion-JPEG: 960 x 540 (30 Hz)
    Motion-JPEG: 848 x 480 (30 Hz)
    Motion-JPEG: 640 x 480 (30 Hz)
    Motion-JPEG: 640 x 360 (30 Hz)
    YUYV 4:2:2: 640 x 480 (30 Hz)
    YUYV 4:2:2: 1280 x 720 (10 Hz)
    YUYV 4:2:2: 640 x 360 (30 Hz)
    YUYV 4:2:2: 424 x 240 (30 Hz)
    YUYV 4:2:2: 320 x 240 (30 Hz)
    YUYV 4:2:2: 320 x 180 (30 Hz)
    YUYV 4:2:2: 160 x 120 (30 Hz)
```

### 驱动支持的格式

驱动自身也有支持的格式。详见 [源代码](include/usb_cam/formats/)。

在查看了[设备支持的格式](#设备支持的格式)之后，通过参数文件（`config/params.yaml`）中的 `pixel_format` 参数指定要使用的格式。

要查看当前所有受支持的驱动格式列表，运行：

```shell
ros2 run usb_cam usb_cam_node_exe --ros-args -p pixel_format:="test"
```

注意："test" 可替换为任何不受支持的像素格式字符串。驱动会检测给定的像素格式是否被支持。

欢迎贡献以添加更多格式和转换！

### 支持的 IO 方法

截至目前，此驱动支持三种不同的 IO 方法：

1. `read`：在用户空间和内核空间之间复制视频帧
2. `mmap`：在内核空间分配的内存映射缓冲区
3. `userptr`：在用户空间分配的内存缓冲区

要了解不同方法的更多信息，请查看这篇提供良好概述的文章：
https://lwn.net/Articles/240667/

## 压缩

非常感谢 [ros2_v4l2_camera 包](https://gitlab.com/boldhearts/ros2_v4l2_camera#usage-1) 在该主题上的文档。

`usb_cam` 应默认支持压缩，因为它使用 `image_transport` 发布图像，只要你的系统安装了 `image_transport_plugins` 包。有了插件，`usb_cam` 包应会自动发布 `compressed` 话题。

不幸的是 `rviz2` 和 `show_image.py` 目前还不支持可视化压缩图像，因此你需要在下游重新发布压缩图像以解压它：

```shell
ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=image_raw/compressed --remap out:=image_raw/uncompressed
```

## 测试

运行此仓库的基本单元测试：

```shell
colcon build --packages-select usb_cam
colcon test --pacakges-select usb_cam
```

### 集成测试

运行此仓库的集成测试：

```shell
colcon build --packages-select usb_cam --cmake-args -DINTEGRATION_TESTS=1
colcon test --pacakges-select usb_cam
```

### 地址和泄漏检测（sanitizing）

在 `CMakeLists.txt` 中集成了用于内存泄漏和地址检测的编译标志，以帮助检测问题。

要启用它们，请传入 `SANITIZE=1` 标志：

```shell
colcon build --packages-select usb_cam --cmake-args -DSANITIZE=1
```

构建完成后，直接运行节点可执行文件并传入任何所需的 `ASAN_OPTIONS`：

```shell
ASAN_OPTIONS=new_delete_type_mismatch=0 ./install/usb_cam/lib/usb_cam/usb_cam_node_exe 
```

在使用 Ctrl+C 关闭可执行文件后，sanitizer 会报告任何内存泄漏。

默认情况下该功能是关闭的，因为开启 sanitizer 会导致体积增大并降低性能。

## 文档

[Doxygen](http://docs.ros.org/indigo/api/usb_cam/html/) 文件可在 ROS wiki 上找到。

### 许可

usb_cam 以 BSD 许可证发布。完整条款请参见 [LICENSE](LICENSE) 文件。

### 作者

完整贡献者名单见 [AUTHORS](AUTHORS.md) 文件。
