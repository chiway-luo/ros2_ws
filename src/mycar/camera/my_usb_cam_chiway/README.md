深度相机安装教程
===


## Install & Dependence
- （1）.驱动准备
```
在课程配套资料中，复制文件夹ros2_astra_camera到你工作空间下的src目录。
git clone https://github.com/orbbec/ros2_astra_camera.git
```
- （2）.安装依赖
```
请调用如下指令安装相机驱动所依赖的功能包。

sudo apt install libgflags-dev nlohmann-json3-dev ros-humble-image-transport ros-humble-image-publisher
```
```
所安装的各功能包作用如下：
libgflags-dev：一个命令行参数处理库，用于处理命令行参数的解析和管理。
nlohmann-json3-dev：这是一个 JSON（JavaScript Object Notation）处理库，用于解析和生成 JSON 数据。
ros-humble-image-transport：是 ROS 图像传输库，用于在 ROS 中传输图像数据。它提供了一些常见的图像传输方法和功能。
ros-humble-image-publisher：是 ROS 图像发布者包，用于在 ROS 中发布图像数据。它可以将图像数据发布到 ROS 系统中的其他节点，以供其他节点订阅和处理。
```
- （3）.安装glog
```
glog 是 Google 的一个 C++ 日志库，旨在提供高效、易用的日志记录功能。可将该功能包安装在当前工作空间的src目录下，安装相关指令如下：
```
```
wget -c https://github.com/google/glog/archive/refs/tags/v0.6.0.tar.gz  -O glog-0.6.0.tar.gz
tar -xzvf glog-0.6.0.tar.gz
cd glog-0.6.0
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig
```

- （4）.安装magic_enum
```
magic_enum 是一个 C++ 的库，提供了对枚举类型的强大而方便的操作和反射支持。可将该功能包安装在当前工作空间的src目录下，安装相关指令如下：
```

```
wget -c https://github.com/Neargye/magic_enum/archive/refs/tags/v0.8.0.tar.gz -O  magic_enum-0.8.0.tar.gz
tar -xzvf magic_enum-0.8.0.tar.gz
cd magic_enum-0.8.0
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig
```
- （5）.安装libuvc
```
libuvc 是一个开源的跨平台的 USB 视频类库，用于在应用程序中与 USB 视频设备进行交互和视频数据的采集。可将该功能包安装在当前工作空间的src目录下，安装相关指令如下：
```
```
git clone https://github.com/libuvc/libuvc.git
cd libuvc
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig
```




## Dataset Preparation
| Dataset | Download |
| ---     | ---   |
| dataset-A | [download]() |
| dataset-B | [download]() |
| dataset-C | [download]() |

## Use
- for train
  ```
  python train.py
  ```
- for test
  ```
  python test.py
  ```
