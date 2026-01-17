IMU基本使用
===
在基于stm32的MyCar导航机器人底盘中已经集成了imu传感器，并且启动底盘驱动节点时，会发布名为imu的话题。

## Papar Information
其使用流程如下：

- 启动机器人底盘；
- rviz2显示imu消息。

## Use
### 启动机器人底盘
```
ros2 launch ros2_stm32_bridge driver.launch
```
### rviz2显示imu消息
#### 提示
- 如果是远程rviz2显示imu消息,需要确保同时安装了imu-tools包。
```
默认情况下，rviz2没有显示imu消息的插件，需要自行安装相关插件，具体安装指令如下：

sudo apt install ros-${ROS_DISTRO}-imu-tools
```
```
安装完毕后，启动rviz2，并且将Fixed Frame设置为imu_link，再添加imu插件并将话题设置为imu，即可显示imu的姿态信息，该姿态会随着底盘的旋转而旋转。
```

