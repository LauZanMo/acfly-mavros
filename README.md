# ACFLY MAVROS

> Author & Maintainer: LauZanMo
>
> 基于acfly-MAVLink的ROS扩展通信节点
>
> 开源不易，请动动小手，点个小小的star和fork吧！

## 依赖

- [ROS](https://www.ros.org/)
- (仿真可选)[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)

## 安装

请确保每条指令都执行成功：

```bash
sudo apt-get install python-catkin-tools python-rosinstall-generator -y
# 如果用的ROS版本是Noetic则使用
# sudo apt install python3-catkin-tools python3-rosinstall-generator python3-osrf-pycommon -y

# 需要替换你的ROS版本，且以下指令需要在同一个终端执行
source /opt/ros/${你的ROS版本}/setup.bash

# 因为acfly增加了自定义mavlink信息，若之前有通过二进制安装过mavros则需要卸载，没有则跳过
sudo apt remove ros-${ROS_DISTRO}-mavlink ros-${ROS_DISTRO}-mavros

# 构建ROS工作空间，可以自行修改路径
mkdir -p ~/acfly_ws/src && cd ~/acfly_ws
catkin init

# 下载mavlink和acfly-mavros
cd src
git clone -b release/${ROS_DISTRO}/mavlink/2022.1.5-1 https://gitee.com/LauZanMo/mavlink
git clone -b acfly-develop https://gitee.com/LauZanMo/acfly-mavros

# 安装依赖，如果rosdep update没执行则需要执行成功才能继续
cd .. && rosdep install --from-paths src --ignore-src -y

# 安装GeographicLib:
./src/acfly-mavros/mavros/scripts/install_geographiclib_datasets.sh

# 第一次编译请执行acfly提供的脚本
./src/acfly-mavros/update_custom_msg.sh
# 后续更改mavros源码只需要执行catkin build

# 每一次开启终端都需要设置环境变量
source devel/setup.bash
```

## 启动

提供官方的launch文件为acfly-mavros/mavros/launch/acfly.launch

启动前请连上飞控串口，将mavlink连接参数中的fcu_url参数修改成实际参数(串口名称，波特率)，再执行：

```bash
roslaunch mavros acfly.launch
```

配置插件文件为acfly-mavros/mavros/launch/acfly_pluginlist.yaml，其中白名单中的插件名为正在使用的插件(黑名单不用管)。

插件参数文件为acfly-mavros/mavros/launch/acfly_config.yaml，可根据自己需要进行修改，后续出参数讲解。

还可以通过udp形式连接acfly地面站(具体请参考mavros与QGC连接)

## 仿真

仿真需要在编译了PX4固件后才能使用。

提供官方的单，多个无人机仿真launch文件为

- acfly-mavros/test_mavros/launch/acfly/acfly_sitl.launch
- acfly-mavros/test_mavros/launch/acfly/multi_uav_acfly_sitl.launch

需要配置好参数才能执行。

## 贡献

你可以通过github的pull request功能向acfly-mavros分享好的想法与代码，在此之前，请在文件夹的根路径下执行：

```bash
find . -regex '.*\.\(cpp\|cu\|cc\|c\|hpp\|h\)' -exec clang-format-13 -style=file -i {} \;
```

以格式化你的代码，方便统一风格。(建议安装clang-format-13，该版本修复了指针对齐的bug)

