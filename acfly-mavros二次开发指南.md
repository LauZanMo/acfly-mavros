# acfly-mavros二次开发指南

> Author & Maintainer 刘正武
>
> 开源不易，请动动小手，点个小小的star和fork吧！

## 介绍

acfly-mavros是基于开源的mavlink官方仓库mavros，修改以适配acfly系列飞控的开源代码仓库，感谢PX4开源社区的分享！

**注意：二次开发需要对C++11和ROS编程有足够的了解！**

## 总体框架剖析

> 由于我是做VO的，对mavlink底层实现不感兴趣，所以这里着重讲解acfly-mavros的C++实现，不做具体代码分析

acfly-mavros(以下简称mavros)文件夹下共有五个模块，他们的功能如下：

- **libmavconn**：mavlink通信的底层实现模块，该模块是通过boost库的socket提供串口(serial)，TCP和UDP的mavlink通信接口函数，实现与飞控(Flight Control Unit，**后面简称FCU**)的mavlink信息交互，同时能转发至地面站(Ground Control Station，**后面简称GCS**)
- **mavros**：mavros主模块，该模块启动了mavros_node的ROS节点，根据配置文件动态加载需要的ROS插件(包括额外插件)，并根据libmavconn提供的接口实现对ROS插件的对应的mavlink信息分发
- **mavros_extras**：mavros额外ROS插件模块，该模块主要提供了一些额外的ROS插件，在mavros_node中动态加载，二次开发建议直接在这里添加自定义的插件
- **mavros_msgs**：mavros的自定义ROS信息模块，由于mavlink与ROS原有的信息不能一一对应，因此mavros自定义了一些ROS话题和服务格式放在该模块
- **test_mavros**：mavros的测试文件模块，二次开发的一些测试文件可以放在这里

也就是说，<strong style="color:red;">我们只需要按照mavros指定的方式写ROS的插件，就能实现mavlink信息与ROS信息的转换！</strong>

## mavros插件框架

不懂ROS插件的同学请先看看[ROS官方介绍](http://wiki.ros.org/pluginlib)

所有mavros插件都继承于mavros自定义的pluginbase类，文件为[mavros_plugin.h](mavros/include/mavros/mavros_plugin.h)中，该类提供了所有mavros插件都需要的一些基础方法，同时留下了一些接口供我们重写(virtual开头的函数)，以下做详细讲解：

- 该类有一个m_uas的指针(unmanned aerial system 无人机系统)，会在ROS插件初始化(initialize函数)的时候被赋值(<strong style="color:red;">重写的时候千万不要忘</strong>)，用于存储FCU上传的一些基本信息(具体信息参考[mavros_uas.h](mavros/include/mavros/mavros_uas.h))，所有ROS插件都可以通过这个指针进行访问

- 该类get_subscriptions函数需要重写，如果你的ROS插件需要订阅从FCU或者GCS发过来的信息，需要做两步：

  - 定义回调函数handle_xxx(函数的输入参数需按格式)
  - get_subscriptions函数中调用make_handler函数

  具体实现可以参考[dummy.cpp](mavros/src/plugins/dummy.cpp)插件的写法，不需要订阅信息则留空get_subscriptions函数的return即可

- 该类connection_cb函数需要重写，当FCU或GCS连接上时会调用该函数，启用该函数需要在初始化(initialize函数)时调用enable_connection_cb函数进行使能
- 该类capabilities_cb需要重写，当FCU或GCS的兼容性(具体定义可查看[mavlink官方介绍](https://mavlink.io/zh/messages/common.html#MAV_PROTOCOL_CAPABILITY))发生改变时会调用该函数(刚连接上也算兼容性改变)，启用该函数需要在初始化(initialize函数)时调用enable_capabilities_cb函数进行使能

### 

#### param

**使用mavlink信息**：

- [PARAM_REQUEST_READ](https://mavlink.io/zh/messages/common.html#PARAM_REQUEST_READ)

- [PARAM_REQUEST_LIST](https://mavlink.io/zh/messages/common.html#PARAM_REQUEST_LIST)

- [PARAM_VALUE](https://mavlink.io/zh/messages/common.html#PARAM_VALUE)

- [PARAM_SET](https://mavlink.io/zh/messages/common.html#PARAM_SET)

**实现功能**：

- 以**服务形式**实现单个参数与参数列表的获取与设置，并对参数索引是否正确进行检测。(索引错误只会报警告，只要参数名正确不影响设置)

#### setpoint_velocity

**使用mavlink信息**：

- [SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/zh/messages/common.html#SET_POSITION_TARGET_LOCAL_NED)

**实现功能**：

- 以**话题形式**接收ROS标准的[twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)信息，以用于与许多开源的规划算法对接

**注意**：

- 有时间戳的和无时间戳的twist话题都能支持，请通过话题名来辨别
- 默认的坐标系为**FLU系**，如有ENU系下的速度控制需求，请通过插件命名空间下的mav_frame服务设置坐标系后，再发布话题

#### setpoint_position

**使用mavlink信息**：

- [SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/zh/messages/common.html#SET_POSITION_TARGET_LOCAL_NED)
- [SET_POSITION_TARGET_GLOBAL_INT](https://mavlink.io/zh/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT)

**实现功能**：

- 以**tf树形式**接收局部位置控制信息，同时以**话题形式**接收[twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)信息，实现位置单环或位置速度双环控制
- 以**话题形式**接收全球位置控制信息，实现全球位置单环控制

**注意**：

- 局部位置控制中**位置必须要有**，速度可以不给
- 局部位置控制开始之前需要通过插件命名空间下set_tf_listen服务**开启位置监听**，结束之后也需要通过该服务**关闭位置监听**
- 默认全球坐标系为**GLOBAL_INT系**，如有GLOBAL_RELATIVE_ALT_INT系或GLOBAL_TERRAIN_ALT_INT系下的位置控制需求，请通过插件命名空间下的global_mav_frame服务设置坐标系后，再发布话题

#### IMU

**使用mavlink信息**：

- [SCALED_IMU](https://mavlink.io/zh/messages/common.html#SCALED_IMU)
- [RAW_IMU](https://mavlink.io/zh/messages/common.html#RAW_IMU)(可弃用)
- [SCALED_PRESSURE](https://mavlink.io/zh/messages/common.html#SCALED_PRESSURE)

- [ATTITUDE](https://mavlink.io/zh/messages/common.html#ATTITUDE)

- [ATTITUDE_QUATERNION](https://mavlink.io/zh/messages/common.html#ATTITUDE_QUATERNION)

- [HIGHRES_IMU](https://mavlink.io/zh/messages/common.html#HIGHRES_IMU)

**实现功能**：

- 以**话题形式**实现上述mavlink信息的转发，具体信息内容参考上述链接

**注意**：

- 各ROS信息的坐标系在具体定义可查询[wiki.ros.org/sensor_msgs](http://wiki.ros.org/sensor_msgs)，在MAVROS启动已有对应的静态TF发布
- [RAW_IMU](https://mavlink.io/zh/messages/common.html#RAW_IMU)信息ACFLY未实现，且感觉没用，后续考虑删减
- [HIGHRES_IMU](https://mavlink.io/zh/messages/common.html#HIGHRES_IMU)信息如果后续有高分辨率惯导可以考虑添加
