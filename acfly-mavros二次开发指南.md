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

## acfly-mavros插件的使用

> 这里先介绍mavros的插件的框架，再具体介绍我修改的一些插件的使用

### mavros插件框架

不懂ROS插件的同学请先看看[ROS官方介绍](http://wiki.ros.org/pluginlib)

所有mavros插件都继承于mavros自定义的pluginbase类，文件为[mavros_plugin.h](mavros/include/mavros/mavros_plugin.h)中，该类提供了所有mavros插件都需要的一些基础方法，同时留下了一些接口供我们重写(virtual开头的函数)，以下做详细讲解：

- 该类有一个m_uas的指针(unmanned aerial system 无人机系统)，会在ROS插件初始化(initialize函数)的时候被赋值(<strong style="color:red;">重写的时候千万不要忘</strong>)，用于存储FCU上传的一些基本信息(具体信息参考[mavros_uas.h](mavros/include/mavros/mavros_uas.h))，所有ROS插件都可以通过这个指针进行访问

- 该类get_subscriptions函数需要重写，如果你的ROS插件需要订阅从FCU或者GCS发过来的信息，需要做两步：

  - 定义回调函数handle_xxx(函数的输入参数需按格式)
  - get_subscriptions函数中调用make_handler函数

  具体实现可以参考[dummy.cpp](mavros/src/plugins/dummy.cpp)插件的写法，不需要订阅信息则留空get_subscriptions函数的return即可

- 该类connection_cb函数需要重写，当FCU或GCS连接上时会调用该函数，启用该函数需要在初始化(initialize函数)时调用enable_connection_cb函数进行使能
- 该类capabilities_cb需要重写，当FCU或GCS的兼容性(具体定义可查看[mavlink官方介绍](https://mavlink.io/zh/messages/common.html#MAV_PROTOCOL_CAPABILITY))发生改变时会调用该函数(刚连接上也算兼容性改变)，启用该函数需要在初始化(initialize函数)时调用enable_capabilities_cb函数进行使能

### acfly-mavros官方插件

> 由于个人精力有限，没能将所有PX4官方的插件都兼容acfly，所以有些插件不一定能用，同时，我对PX4官方的一些插件做了优化，不一定能适用于PX4本身，请谅解！

提示：可以使用ROS提供的rqt工具包验证插件的正确性，验证工具如下：

- **topic monitor**：验证插件ROS话题的发布
- **topic publisher**：验证插件ROS话题的接收
- **service caller**：验证插件的服务
- **runtime viewer**：查看飞控状态的诊断信息

以下插件可查看mavlink信息来理解，<strong style="color:red;">单位全为国际单位，坐标系为ROS官方定义的坐标系(ENU-FLU)</strong>

#### sys_status

**使用mavlink信息**：

- [HEARTBEAT](https://mavlink.io/zh/messages/common.html#HEARTBEAT)
- [SYS_STATUS](https://mavlink.io/zh/messages/common.html#SYS_STATUS)

- [EXTENDED_SYS_STATE](https://mavlink.io/zh/messages/common.html#EXTENDED_SYS_STATE)

- [STATUSTEXT](https://mavlink.io/zh/messages/common.html#STATUSTEXT)

- [AUTOPILOT_VERSION](https://mavlink.io/zh/messages/common.html#AUTOPILOT_VERSION)

- [BATTERY_STATUS](https://mavlink.io/zh/messages/common.html#BATTERY_STATUS)

**实现功能**：

- 以**话题形式**实现上述mavlink信息的转发，具体信息内容参考上述链接
- 以**话题形式**接收[STATUSTEXT](https://mavlink.io/zh/messages/common.html#STATUSTEXT)信息并以mavlink格式转发
- 以**服务形式**实现信息速率设置与载具信息的获取
- 对mavlink心跳、系统状态、电池状态进行可视化诊断

#### command

**使用mavlink信息**：

- [COMMAND_LONG](https://mavlink.io/zh/messages/common.html#COMMAND_LONG)

- [COMMAND_INT](https://mavlink.io/zh/messages/common.html#COMMAND_INT)

- [COMMAND_ACK](https://mavlink.io/zh/messages/common.html#COMMAND_ACK)

**使用mavlink指令**：

- [MAV_CMD_DO_SET_MODE](https://mavlink.io/zh/messages/common.html#MAV_CMD_DO_SET_MODE)
- [MAV_CMD_COMPONENT_ARM_DISARM](https://mavlink.io/zh/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM)
- [MAV_CMD_DO_SET_HOME](https://mavlink.io/zh/messages/common.html#MAV_CMD_DO_SET_HOME)
- [MAV_CMD_NAV_TAKEOFF](https://mavlink.io/zh/messages/common.html#MAV_CMD_NAV_TAKEOFF)
- [MAV_CMD_NAV_LAND](https://mavlink.io/zh/messages/common.html#MAV_CMD_NAV_LAND)
- [MAV_CMD_DO_TRIGGER_CONTROL](https://mavlink.io/zh/messages/common.html#MAV_CMD_DO_TRIGGER_CONTROL)(由于机载电脑有更完善的视觉处理，后续考虑去掉)
- [MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL](https://mavlink.io/zh/messages/common.html#MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL)(由于机载电脑有更完善的视觉处理，后续考虑去掉)

**实现功能**：

- 以**服务形式**实现短指令和长指令信息的下发与回应检测。
- 以**服务形式**实现控制ACFLY主控完成以下动作：
  - 设置模式(/mavros/cmd/do_set_mode)
  - 解锁(/mavros/cmd/arming)
  - 起降(全球坐标系)(/mavros/cmd/takeoff和/mavros/cmd/land)
  - 起降(局部坐标系)(/mavros/cmd/takeoff_local和/mavros/cmd/land_local)
  - 拍照与设置触发间隔

**注意**：原版mavros的**set_mode**是放在**sys_status**插件中的，我修改到了command插件中，使用的信息也不一样，但是PX4仿真可用

#### param

**使用mavlink信息**：

- [PARAM_REQUEST_READ](https://mavlink.io/zh/messages/common.html#PARAM_REQUEST_READ)

- [PARAM_REQUEST_LIST](https://mavlink.io/zh/messages/common.html#PARAM_REQUEST_LIST)

- [PARAM_VALUE](https://mavlink.io/zh/messages/common.html#PARAM_VALUE)

- [PARAM_SET](https://mavlink.io/zh/messages/common.html#PARAM_SET)

**实现功能**：

- 以**服务形式**实现单个参数与参数列表的获取与设置，并对参数索引是否正确进行检测。(索引错误只会报警告，只要参数名正确不影响设置)

#### set_point_raw

**使用mavlink信息**：

- [SET_ATTITUDE_TARGET](https://mavlink.io/zh/messages/common.html#SET_ATTITUDE_TARGET)
- [ATTITUDE_TARGET](https://mavlink.io/zh/messages/common.html#ATTITUDE_TARGET)
- [SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/zh/messages/common.html#SET_POSITION_TARGET_LOCAL_NED)
- [POSITION_TARGET_LOCAL_NED](https://mavlink.io/zh/messages/common.html#POSITION_TARGET_LOCAL_NED)
- [SET_POSITION_TARGET_GLOBAL_INT](https://mavlink.io/zh/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT)
- [POSITION_TARGET_GLOBAL_INT](https://mavlink.io/zh/messages/common.html#POSITION_TARGET_GLOBAL_INT)

**实现功能**

- 以**话题形式**接收mavlink标准格式的ROS信息(mavros_msgs中有定义，与mavlink格式一模一样)

**注意**：

- 使用mavlink标准格式的ROS信息可以实现最全面的控制，但请务必将参数输入正确，mavros不会检查
- set attitude后续只在穿越机中支持，控制形式是否支持请参看mavros启动后输出的信息(如SPR: Set position target local command is supported.就是支持[SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/zh/messages/common.html#SET_POSITION_TARGET_LOCAL_NED))

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
