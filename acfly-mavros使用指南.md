# acfly-mavros使用指南

>Author & Maintainer 刘正武
>
>开源不易，请动动小手，点个小小的star和fork吧！

## 介绍

acfly-mavros是基于开源的mavlink官方仓库mavros，修改以适配acfly系列飞控的开源代码仓库，感谢PX4开源社区的分享！

mavros大部分与ROS交互的处理在插件中，我们主要讲解插件使用，便于理解。



提示：

1. 可以使用ROS提供的rqt工具包验证插件的正确性，验证工具如下：

- **topic monitor**：验证插件ROS话题的发布
- **topic publisher**：验证插件ROS话题的接收
- **service caller**：验证插件的服务
- **runtime viewer**：查看飞控状态的诊断信息

2. 除sys_status和sys_time插件之外，<strong style="color:red;">其他插件都有单独的命名空间</strong>，比如command插件的话题和服务都在/mavros/cmd命名空间下
3. 以下插件可查看mavlink信息来理解，<strong style="color:red;">单位全为国际单位，坐标系为ROS官方定义的坐标系(ENU-FLU)</strong>

## acfly-mavros官方插件使用

> 由于个人精力有限，没能将所有PX4官方的插件都兼容acfly，所以有些插件不一定能用，同时，我对PX4官方的一些插件做了优化，不一定能适用于PX4本身，请谅解！

### sys_status

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

**常用功能**：

可以通过以下话题接收FCU信息：

- /mavros/state：FCU状态，包含传感器健康信息
- /mavros/extended_state： FCU扩展状态
- /mavros/battery: FCU检测电池状态

以上可以通过rqt的runtime viewer来监视，更加直观

同时，可以通过

- /mavros/statustext/send
- /mavros/statustext/recv

向FCU发送与接收信息状态文本信息。

**注意：由于mavros不兼容中文，有时候出现一串红色的问号，通常是磁场受到干扰，FCU向OBC发送“检查航向”的中文所导致。**

### command

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
  - 设置模式(/mavros/cmd/set_mode)
  - 解锁(/mavros/cmd/arming)
  - 起降(全球坐标系)(/mavros/cmd/takeoff和/mavros/cmd/land)
  - 起降(局部坐标系)(/mavros/cmd/takeoff_local和/mavros/cmd/land_local)
  - 拍照与设置触发间隔

**常用功能**：

通常使用mavros的用户都是需要用OBC直接控制FCU的，即OFFBOARD模式(与PX4相同，可以直接套用)，以下为OFFBOARD控制流程：

1. 通过/mavros/cmd/set_mode服务设置FCU模式为OFFBOARD (custom_mode填入OFFBOARD即可，base_mode不用动)
1. 通过/mavros/cmd/arming服务解锁FCU (value需要填为True)
1. 通过setpoint_xxx插件控制无人机

上述操作如需通过软件控制请记得在cmake或者python中导入mavros_msgs的[CommandBool.srv](mavros_msgs/srv/CommandBool.srv)和[CommandSetMode.srv](mavros_msgs/srv/CommandSetMode.srv)服务

**注意**：

1. 进入OFFBOARD需要有XY位置传感器，如GPS、光流、SLAM传感器等
2. 原版mavros的**set_mode**是放在**sys_status**插件中的，我修改到了command插件中，使用的信息也不一样，但是PX4仿真可用
3. cmd插件下别的服务都是用于位置模式(POSITION CTL)，请不要与OFFBOARD混用，如需使用，请参考对应的mavlink信息
4. **takeoff local**和**land local**在FCU端暂未实现

### set_point_raw

**使用mavlink信息**：

- [SET_ATTITUDE_TARGET](https://mavlink.io/zh/messages/common.html#SET_ATTITUDE_TARGET)
- [ATTITUDE_TARGET](https://mavlink.io/zh/messages/common.html#ATTITUDE_TARGET)
- [SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/zh/messages/common.html#SET_POSITION_TARGET_LOCAL_NED)
- [POSITION_TARGET_LOCAL_NED](https://mavlink.io/zh/messages/common.html#POSITION_TARGET_LOCAL_NED)
- [SET_POSITION_TARGET_GLOBAL_INT](https://mavlink.io/zh/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT)
- [POSITION_TARGET_GLOBAL_INT](https://mavlink.io/zh/messages/common.html#POSITION_TARGET_GLOBAL_INT)

**实现功能**：

- 以**话题形式**接收mavlink标准格式的ROS信息(mavros_msgs中有定义，与mavlink格式一模一样)
- 以**话题形式**发布由飞控反馈回来的mavlink标准格式的ROS信息(与发送的一致)

**常用功能**：

目前FCU暂不支持角度控制，因为角度控制可能造成危险，只考虑在将来的穿越机固件上支持，因此启动时报的相关警告仅用于提醒用户

<strong style="color:red;">此处的代码逻辑与PX4相同，之前使用PX4的用户可以优先考虑用该插件进行控制!</strong>

此插件功能比较简单，以下是控制步骤：

1. 用户需要在cmake或python导入mavros_msgs功能包中的相应信息，举个例子，如果我需要控制局部位置，我需要导入mavros_msgs中的[PositionTarget信息](mavros_msgs/msg/PositionTarget.msg)
2. 参考对应的mavlink信息[SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/zh/messages/common.html#SET_POSITION_TARGET_LOCAL_NED)填入相应的值，通过话题发送

**注意**：

- 使用mavlink标准格式的ROS信息可以实现最全面的控制，但请务必将参数输入正确，mavros不会检查
- 控制形式是否支持请参看mavros启动后输出的信息(如SPR: Set position target local command is supported.就是支持[SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/zh/messages/common.html#SET_POSITION_TARGET_LOCAL_NED))
- FCU未实现反馈，所以不会有反馈控制信息