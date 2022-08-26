# acfly-mavros使用指南

>Author & Maintainer 刘正武
>
>开源不易，请点个star和fork吧！

## 介绍

acfly-mavros是基于开源的mavlink官方仓库mavros，修改以适配acfly系列FCU的开源代码仓库，感谢PX4开源社区的分享！



**mavros大部分与ROS交互的处理在插件中，我们主要讲解插件使用，便于理解。**

**相关文件**：

acfly.launch文件会加载两个关于插件的ROS参数文件，分别为：

- [acfly_pluginlist.yaml](mavros/launch/acfly_pluginlist.yaml)
- [acfly_config.yaml](mavros/launch/acfly_config.yaml)

其中acfly_pluginlist.yaml为启动时加载的插件列表，不需要动，<strong style="color:red;">acfly_config.yaml</strong>为插件需要读取的参数，需要<strong style="color:red;">重点关注</strong>，参数均为国际单位



**提示**：

1. 可以使用ROS提供的rqt工具包验证插件的正确性，验证工具如下：

- **topic monitor**：验证插件ROS话题的发布
- **topic publisher**：验证插件ROS话题的接收
- **service caller**：验证插件的服务
- **runtime viewer**：查看FCU状态的诊断信息

2. 除sys_status和sys_time插件之外，<strong style="color:red;">其他插件都有单独的命名空间</strong>，比如command插件的话题和服务都在/mavros/cmd命名空间下
3. 以下插件可查看mavlink信息来理解，<strong style="color:red;">单位全为国际单位，坐标系为ROS官方定义的坐标系(ENU-FLU)</strong>
4. 简称
   1. 飞控：FCU
   2. 板载电脑：OBC
   3. 地面站：GCS


## acfly-mavros官方插件使用

> 由于个人精力有限，没能将所有PX4官方的插件都兼容acfly，所以有些插件不一定能用，同时，我对PX4官方的一些插件做了优化，不一定能适用于PX4本身，请谅解！

### 快速导航

- 设置FCU飞行模式、接收和发送mavlink指令，请看[command](#command)插件
- 通过OBC发送slam类传感器信息，请看[acfly_slam_sensor](#acfly_slam_sensor)插件
- 之前使用PX4用户，需要快速通过指令控制FCU，请看[set_point_raw](#set_point_raw)插件
- 与ROS规划器或ego_planner对接，请看[set_point_velocity](#set_point_velocity)插件
- 无规划器，且需要飞相对或绝对位置的航点，请看[set_point_position](#set_point_position)插件
- 监听FCU传感器信息：
  - IMU信息：[IMU](#IMU)插件
  - 局部位置：[local_position](#local_position)插件
  - 全球位置：[global_position](#global_position)插件


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

- 可以通过以下话题接收FCU信息：

  - /mavros/state：FCU状态，包含传感器健康信息

  - /mavros/extended_state： FCU扩展状态

  - /mavros/battery: FCU检测电池状态


​	以上可以通过rqt的runtime viewer来监视，更加直观

- 可以通过

  - /mavros/statustext/send

  - /mavros/statustext/recv


​	向FCU发送与接收信息状态文本信息。

- 可以通过
  - /mavros/set_stream_rate 调整FCU发送消息的速率
  - /mavros/vehicle_info_get 获取载具信息
  - /mavros/set_message_interval 设置消息间隔

**注意：由于mavros不兼容中文，有时候出现一串红色的问号，通常是磁场受到干扰，FCU向OBC发送“检查航向”的中文所导致，重新标定磁力计即可**



### sys_time

**使用mavlink信息**：

- [TIMESYNC](https://mavlink.io/zh/messages/common.html#TIMESYNC)

- [SYSTEM_TIME](https://mavlink.io/zh/messages/common.html#SYSTEM_TIME)(FCU暂未实现)

**实现功能**：

- 检测FCU和OBC之间的延迟，即RTT(往返时间 round-trip time)，并补偿到FCU发送的传感器信息中(如IMU，GPS)

- 检测FCU端的系统时间(FCU暂未实现)

**常用功能**：

- 可以通过rqt的runtime viewer来监视RTT

**注意**：

- RTT跟连接方式关系较大，如果启动后一直输出"high RTT"，则可以稍微调大[配置文件](mavros\launch\acfly_config.yaml)的time/max_rtt_sample参数，单位为ms

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
- [MAV_CMD_DO_TRIGGER_CONTROL](https://mavlink.io/zh/messages/common.html#MAV_CMD_DO_TRIGGER_CONTROL)(由于OBC有更完善的视觉处理，后续考虑去掉)
- [MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL](https://mavlink.io/zh/messages/common.html#MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL)(由于OBC有更完善的视觉处理，后续考虑去掉)

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



### acfly_slam_sensor

**使用mavlink信息**：

- ACFly_UpdatePosSensor(**自定义消息**)

**实现功能**：

- 以**话题或tf形式**监听位置传感器信息并下发至FCU

**常用功能**：

该插件主要用于类SLAM的位置传感器，支持两种监听位置信息的方式：

1. [TF](http://wiki.ros.org/tf2) (**推荐**)
2. pose话题 (<strong style="color:red;">之前PX4的用户可优先考虑</strong>)

该插件的参数在同名命名空间下：

```yaml
# acfly_slam_sensor
acfly_slam_sensor:
  sensor_id: "camera"                       # 传感器TF名(插件将其与body_id之间建立静态tf)
  body_id: "base_link"                      # 飞控body系(通常不需要修改)
  sensor_body_rotation: [0.0, 0.0, 0.0]     # 传感器与body之间的旋转，以欧拉角rpy表示(ENU-FLU)
  sensor_body_translation: [0.0, 0.0, 0.0]  # 传感器与body之间的位移xyz(ENU-FLU)
  sensor:
    index: 15                               # 飞控注册定位传感器索引号(C9最高为16路)
    delay: 0.05                             # 默认传感器延时(s)
    trust_xy: 0.01                          # 传感器xy方向方差(m^2)
    trust_z: 0.01                           # 传感器z方向方差(m^2)
  tf:
    listen: false                           # 通过监听tf来获取位置信息(false则通过订阅来获取tf信息)
    frame_id: "map"                         # 传感器建图坐标系
    child_frame_id: "base_link"             # 飞控body系(通常不需要修改)
    rate_limit: 10.0                        # 监听频率
```

以下是tf监听流程：

如果需要使用**TF方式**监听位置信息，需要执行以下操作：

- 插件参数acfly_slam_sensor/tf/listen 改为 true

- 通过查询TF**正确填写**(插件会在启动时发布sensor到base_link的tf)

  - acfly_slam_sensor/sensor_id  
    即填入所使用的slam中发布的传感器tf

  - acfly_slam_sensor/tf/frame_id  
    填入所使用slam中的建图坐标系tf，可通过查询tf-tree获得（如vins-fusion的建图坐标系为world，则frame_id填入world）

  - acfly_slam_sensor/sensor_body_rotation和acfly_slam_sensor/sensor_body_translation


以T265为例：

1. 启动T265后，命令行输入“rqt”以打开rqt工具箱
2. rqt左上角找到Plugins -> Visualization -> TF Tree，单击打开，可以看到如下：<img src="images/T265%20TF%E6%A0%91.png" alt="T265 TF树" style="zoom:50%;" />
3. 可知传感器坐标系为camera_link，里程计/地图坐标系为camera_odom_frame(使用自己的SLAM系统需要保证这些坐标系都是右手系) **注意：realsense-ros不同版本tf名字可能不同**
4. 测量传感器 -> 飞控的平移和旋转(FLU系)，**以飞控在T265后0.1m，且相对T265在Z轴正向旋转了180度为例**。

最后可填写得：

```yaml
# acfly_slam_sensor
acfly_slam_sensor:
  sensor_id: "camera_link"
  body_id: "base_link"
  sensor_body_rotation: [0.0, 0.0, 3.1415926535]
  sensor_body_translation: [-0.1, 0.0, 0.0]
  tf:
    listen: true
    frame_id: "camera_odom_frame"
    child_frame_id: "base_link"
    rate_limit: 10.0
```

- 对于位置传感器，推荐监听频率为10-30hz，即acfly_slam_sensor/tf/rate_limit修改为10-30

- 其余可直接设为默认

以下是pose话题监听流程：

- 插件参数acfly_slam_sensor/tf/listen 改为 false
- 发布[mavros/acfly_slam_sensor/pose](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html)或[mavros/acfly_slam_sensor/pose_conv](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)话题，任意选一个即可，仅仅是有无协方差矩阵的区别
  - **注意**：发布的位置需为**里程计/地图 -> FCU的变换**，而不是里程计/地图 -> 传感器的变换

**注意**：

- acfly_slam_sensor/sensor下的参数请根据实际情况修改，如果索引需要修改，请确保与acfly本身的传感器索引没有冲突
- 如果使用了pose话题监听，则**setpoint_position**的**局部位置设置**(也是通过tf实现)不可用



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
- 以**话题形式**发布由FCU反馈回来的mavlink标准格式的ROS信息(与发送的一致)

**常用功能**：

目前FCU暂不支持角度控制，因为角度控制可能造成危险，只考虑在将来的穿越机固件上支持，因此启动时报的相关警告仅用于提醒用户

<strong style="color:red;">此处的代码逻辑与PX4相同，之前使用PX4的用户可以优先考虑用该插件进行控制!</strong>

此插件功能比较简单，以下是控制局部位置的步骤，其他类似：

1. 用户需要在cmake或python导入mavros_msgs功能包中的[PositionTarget信息](mavros_msgs/msg/PositionTarget.msg)
2. 参考对应的mavlink信息[SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/zh/messages/common.html#SET_POSITION_TARGET_LOCAL_NED)填入相应的值，通过/mavros/setpoint_raw/local话题发送

**注意**：

- 使用mavlink标准格式的ROS信息可以实现最全面的控制，但请务必将参数输入正确，mavros不会检查
- 控制形式是否支持请参看mavros启动后输出的信息(如SPR: Set position target local command is supported.就是支持[SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/zh/messages/common.html#SET_POSITION_TARGET_LOCAL_NED))
- FCU未实现反馈，所以不会有反馈控制信息
- 控制需要进入**OFFBOARD模式**并且已经解锁



### setpoint_velocity

**使用mavlink信息**：

- [SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/zh/messages/common.html#SET_POSITION_TARGET_LOCAL_NED)

**实现功能**：

- 以**话题形式**接收ROS标准的[twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)信息，以用于与许多开源的规划算法对接

**常用功能**：

以下以和ROS的路径规划器连接为例子，浙大的ego planner对接思路类似：

1. 通过/mavros/setpoint_velocity/mav_frame服务设置控制坐标系(默认为**FLU系**，使用ROS规划器不需要修改)，需要导入mavros_msgs的[SetMavFrame.srv](mavros_msgs/srv/SetMavFrame.srv)
2. 将ROS规划器的话题重映射(remap)到/mavros/setpoint_velocity/cmd_vel_unstamped话题上，该话题消息格式为[twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)
3. 启动规划器，设定目标

**注意**：

- 有时间戳的和无时间戳的twist话题都能支持：
  - /mavros/setpoint_velocity/cmd_vel为有时间戳twist话题
  - /mavros/setpoint_velocity/cmd_vel_unstamped为无时间戳twist话题
- 默认的坐标系为**FLU系**，如有ENU系下的速度控制需求，请通过插件命名空间下的mav_frame服务设置坐标系后，再发布话题
- 控制需要进入**OFFBOARD模式**并且已经解锁



### setpoint_position

**使用mavlink信息**：

- [SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/zh/messages/common.html#SET_POSITION_TARGET_LOCAL_NED)
- [SET_POSITION_TARGET_GLOBAL_INT](https://mavlink.io/zh/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT)

**实现功能**：

- 以**tf树形式**接收局部位置控制信息，同时以**话题形式**接收[twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)信息，实现位置单环或位置速度双环控制
- 以**话题形式**接收全球位置控制信息，实现全球位置单环控制

**常用功能**：

如果不需要使用ROS规划器，而只是进行简单的控制，可以使用该插件，对于需要做路径规划的用户，建议使用setpoint_position，以下是插件的**局部位置控制流程**：

1. 通过/mavros/setpoint_position/set_tf_listen服务设置打开tf监听(value设置为true)，需要导入mavros_msgs的[SetTFListen.srv](mavros_msgs/srv/SetTFListen.srv)
2. 相对于ac_local_enu发布target_position的tf变换，可以是间接的，举个例子：
   1. 假设slam地图的tf为map，slam传感器tf为sensor (如果使用了acfly_slam_sensor插件，系统会自动发布base_link(imu)与sensor的tf变换)
   2. 那么我们的tf树通常是这样的: map -> sensor -> base_link -> ac_local_enu
   3. enu系下的坐标不够直观，且对于使用ROS的用户，通常是给传感器所建地图的坐标，而不是enu系。为了更加直观，可以直接以map为原点，发布target_position的tf
3. 如果同时需要速度控制，可以通过/mavros/setpoint_position/set_twist发布[twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)信息，但请确保输入正确，系统不会检查，速度坐标系只能是**ENU系**的
4. 结束位置控制需要通过/mavros/setpoint_position/set_tf_listen服务设置关闭tf监听

以下是插件的**全球位置控制流程**：

1. 通过/mavros/setpoint_position/global_mav_frame服务设置控制坐标系(默认为**GLOBAL_INT系**)，需要导入mavros_msgs的[SetMavFrame.srv](mavros_msgs/srv/SetMavFrame.srv)
2. 对/mavros/setpoint_position/global发布全球位置话题

**注意**：

- 该插件需要有一定的ROS的tf知识，不建议给相对ac_local_enu移动的坐标系给发布target_position，这样会导致错误的控制结果

- 局部位置控制中**位置必须要有**，速度可以不给
- 局部位置控制开始之前需要通过插件命名空间下set_tf_listen服务**开启位置监听**，结束之后也需要通过该服务**关闭位置监听**
- 默认全球坐标系为**GLOBAL_INT系**，如有GLOBAL_RELATIVE_ALT_INT系或GLOBAL_TERRAIN_ALT_INT系下的位置控制需求，请通过插件命名空间下的global_mav_frame服务设置坐标系后，再发布话题
- 控制需要进入**OFFBOARD模式**并且已经解锁



### param

**使用mavlink信息**：

- [PARAM_REQUEST_READ](https://mavlink.io/zh/messages/common.html#PARAM_REQUEST_READ)

- [PARAM_REQUEST_LIST](https://mavlink.io/zh/messages/common.html#PARAM_REQUEST_LIST)

- [PARAM_VALUE](https://mavlink.io/zh/messages/common.html#PARAM_VALUE)

- [PARAM_SET](https://mavlink.io/zh/messages/common.html#PARAM_SET)

**实现功能**：

- 以**服务形式**实现单个参数与参数列表的获取与设置，并对参数索引是否正确进行检测。(索引错误只会报警告，只要参数名正确不影响设置)

**由于该插件功能都不怎么常用(可直接用地面站代替)，故不作具体讲解**



### IMU

**使用mavlink信息**：

- [SCALED_IMU](https://mavlink.io/zh/messages/common.html#SCALED_IMU)
- [RAW_IMU](https://mavlink.io/zh/messages/common.html#RAW_IMU)(可弃用)
- [SCALED_PRESSURE](https://mavlink.io/zh/messages/common.html#SCALED_PRESSURE)

- [ATTITUDE](https://mavlink.io/zh/messages/common.html#ATTITUDE)

- [ATTITUDE_QUATERNION](https://mavlink.io/zh/messages/common.html#ATTITUDE_QUATERNION)

- [HIGHRES_IMU](https://mavlink.io/zh/messages/common.html#HIGHRES_IMU)

**实现功能**：

- 以**话题形式**实现上述mavlink信息的转发，具体信息内容参考上述链接

**常用功能**：

可以通过以下话题监听FCU发送的几个主要信息(ROS标准格式)：

- /mavros/imu/data：ROS格式的IMU信息，与FCU发送的[SCALED_IMU](https://mavlink.io/zh/messages/common.html#SCALED_IMU)、[ATTITUDE](https://mavlink.io/zh/messages/common.html#ATTITUDE)或[ATTITUDE_QUATERNION](https://mavlink.io/zh/messages/common.html#ATTITUDE_QUATERNION)(角度有任意一个即可，优先四元数)有关
- /mavros/imu/static_pressure：绝对气压
- /mavros/imu/diff_pressure：相对气压(相对起飞点)

目前FCU固件(20220304)中[SCALED_IMU](https://mavlink.io/zh/messages/common.html#SCALED_IMU)和[ATTITUDE](https://mavlink.io/zh/messages/common.html#ATTITUDE)的发送频率分别为2Hz和15Hz，如果需要使用IMU信息，建议在Commulink.cpp的CommuPortRegister函数中修改这两个信息的发送频率为20Hz(或更高)

**注意**：

- 各ROS信息的坐标系在具体定义可查询[wiki.ros.org/sensor_msgs](http://wiki.ros.org/sensor_msgs)，在MAVROS启动已有对应的静态TF发布
- [RAW_IMU](https://mavlink.io/zh/messages/common.html#RAW_IMU)信息ACFLY未实现，且感觉没用，后续考虑删减
- [HIGHRES_IMU](https://mavlink.io/zh/messages/common.html#HIGHRES_IMU)信息如果后续有高分辨率惯导可以考虑添加
- 不修改固件的调节发送频率的方法：通过sys_status的/mavros/set_stream_rate修改对应信息的频率



### local_position

**使用mavlink信息**：

- [LOCAL_POSITION_NED](https://mavlink.io/zh/messages/common.html#LOCAL_POSITION_NED)
- [LOCAL_POSITION_NED_COV](https://mavlink.io/zh/messages/common.html#LOCAL_POSITION_NED_COV)(FCU未实现)

**实现功能**：

- 以**话题形式**实现上述mavlink信息的转发，具体信息内容参考上述链接
- 以**tf树形式**发布局部位置变换(base_link -> ac_local_enu)

**常用功能**：

可以通过以下话题监听FCU发送的信息(ROS标准格式)：

- /mavros/local_position/pose：局部位置(ENU)
- /mavros/local_position/velocity_local：局部速度(ENU)
- /mavros/local_position/velocity_body：机体速度(FLU)
- /mavros/local_position/odom：里程计数据

具体定义需要参考ROS标准格式(可在运行时通过命令行输入rostopic info xxx查询)

也可以通过tf树直接监听base_link与ac_local_enu的变换(与话题一个效果)

**注意**：

- 使用到的ROS标准格式信息链接：
  - [geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)
  - [geometry_msgs/TwistStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistStamped.html)
  - [nav_msgs/Odometry](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)

- 默认tf树开启(与global_position相斥)
- 不修改固件的调节发送频率的方法：通过sys_status的/mavros/set_stream_rate修改对应信息的频率



### global_position

**使用mavlink信息**：

- [GPS_RAW_INT](https://mavlink.io/zh/messages/common.html#GPS_RAW_INT)
- [GPS_GLOBAL_ORIGIN](https://mavlink.io/zh/messages/common.html#GPS_GLOBAL_ORIGIN)(FCU未实现)
- [GLOBAL_POSITION_INT](https://mavlink.io/zh/messages/common.html#GLOBAL_POSITION_INT)
- [LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET](https://mavlink.io/zh/messages/common.html#LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET)(FCU未实现)
- [HOME_POSITION](https://mavlink.io/zh/messages/common.html#HOME_POSITION)

**实现功能**：

- 以**话题形式**实现上述mavlink信息的转发，具体信息内容参考上述链接
- 以**tf树形式**发布全球转局部的位置变换(base_link -> ac_local_enu)

**常用功能**：

- 可以通过以下话题监听FCU发送的信息(ROS标准格式)：
  - /mavros/global_position/raw/fix：GPS原始观测位置
  - /mavros/global_position/raw/gps_vel：GPS原始观测速度
  - /mavros/global_position/raw/satellites：GPS原始观测卫星数
  - /mavros/global_position/global：FCU解算系统的全球位置
  - /mavros/global_position/local：FCU解算系统的全球位置换算成局部位置
  - /mavros/global_position/rel_alt：FCU解算系统的全球位置换算成相对高度
  - /mavros/global_position/compass_hdg：FCU解算系统的罗盘航向

也可以通过tf树直接监听base_link与ac_local_enu的变换(与local话题一个效果)

**注意**：

- 上述消息需要GPS搜到星方能使用

- 默认tf树关闭(与local_position相斥)
- 不修改固件的调节发送频率的方法：通过sys_status的/mavros/set_stream_rate修改对应信息的频率

