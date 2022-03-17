# acfly插件参数讲解

> Author & Maintainer: 刘正武

## 相关文件

acfly.launch文件会加载两个关于插件的ROS参数文件，路径分别为：

- acfly-mavros/mavros/launch/acfly_pluginlist.yaml
- acfly-mavros/mavros/launch/acfly_config.yaml

其中acfly_pluginlist.yaml为启动时加载的插件列表，不需要动，<strong style="color:red;">acfly_config.yaml</strong>为插件需要读取的参数，需要<strong style="color:red;">重点关注</strong>，下面讲解各插件所需参数。

## acfly_slam_sensor

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
    frame_id: "map"                         # 传感器建图坐标系(通常为map)
    child_frame_id: "base_link"             # 飞控body系(通常不需要修改)
    rate_limit: 10.0                        # 监听频率
```

该插件主要用于类SLAM的位置传感器，支持两种监听位置信息的方式：

1. [TF树](http://wiki.ros.org/tf2)
2. pose话题

如果需要使用TF树方式监听位置信息，需要执行以下操作：

- acfly_slam_sensor/tf/listen 改为 true

- 通过查询TF树正确填写

  - acfly_slam_sensor/sensor_id
  - acfly_slam_sensor/tf/frame_id
  - acfly_slam_sensor/sensor_body_rotation和acfly_slam_sensor/sensor_body_translation

  以T265为例：

  1. 启动T265后，命令行输入“rqt”以打开rqt工具箱
  2. rqt左上角找到Plugins->Visualization->TF Tree，单击打开，可以看到如下：<img src="images/T265%20TF%E6%A0%91.png" alt="T265 TF树" style="zoom:50%;" />
  3. 可知传感器的相机坐标系为camera_link，里程计坐标系为camera_odom_frame(使用自己的SLAM系统需要保证这些坐标系都是FLU系)
  4. 测量传感器->飞控的平移和旋转(FLU系)，**以飞控在T265后0.1m，且相对T265在Z轴正向旋转了180度为例**。

  因此可以填写
  
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

- 对于位置传感器，推荐监听频率为10~30hz，即acfly_slam_sensor/tf/rate_limit修改为10~30
- 其余可直接设为默认，acfly_slam_sensor/sensor/index如果需要修改，请确保与acfly本身的传感器索引没有冲突