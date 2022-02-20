/**
 * @file acfly_position_sensor_base.h
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @brief acfly position sensor template
 * @version 1.0
 * @date 2022-02-18
 *
 * @copyright Copyright (c) 2022 acfly
 * For commercial use, please contact acfly: https://www.acfly.cn
 *
 */

#include <mavros/mavros_plugin.h>
#include <mavros/utils.h>

namespace mavros {
namespace plugin {
/**
 * @brief acfly位置传感器基类
 * @note 具体使用方法请参考acfly_slam_sensor插件
 */
class AcflyPositionSensorBase : public plugin::PluginBase {
public:
    AcflyPositionSensorBase() : PluginBase(), reset_counter(0) {}

    void register_position_sensor(ros::NodeHandle *nh) {
        // 传感器参数
        nh->param<std::string>("sensor/name", sensor_name, "ROS");
        nh->param("sensor/index", sensor_ind, 15);                  // 最高16路
        nh->param("sensor/type", sensor_type, 1);                   // 相对定位
        nh->param("sensor/data_frame", sensor_data_frame, 4);       // SLAM坐标系下的位置
        nh->param("sensor/data_type", sensor_data_type, 2);         // 三轴位置
        nh->param<float>("sensor/delay", sensor_delay, 0.05);       // 延时(s)
        nh->param<float>("sensor/trust_xy", sensor_trust_xy, 0.01); // xy方向方差(m^2)
        nh->param<float>("sensor/trust_z", sensor_trust_z, 0.01);   // z方向方差(m^2)

        mavlink::ACFly::msg::ACFly_RegeisterPosSensor rp{};
        m_uas->msg_set_target(rp);
        mavlink::set_string(rp.sensor_name, sensor_name);
        rp.ind       = static_cast<int8_t>(sensor_ind);
        rp.type      = static_cast<uint8_t>(sensor_type);
        rp.DataFrame = static_cast<uint8_t>(sensor_data_frame);
        rp.DataType  = static_cast<uint8_t>(sensor_data_type);
        rp.delay     = sensor_delay;
        rp.trustXY   = sensor_trust_xy;
        rp.trustZ    = sensor_trust_z;

        UAS_FCU(m_uas)->send_message_ignore_drop(rp);
    }

    void update_position_sensor(const ros::Time &stamp, Eigen::Vector3d pos, Eigen::Vector3d vel,
                                Eigen::Quaterniond att = Eigen::Quaterniond::Identity()) {
        // 未更新则退出
        if (last_transform_stamp == stamp)
            return;

        last_transform_stamp = stamp;

        // 消息标头和传感器基本信息
        mavlink::ACFly::msg::ACFly_UpdatePosSensor up{};
        m_uas->msg_set_target(up);
        up.ind      = static_cast<int8_t>(sensor_ind);
        up.DataType = static_cast<uint8_t>(sensor_data_type);

        // 定位信息
        up.posX       = pos[0];
        up.posY       = pos[1];
        up.posZ       = pos[2];
        up.velX       = vel[0];
        up.velY       = vel[1];
        up.velZ       = vel[2];
        up.AttQuat[0] = att.w();
        up.AttQuat[1] = att.x();
        up.AttQuat[2] = att.y();
        up.AttQuat[3] = att.z();

        // 传感器测量信息
        up.delay   = ros::Time::now().toSec() - stamp.toSec();
        up.trustXY = sensor_trust_xy;
        up.trustZ  = sensor_trust_z;
        up.reset   = reset_counter;

        UAS_FCU(m_uas)->send_message_ignore_drop(up);
    }

protected:
    std::string sensor_name;
    int         sensor_ind, sensor_type;
    int         sensor_data_frame, sensor_data_type;
    float       sensor_delay;
    float       sensor_trust_xy, sensor_trust_z;
    uint8_t     reset_counter;

private:
    ros::Time last_transform_stamp;
};
} // namespace plugin
} // namespace mavros