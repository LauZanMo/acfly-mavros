/**
 * @file acfly_position_sensor_mixin.h
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @author Vladimir Ermakov <vooon341@gmail.com>
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
 * @brief acfly位置传感器模板类
 * @note 具体使用方法请参考acfly_slam_sensor插件
 */
template <class D> class AcflyPositionSensorMixin {
public:
    void register_position_sensor() {
        mavros::UAS *m_uas_ = static_cast<D *>(this)->m_uas;

        mavlink::ACFly::msg::ACFly_RegeisterPosSensor rp{};
        m_uas_->msg_set_target(rp);
        mavlink::set_string(rp.sensor_name, static_cast<D *>(this)->sensor_name);
        rp.ind       = static_cast<int8_t>(static_cast<D *>(this)->sensor_ind);
        rp.type      = static_cast<uint8_t>(static_cast<D *>(this)->sensor_type);
        rp.DataFrame = static_cast<uint8_t>(static_cast<D *>(this)->sensor_data_frame);
        rp.DataType  = static_cast<uint8_t>(static_cast<D *>(this)->sensor_data_type);
        rp.delay     = static_cast<D *>(this)->sensor_delay;
        rp.trustXY   = static_cast<D *>(this)->sensor_trust_xy;
        rp.trustZ    = static_cast<D *>(this)->sensor_trust_z;

        UAS_FCU(m_uas_)->send_message_ignore_drop(rp);
    }

    void update_position_sensor(const ros::Time &stamp, Eigen::Vector3d pos, Eigen::Vector3d vel,
                                Eigen::Quaterniond att = Eigen::Quaterniond::Identity()) {
        // 未更新则退出
        if (last_transform_stamp == stamp)
            return;

        last_transform_stamp = stamp;
        mavros::UAS *m_uas_  = static_cast<D *>(this)->m_uas;

        // 消息标头和传感器基本信息
        mavlink::ACFly::msg::ACFly_UpdatePosSensor up{};
        m_uas_->msg_set_target(up);
        up.ind      = static_cast<int8_t>(static_cast<D *>(this)->sensor_ind);
        up.DataType = static_cast<uint8_t>(static_cast<D *>(this)->sensor_data_type);

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
        up.trustXY = static_cast<D *>(this)->sensor_trust_xy;
        up.trustZ  = static_cast<D *>(this)->sensor_trust_z;
        up.reset   = static_cast<D *>(this)->reset_counter;

        UAS_FCU(m_uas_)->send_message_ignore_drop(up);
    }

private:
    ros::Time last_transform_stamp;
};
} // namespace plugin
} // namespace mavros