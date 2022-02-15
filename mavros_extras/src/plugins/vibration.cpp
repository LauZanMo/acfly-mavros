/**
 * @file vibration.cpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @brief This file is from mavros open source respository, thanks for their contribution.
 * @version 1.0
 * @date 2022-02-05
 *
 * @copyright Copyright (c) 2022 acfly
 * @copyright Copyright 2014,2015,2016,2017 Vladimir Ermakov, Nuno Marques.
 * For commercial use, please contact acfly: https://www.acfly.cn
 *
 */

#include <eigen_conversions/eigen_msg.h>
#include <mavros/mavros_plugin.h>

#include <mavros_msgs/Vibration.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief Vibration plugin
 * @brief 描述无人机振动的ROS插件
 * @note This plugin is intended to publish MAV vibration levels and accelerometer clipping from
 * FCU.
 * @note 该插件用于发布从飞控发来的无人机振动等级和加速度计限幅
 * @warning 振动发布的坐标系为ENU系
 */
class VibrationPlugin : public plugin::PluginBase {
public:
    VibrationPlugin() : PluginBase(), v_nh("~vibration") {}

    void initialize(UAS &uas_) override {
        PluginBase::initialize(uas_);

        v_nh.param<std::string>("frame_id", frame_id, "base_link");

        vibration_pub = v_nh.advertise<mavros_msgs::Vibration>("raw/vibration", 10);
    }

    Subscriptions get_subscriptions() override {
        return {make_handler(&VibrationPlugin::handle_vibration)};
    }

private:
    ros::NodeHandle v_nh;

    std::string frame_id;

    ros::Publisher vibration_pub;

    /* message handlers */
    /* 信息回调句柄 */

    void handle_vibration(const mavlink::mavlink_message_t *msg,
                          mavlink::common::msg::VIBRATION  &vibration) {
        auto vibe_msg = boost::make_shared<mavros_msgs::Vibration>();

        vibe_msg->header = m_uas->synchronized_header(frame_id, vibration.time_usec);

        Eigen::Vector3d vib_enu = {vibration.vibration_x, vibration.vibration_y,
                                   vibration.vibration_z};
        tf::vectorEigenToMsg(ftf::transform_frame_ned_enu(vib_enu), vibe_msg->vibration);

        vibe_msg->clipping[0] = vibration.clipping_0;
        vibe_msg->clipping[1] = vibration.clipping_1;
        vibe_msg->clipping[2] = vibration.clipping_2;

        vibration_pub.publish(vibe_msg);
    }
};
} // namespace extra_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::VibrationPlugin, mavros::plugin::PluginBase)
