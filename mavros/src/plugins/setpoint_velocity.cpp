/**
 * @file setpoint_velocity.cpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @brief This file is from mavros open source respository, thanks for their contribution.
 * @version 1.0
 * @date 2022-01-27
 *
 * @copyright Copyright (c) 2022 acfly
 * @copyright Copyright 2014,2015,2016,2017 Vladimir Ermakov, Nuno Marques.
 * For commercial use, please contact acfly: https://www.acfly.cn
 *
 */

#include <eigen_conversions/eigen_msg.h>
#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <mavros_msgs/SetMavFrame.h>

namespace mavros {
namespace std_plugins {
using mavlink::common::MAV_FRAME;
/**
 * @brief Setpoint velocity plugin
 * @brief offboard模式的飞控控制ROS插件，接收ROS定义的速度信息
 * @warning 该插件依赖于系统兼容性支持
 */
class SetpointVelocityPlugin
    : public plugin::PluginBase,
      private plugin::SetPositionTargetLocalNEDMixin<SetpointVelocityPlugin> {
public:
    SetpointVelocityPlugin() : PluginBase(), sp_nh("~setpoint_velocity") {}

    void initialize(UAS &uas_) override {
        PluginBase::initialize(uas_);

        // cmd_vel usually is the topic used for velocity control in many controllers / planners
        // cmd_vel通常是许多控制器/规划器发送的速度控制话题
        vel_sub           = sp_nh.subscribe("cmd_vel", 10, &SetpointVelocityPlugin::vel_cb, this);
        vel_unstamped_sub = sp_nh.subscribe("cmd_vel_unstamped", 10,
                                            &SetpointVelocityPlugin::vel_unstamped_cb, this);
        mav_frame_srv =
            sp_nh.advertiseService("mav_frame", &SetpointVelocityPlugin::set_mav_frame_cb, this);

        // mav_frame
        // 发送信息的坐标系
        std::string mav_frame_str;
        if (!sp_nh.getParam("mav_frame", mav_frame_str)) {
            mav_frame = MAV_FRAME::LOCAL_NED;
        } else {
            mav_frame = utils::mav_frame_from_str(mav_frame_str);
        }

        enable_capabilities_cb();
    }

    Subscriptions get_subscriptions() override {
        return {/* 禁用接收 */};
    }

private:
    using lock_guard = std::lock_guard<std::recursive_mutex>;
    std::recursive_mutex mutex;

    friend class SetPositionTargetLocalNEDMixin;
    ros::NodeHandle sp_nh;

    ros::Subscriber    vel_sub;
    ros::Subscriber    vel_unstamped_sub;
    ros::ServiceServer mav_frame_srv;

    MAV_FRAME mav_frame;
    bool      set_position_local_support_confirmed;

    /* mid-level functions */
    /* 中间件函数 */

    // Acts when capabilities of the fcu are changed
    // 当飞控兼容性改变时执行该函数
    void capabilities_cb(UAS::MAV_CAP capabilities) override {
        lock_guard lock(mutex);

        if (m_uas->has_capability(UAS::MAV_CAP::SET_POSITION_TARGET_LOCAL_NED)) {
            set_position_local_support_confirmed = true;
        } else {
            set_position_local_support_confirmed = false;
        }
    }

    void send_setpoint_velocity(const ros::Time &stamp, Eigen::Vector3d &vel_req,
                                double yaw_rate_req) {
        using mavlink::common::POSITION_TARGET_TYPEMASK;

        uint16_t type_mask = uint16_t(POSITION_TARGET_TYPEMASK::X_IGNORE) |
                             uint16_t(POSITION_TARGET_TYPEMASK::Y_IGNORE) |
                             uint16_t(POSITION_TARGET_TYPEMASK::Z_IGNORE) |
                             uint16_t(POSITION_TARGET_TYPEMASK::AX_IGNORE) |
                             uint16_t(POSITION_TARGET_TYPEMASK::AY_IGNORE) |
                             uint16_t(POSITION_TARGET_TYPEMASK::AZ_IGNORE) |
                             uint16_t(POSITION_TARGET_TYPEMASK::YAW_IGNORE);

        auto vel = [&]() {
            if (mav_frame == MAV_FRAME::BODY_NED || mav_frame == MAV_FRAME::BODY_OFFSET_NED) {
                return ftf::transform_frame_baselink_aircraft(vel_req);
            } else {
                return ftf::transform_frame_enu_ned(vel_req);
            }
        }();

        auto yaw_rate = [&]() {
            if (mav_frame == MAV_FRAME::BODY_NED || mav_frame == MAV_FRAME::BODY_OFFSET_NED) {
                return ftf::transform_frame_baselink_aircraft(
                    Eigen::Vector3d(0.0, 0.0, yaw_rate_req));
            } else {
                return ftf::transform_frame_enu_ned(Eigen::Vector3d(0.0, 0.0, yaw_rate_req));
            }
        }();

        set_position_target_local_ned(stamp.toNSec() / 1000000, utils::enum_value(mav_frame),
                                      type_mask, Eigen::Vector3d::Zero(), vel,
                                      Eigen::Vector3d::Zero(), 0.0, yaw_rate.z());
    }

    /* ros callbacks */
    /* ROS回调函数 */

    void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &req) {
        if (set_position_local_support_confirmed) {
            Eigen::Vector3d vel_enu;

            tf::vectorMsgToEigen(req->twist.linear, vel_enu);
            send_setpoint_velocity(req->header.stamp, vel_enu, req->twist.angular.z);
        } else {
            ROS_WARN_NAMED("setpoint_raw",
                           "SPR: Operation error, please check capabilities of FCU!");
        }
    }

    void vel_unstamped_cb(const geometry_msgs::Twist::ConstPtr &req) {
        if (set_position_local_support_confirmed) {
            Eigen::Vector3d vel_enu;

            tf::vectorMsgToEigen(req->linear, vel_enu);
            send_setpoint_velocity(ros::Time::now(), vel_enu, req->angular.z);
        } else {
            ROS_WARN_NAMED("setpoint_raw",
                           "SPR: Operation error, please check capabilities of FCU!");
        }
    }

    bool set_mav_frame_cb(mavros_msgs::SetMavFrame::Request  &req,
                          mavros_msgs::SetMavFrame::Response &res) {
        mav_frame                       = static_cast<MAV_FRAME>(req.mav_frame);
        const std::string mav_frame_str = utils::to_string(mav_frame);
        sp_nh.setParam("mav_frame", mav_frame_str);
        res.success = true;
        return true;
    }
};
} // namespace std_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SetpointVelocityPlugin, mavros::plugin::PluginBase)
