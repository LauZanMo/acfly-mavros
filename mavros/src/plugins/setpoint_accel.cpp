/**
 * @file setpoint_accel.cpp
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
#include <mavros/setpoint_mixin.h>
#include <mavros/utils.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <mavros_msgs/SetMavFrame.h>

namespace mavros {
namespace std_plugins {
using mavlink::common::MAV_FRAME;
/**
 * @brief Setpoint acceleration/force plugin
 * @brief offboard模式的飞控控制ROS插件，接收ROS定义的三轴向量信息
 * @warning 该插件依赖于系统兼容性支持
 */
class SetpointAccelerationPlugin
    : public plugin::PluginBase,
      private plugin::SetPositionTargetLocalNEDMixin<SetpointAccelerationPlugin> {
public:
    SetpointAccelerationPlugin() : PluginBase(), sp_nh("~setpoint_accel"), send_force(false) {}

    void initialize(UAS &uas_) override {
        PluginBase::initialize(uas_);

        sp_nh.param("send_force", send_force, false);

        accel_sub = sp_nh.subscribe("accel", 10, &SetpointAccelerationPlugin::accel_cb, this);
        accel_unstamped_sub = sp_nh.subscribe(
            "accel_unstamped", 10, &SetpointAccelerationPlugin::accel_unstamped_cb, this);
        mav_frame_srv = sp_nh.advertiseService("mav_frame",
                                               &SetpointAccelerationPlugin::set_mav_frame_cb, this);

        // mav_frame
        // 发送信息的坐标系
        // 仅LOCAL_NED，LOCAL_OFFSET_NED，BODY_NED，BODY_OFFSET_NED可用
        std::string mav_frame_str;
        if (!sp_nh.getParam("mav_frame", mav_frame_str)) {
            mav_frame = MAV_FRAME::BODY_NED;
        } else {
            mav_frame = utils::mav_frame_from_str(mav_frame_str);
            if (mav_frame != MAV_FRAME::LOCAL_NED && mav_frame != MAV_FRAME::LOCAL_OFFSET_NED &&
                mav_frame != MAV_FRAME::BODY_NED && mav_frame != MAV_FRAME::BODY_OFFSET_NED) {
                mav_frame = MAV_FRAME::BODY_NED;
                ROS_WARN_NAMED("setpoint_accel", "SPA: Invalid frame, set frame to BODY_NED.");
            }
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

    ros::Subscriber    accel_sub;
    ros::Subscriber    accel_unstamped_sub;
    ros::ServiceServer mav_frame_srv;

    bool      send_force;
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

    void send_setpoint_acceleration(const ros::Time &stamp, Eigen::Vector3d &accel_req) {
        using mavlink::common::POSITION_TARGET_TYPEMASK;

        uint16_t type_mask = uint16_t(POSITION_TARGET_TYPEMASK::X_IGNORE) |
                             uint16_t(POSITION_TARGET_TYPEMASK::Y_IGNORE) |
                             uint16_t(POSITION_TARGET_TYPEMASK::Z_IGNORE) |
                             uint16_t(POSITION_TARGET_TYPEMASK::VX_IGNORE) |
                             uint16_t(POSITION_TARGET_TYPEMASK::VY_IGNORE) |
                             uint16_t(POSITION_TARGET_TYPEMASK::VZ_IGNORE) |
                             uint16_t(POSITION_TARGET_TYPEMASK::YAW_IGNORE) |
                             uint16_t(POSITION_TARGET_TYPEMASK::YAW_RATE_IGNORE);

        if (send_force)
            type_mask = type_mask | uint16_t(POSITION_TARGET_TYPEMASK::FORCE_SET);

        auto accel = [&]() {
            if (mav_frame == MAV_FRAME::BODY_NED || mav_frame == MAV_FRAME::BODY_OFFSET_NED) {
                return ftf::transform_frame_baselink_aircraft(accel_req);
            } else {
                return ftf::transform_frame_enu_ned(accel_req);
            }
        }();

        set_position_target_local_ned(
            stamp.toNSec() / 1000000, utils::enum_value(MAV_FRAME::LOCAL_NED), type_mask,
            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), accel, 0.0, 0.0);
    }

    /* ros callbacks */
    /* ROS回调函数 */

    void accel_cb(const geometry_msgs::Vector3Stamped::ConstPtr &req) {
        if (set_position_local_support_confirmed) {
            Eigen::Vector3d accel_req;

            tf::vectorMsgToEigen(req->vector, accel_req);
            send_setpoint_acceleration(req->header.stamp, accel_req);
        } else {
            ROS_WARN_NAMED("setpoint_accel",
                           "SPA: Operation error, please check capabilities of FCU!");
        }
    }

    void accel_unstamped_cb(const geometry_msgs::Vector3::ConstPtr &req) {
        if (set_position_local_support_confirmed) {
            Eigen::Vector3d accel_req;

            tf::vectorMsgToEigen(*req, accel_req);
            send_setpoint_acceleration(ros::Time::now(), accel_req);
        } else {
            ROS_WARN_NAMED("setpoint_accel",
                           "SPA: Operation error, please check capabilities of FCU!");
        }
    }

    bool set_mav_frame_cb(mavros_msgs::SetMavFrame::Request  &req,
                          mavros_msgs::SetMavFrame::Response &res) {
        //检查设置坐标系格式
        if (static_cast<MAV_FRAME>(req.mav_frame) == MAV_FRAME::LOCAL_NED ||
            static_cast<MAV_FRAME>(req.mav_frame) == MAV_FRAME::LOCAL_OFFSET_NED ||
            static_cast<MAV_FRAME>(req.mav_frame) == MAV_FRAME::BODY_NED ||
            static_cast<MAV_FRAME>(req.mav_frame) == MAV_FRAME::BODY_OFFSET_NED) {
            mav_frame                       = static_cast<MAV_FRAME>(req.mav_frame);
            const std::string mav_frame_str = utils::to_string(mav_frame);
            sp_nh.setParam("mav_frame", mav_frame_str);
            res.success = true;
        } else {
            ROS_ERROR_NAMED("setpoint_accel", "SPA: Invalid frame.");
            res.success = false;
        }
        return true;
    }
};
} // namespace std_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SetpointAccelerationPlugin, mavros::plugin::PluginBase)
