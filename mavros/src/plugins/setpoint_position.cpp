/**
 * @file setpoint_position.cpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @brief This file is from mavros open source respository, thanks for their contribution.
 * @version 1.0
 * @date 2022-02-16
 *
 * @copyright Copyright (c) 2022 acfly
 * @copyright Copyright 2014,2015,2016,2017 Vladimir Ermakov.
 * For commercial use, please contact acfly: https://www.acfly.cn
 *
 */

#include <eigen_conversions/eigen_msg.h>
#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>

#include <geometry_msgs/PoseStamped.h>

#include <geographic_msgs/GeoPoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/SetMavFrame.h>
#include <mavros_msgs/SetTFListen.h>

#include <GeographicLib/Geocentric.hpp>

namespace mavros {
namespace std_plugins {
using mavlink::common::MAV_FRAME;
/**
 * @brief Setpoint position plugin
 * @brief offboard模式的飞控控制ROS插件，接收ROS定义的位置信息
 * @warning 该插件依赖于系统兼容性支持
 */
class SetpointPositionPlugin
    : public plugin::PluginBase,
      private plugin::SetPositionTargetLocalNEDMixin<SetpointPositionPlugin>,
      private plugin::SetPositionTargetGlobalIntMixin<SetpointPositionPlugin>,
      private plugin::TF2ListenerMixin<SetpointPositionPlugin> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SetpointPositionPlugin()
        : PluginBase(), sp_nh("~setpoint_position"), tf_listen(false), tf_rate(10.0), set_twist() {}

    void initialize(UAS &uas_) override {
        PluginBase::initialize(uas_);

        // tf params
        // tf参数
        sp_nh.param<std::string>("tf/frame_id", tf_frame_id, "ac_local_enu");
        sp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "target_position");
        sp_nh.param("tf/rate_limit", tf_rate, 10.0);

        // 仅支持通过tf方式设置局部位置，免于用户转换
        ROS_INFO_STREAM_NAMED("setpoint_position", "SPP: Listen to position setpoint transform "
                                                       << tf_frame_id << " -> "
                                                       << tf_child_frame_id);
        tf2_start("setpoint_position_tf", &SetpointPositionPlugin::transform_cb);
        set_twist_sub =
            sp_nh.subscribe("set_twist", 10, &SetpointPositionPlugin::set_twist_cb, this);

        // 需要通过服务打开tf_listen开关才能设置局部位置
        ROS_INFO_STREAM_NAMED(
            "setpoint_position",
            "SPP: Always call set_tf_listen service before and after setting local position.");
        std::string state = tf_listen ? "ON" : "OFF";
        ROS_INFO_STREAM_NAMED("setpoint_position", "SPP: Current listen state is: " + state);
        set_tf_listen_srv = sp_nh.advertiseService("set_tf_listen",
                                                   &SetpointPositionPlugin::set_tf_listen_cb, this);

        // 通过订阅方式设置全球位置
        setpoint_global_sub =
            sp_nh.subscribe("global", 10, &SetpointPositionPlugin::setpoint_global_cb, this);
        global_mav_frame_srv = sp_nh.advertiseService(
            "global_mav_frame", &SetpointPositionPlugin::set_mav_frame_cb, this);

        // global_mav_frame
        // 发送信息的坐标系(全球)
        std::string mav_frame_str;
        if (!sp_nh.getParam("global_mav_frame", mav_frame_str)) {
            global_mav_frame = MAV_FRAME::GLOBAL_INT;
        } else {
            global_mav_frame = utils::mav_frame_from_str(mav_frame_str);
            if (global_mav_frame != MAV_FRAME::GLOBAL_INT &&
                global_mav_frame != MAV_FRAME::GLOBAL_RELATIVE_ALT_INT &&
                global_mav_frame != MAV_FRAME::GLOBAL_TERRAIN_ALT_INT) {
                global_mav_frame = MAV_FRAME::GLOBAL_INT;
                ROS_WARN_NAMED("setpoint_position", "SPP: Invalid frame, set frame to GLOBAL_INT.");
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
    friend class SetPositionTargetGlobalIntMixin;
    friend class TF2ListenerMixin;

    ros::NodeHandle    sp_nh;
    ros::Subscriber    setpoint_global_sub;
    ros::Subscriber    set_twist_sub;
    ros::ServiceServer global_mav_frame_srv;
    ros::ServiceServer set_tf_listen_srv;

    std::string tf_frame_id;
    std::string tf_child_frame_id;

    bool                 tf_listen;
    double               tf_rate;
    geometry_msgs::Twist set_twist;

    MAV_FRAME global_mav_frame;
    bool      set_position_local_support_confirmed;
    bool      set_position_global_support_confirmed;

    /* mid-level functions */
    /* 中间件函数 */

    // Acts when capabilities of the fcu are changed
    // 当飞控兼容性改变时执行该函数
    void capabilities_cb(UAS::MAV_CAP capabilities) override {
        if (m_uas->has_capability(UAS::MAV_CAP::SET_POSITION_TARGET_LOCAL_NED)) {
            set_position_local_support_confirmed = true;
        } else {
            set_position_local_support_confirmed = false;
        }

        if (m_uas->has_capability(UAS::MAV_CAP::SET_POSITION_TARGET_GLOBAL_INT)) {
            set_position_global_support_confirmed = true;
        } else {
            set_position_global_support_confirmed = false;
        }
    }

    void send_position_target(const ros::Time &stamp, const Eigen::Affine3d &tr) {
        using mavlink::common::POSITION_TARGET_TYPEMASK;
        lock_guard lock(mutex);

        uint16_t type_mask;
        if (set_twist != geometry_msgs::Twist()) {
            type_mask = uint16_t(POSITION_TARGET_TYPEMASK::AX_IGNORE) |
                        uint16_t(POSITION_TARGET_TYPEMASK::AY_IGNORE) |
                        uint16_t(POSITION_TARGET_TYPEMASK::AZ_IGNORE);
        } else {
            type_mask = uint16_t(POSITION_TARGET_TYPEMASK::VX_IGNORE) |
                        uint16_t(POSITION_TARGET_TYPEMASK::VY_IGNORE) |
                        uint16_t(POSITION_TARGET_TYPEMASK::VZ_IGNORE) |
                        uint16_t(POSITION_TARGET_TYPEMASK::AX_IGNORE) |
                        uint16_t(POSITION_TARGET_TYPEMASK::AY_IGNORE) |
                        uint16_t(POSITION_TARGET_TYPEMASK::AZ_IGNORE) |
                        uint16_t(POSITION_TARGET_TYPEMASK::YAW_RATE_IGNORE);
        }

        auto p = ftf::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));

        auto q = ftf::transform_orientation_enu_ned(
            ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation())));

        Eigen::Vector3d vel_req;
        tf::vectorMsgToEigen(set_twist.linear, vel_req);
        auto v = ftf::transform_frame_enu_ned(vel_req);

        auto yaw_rate =
            ftf::transform_frame_enu_ned(Eigen::Vector3d(0.0, 0.0, set_twist.angular.z));

        set_position_target_local_ned(
            stamp.toNSec() / 1000000, utils::enum_value(MAV_FRAME::LOCAL_NED), type_mask, p, v,
            Eigen::Vector3d::Zero(), ftf::quaternion_get_yaw(q), yaw_rate.z());
    }

    /* ros callbacks */
    /* ROS回调函数 */

    void transform_cb(const geometry_msgs::TransformStamped &transform) {
        if (set_position_local_support_confirmed) {
            // 控制局部位置需要通过服务将tf_listen打开
            if (tf_listen) {
                Eigen::Affine3d tr;
                // TODO: tf2 5.12版本发布需要将下面的函数替换成tf2::convert()
                tf::transformMsgToEigen(transform.transform, tr);
                send_position_target(transform.header.stamp, tr);
            }
        } else {
            ROS_WARN_THROTTLE_NAMED(5, "setpoint_position",
                                    "SPP: Operation error, please check capabilities of FCU!");
        }
    }

    void set_twist_cb(const geometry_msgs::Twist::ConstPtr &req) {
        lock_guard lock(mutex);
        set_twist = *req;
    }

    void setpoint_global_cb(const geographic_msgs::GeoPoseStamped::ConstPtr &req) {
        using mavlink::common::POSITION_TARGET_TYPEMASK;
        if (set_position_global_support_confirmed) {
            uint16_t type_mask = uint16_t(POSITION_TARGET_TYPEMASK::VX_IGNORE) |
                                 uint16_t(POSITION_TARGET_TYPEMASK::VY_IGNORE) |
                                 uint16_t(POSITION_TARGET_TYPEMASK::VZ_IGNORE) |
                                 uint16_t(POSITION_TARGET_TYPEMASK::AX_IGNORE) |
                                 uint16_t(POSITION_TARGET_TYPEMASK::AY_IGNORE) |
                                 uint16_t(POSITION_TARGET_TYPEMASK::AZ_IGNORE) |
                                 uint16_t(POSITION_TARGET_TYPEMASK::YAW_RATE_IGNORE);

            Eigen::Quaterniond attitude;
            tf::quaternionMsgToEigen(req->pose.orientation, attitude);
            Eigen::Quaterniond q = ftf::transform_orientation_enu_ned(
                ftf::transform_orientation_baselink_aircraft(attitude));

            set_position_target_global_int(
                req->header.stamp.toNSec() / 1000000, utils::enum_value(global_mav_frame),
                type_mask, req->pose.position.latitude * 1e7, req->pose.position.longitude * 1e7,
                req->pose.position.altitude, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                ftf::quaternion_get_yaw(q), 0);
        } else {
            ROS_WARN_THROTTLE_NAMED(5, "setpoint_position",
                                    "SPP: Operation error, please check capabilities of FCU!");
        }
    }

    bool set_mav_frame_cb(mavros_msgs::SetMavFrame::Request  &req,
                          mavros_msgs::SetMavFrame::Response &res) {
        //检查设置坐标系格式
        if (static_cast<MAV_FRAME>(req.mav_frame) == MAV_FRAME::GLOBAL_INT ||
            static_cast<MAV_FRAME>(req.mav_frame) == MAV_FRAME::GLOBAL_RELATIVE_ALT_INT ||
            static_cast<MAV_FRAME>(req.mav_frame) == MAV_FRAME::GLOBAL_TERRAIN_ALT_INT) {
            global_mav_frame                = static_cast<MAV_FRAME>(req.mav_frame);
            const std::string mav_frame_str = utils::to_string(global_mav_frame);
            sp_nh.setParam("global_mav_frame", mav_frame_str);
            res.success = true;
        } else {
            ROS_ERROR_NAMED("setpoint_position", "SPP: Invalid frame.");
            res.success = false;
        }
        return true;
    }

    bool set_tf_listen_cb(mavros_msgs::SetTFListen::Request  &req,
                          mavros_msgs::SetTFListen::Response &res) {
        tf_listen         = req.value;
        res.result        = true;
        std::string state = tf_listen ? "ON" : "OFF";
        ROS_INFO_STREAM_NAMED("setpoint_position", "SPP: Current listen state is: " + state);
        return true;
    }
};
} // namespace std_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SetpointPositionPlugin, mavros::plugin::PluginBase)
