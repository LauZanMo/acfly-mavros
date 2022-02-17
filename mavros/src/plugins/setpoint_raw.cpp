/**
 * @file setpoint_raw.cpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @brief This file is from mavros open source respository, thanks for their contribution.
 * @version 1.0
 * @date 2022-01-26
 *
 * @copyright Copyright (c) 2022 acfly
 * @copyright Copyright 2014,2015,2016,2017 Vladimir Ermakov.
 * For commercial use, please contact acfly: https://www.acfly.cn
 *
 */

#include <eigen_conversions/eigen_msg.h>
#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/PositionTarget.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Setpoint RAW plugin
 * @note Send position setpoints and publish current state (return loop).
 * User can decide what set of filed needed for operation via IGNORE bits.
 * @brief offboard模式飞控控制ROS插件，接收标准mavlink的ROS信息
 * @note 分别发送SET_ATTITUDE_TARGET，SET_POSITION_TARGET_LOCAL_NED和
 * SET_POSITION_TARGET_GLOBAL_INT信息并对相应应答信息进行监控。
 * @warning 该插件依赖于sys_time插件提供time_offset用于消除飞控与机载电脑的时钟偏移
 * @warning 该插件依赖于系统兼容性支持
 */
class SetpointRawPlugin : public plugin::PluginBase,
                          private plugin::SetPositionTargetLocalNEDMixin<SetpointRawPlugin>,
                          private plugin::SetPositionTargetGlobalIntMixin<SetpointRawPlugin>,
                          private plugin::SetAttitudeTargetMixin<SetpointRawPlugin> {
public:
    SetpointRawPlugin()
        : PluginBase(), sp_nh("~setpoint_raw"), set_attitude_support_confirmed(false),
          set_position_local_support_confirmed(false),
          set_position_global_support_confirmed(false) {}

    void initialize(UAS &uas_) override {
        PluginBase::initialize(uas_);

        local_sub         = sp_nh.subscribe("local", 10, &SetpointRawPlugin::local_cb, this);
        global_sub        = sp_nh.subscribe("global", 10, &SetpointRawPlugin::global_cb, this);
        attitude_sub      = sp_nh.subscribe("attitude", 10, &SetpointRawPlugin::attitude_cb, this);
        target_local_pub  = sp_nh.advertise<mavros_msgs::PositionTarget>("target_local", 10);
        target_global_pub = sp_nh.advertise<mavros_msgs::GlobalPositionTarget>("target_global", 10);
        target_attitude_pub = sp_nh.advertise<mavros_msgs::AttitudeTarget>("target_attitude", 10);

        // Set Thrust scaling in config.yaml, otherwise set attitude will be ignored.
        // 在配置文件中设置拉力的比例，否则设置角度的命令会被忽略
        if (!sp_nh.getParam("thrust_scaling", thrust_scaling)) {
            ROS_WARN_NAMED("setpoint_raw", "SPR: thrust_scaling parameter is unset. Attitude (and "
                                           "angular rate/thrust) setpoints will be ignored.");
            thrust_scaling = -1.0;
        }

        enable_capabilities_cb();
    }

    Subscriptions get_subscriptions() override {
        return {
            make_handler(&SetpointRawPlugin::handle_position_target_local_ned),
            make_handler(&SetpointRawPlugin::handle_position_target_global_int),
            make_handler(&SetpointRawPlugin::handle_attitude_target),
        };
    }

private:
    friend class SetPositionTargetLocalNEDMixin;
    friend class SetPositionTargetGlobalIntMixin;
    friend class SetAttitudeTargetMixin;
    ros::NodeHandle sp_nh;

    ros::Subscriber local_sub, global_sub, attitude_sub;
    ros::Publisher  target_local_pub, target_global_pub, target_attitude_pub;

    double thrust_scaling;
    bool   set_attitude_support_confirmed;
    bool   set_position_local_support_confirmed;
    bool   set_position_global_support_confirmed;

    /* message handlers */
    /* 信息回调句柄 */

    //! @warning: 原来的代码坐标系转换好像和实际不符，但是效果一致
    void handle_position_target_local_ned(const mavlink::mavlink_message_t                *msg,
                                          mavlink::common::msg::POSITION_TARGET_LOCAL_NED &tgt) {
        // Transform desired position,velocities,and accels from NED to ENU frame
        // 将期望的位置、速度和加速度从NED转到ENU
        auto position = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.x, tgt.y, tgt.z));
        auto velocity = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.vx, tgt.vy, tgt.vz));
        auto af       = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.afx, tgt.afy, tgt.afz));
        // 为什么这里只用了相对坐标系？怀疑原作者偷懒
        float yaw = ftf::quaternion_get_yaw(ftf::transform_orientation_aircraft_baselink(
            ftf::transform_orientation_ned_enu(ftf::quaternion_from_rpy(0.0, 0.0, tgt.yaw))));
        // 只在Z轴有值的向量ENU->NED和FLU->FRD结果一样，这一步属于节省篇幅
        Eigen::Vector3d ang_vel_ned(0.0, 0.0, tgt.yaw_rate);
        auto            ang_vel_enu = ftf::transform_frame_ned_enu(ang_vel_ned);
        float           yaw_rate    = ang_vel_enu.z();

        auto target = boost::make_shared<mavros_msgs::PositionTarget>();

        target->header.stamp     = m_uas->synchronise_stamp(tgt.time_boot_ms);
        target->coordinate_frame = tgt.coordinate_frame;
        target->type_mask        = tgt.type_mask;
        tf::pointEigenToMsg(position, target->position);
        tf::vectorEigenToMsg(velocity, target->velocity);
        tf::vectorEigenToMsg(af, target->acceleration_or_force);
        target->yaw      = yaw;
        target->yaw_rate = yaw_rate;

        target_local_pub.publish(target);
    }

    void handle_position_target_global_int(const mavlink::mavlink_message_t                 *msg,
                                           mavlink::common::msg::POSITION_TARGET_GLOBAL_INT &tgt) {
        // Transform desired velocities from NED to ENU frame
        // 将期望的速度从NED转到ENU
        auto  velocity = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.vx, tgt.vy, tgt.vz));
        auto  af       = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.afx, tgt.afy, tgt.afz));
        float yaw      = ftf::quaternion_get_yaw(ftf::transform_orientation_aircraft_baselink(
                 ftf::transform_orientation_ned_enu(ftf::quaternion_from_rpy(0.0, 0.0, tgt.yaw))));
        Eigen::Vector3d ang_vel_ned(0.0, 0.0, tgt.yaw_rate);
        auto            ang_vel_enu = ftf::transform_frame_ned_enu(ang_vel_ned);
        float           yaw_rate    = ang_vel_enu.z();

        auto target = boost::make_shared<mavros_msgs::GlobalPositionTarget>();

        target->header.stamp     = m_uas->synchronise_stamp(tgt.time_boot_ms);
        target->coordinate_frame = tgt.coordinate_frame;
        target->type_mask        = tgt.type_mask;
        target->latitude         = tgt.lat_int / 1e7;
        target->longitude        = tgt.lon_int / 1e7;
        target->altitude         = tgt.alt;
        tf::vectorEigenToMsg(velocity, target->velocity);
        tf::vectorEigenToMsg(af, target->acceleration_or_force);
        target->yaw      = yaw;
        target->yaw_rate = yaw_rate;

        target_global_pub.publish(target);
    }

    void handle_attitude_target(const mavlink::mavlink_message_t      *msg,
                                mavlink::common::msg::ATTITUDE_TARGET &tgt) {
        // Transform orientation from aircraft -> NED
        // to baselink -> ENU
        // 将期望的旋转从FRD->NED转到FLU->ENU
        auto orientation = ftf::transform_orientation_ned_enu(
            ftf::transform_orientation_aircraft_baselink(ftf::mavlink_to_quaternion(tgt.q)));

        auto body_rate = ftf::transform_frame_aircraft_baselink(
            Eigen::Vector3d(tgt.body_roll_rate, tgt.body_pitch_rate, tgt.body_yaw_rate));

        auto target = boost::make_shared<mavros_msgs::AttitudeTarget>();

        target->header.stamp = m_uas->synchronise_stamp(tgt.time_boot_ms);
        target->type_mask    = tgt.type_mask;
        tf::quaternionEigenToMsg(orientation, target->orientation);
        tf::vectorEigenToMsg(body_rate, target->body_rate);
        target->thrust = tgt.thrust;

        target_attitude_pub.publish(target);
    }

    /* mid-level functions */
    /* 中间件函数 */

    // Acts when capabilities of the fcu are changed
    // 当飞控兼容性改变时执行该函数
    void capabilities_cb(UAS::MAV_CAP capabilities) override {
        if (m_uas->has_capability(UAS::MAV_CAP::SET_ATTITUDE_TARGET)) {
            set_attitude_support_confirmed = true;
            ROS_INFO_NAMED("setpoint_raw", "SPR: Set attitude target command is supported.");
        } else {
            set_attitude_support_confirmed = false;
            ROS_WARN_NAMED("setpoint_raw", "SPR: Set attitude target command is not supported.");
        }

        if (m_uas->has_capability(UAS::MAV_CAP::SET_POSITION_TARGET_LOCAL_NED)) {
            set_position_local_support_confirmed = true;
            ROS_INFO_NAMED("setpoint_raw", "SPR: Set position target local command is supported.");
        } else {
            set_position_local_support_confirmed = false;
            ROS_WARN_NAMED("setpoint_raw",
                           "SPR: Set position target local command is not supported.");
        }

        if (m_uas->has_capability(UAS::MAV_CAP::SET_POSITION_TARGET_GLOBAL_INT)) {
            set_position_global_support_confirmed = true;
            ROS_INFO_NAMED("setpoint_raw", "SPR: Set position target global command is supported.");
        } else {
            set_position_global_support_confirmed = false;
            ROS_WARN_NAMED("setpoint_raw",
                           "SPR: Set position target global command is not supported.");
        }
    }

    /* ros callbacks */
    /* ROS回调函数 */

    void local_cb(const mavros_msgs::PositionTarget::ConstPtr &req) {
        if (set_position_local_support_confirmed) {
            Eigen::Vector3d position, velocity, af;
            float           yaw, yaw_rate;

            tf::pointMsgToEigen(req->position, position);
            tf::vectorMsgToEigen(req->velocity, velocity);
            tf::vectorMsgToEigen(req->acceleration_or_force, af);

            // Transform frame ENU->NED
            // 坐标系从ENU转到NED
            if (req->coordinate_frame == mavros_msgs::PositionTarget::FRAME_BODY_NED ||
                req->coordinate_frame == mavros_msgs::PositionTarget::FRAME_BODY_OFFSET_NED) {
                position = ftf::transform_frame_baselink_aircraft(position);
                velocity = ftf::transform_frame_baselink_aircraft(velocity);
                af       = ftf::transform_frame_baselink_aircraft(af);
                yaw      = ftf::quaternion_get_yaw(
                         // 视FLU为绝对坐标系
                    ftf::transform_orientation_absolute_frame_baselink_aircraft(
                             ftf::quaternion_from_rpy(0.0, 0.0, req->yaw)));
            } else if (req->coordinate_frame == mavros_msgs::PositionTarget::FRAME_LOCAL_NED ||
                       req->coordinate_frame ==
                           mavros_msgs::PositionTarget::FRAME_LOCAL_OFFSET_NED) {
                position = ftf::transform_frame_enu_ned(position);
                velocity = ftf::transform_frame_enu_ned(velocity);
                af       = ftf::transform_frame_enu_ned(af);
                yaw      = ftf::quaternion_get_yaw(
                         ftf::transform_orientation_baselink_aircraft(ftf::transform_orientation_enu_ned(
                             ftf::quaternion_from_rpy(0.0, 0.0, req->yaw))));
            } else {
                ROS_ERROR_THROTTLE_NAMED(5, "setpoint_raw", "SPR: Invalid frame.");
                return;
            }

            // 只在Z轴有值的向量ENU->NED和FLU->FRD结果一样，这一步属于节省篇幅
            Eigen::Vector3d ang_vel_enu(0.0, 0.0, req->yaw_rate);
            auto            ang_vel_ned = ftf::transform_frame_enu_ned(ang_vel_enu);
            yaw_rate                    = ang_vel_ned.z();

            set_position_target_local_ned(req->header.stamp.toNSec() / 1000000,
                                          req->coordinate_frame, req->type_mask, position, velocity,
                                          af, yaw, yaw_rate);
        } else {
            ROS_WARN_THROTTLE_NAMED(5, "setpoint_raw",
                                    "SPR: Operation error, please check capabilities of FCU!");
        }
    }

    void global_cb(const mavros_msgs::GlobalPositionTarget::ConstPtr &req) {
        if (set_position_global_support_confirmed) {
            Eigen::Vector3d velocity, af;
            float           yaw, yaw_rate;

            tf::vectorMsgToEigen(req->velocity, velocity);
            tf::vectorMsgToEigen(req->acceleration_or_force, af);
            if (req->coordinate_frame != mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT &&
                req->coordinate_frame != mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT &&
                req->coordinate_frame !=
                    mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_TERRAIN_ALT) {
                ROS_ERROR_THROTTLE_NAMED(5, "setpoint_raw", "SPR: Invalid frame.");
                return;
            }

            // Transform frame ENU->NED
            // 坐标系从ENU转到NED
            velocity = ftf::transform_frame_enu_ned(velocity);
            af       = ftf::transform_frame_enu_ned(af);
            yaw      = ftf::quaternion_get_yaw(ftf::transform_orientation_baselink_aircraft(
                     ftf::transform_orientation_enu_ned(ftf::quaternion_from_rpy(0.0, 0.0, req->yaw))));
            Eigen::Vector3d ang_vel_enu(0.0, 0.0, req->yaw_rate);
            auto            ang_vel_ned = ftf::transform_frame_enu_ned(ang_vel_enu);
            yaw_rate                    = ang_vel_ned.z();

            set_position_target_global_int(req->header.stamp.toNSec() / 1000000,
                                           req->coordinate_frame, req->type_mask,
                                           req->latitude * 1e7, req->longitude * 1e7, req->altitude,
                                           velocity, af, yaw, yaw_rate);
        } else {
            ROS_WARN_THROTTLE_NAMED(5, "setpoint_raw",
                                    "SPR: Operation error, Please check capabilities of FCU!");
        }
    }

    void attitude_cb(const mavros_msgs::AttitudeTarget::ConstPtr &req) {
        if (set_attitude_support_confirmed) {
            Eigen::Quaterniond desired_orientation;
            Eigen::Vector3d    baselink_angular_rate;
            Eigen::Vector3d    body_rate;
            double             thrust;

            // ignore thrust is false by default, unless no thrust scaling is set or thrust is zero
            // ignore thrust变量缺省值为false，除非没有设置拉力比例或比例为0
            auto ignore_thrust = req->thrust != 0.0 && thrust_scaling < 0.0;

            if (ignore_thrust) {
                ROS_ERROR_THROTTLE_NAMED(
                    5, "setpoint_raw",
                    "SPR: Recieved thrust, but ignore_thrust is true: the most likely cause of "
                    "this is a failure to specify the thrust_scaling parameters on "
                    "px4/apm_config.yaml. Actuation will be ignored.");
                return;
            } else {
                if (thrust_scaling == 0.0) {
                    ROS_WARN_THROTTLE_NAMED(5, "setpoint_raw",
                                            "thrust_scaling parameter is set to zero.");
                }
                thrust = std::min(1.0, std::max(0.0, req->thrust * thrust_scaling));
            }

            // Take care of attitude setpoint
            // 注意角度设置
            desired_orientation = ftf::to_eigen(req->orientation);

            // Transform desired orientation to represent aircraft->NED,
            // MAVROS operates on orientation of base_link->ENU
            // 将期望旋转从FLU->ENU转到FRD->NED
            auto ned_desired_orientation = ftf::transform_orientation_enu_ned(
                ftf::transform_orientation_baselink_aircraft(desired_orientation));

            body_rate = ftf::transform_frame_baselink_aircraft(ftf::to_eigen(req->body_rate));

            set_attitude_target(req->header.stamp.toNSec() / 1000000, req->type_mask,
                                ned_desired_orientation, body_rate, thrust);
        } else {
            ROS_WARN_THROTTLE_NAMED(5, "setpoint_raw",
                                    "SPR: Operation error, Please check capabilities of FCU!");
        }
    }
};
} // namespace std_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SetpointRawPlugin, mavros::plugin::PluginBase)
