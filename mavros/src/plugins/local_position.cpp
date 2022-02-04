/**
 * @file local_position.cpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Glenn Gregory
 * @author Eddy Scott <scott.edward@aurora.aero>
 * @brief This file is from mavros open source respository, thanks for their contribution.
 * @version 1.0
 * @date 2022-01-26
 *
 * @copyright Copyright (c) 2022 acfly
 * @copyright Copyright 2014,2015,2016,2017 Vladimir Ermakov, Glenn Gregory, Eddy Scott.
 * For commercial use, please contact acfly: https://www.acfly.cn
 *
 */

#include <eigen_conversions/eigen_msg.h>
#include <mavros/mavros_plugin.h>

#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <nav_msgs/Odometry.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Local position plugin.
 * @note publish local position to TF, PositionStamped, TwistStamped and Odometry
 * @brief 局部位置的ROS插件
 * @note 该插件可以通过TF，PositionStamped，TwistStamped和Odometry格式发布局部位置
 * @warning 该插件依赖于sys_time插件提供time_offset用于消除飞控与机载电脑的时钟偏移
 */
class LocalPositionPlugin : public plugin::PluginBase {
public:
    LocalPositionPlugin()
        : PluginBase(), lp_nh("~local_position"), tf_send(false), has_local_position_ned(false),
          has_local_position_ned_cov(false) {}

    void initialize(UAS &uas_) override {
        PluginBase::initialize(uas_);

        // general params
        // 通用参数
        lp_nh.param<std::string>("frame_id", frame_id, "ac_local_enu");
        // tf subsection
        // tf子块
        lp_nh.param("tf/send", tf_send, true);
        lp_nh.param<std::string>("tf/frame_id", tf_frame_id, "ac_base_flu");
        lp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "ac_local_enu");

        // fused local position
        // 融合的局部位置
        local_position = lp_nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
        local_position_cov =
            lp_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_cov", 10);
        local_velocity_local = lp_nh.advertise<geometry_msgs::TwistStamped>("velocity_local", 10);
        local_velocity_body  = lp_nh.advertise<geometry_msgs::TwistStamped>("velocity_body", 10);
        local_velocity_cov =
            lp_nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("velocity_body_cov", 10);
        local_accel = lp_nh.advertise<geometry_msgs::AccelWithCovarianceStamped>("accel", 10);
        local_odom  = lp_nh.advertise<nav_msgs::Odometry>("odom", 10);
    }

    Subscriptions get_subscriptions() override {
        return {
            make_handler(&LocalPositionPlugin::handle_local_position_ned),
            make_handler(&LocalPositionPlugin::handle_local_position_ned_cov),
        };
    }

private:
    ros::NodeHandle lp_nh;

    ros::Publisher local_position;
    ros::Publisher local_position_cov;
    ros::Publisher local_velocity_local;
    ros::Publisher local_velocity_body;
    ros::Publisher local_velocity_cov;
    ros::Publisher local_accel;
    ros::Publisher local_odom;

    std::string frame_id;
    std::string tf_frame_id;
    std::string tf_child_frame_id;
    bool        tf_send;
    bool        has_local_position_ned;
    bool        has_local_position_ned_cov;

    /* mid-level functions */
    /* 中间件函数 */

    void publish_tf(boost::shared_ptr<nav_msgs::Odometry> &odom) {
        if (tf_send) {
            geometry_msgs::TransformStamped transform;
            Eigen::Quaterniond              q;
            geometry_msgs::Quaternion       q_msg;
            transform.header.stamp            = odom->header.stamp;
            transform.header.frame_id         = tf_frame_id;
            transform.child_frame_id          = tf_child_frame_id;
            transform.transform.translation.x = -odom->pose.pose.position.x;
            transform.transform.translation.y = -odom->pose.pose.position.y;
            transform.transform.translation.z = -odom->pose.pose.position.z;
            tf::quaternionMsgToEigen(odom->pose.pose.orientation, q);
            tf::quaternionEigenToMsg(q.inverse(), q_msg);
            transform.transform.rotation = q_msg;
            m_uas->tf2_broadcaster.sendTransform(transform);
        }
    }

    /* message handlers */
    /* 信息回调句柄 */

    void handle_local_position_ned(const mavlink::mavlink_message_t         *msg,
                                   mavlink::common::msg::LOCAL_POSITION_NED &pos_ned) {
        has_local_position_ned = true;

        // transform FCU position and velocity data
        // 转换飞控上传的位置和速度数据
        auto enu_position =
            ftf::transform_frame_ned_enu(Eigen::Vector3d(pos_ned.x, pos_ned.y, pos_ned.z));
        auto enu_velocity =
            ftf::transform_frame_ned_enu(Eigen::Vector3d(pos_ned.vx, pos_ned.vy, pos_ned.vz));

        // get odom information
        // Note: this orientation describes baselink->ENU transform
        // 构建里程计数据
        // 注意：该旋转描述了FLU到ENU之间的变换
        auto               enu_orientation_msg  = m_uas->get_attitude_orientation_enu();
        auto               baselink_angular_msg = m_uas->get_attitude_angular_velocity_enu();
        Eigen::Quaterniond enu_orientation;
        tf::quaternionMsgToEigen(enu_orientation_msg, enu_orientation);
        auto baselink_linear =
            ftf::transform_frame_enu_baselink(enu_velocity, enu_orientation.inverse());

        auto odom            = boost::make_shared<nav_msgs::Odometry>();
        odom->header         = m_uas->synchronized_header(frame_id, pos_ned.time_boot_ms);
        odom->child_frame_id = tf_child_frame_id;

        tf::pointEigenToMsg(enu_position, odom->pose.pose.position);
        odom->pose.pose.orientation = enu_orientation_msg;
        tf::vectorEigenToMsg(baselink_linear, odom->twist.twist.linear);
        odom->twist.twist.angular = baselink_angular_msg;

        // publish odom if we don't have LOCAL_POSITION_NED_COV
        // 如果没有LOCAL_POSITION_NED_COV信息则发布里程计数据
        if (!has_local_position_ned_cov) {
            local_odom.publish(odom);
        }

        // publish pose
        // 发布位姿
        auto pose    = boost::make_shared<geometry_msgs::PoseStamped>();
        pose->header = odom->header;
        pose->pose   = odom->pose.pose;
        local_position.publish(pose);

        // publish velocity
        // velocity in the body frame
        // 发布速度，速度坐标系为FLU
        auto twist_body             = boost::make_shared<geometry_msgs::TwistStamped>();
        twist_body->header.stamp    = odom->header.stamp;
        twist_body->header.frame_id = tf_child_frame_id;
        twist_body->twist.linear    = odom->twist.twist.linear;
        twist_body->twist.angular   = baselink_angular_msg;
        local_velocity_body.publish(twist_body);

        // velocity in the local frame
        // 将速度转到ENU
        auto twist_local             = boost::make_shared<geometry_msgs::TwistStamped>();
        twist_local->header.stamp    = twist_body->header.stamp;
        twist_local->header.frame_id = tf_child_frame_id;
        tf::vectorEigenToMsg(enu_velocity, twist_local->twist.linear);
        tf::vectorEigenToMsg(
            ftf::transform_frame_baselink_enu(ftf::to_eigen(baselink_angular_msg), enu_orientation),
            twist_local->twist.angular);

        local_velocity_local.publish(twist_local);

        // publish TF
        // 发布TF
        publish_tf(odom);
    }

    void handle_local_position_ned_cov(const mavlink::mavlink_message_t             *msg,
                                       mavlink::common::msg::LOCAL_POSITION_NED_COV &pos_ned) {
        has_local_position_ned_cov = true;

        auto enu_position =
            ftf::transform_frame_ned_enu(Eigen::Vector3d(pos_ned.x, pos_ned.y, pos_ned.z));
        auto enu_velocity =
            ftf::transform_frame_ned_enu(Eigen::Vector3d(pos_ned.vx, pos_ned.vy, pos_ned.vz));

        auto               enu_orientation_msg  = m_uas->get_attitude_orientation_enu();
        auto               baselink_angular_msg = m_uas->get_attitude_angular_velocity_enu();
        Eigen::Quaterniond enu_orientation;
        tf::quaternionMsgToEigen(enu_orientation_msg, enu_orientation);
        auto baselink_linear =
            ftf::transform_frame_enu_baselink(enu_velocity, enu_orientation.inverse());

        auto odom            = boost::make_shared<nav_msgs::Odometry>();
        odom->header         = m_uas->synchronized_header(frame_id, pos_ned.time_usec);
        odom->child_frame_id = tf_child_frame_id;

        tf::pointEigenToMsg(enu_position, odom->pose.pose.position);
        odom->pose.pose.orientation = enu_orientation_msg;
        tf::vectorEigenToMsg(baselink_linear, odom->twist.twist.linear);
        odom->twist.twist.angular = baselink_angular_msg;

        odom->pose.covariance[0]  = pos_ned.covariance[0];  // x
        odom->pose.covariance[7]  = pos_ned.covariance[9];  // y
        odom->pose.covariance[14] = pos_ned.covariance[17]; // z

        odom->twist.covariance[0]  = pos_ned.covariance[24]; // vx
        odom->twist.covariance[7]  = pos_ned.covariance[30]; // vy
        odom->twist.covariance[14] = pos_ned.covariance[35]; // vz
        // TODO: orientation + angular velocity covariances from ATTITUDE_QUATERION_COV

        // publish odom
        // 发布里程计数据
        local_odom.publish(odom);

        // publish pose_cov
        // 发布带协方差的位姿
        auto pose_cov    = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
        pose_cov->header = odom->header;
        pose_cov->pose   = odom->pose;
        local_position_cov.publish(pose_cov);

        // publish velocity_cov
        // 发布带协方差的速度
        auto twist_cov          = boost::make_shared<geometry_msgs::TwistWithCovarianceStamped>();
        twist_cov->header.stamp = odom->header.stamp;
        twist_cov->header.frame_id = odom->child_frame_id;
        twist_cov->twist           = odom->twist;
        local_velocity_cov.publish(twist_cov);

        // publish pose, velocity, tf if we don't have LOCAL_POSITION_NED
        // 如果没有LOCAL_POSITION_NED信息则发布位姿，速度和TF
        if (!has_local_position_ned) {
            auto pose    = boost::make_shared<geometry_msgs::PoseStamped>();
            pose->header = odom->header;
            pose->pose   = odom->pose.pose;
            local_position.publish(pose);

            auto twist             = boost::make_shared<geometry_msgs::TwistStamped>();
            twist->header.stamp    = odom->header.stamp;
            twist->header.frame_id = odom->child_frame_id;
            twist->twist           = odom->twist.twist;
            local_velocity_body.publish(twist);

            // publish TF
            // 发布TF
            publish_tf(odom);
        }

        // publish accelerations
        // 发布加速度
        auto accel    = boost::make_shared<geometry_msgs::AccelWithCovarianceStamped>();
        accel->header = odom->header;

        auto enu_accel =
            ftf::transform_frame_ned_enu(Eigen::Vector3d(pos_ned.ax, pos_ned.ay, pos_ned.az));
        tf::vectorEigenToMsg(enu_accel, accel->accel.accel.linear);

        accel->accel.covariance[0]  = pos_ned.covariance[39];
        accel->accel.covariance[7]  = pos_ned.covariance[42];
        accel->accel.covariance[14] = pos_ned.covariance[44];

        local_accel.publish(accel);
    }
};
} // namespace std_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::LocalPositionPlugin, mavros::plugin::PluginBase)
