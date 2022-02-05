/**
 * @file ac_position_sensor.cpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @brief acfly position sensor plugin.
 * @version 1.0
 * @date 2022-02-03
 *
 * @copyright Copyright (c) 2022 acfly
 * For commercial use, please contact acfly: https://www.acfly.cn
 *
 */

#include <eigen_conversions/eigen_msg.h>
#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief acfly position sensor send plugin
 * @brief acfly位置传感器发送插件
 * @warning acfly自定义的信息全部直接以ENU-FLU系发送，与PX4不同
 */
class AcPositionSensorPlugin : public plugin::PluginBase,
                               private plugin::TF2ListenerMixin<AcPositionSensorPlugin> {
public:
    AcPositionSensorPlugin()
        : PluginBase(), aps_nh("~acfly_position_sensor"), tf_rate(10.0), reset_counter(0) {}

    void initialize(UAS &uas_) override {
        PluginBase::initialize(uas_);

        // 传感器到飞控body系的三维变换
        std::vector<double> rot{}, trans{};
        std::string         sensor_id, body_id;
        aps_nh.param<std::string>("sensor_id", sensor_id, "camera");
        aps_nh.param<std::string>("body_id", body_id, "ac_base_flu");
        aps_nh.getParam("sensor_body_rotation", rot);
        aps_nh.getParam("sensor_body_translation", trans);
        Eigen::Affine3d tr_sensor_body(ftf::quaternion_from_rpy(rot[0], rot[1], rot[2]));
        tr_sensor_body.translation() = Eigen::Vector3d(trans[0], trans[1], trans[2]);

        // 传感器参数
        aps_nh.param<std::string>("sensor/name", sensor_name, "ROS");
        aps_nh.param("sensor/index", sensor_ind, 15);                  // 最高16路
        aps_nh.param("sensor/type", sensor_type, 1);                   // 相对定位
        aps_nh.param("sensor/data_frame", sensor_data_frame, 4);       // SLAM坐标系下的位置
        aps_nh.param("sensor/data_type", sensor_data_type, 2);         // 三轴位置
        aps_nh.param<float>("sensor/delay", sensor_delay, 0.05);       // 延时(s)
        aps_nh.param<float>("sensor/trust_xy", sensor_trust_xy, 0.01); // 方差(m^2)
        aps_nh.param<float>("sensor/trust_z", sensor_trust_z, 0.01);

        // tf参数
        bool tf_listen;
        aps_nh.param("tf/listen", tf_listen, true);
        aps_nh.param<std::string>("tf/frame_id", tf_frame_id, "map");
        aps_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "ac_base_flu");
        aps_nh.param("tf/rate_limit", tf_rate, 10.0);

        // 添加静态tf(SLAM传感器与飞控body系之间的变换)
        std::vector<geometry_msgs::TransformStamped> transform_vector;
        m_uas->add_static_transform(sensor_id, body_id, tr_sensor_body, transform_vector);
        m_uas->tf2_static_broadcaster.sendTransform(transform_vector);

        // 注册传感器信息
        register_position_sensor();

        // 启动tf监听线程(详细请看TF2ListenerMixin)或订阅
        if (tf_listen) {
            ROS_INFO_STREAM_NAMED("acfly_position_sensor", "APS: Listen to transform "
                                                               << tf_frame_id << " -> "
                                                               << tf_child_frame_id);
            tf2_start("acfly_pose_tf", &AcPositionSensorPlugin::transform_cb);
        } else {
            ROS_INFO_STREAM_NAMED("acfly_position_sensor", "APS: Subscribe pose");
            pose_sub = aps_nh.subscribe("pose", 10, &AcPositionSensorPlugin::pose_cb, this);
            pose_cov_sub =
                aps_nh.subscribe("pose_cov", 10, &AcPositionSensorPlugin::pose_cov_cb, this);
        }

        // 回环检测
        loop_sub = aps_nh.subscribe("loop", 10, &AcPositionSensorPlugin::loop_cb, this);
    }

    Subscriptions get_subscriptions() override {
        return {/* 禁用接收 */};
    }

private:
    friend class TF2ListenerMixin;
    ros::NodeHandle aps_nh;

    ros::Subscriber pose_sub;
    ros::Subscriber pose_cov_sub;
    ros::Subscriber loop_sub;

    std::string tf_frame_id;
    std::string tf_child_frame_id;
    double      tf_rate;
    ros::Time   last_transform_stamp;

    std::string sensor_name;
    int         sensor_ind, sensor_type;
    int         sensor_data_frame, sensor_data_type;
    float       sensor_delay;
    float       sensor_trust_xy, sensor_trust_z;
    uint8_t     reset_counter;

    /* low-level send */
    /* 底层发送 */

    void register_position_sensor() {
        mavlink::commonACFly::msg::ACFly_RegeisterPosSensor rp{};
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

    void update_position_sensor(const ros::Time &stamp, const Eigen::Affine3d &tr) {
        if (last_transform_stamp == stamp) {
            ROS_DEBUG_THROTTLE_NAMED(10, "acfly_position_sensor",
                                     "APS: Same transform as last one, dropped.");
            return;
        }
        last_transform_stamp = stamp;

        // 消息标头和传感器基本信息
        mavlink::commonACFly::msg::ACFly_UpdatePosSensor up{};
        m_uas->msg_set_target(up);
        up.ind      = sensor_ind;
        up.DataType = -1; // -1为保持不变

        // 定位信息
        up.posX       = tr.translation().x();
        up.posY       = tr.translation().y();
        up.posZ       = tr.translation().z();
        auto q        = Eigen::Quaterniond(tr.rotation());
        up.AttQuat[0] = q.w();
        up.AttQuat[1] = q.x();
        up.AttQuat[2] = q.y();
        up.AttQuat[3] = q.z();

        // 传感器测量信息
        up.delay   = ros::Time::now().toSec() - stamp.toSec();
        up.trustXY = sensor_trust_xy;
        up.trustZ  = sensor_trust_z;
        up.reset   = reset_counter;

        UAS_FCU(m_uas)->send_message_ignore_drop(up);
    }

    /* ros callbacks */
    /* ROS回调函数 */

    void transform_cb(const geometry_msgs::TransformStamped &transform) {
        Eigen::Affine3d tr;
        tf::transformMsgToEigen(transform.transform, tr);

        update_position_sensor(transform.header.stamp, tr);
    }

    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pose_stamp) {
        Eigen::Affine3d tr;
        tf::poseMsgToEigen(pose_stamp->pose, tr);

        update_position_sensor(pose_stamp->header.stamp, tr);
    }

    void pose_cov_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_cov_stamp) {
        Eigen::Affine3d tr;
        tf::poseMsgToEigen(pose_cov_stamp->pose.pose, tr);

        update_position_sensor(pose_cov_stamp->header.stamp, tr);
    }

    void loop_cb(const std_msgs::Bool::ConstPtr &loop) {
        if (loop->data)
            reset_counter++;
    }
};
} // namespace std_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::AcPositionSensorPlugin, mavros::plugin::PluginBase)