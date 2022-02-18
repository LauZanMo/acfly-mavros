/**
 * @file acfly_slam_sensor.cpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @brief acfly slam sensor plugin.
 * @version 1.0
 * @date 2022-02-03
 *
 * @copyright Copyright (c) 2022 acfly
 * For commercial use, please contact acfly: https://www.acfly.cn
 *
 */

#include <eigen_conversions/eigen_msg.h>
#include <mavros/acfly_position_sensor_mixin.h>
#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief acfly slam sensor send plugin
 * @brief acfly位置传感器发送插件，专门用来发送slam信息
 * @warning acfly自定义的信息全部直接以ENU-FLU系发送，与PX4不同
 */
class AcflySlamSensorPlugin : public plugin::PluginBase,
                              private plugin::TF2ListenerMixin<AcflySlamSensorPlugin>,
                              private plugin::AcflyPositionSensorMixin<AcflySlamSensorPlugin> {
public:
    AcflySlamSensorPlugin()
        : PluginBase(), ass_nh("~acfly_slam_sensor"), tf_rate(10.0), reset_counter(0) {}

    void initialize(UAS &uas_) override {
        PluginBase::initialize(uas_);

        // 传感器到飞控body系的三维变换
        Eigen::Affine3d     tr_sensor_body{};
        std::vector<double> rot{}, trans{};
        std::string         sensor_id, body_id;
        ass_nh.param<std::string>("sensor_id", sensor_id, "camera");
        ass_nh.param<std::string>("body_id", body_id, "base_link");
        // 获取sensor->body的变换
        if (ass_nh.getParam("sensor_body_rotation", rot)) {
            tr_sensor_body = Eigen::Affine3d(ftf::quaternion_from_rpy(rot[0], rot[1], rot[2]));
        } else {
            tr_sensor_body = Eigen::Affine3d(ftf::quaternion_from_rpy(0, 0, 0));
            ROS_WARN_STREAM_NAMED(
                "acfly_slam_sensor",
                "ASS: No rotation parameter between sensor and body, set to default(0, 0, 0)");
        }
        if (ass_nh.getParam("sensor_body_translation", trans)) {
            tr_sensor_body.translation() = Eigen::Vector3d(trans[0], trans[1], trans[2]);
        } else {
            tr_sensor_body.translation() = Eigen::Vector3d(0, 0, 0);
            ROS_WARN_STREAM_NAMED(
                "acfly_slam_sensor",
                "ASS: No translation parameter between sensor and body, set to default(0, 0, 0)");
        }

        // 传感器参数
        ass_nh.param<std::string>("sensor/name", sensor_name, "ROS");
        ass_nh.param("sensor/index", sensor_ind, 15);                  // 最高16路
        ass_nh.param("sensor/type", sensor_type, 1);                   // 相对定位
        ass_nh.param("sensor/data_frame", sensor_data_frame, 4);       // SLAM坐标系下的位置
        ass_nh.param("sensor/data_type", sensor_data_type, 2);         // 三轴位置
        ass_nh.param<float>("sensor/delay", sensor_delay, 0.05);       // 延时(s)
        ass_nh.param<float>("sensor/trust_xy", sensor_trust_xy, 0.01); // 方差(m^2)
        ass_nh.param<float>("sensor/trust_z", sensor_trust_z, 0.01);

        // tf参数
        bool tf_listen;
        ass_nh.param("tf/listen", tf_listen, true);
        ass_nh.param<std::string>("tf/frame_id", tf_frame_id, "map");
        ass_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "base_link");
        ass_nh.param("tf/rate_limit", tf_rate, 10.0);

        // 添加静态tf(SLAM传感器与飞控body系之间的变换)
        std::vector<geometry_msgs::TransformStamped> transform_vector;
        m_uas->add_static_transform(sensor_id, body_id, tr_sensor_body, transform_vector);
        m_uas->tf2_static_broadcaster.sendTransform(transform_vector);

        // 注册传感器信息
        register_position_sensor();

        // 启动tf监听线程(详细请看TF2ListenerMixin)或订阅
        if (tf_listen) {
            ROS_INFO_STREAM_NAMED("acfly_slam_sensor", "ASS: Listen to transform "
                                                           << tf_frame_id << " -> "
                                                           << tf_child_frame_id);
            tf2_start("acfly_slam_tf", &AcflySlamSensorPlugin::transform_cb);
        } else {
            ROS_INFO_STREAM_NAMED("acfly_slam_sensor", "ASS: Subscribe pose");
            pose_sub = ass_nh.subscribe("pose", 10, &AcflySlamSensorPlugin::pose_cb, this);
            pose_cov_sub =
                ass_nh.subscribe("pose_cov", 10, &AcflySlamSensorPlugin::pose_cov_cb, this);
        }

        // 回环检测
        loop_sub = ass_nh.subscribe("loop", 10, &AcflySlamSensorPlugin::loop_cb, this);
    }

    Subscriptions get_subscriptions() override {
        return {/* 禁用接收 */};
    }

private:
    friend class TF2ListenerMixin;
    friend class AcflyPositionSensorMixin;
    ros::NodeHandle ass_nh;

    ros::Subscriber pose_sub;
    ros::Subscriber pose_cov_sub;
    ros::Subscriber loop_sub;

    std::string tf_frame_id;
    std::string tf_child_frame_id;
    double      tf_rate;

    std::string sensor_name;
    int         sensor_ind, sensor_type;
    int         sensor_data_frame, sensor_data_type;
    float       sensor_delay;
    float       sensor_trust_xy, sensor_trust_z;
    uint8_t     reset_counter;

    /* ros callbacks */
    /* ROS回调函数 */

    void transform_cb(const geometry_msgs::TransformStamped &transform) {
        Eigen::Affine3d tr;
        tf::transformMsgToEigen(transform.transform, tr);

        update_position_sensor(transform.header.stamp, tr.translation(), Eigen::Vector3d::Zero(),
                               Eigen::Quaterniond(tr.rotation()));
    }

    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pose_stamp) {
        Eigen::Affine3d tr;
        tf::poseMsgToEigen(pose_stamp->pose, tr);

        update_position_sensor(pose_stamp->header.stamp, tr.translation(), Eigen::Vector3d::Zero(),
                               Eigen::Quaterniond(tr.rotation()));
    }

    void pose_cov_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_cov_stamp) {
        Eigen::Affine3d tr;
        tf::poseMsgToEigen(pose_cov_stamp->pose.pose, tr);

        update_position_sensor(pose_cov_stamp->header.stamp, tr.translation(),
                               Eigen::Vector3d::Zero(), Eigen::Quaterniond(tr.rotation()));
    }

    void loop_cb(const std_msgs::Bool::ConstPtr &loop) {
        // 可能会有多线程读写问题
        if (loop->data)
            reset_counter++;
    }
};
} // namespace std_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::AcflySlamSensorPlugin, mavros::plugin::PluginBase)