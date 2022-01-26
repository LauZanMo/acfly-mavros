/**
 * @file global_position.cpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @brief This file is from mavros open source respository, thanks for their contribution.
 * @version 1.0
 * @date 2022-01-25
 *
 * @copyright Copyright (c) 2022 acfly
 * @copyright Copyright 2014,2015,2016,2017 Vladimir Ermakov, Nuno Marques.
 * For commercial use, please contact acfly: https://www.acfly.cn
 *
 */

#include <GeographicLib/Geocentric.hpp>
#include <angles/angles.h>
#include <eigen_conversions/eigen_msg.h>
#include <mavros/mavros_plugin.h>

#include <geographic_msgs/GeoPointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt32.h>

#include <mavros_msgs/HomePosition.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Global position plugin.
 * @note Publishes global position. Conversion from GPS LLA to ECEF allows
 * publishing local position to TF and PoseWithCovarianceStamped.
 *
 * @brief 全球位置的ROS插件
 * @note 发布全球位置，从GPS纬经高到地心地固坐标系的转换(与GPS原点，即起飞点或自设定的点)，
 * 使得该插件能通过tf和PoseWithCovarianceStamped格式发布局部位置
 * @warning 该插件依赖于sys_time插件提供time_offset用于消除飞控与机载电脑的时钟偏移
 */
class GlobalPositionPlugin : public plugin::PluginBase {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GlobalPositionPlugin()
        : PluginBase(), gp_nh("~global_position"), tf_send(false), use_relative_alt(true),
          is_map_init(false), outdoor_switch(false), rot_cov(99999.0) {}

    void initialize(UAS &uas_) override {
        PluginBase::initialize(uas_);

        // general params
        // 通用参数
        gp_nh.param<std::string>("frame_id", frame_id, "ac_map_enu");
        gp_nh.param<std::string>("child_frame_id", child_frame_id, "ac_base_enu");
        gp_nh.param<std::string>("global_frame_id", global_frame_id, "ac_earth");
        gp_nh.param("rot_covariance", rot_cov, 99999.0);
        gp_nh.param("gps_uere", gps_uere, 1.0);
        gp_nh.param("use_relative_alt", use_relative_alt, true);
        // tf subsection
        // tf子块
        gp_nh.param("tf/send", tf_send, false);
        gp_nh.param<std::string>("tf/frame_id", tf_frame_id, "ac_map_enu");
        gp_nh.param<std::string>("tf/global_frame_id", tf_global_frame_id,
                                 "ac_earth"); // The global_origin should be represented as "earth"
                                              // coordinate frame (ECEF) (REP 105)
                                              // 全球定位原点应以地心地固坐标系表示
        gp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "ac_base_enu");

        UAS_DIAG(m_uas).add("GPS", this, &GlobalPositionPlugin::gps_diag_run);

        // gps data
        // GPS原始数据
        raw_fix_pub = gp_nh.advertise<sensor_msgs::NavSatFix>("raw/fix", 10);
        raw_vel_pub = gp_nh.advertise<geometry_msgs::TwistStamped>("raw/gps_vel", 10);
        raw_sat_pub = gp_nh.advertise<std_msgs::UInt32>("raw/satellites", 10);

        // fused global position
        // 融合的全球定位
        gp_fix_pub     = gp_nh.advertise<sensor_msgs::NavSatFix>("global", 10);
        gp_odom_pub    = gp_nh.advertise<nav_msgs::Odometry>("local", 10);
        gp_rel_alt_pub = gp_nh.advertise<std_msgs::Float64>("rel_alt", 10);
        gp_hdg_pub     = gp_nh.advertise<std_msgs::Float64>("compass_hdg", 10);

        // global origin
        // 全球定位原点
        gp_global_origin_pub = gp_nh.advertise<geographic_msgs::GeoPointStamped>("gp_origin", 10);
        gp_set_global_origin_sub =
            gp_nh.subscribe("set_gp_origin", 10, &GlobalPositionPlugin::set_gp_origin_cb, this);

        // offset from local position to the global origin ("earth")
        // 局部定位与全球定位原点之间的偏移
        gp_global_offset_pub = gp_nh.advertise<geometry_msgs::PoseStamped>("gp_lp_offset", 10);
    }

    Subscriptions get_subscriptions() override {
        return {
            make_handler(&GlobalPositionPlugin::handle_gps_raw_int),
            make_handler(&GlobalPositionPlugin::handle_global_position_int),
            make_handler(&GlobalPositionPlugin::handle_gps_global_origin),
            make_handler(&GlobalPositionPlugin::handle_lpned_system_global_offset),
            make_handler(&GlobalPositionPlugin::handle_home_position),
        };
    }

private:
    ros::NodeHandle gp_nh;

    ros::Publisher raw_fix_pub;
    ros::Publisher raw_vel_pub;
    ros::Publisher raw_sat_pub;
    ros::Publisher gp_odom_pub;
    ros::Publisher gp_fix_pub;
    ros::Publisher gp_hdg_pub;
    ros::Publisher gp_rel_alt_pub;
    ros::Publisher gp_global_origin_pub;
    ros::Publisher gp_global_offset_pub;

    ros::Subscriber gp_set_global_origin_sub;

    std::string frame_id;
    std::string child_frame_id;
    std::string global_frame_id;
    std::string tf_frame_id;
    std::string tf_global_frame_id;
    std::string tf_child_frame_id;

    bool tf_send;
    bool use_relative_alt;
    bool outdoor_switch;
    bool is_map_init;

    double rot_cov;
    double gps_uere;

    Eigen::Vector3d map_origin{};  // geodetic origin of map frame [lla]
                                   // map坐标系原点的纬经高
    Eigen::Vector3d ecef_origin{}; // geocentric origin of map frame [m]
                                   // map坐标系原点的地心地固坐标系坐标
    Eigen::Vector3d local_ecef{};  // local ECEF coordinates on map frame [m]
                                   // 地心地固坐标系转换到map坐标系的坐标

    /* mid-level functions */
    /* 中间件函数 */

    template <typename MsgT> inline void fill_lla(MsgT &msg, sensor_msgs::NavSatFix::Ptr fix) {
        fix->latitude  = msg.lat / 1E7;                                         // deg 度
        fix->longitude = msg.lon / 1E7;                                         // deg 度
        fix->altitude  = msg.alt / 1E3 + m_uas->geoid_to_ellipsoid_height(fix); // meter 米
    }

    inline void fill_unknown_cov(sensor_msgs::NavSatFix::Ptr fix) {
        fix->position_covariance.fill(0.0);
        fix->position_covariance[0]   = -1.0;
        fix->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }

    /* message handlers */
    /* 信息回调句柄 */

    void handle_gps_raw_int(const mavlink::mavlink_message_t  *msg,
                            mavlink::common::msg::GPS_RAW_INT &raw_gps) {
        auto fix = boost::make_shared<sensor_msgs::NavSatFix>();

        fix->header = m_uas->synchronized_header(child_frame_id, raw_gps.time_usec);

        fix->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
        if (raw_gps.fix_type > 2) {
            fix->status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
            outdoor_switch     = true;
        } else {
            if (outdoor_switch) {
                ROS_WARN_NAMED("global_position", "GP: No GPS fix");
                outdoor_switch = false;
            }
            fix->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        }

        fill_lla(raw_gps, fix);

        float eph = (raw_gps.eph != UINT16_MAX) ? raw_gps.eph / 1E2F : NAN;
        float epv = (raw_gps.epv != UINT16_MAX) ? raw_gps.epv / 1E2F : NAN;

        ftf::EigenMapCovariance3d gps_cov(fix->position_covariance.data());

        // With mavlink v2.0 use accuracies reported by sensor
        // mavlink v2.0使用传感器本身输出的精度
        if (msg->magic == MAVLINK_STX && raw_gps.h_acc > 0 && raw_gps.v_acc > 0) {
            gps_cov.diagonal() << std::pow(raw_gps.h_acc / 1E3, 2),
                std::pow(raw_gps.h_acc / 1E3, 2), std::pow(raw_gps.v_acc / 1E3, 2);
            fix->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        }
        // With mavlink v1.0 approximate accuracies by DOP
        // mavlink v1.0使用由DOP(精度因子)计算的大致精度
        else if (!std::isnan(eph) && !std::isnan(epv)) {
            gps_cov.diagonal() << std::pow(eph * gps_uere, 2), std::pow(eph * gps_uere, 2),
                std::pow(epv * gps_uere, 2);
            fix->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        } else {
            fill_unknown_cov(fix);
        }

        // store & publish
        // 保存并发布
        m_uas->update_gps_fix_epts(fix, eph, epv, raw_gps.fix_type, raw_gps.satellites_visible);
        raw_fix_pub.publish(fix);

        if (raw_gps.vel != UINT16_MAX && raw_gps.cog != UINT16_MAX) {
            double speed  = raw_gps.vel / 1E2;                       // m/s
            double course = angles::from_degrees(raw_gps.cog / 1E2); // rad

            auto vel = boost::make_shared<geometry_msgs::TwistStamped>();

            vel->header.stamp    = fix->header.stamp;
            vel->header.frame_id = frame_id;

            vel->twist.linear.x = speed * std::sin(course);
            vel->twist.linear.y = speed * std::cos(course);

            raw_vel_pub.publish(vel);
        }

        // publish satellite count
        // 打印可见星数
        auto sat_cnt  = boost::make_shared<std_msgs::UInt32>();
        sat_cnt->data = raw_gps.satellites_visible;
        raw_sat_pub.publish(sat_cnt);
    }

    void handle_gps_global_origin(const mavlink::mavlink_message_t        *msg,
                                  mavlink::common::msg::GPS_GLOBAL_ORIGIN &glob_orig) {
        auto g_origin = boost::make_shared<geographic_msgs::GeoPointStamped>();
        // requires Mavlink msg update
        // 需要飞控mavlink更新才能使用
        // auto header = m_uas->synchronized_header(global_frame_id, glob_orig.time_boot_ms);

        g_origin->header.frame_id = global_frame_id;
        g_origin->header.stamp    = ros::Time::now();

        g_origin->position.latitude  = glob_orig.latitude / 1E7;
        g_origin->position.longitude = glob_orig.longitude / 1E7;
        g_origin->position.altitude =
            glob_orig.altitude / 1E3 +
            m_uas->geoid_to_ellipsoid_height(
                &g_origin->position); // convert height amsl to height above the ellipsoid
                                      // 平均海拔转换为椭球高

        try {
            /**
             * @brief Conversion from geodetic coordinates (LLA) to ECEF (Earth-Centered,
             * Earth-Fixed) Note: "earth" frame, in ECEF, of the global origin
             * @brief 纬经高转换为地心地固坐标系下的坐标，earth原点是在地心地固坐标系下的原点
             */
            GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(),
                                            GeographicLib::Constants::WGS84_f());

            earth.Forward(g_origin->position.latitude, g_origin->position.longitude,
                          g_origin->position.altitude, g_origin->position.latitude,
                          g_origin->position.longitude, g_origin->position.altitude);

            gp_global_origin_pub.publish(g_origin);
        } catch (const std::exception &e) {
            ROS_INFO_STREAM("GP: Caught exception: " << e.what() << std::endl);
        }
    }

    // TODO: Handler for GLOBAL_POSITION_INT_COV
    // TODO: 加入GLOBAL_POSITION_INT_COV的回调函数，引入协方差

    void handle_global_position_int(const mavlink::mavlink_message_t          *msg,
                                    mavlink::common::msg::GLOBAL_POSITION_INT &gpos) {
        auto odom            = boost::make_shared<nav_msgs::Odometry>();
        auto fix             = boost::make_shared<sensor_msgs::NavSatFix>();
        auto relative_alt    = boost::make_shared<std_msgs::Float64>();
        auto compass_heading = boost::make_shared<std_msgs::Float64>();

        auto header = m_uas->synchronized_header(child_frame_id, gpos.time_boot_ms);

        // Global position fix
        // 全球定位
        fix->header = header;

        fill_lla(gpos, fix);

        // fill GPS status fields using GPS_RAW data
        // 利用GPS_RAW数据构建GPS状态栏
        auto raw_fix = m_uas->get_gps_fix();
        if (raw_fix) {
            fix->status.service           = raw_fix->status.service;
            fix->status.status            = raw_fix->status.status;
            fix->position_covariance      = raw_fix->position_covariance;
            fix->position_covariance_type = raw_fix->position_covariance_type;
        } else {
            // no GPS_RAW_INT -> fix status unknown
            // 无GPS_RAW_INT -> 定位状态未知
            fix->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
            fix->status.status  = sensor_msgs::NavSatStatus::STATUS_NO_FIX;

            // we don't know covariance
            // 不知道协方差，使用缺省值
            fill_unknown_cov(fix);
        }

        relative_alt->data    = gpos.relative_alt / 1E3;                         // in meters
        compass_heading->data = (gpos.hdg != UINT16_MAX) ? gpos.hdg / 1E2 : NAN; // in degrees

        /**
         * @brief 全球定位里程计:
         *
         * X: 球坐标x轴(米)
         * Y: 球坐标Y轴(米)
         * Z: 球坐标Z轴(米)
         * VX: 纬速度 (m/s)
         * VY: 经速度 (m/s)
         * VZ: 高速度 (m/s)
         * Angular rates: 未知
         * Pose covariance: 以固定对角矩阵加入计算
         * Velocity covariance: 未知
         */
        odom->header.stamp    = header.stamp;
        odom->header.frame_id = frame_id;
        odom->child_frame_id  = child_frame_id;

        // Linear velocity
        // 线速度
        tf::vectorEigenToMsg(Eigen::Vector3d(gpos.vy, gpos.vx, gpos.vz) / 1E2,
                             odom->twist.twist.linear);

        // Velocity covariance unknown
        // 速度协方差未知
        ftf::EigenMapCovariance6d vel_cov_out(odom->twist.covariance.data());
        vel_cov_out.fill(0.0);
        vel_cov_out(0) = -1.0;

        // Current fix in ECEF
        // 当前定位通过地心地固坐标系表示
        Eigen::Vector3d map_point;

        try {
            /**
             * @brief Conversion from geodetic coordinates (LLA) to ECEF (Earth-Centered,
             * Earth-Fixed)
             *
             * Note: "ecef_origin" is the origin of "map" frame, in ECEF, and the local coordinates
             * are in spherical coordinates, with the orientation in ENU (just like what is applied
             * on Gazebo)
             *
             * @brief 纬经高转换为地心地固坐标系下的坐标
             *
             * 注意：ecef_origin是map坐标系原点在地心地固坐标系下的坐标，然而局部坐标是球坐标，旋转是相对ENU坐标系
             * 表示的，与Gazebo一致
             */
            GeographicLib::Geocentric map(GeographicLib::Constants::WGS84_a(),
                                          GeographicLib::Constants::WGS84_f());

            /**
             * @brief Checks if the "map" origin is set.
             * - If not, and the home position is also not received, it sets the current fix as the
             * origin;
             * - If the home position is received, it sets the "map" origin;
             * - If the "map" origin is set, then it applies the rotations to the offset between the
             * origin and the current local geocentric coordinates.
             *
             * @brief 检查map原点是否被设置
             * - 如果没有，且返航点位置也未收到，则设置当前位置为原点
             * - 如果返航点已知，则设置返航点为map原点
             * - 如果map原点已被设置，则将该原点和当前局部地心坐标之间的偏移进行旋转
             */
            // Current fix to ECEF
            // 以地心地固坐标系表示的当前定位
            map.Forward(fix->latitude, fix->longitude, fix->altitude, map_point.x(), map_point.y(),
                        map_point.z());

            // Set the current fix as the "map" origin if it's not set
            // 设置当前位置为map原点
            if (!is_map_init && fix->status.status >= sensor_msgs::NavSatStatus::STATUS_FIX) {
                map_origin.x() = fix->latitude;
                map_origin.y() = fix->longitude;
                map_origin.z() = fix->altitude;

                ecef_origin = map_point; // Local position is zero
                                         // 局部位置为0
                is_map_init = true;
            }
        } catch (const std::exception &e) {
            ROS_INFO_STREAM("GP: Caught exception: " << e.what() << std::endl);
        }

        // Compute the local coordinates
        // 计算局部坐标(球坐标系)
        local_ecef = map_point - ecef_origin;
        // Compute the local coordinates in ENU
        // 计算ENU坐标系下的局部坐标
        tf::pointEigenToMsg(ftf::transform_frame_ecef_enu(local_ecef, map_origin),
                            odom->pose.pose.position);

        /**
         * @brief By default, we are using the relative altitude instead of the geocentric
         * altitude, which is relative to the WGS-84 ellipsoid
         * @brief 通常以相对高度代替地心高度，相对高度与WGS-84椭球模型有关
         */
        if (use_relative_alt)
            odom->pose.pose.position.z = relative_alt->data;

        odom->pose.pose.orientation = m_uas->get_attitude_orientation_enu();

        // Use ENU covariance to build XYZRPY covariance
        // 使用ENU系下的协方差构建位置与姿态的协方差
        ftf::EigenMapConstCovariance3d gps_cov(fix->position_covariance.data());
        ftf::EigenMapCovariance6d      pos_cov_out(odom->pose.covariance.data());
        pos_cov_out.setZero();
        pos_cov_out.block<3, 3>(0, 0) = gps_cov;
        pos_cov_out.block<3, 3>(3, 3).diagonal() << rot_cov, rot_cov, rot_cov;

        // publish
        // 发布ROS消息
        gp_fix_pub.publish(fix);
        gp_odom_pub.publish(odom);
        gp_rel_alt_pub.publish(relative_alt);
        gp_hdg_pub.publish(compass_heading);

        // TF
        // 发布TF
        if (tf_send) {
            geometry_msgs::TransformStamped transform;

            transform.header.stamp    = odom->header.stamp;
            transform.header.frame_id = tf_frame_id;
            transform.child_frame_id  = tf_child_frame_id;

            // set rotation
            // 设置旋转
            transform.transform.rotation = odom->pose.pose.orientation;

            // set origin
            // 设置原点
            transform.transform.translation.x = odom->pose.pose.position.x;
            transform.transform.translation.y = odom->pose.pose.position.y;
            transform.transform.translation.z = odom->pose.pose.position.z;

            m_uas->tf2_broadcaster.sendTransform(transform);
        }
    }

    void handle_lpned_system_global_offset(
        const mavlink::mavlink_message_t                              *msg,
        mavlink::common::msg::LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET &offset) {
        auto global_offset    = boost::make_shared<geometry_msgs::PoseStamped>();
        global_offset->header = m_uas->synchronized_header(tf_global_frame_id, offset.time_boot_ms);

        auto enu_position =
            ftf::transform_frame_ned_enu(Eigen::Vector3d(offset.x, offset.y, offset.z));
        auto enu_baselink_orientation =
            ftf::transform_orientation_aircraft_baselink(ftf::transform_orientation_ned_enu(
                ftf::quaternion_from_rpy(offset.roll, offset.pitch, offset.yaw)));

        tf::pointEigenToMsg(enu_position, global_offset->pose.position);
        tf::quaternionEigenToMsg(enu_baselink_orientation, global_offset->pose.orientation);

        gp_global_offset_pub.publish(global_offset);

        // TF
        // 发布TF
        if (tf_send) {
            geometry_msgs::TransformStamped transform;

            transform.header.stamp    = global_offset->header.stamp;
            transform.header.frame_id = tf_global_frame_id;
            transform.child_frame_id  = tf_frame_id;

            // set rotation
            // 设置旋转
            transform.transform.rotation = global_offset->pose.orientation;

            // set origin
            // 设置原点
            transform.transform.translation.x = global_offset->pose.position.x;
            transform.transform.translation.y = global_offset->pose.position.y;
            transform.transform.translation.z = global_offset->pose.position.z;

            m_uas->tf2_broadcaster.sendTransform(transform);
        }
    }

    // handle home position to set "map" origin
    // 获取返航点用于设置map原点
    // 注意：acfly的返航点是主动发送的，不需要请求，与px4不同
    void handle_home_position(const mavlink::mavlink_message_t    *msg,
                              mavlink::common::msg::HOME_POSITION &home_position) {
        auto hp        = boost::make_shared<mavros_msgs::HomePosition>();
        map_origin.x() = hp->geo.latitude = home_position.latitude / 1E7;   // deg 度
        map_origin.y() = hp->geo.longitude = home_position.longitude / 1E7; // deg 度
        map_origin.z() =
            home_position.altitude / 1E3 + m_uas->geoid_to_ellipsoid_height(&hp->geo); // meter 米

        try {
            /**
             * @brief Conversion from geodetic coordinates (LLA) to ECEF (Earth-Centered,
             * Earth-Fixed)
             * @brief 纬经高转换为地心地固坐标系下的坐标
             */
            GeographicLib::Geocentric map(GeographicLib::Constants::WGS84_a(),
                                          GeographicLib::Constants::WGS84_f());

            // map_origin to ECEF
            // map坐标系原点的地心地固坐标系坐标
            map.Forward(map_origin.x(), map_origin.y(), map_origin.z(), ecef_origin.x(),
                        ecef_origin.y(), ecef_origin.z());
        } catch (const std::exception &e) {
            ROS_INFO_STREAM("GP: Caught exception: " << e.what() << std::endl);
        }

        is_map_init = true;
    }

    /* diagnostics */
    /* 诊断 */
    void gps_diag_run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
        int   fix_type, satellites_visible;
        float eph, epv;

        m_uas->get_gps_epts(eph, epv, fix_type, satellites_visible);

        if (satellites_visible <= 0)
            stat.summary(1, "No satellites");
        else if (fix_type < 2)
            stat.summary(1, "No fix");
        else if (fix_type == 2)
            stat.summary(0, "2D fix");
        else if (fix_type >= 3)
            stat.summary(0, "3D fix");

        stat.addf("Satellites visible", "%zd", satellites_visible);
        stat.addf("Fix type", "%d", fix_type);

        if (!std::isnan(eph))
            stat.addf("EPH (m)", "%.2f", eph);
        else
            stat.add("EPH (m)", "Unknown");

        if (!std::isnan(epv))
            stat.addf("EPV (m)", "%.2f", epv);
        else
            stat.add("EPV (m)", "Unknown");
    }

    /* ros callbacks */
    /* ROS回调函数 */

    void set_gp_origin_cb(const geographic_msgs::GeoPointStamped::ConstPtr &req) {
        mavlink::common::msg::SET_GPS_GLOBAL_ORIGIN gpo = {};

        Eigen::Vector3d global_position;

        gpo.target_system = m_uas->get_tgt_system();
        // TODO: requires Mavlink msg update
        // TODO: 需要飞控mavlink更新才能使用(感觉这个时间没必要？)
        // gpo.time_boot_ms = stamp.toNSec() / 1000;

        gpo.latitude  = req->position.latitude * 1E7;
        gpo.longitude = req->position.longitude * 1E7;
        gpo.altitude =
            (req->position.altitude + m_uas->ellipsoid_to_geoid_height(&req->position)) * 1E3;

        UAS_FCU(m_uas)->send_message_ignore_drop(gpo);
    }
};
} // namespace std_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::GlobalPositionPlugin, mavros::plugin::PluginBase)
