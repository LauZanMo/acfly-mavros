/**
 * @file imu.cpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @brief This file is from mavros open source respository, thanks for their contribution.
 * @version 1.0
 * @date 2022-01-24
 *
 * @copyright Copyright (c) 2022 acfly
 * @copyright Copyright 2014,2015,2016,2017 Vladimir Ermakov.
 * For commercial use, please contact acfly: https://www.acfly.cn
 *
 */

#include <cmath>
#include <eigen_conversions/eigen_msg.h>
#include <mavros/mavros_plugin.h>

#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>

namespace mavros {
namespace std_plugins {
// 单位转换比例系数
// Gauss to Tesla coeff
static constexpr double GAUSS_TO_TESLA = 1.0e-4;
// millTesla to Tesla coeff
static constexpr double MILLIT_TO_TESLA = 1000.0;
// millRad/Sec to Rad/Sec coeff
static constexpr double MILLIRS_TO_RADSEC = 1.0e-3;
// millG to m/s**2 coeff
static constexpr double MILLIG_TO_MS2 = 9.80665 / 1000.0;
// millm/s**2 to m/s**2 coeff
static constexpr double MILLIMS2_TO_MS2 = 1.0e-3;
// millBar to Pascal coeff
static constexpr double MILLIBAR_TO_PASCAL = 1.0e2;
// Radians to degrees
static constexpr double RAD_TO_DEG = 180.0 / M_PI;

/**
 * @brief IMU and attitude data publication plugin
 * @brief IMU和角度数据发布ROS插件
 * @note 该插件会将IMU，气压计，磁力计和角度数据进行存储，供其他组件使用，并以ROS信息发布
 * @warning 该插件依赖于sys_time插件提供time_offset用于消除飞控与机载电脑的时钟偏移
 */
class IMUPlugin : public plugin::PluginBase {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IMUPlugin()
        : PluginBase(), imu_nh("~imu"), has_hr_imu(false), has_raw_imu(false),
          has_scaled_imu(false), has_att_quat(false), received_linear_accel(false),
          linear_accel_vec_flu(Eigen::Vector3d::Zero()),
          linear_accel_vec_frd(Eigen::Vector3d::Zero()) {}

    void initialize(UAS &uas_) override {
        PluginBase::initialize(uas_);

        double linear_stdev, angular_stdev, orientation_stdev, mag_stdev;

        /**
         * @warning A rotation from the aircraft-frame to the base_link frame is applied.
         * Additionally, it is reported the orientation of the vehicle to describe the
         * transformation from the ENU frame to the base_link frame (ENU <-> base_link).
         * THIS ORIENTATION IS NOT THE SAME AS THAT REPORTED BY THE FCU (NED <-> aircraft).
         *
         * @warning 从aircraft坐标系到base_link坐标系的旋转已被启用，另外，载具的旋转描述的是从东北
         * 天坐标系到前左上坐标系的变换(ENU <->base_link)，该旋转与飞控发送的信息不一致，飞控发的
         * mavlink信息是从北东地坐标系到前右下坐标系的变换(NED <-> aircraft)
         */
        imu_nh.param<std::string>("base_frame_flu_id", base_frame_flu_id, "ac_base_flu");
        imu_nh.param<std::string>("base_frame_frd_id", base_frame_frd_id, "ac_base_frd");
        imu_nh.param("linear_acceleration_stdev", linear_stdev,
                     0.0003); // MPU6000默认参数
        imu_nh.param("angular_velocity_stdev", angular_stdev,
                     0.02 * (M_PI / 180.0)); // MPU6000默认参数
        imu_nh.param("orientation_stdev", orientation_stdev, 1.0);
        imu_nh.param("magnetic_stdev", mag_stdev, 0.0);

        setup_covariance(linear_acceleration_cov, linear_stdev);
        setup_covariance(angular_velocity_cov, angular_stdev);
        setup_covariance(orientation_cov, orientation_stdev);
        setup_covariance(magnetic_cov, mag_stdev);
        setup_covariance(unk_orientation_cov, 0.0);

        imu_pub          = imu_nh.advertise<sensor_msgs::Imu>("data", 10);
        magn_pub         = imu_nh.advertise<sensor_msgs::MagneticField>("mag", 10);
        temp_imu_pub     = imu_nh.advertise<sensor_msgs::Temperature>("temperature_imu", 10);
        temp_baro_pub    = imu_nh.advertise<sensor_msgs::Temperature>("temperature_baro", 10);
        static_press_pub = imu_nh.advertise<sensor_msgs::FluidPressure>("static_pressure", 10);
        diff_press_pub   = imu_nh.advertise<sensor_msgs::FluidPressure>("diff_pressure", 10);
        imu_raw_pub      = imu_nh.advertise<sensor_msgs::Imu>("data_raw", 10);

        // Reset flags on connection change
        // 状态变化后重置标志
        enable_connection_cb();
    }

    Subscriptions get_subscriptions() override {
        return {
            make_handler(&IMUPlugin::handle_attitude),
            make_handler(&IMUPlugin::handle_attitude_quaternion),
            make_handler(&IMUPlugin::handle_highres_imu),
            make_handler(&IMUPlugin::handle_raw_imu),
            make_handler(&IMUPlugin::handle_scaled_imu),
            make_handler(&IMUPlugin::handle_scaled_pressure),
        };
    }

private:
    ros::NodeHandle imu_nh;
    std::string     base_frame_flu_id, base_frame_frd_id;

    ros::Publisher imu_pub;
    ros::Publisher imu_raw_pub;
    ros::Publisher magn_pub;
    ros::Publisher temp_imu_pub;
    ros::Publisher temp_baro_pub;
    ros::Publisher static_press_pub;
    ros::Publisher diff_press_pub;

    bool              has_hr_imu;
    bool              has_raw_imu;
    bool              has_scaled_imu;
    bool              has_att_quat;
    bool              received_linear_accel;
    Eigen::Vector3d   linear_accel_vec_flu;
    Eigen::Vector3d   linear_accel_vec_frd;
    ftf::Covariance3d linear_acceleration_cov;
    ftf::Covariance3d angular_velocity_cov;
    ftf::Covariance3d orientation_cov;
    ftf::Covariance3d unk_orientation_cov;
    ftf::Covariance3d magnetic_cov;

    /* mid-level functions */
    /* 中间件函数 */

    /**
     * @brief 初始化3*3协方差矩阵
     * @param cov		协方差矩阵
     * @param stdev		标准差
     * @remarks		    由协方差计算对角矩阵
     */
    void setup_covariance(ftf::Covariance3d &cov, double stdev) {
        ftf::EigenMapCovariance3d c(cov.data());

        c.setZero();
        if (stdev) {
            double sr = stdev * stdev;
            c.diagonal() << sr, sr, sr;
        } else {
            c(0, 0) = -1.0;
        }
    }

    /**
     * @brief 构建并发布IMU data信息
     * @param time_boot_ms     消息时间戳(未同步)
     * @param orientation_enu  从ENU到FLU的旋转
     * @param orientation_ned  从NED到FRD的旋转
     * @param gyro_flu         FLU坐标系下的角速度
     * @param gyro_frd         FRD坐标系下的角速度
     */
    void publish_imu_data(uint32_t time_boot_ms, Eigen::Quaterniond &orientation_enu,
                          Eigen::Quaterniond &orientation_ned, Eigen::Vector3d &gyro_flu,
                          Eigen::Vector3d &gyro_frd) {
        auto imu_ned_msg = boost::make_shared<sensor_msgs::Imu>();
        auto imu_enu_msg = boost::make_shared<sensor_msgs::Imu>();

        // Fill message header
        // 填充信息标头
        imu_enu_msg->header = m_uas->synchronized_header(base_frame_flu_id, time_boot_ms);
        imu_ned_msg->header = m_uas->synchronized_header(base_frame_frd_id, time_boot_ms);

        // Convert from Eigen::Quaternond to geometry_msgs::Quaternion
        // 将信息从Eigen::Quaternond格式转化为to geometry_msgs::Quaternion格式
        tf::quaternionEigenToMsg(orientation_enu, imu_enu_msg->orientation);
        tf::quaternionEigenToMsg(orientation_ned, imu_ned_msg->orientation);

        // Convert from Eigen::Vector3d to geometry_msgs::Vector3
        // 将信息从Eigen::Vector3d格式转化为geometry_msgs::Vector3格式
        tf::vectorEigenToMsg(gyro_flu, imu_enu_msg->angular_velocity);
        tf::vectorEigenToMsg(gyro_frd, imu_ned_msg->angular_velocity);

        // Eigen::Vector3d from HIGHRES_IMU or RAW_IMU, to geometry_msgs::Vector3
        // 将信息从Eigen::Vector3d格式转化为geometry_msgs::Vector3格式
        tf::vectorEigenToMsg(linear_accel_vec_flu, imu_enu_msg->linear_acceleration);
        tf::vectorEigenToMsg(linear_accel_vec_frd, imu_ned_msg->linear_acceleration);

        // Pass ENU msg covariances
        // 传递ENU信息的协方差
        imu_enu_msg->orientation_covariance         = orientation_cov;
        imu_enu_msg->angular_velocity_covariance    = angular_velocity_cov;
        imu_enu_msg->linear_acceleration_covariance = linear_acceleration_cov;

        // Pass NED msg covariances
        // 传递NED信息的协方差
        imu_ned_msg->orientation_covariance         = orientation_cov;
        imu_ned_msg->angular_velocity_covariance    = angular_velocity_cov;
        imu_ned_msg->linear_acceleration_covariance = linear_acceleration_cov;

        if (!received_linear_accel) {
            // Set element 0 of covariance matrix to -1 if no data received as per sensor_msgs/Imu
            // defintion
            // 根据sensor_msgs/Imu定义,如果没有收到数据，则将协方差矩阵的元素0设置为-1
            imu_enu_msg->linear_acceleration_covariance[0] = -1;
            imu_ned_msg->linear_acceleration_covariance[0] = -1;
        }

        /**
         * Store attitude in base_link ENU
         * 保存该角度
         * @snippet src/plugins/imu.cpp store_enu
         */
        m_uas->update_attitude_imu_enu(imu_enu_msg);

        /**
         * Store attitude in aircraft NED
         * 保存该角度
         * @snippet src/plugins/imu.cpp store_ned
         */
        m_uas->update_attitude_imu_ned(imu_ned_msg);

        /**
         * Publish only base_link ENU message
         * 仅发布ENU到FLU的imu信息
         * @snippet src/plugins/imu.cpp pub_enu
         */
        imu_pub.publish(imu_enu_msg);
    }

    /**
     * @brief 构建并发布IMU data_raw信息，保存imu数据的线加速度
     * @param header      信息坐标系id和时间戳
     * @param gyro_flu    FLU坐标系下的角速度
     * @param accel_flu   FLU坐标系下的线加速度
     * @param accel_frd   FRD坐标系下的线加速度
     */
    void publish_imu_data_raw(std_msgs::Header &header, Eigen::Vector3d &gyro_flu,
                              Eigen::Vector3d &accel_flu, Eigen::Vector3d &accel_frd) {
        auto imu_msg = boost::make_shared<sensor_msgs::Imu>();

        // Fill message header
        // 填充信息标头
        imu_msg->header = header;

        tf::vectorEigenToMsg(gyro_flu, imu_msg->angular_velocity);
        tf::vectorEigenToMsg(accel_flu, imu_msg->linear_acceleration);

        // Save readings
        // 保存信息
        linear_accel_vec_flu  = accel_flu;
        linear_accel_vec_frd  = accel_frd;
        received_linear_accel = true;

        imu_msg->orientation_covariance         = unk_orientation_cov;
        imu_msg->angular_velocity_covariance    = angular_velocity_cov;
        imu_msg->linear_acceleration_covariance = linear_acceleration_cov;

        // Publish message [FLU frame]
        // 发布FLU坐标系下的imu_raw信息
        imu_raw_pub.publish(imu_msg);
    }

    /**
     * @brief 发布磁场信息
     * @param header	Message frame_id and timestamp
     * @param mag_field	Magnetic field in the base_link ENU frame
     */
    void publish_mag(std_msgs::Header &header, Eigen::Vector3d &mag_field) {
        auto magn_msg = boost::make_shared<sensor_msgs::MagneticField>();

        // Fill message header
        // 填充信息标头
        magn_msg->header = header;

        tf::vectorEigenToMsg(mag_field, magn_msg->magnetic_field);
        magn_msg->magnetic_field_covariance = magnetic_cov;

        // Publish message [ENU frame]
        // 发布ENU坐标系下的磁场信息
        magn_pub.publish(magn_msg);
    }

    /* message handlers */
    /* 信息回调句柄 */

    void handle_attitude(const mavlink::mavlink_message_t *msg,
                         mavlink::common::msg::ATTITUDE   &att) {
        if (has_att_quat)
            return;

        /**
         * Orientation on the NED-aicraft frame:
         * 从NED坐标系到FRD坐标系的旋转
         * @snippet src/plugins/imu.cpp ned_aircraft_orient1
         */
        auto ned_aircraft_orientation = ftf::quaternion_from_rpy(att.roll, att.pitch, att.yaw);

        /**
         * Angular velocity on the NED-aicraft frame:
         * 从NED坐标系到FRD坐标系的角速度
         * @snippet src/plugins/imu.cpp ned_ang_vel1
         */
        auto gyro_frd = Eigen::Vector3d(att.rollspeed, att.pitchspeed, att.yawspeed);

        /**
         * The RPY describes the rotation: aircraft->NED.
         * It is required to change this to aircraft->base_link:
         * 上传的欧拉角描述的旋转是：FRD->NED
         * 需要转换为FLU坐标系，即描述：FLU->ENU
         * @snippet src/plugins/imu.cpp ned->baselink->enu
         */
        auto enu_baselink_orientation = ftf::transform_orientation_aircraft_baselink(
            ftf::transform_orientation_ned_enu(ned_aircraft_orientation));

        /**
         * The angular velocity expressed in the aircraft frame.
         * It is required to apply the static rotation to get it into the base_link frame:
         * 上传的角速度是在FRD坐标系下的
         * 将其做一个常值旋转转换到FLU坐标系
         * @snippet src/plugins/imu.cpp rotate_gyro
         */
        auto gyro_flu = ftf::transform_frame_aircraft_baselink(gyro_frd);

        publish_imu_data(att.time_boot_ms, enu_baselink_orientation, ned_aircraft_orientation,
                         gyro_flu, gyro_frd);
    }

    void handle_attitude_quaternion(const mavlink::mavlink_message_t          *msg,
                                    mavlink::common::msg::ATTITUDE_QUATERNION &att_q) {
        ROS_INFO_COND_NAMED(!has_att_quat, "imu", "IMU: Attitude quaternion IMU detected!");
        has_att_quat = true;

        /**
         * Orientation on the NED-aicraft frame:
         * 从NED坐标系到FRD坐标系的旋转
         * @snippet src/plugins/imu.cpp ned_aircraft_orient2
         */
        auto ned_aircraft_orientation = Eigen::Quaterniond(att_q.q1, att_q.q2, att_q.q3, att_q.q4);

        /**
         * Angular velocity on the NED-aicraft frame:
         * 从NED坐标系到FRD坐标系的角速度
         * @snippet src/plugins/imu.cpp ned_ang_vel2
         */
        auto gyro_frd = Eigen::Vector3d(att_q.rollspeed, att_q.pitchspeed, att_q.yawspeed);

        /**
         * MAVLink quaternion exactly matches Eigen convention.
         * The RPY describes the rotation: aircraft->NED.
         * It is required to change this to aircraft->base_link:
         * mavlink四元数与Eigen完全一致
         * 上传的欧拉角描述的旋转是：FRD->NED
         * 需要转换为FLU坐标系，即描述：FLU->ENU
         * @snippet src/plugins/imu.cpp ned->baselink->enu
         */
        auto enu_baselink_orientation = ftf::transform_orientation_aircraft_baselink(
            ftf::transform_orientation_ned_enu(ned_aircraft_orientation));

        /**
         * The angular velocity expressed in the aircraft frame.
         * It is required to apply the static rotation to get it into the base_link frame:
         * 上传的角速度是在FRD坐标系下的
         * 将其做一个常值旋转转换到FLU坐标系
         * @snippet src/plugins/imu.cpp rotate_gyro
         */
        auto gyro_flu = ftf::transform_frame_aircraft_baselink(gyro_frd);

        publish_imu_data(att_q.time_boot_ms, enu_baselink_orientation, ned_aircraft_orientation,
                         gyro_flu, gyro_frd);
    }

    void handle_highres_imu(const mavlink::mavlink_message_t  *msg,
                            mavlink::common::msg::HIGHRES_IMU &imu_hr) {
        ROS_INFO_COND_NAMED(!has_hr_imu, "imu", "IMU: High resolution IMU detected!");
        has_hr_imu = true;

        auto header = m_uas->synchronized_header(base_frame_flu_id, imu_hr.time_usec);
        /**
         * @todo Make more paranoic check of HIGHRES_IMU.fields_updated
         * @todo 对HIGHRES_IMU.fields_updated进行更细致的检查
         */

        /**
         * Check if accelerometer + gyroscope data are available.
         * Data is expressed in aircraft frame it is required to rotate to the base_link frame:
         * 检查加速度计和陀螺数据是否可用
         * 数据以FRD坐标系表示因此需要转化为FLU坐标系
         * @snippet src/plugins/imu.cpp accel_available
         */
        if (imu_hr.fields_updated & ((7 << 3) | (7 << 0))) {
            auto gyro_flu = ftf::transform_frame_aircraft_baselink(
                Eigen::Vector3d(imu_hr.xgyro, imu_hr.ygyro, imu_hr.zgyro));

            auto accel_frd = Eigen::Vector3d(imu_hr.xacc, imu_hr.yacc, imu_hr.zacc);
            auto accel_flu = ftf::transform_frame_aircraft_baselink(accel_frd);

            publish_imu_data_raw(header, gyro_flu, accel_flu, accel_frd);
        }

        /**
         * Check if magnetometer data is available:
         * 检查磁力计数据是否可用
         * @snippet src/plugins/imu.cpp mag_available
         */
        if (imu_hr.fields_updated & (7 << 6)) {
            auto mag_field = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(
                Eigen::Vector3d(imu_hr.xmag, imu_hr.ymag, imu_hr.zmag) * GAUSS_TO_TESLA);

            publish_mag(header, mag_field);
        }

        /**
         * Check if static pressure sensor data is available:
         * 检查气压计数据是否可用
         * @snippet src/plugins/imu.cpp static_pressure_available
         */
        if (imu_hr.fields_updated & (1 << 9)) {
            auto static_pressure_msg = boost::make_shared<sensor_msgs::FluidPressure>();

            static_pressure_msg->header         = header;
            static_pressure_msg->fluid_pressure = imu_hr.abs_pressure;

            static_press_pub.publish(static_pressure_msg);
        }

        /**
         * Check if differential pressure sensor data is available:
         * 检查差分气压计数据是否可用
         * @snippet src/plugins/imu.cpp differential_pressure_available
         */
        if (imu_hr.fields_updated & (1 << 10)) {
            auto differential_pressure_msg = boost::make_shared<sensor_msgs::FluidPressure>();

            differential_pressure_msg->header         = header;
            differential_pressure_msg->fluid_pressure = imu_hr.diff_pressure;

            diff_press_pub.publish(differential_pressure_msg);
        }

        /**
         * Check if temperature data is available:
         * 检查温度数据是否可用
         * @snippet src/plugins/imu.cpp temperature_available
         */
        if (imu_hr.fields_updated & (1 << 12)) {
            auto temp_msg = boost::make_shared<sensor_msgs::Temperature>();

            temp_msg->header      = header;
            temp_msg->temperature = imu_hr.temperature;

            temp_imu_pub.publish(temp_msg);
        }
    }

    void handle_raw_imu(const mavlink::mavlink_message_t *msg,
                        mavlink::common::msg::RAW_IMU    &imu_raw) {
        ROS_INFO_COND_NAMED(!has_raw_imu, "imu", "IMU: Raw IMU message used.");
        has_raw_imu = true;

        if (has_hr_imu || has_scaled_imu)
            return;

        auto imu_msg = boost::make_shared<sensor_msgs::Imu>();
        auto header  = m_uas->synchronized_header(base_frame_flu_id, imu_raw.time_usec);

        auto gyro_flu = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(
            Eigen::Vector3d(imu_raw.xgyro, imu_raw.ygyro, imu_raw.zgyro) * MILLIRS_TO_RADSEC);
        auto accel_frd = Eigen::Vector3d(imu_raw.xacc, imu_raw.yacc, imu_raw.zacc);
        auto accel_flu = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(accel_frd);

        if (m_uas->is_ardupilotmega()) {
            accel_frd *= MILLIG_TO_MS2;
            accel_flu *= MILLIG_TO_MS2;
        } else if (m_uas->is_px4()) {
            accel_frd *= MILLIMS2_TO_MS2;
            accel_flu *= MILLIMS2_TO_MS2;
        }

        publish_imu_data_raw(header, gyro_flu, accel_flu, accel_frd);

        if (!m_uas->is_ardupilotmega()) {
            ROS_WARN_THROTTLE_NAMED(60, "imu",
                                    "IMU: linear acceleration on RAW_IMU known on APM only.");
            ROS_WARN_THROTTLE_NAMED(60, "imu",
                                    "IMU: ~imu/data_raw stores unscaled raw acceleration report.");
            linear_accel_vec_flu.setZero();
            linear_accel_vec_frd.setZero();
        }

        /**
         * Magnetic field data:
         * 磁场数据
         * @snippet src/plugins/imu.cpp mag_field
         */
        auto mag_field = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(
            Eigen::Vector3d(imu_raw.xmag, imu_raw.ymag, imu_raw.zmag) * MILLIT_TO_TESLA);

        publish_mag(header, mag_field);
    }

    void handle_scaled_imu(const mavlink::mavlink_message_t *msg,
                           mavlink::common::msg::SCALED_IMU &imu_raw) {
        if (has_hr_imu)
            return;

        ROS_INFO_COND_NAMED(!has_scaled_imu, "imu", "IMU: Scaled IMU message used.");
        has_scaled_imu = true;

        auto imu_msg = boost::make_shared<sensor_msgs::Imu>();
        auto header  = m_uas->synchronized_header(base_frame_flu_id, imu_raw.time_boot_ms);

        auto gyro_flu = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(
            Eigen::Vector3d(imu_raw.xgyro, imu_raw.ygyro, imu_raw.zgyro) * MILLIRS_TO_RADSEC);
        auto accel_frd = Eigen::Vector3d(Eigen::Vector3d(imu_raw.xacc, imu_raw.yacc, imu_raw.zacc) *
                                         MILLIG_TO_MS2);
        auto accel_flu = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(accel_frd);

        publish_imu_data_raw(header, gyro_flu, accel_flu, accel_frd);

        /**
         * Magnetic field data:
         * 磁场数据
         * @snippet src/plugins/imu.cpp mag_field
         */
        auto mag_field = ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(
            Eigen::Vector3d(imu_raw.xmag, imu_raw.ymag, imu_raw.zmag) * MILLIT_TO_TESLA);

        publish_mag(header, mag_field);
    }

    void handle_scaled_pressure(const mavlink::mavlink_message_t      *msg,
                                mavlink::common::msg::SCALED_PRESSURE &press) {
        if (has_hr_imu)
            return;

        auto header = m_uas->synchronized_header(base_frame_flu_id, press.time_boot_ms);

        auto temp_msg         = boost::make_shared<sensor_msgs::Temperature>();
        temp_msg->header      = header;
        temp_msg->temperature = press.temperature / 100.0;
        temp_baro_pub.publish(temp_msg);

        auto static_pressure_msg            = boost::make_shared<sensor_msgs::FluidPressure>();
        static_pressure_msg->header         = header;
        static_pressure_msg->fluid_pressure = press.press_abs * 100.0;
        static_press_pub.publish(static_pressure_msg);

        auto differential_pressure_msg    = boost::make_shared<sensor_msgs::FluidPressure>();
        differential_pressure_msg->header = header;
        differential_pressure_msg->fluid_pressure = press.press_diff * 100.0;
        diff_press_pub.publish(differential_pressure_msg);
    }

    // Checks for connection and overrides variable values
    // 检查连接并覆盖变量值
    void connection_cb(bool connected) override {
        has_hr_imu     = false;
        has_scaled_imu = false;
        has_att_quat   = false;
    }
};
} // namespace std_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::IMUPlugin, mavros::plugin::PluginBase)
