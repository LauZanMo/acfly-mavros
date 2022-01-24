/**
 * @file mavros.h
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @brief This file is from mavros open source respository, thanks for their contribution.
 * @version 1.0
 * @date 2022-01-25
 *
 * @copyright Copyright (c) 2022 acfly
 * @copyright Copyright 2014,2015,2016,2017 Vladimir Ermakov.
 * For commercial use, please contact acfly: https://www.acfly.cn
 *
 */

#pragma once

#include <array>
#include <mavconn/interface.h>
#include <mavros/mavlink_diag.h>
#include <mavros/mavros_plugin.h>
#include <mavros/utils.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

namespace mavros {
/**
 * @brief MAVROS node class
 *
 * This class implement mavros_node
 */
class MavRos {
public:
    MavRos();
    ~MavRos(){};

    void spin();

private:
    ros::NodeHandle mavlink_nh;
    // fcu_link stored in mav_uas
    mavconn::MAVConnInterface::Ptr gcs_link;
    bool                           gcs_quiet_mode;
    ros::Time                      last_message_received_from_gcs;
    ros::Duration                  conn_timeout;

    ros::Publisher  mavlink_pub;
    ros::Subscriber mavlink_sub;

    diagnostic_updater::Updater gcs_diag_updater;
    MavlinkDiag                 fcu_link_diag;
    MavlinkDiag                 gcs_link_diag;

    pluginlib::ClassLoader<plugin::PluginBase> plugin_loader;
    std::vector<plugin::PluginBase::Ptr>       loaded_plugins;

    //! FCU link -> router -> plugin handler
    std::unordered_map<mavlink::msgid_t, plugin::PluginBase::Subscriptions> plugin_subscriptions;

    //! UAS object passed to all plugins
    UAS mav_uas;

    //! fcu link -> ros
    void mavlink_pub_cb(const mavlink::mavlink_message_t *mmsg, const mavconn::Framing framing);
    //! ros -> fcu link
    void mavlink_sub_cb(const mavros_msgs::Mavlink::ConstPtr &rmsg);

    //! message router
    void plugin_route_cb(const mavlink::mavlink_message_t *mmsg, const mavconn::Framing framing);

    //! load plugin
    void add_plugin(std::string &pl_name, ros::V_string &blacklist, ros::V_string &whitelist);

    //! start mavlink app on USB
    void startup_px4_usb_quirk();
    void log_connect_change(bool connected);
};
} // namespace mavros
