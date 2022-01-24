/**
 * @file uas_timesync.cpp
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

#include <array>
#include <mavros/mavros_uas.h>
#include <mavros/px4_custom_mode.h>
#include <mavros/utils.h>
#include <stdexcept>
#include <unordered_map>

using namespace mavros;

/* -*- time syncronise functions -*- */

static inline ros::Time ros_time_from_ns(const uint64_t stamp_ns) {
    return ros::Time(stamp_ns / 1000000000UL,  // t_sec
                     stamp_ns % 1000000000UL); // t_nsec
}

ros::Time UAS::synchronise_stamp(uint32_t time_boot_ms) {
    // copy offset from atomic var
    uint64_t offset_ns = time_offset;

    if (offset_ns > 0 || tsync_mode == timesync_mode::PASSTHROUGH) {
        uint64_t stamp_ns = static_cast<uint64_t>(time_boot_ms) * 1000000UL + offset_ns;
        return ros_time_from_ns(stamp_ns);
    } else
        return ros::Time::now();
}

ros::Time UAS::synchronise_stamp(uint64_t time_usec) {
    uint64_t offset_ns = time_offset;

    if (offset_ns > 0 || tsync_mode == timesync_mode::PASSTHROUGH) {
        uint64_t stamp_ns = time_usec * 1000UL + offset_ns;
        return ros_time_from_ns(stamp_ns);
    } else
        return ros::Time::now();
}
