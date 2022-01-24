/**
 * @file mavlink_diag.cpp
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

#include <mavros/mavlink_diag.h>

using namespace mavros;

MavlinkDiag::MavlinkDiag(std::string name)
    : diagnostic_updater::DiagnosticTask(name), last_drop_count(0), is_connected(false){};

void MavlinkDiag::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    if (auto link = weak_link.lock()) {
        auto mav_status = link->get_status();
        auto iostat     = link->get_iostat();

        stat.addf("Received packets:", "%u", mav_status.packet_rx_success_count);
        stat.addf("Dropped packets:", "%u", mav_status.packet_rx_drop_count);
        stat.addf("Buffer overruns:", "%u", mav_status.buffer_overrun);
        stat.addf("Parse errors:", "%u", mav_status.parse_error);
        stat.addf("Rx sequence number:", "%u", mav_status.current_rx_seq);
        stat.addf("Tx sequence number:", "%u", mav_status.current_tx_seq);

        stat.addf("Rx total bytes:", "%u", iostat.rx_total_bytes);
        stat.addf("Tx total bytes:", "%u", iostat.tx_total_bytes);
        stat.addf("Rx speed:", "%f", iostat.rx_speed);
        stat.addf("Tx speed:", "%f", iostat.tx_speed);

        if (mav_status.packet_rx_drop_count > last_drop_count)
            stat.summaryf(1, "%d packeges dropped since last report",
                          mav_status.packet_rx_drop_count - last_drop_count);
        else if (is_connected)
            stat.summary(0, "connected");
        else
            // link operational, but not connected
            stat.summary(1, "not connected");

        last_drop_count = mav_status.packet_rx_drop_count;
    } else {
        stat.summary(2, "not connected");
    }
}
