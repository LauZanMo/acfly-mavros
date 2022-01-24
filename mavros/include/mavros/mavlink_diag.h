/**
 * @file mavlink_diag.h
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

#include <diagnostic_updater/diagnostic_updater.h>
#include <mavconn/interface.h>

namespace mavros {
class MavlinkDiag : public diagnostic_updater::DiagnosticTask {
public:
    explicit MavlinkDiag(std::string name);

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

    void set_mavconn(const mavconn::MAVConnInterface::Ptr &link) {
        weak_link = link;
    }

    void set_connection_status(bool connected) {
        is_connected = connected;
    }

private:
    mavconn::MAVConnInterface::WeakPtr weak_link;
    unsigned int                       last_drop_count;
    std::atomic<bool>                  is_connected;
};
}; // namespace mavros
