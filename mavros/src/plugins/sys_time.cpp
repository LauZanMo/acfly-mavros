/**
 * @file sys_time.cpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @author M.H.Kabir <mhkabir98@gmail.com>
 * @brief This file is from mavros open source respository, thanks for their contribution.
 * @version 1.0
 * @date 2022-01-23
 *
 * @copyright Copyright (c) 2022 acfly
 * @copyright Copyright 2014,2015,2016,2017 Vladimir Ermakov, M.H.Kabir.
 * For commercial use, please contact acfly: https://www.acfly.cn
 *
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/TimesyncStatus.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/TimeReference.h>
#include <std_msgs/Duration.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief syncronization status publisher, based on diagnostic_updater::FrequencyStatus
 * @brief 同步状态发布者，基于diagnistic_updater功能包中的diagnistic_updater::FrequencyStatus
 * @note 启动后可使用rqt工具中的runtime monitor，进行飞控同步状态诊断
 */
class TimeSyncStatus : public diagnostic_updater::DiagnosticTask {
public:
    TimeSyncStatus(const std::string &name, size_t win_size)
        : diagnostic_updater::DiagnosticTask(name), times_(win_size), seq_nums_(win_size),
          window_size_(win_size), min_freq_(0.01), max_freq_(10), tolerance_(0.1), last_rtt_(0),
          rtt_sum_(0), last_remote_ts_(0), offset_(0) {
        clear();
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);

        ros::Time curtime = ros::Time::now();
        count_            = 0;
        rtt_sum_          = 0;

        for (int i = 0; i < window_size_; i++) {
            times_[i]    = curtime;
            seq_nums_[i] = count_;
        }

        hist_indx_ = 0;
    }

    void tick(int64_t rtt_ns, uint64_t remote_timestamp_ns, int64_t time_offset_ns) {
        std::lock_guard<std::mutex> lock(mutex_);

        count_++;
        last_rtt_ = rtt_ns;
        rtt_sum_ += rtt_ns;
        last_remote_ts_ = remote_timestamp_ns;
        offset_         = time_offset_ns;
    }

    void set_timestamp(uint64_t remote_timestamp_ns) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_remote_ts_ = remote_timestamp_ns;
    }

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
        std::lock_guard<std::mutex> lock(mutex_);

        ros::Time curtime     = ros::Time::now();
        int       curseq      = count_;
        int       events      = curseq - seq_nums_[hist_indx_];
        double    window      = (curtime - times_[hist_indx_]).toSec();
        double    freq        = events / window;
        seq_nums_[hist_indx_] = curseq;
        times_[hist_indx_]    = curtime;
        hist_indx_            = (hist_indx_ + 1) % window_size_;

        if (events == 0) {
            stat.summary(2, "No events recorded.");
        } else if (freq < min_freq_ * (1 - tolerance_)) {
            stat.summary(1, "Frequency too low.");
        } else if (freq > max_freq_ * (1 + tolerance_)) {
            stat.summary(1, "Frequency too high.");
        } else {
            stat.summary(0, "Normal");
        }

        stat.addf("Timesyncs since startup", "%d", count_);
        stat.addf("Frequency (Hz)", "%f", freq);
        stat.addf("Last RTT (ms)", "%0.6f", last_rtt_ / 1e6);
        stat.addf("Mean RTT (ms)", "%0.6f", (count_) ? rtt_sum_ / count_ / 1e6 : 0.0);
        stat.addf("Last remote time (s)", "%0.9f", last_remote_ts_ / 1e9);
        stat.addf("Estimated time offset (s)", "%0.9f", offset_ / 1e9);
    }

private:
    int                    count_;
    std::vector<ros::Time> times_;
    std::vector<int>       seq_nums_;
    int                    hist_indx_;
    std::mutex             mutex_;
    const size_t           window_size_;
    const double           min_freq_;
    const double           max_freq_;
    const double           tolerance_;
    int64_t                last_rtt_;
    int64_t                rtt_sum_;
    uint64_t               last_remote_ts_;
    int64_t                offset_;
};

/**
 * @brief System time plugin
 * @brief 系统时间ROS插件
 * @note 该插件会接收与发布系统时间，并与飞控进行时间同步与诊断
 */
class SystemTimePlugin : public plugin::PluginBase {
public:
    SystemTimePlugin()
        : PluginBase(), nh("~"), dt_diag("Time Sync", 10), time_offset(0.0), time_skew(0.0),
          sequence(0), filter_alpha(0), filter_beta(0), high_rtt_count(0), high_deviation_count(0) {
    }

    using TSM = UAS::timesync_mode;

    void initialize(UAS &uas_) override {
        PluginBase::initialize(uas_);

        double      conn_system_time_d;
        double      conn_timesync_d;
        std::string ts_mode_str;

        ros::WallDuration conn_system_time;
        ros::WallDuration conn_timesync;

        if (nh.getParam("conn/system_time_rate", conn_system_time_d) &&
            conn_system_time_d != 0.0) {
            conn_system_time = ros::WallDuration(ros::Rate(conn_system_time_d));
        }

        if (nh.getParam("conn/timesync_rate", conn_timesync_d) && conn_timesync_d != 0.0) {
            conn_timesync = ros::WallDuration(ros::Rate(conn_timesync_d));
        }

        nh.param<std::string>("time/time_ref_source", time_ref_source, "fcu");
        nh.param<std::string>("time/timesync_mode", ts_mode_str, "MAVLINK");

        // Filter gains
        //
        // Alpha : Used to smooth the overall clock offset estimate. Smaller values will lead
        // to a smoother estimate, but track time drift more slowly, introducing a bias
        // in the estimate. Larger values will cause low-amplitude oscillations.
        //
        // Beta : Used to smooth the clock skew estimate. Smaller values will lead to a
        // tighter estimation of the skew (derivative), but will negatively affect how fast the
        // filter reacts to clock skewing (e.g cause by temperature changes to the oscillator).
        // Larger values will cause large-amplitude oscillations.
        //
        // 滤波器增益
        //
        // Alpha：用于平滑整体时钟偏移估计，该值较小会有更平滑的估计，但跟踪漂移的反应速度更慢，从而对估计引入误
        // 差，该值较大则会导致估计小幅度震荡
        //
        // Beta：用于平滑时钟斜率估计，该值较小能更精确地估计斜率(也就是导数)，但滤波器对时钟斜率的反应速度更慢
        // (如振荡器温度变化引起的偏差)，该值较大则会导致斜率较大的振幅振荡。
        nh.param("time/timesync_alpha_initial", filter_alpha_initial, 0.05f);
        nh.param("time/timesync_beta_initial", filter_beta_initial, 0.05f);
        nh.param("time/timesync_alpha_final", filter_alpha_final, 0.003f);
        nh.param("time/timesync_beta_final", filter_beta_final, 0.003f);
        filter_alpha = filter_alpha_initial;
        filter_beta  = filter_beta_initial;

        // Filter gain scheduling
        //
        // The filter interpolates between the initial and final gains while the number of
        // exhanged timesync packets is less than convergence_window. A lower value will
        // allow the timesync to converge faster, but with potentially less accurate initial
        // offset and skew estimates.
        //
        // 滤波器增益调度
        //
        // 当交换的时间同步数据包的数量小于收敛窗口时，滤波器在初始和最终增益之间进行插值。该值较低的值将允许时间
        // 同步更快地收敛，但初始偏移和偏斜估计可能不太准确。
        nh.param("time/convergence_window", convergence_window, 500);

        // Outlier rejection and filter reset
        //
        // Samples with round-trip time higher than max_rtt_sample are not used to update the
        // filter. More than max_consecutive_high_rtt number of such events in a row will throw a
        // warning but not reset the filter. Samples whose calculated clock offset is more than
        // max_deviation_sample off from the current estimate are not used to update the filter.
        // More than max_consecutive_high_deviation number of such events in a row will reset the
        // filter. This usually happens only due to a time jump on the remote system.
        //
        // 异常值拒绝和滤波器重置
        //
        // 往返时间高于max_rtt_sample参数的样本不用于更新滤波器。
        // 连续超过max_consecutive_high_rtt参数的此类 事件将引发警告但不会重置滤波器。
        // 计算出的时钟偏移量大于当前估计的max_deviation_sample参数的样本不用于 更新滤波器。
        // 连续超过max_consecutive_high_deviation参数的此类事件将重置滤波器。这通常由于远端系统
        // (飞控)上的时间跳跃而发生。
        nh.param("time/max_rtt_sample", max_rtt_sample, 10);              // in ms
        nh.param("time/max_deviation_sample", max_deviation_sample, 100); // in ms
        nh.param("time/max_consecutive_high_rtt", max_cons_high_rtt, 5);
        nh.param("time/max_consecutive_high_deviation", max_cons_high_deviation, 5);

        // Set timesync mode
        // 设置同步模式
        auto ts_mode = utils::timesync_mode_from_str(ts_mode_str);
        m_uas->set_timesync_mode(ts_mode);
        ROS_INFO_STREAM_NAMED("time", "TM: Timesync mode: " << utils::to_string(ts_mode));

        nh.param("time/publish_sim_time", publish_sim_time, false);
        if (publish_sim_time) {
            sim_time_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 10);
            ROS_INFO_STREAM_NAMED("time", "TM: Publishing sim time");
        } else {
            ROS_INFO_STREAM_NAMED("time", "TM: Not publishing sim time");
        }
        time_ref_pub = nh.advertise<sensor_msgs::TimeReference>("time_reference", 10);

        timesync_status_pub = nh.advertise<mavros_msgs::TimesyncStatus>("timesync_status", 10);

        // timer for sending system time messages
        // 用于发送系统时间信息的定时器
        if (!conn_system_time.isZero()) {
            sys_time_timer =
                nh.createWallTimer(conn_system_time, &SystemTimePlugin::sys_time_cb, this);
            sys_time_timer.start();
        }

        // timer for sending timesync messages
        // 用于发送同步信息的定时器
        if (!conn_timesync.isZero() && !(ts_mode == TSM::NONE || ts_mode == TSM::PASSTHROUGH)) {
            // enable timesync diag only if that feature enabled
            // 使能时间同步诊断，仅在该特性使能的情况下
            UAS_DIAG(m_uas).add(dt_diag);

            timesync_timer =
                nh.createWallTimer(conn_timesync, &SystemTimePlugin::timesync_cb, this);
            timesync_timer.start();
        }
    }

    Subscriptions get_subscriptions() override {
        return {
            make_handler(&SystemTimePlugin::handle_system_time),
            make_handler(&SystemTimePlugin::handle_timesync),
        };
    }

private:
    ros::NodeHandle nh;
    ros::Publisher  sim_time_pub;
    ros::Publisher  time_ref_pub;
    ros::Publisher  timesync_status_pub;

    ros::WallTimer sys_time_timer;
    ros::WallTimer timesync_timer;

    TimeSyncStatus dt_diag;

    std::string time_ref_source;

    // Estimated statistics
    // 估计数据
    double time_offset;
    double time_skew;

    // Filter parameters
    // 滤波器参数
    uint32_t sequence;
    double   filter_alpha;
    double   filter_beta;

    // Filter settings
    // 滤波器设置
    float filter_alpha_initial;
    float filter_beta_initial;
    float filter_alpha_final;
    float filter_beta_final;
    int   convergence_window;

    // Outlier rejection
    // 异常值剔除
    int max_rtt_sample;
    int max_deviation_sample;
    int max_cons_high_rtt;
    int max_cons_high_deviation;
    int high_rtt_count;
    int high_deviation_count;

    bool publish_sim_time;

    /* message handlers */
    /* 信息回调句柄 */

    void handle_system_time(const mavlink::mavlink_message_t  *msg,
                            mavlink::common::msg::SYSTEM_TIME &mtime) {
        // date -d @1234567890: Sat Feb 14 02:31:30 MSK 2009
        // 日期 -d @1234567890: 2009年2月14日星期六 时间02:31:30
        const bool fcu_time_valid = mtime.time_unix_usec > 1234567890ULL * 1000000;

        if (fcu_time_valid) {
            // continious publish for ntpd
            // 为ntpd服务持续发布时间信息
            auto      time_unix = boost::make_shared<sensor_msgs::TimeReference>();
            ros::Time time_ref(mtime.time_unix_usec / 1000000,           // t_sec
                               (mtime.time_unix_usec % 1000000) * 1000); // t_nsec

            time_unix->header.stamp = ros::Time::now();
            time_unix->time_ref     = time_ref;
            time_unix->source       = time_ref_source;

            time_ref_pub.publish(time_unix);
            if (publish_sim_time) {
                auto clock   = boost::make_shared<rosgraph_msgs::Clock>();
                clock->clock = time_ref;
                sim_time_pub.publish(clock);
            }
        } else {
            ROS_WARN_THROTTLE_NAMED(60, "time", "TM: Wrong FCU time.");
        }
    }

    void handle_timesync(const mavlink::mavlink_message_t *msg,
                         mavlink::common::msg::TIMESYNC   &tsync) {
        uint64_t now_ns = ros::Time::now().toNSec();

        if (tsync.tc1 == 0) {
            send_timesync_msg(now_ns, tsync.ts1);
            return;
        } else if (tsync.tc1 > 0) {
            // Time offset between this system and the remote system is calculated assuming RTT for
            // the timesync packet is roughly equal both ways.
            // 计算该系统与远端系统(飞控)时间偏移，是基于双向发送时间同步数据包所需的RTT(往返时延)相等的假设
            add_timesync_observation((tsync.ts1 + now_ns - tsync.tc1 * 2) / 2, tsync.ts1,
                                     tsync.tc1);
        }
    }

    /* mid-level functions */
    /* 中间件函数 */

    void send_timesync_msg(uint64_t tc1, uint64_t ts1) {
        mavlink::common::msg::TIMESYNC tsync{};
        tsync.tc1 = tc1;
        tsync.ts1 = ts1;

        UAS_FCU(m_uas)->send_message_ignore_drop(tsync);
    }

    void add_timesync_observation(int64_t offset_ns, uint64_t local_time_ns,
                                  uint64_t remote_time_ns) {
        uint64_t now_ns = ros::Time::now().toNSec();

        // Calculate the round trip time (RTT) it took the timesync packet to bounce back to us from
        // remote system
        // 通过从远端系统(飞控)应答回来的时间同步数据包计算RTT(往返时延)
        uint64_t rtt_ns = now_ns - local_time_ns;

        // Calculate the difference of this sample from the current estimate
        // 计算该样本与当前估计值的差异
        uint64_t deviation = llabs(int64_t(time_offset) - offset_ns);

        if (rtt_ns < max_rtt_sample * 1000000ULL) { // Only use samples with low RTT
            if (sync_converged() && (deviation > max_deviation_sample * 1000000ULL)) {
                // Increment the counter if we have a good estimate and are getting samples far from
                // the estimate
                // 如果有一个好的估计但是样本远离该估计，则增加计数器数值
                high_deviation_count++;

                // We reset the filter if we received consecutive samples which violate our present
                // estimate. This is most likely due to a time jump on the offboard system.
                // 如果连续采集到与当前估计矛盾的样本，滤波器会被重置。
                // 这很可能是由于offboard系统的时间跳跃。
                if (high_deviation_count > max_cons_high_deviation) {
                    ROS_ERROR_NAMED("time",
                                    "TM : Time jump detected. Resetting time synchroniser.");

                    // Reset the filter
                    reset_filter();

                    // Reset diagnostics
                    dt_diag.clear();
                    dt_diag.set_timestamp(remote_time_ns);
                }
            } else {
                // Filter gain scheduling
                // 滤波器增益调度
                if (!sync_converged()) {
                    // Interpolate with a sigmoid function
                    float progress = float(sequence) / convergence_window;
                    float p        = 1.0f - expf(0.5f * (1.0f - 1.0f / (1.0f - progress)));
                    filter_alpha   = p * filter_alpha_final + (1.0f - p) * filter_alpha_initial;
                    filter_beta    = p * filter_beta_final + (1.0f - p) * filter_beta_initial;
                } else {
                    filter_alpha = filter_alpha_final;
                    filter_beta  = filter_beta_final;
                }

                // Perform filter update
                // 滤波器更新
                add_sample(offset_ns);

                // Save time offset for other components to use
                // 保存时间偏移供其他组件使用
                m_uas->set_time_offset(sync_converged() ? time_offset : 0);

                // Increment sequence counter after filter update
                // 滤波器更新后增加序列数
                sequence++;

                // Reset high deviation count after filter update
                // 滤波器更新后重置导数数值过高的计数
                high_deviation_count = 0;

                // Reset high RTT count after filter update
                // 滤波器更新后重置RTT(往返时延)数值过高的计数
                high_rtt_count = 0;
            }
        } else {
            // Increment counter if round trip time is too high for accurate timesync
            high_rtt_count++;

            if (high_rtt_count > max_cons_high_rtt) {
                // Issue a warning to the user if the RTT is constantly high
                ROS_WARN_NAMED("time", "TM : RTT too high for timesync: %0.2f ms.",
                               rtt_ns / 1000000.0);

                // Reset counter
                high_rtt_count = 0;
            }
        }

        // Publish timesync status
        // 发布时间同步状态
        auto timesync_status = boost::make_shared<mavros_msgs::TimesyncStatus>();

        timesync_status->header.stamp        = ros::Time::now();
        timesync_status->remote_timestamp_ns = remote_time_ns;
        timesync_status->observed_offset_ns  = offset_ns;
        timesync_status->estimated_offset_ns = time_offset;
        timesync_status->round_trip_time_ms  = float(rtt_ns / 1000000.0);

        timesync_status_pub.publish(timesync_status);

        // Update diagnostics
        // 更新诊断
        dt_diag.tick(rtt_ns, remote_time_ns, time_offset);
    }

    void add_sample(int64_t offset_ns) {
        /**
         * Online exponential smoothing filter. The derivative of the estimate is also
         * estimated in order to produce an estimate without steady state lag:
         * https://en.wikipedia.org/wiki/Exponential_smoothing#Double_exponential_smoothing
         *
         * 在线指数平滑滤波器，估计的导数也会被估计，以产生无稳态滞后的估计：
         * https://en.wikipedia.org/wiki/Exponential_smoothing#Double_exponential_smoothing
         */

        double time_offset_prev = time_offset;

        if (sequence == 0) {
            // First offset sample
            // 首次偏移采样
            time_offset = offset_ns;
        } else {
            // Update the clock offset estimate
            // 更新时钟偏移估计
            time_offset =
                filter_alpha * offset_ns + (1.0 - filter_alpha) * (time_offset + time_skew);

            // Update the clock skew estimate
            // 更新时钟斜率估计
            time_skew =
                filter_beta * (time_offset - time_offset_prev) + (1.0 - filter_beta) * time_skew;
        }
    }

    void reset_filter() {
        // Do a full reset of all statistics and parameters
        // 所有数据与参数重置
        sequence             = 0;
        time_offset          = 0.0;
        time_skew            = 0.0;
        filter_alpha         = filter_alpha_initial;
        filter_beta          = filter_beta_initial;
        high_deviation_count = 0;
        high_rtt_count       = 0;
    }

    inline bool sync_converged() {
        return sequence >= convergence_window;
    }

    uint64_t get_monotonic_now(void) {
        struct timespec spec;
        clock_gettime(CLOCK_MONOTONIC, &spec);

        return spec.tv_sec * 1000000000ULL + spec.tv_nsec;
    }

    /* ros callbacks */
    /* ROS回调函数 */

    void sys_time_cb(const ros::WallTimerEvent &event) {
        // For filesystem only
        uint64_t time_unix_usec = ros::Time::now().toNSec() / 1000; // nano -> micro

        mavlink::common::msg::SYSTEM_TIME mtime{};
        mtime.time_unix_usec = time_unix_usec;

        UAS_FCU(m_uas)->send_message_ignore_drop(mtime);
    }

    void timesync_cb(const ros::WallTimerEvent &event) {
        auto ts_mode = m_uas->get_timesync_mode();
        if (ts_mode == TSM::MAVLINK) {
            send_timesync_msg(0, ros::Time::now().toNSec());
        } else if (ts_mode == TSM::ONBOARD) {
            // Calculate offset between CLOCK_REALTIME (ros::WallTime) and CLOCK_MONOTONIC
            // 计算CLOCK_REALTIME和CLOCK_MONOTONIC的偏移
            uint64_t realtime_now_ns  = ros::Time::now().toNSec();
            uint64_t monotonic_now_ns = get_monotonic_now();

            add_timesync_observation(realtime_now_ns - monotonic_now_ns, realtime_now_ns,
                                     monotonic_now_ns);
        }
    }
};
} // namespace std_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SystemTimePlugin, mavros::plugin::PluginBase)
