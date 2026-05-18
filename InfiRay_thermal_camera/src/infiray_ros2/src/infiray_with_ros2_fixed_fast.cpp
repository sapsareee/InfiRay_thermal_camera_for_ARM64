#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "IRCNetSDK.h"
#include "IRCNetSDKDef.h"

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/header.hpp"

static double rawToCelsius(double raw)
{
    double calcValue = raw;
    double divisor = 10.0;

    if (calcValue > 7300.0) {
        calcValue = calcValue - 3300.0;
        divisor = 15.0;
    } else {
        calcValue = calcValue + 7000.0;
        divisor = 30.0;
    }

    return (calcValue / divisor) - 273.15;
}

struct TempTrendSample
{
    std::chrono::steady_clock::time_point stamp;
    double avgCelsius;
};

static std::mutex g_frame_mutex;
static std::condition_variable g_frame_cv;
static std::vector<uint8_t> g_frame_buffer;
static int g_frame_width = 0;
static int g_frame_height = 0;
static std::atomic<bool> g_has_new_frame{false};
static std::atomic<bool> g_running{true};

static std::mutex g_temp_mutex;
static std::vector<uint16_t> g_temp_buffer;
static int g_temp_width = 0;
static int g_temp_height = 0;

static void video_callback(IRC_NET_HANDLE, IRC_NET_VIDEO_INFO_CB *video_info, IRC_NET_IVS_INFO_CB *, void *)
{
    if (video_info == nullptr || video_info->frame == nullptr || video_info->width <= 0 || video_info->height <= 0) {
        return;
    }

    const std::size_t frame_bytes = static_cast<std::size_t>(video_info->width) * static_cast<std::size_t>(video_info->height) * 4;

    {
        std::lock_guard<std::mutex> lock(g_frame_mutex);
        g_frame_width = video_info->width;
        g_frame_height = video_info->height;
        g_frame_buffer.resize(frame_bytes);
        std::memcpy(g_frame_buffer.data(), video_info->frame, frame_bytes);
        g_has_new_frame.store(true, std::memory_order_release);
    }

    g_frame_cv.notify_one();
}

static void temp_callback(IRC_NET_HANDLE, IRC_NET_TEMP_INFO_CB *temp_info, IRC_NET_TEMP_EXT_INFO_CB *, void *)
{
    if (temp_info == nullptr || temp_info->temp == nullptr || temp_info->width <= 0 || temp_info->height <= 0) {
        return;
    }

    const std::size_t pixel_count = static_cast<std::size_t>(temp_info->width) * static_cast<std::size_t>(temp_info->height);

    std::lock_guard<std::mutex> lock(g_temp_mutex);
    g_temp_width = temp_info->width;
    g_temp_height = temp_info->height;
    g_temp_buffer.resize(pixel_count);
    std::memcpy(g_temp_buffer.data(), temp_info->temp, pixel_count * sizeof(uint16_t));
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("thermal_camera_node");

    node->declare_parameter("show_display", false);
    node->declare_parameter("camera_ip", "192.168.1.123");
    node->declare_parameter("camera_port", 80);
    node->declare_parameter("camera_username", "admin");
    node->declare_parameter("camera_password", "admin");
    node->declare_parameter("camera_channel", 1);

    const bool show_display = node->get_parameter("show_display").as_bool();
    const std::string camera_ip = node->get_parameter("camera_ip").as_string();
    const int camera_port = node->get_parameter("camera_port").as_int();
    const std::string camera_username = node->get_parameter("camera_username").as_string();
    const std::string camera_password = node->get_parameter("camera_password").as_string();
    const int camera_channel = node->get_parameter("camera_channel").as_int();

    auto image_pub = node->create_publisher<sensor_msgs::msg::Image>("/thermal/image", rclcpp::SensorDataQoS());
    auto temp_pub = node->create_publisher<std_msgs::msg::Float32>("/thermal/max_temp", rclcpp::SensorDataQoS());
    auto fire_pub = node->create_publisher<std_msgs::msg::Bool>("/thermal/fire_detected", rclcpp::QoS(10).reliable());

    const int init_ret = IRC_NET_Init();
    if (init_ret != IRC_NET_ERROR_OK) {
        RCLCPP_ERROR(node->get_logger(), "IRC_NET_Init failed. ret=%d", init_ret);
        rclcpp::shutdown();
        return -1;
    }

    (void)IRC_NET_SetLogParam(IRC_NET_LOG_LEVEL_DEBUG, "/tmp/infiray_sdk_log/", 20);

    IRC_NET_LOGIN_INFO login_info{};
    std::strncpy(login_info.ip, camera_ip.c_str(), sizeof(login_info.ip) - 1);
    std::strncpy(login_info.username, camera_username.c_str(), sizeof(login_info.username) - 1);
    std::strncpy(login_info.password, camera_password.c_str(), sizeof(login_info.password) - 1);
    login_info.port = camera_port;

    IRC_NET_HANDLE handle = 0;
    const int login_ret = IRC_NET_Login(&login_info, &handle);
    if (login_ret != IRC_NET_ERROR_OK) {
        RCLCPP_ERROR(node->get_logger(), "IRC_NET_Login failed. ret=%d ip=%s port=%d user=%s", login_ret, login_info.ip, login_info.port, login_info.username);
        IRC_NET_Deinit();
        rclcpp::shutdown();
        return -1;
    }

    IRC_NET_PREVIEW_INFO preview_info{camera_channel, IRC_NET_STREAM_MAIN, IRC_NET_FRAME_FMT_BGRA};
    const int preview_ret = IRC_NET_StartPreview_V2(handle, &preview_info, video_callback, nullptr);
    if (preview_ret != IRC_NET_ERROR_OK) {
        RCLCPP_ERROR(node->get_logger(), "IRC_NET_StartPreview_V2 failed. ret=%d", preview_ret);
        IRC_NET_Logout(handle);
        IRC_NET_Deinit();
        rclcpp::shutdown();
        return -1;
    }

    const int pull_temp_ret = IRC_NET_StartPullTemp_V2(handle, temp_callback, nullptr);
    if (pull_temp_ret != IRC_NET_ERROR_OK) {
        RCLCPP_ERROR(node->get_logger(), "IRC_NET_StartPullTemp_V2 failed. ret=%d", pull_temp_ret);
        IRC_NET_StopPreview(handle);
        IRC_NET_Logout(handle);
        IRC_NET_Deinit();
        rclcpp::shutdown();
        return -1;
    }

    if (show_display) {
        cv::namedWindow("Thermal", cv::WINDOW_NORMAL);
    }

    using namespace std::chrono;
    const double TARGET_IMAGE_FPS = 10.0;
    const auto IMAGE_FRAME_INTERVAL = duration<double>(1.0 / TARGET_IMAGE_FPS);
    auto last_image_pub_time = steady_clock::now();
    double current_fps = 0.0;
    auto last_fps_calc = steady_clock::now();
    int fps_count = 0;

    const double FIRE_THRESHOLD_C = 25.0;
    const double FIRE_HOLD_SECONDS = 1.0;
    const double TREND_WINDOW_SECONDS = 3.0;
    const double AVG_TEMP_MIN_FOR_FIRE = 50.0;
    const double TREND_MIN_C_PER_SEC = 0.3;

    auto hot_above_since = steady_clock::time_point::min();
    std::deque<TempTrendSample> trend_history;

    while (rclcpp::ok() && g_running.load(std::memory_order_acquire)) {
        std::vector<uint8_t> local_frame;
        int local_width = 0;
        int local_height = 0;

        {
            std::unique_lock<std::mutex> lock(g_frame_mutex);
            g_frame_cv.wait_for(lock, std::chrono::milliseconds(100), [] {
                return g_has_new_frame.load(std::memory_order_acquire) || !g_running.load(std::memory_order_acquire) || !rclcpp::ok();
            });

            if (!rclcpp::ok() || !g_running.load(std::memory_order_acquire)) {
                break;
            }

            if (!g_has_new_frame.load(std::memory_order_acquire)) {
                rclcpp::spin_some(node);
                continue;
            }

            local_width = g_frame_width;
            local_height = g_frame_height;
            local_frame = g_frame_buffer;
            g_has_new_frame.store(false, std::memory_order_release);
        }

        if (local_width <= 0 || local_height <= 0 || local_frame.empty()) {
            rclcpp::spin_some(node);
            continue;
        }

        cv::Mat bgra(local_height, local_width, CV_8UC4, local_frame.data());
        cv::Mat display_mat;
        cv::cvtColor(bgra, display_mat, cv::COLOR_BGRA2BGR);

        cv::Mat gray;
        cv::cvtColor(display_mat, gray, cv::COLOR_BGR2GRAY);

        cv::Mat avg_mat;
        cv::boxFilter(gray, avg_mat, CV_8U, cv::Size(30, 30));

        double min_val = 0.0;
        double max_val = 0.0;
        cv::Point min_loc;
        cv::Point max_loc;
        cv::minMaxLoc(avg_mat, &min_val, &max_val, &min_loc, &max_loc);

        int rect_x = std::max(0, max_loc.x - 15);
        int rect_y = std::max(0, max_loc.y - 15);
        if (rect_x + 30 > local_width) rect_x = std::max(0, local_width - 30);
        if (rect_y + 30 > local_height) rect_y = std::max(0, local_height - 30);

        double avgCelsius = 0.0;
        double maxCelsius = 0.0;
        double trendCelsiusPerSec = 0.0;
        double holdSeconds = 0.0;
        bool is_temp_valid = false;
        bool fireCandidate = false;
        uint16_t maxRawTemp = 0;
        {
            std::lock_guard<std::mutex> lock(g_temp_mutex);
            if (!g_temp_buffer.empty() && g_temp_width == local_width && g_temp_height == local_height) {
                long long sumRaw = 0;
                int count = 0;
                for (int yy = rect_y; yy < rect_y + 30; ++yy) {
                    for (int xx = rect_x; xx < rect_x + 30; ++xx) {
                        uint16_t raw = g_temp_buffer[static_cast<std::size_t>(yy) * local_width + xx];
                        sumRaw += raw;
                        if (raw > maxRawTemp) maxRawTemp = raw;
                        ++count;
                    }
                }
                if (count > 0) {
                    double avgRaw = (double)sumRaw / (double)count;
                    // Choose conversion based on observed raw range:
                    // - legacy SDK (used on laptop) produces large raw (>7300) requiring rawToCelsius
                    // - IRCNet SDK raw is deci-Kelvin (raw/10 -> Kelvin)
                    if (avgRaw > 7300.0 || maxRawTemp > 7300) {
                        avgCelsius = rawToCelsius(avgRaw);
                        maxCelsius = rawToCelsius((double)maxRawTemp);
                    } else {
                        avgCelsius = (avgRaw / 10.0) - 273.15;
                        maxCelsius = ((double)maxRawTemp / 10.0) - 273.15;
                    }
                    is_temp_valid = true;

                    auto now = steady_clock::now();
                    trend_history.push_back({now, avgCelsius});
                    while (!trend_history.empty()) {
                        auto age = duration<double>(now - trend_history.front().stamp).count();
                        if (age <= TREND_WINDOW_SECONDS) break;
                        trend_history.pop_front();
                    }

                    if (trend_history.size() >= 2) {
                        const auto &first = trend_history.front();
                        const auto &last = trend_history.back();
                        double elapsed = duration<double>(last.stamp - first.stamp).count();
                        if (elapsed > 0.0) {
                            trendCelsiusPerSec = (last.avgCelsius - first.avgCelsius) / elapsed;
                        }
                    }

                    if (maxCelsius >= FIRE_THRESHOLD_C) {
                        if (hot_above_since == steady_clock::time_point::min()) hot_above_since = now;
                        holdSeconds = duration<double>(now - hot_above_since).count();
                    } else {
                        hot_above_since = steady_clock::time_point::min();
                        holdSeconds = 0.0;
                    }

                    const bool sustainedHot = (holdSeconds >= FIRE_HOLD_SECONDS);
                    const bool avgHotEnough = (avgCelsius >= AVG_TEMP_MIN_FOR_FIRE);
                    const bool trendRisingEnough = (trendCelsiusPerSec >= TREND_MIN_C_PER_SEC);
                    fireCandidate = sustainedHot && (avgHotEnough || trendRisingEnough);
                }
            }
        }

        cv::Rect hot_zone(rect_x, rect_y, 30, 30);
        cv::rectangle(display_mat, hot_zone, cv::Scalar(0, 255, 0), 2);

        char text_buffer[64];
        if (is_temp_valid) {
            std::snprintf(text_buffer, sizeof(text_buffer), "Peak: %.1f C", maxCelsius);
        } else {
            std::snprintf(text_buffer, sizeof(text_buffer), "Wait...");
        }

        cv::Point text_loc(hot_zone.x, std::max(20, hot_zone.y - 10));
        cv::putText(display_mat, text_buffer, text_loc, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);

        // FPS 계산
        fps_count++;
        auto now_for_fps = steady_clock::now();
        auto fps_elapsed = duration<double>(now_for_fps - last_fps_calc).count();
        if (fps_elapsed >= 1.0) {
            current_fps = fps_count / fps_elapsed;
            fps_count = 0;
            last_fps_calc = now_for_fps;
        }

        // 오버레이 지표
        char metrics1[96];
        char metrics2[96];
        char metrics3[96];
        char metrics4[96];
        char metrics5[128];

        std::snprintf(metrics1, sizeof(metrics1), "Avg: %.1f C", avgCelsius);
        std::snprintf(metrics2, sizeof(metrics2), "Max: %.1f C", maxCelsius);
        std::snprintf(metrics3, sizeof(metrics3), "Trend: %.2f C/s", trendCelsiusPerSec);
        std::snprintf(metrics4, sizeof(metrics4), "Hold: %.2f s  Fire: %s", holdSeconds, fireCandidate ? "ON" : "OFF");
        std::snprintf(metrics5, sizeof(metrics5), "FPS: %.1f  Peak:(%d,%d) Raw:%u", current_fps, max_loc.x, max_loc.y, (unsigned)maxRawTemp);

        const int overlayX = 10;
        const int overlayY = 18;
        const int lineGap = 16;

        cv::putText(display_mat, metrics1, cv::Point(overlayX, overlayY), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255,255,255), 1);
        cv::putText(display_mat, metrics2, cv::Point(overlayX, overlayY + lineGap), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255,255,255), 1);
        cv::putText(display_mat, metrics3, cv::Point(overlayX, overlayY + lineGap*2), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255,255,255), 1);
        cv::putText(display_mat, metrics4, cv::Point(overlayX, overlayY + lineGap*3), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(fireCandidate?0:255, fireCandidate?255:255, 0), 1);
        cv::putText(display_mat, metrics5, cv::Point(overlayX, overlayY + lineGap*4), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200,200,200), 1);

        std_msgs::msg::Header header;
        header.stamp = node->now();
        header.frame_id = "thermal_camera_frame";
        auto img_msg = cv_bridge::CvImage(header, "bgr8", display_mat).toImageMsg();
        image_pub->publish(*img_msg);

        std_msgs::msg::Float32 temp_msg;
        temp_msg.data = is_temp_valid ? (float)avgCelsius : 0.0f;
        temp_pub->publish(temp_msg);

        std_msgs::msg::Bool fire_msg;
        fire_msg.data = (is_temp_valid && fireCandidate);
        fire_pub->publish(fire_msg);

        if (show_display) {
            cv::imshow("Thermal", display_mat);
            if (cv::waitKey(1) == 27) {
                g_running.store(false, std::memory_order_release);
            }
        }

        rclcpp::spin_some(node);
    }

    IRC_NET_StopPullTemp(handle);
    IRC_NET_StopPreview(handle);
    IRC_NET_Logout(handle);
    IRC_NET_Deinit();

    if (show_display) {
        cv::destroyAllWindows();
    }

    rclcpp::shutdown();
    return 0;
}