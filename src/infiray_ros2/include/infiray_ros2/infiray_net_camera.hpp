#ifndef INFIRAY_ROS2_INFIRAY_NET_CAMERA_HPP_
#define INFIRAY_ROS2_INFIRAY_NET_CAMERA_HPP_

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>

#include "IRCNetSDK.h"

namespace infiray_ros2
{

class InfirayNetCamera
{
public:
    struct Config
    {
        std::string ip;
        int port{80};
        std::string username;
        std::string password;
        int requested_frame_rate{0};
    };

    struct DeviceInfo
    {
        int channel_count{0};
        int optical_channel{-1};
        int infrared_channel{-1};
        int product_generation{0};
        int model{0};
        int frame_rate{0};
        int get_frame_rate_result{IRC_NET_ERROR_FAILED};
        int set_frame_rate_result{IRC_NET_ERROR_OK};
    };

    using VideoCallback =
        std::function<void(const std::uint8_t*, int, int)>;
    using TemperatureCallback =
        std::function<void(const std::uint16_t*, int, int, std::uint64_t)>;

    InfirayNetCamera() = default;
    ~InfirayNetCamera();

    InfirayNetCamera(const InfirayNetCamera&) = delete;
    InfirayNetCamera& operator=(const InfirayNetCamera&) = delete;

    bool start(
        const Config& config,
        VideoCallback video_callback,
        TemperatureCallback temperature_callback,
        DeviceInfo& device_info,
        std::string& error);

    void stop();
    bool running() const noexcept;

private:
    static void videoCallbackBridge(
        IRC_NET_HANDLE handle,
        char* frame,
        int width,
        int height,
        void* user_data);

    static void temperatureCallbackBridge(
        IRC_NET_HANDLE handle,
        IRC_NET_TEMP_INFO_CB* temperature_info,
        IRC_NET_TEMP_EXT_INFO_CB* extended_info,
        void* user_data);

    void cleanup() noexcept;

    IRC_NET_HANDLE handle_{0};
    VideoCallback video_callback_;
    TemperatureCallback temperature_callback_;
    std::atomic<bool> accept_callbacks_{false};
    bool sdk_initialized_{false};
    bool logged_in_{false};
    bool preview_started_{false};
    bool temperature_started_{false};
};

}  // namespace infiray_ros2

#endif  // INFIRAY_ROS2_INFIRAY_NET_CAMERA_HPP_
