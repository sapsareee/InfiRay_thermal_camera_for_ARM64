#include "infiray_ros2/infiray_net_camera.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <utility>

namespace infiray_ros2
{

namespace
{

std::string sdkError(const char* operation, int result)
{
    return std::string(operation) + " failed with SDK error " +
           std::to_string(result);
}

}  // namespace

InfirayNetCamera::~InfirayNetCamera()
{
    stop();
}

bool InfirayNetCamera::start(
    const Config& config,
    VideoCallback video_callback,
    TemperatureCallback temperature_callback,
    DeviceInfo& device_info,
    std::string& error)
{
    stop();
    error.clear();
    device_info = DeviceInfo{};

    if (config.ip.empty() || config.port <= 0 || config.port > 65535 ||
        config.username.empty()) {
        error = "invalid camera connection parameters";
        return false;
    }
    if (!video_callback || !temperature_callback) {
        error = "video and temperature callbacks are required";
        return false;
    }

    int result = IRC_NET_Init();
    if (result != IRC_NET_ERROR_OK) {
        error = sdkError("IRC_NET_Init", result);
        return false;
    }
    sdk_initialized_ = true;

    IRC_NET_LOGIN_INFO login_info{};
    std::snprintf(login_info.ip, sizeof(login_info.ip), "%s", config.ip.c_str());
    login_info.port = config.port;
    std::snprintf(
        login_info.username,
        sizeof(login_info.username),
        "%s",
        config.username.c_str());
    std::snprintf(
        login_info.password,
        sizeof(login_info.password),
        "%s",
        config.password.c_str());

    result = IRC_NET_Login(&login_info, &handle_);
    if (result != IRC_NET_ERROR_OK) {
        error = sdkError("IRC_NET_Login", result);
        cleanup();
        return false;
    }
    logged_in_ = true;

    IRC_NET_DEV_INFO sdk_device_info{};
    result = IRC_NET_GetDevInfo(handle_, &sdk_device_info);
    if (result != IRC_NET_ERROR_OK) {
        error = sdkError("IRC_NET_GetDevInfo", result);
        cleanup();
        return false;
    }

    device_info.channel_count = sdk_device_info.channelNum;
    device_info.optical_channel = sdk_device_info.optChannel;
    device_info.infrared_channel = sdk_device_info.irChannel;
    device_info.product_generation = sdk_device_info.productGeneration;
    device_info.model = sdk_device_info.model;
    if (device_info.infrared_channel < 0) {
        error = "camera did not report an infrared channel";
        cleanup();
        return false;
    }

    device_info.get_frame_rate_result =
        IRC_NET_GetFrameRate(handle_, &device_info.frame_rate);
    if (config.requested_frame_rate > 0 &&
        (device_info.get_frame_rate_result != IRC_NET_ERROR_OK ||
         device_info.frame_rate != config.requested_frame_rate)) {
        device_info.set_frame_rate_result =
            IRC_NET_SetFrameRate(handle_, config.requested_frame_rate);
        if (device_info.set_frame_rate_result == IRC_NET_ERROR_OK) {
            device_info.frame_rate = config.requested_frame_rate;
            // 일부 펌웨어는 프레임률 적용 시 스트림 서비스를 재시작한다.
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    }

    video_callback_ = std::move(video_callback);
    temperature_callback_ = std::move(temperature_callback);
    accept_callbacks_.store(true, std::memory_order_release);

    IRC_NET_PREVIEW_INFO preview_info{};
    preview_info.channel = device_info.infrared_channel;
    preview_info.streamType = IRC_NET_STREAM_MAIN;
    preview_info.frameFmt = IRC_NET_FRAME_FMT_GRAY;

    result = IRC_NET_StartPreview(
        handle_,
        &preview_info,
        &InfirayNetCamera::videoCallbackBridge,
        this);
    if (result != IRC_NET_ERROR_OK) {
        error = sdkError("IRC_NET_StartPreview", result);
        cleanup();
        return false;
    }
    preview_started_ = true;

    result = IRC_NET_StartPullTemp_V2(
        handle_,
        &InfirayNetCamera::temperatureCallbackBridge,
        this);
    if (result != IRC_NET_ERROR_OK) {
        error = sdkError("IRC_NET_StartPullTemp_V2", result);
        cleanup();
        return false;
    }
    temperature_started_ = true;
    return true;
}

void InfirayNetCamera::stop()
{
    cleanup();
}

bool InfirayNetCamera::running() const noexcept
{
    return preview_started_ && temperature_started_;
}

void InfirayNetCamera::videoCallbackBridge(
    IRC_NET_HANDLE,
    char* frame,
    int width,
    int height,
    void* user_data)
{
    auto* camera = static_cast<InfirayNetCamera*>(user_data);
    if (camera == nullptr ||
        !camera->accept_callbacks_.load(std::memory_order_acquire) ||
        frame == nullptr || width <= 0 || height <= 0) {
        return;
    }

    try {
        camera->video_callback_(
            reinterpret_cast<const std::uint8_t*>(frame),
            width,
            height);
    } catch (...) {
        // C ABI 콜백 경계를 넘어 예외가 전달되지 않게 한다.
    }
}

void InfirayNetCamera::temperatureCallbackBridge(
    IRC_NET_HANDLE,
    IRC_NET_TEMP_INFO_CB* temperature_info,
    IRC_NET_TEMP_EXT_INFO_CB* extended_info,
    void* user_data)
{
    auto* camera = static_cast<InfirayNetCamera*>(user_data);
    if (camera == nullptr ||
        !camera->accept_callbacks_.load(std::memory_order_acquire) ||
        temperature_info == nullptr || temperature_info->temp == nullptr ||
        temperature_info->width <= 0 || temperature_info->height <= 0) {
        return;
    }

    const std::uint64_t timestamp =
        extended_info == nullptr ? 0 : extended_info->utcTime;
    try {
        camera->temperature_callback_(
            reinterpret_cast<const std::uint16_t*>(temperature_info->temp),
            temperature_info->width,
            temperature_info->height,
            timestamp);
    } catch (...) {
        // C ABI 콜백 경계를 넘어 예외가 전달되지 않게 한다.
    }
}

void InfirayNetCamera::cleanup() noexcept
{
    accept_callbacks_.store(false, std::memory_order_release);

    if (temperature_started_) {
        IRC_NET_StopPullTemp(handle_);
        temperature_started_ = false;
    }
    if (preview_started_) {
        IRC_NET_StopPreview(handle_);
        preview_started_ = false;
    }
    if (logged_in_) {
        IRC_NET_Logout(handle_);
        logged_in_ = false;
    }
    handle_ = 0;
    if (sdk_initialized_) {
        IRC_NET_Deinit();
        sdk_initialized_ = false;
    }

    video_callback_ = nullptr;
    temperature_callback_ = nullptr;
}

}  // namespace infiray_ros2
