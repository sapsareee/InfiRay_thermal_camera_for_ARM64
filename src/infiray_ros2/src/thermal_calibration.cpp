#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "Fieldscale.h"
#include "camera_info_manager/camera_info_manager.hpp"
#include "infiray_ros2/infiray_net_camera.hpp"
#include "opencv2/core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace infiray_ros2
{

namespace
{

constexpr std::uint32_t kImageWidth = 384;
constexpr std::uint32_t kImageHeight = 288;
constexpr char kCameraName[] = "at3003x";
constexpr char kNodeNamespace[] = "/thermal";
// AT3003X SDK_NET mode 1 suppresses the camera-generated temperature OSD.
constexpr int kOsdDisabledMode = 1;

enum class CalibrationImageMode : std::int64_t
{
    Basic = 1,
    Fieldscale = 2
};

CalibrationImageMode selectCalibrationImageMode(std::int64_t configured_mode)
{
    if (configured_mode == static_cast<std::int64_t>(
            CalibrationImageMode::Basic)) {
        return CalibrationImageMode::Basic;
    }
    if (configured_mode == static_cast<std::int64_t>(
            CalibrationImageMode::Fieldscale)) {
        return CalibrationImageMode::Fieldscale;
    }
    if (configured_mode != 0) {
        throw std::invalid_argument("image_mode must be 0, 1, or 2");
    }

    while (true) {
        std::cout
            << "\nSelect calibration image mode:\n"
            << "  1: Basic SDK grayscale image\n"
            << "  2: Fieldscale-enhanced thermal image\n"
            << "Enter 1 or 2: "
            << std::flush;

        std::string input;
        if (!std::getline(std::cin, input)) {
            throw std::runtime_error(
                "Unable to read image mode from stdin; use the ROS parameter "
                "image_mode:=1 or image_mode:=2 for non-interactive execution");
        }
        if (input == "1") {
            return CalibrationImageMode::Basic;
        }
        if (input == "2") {
            return CalibrationImageMode::Fieldscale;
        }
        std::cout << "Invalid selection. Please enter 1 or 2.\n";
    }
}

const char* imageModeName(CalibrationImageMode mode)
{
    return mode == CalibrationImageMode::Basic ?
           "basic SDK grayscale" :
           "Fieldscale-enhanced thermal";
}

sensor_msgs::msg::CameraInfo makeUncalibratedCameraInfo(
    std::uint32_t width,
    std::uint32_t height)
{
    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.width = width;
    camera_info.height = height;
    camera_info.distortion_model = "plumb_bob";
    return camera_info;
}

}  // namespace

class ThermalCalibrationNode final : public rclcpp::Node
{
public:
    ThermalCalibrationNode()
    : Node("thermal_calibration", kNodeNamespace)
    {
        const auto camera_ip =
            declare_parameter<std::string>("camera_ip", "192.168.1.123");
        const auto camera_username =
            declare_parameter<std::string>("camera_username", "admin");
        const auto camera_password =
            declare_parameter<std::string>("camera_password", "admin");
        const auto camera_control_port =
            declare_parameter<std::int64_t>("camera_control_port", 80);
        const auto target_image_fps =
            declare_parameter<double>("target_image_fps", 30.0);
        const auto camera_info_url = declare_parameter<std::string>(
            "camera_info_url",
            "file://${ROS_HOME}/camera_info/at3003x.yaml");
        frame_id_ = declare_parameter<std::string>(
            "frame_id", "thermal_optical_frame");
        invert_image_ = declare_parameter<bool>("invert_image", false);
        image_mode_ = selectCalibrationImageMode(
            declare_parameter<std::int64_t>("image_mode", 0));

        const auto fieldscale_max_diff =
            declare_parameter<double>("fieldscale_max_diff", 400.0);
        const auto fieldscale_min_diff =
            declare_parameter<double>("fieldscale_min_diff", 400.0);
        const auto fieldscale_iterations = static_cast<int>(
            declare_parameter<std::int64_t>("fieldscale_iterations", 7));
        const auto fieldscale_gamma =
            declare_parameter<double>("fieldscale_gamma", 1.5);
        const auto fieldscale_clahe =
            declare_parameter<bool>("fieldscale_clahe", false);
        const auto fieldscale_video =
            declare_parameter<bool>("fieldscale_video", true);

        validateParameters(
            camera_ip,
            camera_username,
            camera_control_port,
            target_image_fps);

        if (image_mode_ == CalibrationImageMode::Fieldscale) {
            if (fieldscale_max_diff < 0.0 || fieldscale_min_diff < 0.0 ||
                fieldscale_iterations <= 0 || fieldscale_gamma <= 0.0) {
                throw std::invalid_argument(
                    "Fieldscale differences must be non-negative, and "
                    "iterations and gamma must be positive");
            }
            cv::setNumThreads(1);
            fieldscale_ = std::make_unique<Fieldscale>(
                fieldscale_max_diff,
                fieldscale_min_diff,
                fieldscale_iterations,
                fieldscale_gamma,
                fieldscale_clahe,
                fieldscale_video);
        }

        camera_info_manager_ =
            std::make_unique<camera_info_manager::CameraInfoManager>(
                this, kCameraName, camera_info_url);
        if (!camera_info_manager_->validateURL(camera_info_url)) {
            throw std::invalid_argument(
                "camera_info_url must be empty or use a supported URL such as "
                "file:///absolute/path/at3003x.yaml");
        }

        // Force the initial load before streaming so load errors are reported once
        // at startup. An absent file is expected before the first calibration.
        (void)camera_info_manager_->getCameraInfo();
        if (camera_info_manager_->isCalibrated()) {
            RCLCPP_INFO(
                get_logger(),
                "Loaded AT3003X calibration from %s",
                camera_info_url.c_str());
        } else {
            RCLCPP_WARN(
                get_logger(),
                "No valid calibration loaded; publishing uncalibrated CameraInfo");
        }

        image_publisher_ = create_publisher<sensor_msgs::msg::Image>(
            "image_raw", rclcpp::SensorDataQoS().keep_last(1));
        camera_info_publisher_ =
            create_publisher<sensor_msgs::msg::CameraInfo>(
                "camera_info", rclcpp::SensorDataQoS().keep_last(1));

        const auto publish_period = std::chrono::duration_cast<
            std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / target_image_fps));
        publish_timer_ = create_wall_timer(
            publish_period,
            std::bind(&ThermalCalibrationNode::publishLatestFrame, this));

        InfirayNetCamera::Config camera_config;
        camera_config.ip = camera_ip;
        camera_config.port = static_cast<int>(camera_control_port);
        camera_config.username = camera_username;
        camera_config.password = camera_password;
        camera_config.requested_frame_rate = 0;
        camera_config.requested_osd_mode = kOsdDisabledMode;

        InfirayNetCamera::DeviceInfo device_info;
        std::string camera_error;
        if (!camera_.start(
                camera_config,
                [this](const std::uint8_t* frame, int width, int height) {
                    if (image_mode_ == CalibrationImageMode::Basic) {
                        receiveGrayFrame(frame, width, height);
                    }
                },
                [this](
                    const std::uint16_t* frame,
                    int width,
                    int height,
                    std::uint64_t) {
                    if (image_mode_ == CalibrationImageMode::Fieldscale) {
                        receiveTemperatureFrame(frame, width, height);
                    }
                },
                device_info,
                camera_error)) {
            RCLCPP_ERROR(
                get_logger(),
                "AT3003X start failed: %s. Ensure no other node is connected "
                "and ability.csv/custom.csv are installed beside the executable.",
                camera_error.c_str());
            throw std::runtime_error(camera_error);
        }

        RCLCPP_INFO(
            get_logger(),
            "AT3003X connected: model=%d generation=%d IR channel=%d; "
            "publishing %ux%u mono8 at up to %.2f Hz%s",
            device_info.model,
            device_info.product_generation,
            device_info.infrared_channel,
            kImageWidth,
            kImageHeight,
            target_image_fps,
            invert_image_ ? " (inverted)" : "");
        RCLCPP_INFO(
            get_logger(),
            "Calibration image mode: %lld (%s)",
            static_cast<long long>(image_mode_),
            imageModeName(image_mode_));
        RCLCPP_INFO(
            get_logger(),
            "Camera OSD disabled for calibration (previous mode: %d; "
            "restored on shutdown)",
            device_info.original_osd_mode);
        RCLCPP_INFO(
            get_logger(),
            "Topics: %s/image_raw, %s/camera_info; service: %s/set_camera_info",
            kNodeNamespace,
            kNodeNamespace,
            kNodeNamespace);
    }

    ~ThermalCalibrationNode() override
    {
        camera_.stop();
    }

private:
    struct PendingFrame
    {
        std::vector<std::uint8_t> data;
        std::vector<std::uint16_t> thermal_data;
        rclcpp::Time stamp;
        std::uint32_t width{0};
        std::uint32_t height{0};
    };

    void validateParameters(
        const std::string& camera_ip,
        const std::string& camera_username,
        std::int64_t camera_control_port,
        double target_image_fps) const
    {
        if (camera_ip.empty()) {
            throw std::invalid_argument("camera_ip must not be empty");
        }
        if (camera_username.empty()) {
            throw std::invalid_argument("camera_username must not be empty");
        }
        if (camera_control_port <= 0 || camera_control_port > 65535) {
            throw std::invalid_argument(
                "camera_control_port must be in [1, 65535]");
        }
        if (target_image_fps <= 0.0 || target_image_fps > 60.0) {
            throw std::invalid_argument(
                "target_image_fps must be in (0, 60]");
        }
        if (frame_id_.empty()) {
            throw std::invalid_argument("frame_id must not be empty");
        }
    }

    void receiveGrayFrame(
        const std::uint8_t* frame,
        int width,
        int height)
    {
        if (frame == nullptr || width <= 0 || height <= 0) {
            return;
        }
        if (width != static_cast<int>(kImageWidth) ||
            height != static_cast<int>(kImageHeight)) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Rejected SDK frame with unexpected resolution %dx%d "
                "(AT3003X calibration requires %ux%u)",
                width,
                height,
                kImageWidth,
                kImageHeight);
            return;
        }

        const auto pixel_count =
            static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
        PendingFrame next_frame;
        next_frame.stamp = now();
        next_frame.width = static_cast<std::uint32_t>(width);
        next_frame.height = static_cast<std::uint32_t>(height);
        next_frame.data.assign(frame, frame + pixel_count);

        std::lock_guard<std::mutex> lock(frame_mutex_);
        pending_frame_ = std::move(next_frame);
        has_new_frame_ = true;
    }

    void receiveTemperatureFrame(
        const std::uint16_t* frame,
        int width,
        int height)
    {
        if (frame == nullptr || width <= 0 || height <= 0) {
            return;
        }
        if (width != static_cast<int>(kImageWidth) ||
            height != static_cast<int>(kImageHeight)) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Rejected SDK thermal frame with unexpected resolution %dx%d "
                "(AT3003X calibration requires %ux%u)",
                width,
                height,
                kImageWidth,
                kImageHeight);
            return;
        }

        const auto pixel_count =
            static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
        PendingFrame next_frame;
        next_frame.stamp = now();
        next_frame.width = static_cast<std::uint32_t>(width);
        next_frame.height = static_cast<std::uint32_t>(height);
        next_frame.thermal_data.assign(frame, frame + pixel_count);

        std::lock_guard<std::mutex> lock(frame_mutex_);
        pending_frame_ = std::move(next_frame);
        has_new_frame_ = true;
    }

    void publishLatestFrame()
    {
        PendingFrame frame;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (!has_new_frame_) {
                return;
            }
            frame = std::move(pending_frame_);
            has_new_frame_ = false;
        }

        if (image_mode_ == CalibrationImageMode::Fieldscale) {
            const auto pixel_count =
                static_cast<std::size_t>(frame.width) *
                static_cast<std::size_t>(frame.height);
            if (fieldscale_ == nullptr ||
                frame.thermal_data.size() != pixel_count) {
                return;
            }

            try {
                cv::Mat raw_thermal(
                    static_cast<int>(frame.height),
                    static_cast<int>(frame.width),
                    CV_16UC1,
                    frame.thermal_data.data());
                auto enhanced = fieldscale_->process(raw_thermal);
                if (enhanced.type() != CV_8UC1 ||
                    enhanced.rows != static_cast<int>(frame.height) ||
                    enhanced.cols != static_cast<int>(frame.width)) {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Fieldscale returned an unexpected image format");
                    return;
                }
                if (!enhanced.isContinuous()) {
                    enhanced = enhanced.clone();
                }
                const auto* begin = enhanced.ptr<std::uint8_t>(0);
                frame.data.assign(begin, begin + pixel_count);
            } catch (const cv::Exception& error) {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    5000,
                    "Fieldscale processing failed: %s",
                    error.what());
                return;
            }
        }

        if (invert_image_) {
            std::transform(
                frame.data.begin(),
                frame.data.end(),
                frame.data.begin(),
                [](std::uint8_t value) {
                    return static_cast<std::uint8_t>(255U - value);
                });
        }

        std_msgs::msg::Header header;
        header.stamp = frame.stamp;
        header.frame_id = frame_id_;

        sensor_msgs::msg::Image image;
        image.header = header;
        image.height = frame.height;
        image.width = frame.width;
        image.encoding = sensor_msgs::image_encodings::MONO8;
        image.is_bigendian = false;
        image.step = frame.width;
        image.data = std::move(frame.data);

        auto camera_info = camera_info_manager_->getCameraInfo();
        const bool calibrated = camera_info.k[0] != 0.0;
        if (calibrated &&
            (camera_info.width != frame.width ||
             camera_info.height != frame.height)) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Ignoring calibration for %ux%u because the SDK image is %ux%u",
                camera_info.width,
                camera_info.height,
                frame.width,
                frame.height);
            camera_info =
                makeUncalibratedCameraInfo(frame.width, frame.height);
        } else if (!calibrated) {
            camera_info.width = frame.width;
            camera_info.height = frame.height;
            if (camera_info.distortion_model.empty()) {
                camera_info.distortion_model = "plumb_bob";
            }
        }
        camera_info.header = header;

        image_publisher_->publish(image);
        camera_info_publisher_->publish(camera_info);
    }

    std::string frame_id_;
    bool invert_image_{false};
    CalibrationImageMode image_mode_{CalibrationImageMode::Basic};

    std::mutex frame_mutex_;
    PendingFrame pending_frame_;
    bool has_new_frame_{false};

    InfirayNetCamera camera_;
    std::unique_ptr<Fieldscale> fieldscale_;
    std::unique_ptr<camera_info_manager::CameraInfoManager>
        camera_info_manager_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
        camera_info_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace infiray_ros2

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try {
        auto node =
            std::make_shared<infiray_ros2::ThermalCalibrationNode>();
        rclcpp::spin(node);
    } catch (const std::exception& error) {
        std::cerr << "thermal_calibration: " << error.what() << '\n';
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}
