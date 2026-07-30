#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "Fieldscale.h"
#include "camera_info_manager/camera_info_manager.hpp"
#include "infiray_ros2/infiray_net_camera.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace infiray_ros2 {

namespace {

constexpr std::uint32_t kDefaultWidth = 384;
constexpr std::uint32_t kDefaultHeight = 288;
constexpr char kCameraName[] = "at3003x";
constexpr char kNodeNamespace[] = "/thermal";
constexpr double kKelvinOffset = 273.15;
constexpr double kRawUnitsPerKelvin = 10.0;

std::uint16_t celsiusToRadiometric(double celsius) {
  const double raw = (celsius + kKelvinOffset) * kRawUnitsPerKelvin;
  if (raw < 0.0 ||
      raw > static_cast<double>(std::numeric_limits<std::uint16_t>::max())) {
    throw std::invalid_argument(
        "temperature mapping range does not fit uint16 Kelvin x 10");
  }
  return static_cast<std::uint16_t>(std::lround(raw));
}

sensor_msgs::msg::CameraInfo makeUncalibratedCameraInfo(std::uint32_t width,
                                                        std::uint32_t height) {
  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.width = width;
  camera_info.height = height;
  camera_info.distortion_model = "plumb_bob";
  return camera_info;
}

bool hostIsBigEndian() {
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  return true;
#else
  return false;
#endif
}

} // namespace

class ThermalSlamNode final : public rclcpp::Node {
public:
  ThermalSlamNode() : Node("thermal_slam", kNodeNamespace) {
    const auto camera_ip =
        declare_parameter<std::string>("camera_ip", "192.168.1.123");
    const auto camera_username =
        declare_parameter<std::string>("camera_username", "admin");
    const auto camera_password =
        declare_parameter<std::string>("camera_password", "admin");
    const auto camera_control_port =
        declare_parameter<std::int64_t>("camera_control_port", 80);
    const auto camera_info_url = declare_parameter<std::string>(
        "camera_info_url", "file://${ROS_HOME}/camera_info/at3003x.yaml");
    frame_id_ =
        declare_parameter<std::string>("frame_id", "thermal_optical_frame");
    const auto configured_width =
        declare_parameter<std::int64_t>("image_width", kDefaultWidth);
    const auto configured_height =
        declare_parameter<std::int64_t>("image_height", kDefaultHeight);
    if (configured_width <= 0 || configured_height <= 0 ||
        configured_width > std::numeric_limits<std::uint32_t>::max() ||
        configured_height > std::numeric_limits<std::uint32_t>::max()) {
      throw std::invalid_argument(
          "image_width and image_height must fit positive uint32");
    }
    expected_width_ = static_cast<std::uint32_t>(configured_width);
    expected_height_ = static_cast<std::uint32_t>(configured_height);

    tone_mapping_mode_ = declare_parameter<std::string>("tone_mapping_mode",
                                                        "startup_percentile");
    fixed_min_temperature_c_ =
        declare_parameter<double>("fixed_min_temperature_c", 10.0);
    fixed_max_temperature_c_ =
        declare_parameter<double>("fixed_max_temperature_c", 50.0);
    startup_calibration_seconds_ =
        declare_parameter<double>("startup_calibration_seconds", 2.0);
    startup_low_percentile_ =
        declare_parameter<double>("startup_low_percentile", 0.01);
    startup_high_percentile_ =
        declare_parameter<double>("startup_high_percentile", 0.99);
    minimum_mapping_span_c_ =
        declare_parameter<double>("minimum_mapping_span_c", 2.0);

    fieldscale_enabled_ = declare_parameter<bool>("fieldscale_enabled", false);
    fieldscale_strength_ =
        declare_parameter<double>("fieldscale_strength", 0.15);
    fieldscale_max_diff_ =
        declare_parameter<double>("fieldscale_max_diff", 400.0);
    fieldscale_min_diff_ =
        declare_parameter<double>("fieldscale_min_diff", 400.0);
    const auto fieldscale_iterations =
        declare_parameter<std::int64_t>("fieldscale_iterations", 5);
    if (fieldscale_iterations <= 0 ||
        fieldscale_iterations > std::numeric_limits<int>::max()) {
      throw std::invalid_argument(
          "fieldscale_iterations must fit a positive int");
    }
    fieldscale_iterations_ = static_cast<int>(fieldscale_iterations);
    fieldscale_gamma_ = declare_parameter<double>("fieldscale_gamma", 1.0);
    fieldscale_clahe_ = declare_parameter<bool>("fieldscale_clahe", false);
    fieldscale_video_ = declare_parameter<bool>("fieldscale_video", true);

    invert_image_ = declare_parameter<bool>("invert_image", false);
    unsharp_amount_ = declare_parameter<double>("unsharp_amount", 0.7);
    unsharp_sigma_ = declare_parameter<double>("unsharp_sigma", 1.0);
    clahe_enabled_ = declare_parameter<bool>("clahe_enabled", false);
    clahe_clip_limit_ = declare_parameter<double>("clahe_clip_limit", 1.5);
    clahe_grid_size_ =
        static_cast<int>(declare_parameter<std::int64_t>("clahe_grid_size", 8));

    publish_radiometric_ = declare_parameter<bool>("publish_radiometric", true);
    use_camera_timestamp_ =
        declare_parameter<bool>("use_camera_timestamp", true);
    start_preview_for_sdk_compatibility_ =
        declare_parameter<bool>("start_preview_for_sdk_compatibility", false);
    drop_flat_frames_ = declare_parameter<bool>("drop_flat_frames", true);
    minimum_frame_span_c_ =
        declare_parameter<double>("minimum_frame_span_c", 0.2);
    discard_frames_after_manual_nuc_ = static_cast<int>(
        declare_parameter<std::int64_t>("discard_frames_after_manual_nuc", 5));

    validateParameters(camera_ip, camera_username, camera_control_port);

    fixed_mapping_low_ = celsiusToRadiometric(fixed_min_temperature_c_);
    fixed_mapping_high_ = celsiusToRadiometric(fixed_max_temperature_c_);
    minimum_mapping_span_raw_ = std::max<std::uint16_t>(
        1U, static_cast<std::uint16_t>(
                std::lround(minimum_mapping_span_c_ * kRawUnitsPerKelvin)));
    minimum_frame_span_raw_ = std::max<std::uint16_t>(
        1U, static_cast<std::uint16_t>(
                std::lround(minimum_frame_span_c_ * kRawUnitsPerKelvin)));

    if (tone_mapping_mode_ == "fixed") {
      mapping_low_ = fixed_mapping_low_;
      mapping_high_ = fixed_mapping_high_;
      mapping_ready_ = true;
    } else {
      mapping_histogram_.fill(0);
    }

    if (clahe_enabled_) {
      clahe_ = cv::createCLAHE(clahe_clip_limit_,
                               cv::Size(clahe_grid_size_, clahe_grid_size_));
    }
    if (fieldscale_enabled_ && fieldscale_strength_ > 0.0) {
      fieldscale_ = std::make_unique<Fieldscale>(
          fieldscale_max_diff_, fieldscale_min_diff_, fieldscale_iterations_,
          fieldscale_gamma_, fieldscale_clahe_, fieldscale_video_);
      RCLCPP_INFO(
          get_logger(),
          "Mild Fieldscale enabled: strength=%.2f, iterations=%d, gamma=%.2f, "
          "clahe=%s, video smoothing=%s",
          fieldscale_strength_, fieldscale_iterations_, fieldscale_gamma_,
          fieldscale_clahe_ ? "true" : "false",
          fieldscale_video_ ? "true" : "false");
    } else {
      RCLCPP_INFO(get_logger(), "Mild Fieldscale disabled");
    }
    cv::setNumThreads(1);

    camera_info_manager_ =
        std::make_unique<camera_info_manager::CameraInfoManager>(
            this, kCameraName, camera_info_url);
    if (!camera_info_manager_->validateURL(camera_info_url)) {
      throw std::invalid_argument(
          "camera_info_url must be empty or a supported URL");
    }
    (void)camera_info_manager_->getCameraInfo();

    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    slam_publisher_ =
        create_publisher<sensor_msgs::msg::Image>("image_slam", sensor_qos);
    if (publish_radiometric_) {
      radiometric_publisher_ = create_publisher<sensor_msgs::msg::Image>(
          "image_radiometric", sensor_qos);
    }
    camera_info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera_info", sensor_qos);

    manual_nuc_service_ = create_service<std_srvs::srv::Trigger>(
        "manual_nuc",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          runManualNuc(*response);
        });

    InfirayNetCamera::Config camera_config;
    camera_config.ip = camera_ip;
    camera_config.port = static_cast<int>(camera_control_port);
    camera_config.username = camera_username;
    camera_config.password = camera_password;
    camera_config.requested_frame_rate = 0;
    camera_config.requested_osd_mode = -1;
    camera_config.enable_preview = start_preview_for_sdk_compatibility_;
    camera_config.enable_temperature = true;

    InfirayNetCamera::VideoCallback video_callback;
    if (start_preview_for_sdk_compatibility_) {
      // Some older firmware requires Preview to be open before temperature
      // pulling. Frames are deliberately ignored and never enter SLAM.
      video_callback = [](const std::uint8_t *, int, int) {};
    }

    InfirayNetCamera::DeviceInfo device_info;
    std::string camera_error;
    if (!camera_.start(
            camera_config, std::move(video_callback),
            [this](const std::uint16_t *frame, int width, int height,
                   std::uint64_t camera_timestamp_ms) {
              receiveRadiometricFrame(frame, width, height,
                                      camera_timestamp_ms);
            },
            device_info, camera_error)) {
      RCLCPP_ERROR(get_logger(), "Thermal SLAM camera start failed: %s",
                   camera_error.c_str());
      throw std::runtime_error(camera_error);
    }

    worker_running_.store(true, std::memory_order_release);
    worker_thread_ = std::thread(&ThermalSlamNode::processingLoop, this);

    RCLCPP_INFO(get_logger(),
                "Thermal SLAM stream started: %ux%u radiometric Kelvin x 10, "
                "temperature-only=%s",
                expected_width_, expected_height_,
                start_preview_for_sdk_compatibility_ ? "false" : "true");
    RCLCPP_INFO(get_logger(),
                "Topics: %s/image_slam (mono8), "
                "%s/image_radiometric (mono16), %s/camera_info",
                kNodeNamespace, kNodeNamespace, kNodeNamespace);
    RCLCPP_WARN(
        get_logger(),
        "SDK temperature frames are uint16 Kelvin x 10 radiometric data; "
        "they are not confirmed detector RAW14 values");
    RCLCPP_INFO(
        get_logger(),
        "Automatic NUC state is not changed by this node. For uninterrupted "
        "tracking, select manual shutter mode in the camera UI and call "
        "%s/manual_nuc while the platform is stationary",
        kNodeNamespace);

    if (mapping_ready_) {
      logMappingRange("fixed");
    } else {
      RCLCPP_INFO(
          get_logger(),
          "Collecting %.2f seconds of radiometric data; image_slam starts "
          "after the percentile mapping range is frozen",
          startup_calibration_seconds_);
    }
  }

  ~ThermalSlamNode() override {
    camera_.stop();
    worker_running_.store(false, std::memory_order_release);
    frame_condition_.notify_all();
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

private:
  struct PendingFrame {
    std::vector<std::uint16_t> pixels;
    rclcpp::Time stamp;
    std::uint32_t width{0};
    std::uint32_t height{0};
    std::uint64_t sequence{0};
  };

  void validateParameters(const std::string &camera_ip,
                          const std::string &camera_username,
                          std::int64_t camera_control_port) const {
    const bool finite_tone_parameters =
        std::isfinite(fixed_min_temperature_c_) &&
        std::isfinite(fixed_max_temperature_c_) &&
        std::isfinite(startup_calibration_seconds_) &&
        std::isfinite(startup_low_percentile_) &&
        std::isfinite(startup_high_percentile_) &&
        std::isfinite(minimum_mapping_span_c_) &&
        std::isfinite(minimum_frame_span_c_) &&
        std::isfinite(fieldscale_strength_) &&
        std::isfinite(fieldscale_max_diff_) &&
        std::isfinite(fieldscale_min_diff_) &&
        std::isfinite(fieldscale_gamma_) && std::isfinite(unsharp_amount_) &&
        std::isfinite(unsharp_sigma_) && std::isfinite(clahe_clip_limit_);
    if (!finite_tone_parameters) {
      throw std::invalid_argument(
          "temperature and image processing parameters must be finite");
    }
    if (camera_ip.empty() || camera_username.empty()) {
      throw std::invalid_argument(
          "camera_ip and camera_username must not be empty");
    }
    if (camera_control_port <= 0 || camera_control_port > 65535) {
      throw std::invalid_argument("camera_control_port must be in [1, 65535]");
    }
    if (expected_width_ == 0 || expected_height_ == 0) {
      throw std::invalid_argument(
          "image_width and image_height must be positive");
    }
    if (frame_id_.empty()) {
      throw std::invalid_argument("frame_id must not be empty");
    }
    if (tone_mapping_mode_ != "fixed" &&
        tone_mapping_mode_ != "startup_percentile") {
      throw std::invalid_argument(
          "tone_mapping_mode must be fixed or startup_percentile");
    }
    if (fixed_min_temperature_c_ >= fixed_max_temperature_c_) {
      throw std::invalid_argument("fixed_min_temperature_c must be less than "
                                  "fixed_max_temperature_c");
    }
    if (startup_calibration_seconds_ <= 0.0 || startup_low_percentile_ < 0.0 ||
        startup_high_percentile_ > 1.0 ||
        startup_low_percentile_ >= startup_high_percentile_) {
      throw std::invalid_argument(
          "invalid startup percentile mapping parameters");
    }
    if (minimum_mapping_span_c_ <= 0.0 || minimum_frame_span_c_ < 0.0) {
      throw std::invalid_argument(
          "mapping span must be positive and frame span non-negative");
    }
    const double maximum_span_c =
        static_cast<double>(std::numeric_limits<std::uint16_t>::max()) /
        kRawUnitsPerKelvin;
    if (minimum_mapping_span_c_ > maximum_span_c ||
        minimum_frame_span_c_ > maximum_span_c) {
      throw std::invalid_argument(
          "mapping and frame spans must fit uint16 Kelvin x 10");
    }
    if (fieldscale_strength_ < 0.0 || fieldscale_strength_ > 1.0) {
      throw std::invalid_argument("fieldscale_strength must be in [0, 1]");
    }
    if (fieldscale_max_diff_ < 0.0 || fieldscale_min_diff_ < 0.0 ||
        fieldscale_gamma_ <= 0.0) {
      throw std::invalid_argument(
          "Fieldscale differences must be non-negative and gamma positive");
    }
    if (unsharp_amount_ < 0.0 || unsharp_sigma_ <= 0.0) {
      throw std::invalid_argument(
          "unsharp_amount must be non-negative and sigma positive");
    }
    if (clahe_clip_limit_ <= 0.0 || clahe_grid_size_ <= 0) {
      throw std::invalid_argument(
          "CLAHE clip limit and grid size must be positive");
    }
    if (discard_frames_after_manual_nuc_ < 0) {
      throw std::invalid_argument(
          "discard_frames_after_manual_nuc must be non-negative");
    }
  }

  rclcpp::Time resolveFrameStamp(std::uint64_t camera_timestamp_ms) {
    const auto receive_stamp = now();
    if (!use_camera_timestamp_ || camera_timestamp_ms == 0 ||
        camera_timestamp_ms >
            static_cast<std::uint64_t>(
                std::numeric_limits<std::int64_t>::max() / 1000000LL)) {
      return receive_stamp;
    }

    const auto camera_ns =
        static_cast<std::int64_t>(camera_timestamp_ms * 1000000ULL);
    const auto receive_ns = receive_stamp.nanoseconds();

    std::lock_guard<std::mutex> lock(timestamp_mutex_);
    if (!timestamp_offset_initialized_) {
      timestamp_offset_ns_ = receive_ns - camera_ns;
      timestamp_offset_initialized_ = true;
    }

    std::int64_t resolved_ns = camera_ns + timestamp_offset_ns_;
    if (resolved_ns <= last_timestamp_ns_) {
      // Firmware timestamp resets or duplicates must not create
      // non-monotonic image timestamps for ORB-SLAM/IMU synchronization.
      resolved_ns = std::max(receive_ns, last_timestamp_ns_ + 1);
      timestamp_offset_ns_ = resolved_ns - camera_ns;
    }
    last_timestamp_ns_ = resolved_ns;
    return rclcpp::Time(resolved_ns, receive_stamp.get_clock_type());
  }

  void receiveRadiometricFrame(const std::uint16_t *frame, int width,
                               int height, std::uint64_t camera_timestamp_ms) {
    if (frame == nullptr || width <= 0 || height <= 0) {
      return;
    }
    if (width != static_cast<int>(expected_width_) ||
        height != static_cast<int>(expected_height_)) {
      rejected_frames_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Rejected radiometric frame %dx%d; expected %ux%u",
                            width, height, expected_width_, expected_height_);
      return;
    }

    const auto pixel_count =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    static thread_local std::vector<std::uint16_t> callback_buffer;
    callback_buffer.assign(frame, frame + pixel_count);

    PendingFrame next_frame;
    next_frame.stamp = resolveFrameStamp(camera_timestamp_ms);
    next_frame.width = static_cast<std::uint32_t>(width);
    next_frame.height = static_cast<std::uint32_t>(height);
    next_frame.sequence =
        received_frames_.fetch_add(1, std::memory_order_relaxed) + 1;

    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (has_pending_frame_) {
        overwritten_frames_.fetch_add(1, std::memory_order_relaxed);
      }
      next_frame.pixels.swap(callback_buffer);
      pending_frame_ = std::move(next_frame);
      has_pending_frame_ = true;
    }
    frame_condition_.notify_one();
  }

  void processingLoop() {
    auto stats_start = std::chrono::steady_clock::now();
    std::uint64_t previous_received = 0;
    std::uint64_t previous_published = 0;

    while (worker_running_.load(std::memory_order_acquire)) {
      PendingFrame frame;
      {
        std::unique_lock<std::mutex> lock(frame_mutex_);
        frame_condition_.wait(lock, [this] {
          return has_pending_frame_ ||
                 !worker_running_.load(std::memory_order_acquire);
        });
        if (!worker_running_.load(std::memory_order_acquire) &&
            !has_pending_frame_) {
          break;
        }
        frame = std::move(pending_frame_);
        has_pending_frame_ = false;
      }

      try {
        processFrame(frame);
      } catch (const cv::Exception &error) {
        rejected_frames_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                              "Thermal SLAM image processing failed: %s",
                              error.what());
      } catch (const std::exception &error) {
        rejected_frames_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                              "Thermal SLAM frame rejected: %s", error.what());
      }

      const auto stats_now = std::chrono::steady_clock::now();
      const double elapsed =
          std::chrono::duration<double>(stats_now - stats_start).count();
      if (elapsed >= 2.0) {
        const auto received = received_frames_.load(std::memory_order_relaxed);
        const auto published =
            published_frames_.load(std::memory_order_relaxed);
        RCLCPP_INFO(get_logger(),
                    "thermal_slam input %.1f Hz, output %.1f Hz, "
                    "latest-frame drops %llu, flat/NUC drops %llu, "
                    "rejected %llu",
                    static_cast<double>(received - previous_received) / elapsed,
                    static_cast<double>(published - previous_published) /
                        elapsed,
                    static_cast<unsigned long long>(
                        overwritten_frames_.load(std::memory_order_relaxed)),
                    static_cast<unsigned long long>(
                        skipped_slam_frames_.load(std::memory_order_relaxed)),
                    static_cast<unsigned long long>(
                        rejected_frames_.load(std::memory_order_relaxed)));
        previous_received = received;
        previous_published = published;
        stats_start = stats_now;
      }
    }
  }

  void processFrame(const PendingFrame &frame) {
    if (frame.pixels.empty()) {
      return;
    }

    if (publish_radiometric_) {
      publishRadiometric(frame);
    }

    cv::Mat raw(static_cast<int>(frame.height), static_cast<int>(frame.width),
                CV_16UC1, const_cast<std::uint16_t *>(frame.pixels.data()));
    double raw_min = 0.0;
    double raw_max = 0.0;
    cv::minMaxLoc(raw, &raw_min, &raw_max);
    const auto raw_span =
        static_cast<std::uint16_t>(std::max(0.0, raw_max - raw_min));

    if (nuc_in_progress_.load(std::memory_order_acquire) ||
        consumeManualNucDiscard()) {
      skipped_slam_frames_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    if (drop_flat_frames_ && raw_span < minimum_frame_span_raw_) {
      skipped_slam_frames_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Dropping flat radiometric frame (span %.1f C); "
                           "possible NUC shutter or textureless scene",
                           (raw_max - raw_min) / kRawUnitsPerKelvin);
      return;
    }

    if (!mapping_ready_) {
      updateStartupMapping(frame);
      if (!mapping_ready_) {
        // Build Fieldscale's temporal fields and pay its first-frame
        // allocation cost before the SLAM image stream starts.
        if (fieldscale_ != nullptr) {
          (void)fieldscale_->process(raw);
        }
        return;
      }
    }

    const double scale =
        255.0 / static_cast<double>(mapping_high_ - mapping_low_);
    const double offset = -static_cast<double>(mapping_low_) * scale;
    cv::Mat mono8;
    raw.convertTo(mono8, CV_8UC1, scale, offset);

    if (fieldscale_ != nullptr) {
      const cv::Mat fieldscaled = fieldscale_->process(raw);
      if (fieldscaled.type() != CV_8UC1 || fieldscaled.size() != mono8.size()) {
        throw std::runtime_error(
            "Fieldscale returned an unexpected image format");
      }
      cv::addWeighted(mono8, 1.0 - fieldscale_strength_, fieldscaled,
                      fieldscale_strength_, 0.0, mono8);
    }

    if (unsharp_amount_ > 0.0) {
      cv::Mat blurred;
      cv::GaussianBlur(mono8, blurred, cv::Size(), unsharp_sigma_,
                       unsharp_sigma_, cv::BORDER_REPLICATE);
      cv::addWeighted(mono8, 1.0 + unsharp_amount_, blurred, -unsharp_amount_,
                      0.0, mono8);
    }
    if (clahe_ != nullptr) {
      clahe_->apply(mono8, mono8);
    }
    if (invert_image_) {
      cv::bitwise_not(mono8, mono8);
    }

    publishSlamImage(frame, mono8);
  }

  void publishRadiometric(const PendingFrame &frame) {
    sensor_msgs::msg::Image image;
    image.header.stamp = frame.stamp;
    image.header.frame_id = frame_id_;
    image.height = frame.height;
    image.width = frame.width;
    image.encoding = sensor_msgs::image_encodings::MONO16;
    image.is_bigendian = hostIsBigEndian();
    image.step =
        frame.width * static_cast<std::uint32_t>(sizeof(std::uint16_t));
    image.data.resize(frame.pixels.size() * sizeof(std::uint16_t));
    std::memcpy(image.data.data(), frame.pixels.data(), image.data.size());
    radiometric_publisher_->publish(image);
  }

  void publishSlamImage(const PendingFrame &frame, const cv::Mat &mono8) {
    const auto pixel_count = static_cast<std::size_t>(frame.width) *
                             static_cast<std::size_t>(frame.height);

    sensor_msgs::msg::Image image;
    image.header.stamp = frame.stamp;
    image.header.frame_id = frame_id_;
    image.height = frame.height;
    image.width = frame.width;
    image.encoding = sensor_msgs::image_encodings::MONO8;
    image.is_bigendian = false;
    image.step = frame.width;
    image.data.assign(mono8.ptr<std::uint8_t>(0),
                      mono8.ptr<std::uint8_t>(0) + pixel_count);

    auto camera_info = camera_info_manager_->getCameraInfo();
    const bool calibrated = camera_info.k[0] != 0.0;
    if (calibrated && (camera_info.width != frame.width ||
                       camera_info.height != frame.height)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Ignoring %ux%u calibration for %ux%u SLAM image",
                            camera_info.width, camera_info.height, frame.width,
                            frame.height);
      camera_info = makeUncalibratedCameraInfo(frame.width, frame.height);
    } else if (!calibrated) {
      camera_info.width = frame.width;
      camera_info.height = frame.height;
      if (camera_info.distortion_model.empty()) {
        camera_info.distortion_model = "plumb_bob";
      }
    }
    camera_info.header = image.header;

    slam_publisher_->publish(image);
    camera_info_publisher_->publish(camera_info);
    published_frames_.fetch_add(1, std::memory_order_relaxed);
  }

  void updateStartupMapping(const PendingFrame &frame) {
    const auto now_steady = std::chrono::steady_clock::now();
    if (!startup_mapping_started_) {
      startup_mapping_started_ = true;
      startup_mapping_start_ = now_steady;
    }

    for (const auto value : frame.pixels) {
      if (value == 0 || value == std::numeric_limits<std::uint16_t>::max()) {
        continue;
      }
      ++mapping_histogram_[value];
      ++mapping_histogram_samples_;
    }

    const double elapsed =
        std::chrono::duration<double>(now_steady - startup_mapping_start_)
            .count();
    if (elapsed < startup_calibration_seconds_) {
      return;
    }
    if (mapping_histogram_samples_ == 0) {
      mapping_low_ = fixed_mapping_low_;
      mapping_high_ = fixed_mapping_high_;
      mapping_ready_ = true;
      RCLCPP_ERROR(get_logger(),
                   "Startup tone mapping received no valid samples; using the "
                   "configured fixed range");
      logMappingRange("fixed fallback");
      return;
    }

    mapping_low_ = histogramPercentile(startup_low_percentile_);
    mapping_high_ = histogramPercentile(startup_high_percentile_);
    enforceMinimumMappingSpan();
    mapping_ready_ = true;
    logMappingRange("startup percentile, now frozen");
  }

  std::uint16_t histogramPercentile(double percentile) const {
    const auto target = std::max<std::uint64_t>(
        1U, static_cast<std::uint64_t>(std::ceil(
                percentile * static_cast<double>(mapping_histogram_samples_))));
    std::uint64_t cumulative = 0;
    for (std::size_t value = 0; value < mapping_histogram_.size(); ++value) {
      cumulative += mapping_histogram_[value];
      if (cumulative >= target) {
        return static_cast<std::uint16_t>(value);
      }
    }
    return std::numeric_limits<std::uint16_t>::max();
  }

  void enforceMinimumMappingSpan() {
    if (mapping_high_ > mapping_low_ &&
        mapping_high_ - mapping_low_ >= minimum_mapping_span_raw_) {
      return;
    }

    const auto midpoint = (static_cast<std::uint32_t>(mapping_low_) +
                           static_cast<std::uint32_t>(mapping_high_)) /
                          2U;
    const auto half_span =
        static_cast<std::uint32_t>(minimum_mapping_span_raw_) / 2U;
    const auto low = midpoint > half_span ? midpoint - half_span : 0U;
    const auto high =
        std::min<std::uint32_t>(std::numeric_limits<std::uint16_t>::max(),
                                low + minimum_mapping_span_raw_);
    mapping_low_ = static_cast<std::uint16_t>(
        high >= minimum_mapping_span_raw_ ? high - minimum_mapping_span_raw_
                                          : 0U);
    mapping_high_ = static_cast<std::uint16_t>(high);
  }

  void logMappingRange(const char *source) {
    const double low_c =
        static_cast<double>(mapping_low_) / kRawUnitsPerKelvin - kKelvinOffset;
    const double high_c =
        static_cast<double>(mapping_high_) / kRawUnitsPerKelvin - kKelvinOffset;
    RCLCPP_INFO(get_logger(),
                "Tone mapping range (%s): raw [%u, %u], %.2f to %.2f C", source,
                mapping_low_, mapping_high_, low_c, high_c);
  }

  bool consumeManualNucDiscard() {
    int remaining =
        manual_nuc_discard_remaining_.load(std::memory_order_acquire);
    while (remaining > 0) {
      if (manual_nuc_discard_remaining_.compare_exchange_weak(
              remaining, remaining - 1, std::memory_order_acq_rel,
              std::memory_order_acquire)) {
        return true;
      }
    }
    return false;
  }

  void runManualNuc(std_srvs::srv::Trigger::Response &response) {
    nuc_in_progress_.store(true, std::memory_order_release);
    std::string error;
    const bool success = camera_.correctShutter(error);
    if (success) {
      manual_nuc_discard_remaining_.store(discard_frames_after_manual_nuc_,
                                          std::memory_order_release);
      response.success = true;
      response.message = "manual shutter correction completed; post-NUC frames "
                         "will be discarded";
    } else {
      response.success = false;
      response.message = error;
    }
    nuc_in_progress_.store(false, std::memory_order_release);
  }

  std::string frame_id_;
  std::uint32_t expected_width_{kDefaultWidth};
  std::uint32_t expected_height_{kDefaultHeight};

  std::string tone_mapping_mode_;
  double fixed_min_temperature_c_{10.0};
  double fixed_max_temperature_c_{50.0};
  double startup_calibration_seconds_{2.0};
  double startup_low_percentile_{0.01};
  double startup_high_percentile_{0.99};
  double minimum_mapping_span_c_{2.0};
  std::uint16_t fixed_mapping_low_{0};
  std::uint16_t fixed_mapping_high_{0};
  std::uint16_t mapping_low_{0};
  std::uint16_t mapping_high_{0};
  std::uint16_t minimum_mapping_span_raw_{1};
  bool mapping_ready_{false};
  bool startup_mapping_started_{false};
  std::chrono::steady_clock::time_point startup_mapping_start_;
  std::array<std::uint64_t, 65536> mapping_histogram_{};
  std::uint64_t mapping_histogram_samples_{0};

  bool fieldscale_enabled_{false};
  double fieldscale_strength_{0.15};
  double fieldscale_max_diff_{400.0};
  double fieldscale_min_diff_{400.0};
  int fieldscale_iterations_{5};
  double fieldscale_gamma_{1.0};
  bool fieldscale_clahe_{false};
  bool fieldscale_video_{true};
  std::unique_ptr<Fieldscale> fieldscale_;

  bool invert_image_{false};
  double unsharp_amount_{0.7};
  double unsharp_sigma_{1.0};
  bool clahe_enabled_{false};
  double clahe_clip_limit_{1.5};
  int clahe_grid_size_{8};
  cv::Ptr<cv::CLAHE> clahe_;

  bool publish_radiometric_{true};
  bool use_camera_timestamp_{true};
  bool start_preview_for_sdk_compatibility_{false};
  bool drop_flat_frames_{true};
  double minimum_frame_span_c_{0.2};
  std::uint16_t minimum_frame_span_raw_{1};
  int discard_frames_after_manual_nuc_{5};

  std::mutex frame_mutex_;
  std::condition_variable frame_condition_;
  PendingFrame pending_frame_;
  bool has_pending_frame_{false};
  std::atomic<bool> worker_running_{false};
  std::thread worker_thread_;

  std::mutex timestamp_mutex_;
  bool timestamp_offset_initialized_{false};
  std::int64_t timestamp_offset_ns_{0};
  std::int64_t last_timestamp_ns_{std::numeric_limits<std::int64_t>::min()};

  std::atomic<bool> nuc_in_progress_{false};
  std::atomic<int> manual_nuc_discard_remaining_{0};
  std::atomic<std::uint64_t> received_frames_{0};
  std::atomic<std::uint64_t> overwritten_frames_{0};
  std::atomic<std::uint64_t> rejected_frames_{0};
  std::atomic<std::uint64_t> skipped_slam_frames_{0};
  std::atomic<std::uint64_t> published_frames_{0};

  InfirayNetCamera camera_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr slam_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr radiometric_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
      camera_info_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr manual_nuc_service_;
};

} // namespace infiray_ros2

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<infiray_ros2::ThermalSlamNode>();
    rclcpp::spin(node);
  } catch (const std::exception &error) {
    std::cerr << "thermal_slam: " << error.what() << '\n';
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
