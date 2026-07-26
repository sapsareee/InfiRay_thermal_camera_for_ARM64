cmake_minimum_required(VERSION 3.10)
project(infiray_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -march=native -DNDEBUG -Wall -DLINUX -D_LINUX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package(Threads REQUIRED)

set(
  INFIRAY_SDK_DIR
  "/home/hyuns/dev/sdks/infiray_sdk/SDK_NET_Linux_aarch64--glibc--stable-2022.08-1_V2.0.0.1/SDK"
  CACHE PATH
  "Path to the InfiRay SDK_NET SDK directory"
)
if(DEFINED ENV{INFIRAY_SDK_DIR} AND NOT "$ENV{INFIRAY_SDK_DIR}" STREQUAL "")
  set(INFIRAY_SDK_DIR "$ENV{INFIRAY_SDK_DIR}")
endif()

set(INFIRAY_SDK_INCLUDE_DIR "${INFIRAY_SDK_DIR}/include")
set(INFIRAY_SDK_LIBRARY_DIR "${INFIRAY_SDK_DIR}/libs")
set(INFIRAY_SDK_LIBRARY "${INFIRAY_SDK_LIBRARY_DIR}/libIRCNetSDK.so")

foreach(required_path
    "${INFIRAY_SDK_INCLUDE_DIR}/IRCNetSDK.h"
    "${INFIRAY_SDK_LIBRARY}"
    "${INFIRAY_SDK_DIR}/ability.csv"
    "${INFIRAY_SDK_DIR}/custom.csv")
  if(NOT EXISTS "${required_path}")
    message(FATAL_ERROR "Required InfiRay SDK file not found: ${required_path}")
  endif()
endforeach()

set(FIELDSCALE_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../fieldscale/cpp")
if(NOT EXISTS "${FIELDSCALE_CPP_DIR}/Fieldscale.cpp")
  message(FATAL_ERROR
    "Fieldscale submodule is missing. Run: "
    "git submodule update --init --recursive")
endif()
add_subdirectory(
  "${FIELDSCALE_CPP_DIR}"
  "${CMAKE_CURRENT_BINARY_DIR}/fieldscale"
  EXCLUDE_FROM_ALL
)

add_executable(
  thermal_camera_fire_detect
  src/infiray_fire_detect.cpp
  src/infiray_net_camera.cpp
)
add_executable(
  thermal_camera_fire_detect_raw16
  src/infiray_fire_detect_raw16.cpp
  src/infiray_net_camera.cpp
)


# InfiRay SDK_NET 헤더 및 Fieldscale C++ 헤더
target_include_directories(
  thermal_camera_fire_detect
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${INFIRAY_SDK_INCLUDE_DIR}"
)
target_include_directories(
  thermal_camera_fire_detect_raw16
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${INFIRAY_SDK_INCLUDE_DIR}"
    "${FIELDSCALE_CPP_DIR}"
)

# ROS2 및 OpenCV 의존성을 아주 깔끔하게 주입
ament_target_dependencies(
  thermal_camera_fire_detect
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)
ament_target_dependencies(
  thermal_camera_fire_detect_raw16
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

# SDK_NET 본체만 직접 링크한다. SDK가 함께 제공한 OpenSSL/Poco/FFmpeg 등
# 간접 의존성은 DT_RPATH로 해석한다. 시스템 경로를 먼저 두어 ROS/OpenCV가
# 요구하는 symbol-versioned libcurl을 유지하고, 시스템에 없는 SDK 전용
# OpenSSL 1.1/Poco/재생 라이브러리만 SDK 배포본에서 찾는다.
set(INFIRAY_RUNTIME_PATHS
  "/lib/aarch64-linux-gnu"
  "/usr/lib/aarch64-linux-gnu"
  "${INFIRAY_SDK_LIBRARY_DIR}"
)
foreach(target_name
    thermal_camera_fire_detect
    thermal_camera_fire_detect_raw16)
  target_link_libraries(${target_name}
    opencv_core
    opencv_imgproc
    opencv_highgui
    "${INFIRAY_SDK_LIBRARY}"
    Threads::Threads
  )

  set_target_properties(${target_name} PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    BUILD_RPATH "${INFIRAY_RUNTIME_PATHS}"
    INSTALL_RPATH "${INFIRAY_RUNTIME_PATHS}"
  )
  target_link_options(${target_name} PRIVATE "-Wl,--disable-new-dtags")
endforeach()

target_link_libraries(thermal_camera_fire_detect_raw16 Fieldscale)

install(
  TARGETS
    thermal_camera_fire_detect
    thermal_camera_fire_detect_raw16
  DESTINATION lib/${PROJECT_NAME}
)

# SDK_NET은 실행 파일의 디렉터리에서 모델/기능 매핑 CSV를 찾는다.
install(
  FILES
    "${INFIRAY_SDK_DIR}/ability.csv"
    "${INFIRAY_SDK_DIR}/custom.csv"
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
