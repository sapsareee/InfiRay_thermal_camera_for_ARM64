cmake_minimum_required(VERSION 3.10)
project(infiray_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_options(-O3 -DNDEBUG -Wall -DLINUX -D_LINUX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package(CURL REQUIRED)

set(INFIRAY_SDK_DIR "" CACHE PATH "Path to InfiRay SDK root (must contain include/ and libs/)")

if(NOT INFIRAY_SDK_DIR)
  set(_sdk_candidates
    "$ENV{INFIRAY_SDK_DIR}"
    "/home/gadget/dev/sdks/infiray_thermal_camera_sdk/SDK_NET_Linux_aarch64--glibc--stable-2022.08-1_V2.0.0.1/SDK"
    "/opt/infiray/SDK"
  )

  foreach(_candidate IN LISTS _sdk_candidates)
    if(_candidate AND EXISTS "${_candidate}/include/IRCNetSDK.h" AND EXISTS "${_candidate}/libs")
      set(INFIRAY_SDK_DIR "${_candidate}")
      break()
    endif()
  endforeach()
endif()

if(NOT INFIRAY_SDK_DIR OR NOT EXISTS "${INFIRAY_SDK_DIR}/include/IRCNetSDK.h")
  message(FATAL_ERROR
    "InfiRay SDK was not found. Set INFIRAY_SDK_DIR to a valid SDK path.\n"
    "Expected files:\n"
    "  ${INFIRAY_SDK_DIR}/include/IRCNetSDK.h\n"
    "  ${INFIRAY_SDK_DIR}/libs/libIRCNetSDK.so\n"
    "Example:\n"
    "  colcon build --packages-select infiray_ros2 --cmake-args -DINFIRAY_SDK_DIR=/home/gadget/dev/sdks/infiray_thermal_camera_sdk/SDK_NET_Linux_aarch64--glibc--stable-2022.08-1_V2.0.0.1/SDK")
endif()

find_library(INFIRAY_IRCNET_LIB
  NAMES IRCNetSDK
  PATHS "${INFIRAY_SDK_DIR}/libs"
  NO_DEFAULT_PATH
  REQUIRED
)

# add_executable(thermal_camera_node src/infiray_with_ros2.cpp)
add_executable(thermal_camera_node src/infiray_with_ros2_fixed_fast.cpp)

target_include_directories(thermal_camera_node PUBLIC
  ${INFIRAY_SDK_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(thermal_camera_node rclcpp sensor_msgs std_msgs cv_bridge)

target_link_libraries(thermal_camera_node
  ${INFIRAY_IRCNET_LIB}
  ${OpenCV_LIBS}
  CURL::libcurl
  pthread
  dl
  m
  rt
)

ament_environment_hooks(
  ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/infiray_sdk_library_path.sh.in
  ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/infiray_sdk_library_path.dsv.in
)

install(TARGETS thermal_camera_node DESTINATION lib/${PROJECT_NAME})

if(EXISTS "${INFIRAY_SDK_DIR}/ability.csv")
  install(FILES "${INFIRAY_SDK_DIR}/ability.csv" DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "ability.csv was not found in INFIRAY_SDK_DIR. Runtime login may fail.")
endif()

ament_package()