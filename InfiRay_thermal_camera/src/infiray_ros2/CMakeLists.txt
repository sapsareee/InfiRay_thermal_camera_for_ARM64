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
set(INFIRAY_SDK_FLAVOR "auto" CACHE STRING "InfiRay SDK flavor: auto, ircnet, or legacy")

if(NOT INFIRAY_SDK_DIR)
  set(_sdk_candidates
    "$ENV{INFIRAY_SDK_DIR}"
    "/home/gadget/dev/sdks/infiray_thermal_camera_sdk/SDK_NET_Linux_aarch64--glibc--stable-2022.08-1_V2.0.0.1/SDK"
    "/opt/infiray/SDK"
  )

  foreach(_candidate IN LISTS _sdk_candidates)
    if(NOT _candidate OR NOT EXISTS "${_candidate}/libs")
      continue()
    endif()

    if(EXISTS "${_candidate}/include/IRCNetSDK.h" AND EXISTS "${_candidate}/libs/libIRCNetSDK.so")
      set(INFIRAY_SDK_DIR "${_candidate}")
      set(INFIRAY_SDK_FLAVOR "ircnet")
      break()
    elseif(EXISTS "${_candidate}/include/InfraredTempSDK.h" AND EXISTS "${_candidate}/libs/libInfraredTempSDK.so")
      set(INFIRAY_SDK_DIR "${_candidate}")
      set(INFIRAY_SDK_FLAVOR "legacy")
      break()
    endif()
  endforeach()
endif()

if(INFIRAY_SDK_FLAVOR STREQUAL "auto")
  if(EXISTS "${INFIRAY_SDK_DIR}/include/IRCNetSDK.h" AND EXISTS "${INFIRAY_SDK_DIR}/libs/libIRCNetSDK.so")
    set(INFIRAY_SDK_FLAVOR "ircnet")
  elseif(EXISTS "${INFIRAY_SDK_DIR}/include/InfraredTempSDK.h" AND EXISTS "${INFIRAY_SDK_DIR}/libs/libInfraredTempSDK.so")
    set(INFIRAY_SDK_FLAVOR "legacy")
  endif()
endif()

if(NOT INFIRAY_SDK_DIR OR INFIRAY_SDK_FLAVOR STREQUAL "auto")
  message(FATAL_ERROR
    "InfiRay SDK was not found. Set INFIRAY_SDK_DIR to a valid SDK path.\n"
    "Expected files:\n"
    "  ${INFIRAY_SDK_DIR}/include/IRCNetSDK.h and ${INFIRAY_SDK_DIR}/libs/libIRCNetSDK.so\n"
    "  or ${INFIRAY_SDK_DIR}/include/InfraredTempSDK.h and ${INFIRAY_SDK_DIR}/libs/libInfraredTempSDK.so\n"
    "Example for the current arm SDK:\n"
    "  colcon build --packages-select infiray_ros2 --cmake-args -DINFIRAY_SDK_DIR=/home/gadget/dev/sdks/infiray_thermal_camera_sdk/SDK_NET_Linux_aarch64--glibc--stable-2022.08-1_V2.0.0.1/SDK")
endif()

if(INFIRAY_SDK_FLAVOR STREQUAL "ircnet")
  find_library(INFIRAY_SDK_MAIN_LIB
    NAMES IRCNetSDK
    PATHS "${INFIRAY_SDK_DIR}/libs"
    NO_DEFAULT_PATH
    REQUIRED
  )
elseif(INFIRAY_SDK_FLAVOR STREQUAL "legacy")
  find_library(INFIRAY_SDK_MAIN_LIB
    NAMES InfraredTempSDK
    PATHS "${INFIRAY_SDK_DIR}/libs"
    NO_DEFAULT_PATH
    REQUIRED
  )
  find_library(INFIRAY_IRNET_CLIENT_LIB
    NAMES IRNetClient
    PATHS "${INFIRAY_SDK_DIR}/libs"
    NO_DEFAULT_PATH
    REQUIRED
  )
  find_library(INFIRAY_HYVSTREAM_LIB
    NAMES hyvstream
    PATHS "${INFIRAY_SDK_DIR}/libs"
    NO_DEFAULT_PATH
    REQUIRED
  )
  find_library(INFIRAY_HTTPCLIENT_LIB
    NAMES httpclient
    PATHS "${INFIRAY_SDK_DIR}/libs"
    NO_DEFAULT_PATH
    REQUIRED
  )
endif()

# Allow overriding which node source is built by passing -DINFIRAY_NODE_SOURCE=...
# If not provided, choose based on detected SDK flavor.
if(NOT DEFINED INFIRAY_NODE_SOURCE)
  if(INFIRAY_SDK_FLAVOR STREQUAL "ircnet")
    set(INFIRAY_NODE_SOURCE src/infiray_with_ros2_fixed_fast.cpp)
  elseif(INFIRAY_SDK_FLAVOR STREQUAL "legacy")
    set(INFIRAY_NODE_SOURCE src/infiray_fire_detect.cpp)
  endif()
endif()

add_executable(thermal_camera_node ${INFIRAY_NODE_SOURCE})

target_include_directories(thermal_camera_node PUBLIC
  ${INFIRAY_SDK_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(thermal_camera_node rclcpp sensor_msgs std_msgs cv_bridge)

target_link_libraries(thermal_camera_node
  ${INFIRAY_SDK_MAIN_LIB}
  ${OpenCV_LIBS}
  CURL::libcurl
  pthread
  dl
  m
  rt
)

if(INFIRAY_SDK_FLAVOR STREQUAL "legacy")
  target_link_libraries(thermal_camera_node
    ${INFIRAY_IRNET_CLIENT_LIB}
    ${INFIRAY_HYVSTREAM_LIB}
    ${INFIRAY_HTTPCLIENT_LIB}
    curl
    ssl
    crypto
    z
    lzma
  )
endif()

set_target_properties(thermal_camera_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${INFIRAY_SDK_DIR}/libs"
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