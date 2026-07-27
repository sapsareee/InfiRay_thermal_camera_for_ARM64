# InfiRay 열화상 카메라 ROS 2 (Jetson ARM64)

이 저장소는 InfiRay 열화상 카메라의 영상과 온도 데이터를 ARM64 Jetson에서
수신하고 ROS 2 토픽으로 발행합니다. 기존 AMD64 구현의 화재 판정 로직과 토픽
구조를 유지하면서 InfiRay `SDK_NET` Linux aarch64 SDK에 맞게 이식한
워크스페이스입니다.

## 제공 노드

| 실행 파일 | 기본 발행률 | 설명 |
| --- | ---: | --- |
| `thermal_camera_fire_detect` | 10 Hz | 일반 열화상 표시와 화재 감지 |
| `thermal_camera_fire_detect_raw16` | 30 Hz | Fieldscale 영상 처리가 추가된 노드 |
| `thermal_calibration` | 30 Hz | 오버레이 없는 AT3003X mono8 영상과 CameraInfo |

화재 감지 노드 두 개는 다음 기능을 제공합니다.

- SDK_NET을 통한 열화상 영상과 온도 프레임 수신
- `/thermal/image` 영상 발행
- 관심 영역의 최고 온도와 온도 변화 추세 계산
- 온도와 지속 시간 조건을 사용한 화재 감지
- 최신 프레임 우선 처리로 누적 지연 방지
- 선택적인 OpenCV 로컬 화면

## 실행 환경

- NVIDIA Jetson ARM64 (`uname -m` 결과가 `aarch64`)
- Ubuntu 22.04
- ROS 2 Humble
- OpenCV 및 ROS 2 `cv_bridge`
- InfiRay SDK_NET Linux aarch64 V2.0.0.1
- 카메라와 Jetson이 통신 가능한 동일 네트워크

ROS 2 Humble이 이미 설치되어 있다는 전제에서 필요한 패키지는 다음과 같습니다.

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  libopencv-dev \
  python3-colcon-common-extensions \
  ros-humble-cv-bridge \
  ros-humble-camera-info-manager \
  ros-humble-rclcpp \
  ros-humble-sensor-msgs \
  ros-humble-std-msgs
```

실제 보정과 보정 영상 검증에는 다음 패키지도 필요합니다.

```bash
sudo apt install -y \
  ros-humble-camera-calibration \
  ros-humble-image-proc
```

## 저장소 받기

Fieldscale 소스는 이 저장소의 `src/fieldscale`에 포함되어 있으므로 일반
`git clone`만으로 함께 내려받을 수 있습니다.

```bash
cd ~/dev/repos
git clone \
  https://github.com/sapsareee/InfiRay_thermal_camera_for_ARM64.git \
  Infiray_ws
cd Infiray_ws
```

## ARM64 SDK 배치

현재 기본 SDK 경로는 다음과 같습니다.

```text
/home/hyuns/dev/sdks/infiray_sdk/
└── SDK_NET_Linux_aarch64--glibc--stable-2022.08-1_V2.0.0.1/
    └── SDK/
        ├── ability.csv
        ├── custom.csv
        ├── include/IRCNetSDK.h
        └── libs/libIRCNetSDK.so
```

SDK를 다른 곳에 두었다면 `SDK` 디렉터리 자체를
`INFIRAY_SDK_DIR`로 지정합니다.

```bash
export INFIRAY_SDK_DIR=/path/to/SDK_NET_Linux_aarch64/SDK
```

경로가 올바른지 확인할 수 있습니다.

```bash
export INFIRAY_SDK_DIR=/home/hyuns/dev/sdks/infiray_sdk/SDK_NET_Linux_aarch64--glibc--stable-2022.08-1_V2.0.0.1/SDK
test -f "$INFIRAY_SDK_DIR/include/IRCNetSDK.h"
test -f "$INFIRAY_SDK_DIR/libs/libIRCNetSDK.so"
test -f "$INFIRAY_SDK_DIR/ability.csv"
test -f "$INFIRAY_SDK_DIR/custom.csv"
file "$INFIRAY_SDK_DIR/libs/libIRCNetSDK.so"
```

`libIRCNetSDK.so`는 `ARM aarch64` 라이브러리로 표시되어야 합니다.
`ability.csv`와 `custom.csv`는 실시간 데이터를 저장하는 파일이 아니라 카메라
모델과 기능을 SDK에 알려주는 정적 설정 파일입니다. 빌드할 때 실행 파일 옆으로
자동 설치되며, 없으면 SDK 로그인이나 초기화가 실패할 수 있습니다.

## 빌드

```bash
cd ~/dev/repos/Infiray_ws

source /opt/ros/humble/setup.bash
export INFIRAY_SDK_DIR=/home/hyuns/dev/sdks/infiray_sdk/SDK_NET_Linux_aarch64--glibc--stable-2022.08-1_V2.0.0.1/SDK

colcon build \
  --packages-select infiray_ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

빌드가 끝나면 현재 터미널에 워크스페이스 환경을 적용합니다.

```bash
source install/setup.bash
ros2 pkg executables infiray_ros2
```

정상이면 다음 세 실행 파일이 표시됩니다.

```text
infiray_ros2 thermal_calibration
infiray_ros2 thermal_camera_fire_detect
infiray_ros2 thermal_camera_fire_detect_raw16
```

새 터미널에서는 실행 전에 환경을 다시 불러와야 합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/dev/repos/Infiray_ws/install/setup.bash
```

`Package 'infiray_ros2' not found`가 나오면 대부분 두 번째 `source`를 하지 않은
경우입니다.

## 카메라 기본 설정

코드에 설정된 기본값은 다음과 같습니다.

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `camera_ip` | `192.168.1.123` | 카메라 IP |
| `camera_username` | `admin` | 로그인 사용자 이름 |
| `camera_password` | `admin` | 로그인 비밀번호 |
| `camera_control_port` | `80` | SDK 제어 포트 |
| `camera_image_fps` | `0` | `0`이면 카메라 FPS를 변경하지 않음 |
| `show_display` | `false` | OpenCV 로컬 화면 표시 |

기본값이 실제 카메라와 같으면 추가 파라미터 없이 실행할 수 있습니다. 다른
카메라 설정을 사용한다면 실행할 때 파라미터를 명시합니다.

## 실행

### 일반 노드

```bash
ros2 run infiray_ros2 thermal_camera_fire_detect
```

OpenCV 화면을 함께 표시:

```bash
ros2 run infiray_ros2 thermal_camera_fire_detect --ros-args \
  -p show_display:=true
```

카메라 접속 정보와 발행률을 직접 지정:

```bash
ros2 run infiray_ros2 thermal_camera_fire_detect --ros-args \
  -p camera_ip:=192.168.1.123 \
  -p camera_username:=admin \
  -p camera_password:=admin \
  -p camera_control_port:=80 \
  -p target_image_fps:=10.0
```

### Raw16/Fieldscale 노드

```bash
ros2 run infiray_ros2 thermal_camera_fire_detect_raw16
```

OpenCV 화면을 함께 표시:

```bash
ros2 run infiray_ros2 thermal_camera_fire_detect_raw16 --ros-args \
  -p show_display:=true
```

Raw16 노드의 기본 발행률은 30 Hz이며 다음 Fieldscale 파라미터를 지원합니다.

| 파라미터 | 기본값 |
| --- | ---: |
| `target_image_fps` | `30.0` |
| `fieldscale_enabled` | `true` |
| `fieldscale_max_diff` | `400.0` |
| `fieldscale_min_diff` | `400.0` |
| `fieldscale_iterations` | `7` |
| `fieldscale_gamma` | `1.5` |
| `fieldscale_clahe` | `false` |
| `fieldscale_video` | `true` |

### AT3003X 기하 캘리브레이션 노드

기존 화재 감지 노드를 먼저 종료한 뒤 실행합니다. 같은 카메라에 두 노드를
동시에 연결하면 안 됩니다.

```bash
ros2 run infiray_ros2 thermal_calibration
```

이 노드는 리사이즈, 컬러맵, Fieldscale, 온도 분석, ROI와 문자 오버레이를
적용하지 않은 SDK grayscale 프레임만 발행합니다. 실행 중에는 카메라 펌웨어의
온도 OSD도 끄고, 종료할 때 이전 OSD 모드로 복원합니다.

| 파라미터 | 기본값 |
| --- | --- |
| `camera_ip` | `192.168.1.123` |
| `camera_username` | `admin` |
| `camera_password` | `admin` |
| `camera_control_port` | `80` |
| `target_image_fps` | `30.0` |
| `camera_info_url` | `file://${ROS_HOME}/camera_info/at3003x.yaml` |
| `frame_id` | `thermal_optical_frame` |
| `invert_image` | `false` |

발행 인터페이스는 다음과 같습니다.

```text
/thermal/image_raw       sensor_msgs/msg/Image (mono8, 384x288)
/thermal/camera_info     sensor_msgs/msg/CameraInfo
/thermal/set_camera_info sensor_msgs/srv/SetCameraInfo
```

영상과 CameraInfo에는 같은 timestamp와 frame ID가 사용됩니다. 미보정 상태에도
CameraInfo의 너비와 높이는 384×288로 발행되며 K, R, P는 0입니다.

체커보드가 열화상에서 선명한지 확인한 다음 실제 내부 코너 수와 한 칸 크기를
사용해 보정합니다. 다음 명령은 내부 코너 8×6, 한 칸 50 mm인 보드의 예입니다.

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.050 \
  image:=/thermal/image_raw \
  camera:=/thermal
```

계산 후 `COMMIT`을 누르면 `/thermal/set_camera_info`가 호출되고 기본 설정에서는
`~/.ros/camera_info/at3003x.yaml`에 K, D, R, P가 저장됩니다. 다음 실행부터 같은
파일을 자동으로 다시 읽습니다.

프로그램은 `Ctrl+C`로 종료합니다. OpenCV 창이 활성화된 상태에서는 `Esc`로
종료하고 숫자 `1`, `2`로 컬러/흑백 표시를 전환할 수 있습니다.

## OpenCV 화면과 SSH

`show_display:=true`는 GUI 세션이 있는 환경에서만 동작합니다.

- Jetson 데스크톱 사용자로 실행: Jetson 모니터에 창 표시
- X11 포워딩 SSH로 실행: 접속한 PC에 창 표시
- VS Code Remote SSH 터미널: 기본적으로 `$DISPLAY`가 없어 창을 표시하지 못함

X11 포워딩은 접속하는 PC의 별도 터미널에서 시작합니다.

```bash
ssh -X hyuns@JETSON_IP
echo "$DISPLAY"
```

`localhost:10.0`과 같은 값이 출력되면 노드를 실행할 수 있습니다. `-X`에서 GTK
창이 열리지 않을 경우 신뢰할 수 있는 본인 소유 Jetson에 한해 `ssh -Y`를 사용할
수 있습니다. Windows에는 MobaXterm 또는 VcXsrv, macOS에는 XQuartz 같은 X
서버가 추가로 필요합니다.

다음 오류는 카메라 문제가 아니라 GUI 화면 연결이 없다는 의미입니다.

```text
Can't initialize GTK backend in function 'cvInitSystem'
```

SSH에서 GUI가 필요 없다면 `show_display`를 생략하거나 `false`로 실행합니다.

## ROS 2 토픽

| 토픽 | 메시지 형식 | QoS | 설명 |
| --- | --- | --- | --- |
| `/thermal/image` | `sensor_msgs/msg/Image` | Best Effort, depth 1 | 열화상 영상 |
| `/thermal/max_temperature` | `std_msgs/msg/Float32` | Reliable, depth 1 | 관심 영역 최고 온도(°C) |
| `/thermal/temperature_trend` | `std_msgs/msg/Float32` | Reliable, depth 1 | 초당 온도 변화량(°C/s) |
| `/thermal/fire_detected` | `std_msgs/msg/Bool` | Reliable, depth 1 | 화재 감지 결과 |

다른 터미널에서 다음처럼 확인합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/dev/repos/Infiray_ws/install/setup.bash

ros2 topic list
ros2 topic hz /thermal/image
ros2 topic echo /thermal/max_temperature --once
ros2 topic echo /thermal/temperature_trend --once
ros2 topic echo /thermal/fire_detected --once
```

정상 실행 로그에는 다음 정보가 표시됩니다.

```text
SDK_NET camera connected
Camera source frame rate: 50 Hz
```

입력이 약 50 Hz이고 일반 노드 발행률이 10 Hz일 때 표시되는 `drop`은 오류나
네트워크 손실이 아닙니다. 최신 프레임만 10 Hz로 발행하면서 의도적으로 건너뛴
이전 프레임 수입니다. `reject 0`이고 입력이 지속적으로 들어오면 정상입니다.

## 대시보드 영상 연동

ROS Bridge와 `web_video_server`가 설치되어 있다면 별도 터미널에서 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/dev/repos/Infiray_ws/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

```bash
source /opt/ros/humble/setup.bash
source ~/dev/repos/Infiray_ws/install/setup.bash
ros2 run web_video_server web_video_server
```

영상 URL:

```text
http://JETSON_IP:8080/stream?topic=/thermal/image
```

## 문제 해결

### `Package 'infiray_ros2' not found`

```bash
source /opt/ros/humble/setup.bash
source ~/dev/repos/Infiray_ws/install/setup.bash
```

`install/setup.bash`가 없다면 아직 빌드하지 않았거나 빌드에 실패한 것입니다.

### SDK 파일을 찾지 못하는 CMake 오류

```bash
export INFIRAY_SDK_DIR=/실제/SDK/디렉터리
colcon build --packages-select infiray_ros2 \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

`INFIRAY_SDK_DIR`는 SDK 패키지의 최상위 압축 해제 경로가 아니라
`include`, `libs`, 두 CSV가 들어 있는 `SDK` 디렉터리여야 합니다.

### 카메라 로그인 또는 초기화 실패

- Jetson과 카메라의 IP 대역 확인
- `camera_ip`, `camera_username`, `camera_password`, 포트 확인
- 설치 디렉터리의 `ability.csv`, `custom.csv` 확인

```bash
ls -l install/infiray_ros2/lib/infiray_ros2/ability.csv
ls -l install/infiray_ros2/lib/infiray_ros2/custom.csv
```

### 라이브러리 로딩 확인

```bash
ldd install/infiray_ros2/lib/infiray_ros2/thermal_camera_fire_detect \
  | grep "not found"
```

아무것도 출력되지 않으면 누락된 공유 라이브러리가 없습니다. 빌드 시 ARM 시스템
라이브러리를 우선하고 SDK 라이브러리를 뒤에 두는 RPATH가 실행 파일에
자동 설정됩니다.

## 구현 참고

- SDK_NET은 RTSP 전송 방식을 내부에서 관리합니다.
- `use_low_latency_rtsp`와 `rtsp_transport`는 기존 실행 명령과의 호환성을 위해
  남아 있지만 SDK_NET의 실제 전송 방식을 변경하지 않습니다.
- 온도 프레임은 SDK가 제공하는 `uint16_t` Kelvin × 10 값이며 코드에서 섭씨로
  변환합니다.
- `pts_0`, `firstPts`, `Get time freq` 등은 벤더 SDK 내부 출력으로 실패
  메시지가 아닙니다.
- `unknown frame type` 또는 종료 시 `NetFramework` 메시지는 영상과 온도 토픽이
  정상이고 프로그램이 종료된다면 대체로 벤더 라이브러리 내부 경고입니다.
