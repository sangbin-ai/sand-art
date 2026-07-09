### sand_art

ROS 2 기반의 Sand Art 로봇 프로젝트입니다.
입력 이미지를 스켈레톤화하고, 외곽선/중간 디테일/내부 디테일 3단계 레이어로 분리한 뒤, 각 레이어를 로봇이 따라 그릴 수 있는 좌표 경로로 변환합니다.

## 프로젝트 개요

`sand_art`는 이미지 기반 샌드아트 자동 그리기를 위한 ROS 2 패키지입니다.

주요 처리 흐름은 다음과 같습니다.

1. 이미지를 입력받아 엣지 검출 및 스켈레톤화
2. 스켈레톤 경로를 3개 레이어로 분리
3. 픽셀 좌표를 로봇 베이스 기준 mm 좌표로 변환
4. `SandStroke` 경로 메시지 생성
5. Doosan 로봇 제어 노드로 경로 전달 및 실행

## 주요 기능

* 이미지 엣지 검출 및 스켈레톤 처리
* 외곽/중간/내부 디테일 3단계 레이어 분리
* 스켈레톤 경로를 stroke 단위로 변환
* 로봇 좌표계 기준 waypoint 생성
* ROS 2 service 기반 경로 전달
* `/bond` heartbeat 기반 노드 상태 감시
* 실제 로봇 실행 또는 dry-run 테스트 지원

## 패키지 구조

```text
sand-art/
├── README.md
├── .gitignore
└── src/
    ├── sandart/
    │   ├── launch/
    │   │   └── sandart.launch.py
    │   ├── sandart/
    │   │   ├── lifecycle_manage_node.py
    │   │   ├── path_plan_node.py
    │   │   ├── sandart_movesx_node.py
    │   │   └── skeleton_processor_node.py
    │   ├── package.xml
    │   ├── setup.py
    │   └── setup.cfg
    └── sandart_msgs/
        ├── msg/
        │   ├── Bond.msg
        │   ├── SandPoint.msg
        │   └── SandStroke.msg
        ├── srv/
        │   ├── PathPlanList.srv
        │   ├── ProcessImage.srv
        │   └── TripleLayerdImages.srv
        ├── CMakeLists.txt
        └── package.xml
```

## 노드 설명

### `skeleton_processor_node`

이미지를 입력받아 스켈레톤 기반의 3단계 레이어 이미지로 변환합니다.

주요 역할:

* 이미지 resize
* Gaussian blur
* Sobel edge 검출
* skeletonize 처리
* 외곽/중간/내부 레이어 분리
* 처리 결과를 path planning 단계로 전달

### `path_plan_node`

스켈레톤화된 3개 레이어 이미지를 로봇이 따라갈 수 있는 stroke 경로로 변환합니다.

주요 역할:

* `/triple_layerd_images` 서비스 제공
* `sensor_msgs/Image`를 OpenCV 이미지로 변환
* 연결된 스켈레톤 픽셀을 stroke 단위로 추적
* 픽셀 좌표를 로봇 기준 mm 좌표로 변환
* `SandStroke[]` 형태의 path 생성
* `/dsr01/path_plan_list` 서비스로 경로 전달
* 디버깅용 `strokes_YYYYMMDD_HHMMSS.txt` 저장

레이어 매핑:

| Path    | Layer                   | 의미     | 기본 색상  |
| ------- | ----------------------- | ------ | ------ |
| `path1` | `level0_outer_thickest` | 외곽선    | RED    |
| `path2` | `level1_middle`         | 중간 디테일 | YELLOW |
| `path3` | `level2_inner_thinnest` | 내부 디테일 | BLUE   |

### `sandart_movesx_node`

생성된 path를 받아 로봇 동작으로 실행하는 노드입니다.

주요 역할:

* `/dsr01/path_plan_list` 서비스 제공
* `path1`, `path2`, `path3` 경로 수신
* waypoint 중복 제거
* stroke별 pen-up / pen-down 이동 처리
* 실제 로봇 실행 또는 dry-run 테스트 수행

### `lifecycle_manage_node`

`/bond` 토픽을 구독하여 각 노드의 heartbeat 상태를 감시합니다.

주요 역할:

* heartbeat 수신
* 노드별 마지막 수신 시간 저장
* timeout 발생 시 DEAD 경고 출력
* 노드 재시작 감지

## 메시지 및 서비스

### Message

#### `SandPoint.msg`

```text
float64 x
float64 y
```

로봇 좌표계 기준 2D waypoint를 표현합니다.

#### `SandStroke.msg`

```text
int32 strength
SandPoint[] points
```

하나의 stroke를 표현합니다.
`strength`는 레이어 강도 또는 로봇 압력/깊이 제어용 값으로 사용할 수 있습니다.

#### `Bond.msg`

노드 heartbeat 상태를 전달하기 위한 메시지입니다.

```text
string id
string instance_id
bool active
float64 heartbeat_timeout
float64 heartbeat_period
```

### Service

#### `TripleLayerdImages.srv`

```text
sensor_msgs/Image level0_outer_thickest
sensor_msgs/Image level1_middle
sensor_msgs/Image level2_inner_thinnest
sensor_msgs/Image full_color_skeleton
---
bool accepted
string message
```

스켈레톤 처리 결과인 3개 레이어 이미지를 path planner로 전달합니다.

#### `PathPlanList.srv`

```text
sandart_msgs/SandStroke[] path1
sandart_msgs/SandStroke[] path2
sandart_msgs/SandStroke[] path3
---
bool accepted
string message
```

생성된 로봇 경로를 실행 노드로 전달합니다.

#### `ProcessImage.srv`

```text
string image_path
---
bool accepted
string message
```

이미지 파일 경로를 입력받아 처리 요청을 수행하기 위한 서비스입니다.

## 개발 환경

권장 환경:

* Ubuntu 22.04
* ROS 2 Humble
* Python 3.10+
* OpenCV
* NumPy
* NetworkX
* scikit-image
* Doosan ROS 2 패키지

필요 ROS 2 패키지 예시:

* `rclpy`
* `std_msgs`
* `sensor_msgs`
* `action_msgs`
* `dsr_msgs`
* `dsr_common2`
* `sandart_msgs`

## 설치 방법

ROS 2 워크스페이스를 생성합니다.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

저장소를 clone합니다.

```bash
git clone https://github.com/sangbin-ai/sand-art.git
```

워크스페이스 루트로 이동합니다.

```bash
cd ~/ros2_ws
```

Python 의존성을 설치합니다.

```bash
pip install opencv-python numpy networkx scikit-image
```

ROS 2 패키지를 빌드합니다.

```bash
colcon build
```

환경을 적용합니다.

```bash
source install/setup.bash
```

## 실행 방법

전체 launch 파일로 실행합니다.

```bash
ros2 launch sandart sandart.launch.py
```

개별 노드를 실행할 수도 있습니다.

```bash
ros2 run sandart skeleton_processor_node
ros2 run sandart path_plan_node
ros2 run sandart sandart_movesx_node
ros2 run sandart lifecycle_manage_node
```

## 기본 실행 흐름

1. `lifecycle_manage_node` 실행
2. `sandart_movesx_node` 실행
3. `path_plan_node` 실행
4. `skeleton_processor_node`에서 이미지 처리 요청
5. 3개 레이어 이미지 생성
6. `path_plan_node`가 stroke 경로 생성
7. `sandart_movesx_node`가 경로를 받아 로봇 동작 실행

## 좌표 변환 설정

`path_plan_node.py`의 `CONFIG` 값으로 보드 크기와 로봇 기준 원점을 설정합니다.

```python
CONFIG = {
    "min_stroke_px": 5,
    "resample_mm": 8.0,
    "max_strokes": 20,
    "board_origin_xy": (250.0, -150.0),
    "board_width_mm": 200.0,
    "board_height_mm": 200.0,
}
```

설정 의미:

| 항목                | 설명                     |
| ----------------- | ---------------------- |
| `min_stroke_px`   | 너무 작은 노이즈 stroke 제거 기준 |
| `resample_mm`     | waypoint 간격            |
| `max_strokes`     | 레이어당 최대 stroke 개수      |
| `board_origin_xy` | 로봇 좌표계 기준 보드 원점        |
| `board_width_mm`  | 보드 가로 크기               |
| `board_height_mm` | 보드 세로 크기               |

실제 로봇 환경에 맞게 반드시 보드 원점과 크기를 보정해야 합니다.

## Dry-run 테스트

실제 로봇을 움직이기 전에 dry-run 모드로 경로 생성과 서비스 연결을 먼저 확인하는 것을 권장합니다.

`sandart_movesx_node.py`에서 로봇 실행 여부를 제어하는 설정값을 확인한 뒤, 실제 로봇 동작 전에 `RUN_ROBOT=False` 상태로 테스트하세요.

## 디버깅

`path_plan_node`는 경로 생성 후 다음 형식의 txt 파일을 저장합니다.

```text
strokes_YYYYMMDD_HHMMSS.txt
```

파일에는 path별 stroke와 waypoint 좌표가 저장됩니다.

예시:

```text
=== path1 RED ===
stroke1 (점 92개)
 waypoint1 322.0, 13.5
 waypoint2 328.63, 11.0

=== path2 YELLOW ===
stroke1 (점 30개)
 waypoint1 366.0, -68.0
```

이 파일을 이용해 로봇 실행 전 좌표가 정상적으로 생성되었는지 확인할 수 있습니다.

## 주의 사항

현재 launch 파일에서 실행 패키지명이 `rokey`로 지정되어 있을 수 있습니다.
실제 패키지명이 `sandart`라면 `src/sandart/launch/sandart.launch.py`의 `package` 값을 아래처럼 수정해야 합니다.

```python
Node(
    package="sandart",
    executable="path_plan_node",
    name="path_plan_node",
    output="screen",
)
```

수정 대상 노드:

* `path_plan_node`
* `skeleton_processor_node`
* `lifecycle_manage_node`
* `sandart_movesx_node`

또한 실제 로봇을 연결하기 전에는 다음 사항을 확인해야 합니다.

* 로봇 좌표계와 보드 좌표계 보정
* z축 높이 및 pen-up / pen-down 위치
* 로봇 속도와 가속도 제한
* emergency stop 동작 여부
* dry-run 테스트 결과
* 생성된 waypoint가 작업 영역 안에 있는지 여부

## 향후 개선 사항

* launch 파일 패키지명 정리
* `package.xml` 설명 및 license 정보 업데이트
* 파라미터를 launch argument로 분리
* 이미지 입력 예제 추가
* 샘플 이미지 및 결과 이미지 추가
* 로봇 미연결 환경용 시뮬레이션 모드 강화
* 테스트 코드 보강

## License

현재 license 정보가 명시되어 있지 않습니다.
공개 저장소로 배포할 경우 MIT, Apache-2.0 등 적절한 라이선스를 선택해 추가하는 것을 권장합니다.
