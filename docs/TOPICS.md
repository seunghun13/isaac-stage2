# 최종 18토픽

원본 `integration_common.topics()`와 최종 bag 감사의 실제 메시지 수를 대조했다.

| 토픽 | ROS 타입 | 메시지 수 |
|---|---|---:|
| `/stage2/session` | `std_msgs/msg/String` | 1 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 1 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 1801 |
| `/stage2/time_mapping` | `std_msgs/msg/String` | 26567 |
| `/stage2/drone/waypoints` | `std_msgs/msg/String` | 1 |
| `/stage2/drone/gt_pose` | `geometry_msgs/msg/PoseStamped` | 1801 |
| `/stage2/drone/gt_velocity` | `geometry_msgs/msg/Vector3Stamped` | 1801 |
| `/stage2/drone/lidar/points_raw` | `sensor_msgs/msg/PointCloud2` | 3602 |
| `/stage2/rover/waypoints` | `std_msgs/msg/String` | 1 |
| `/stage2/rover/gt_pose` | `geometry_msgs/msg/PoseStamped` | 1801 |
| `/stage2/rover/gt_velocity` | `geometry_msgs/msg/Vector3Stamped` | 1801 |
| `/stage2/rover/lidar/points_raw` | `sensor_msgs/msg/PointCloud2` | 3602 |
| `/stage2/cameras/cam_01/image_raw` | `sensor_msgs/msg/Image` | 451 |
| `/stage2/cameras/cam_01/camera_info` | `sensor_msgs/msg/CameraInfo` | 451 |
| `/stage2/cameras/cam_02/image_raw` | `sensor_msgs/msg/Image` | 451 |
| `/stage2/cameras/cam_02/camera_info` | `sensor_msgs/msg/CameraInfo` | 451 |
| `/stage2/cameras/cam_03/image_raw` | `sensor_msgs/msg/Image` | 451 |
| `/stage2/cameras/cam_03/camera_info` | `sensor_msgs/msg/CameraInfo` | 451 |

GT pose는 USD TrackingCenter의 실제 readback이며 velocity는 지정 경로에서 계산한 월드 선속도다. 물리 비행/휠 동역학의 측정 속도가 아니다.

PointCloud2에는 원래 native 시각과 signed offset을 보존한다. 같은 장면 단계에서 나온 두 완료 구간의 출력이 함께 기록되므로 raw cloud 메시지 수와 선택 상태 수가 같지 않다. 각 센서의 최종 선택 상태는 1,801개다.

`/clock`과 메시지 header는 시뮬레이션 global 시각을 설명하고, bag recordtime은 DDS 수신 벽시각이다. 두 시간을 같은 의미로 해석하지 않는다.
