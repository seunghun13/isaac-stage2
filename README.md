# Isaac Sim Stage2 — 드론·로버 30초 rosbag2 수집

2026-09-20에 최종 수집을 완료한 **QR 제거 v7 소스와 검증 기록**이다. 세 카메라, 드론·로버 GT, 두 LiDAR와 시뮬레이션 시계를 함께 저장한다. 최종 수집에 연결된 핵심 29개 파일은 당시 SHA-256과 동일한 바이트로 보존했다.

이 저장소는 검증된 실행 환경의 소스 스냅샷이다. Isaac Sim 설치본, 항공기·격납고·로봇·센서 에셋, 원본 rosbag2는 별도로 필요하다. 새로운 PC에서의 설치·실행까지 검증한 독립 배포판은 아니다.

## 수집 구성

| 항목 | 최종 설정 |
|---|---|
| 장면 시간 | 30.000001565초, 60Hz, 1,801개 상태 |
| 카메라 | 3대, 각 451 RGB + 451 CameraInfo; 약 15Hz |
| 해상도 | cam_01·cam_03: 3840×2160 / cam_02: 5320×4600 |
| 로봇 | 드론 1대 + 로버 1대, 직선 이동 1m/s, USD로 지정한 이동 |
| 로버 경로 | (2,0) → (2,6) → (8,6) → (8,0) → (2,0), 각 꼭짓점에서 1초 제자리 90° 회전 |
| 드론 경로 | 상승·전진·복귀·착륙의 29.667m 경로 |
| LiDAR | 드론 Ouster OS1 계열, 로버 VLP-16 사양 근사 센서; 각각 3,602개 raw PointCloud2 |
| GT | 각 로봇의 USD TrackingCenter 위치·자세와 경로 기반 월드 선속도 |
| 기록 | 18토픽, 45,486메시지, 정적 TF 7개 |
| QR·색 평판 | 최종 수집에서는 모두 없음 |

실제 추진력·휠 구동 물리로 움직인 실험은 아니다. 최종 30초 장면 수집에는 실제 약 55분이 소요됐다.

## 검증된 시간 대응 범위

`agreed_scene_step_sync_pass=true`이며, 합의한 범위는 **카메라·GT·global 시각의 수치 대응과 보정된 60Hz LiDAR 장면 단계 대응**이다. 최종 30초 동안 1,353개 영상의 시각 메타데이터와 GT/clock 연속성, 선택 LiDAR 점 13,063,574개의 유일한 현재 장면 단계 대응을 확인했다.

원본 LiDAR 시각은 수정하거나 잘라내지 않는다. 원시 발사 시계와 보정된 장면 단계는 구분한다. 다음 과학적 한계는 그대로 남긴다.

- `camera_image_content_time_verified=null`: QR가 없는 최종 영상의 내용을 직접 시간코드로 판독한 결과는 없다.
- `fine_global_beam_emission_verified=false`: 각 빔의 정밀한 글로벌 발사 시각을 인증한 것이 아니다.
- `all_sensor_acquisition_sync_verified=false`: 위의 합의 범위를 초과하는 전체 센서 취득 시각 인증은 하지 않는다.
- 재생 시험은 18토픽 대표 35메시지와 지연 구독 시험이다. 전체 bag의 공식 `ros2 bag play` 검증으로 확대하지 않는다.

자세한 내용: [최종 결과](evidence/final_noqr/RESULTS.md), [시간 대응 해석](evidence/final_noqr/TIME_MAPPING_GUIDE.md), [수집·검증 기준](evidence/final_noqr/ACCEPTANCE.json).

## 코드 안내

| 경로 | 역할 |
|---|---|
| `source/stage2/integration_20260920_v7/` | 최종 취득 루프, 두 로봇 경로, 원시 LiDAR 파서, 시간 대응, ROS 발행·bag 기록, 전체 bag 분석 |
| `source/stage2/scripts/view_hangar_bootstrap.py` | 최종 v7 장면을 여는 Isaac Sim 진입 코드 |
| `source/stage1/collection_20260918_v1/` | 최종 Stage2가 사용하는 시계·렌더링·LiDAR·IPC 보조 코드 |
| `source/stage1/scripts/`, `source/stage1/config/` | 카메라·TrackingCenter·경로 지원 코드와 설정 |
| `source/scripts/`, `source/mro_runtime/` | LiDAR 구조, 파일 경계 검사, 자원 관리 지원 코드 |
| `evidence/final_noqr/`, `evidence/qr_on/` | 최종 QR-free 결과와 선행 QR-ON 검증 근거 |
| `provenance/` | 원본 소스 해시와 게시 파일 출처 |
| `tools/verify_snapshot.py` | 서버 접속 없이 파일·구문·기록 기준 확인 |

읽는 순서: `integration_common.py` → `integration_scene.py` → `integration_runtime.py` → `native_snapshot.py` / `scene_time_mapping.py` → `integration_worker.py` → `audit_integration.py`.

## 로컬 확인

저장소 루트에서 Python 3.10 이상으로 실행한다. 이 명령은 네트워크에 접속하거나 시뮬레이터를 실행하지 않는다.

```console
python -B tools/verify_snapshot.py
```

[실행 환경 및 운영 흐름](docs/ENVIRONMENT.md), [토픽 목록](docs/TOPICS.md), [게시본 변경·제외 항목](docs/PUBLICATION.md)을 먼저 읽는다. 실제 실행은 기존의 준비된 Linux/Isaac Sim 환경과 환경별 자원·소유 프로세스 검사를 전제로 한다.

## 데이터

원본 `bag.db3`는 56,535,818,240바이트(52.653GiB)이며 Git에 포함하지 않는다. 전체 데이터 전달용 압축본과 Drive 전송은 별도 작업이다. 이 저장소 게시가 Drive 업로드 완료를 의미하지 않는다.

최종 DB SHA-256:

```text
02c49c839ea3a6ebe0222998447d016bfb854b9423e5eac01bfb7b17cd1eea15
```

기존 보고서의 서버 경로와 실행 상태는 수집 당시의 기록이다. 저장소를 clone한다고 서버에 접속하거나 기록이 시작되지 않는다.
