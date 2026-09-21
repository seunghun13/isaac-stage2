# Stage2 QR 없는 최종30초 rosbag2

**드론·로버 QR을 모두 제거한 별도v7에서1초·회전 포함5초 회귀, 최종30초 수집과 전체bag 검사를 완료했다. 사용자가 합의한 카메라/GT/global 시각 및 보정된60Hz LiDAR 장면 단계 대응 범위에서 최종 수집본이다.**

원본 LiDAR 발사 시각의 정밀한 글로벌 변환이나 QR-free 영상 내용의 직접 시간코드 검증을 새로 통과했다고 표시하지 않는다. `camera_image_content_time_verified=null`, `fine_global_beam_emission_verified=false`, `all_sensor_acquisition_sync_verified=false`는 유지한다. 이번 범위의 완료 판정은 `agreed_scene_step_sync_pass=true`이며 [ACCEPTANCE.json](ACCEPTANCE.json)에 사용자 합의 범위를 명시했다.

## 최종 데이터 위치

서버 case: `/mnt/DATA/workspace/ws_minho/mro_1/stage2/outputs/integrated7_noqr_final01_24e1e6cc10`

rosbag2: `/mnt/DATA/workspace/ws_minho/mro_1/stage2/outputs/integrated7_noqr_final01_24e1e6cc10/bag`

DB: 56,535,818,240bytes (52.653GiB), 45,486메시지,18토픽. 취득은 시뮬레이션 30.000001565초, 실제 55.02분이었다. DB SHA256: `02c49c839ea3a6ebe0222998447d016bfb854b9423e5eac01bfb7b17cd1eea15`.

전체 case의 `raw/`, states/images/native_events, producer/driver 보고서, spec 및 원본 소스 해시를 함께 보존한다. bag 내부 session에는 소스/설정 해시, QR-ON 검증 자료의 해시, 센서 실제 발사 설정과 시간 대응 의미가 들어 있다. raw점군의 원본 uint64 점 시각/offset/index 및 선택 전 출력은 보존됐다.

## 수집 항목

- 세 카메라 각각451RGB와451CameraInfo: cam01/03=3840×2160, cam02=5320×4600, rgb8, 약15Hz 시뮬레이션 간격.
- 드론·로버 각각1801 GT PoseStamped와1801 trajectory velocity: 실제 USD TrackingCenter readback, 추진/엔진/휠 물리운동 아님.
- 두 독립 LiDAR raw PointCloud2: 드론 3,602개, 로버 3,602개. 두 완료 구간의 정상 출력은 모두 저장하고 각 global 단계의 최종 선택표는 별도 유지한다.
- /clock1801개, time_mapping, session, 두 waypoint, /tf_static. QR용 두 변환만 빠져 정적TF7개다.

드론은 원래29.667m 경로, 로버는 XY(2,0)→(2,6)→(8,6)→(8,0)→(2,0)의6m 네모 경로다. 둘 다 직선1m/s USD 이동이며 로버는 꼭짓점에서1초간 제자리90° 회전한다. 경로 확인은 이동 계획 검증이며 센서 시간 지연 기준으로 사용하지 않았다.

## QR 제거 회귀

| 수집 | 시뮬레이션 초 | global/GT 상태 | RGB | 저장 점군 | 실제 초 |
|---|---:|---:|---:|---:|---:|
| smoke02 | 1.000000052 | 61 | 48 | 244 | 141.6 |
| pilot02 | 5.000000261 | 301 | 228 | 1,204 | 577.5 |
| final01 | 30.000001565 | 1,801 | 1,353 | 7,204 | 3301.2 |

[SOURCE_DIFF.patch](SOURCE_DIFF.patch)는 QR 생성/갱신, QR TF, 검증 메타데이터 변경을 보여준다. clock·warmup·두 번 완료 순서·카메라 취득·원본 점군 파서·선택 규칙·DDS 직렬화는 보존됐다. 실제1초/5초 bag에서 이전 QR-ON과 상대 시각 시퀀스·카메라 설정·경로·GT·센서 발사 설정·QR 외 정적TF를 대조했다. [REGRESSION_COMPARISON.json](REGRESSION_COMPARISON.json)에 근거가 있다.

활성 장면의 잔여 QR/색 평판 거부 검사, 저장session layer,7TF, bag 영상 미리보기에서 부재를 확인했다. QR 제거로 RGB와 LiDAR 반환점은 달라질 수 있으며 동일 픽셀/동일 점군을 요구하지 않았다. QR가 없는 영상을 미판독/QR시간검증pass로 집계하지 않았다.

v6의 첫5초pilot은 유효 반환0개 패킷의 빈 배열min 처리 오류로0상태/0RGB에서 중단됐으며 실패와 원본4800항목을 보존했다. v7은 정상구조/유효반환0개 패킷을 width0 PointCloud2와 원본raw로 남기고 유효점시각범위를null로 표시한다. 그 frame 구간도 전체 후보검색에 남긴다. 마지막nonempty 선택 규칙, 타임스탬프, 원본 구조검사는 그대로다. 실제 실패원본의 오류 재현/수정 검사 및 새1초/5초 회귀를 통과했다. 최종bag의 유효반환0개 패킷은 0개다. [EMPTY_RETURN_FIX.md](EMPTY_RETURN_FIX.md)와 [EMPTY_RETURN_DIFF.patch](EMPTY_RETURN_DIFF.patch)에 변경 근거가 있다.

## 30초 시간 대응

모든1353영상의 native/reference/header/global 수치 최대 차이 0ns, 두GT 및clock1801단계 연속, 두LiDAR reference/global 최대 차이 0ns다. 이 숫자는 물리적 나노초 정확도를 뜻하지 않는다.

아래는 선택점의 발사 그룹 기준량을 전체bag의 모든 native frame 구간에 대조한 결과다. 기대GT/QR 위치로 대응 후보를 고르지 않았다.

| 센서 | 선택 패킷 | 선택 유효 점 | 현재 단계만 | 다른 단계만 | 여러 단계 | 구간 없음 |
|---|---:|---:|---:|---:|---:|---:|
| rover | 1,801 | 8,356,848 | 8,356,848 | 0 | 0 | 0 |
| drone | 1,801 | 4,706,726 | 4,706,726 | 0 | 0 | 0 |

이는 native emission 값을 그대로 global 시각으로 바꾸는 방법이 아니다. 원본 packet+signed offset 시각은 그대로 저장하고, 실제 채널 fireTimeNs를 분리한 파생 기준량으로 장면 단계를 찾는다. 단계 해상도60Hz, 약16.7ms다. 선행 고정 평판의 독립 코드 실험을 보정 근거로 연결했고 이번 전체30초에서는 수치 대응의 연속성을 검증했다. `native_emission_interval`의 과거 구간 없음34점/코드 충돌320점을 없었던 결과로 바꾸지 않는다.

이번bag에서도 원본 발사 시각이 자체native frame 구간 밖인 점은 드론 25,165개, 로버 0개이며 모두 원본 그대로 보존했다. 위 표의0건은 최종 선택점의 파생 장면 단계 대응 결과이므로 이 원본 발사 시각 통계와 혼동하지 않는다. 전체 저장 유효점은 드론 9,481,166개, 로버 16,734,195개다.

## 저장 및 재생 검증

모든 RGB 원본 해시, PointCloud2 원본 payload/점 시각, publisher→DDS→bag CDR, GT/CameraInfo/TF/경로/session을 검사했다. SQLite quick_check 통과, 감사 전후 DB 전체SHA/mtime/size와 metadata 보존 통과. 제한된 재생 시험은18토픽 대표35메시지의 정확한CDR와 지연 구독시 설정/TF/두경로 수신을 확인했다. 전체bag 재생 또는 공식 ros2bag CLI 검증으로 확대하지 않는다.

이전 QR-ON30초 case 및 Stage1 자료, 이전Stage2 소스와 실패 사례를 보존했다. 새source는 `stage2/integration_20260920_v7`, 이전v5/v6 bootstrap은 각각 `stage2/manifests/noqr_initial_revision`, `stage2/manifests/noqr_empty_return_initial_revision`에 보존됐다. local 증거는 전체rawbag을 복사한 것이 아니라 해시 확인한 보고서/시간표/미리보기다.

2026-09-20 23:24:00 KST: Isaac Sim 일시정지, 기록·분석worker없음, SSH연결 유지, 사용완료세션이다. Stage1release37/protected23, Stage2release와 이전버전해시, 이전bagstat/metadata, GPU1전용 ROroot/ROMRO/RWStage2를 재확인했다. 추가수집/자동재시도 없음.

이번 합의 범위의2단계 수집은 완료다. 다중 카메라 음영 구간의 경로 복원 성능 평가는 이후 데이터 활용 단계이며, 이번 최종bag 수집의 미완료 조건으로 남기지 않는다.
