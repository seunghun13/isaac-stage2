# 실행 환경과 운영 흐름

## 확인된 환경

- Linux, NVIDIA RTX A6000의 GPU 1을 사용한 기존 MRO 환경.
- Isaac Sim 5.0과 포함된 Kit Python 3.11, `carb`, `omni`, `pxr`, RTX LiDAR 및 ROS 2 bridge 확장.
- ROS 2 Humble 메시지와 `rclpy`, Fast DDS, NumPy. 전체 bag 분석에는 `rosbags==0.11.5`, OpenCV도 사용한다.
- 프로세스 격리에 `bubblewrap`(`bwrap`)을 사용한다.
- `source/`는 서버의 MRO 프로젝트 루트 아래 상대 배치를 보존한다. 수집 원본 코드의 기준 루트는 `/mnt/DATA/workspace/ws_minho/mro_1`이다. Isaac 설치 디렉터리의 기존 이름은 코드에서 `issacsim`이다.

Python 패키지만 설치해서 수집할 수 있는 프로젝트는 아니다. Isaac 설치본·ROS bridge·USD 에셋의 버전과 준비된 장면이 함께 필요하다. 별도 호스트로 이식하거나 경로를 바꾸면 신규 검증 대상이다.

## 별도로 준비해야 하는 항목

1. `hangar/scene.usda`와 연결되는 격납고·항공기·로봇 에셋, RTX 센서 USD 및 텍스처. 이 저장소의 USDA는 구성 레이어이며 전체 에셋이 아니다.
2. 설치된 Isaac Sim, ROS bridge, NumPy·OpenCV·rosbags 실행 환경.
3. 해당 호스트의 `config/launch_policy.json`, `config/runtime_environment.json`, `config/storage_budget.json`, `manifests/preparation_result.json`.
4. Stage2의 `config/viewing_session.json`, 배포 release/source-origins manifest와 시작/종료 소유권·잠금 정보. PID, GPU UUID, 저장소 기준값과 실행 ID는 과거 값을 재사용하지 않는다.
5. 해당 배포에 대해 검증된 `stage2/manifests/noqr_stage2_30s_readiness.json`. 저장소의 역사적 보고서로 새로운 실행 준비 완료를 대신하지 않는다.

호스트 설정이나 비밀번호·SSH 키·토큰은 포함하지 않았다. `stage2_supervisor.py`의 공개 엔드포인트는 게시용 문서 주소 `192.0.2.1`로 치환했으므로 실제 호스트에서 그대로 실행하는 설정이 아니다. 전체 프로젝트 루트를 덮어쓰는 설치 스크립트는 제공하지 않는다.

## 기존 환경에서의 실행 흐름

1. 원본 장면·이전 bag을 보존하고, 현재 GPU/CPU/스토리지와 소유 프로세스를 확인한다. 기존 감독기는 GPU 1 전용 실행, 최소 여유 VRAM, 온도 및 저장소 경계를 검사한다.
2. 준비된 호스트에서 `start_stage2.py`가 감독기와 새 Isaac 세션을 시작한다. `--execute`가 없으면 자원 조회만 한다. bootstrap이 v7 장면을 열고 `stage2_open` 상태를 기록한다.
3. 새 case에 `spec.json`과 `request.json`을 준비한 뒤 `integration_driver.py`가 구독·발행 프로세스를 시작한다. DDS 준비 확인 후 bootstrap에 요청을 전달한다. 기존 세션은 한 번 사용하면 재사용하지 않는다.
4. 허용 프로파일은 1초(`ticks=60`, offset 0), 회전 포함 5초(`ticks=300`, offset 4), 최종 30초(`ticks=1800`, offset 0)이다. 60Hz, camera stride 4, native 해상도, completion cycles 2, QR·색 평판 OFF를 유지한다.
5. bootstrap → `integration_runtime.execute()`가 global 시계, 렌더 완료, 센서 취득 및 USD 이동을 진행한다. worker는 18토픽의 원본 CDR을 SQLite rosbag2로 저장한다.
6. case와 raw를 읽기 전용으로 두고 `audit_integration.py CASE NEW_AUDIT_DIRECTORY`로 분석한다. 분석 출력은 별도 새 디렉터리에 쓴다. `audit_scene_mapping.py`가 전체 native 구간을 후보로 점 대응을 검사한다.

원래 운영자의 수집 요청 코드는 [run_capture_original.py](../reference/run_capture_original.py)에 보존했다. 이 파일은 당시 Windows/WSL SSH 어댑터를 통해 서버 코드를 전달한 기록이며, 어댑터와 인증 설정은 포함하지 않는다. 업로드된 파일을 실행해 새 수집을 시작한 것은 아니다.

## 경로 복원 단계와 구분

일부 카메라에서 로봇이 일시적으로 보이지 않는 것은 허용한다. 로봇별로 세 카메라 모두에서 사라지는 구간은 별도 확인해야 한다. 이후 경로 복원 성능을 평가할 때 GT와 지정 경로는 평가 기준으로만 사용하고 복원 입력으로 누설하지 않는다. 위치·재투영 오차를 센서 시간 지연의 증거로 사용하지 않는다.
