# 최종 QR-free bag의 시간축 읽기

이 문서는 데이터 해석 안내이며 실행 완료 여부는 RESULTS.md/SUMMARY.json에서 확인한다.

카메라 Image/CameraInfo와 두 로봇 GT의 header는 SimulationManager의 실제 관측 글로벌 시각을 ns로 반올림한 값이다. /clock와 이 시각으로 연결한다. 카메라가 약15Hz, GT/global 상태는60Hz이므로 모든 GT시점에 영상이 있는 것은 아니다. bag record time은 DDS 수신 wall 시각이며 글로벌 취득 시각 대신 쓰지 않는다.

LiDAR PointCloud2.header는 원본 센서 packet 시각이다. 카메라 header와 숫자를 직접 맞추거나 고정 offset을 빼지 않는다. `/stage2/time_mapping`에서 `type=lidar`, `sensor`, `native_header_ns`로 같은 센서의 PointCloud2를 찾는다. 실제 bag 감사가 이 키의 유일성을 확인한다.

- `selected_for_completed_state=true`인 점군은 고정2회 완료 구간에서 마지막 정상nonempty 패킷이다. global 상태별 선택표는 `type=completion`의 `selected_raw_files`에 있다. 선택 전 패킷도 보존돼 있다.
- `scene_time_mapping.observed_scene_step`와 `observed_scene_stamp_ns`가 대응 장면 단계/글로벌 시각이다. 선행 독립 고정평판 실험의 보정과 해당 bag의 전체구간 감사 결과를 함께 사용한다.
- 원본 점 시각은 native packet 시각 + signed timeOffsetNs다. 원본 uint64/offset/index는 변경하지 않았다.
- `derived_group_reference_*`는 원본 점 시각에서 해당 채널의 실제 fireTimeNs를 분리한 분석값이다. 채널은 `(original_element_index // max_returns) % channels`다. 이 값을 전체 native frame 시작~끝 구간에 대조해 global 단계를 찾는다. 원본 발사 시각을 대체하는 정밀 global beam time이 아니다.
- v7의 `zero_valid_return_packet=true`는 정상 구조지만 유효 반환점0개인 패킷이다. width0 PointCloud2와 전체 원본raw를 보존하며 유효점 시각 min/max는null이다. 이 패킷은 nonempty 최종선택에서 제외되지만 native frame 구간은 전체 후보검색에 남긴다. null을0시각이나 통과한 점으로 바꾸지 않는다.

`/stage2/session`은 runtime_profiles, 보정 근거/소스 해시, QR-ON 검증자료 provenance, 카메라·경로·TF를 포함한다. v7의 `ACCEPTANCE.json`과 `EMPTY_RETURN_FIX.md`도 source manifest에 연결된다. 실제 source 디렉터리와 case의 raw sidecar를 함께 보존해야 원본 패킷 감사까지 재현할 수 있다.

허용된 동기화 범위는 카메라/GT/global 수치 일치와60Hz(약16.7ms) LiDAR 장면 단계 대응의 연속성이다. QR 제거 영상은 직접 QR 내용 검증이없음(null)이며, 정밀한 개별 빔 발사 시각과 글로벌 시간의 완전한 일치 인증은false다. QR-ON 자료에 있던 미판독과 native_emission_interval 충돌을 소급 지우지 않는다.
