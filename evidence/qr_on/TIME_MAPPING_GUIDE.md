# bag에서 글로벌 장면 단계에 연결하는 방법

카메라 Image/CameraInfo, 두 로봇 GT, `/clock`는 실제 관측한 SimulationManager 글로벌 시각을 사용한다. bag 저장 시각은 DDS 수신 wall 시각이다. LiDAR PointCloud2.header는 원본 센서 packet 시각을 유지하므로 이미지 header와 직접 비교하지 않는다.

`/stage2/session`에는 두 센서의 `runtime_profiles`, 발사 시차와 반환 수, `scene_time_mapping_contract`, 소스/보정 근거 해시가 있다. `/stage2/time_mapping`에서 `type=lidar`인 행의 `sensor`와 `native_header_ns`로 같은 센서의 PointCloud2를 찾는다. 현재 전체 bag 감사에서는 이 키의 유일성도 검사한다.

- `selected_for_completed_state=true`: 두 번 완료 구간의 마지막 정상 nonempty 패킷. 선택은 영상/점 내용 정답을 보지 않는다. 그 외 원시 점군도 topic에 보존된다.
- `observed_clock.step` 및 `scene_time_mapping.observed_scene_stamp_ns`: 그 패킷이 관측된 글로벌 장면 단계/시각이다. 관측만으로 내용 일치를 인증하지 않는다.
- `scene_time_mapping.native_packet_time_ns`: 원래 packet 시각이다. 원본 점 시각은 native packet 시각 + signed offset으로 재구성한다.
- `scene_time_mapping.derived_group_reference_*`: 실제 채널 fireTimeNs를 분리한 분석용 발사 그룹 기준량이다. 원본 발사 시각을 대체하지 않는다. 채널은 `(원본 element_index // max_returns) % channels`다.
- `native_frame_start_ns/end_ns`: 그룹 기준량을 포함하는 native 구간 후보를 찾는 데 사용한다. **전체 bag의 모든 구간**을 조회하고, 서로 다른 글로벌 단계가 후보로 나오면 모호함으로 남긴다. 예상 GT나 QR 정답으로 후보를 선택하지 않는다.

`type=completion` 중 `selected_raw_files`가 있는 행은 글로벌 단계별 최종 선택표와 두 완료 구간의 근거다. `SCENE_MAPPING.json`과 `scene_mapping_points.jsonl`은 저장된 bag 및 원본 raw에서 재계산한 점별 대응 집계다. 사용자는 해당 bag의 감사 결과까지 함께 보아야 한다. runtime의 자기 구간 검사만으로 전체 구간의 모호함 검사를 대체할 수 없다.

검증 해상도는 60Hz 장면 단계다. 나노초 표기는 수치 표현이며 개별 빔의 물리적 취득 정확도는 아니다. 고정 평판 실험에서 얻은 보정 근거와 실제 항공기 bag의 수치 대응 검사는 구분한다. QR 미판독이나 LiDAR의 독립 내용 근거 부족을 자동으로 통과 처리하지 않는다.
