# Stage2 QR 제거 후 최종 수집

사용자는 QR 판독을 제외한 공통 시간축과 검증된60Hz LiDAR 장면 단계 대응을 설명받은 뒤, 모든 QR 제거와 진행을 요청했다. 이 범위의 최종본을 만들며 과거 all_sensor_acquisition_sync_verified=false, QR 직접 내용 근거, 정밀 native beam/global 시각의 한계는 소급 변경하지 않는다.

별도 v6를 만들고 드론/로버 보드 생성·업데이트·QR TF만 제거한다. 경로·속도1m/s·카메라·60Hz native clock·warmup30회·고정2회 rendering completion·선택 기준·raw header/points·DDS 직렬화는 유지한다. QR 제거로 영상/점군 내용과 렌더 부하가 변할 수 있으며 바이트 동일성을 요구하지 않는다.

원본 v5와 기존 bag 보존 → 소스/경로 비교 → 새 세션 native1s → 회전 포함5s(route4..9) → 실제 bag 감사/18topic replay → 실제 pilot의 시간·저장량으로 예산 확인 → 새 세션 QR-free30s → 전체 bag 읽기 전용 감사. 18토픽 유지, 정적TF는 QR2개 제외7개. QR 판독은 수행하지 않으며 pass/unreadable로 표시하지 않는다.

최종 통과 범위는 ACCEPTANCE.json에 명시한다. 모든 point 원본 시각과 선택 전raw를 보존한다. 단계 대응은 모든 native 구간을 후보로 검사해 선택점이 현재 단계에만 연결돼야 한다. GT 위치로 센서 지연을 평가하지 않는다. 시각 숫자를 임의로 수정하지 않는다. 기존8GiB VRAM/75C시작/85C운영/2TiB증가/1.13TiB잔여 조건을 유지한다. fresh owned restart 전에 GPU/CPU/storage/ownership을 확인하고 다른 사용자의 작업을 건드리지 않는다.

## 초기 빈 반환 처리 수정(v7)

QR-free v6의5초pilot은 시작부의 로버4800항목/유효반환0개 패킷에서 빈 배열min 연산으로0상태/0RGB에 중단됐다. 실패와 원본raw는 보존한다. v7은 유효점0개의 정상구조 패킷을 width0 PointCloud2와 원본raw로 기록하고 유효점 시각min/max를null로 둔다. native frame 구간도 전체 후보검색에 유지한다. 최종 선택은 기존2회 완료 구간의 마지막 정상nonempty로 유지하며 선택변경·시각창작·점삭제·구조검사 완화는 없다. v7에서1초·5초 회귀를 다시 실행한다.
