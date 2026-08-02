# 2026-08-02 주행 기록

## 요약

- route v4 웨이포인트와 안전 회랑을 차량에 배포했고, 장애물 거리를 bounding box가 아니라 실제 LiDAR return 기준으로 계산하도록 수정했다.
- 21:17~21:58 사이의 완료된 blackbox 3개를 보존했다. 합계 1,126,515개 메시지, 약 36분 45초 분량이다.
- 세 번째 기록에서 주행 상태는 waypoint 0에서 시작해 최대 1126까지 진행했다. waypoint 536~548 및 919~946 부근에서 장애물 상태가 반복됐고, 긴 정지는 주로 `MANUAL_MODE`와 `PAUSED` 상태였다.
- 위치추정 진단은 세 기록 모두 `TRACKING`을 유지했다. 세 번째 기록에서 level 0의 `CLOUD_ODOMETRY_TIME_MISMATCH`가 단발성으로 7회 기록됐지만 `LOST`나 `DEGRADED` 전환은 없었다.
- 전역/로컬 위치추정 실험과 shadow-safety 프로토타입은 완료되지 않아 이번 기록 브랜치에 포함하지 않았다.

## 작업 및 주행 타임라인

| 시각 (KST) | 내용 |
|---|---|
| 19:00경 | 전체 토픽에 가까운 debug 녹화와 blackbox 녹화가 비정상 종료되어 `.bag.active` 파일 2개가 남음. NUC에 원본 그대로 보존함. |
| 20:37 | 장애물 거리 계산을 bounding box가 아닌 실제 return 기준으로 바꾸는 커밋 `5599d30` 작성. |
| 20:42 | route v4 웨이포인트와 안전 회랑 배포 커밋 `9092d10` 작성. |
| 21:17:46~21:22:18 | 주행 기록 1. waypoint 2에서 시작해 최대 48까지 진행. 21:19:42부터 기록 종료까지 약 156.1초 정지. |
| 21:24:37~21:35:21 | 주행 기록 2. 전체 구간이 `HOLD:PAUSED`로 기록되어 실제 경로 진행은 확인되지 않음. |
| 21:36:34~21:58:02 | 주행 기록 3. waypoint 0에서 최대 1126까지 진행. 장애물 상태 30개 구간과 실제 정지 18개 구간이 검출됨. |

## 보존한 bag

| 파일 | 크기 | 길이 | 메시지 | 주행 상태 요약 |
|---|---:|---:|---:|---|
| `blackbox_20260802_211746.bag` | 6,635,839 B | 271.888초 | 138,890 | wp 2~48, 장시간 일시정지 1회 |
| `blackbox_20260802_212436.bag` | 15,473,339 B | 644.772초 | 329,378 | 전 구간 `PAUSED`, 진행 확인 불가 |
| `blackbox_20260802_213633.bag` | 33,422,416 B | 1,288.399초 | 658,247 | wp 0~1126, 장애물/수동/일시정지 구간 포함 |

세 파일은 모두 index가 정상이고 LZ4로 압축되어 있다. SHA-256, 정확한 시작/종료 시각 및 차량 상태는 [`blackbox/manifest_20260802.json`](../blackbox/manifest_20260802.json)에 기록했다.

## 세 번째 주행 상세

- 초기 `PAUSED`: 21:36:34~21:37:50, waypoint 0.
- 첫 장애물 상태 군집: 21:42:05~21:42:20, waypoint 536~553 부근. 개별 상태 구간은 대부분 1초 안팎이었다.
- 장시간 수동/일시정지: 21:45:29~21:50:50, waypoint 899~917, 실제 속도 기준 약 321.1초 정지.
- 두 번째 장애물 상태 군집: 21:50:53~21:51:33, waypoint 919~946. 가장 긴 연속 장애물 상태는 약 5.8초였다.
- 장시간 수동/일시정지: 21:51:34~21:54:22, waypoint 946~1049, 실제 속도 기준 약 167.8초 정지.
- 마지막 장시간 정지: 21:54:49~기록 종료, waypoint 1125~1126, 약 192.7초. 기록 종료 전 재출발은 확인되지 않았다.
- 실제 속도 기준 정지 18개 구간의 합은 약 706.7초이며, 20초 미만의 짧은 정지 합은 약 25.1초다.
- `tip_guard/status`에서 비정상 상태는 없었고, `/perception/objects_summary`의 status 6,442개는 모두 `OK`였다.

자동 추출 결과 원본은 다음 파일에 있다.

- [`recorded_stop_warning_analysis.json`](../blackbox/analysis_20260802/recorded_stop_warning_analysis.json): bag별 상태, 실제 정지, 위치추정 진단, 모드 변경
- [`recorded_stop_warning_events.csv`](../blackbox/analysis_20260802/recorded_stop_warning_events.csv): 상태 이벤트 표

## 녹화된 토픽과 한계

완료된 세 blackbox에는 다음 13개 토픽이 공통으로 들어 있다.

`/Odometry`, `/cmd_vel`, `/cmd_vel_gated`, `/cmd_vel_raw`, `/fast_lio_icp/localization_diagnostics`, `/fast_lio_icp/pose`, `/livox/imu`, `/mode_cmd`, `/perception/objects_summary`, `/tip_guard/status`, `/waypoint_follower/status`, `/wheel_cmd`, `/wheel_status`

따라서 오늘 발행된 모든 ROS 토픽이 Git에 올라간 것은 아니다. 특히 원시/대용량 토픽인 `/livox/lidar`, `/cloud_registered_body`, `/tf`, `/tf_static`은 완료된 blackbox 3개에 없다.

전체 토픽에 가까운 녹화는 NUC의 `/home/mprp3/localization_trials/debug_20260802_190304.bag.active`에 남아 있다. 약 31.4 GB이며 정상 종료되지 않아 index가 없는 `.active` 상태라 이번 Git LFS 업로드에서는 제외했다. 같은 위치의 `blackbox_20260802_190040.bag.active` 약 126 MB도 동일하게 제외했다. 두 원본은 삭제하거나 변경하지 않았다.

bag 자체에는 사용한 route의 식별자나 hash를 기록하는 토픽이 없다. 차량에서 확인한 당시 저장소 기본 route는 v4였지만, 이는 bag 내용만으로 재구성한 결론은 아니다.

## 안전 로직 관련 결정

오늘 논의한 시험 방향은 다음과 같다.

- 장애물 정지는 계속 실제 정지로 동작시킨다.
- 나머지 정지 로직은 삭제하지 않고 시험 중에만 끈다.
- 꺼진 로직이 켜져 있었다면 발행했을 정지 판단을 shadow 데이터로 남겨 bag에 저장한다.

이 방향의 로컬 프로토타입은 아직 설치 및 시험이 끝나지 않았다. 오늘의 차량 주행 기록과 혼동되지 않도록 이번 기록 브랜치에는 넣지 않았다.

## 미완료 및 다음 작업

- 차량 작업 트리의 전역/로컬 위치추정 관련 수정 4개 파일은 미완료 상태로 남아 있으며 별도 세션에서 이어서 검증한다.
- `.bag.active` 두 파일이 필요하면 NUC에서 복사한 뒤 `rosbag reindex` 등으로 복구 가능성을 먼저 확인하고, 31 GB 원본은 Git이 아닌 별도 대용량 저장소에 보관한다.
- 다음 주행 전 blackbox 토픽 목록에 route ID/hash와 safety mode/shadow decision 토픽을 추가해, 어떤 경로와 안전 설정으로 달렸는지 bag만으로 확인할 수 있게 한다.
- shadow-safety 구현을 완료한 뒤 장애물 정지만 실제로 유지되는지, 나머지 정지 판단이 주행을 막지 않으면서 bag에 남는지 재검증한다.
