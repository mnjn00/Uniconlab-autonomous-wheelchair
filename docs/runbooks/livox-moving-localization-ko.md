# Livox 이동 로컬라이제이션 현장 절차

## 고정 자산

- 정본 지도: `mergedmap.ply`
- 주행용 지도: `merged_0707_0725_0p20m_xyzi.pcd`
- 경로: `20260812_route_v6_v8_waypoints.json` (v6 선호, v8 주행가능 hard mask)
- 기본 IMU: Livox MID-360 내장 IMU
- 최고 주행 속도: 1.0 m/s (2026-08-12 운영자 지시)

`mergedmap.ply`는 원본 보관과 해시 검증에 사용한다. 실제 ICP와 자동
초기화 fallback은 동일한 0.20 m PCD를 사용한다. RViz에서 빨간 지도와
초록 live cloud를 비교하되, 지도는 움직이지 않는다.

## 알려진 시작점 자동 정합

휠체어를 0727 경로의 첫 위치(사자상 원형 부근)에 정지시킨 뒤
`start_wheelchair_localization.sh`를 실행한다. 자동 초기화기는 첫
waypoint를 prior로 먼저 전달한다. 로컬라이저 상태는 `MANUAL_ALIGN`에서
`VERIFYING`으로 바뀌고, 연속 ICP 합의가 성공해야만 `TRACKING`이 된다.
자동 prior가 실패하면 그때만 mapping trajectory 전역 탐색을 수행한다.

`TRACKING` 전에는 wheel base와 follower가 올라오지 않으며, follower는
올라온 뒤에도 PAUSED 상태다. `TRACKING`이 아닌 상태에서 주행을 시작하지
않는다.

## `hdl_localization` 참고 범위

[`koide3/hdl_localization`](https://github.com/koide3/hdl_localization)은 IMU
예측과 NDT scan matching을 결합하고, 초기 자세 입력과 명시적인 global
relocalization을 분리한다. 이 운용 방식 가운데 **알려진 시작 자세를 먼저
넣고 scan matching의 수렴 상태로 승인한 뒤, 실패할 때만 전역 탐색**하는
흐름을 참고했다.

패키지 자체는 도입하지 않는다. 현재 FAST-LIO가 Livox 내장 IMU를 이미
융합하고 있고, 주행 로컬라이저는 GICP/ICP의 연속 합의와 안전 상태를
제공한다. `hdl_localization`을 함께 실행하면 별도의 UKF/NDT, ROS 1
nodelet와 지도 서버가 중복된다. 또한 upstream의 global relocalization은
서비스를 명시적으로 호출해야 하므로 시작점 자동 정합을 그대로 제공하는
대체재도 아니다. 따라서 현재 구성은 알려진 0727 시작점 prior를 우선하고,
기존 전역 탐색을 fallback으로 유지한다.

## 수동 fallback

자동 및 전역 fallback이 모두 실패한 경우에만 RViz의 **2D Pose Estimate**로
현재 위치와 방향을 지정한다. 자동 보정을 명시적으로 켠다.

```bash
rosservice call /fast_lio_icp/enable_auto_correction "data: true"
```

`VERIFYING`에서 `TRACKING`이 되지 않거나 정합이 잘못되면 즉시 보정을 끈다.

```bash
rosservice call /fast_lio_icp/enable_auto_correction "data: false"
```

그 후 지도/현재 cloud, `/Odometry`, `/livox/imu`, 초기 seed와 diagnostics를
기록해 원인을 확인한다. 지도를 live cloud에 맞춰 옮겨서 문제를 숨기지
않는다.
