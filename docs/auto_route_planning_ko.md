# 자동 경로 생성 (start + goal → navfn → DWA)

지금까지는 손으로 그린/녹화한 고정 경로(waypoints JSON)와 그 경로를 따라 역산한
안전 밴드만 로드해서, 그 고정선 안에서 DWA가 주행했습니다. 이 워크플로우는
**시작점과 goal만으로 ROS 전역 플래너(navfn)가 경로를 자유롭게 만들되, 절대
떨어지지 않도록** 3D 맵의 낙하/연석/급경사 분석을 costmap에 구워 넣습니다.

## 왜 2D 맵만으로는 안 되는가

`data/hanyang_aegimun_loop/map.pgm`은 높이밴드 투영이라 벽·기둥만 알고 **연석·낭떠러지
정보가 없습니다.** 낙하 안전은 오직 (1) 밴드의 측정된 edge, (2) 3D 맵 통행성 분석
(`terrain_graph.py`의 step/steep/obstruction → `reachable` 설정공간)에만 있습니다.
그래서 자동 생성 경로는 이 3D 분석을 ROS costmap으로 구워 navfn이 낭떠러지 셀을
관통하지 못하게 합니다.

## 방어 깊이 (defence in depth)

1. **costmap (계획 시)**: 낙하/연석/급경사/장애물/미매핑 셀을 전부 치명(lethal)으로.
   navfn(`allow_unknown: false`)이 그 셀을 절대 지나지 못함.
2. **안전 밴드 (런타임)**: 새 경로를 따라 밴드를 다시 만들어 DWA가 롤아웃마다
   밴드 이탈을 거부. costmap이 놓친 것을 한 번 더 막음.

## 파이프라인

```
3D 맵(.pcd)
  └─ bake_dropsafe_costmap.py ──→ dropsafe.pgm + dropsafe.yaml + dropsafe.npz
                                       │ (낙하안전 점유맵, ROS map_server 호환)
                                       ▼
        auto_planner.launch (map_server + move_base/navfn)
                                       │
            make_plan_client.py  (start, goal) ──→ /move_base/make_plan
                                       │
                                       ▼
                                 plan.json (nav_msgs/Path)
                                       │
            path_to_route_assets.py ───┤
              ├─ route waypoints JSON (follower 호환, reference_point=chair_centre)
              └─ make_route_safety_band.py ──→ band.json (새 경로 따라 낙하 측정)
                                       │
                                       ▼
            follower(dwa_follower.py) 가 ROUTE/BAND 파일로 로드 → 주행
```

## 단계별 명령 (NUC)

### 1. 낙하안전 costmap 굽기 (오프라인, 맵 볼륨 연결 후 한 번)

```bash
cd ~/wheelchair_localization_src
python3 tools/bake_dropsafe_costmap.py \
    /Volumes/<map-volume>/merged_0707_0725_v1/merged_0707_0725_0p20m_xyzi.pcd \
    --start -7.85,-2.85 --goal 156.159,-84.341 \
    --out data/dropsafe/dropsafe --corridor-m 20
```

- `--start`/`--goal`: map frame 좌표 (미터). 이 둘을 잇는 선 주변 `--corridor-m`
  폭을 분석한다 (기본 20 m, 우회 여유).
- 산출물: `dropsafe.pgm`(free=254, lethal=0), `dropsafe.yaml`, `dropsafe.npz`(원시 마스크).
- start/goal이 낙하안전 자유공간이 아니면 경고한다 — navfn이 가장 가까운 free
  cell로 스냅하는 것을 막기 위해(연석 끝에서 잘못된 쪽으로 갈 수 있음).

> 같은 costmap으로 여러 goal을 계획할 수 있다. start·goal이 바뀌어도 맵이 같으면
> 다시 구울 필요 없다. 단 분석 반경(`--corridor-m`) 밖의 goal은 커버되지 않는다.

### 2. navfn으로 전역 경로 계획

```bash
# 터미널 A — map_server + move_base/navfn (계획 전용, 주행 X)
source ~/livox_static_localization_ws/devel/setup.bash
roslaunch wheelchair_navigation auto_planner.launch \
    dropsafe_map:=$PWD/data/dropsafe/dropsafe.yaml

# 터미널 B — start·goal 로 make_plan 호출
rosrun wheelchair_navigation make_plan_client.py \
    _start_x:=-7.85 _start_y:=-2.85 \
    _goal_x:=156.159 _goal_y:=-84.341 \
    _out_path:=/tmp/plan.json
```

- `/move_base/make_plan`(nav_msgs/GetPlan)을 호출해 `nav_msgs/Path`를 JSON으로 저장.
- 경로가 없으면(끊어졌거나 start/goal이 free가 아니면) 에러로 끝난다.
- `cmd_vel`은 `/cmd_vel_nav_unused`로 리매핑해 **절대 구동하지 않는다.**

### 3. Path → follower 자산 변환 + 밴드 생성

```bash
python3 tools/path_to_route_assets.py /tmp/plan.json \
    --costmap data/dropsafe/dropsafe.npz \
    --out-route routes/auto_route_waypoints.json \
    --out-band-prefix routes/auto_route_safety_band \
    --map-pcd /Volumes/<map-volume>/merged_0707_0725_v1/merged_0707_0p20m_xyzi.pcd \
    --body-frame-profile builtin --step 0.2
```

- 웨이포인트를 0.2 m 간격으로 재샘플, 접선 yaw 계산, costmap 지면에서 z 샘플링.
- `make_route_safety_band.py`가 새 경로를 따라 3D 맵에서 낙하 edge를 다시 측정해
  밴드를 만든다 → DWA 런타임 낙하방지가 새 경로를 커버.

### 4. follower 구동 (기존과 동일, 파일만 교체)

```bash
ROUTE=~/wheelchair_localization_src/routes/auto_route_waypoints.json \
BAND=~/wheelchair_localization_src/routes/auto_route_safety_band.json \
PROFILE=dwa \
./tools/start_wheelchair_localization.sh
./tools/go.sh
```

## 안전 한계 (변하지 않는 것)

- 이 워크플로우는 **소프트웨어 릴리스 권한, 캠퍼스 운행 권한, 탑승 권한을 부여하지
  않는다.** `contracts/wp0/A16-release-authority.yaml`의 모든 게이트는 여전히 미승인.
- costmap의 낙하 분석은 커밋된 3D 맵의 해상도(0.20 m 복셀)에 묶인다. 맵이 보지
  못한 낙하는 밴드도 costmap도 잡지 못한다 — 운전자 조이스틱이 최후의 실패세이프.
- 분석 반경(`--corridor-m`) 밖의 지형은 치명 셀로 처리된다(미매핑=위험).
- 밴드의 edge는 **측정된** 것이어야 안전하다. v5 밴드처럼 손그림 only edge면
  costmap이 1차 방어이고 밴드는 보조만 한다 — costmap이 거부한 셀을 밴드가 다시
  열 수는 없다(밴드는 좁히기만 하므로).
