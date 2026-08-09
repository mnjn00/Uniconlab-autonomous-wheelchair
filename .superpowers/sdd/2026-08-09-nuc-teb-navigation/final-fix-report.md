# NUC TEB Navigation 최종 리뷰 수정 보고서

기준 커밋: `66a141fb4f7da255bc7bb13fefe42981ebd7ec4d`
브랜치: `codex/nuc-teb-navigation`

## Finding 처리

1. Important — `docs/nuc_teb_navigation.md`
   - `/cmd_vel`을 이미 구독 중인 노드가 동일 ROS master에서 메시지를 소비하면 실제 모터가 구동될 수 있음을 명시했다.
   - 로컬 검사는 격리된 ROS master 또는 시뮬레이션, 하드웨어 비활성화 또는 물리적 분리, launch 전 `rostopic info /cmd_vel`로 subscriber 부재 확인을 모두 충족해야 한다고 문서화했다.
   - 기존 로컬 launch 명령과 `rostopic info /cmd_vel`, `rostopic echo /cmd_vel` 명령은 변경하지 않았다.

2. Minor — `src/wheelchair_navigation/config/teb_local_planner.yaml`
   - `min_obstacle_dist: 0.6` 값은 변경하지 않고, active polygon 경계 밖 clearance라는 정확한 주석으로 바꾸었다.

3. Minor — `src/wheelchair_navigation/tests/test_navigation_static.py`
   - third-party `yaml` import를 표준 라이브러리 import 뒤의 별도 그룹으로 분리했다.

4. Minor — `docs/reference/nuc_teb/teb_local_planner.yaml`
   - 수정하지 않았다. NUC SHA 보존에 필요한 trailing spaces는 의도적으로 유지했다.
   - `test_nuc_teb_reference_snapshots_match_source_hashes`가 reference의 정규화 LF SHA-256을 검증했고, `git diff --name-only -- docs/reference/nuc_teb/teb_local_planner.yaml`은 출력이 없었다.

## 검증

실행 명령:

```powershell
py -3 -m pytest -q src/wheelchair_navigation/tests/test_navigation_static.py::test_nuc_teb_reference_snapshots_match_source_hashes src/wheelchair_navigation/tests/test_navigation_static.py::test_active_teb_preserves_nuc_tuning_except_github_footprint src/wheelchair_navigation/tests/test_navigation_static.py::test_navigation_launch_routes_move_base_to_raw_nav_topic_only src/wheelchair_navigation/tests/test_navigation_static.py::test_teb_runtime_dependencies_and_launch_graph_are_explicit
```

정확한 결과:

```text
pytest 9.1.1
....                                                                     [100%]
============================== warnings summary ===============================
..\\..\\..\\AppData\\Roaming\\Python\\Python314\\site-packages\\_pytest\\cacheprovider.py:469
  C:\\Users\\npgy2\\AppData\\Roaming\\Python\\Python314\\site-packages\\_pytest\\cacheprovider.py:469: PytestCacheWarning: could not create cache path C:\\Users\\npgy2\\Documents\\전동휠체어\\Uniconlab-autonomous-wheelchair-nuc-teb-impl\\src\\wheelchair_navigation\\.pytest_cache\\v\\cache\\nodeids: [WinError 5] 액세스가 거부되었습니다: 'C:\\Users\\npgy2\\Documents\\전동휠체어\\Uniconlab-autonomous-wheelchair-nuc-teb-impl\\src\\wheelchair_navigation\\pytest-cache-files-z32n7yhi'
    config.cache.set("cache/nodeids", sorted(self.cached_nodeids))

-- Docs: https://docs.pytest.org/en/stable/how-to/capture-warnings.html
4 passed, 1 warning in 0.29s
```

추가 확인:

- `git diff --check` 출력 없음: 이번 수정 범위의 whitespace 오류 없음.
- `git diff --name-only -- docs/reference/nuc_teb/teb_local_planner.yaml` 출력 없음: trailing spaces가 있는 reference는 예외가 아니라 미변경 상태로 보존됨.
- 문서에서 ROS master/subscriber 위험, 세 가지 사전 조건, 기존 launch/info/echo 명령을 확인했다.

## 커밋

단일 최종 커밋: `docs: address final NUC TEB review findings`
