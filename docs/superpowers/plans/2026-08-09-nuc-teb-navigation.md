# NUC TEB Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Preserve the GitHub wheelchair stack and replace only its active `move_base` local planner with a provenance-checked port of the NUC TEB configuration.

**Architecture:** Keep the repository's Navfn, TF/URDF, costmaps, localization, map, bringup, and guarded command topology. Store exact NUC source snapshots under documentation, derive the active TEB configuration from that snapshot with one explicit footprint adaptation, and keep the launch output topic configurable so integrated runs use `/cmd_vel_nav` while local inspection can request `/cmd_vel`.

**Tech Stack:** ROS 1 Noetic, `move_base`, `teb_local_planner`, `costmap_converter`, ROS launch XML, YAML, Python 3, pytest.

## Global Constraints

- Branch: `codex/nuc-teb-navigation`, based on GitHub `main` commit `a54f130`.
- Do not change the repository's TF, URDF, costmaps, localization, maps, safety gate, or hardware authorization model.
- Do not add Pure Pursuit, S-curve waypoint following, motor enablement, UART, MQTT, or NUC deployment.
- Preserve the NUC source snapshots with normalized-LF SHA-256 values `dec3b50729fe9c139b6e7aaead24ce33ab39862a55538aefdc710acedbf0dc3c` and `eec4ca2a275b53eb214ece71c74fc1557856c0286e10890aa419fbd383b6905a`.
- Preserve all NUC TEB tuning values except `footprint_model`; the active footprint must match the GitHub costmap rectangle.
- Keep `/cmd_vel_nav` as the launch default. Local direct inspection uses `cmd_vel_nav_topic:=/cmd_vel` without adding a motor endpoint.
- Keep the legacy DWA configuration file intact, but do not load it from the active navigation launch.
- Follow TDD: add a focused failing test, observe the expected failure, make the minimum implementation, rerun, then commit.
- Existing Windows baseline: the map SHA test fails only because `core.autocrlf=true` checks `map.yaml` out with CRLF; LF normalization produces the committed expected hash. Do not alter the map for this feature.

---

## File Map

- Create `docs/reference/nuc_teb/move_base.yaml`: byte-content snapshot of the NUC `move_base.yaml`, with LF normalization allowed for cross-platform checkout.
- Create `docs/reference/nuc_teb/teb_local_planner.yaml`: byte-content snapshot of the NUC TEB file.
- Create `docs/reference/nuc_teb/SHA256SUMS`: source host, source paths, acquisition date, and normalized-LF hashes.
- Create `src/wheelchair_navigation/config/teb_local_planner.yaml`: active TEB configuration, identical to the source tuning except for the GitHub polygon footprint.
- Modify `src/wheelchair_navigation/config/move_base.yaml`: change only `base_local_planner` from DWA to TEB.
- Modify `src/wheelchair_navigation/launch/navigation.launch`: load the TEB configuration instead of the DWA configuration.
- Modify `src/wheelchair_navigation/package.xml`: add TEB and costmap converter runtime dependencies.
- Modify `src/wheelchair_navigation/tests/test_navigation_static.py`: verify provenance, adaptation, plugin selection, dependency declarations, launch graph, and output-topic contract.
- Create `docs/nuc_teb_navigation.md`: document provenance, active adaptation, local `/cmd_vel` inspection, guarded bringup, and non-goals.

### Task 1: Preserve and authenticate the NUC source files

**Files:**
- Create: `docs/reference/nuc_teb/move_base.yaml`
- Create: `docs/reference/nuc_teb/teb_local_planner.yaml`
- Create: `docs/reference/nuc_teb/SHA256SUMS`
- Modify: `src/wheelchair_navigation/tests/test_navigation_static.py`

**Interfaces:**
- Consumes: read-only SSH access to `mprp3@10.242.33.199`.
- Produces: immutable reference YAML used by Task 2 and normalized-LF checksum assertions.

- [ ] **Step 1: Add the failing provenance test**

Add these definitions to `test_navigation_static.py`:

```python
REFERENCE_TEB = ROOT / "docs" / "reference" / "nuc_teb"


def _normalized_lf_sha256(path):
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def test_nuc_teb_reference_snapshots_match_source_hashes():
    assert _normalized_lf_sha256(REFERENCE_TEB / "move_base.yaml") == (
        "dec3b50729fe9c139b6e7aaead24ce33ab39862a55538aefdc710acedbf0dc3c"
    )
    assert _normalized_lf_sha256(REFERENCE_TEB / "teb_local_planner.yaml") == (
        "eec4ca2a275b53eb214ece71c74fc1557856c0286e10890aa419fbd383b6905a"
    )
    manifest = (REFERENCE_TEB / "SHA256SUMS").read_text()
    assert "mprp3@10.242.33.199" in manifest
    assert "2026-08-09" in manifest
```

- [ ] **Step 2: Run the focused test and observe the missing-reference failure**

Run:

```powershell
py -3 -m pytest src/wheelchair_navigation/tests/test_navigation_static.py::test_nuc_teb_reference_snapshots_match_source_hashes -q -p no:cacheprovider
```

Expected: `FileNotFoundError` for `docs/reference/nuc_teb/move_base.yaml`.

- [ ] **Step 3: Read both NUC source files without changing the NUC**

Run and capture the complete stdout:

```powershell
ssh -o BatchMode=yes mprp3@10.242.33.199 "sed -n '1,$p' /home/mprp3/catkin_ws/src/base_model/config/move_base.yaml"
ssh -o BatchMode=yes mprp3@10.242.33.199 "sed -n '1,$p' /home/mprp3/catkin_ws/src/base_model/config/teb_local_planner.yaml"
ssh -o BatchMode=yes mprp3@10.242.33.199 "sha256sum /home/mprp3/catkin_ws/src/base_model/config/move_base.yaml /home/mprp3/catkin_ws/src/base_model/config/teb_local_planner.yaml"
```

Create both reference files from the captured output with `apply_patch`, preserving every YAML line and the final newline. Create `SHA256SUMS` with these exact records:

```text
source_host: mprp3@10.242.33.199
acquired_at: 2026-08-09 Asia/Seoul
dec3b50729fe9c139b6e7aaead24ce33ab39862a55538aefdc710acedbf0dc3c  /home/mprp3/catkin_ws/src/base_model/config/move_base.yaml
eec4ca2a275b53eb214ece71c74fc1557856c0286e10890aa419fbd383b6905a  /home/mprp3/catkin_ws/src/base_model/config/teb_local_planner.yaml
```

- [ ] **Step 4: Run the provenance test and verify it passes**

Run the Step 2 command again.

Expected: `1 passed`.

- [ ] **Step 5: Commit the authenticated source snapshot**

```powershell
git add docs/reference/nuc_teb src/wheelchair_navigation/tests/test_navigation_static.py
git commit -m "docs: preserve NUC TEB source configuration"
```

### Task 2: Create the active TEB configuration with one explicit geometry adaptation

**Files:**
- Create: `src/wheelchair_navigation/config/teb_local_planner.yaml`
- Modify: `src/wheelchair_navigation/tests/test_navigation_static.py`

**Interfaces:**
- Consumes: `docs/reference/nuc_teb/teb_local_planner.yaml` from Task 1 and the rectangle in `config/costmap_common.yaml`.
- Produces: `TebLocalPlannerROS` parameters for `move_base`, with source tuning equality and a polygon `footprint_model`.

- [ ] **Step 1: Add the failing source-equality and footprint test**

Add `import yaml` and this test:

```python
def test_active_teb_preserves_nuc_tuning_except_github_footprint():
    source = yaml.safe_load((REFERENCE_TEB / "teb_local_planner.yaml").read_text())
    active = yaml.safe_load(_text("config/teb_local_planner.yaml"))
    source_teb = dict(source["TebLocalPlannerROS"])
    active_teb = dict(active["TebLocalPlannerROS"])
    source_footprint = source_teb.pop("footprint_model")
    active_footprint = active_teb.pop("footprint_model")

    assert active_teb == source_teb
    assert source_footprint == {
        "type": "line",
        "line_start": [0.45, 0.0],
        "line_end": [-0.45, 0.0],
    }

    costmap = _text("config/costmap_common.yaml")
    footprint_match = re.search(r"^footprint:\s*(.+)$", costmap, re.MULTILINE)
    assert footprint_match
    github_footprint = ast.literal_eval(footprint_match.group(1))
    assert active_footprint == {
        "type": "polygon",
        "vertices": github_footprint,
    }
```

- [ ] **Step 2: Run the focused test and observe the missing-active-config failure**

```powershell
py -3 -m pytest src/wheelchair_navigation/tests/test_navigation_static.py::test_active_teb_preserves_nuc_tuning_except_github_footprint -q -p no:cacheprovider
```

Expected: `FileNotFoundError` for `config/teb_local_planner.yaml`.

- [ ] **Step 3: Add the minimal active TEB YAML**

Create `config/teb_local_planner.yaml` from the complete Task 1 reference. Preserve every key and value, replacing only this source block:

```yaml
  footprint_model:
      type: "line"
      line_start: [0.45, 0.0]
      line_end: [-0.45, 0.0]
```

with:

```yaml
  footprint_model:
      type: "polygon"
      vertices: [[0.485, 0.300], [0.485, -0.300], [-0.485, -0.300], [-0.485, 0.300]]
```

- [ ] **Step 4: Run the focused test and verify it passes**

Run the Step 2 command again.

Expected: `1 passed`.

- [ ] **Step 5: Commit the active TEB parameters**

```powershell
git add src/wheelchair_navigation/config/teb_local_planner.yaml src/wheelchair_navigation/tests/test_navigation_static.py
git commit -m "feat: add NUC-derived TEB parameters"
```

### Task 3: Select TEB in the active navigation graph

**Files:**
- Modify: `src/wheelchair_navigation/config/move_base.yaml`
- Modify: `src/wheelchair_navigation/launch/navigation.launch`
- Modify: `src/wheelchair_navigation/package.xml`
- Modify: `src/wheelchair_navigation/tests/test_navigation_static.py`

**Interfaces:**
- Consumes: active TEB YAML from Task 2.
- Produces: one `move_base` instance using Navfn globally, TEB locally, repository costmaps and frames, and the existing configurable command remap.

- [ ] **Step 1: Change the static contract tests to require TEB**

In `test_move_base_uses_ros1_navigation_stack_components`, replace the DWA plugin assertion and launch namespace assertion with:

```python
assert "base_local_planner: teb_local_planner/TebLocalPlannerROS" in text

assert namespaces["teb_local_planner.yaml"] == "move_base"
assert "dwa_local_planner.yaml" not in namespaces
```

Add this test:

```python
def test_teb_runtime_dependencies_and_launch_graph_are_explicit():
    package = (NAV / "package.xml").read_text()
    assert "<exec_depend>teb_local_planner</exec_depend>" in package
    assert "<exec_depend>costmap_converter</exec_depend>" in package

    launch_text = _text("launch/navigation.launch")
    assert "config/teb_local_planner.yaml" in launch_text
    assert "config/dwa_local_planner.yaml" not in launch_text
    for forbidden in ("waypoint_follower.py", "pure_pursuit", "s_curve", "wheel_cmd.py", "uart.py"):
        assert forbidden not in launch_text
```

Keep `test_navigation_launch_routes_move_base_to_raw_nav_topic_only` unchanged so it continues to prove that the default is `/cmd_vel_nav` and the remap target is controlled by `$(arg cmd_vel_nav_topic)`.

- [ ] **Step 2: Run the two contract tests and observe TEB-selection failures**

```powershell
py -3 -m pytest src/wheelchair_navigation/tests/test_navigation_static.py::test_move_base_uses_ros1_navigation_stack_components src/wheelchair_navigation/tests/test_navigation_static.py::test_teb_runtime_dependencies_and_launch_graph_are_explicit -q -p no:cacheprovider
```

Expected: failures showing the DWA plugin/config is still active and TEB dependencies are absent.

- [ ] **Step 3: Make only the active planner wiring changes**

In `config/move_base.yaml`, change exactly:

```yaml
base_local_planner: teb_local_planner/TebLocalPlannerROS
```

Do not copy the NUC controller frequencies or recovery settings into this active file.

In `launch/navigation.launch`, replace only the DWA rosparam load with:

```xml
  <rosparam command="load" file="$(find wheelchair_navigation)/config/teb_local_planner.yaml" ns="move_base"/>
```

Keep this command remap unchanged:

```xml
    <remap from="cmd_vel" to="$(arg cmd_vel_nav_topic)"/>
```

In `package.xml`, retain existing dependencies and add:

```xml
  <exec_depend>teb_local_planner</exec_depend>
  <exec_depend>costmap_converter</exec_depend>
```

- [ ] **Step 4: Run the TEB contract tests and the output-topic test**

```powershell
py -3 -m pytest src/wheelchair_navigation/tests/test_navigation_static.py::test_move_base_uses_ros1_navigation_stack_components src/wheelchair_navigation/tests/test_navigation_static.py::test_teb_runtime_dependencies_and_launch_graph_are_explicit src/wheelchair_navigation/tests/test_navigation_static.py::test_navigation_launch_routes_move_base_to_raw_nav_topic_only -q -p no:cacheprovider
```

Expected: `3 passed`.

- [ ] **Step 5: Commit the active planner switch**

```powershell
git add src/wheelchair_navigation/config/move_base.yaml src/wheelchair_navigation/launch/navigation.launch src/wheelchair_navigation/package.xml src/wheelchair_navigation/tests/test_navigation_static.py
git commit -m "feat: run move_base with NUC-derived TEB"
```

### Task 4: Document local validation and verify the branch

**Files:**
- Create: `docs/nuc_teb_navigation.md`

**Interfaces:**
- Consumes: the completed TEB launch graph.
- Produces: operator-facing commands that distinguish direct local command inspection from guarded integration.

- [ ] **Step 1: Write the operator document**

Create `docs/nuc_teb_navigation.md` with these exact operational sections and commands:

```markdown
# NUC-derived TEB navigation

This branch keeps the repository TF, URDF, costmaps, localization, Navfn global planner, and safety topology. Only the `move_base` local planner is TEB.

## ROS dependencies

```bash
sudo apt install ros-noetic-teb-local-planner ros-noetic-costmap-converter
```

## Local command inspection

This publishes planner output on `/cmd_vel` for local inspection. It does not add a motor driver.

```bash
roslaunch wheelchair_navigation navigation.launch use_sim_time:=true cmd_vel_nav_topic:=/cmd_vel
rostopic info /cmd_vel
rostopic echo /cmd_vel
```

## Guarded integrated path

Normal bringup keeps `/cmd_vel_nav -> safety_gate -> /cmd_vel_safe`.

```bash
roslaunch wheelchair_bringup sim_bringup.launch
rostopic info /cmd_vel_nav
rostopic info /cmd_vel_safe
```

## Provenance and adaptation

The NUC source snapshots and SHA-256 records are in `docs/reference/nuc_teb/`. The active TEB tuning preserves those values, while its footprint is a polygon matching `wheelchair_navigation/config/costmap_common.yaml`.
```

- [ ] **Step 2: Run focused navigation tests**

```powershell
py -3 -m pytest src/wheelchair_navigation/tests/test_navigation_static.py -q -p no:cacheprovider
```

Expected on an LF checkout: all navigation static tests pass. Expected on this Windows CRLF checkout: only `test_navigation_launch_binds_exactly_one_current_hanyang_map_server` retains the documented pre-existing map hash failure; every TEB-related test passes.

- [ ] **Step 3: Run repository formatting and change-scope checks**

```powershell
git diff --check
git status --short
git diff --stat a54f130..HEAD
```

Expected: no whitespace errors; only the design/plan/reference/TEB navigation files listed in this plan are changed.

- [ ] **Step 4: Commit the operator document**

```powershell
git add docs/nuc_teb_navigation.md
git commit -m "docs: explain local TEB validation"
```

- [ ] **Step 5: Perform completion verification**

Run the focused TEB tests by selecting their node IDs, run the full navigation static file, inspect `git status --short`, and record the exact pass/fail counts. Do not claim ROS runtime validation unless a ROS 1 Noetic environment is available and `roslaunch` was actually executed.
