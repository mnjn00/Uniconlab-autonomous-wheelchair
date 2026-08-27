"""Static contracts for the optional RTX 2060 CUDA-PointPillars build.

CI has no NVIDIA GPU, so the CUDA target stays opt-in. These tests prevent the
field profile from regressing into a launcher that merely names a detector
without building, running, measuring, and gating on actual GPU inference.
"""

from pathlib import Path


PACKAGE = Path(__file__).parents[1]
ROOT = PACKAGE.parents[1]
PIN = "ce7e2bd694c90207435c8751d61cdb38d48a9f4c"


def text(path):
    return path.read_text(encoding="utf-8")


def test_cpp_node_calls_the_pinned_cuda_tensorrt_core():
    source = text(PACKAGE / "src" / "rtx_pointpillars_node.cpp")

    assert PIN in source
    assert "cudaSetDevice" in source
    assert "cudaStreamCreateWithFlags" in source
    assert "pointpillar::lidar::create_core" in source
    assert "core_->forward" in source
    assert "vision_msgs::Detection3DArray" in source
    assert '"/pointpillars/detections"' in source
    assert '"/pointpillars/status"' in source
    assert '\\"gpu_active\\\":true' in source
    assert "RTX 2060" in source


def test_cuda_build_is_opt_in_for_cpu_only_ci_but_real_on_the_nuc():
    cmake = text(PACKAGE / "CMakeLists.txt")
    package = text(PACKAGE / "package.xml")

    assert 'option(ENABLE_RTX_POINTPILLARS' in cmake
    assert '"Build the NVIDIA CUDA-PointPillars ROS1 inference node" OFF' in cmake
    assert "find_package(CUDA REQUIRED)" in cmake
    assert "libpointpillar_core" in cmake or "pointpillar_core" in cmake
    assert "rtx_pointpillars_node" in cmake
    assert "CUDA_POINTPILLARS_ROOT" in cmake
    assert "<depend>vision_msgs</depend>" in package


def test_setup_builds_an_fp16_engine_on_the_installed_gpu():
    setup = text(ROOT / "tools" / "setup_rtx2060_pointpillars.sh")

    assert "NVIDIA-AI-IOT/CUDA-PointPillars.git" in setup
    assert PIN in setup
    assert "git -C \"$POINTPILLARS_ROOT\" lfs pull" in setup
    assert "libpointpillar_core.so" in setup
    assert "--fp16" in setup
    assert "--plugins=\"$CORE_LIB\"" in setup
    assert "--inputIOFormats=fp16:chw,int32:chw,int32:chw" in setup
    assert "pointpillar.plan.meta" in setup
    assert "official sample inference" in setup
    assert "-DENABLE_RTX_POINTPILLARS=ON" in setup
    assert "pointpillars.env" in setup


def test_default_hybrid_start_really_launches_and_waits_for_gpu_inference():
    start = text(ROOT / "tools" / "start_hybrid_avoidance.sh")

    assert 'START_POINTPILLARS="${START_POINTPILLARS:-true}"' in start
    assert "rtx_pointpillars_node" in start
    assert "check_rtx2060_pointpillars.sh\" 30" in start
    assert "POINTPILLARS_DETECTIONS_TOPIC" in start
    assert "LEARNED_VISION_TOPIC=\"$POINTPILLARS_DETECTIONS_TOPIC\"" in start
    assert "_require_gpu_detector:=\"$START_POINTPILLARS\"" in start
    assert "/pointpillars/detections" in start
    assert "/pointpillars/status" in start


def test_preflight_and_go_refuse_fake_or_stale_gpu_labels():
    preflight = text(PACKAGE / "scripts" / "hybrid_preflight.py")
    go = text(ROOT / "tools" / "go_hybrid.sh")
    check = text(ROOT / "tools" / "check_rtx2060_pointpillars.sh")

    for required in (
        '"/pointpillars/status"', 'gpu.get("gpu_active") is not True',
        '"RTX 2060" not in device_name', 'gpu.get("upstream_commit")',
        'gpu.get("inference_ms")', 'gpu.get("used_points", 0)',
    ):
        assert required in preflight
    assert "rosnode ping -c1 /rtx_pointpillars" in go
    assert "check_rtx2060_pointpillars.sh\" 5" in go
    assert "_require_gpu_detector:=\"$START_POINTPILLARS\"" in go
    assert "nvidia-smi" in check
    assert 'status != "OK"' in check
    assert 'data.get("gpu_active") is not True' in check


def test_hybrid_entry_point_exposes_gpu_setup_and_observability():
    entry = text(ROOT / "tools" / "hybrid.sh")
    config = text(PACKAGE / "config" / "pointpillars_rtx2060.yaml")

    assert "setup-gpu" in entry
    assert "gpu-status" in entry
    assert "setup_rtx2060_pointpillars.sh" in entry
    assert "check_rtx2060_pointpillars.sh" in entry
    assert "require_rtx2060: true" in config
    assert "max_inference_ms: 90.0" in config
    assert "person_score_threshold: 0.30" in config
