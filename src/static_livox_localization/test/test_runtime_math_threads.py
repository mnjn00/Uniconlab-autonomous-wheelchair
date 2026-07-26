import os
from pathlib import Path
import subprocess
import sys

import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"
THREAD_VARS = (
    "OPENBLAS_NUM_THREADS",
    "OMP_NUM_THREADS",
    "MKL_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
)


def import_node_and_read_thread_env(module_name, configured=None):
    env = os.environ.copy()
    for name in THREAD_VARS:
        env.pop(name, None)
    env.update(configured or {})
    code = (
        "import os,sys;"
        f"sys.path.insert(0,{str(SCRIPTS)!r});"
        f"import {module_name};"
        f"print('|'.join(os.environ.get(k,'') for k in {THREAD_VARS!r}))"
    )
    result = subprocess.run(
        [sys.executable, "-c", code],
        check=True,
        capture_output=True,
        env=env,
        text=True,
    )
    return result.stdout.strip().splitlines()[-1]


@pytest.mark.parametrize("module_name", ["safety_gate", "waypoint_follower"])
def test_motion_nodes_limit_numeric_worker_threads_before_numpy_import(module_name):
    assert import_node_and_read_thread_env(module_name) == "1|1|1|1"


def test_operator_thread_override_is_preserved():
    configured = {
        "OPENBLAS_NUM_THREADS": "2",
        "OMP_NUM_THREADS": "3",
        "MKL_NUM_THREADS": "4",
        "NUMEXPR_NUM_THREADS": "5",
    }

    assert (
        import_node_and_read_thread_env("waypoint_follower", configured)
        == "2|3|4|5"
    )
