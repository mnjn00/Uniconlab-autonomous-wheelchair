"""Pytest collection policy for the historical NUC snapshot.

``test_job_runner.py`` is a standalone, side-effecting smoke program: it opens
sockets, spawns shell jobs and calls ``sys.exit`` at module scope. It is kept
as captured NUC evidence and can still be run directly, but it is not a pytest
module and must not be imported during the repository suite.
"""

collect_ignore = ["test_job_runner.py"]
