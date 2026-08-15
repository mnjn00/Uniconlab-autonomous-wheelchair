"""JobRunner 검증 — 가짜 스크립트로, NUC 없이.

허용목록 밖 이름 거부 / 스크립트 부재 / 중복 실행 방지 / 취소 / 실패 exit code 전달 /
--allow-scripts 게이트 / 폰 입력이 셸에 닿지 않는지.
"""
import importlib.util
import json
import os
import shutil
import socket
import stat
import sys
import tempfile
import threading
import time

BRIDGE = r"c:\Users\npgy2\.anaconda\intern\Uniconlab-autonomous-wheelchair\scripts\ros1_bluetooth_bridge.py"
spec = importlib.util.spec_from_file_location("btbridge", BRIDGE)
bb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bb)

# Windows has no /bin/bash, and the execution path is exactly what needs
# testing, so swap in whatever bash this machine has. On the NUC it stays
# /bin/bash.
if not os.path.exists(bb.BASH):
    for cand in (r"C:\Program Files\Git\bin\bash.exe",
                 r"C:\Program Files\Git\usr\bin\bash.exe",
                 shutil.which("bash")):
        if cand and os.path.exists(cand):
            bb.BASH = cand
            break
print("bash =", bb.BASH)

try:                       # console is cp949; the labels below are Korean
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

failures = []


def check(name, cond, note=""):
    print("  %-58s %s%s" % (name, "PASS" if cond else "FAIL", "" if not note else "  (%s)" % note))
    if not cond:
        failures.append(name)


def make_scripts(tmp, kinds):
    """kinds: name -> shell body"""
    for job, (filename, _label) in bb.JobRunner.JOBS.items():
        if job not in kinds:
            continue
        path = os.path.join(tmp, filename)
        with open(path, "w", newline="\n") as fh:
            fh.write("#!/bin/bash\n" + kinds[job] + "\n")
        os.chmod(path, os.stat(path).st_mode | stat.S_IEXEC)


print("\n--- JobRunner 검증 ---")
tmp = tempfile.mkdtemp(prefix="btjobs_")
try:
    # ---------------------------------------------------------------- 게이트
    off = bb.JobRunner(tmp, enabled=False)
    ok, detail = off.start("stack")
    check("--allow-scripts 없으면 전면 거부", ok is False and "disabled" in detail)

    # ------------------------------------------------------------ 허용목록
    on = bb.JobRunner(tmp, enabled=True)
    ok, detail = on.start("rm -rf /")
    check("허용목록 밖 이름 거부", ok is False and "unknown job" in detail, detail[:50])
    ok, detail = on.start("../../../bin/sh")
    check("경로 탈출 시도 거부", ok is False and "unknown job" in detail)
    check("JOBS 에 trial 없음", "trial" not in bb.JobRunner.JOBS)
    check("허용목록은 stack/drive/halt 뿐",
          sorted(bb.JobRunner.JOBS) == ["drive", "halt", "stack"])

    # -------------------------------------------------------- 스크립트 부재
    ok, detail = on.start("stack")
    check("스크립트 부재 시 경로와 함께 거부",
          ok is False and "not found" in detail, detail[:60])
    check("available() 가 부재를 보고", on.available() == {"stack": False, "drive": False, "halt": False})

    # ------------------------------------------------------------ 정상 실행
    make_scripts(tmp, {"stack": "echo 'bringing up sensors'\nsleep 2\necho done",
                       "drive": "echo DRIVING\nexit 0",
                       "halt": "echo STOPPED\nexit 0"})
    check("available() 가 존재를 보고", on.available() == {"stack": True, "drive": True, "halt": True})

    ok, detail = on.start("stack")
    check("스택 기동 성공", ok is True, detail[:60])
    time.sleep(0.6)
    snap = on.snapshot()
    check("실행 중 상태 보고", snap["job_state"] == "running", snap["job_state"])
    check("작업 이름 노출", snap["job_name"] == "stack")
    check("경과 시간 보고", isinstance(snap["job_elapsed_s"], float))

    # ------------------------------------------------------------ 중복 방지
    ok, detail = on.start("drive")
    check("실행 중 다른 작업 거부", ok is False and "still running" in detail, detail[:50])

    # 완료 대기
    for _ in range(40):
        if on.snapshot()["job_state"] != "running":
            break
        time.sleep(0.25)
    snap = on.snapshot()
    check("성공 시 succeeded", snap["job_state"] == "succeeded", snap["job_state"])
    check("exit code 0 전달", snap["job_exit_code"] == 0)
    check("마지막 로그 줄 전달", (snap["job_tail"] or "").strip() == "done", repr(snap["job_tail"]))

    # ------------------------------------------------------------ 실패 전파
    make_scripts(tmp, {"drive": "echo 'localization is silent, not TRACKING' >&2\nexit 3"})
    on2 = bb.JobRunner(tmp, enabled=True)
    on2.start("drive")
    for _ in range(40):
        if on2.snapshot()["job_state"] != "running":
            break
        time.sleep(0.25)
    snap = on2.snapshot()
    check("실패 시 failed", snap["job_state"] == "failed", snap["job_state"])
    check("실패 exit code 전달", snap["job_exit_code"] == 3, str(snap["job_exit_code"]))
    check("스크립트 거부 사유가 tail 로 전달",
          "TRACKING" in (snap["job_tail"] or ""), repr(snap["job_tail"]))

    # ---------------------------------------------------------------- 취소
    make_scripts(tmp, {"stack": "sleep 60"})
    on3 = bb.JobRunner(tmp, enabled=True)
    on3.start("stack")
    time.sleep(0.6)
    check("취소 전 running", on3.snapshot()["job_state"] == "running")
    ok, detail = on3.cancel()
    check("취소 성공", ok is True, detail[:40])
    time.sleep(1.0)
    check("취소 후 더 이상 running 아님", on3.snapshot()["job_state"] != "running",
          on3.snapshot()["job_state"])
    ok, detail = on3.cancel()
    check("돌고 있지 않을 때 취소는 거부", ok is False and "no job" in detail)

    # ------------------------------------------------ 프로토콜 레벨 (Session)
    print("\n--- 프로토콜 레벨 ---")
    make_scripts(tmp, {"stack": "echo up\nexit 0", "drive": "echo go\nexit 0",
                       "halt": "echo stop\nexit 0"})

    class FakeRos:
        connected = False
        allow_commands = False

        def follower_available(self):
            return False

        def set_follower(self, running, ensure_auto=False):
            return False, "commands disabled"

    state = bb.BridgeState()
    jobs = bb.JobRunner(tmp, enabled=True)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]

    def serve():
        conn, _ = srv.accept()
        bb.Session(conn, state, FakeRos(), 1.0, 4.0, jobs=jobs).serve()
        conn.close()

    threading.Thread(target=serve, daemon=True).start()
    time.sleep(0.3)
    cli = socket.create_connection(("127.0.0.1", port))
    stream = cli.makefile("rb")

    def nxt(kind, timeout=6.0):
        end = time.time() + timeout
        while time.time() < end:
            line = stream.readline()
            if not line:
                return None
            obj = json.loads(line.decode())
            if obj.get("type") == kind:
                return obj
        return None

    def send(o):
        cli.sendall((json.dumps(o) + "\n").encode())

    # 경로 프레임이 먼저 와야 앱이 포즈보다 먼저 지도를 그린다
    route_frame = nxt("route", timeout=3.0)
    check("경로 프레임 없음(--route 미지정 세션)", route_frame is None or "points" in route_frame)

    frame = nxt("telemetry")
    check("네비게이션 필드 존재",
          frame is not None and all(k in frame for k in
              ("pose_x", "pose_yaw_deg", "loc_fitness", "wp_index", "follower_state")))
    check("텔레메트리에 job 필드 존재",
          frame is not None and "job_state" in frame and "jobs_available" in frame)
    check("scripts_enabled 노출", frame.get("scripts_enabled") is True)

    send({"command": "stack_start"})
    ack = nxt("ack")
    check("stack_start 는 confirm 없으면 거부", ack is not None and ack["ok"] is False,
          (ack or {}).get("detail", "")[:40])

    send({"command": "stack_start", "confirm": True})
    ack = nxt("ack")
    check("stack_start confirm 시 실행", ack is not None and ack["ok"] is True,
          (ack or {}).get("detail", "")[:50])

    send({"command": "job_cancel"})
    nxt("ack")

    send({"command": "drive_stop"})
    ack = nxt("ack")
    check("drive_stop 은 스크립트로 실행", ack is not None and ack["ok"] is True,
          (ack or {}).get("detail", "")[:50])

    send({"command": "job_start", "name": "anything"})
    ack = nxt("ack")
    check("임의 명령 이름은 unknown 처리", ack is not None and ack["ok"] is False)

    check("링크 살아있음", nxt("telemetry") is not None)
    cli.close()
    srv.close()

    # ---------------------------- E-STOP 이 팔로워를 멈추는가 (안전 회귀)
    print("\n--- E-STOP 회귀 ---")

    class RecordingRos(bb.RosLink):
        def __init__(self, st):
            bb.RosLink.__init__(self, st, True, "test")
            self.published = []
            self.follower_calls = []

        def _publish_mode(self, value):
            self.published.append(value)
            return True, "mode_cmd=%d published" % value

        def set_follower(self, running, ensure_auto=False):
            self.follower_calls.append((running, ensure_auto))
            return True, "PAUSED" if not running else "ENABLED"

    rec = RecordingRos(bb.BridgeState())
    ok, detail = rec.engage_estop()
    check("E-STOP 이 mode_cmd=77 발행", ok and 77 in rec.published, str(rec.published))
    check("mode_cmd 가 서비스 호출보다 먼저", rec.published[0] == 77)
    check("E-STOP 이 팔로워도 정지 (해제 시 급발진 방지)",
          (False, False) in rec.follower_calls, str(rec.follower_calls))

    rec2 = RecordingRos(bb.BridgeState())
    rec2.release_estop()
    check("해제는 mode_cmd=65 만 하고 주행 재개 안 함",
          rec2.published == [65] and rec2.follower_calls == [],
          "published=%s follower=%s" % (rec2.published, rec2.follower_calls))

finally:
    shutil.rmtree(tmp, ignore_errors=True)

print("\n--- %s ---\n" % ("전부 통과" if not failures
                          else "%d개 실패: %s" % (len(failures), ", ".join(failures))))
sys.exit(1 if failures else 0)
