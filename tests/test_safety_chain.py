from pathlib import Path


ROOT = Path(__file__).parents[1]


def gate_text():
    return (ROOT / "src" / "static_livox_localization" / "scripts"
            / "safety_gate.py").read_text(encoding="utf-8")


def tip_guard_text():
    return (ROOT / "src" / "static_livox_localization" / "scripts"
            / "tip_guard.py").read_text(encoding="utf-8")


def uart_text():
    return (ROOT / "tools" / "base_model_uart_watchdog.py").read_text(
        encoding="utf-8")


def test_gate_is_route_agnostic_and_feeds_tip_guard_not_the_base_directly():
    text = gate_text()
    assert '"/cmd_vel_raw"' in text
    assert '"/cmd_vel_gated"' in text
    assert "route" not in text.lower().replace("routes or planning", "")


def test_tip_guard_is_the_final_stage_publishing_cmd_vel():
    gate = gate_text()
    guard = tip_guard_text()
    assert '"/cmd_vel_gated"' in guard
    assert '"/cmd_vel"' in guard
    assert '"/cmd_vel"' not in gate


def test_gate_replaces_stale_input_with_stop_and_always_publishes():
    text = gate_text()
    assert "INPUT_STALE_S" in text
    assert "CLOUD_STALE_S" in text
    assert "self.pub.publish(out)" in text
    assert "rospy.on_shutdown" in text


def test_gate_blocks_close_obstacles_and_missing_sensing():
    text = gate_text()
    for reason in ('"OBSTACLE"', '"NO_CLOUD"'):
        assert reason in text
    assert "map-band containment" in text


def test_gate_forbids_reverse_and_clamps_speeds():
    text = gate_text()
    assert "max(0.0, min(HARD_V_LIMIT" in text
    assert "HARD_W_LIMIT" in text


def test_uart_watchdog_stops_motors_when_command_stream_dies():
    text = uart_text()
    assert "WATCHDOG_TIMEOUT_S = 0.6" in text
    assert "WatchdogTick" in text
    assert "self.TX([65] + self.stop_cmd)" in text


def test_uart_watchdog_only_acts_in_auto_mode():
    text = uart_text()
    tick = text.index("def WatchdogTick")
    guard = text.index("if self.mode != 65", tick)
    send = text.index("self.TX", guard)
    assert tick < guard < send


def test_uart_mode_initialized_and_tx_locked():
    text = uart_text()
    assert "self.mode = None" in text
    assert "self.tx_lock" in text
    assert "with self.tx_lock:" in text
