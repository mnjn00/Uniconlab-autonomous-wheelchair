import importlib.util
import sys
import types
from pathlib import Path


class FakePort:
    def __init__(self):
        self.rx = []
        self.tx = []

    def read(self):
        return bytes([self.rx.pop(0)]) if self.rx else b""

    def write(self, value):
        self.tx.append(value)

    def close(self):
        pass


class FakePublisher:
    def __init__(self, *args, **kwargs):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeTime:
    @staticmethod
    def now():
        return FakeTime()

    def __sub__(self, other):
        return types.SimpleNamespace(to_sec=lambda: 0.0)


def load_watchdog():
    port = FakePort()
    serial = types.ModuleType("serial")
    serial.EIGHTBITS = 8
    serial.PARITY_NONE = "N"
    serial.STOPBITS_ONE = 1
    serial.Serial = lambda **kwargs: port

    rospy = types.ModuleType("rospy")
    rospy.Time = FakeTime
    rospy.Duration = lambda value: value
    rospy.Timer = lambda *args, **kwargs: None
    rospy.Publisher = FakePublisher
    rospy.Subscriber = lambda *args, **kwargs: None
    rospy.init_node = lambda *args, **kwargs: None
    rospy.logwarn_throttle = lambda *args, **kwargs: None
    rospy.logwarn = lambda *args, **kwargs: None
    rospy.logerr = lambda *args, **kwargs: None

    messages = types.ModuleType("std_msgs.msg")
    messages.Int16 = type("Int16", (), {})
    messages.Int16MultiArray = type("Int16MultiArray", (), {"data": None})
    package = types.ModuleType("std_msgs")
    package.msg = messages

    saved = {name: sys.modules.get(name) for name in
             ("serial", "rospy", "std_msgs", "std_msgs.msg")}
    sys.modules.update(serial=serial, rospy=rospy, std_msgs=package)
    sys.modules["std_msgs.msg"] = messages
    try:
        path = Path(__file__).parents[1] / "tools" / "base_model_uart_watchdog.py"
        spec = importlib.util.spec_from_file_location("uart_watchdog_test", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    finally:
        for name, prior in saved.items():
            if prior is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = prior
    return module, port


def feed(uart, port, frame):
    port.rx.extend(frame)
    for _ in frame:
        uart.RX()


def checksum(values):
    return (~sum(values) + 1) & 0xFF


def test_checksum_valid_truncated_frame_cannot_latch_auto():
    module, port = load_watchdog()
    uart = module.UARTCommunication()
    feed(uart, port, [72, module.AUTO_MODE, checksum([module.AUTO_MODE]), 13, 10])
    assert uart.mode is None
    assert uart.uart_pub.messages == []


def test_exact_controller_status_frame_latches_a_defined_mode():
    module, port = load_watchdog()
    uart = module.UARTCommunication()
    body = [module.MANUAL_MODE, 83, 33, 67, 33, 1, 88]
    frame = [72] + body + [checksum(body), 13, 10]
    assert len(frame) == module.STATUS_FRAME_LEN
    feed(uart, port, frame)
    assert uart.mode == module.MANUAL_MODE
    assert uart.uart_pub.messages[-1].data == frame


def test_unknown_mode_is_published_but_never_latched():
    module, port = load_watchdog()
    uart = module.UARTCommunication()
    body = [99, 83, 33, 67, 33, 1, 88]
    frame = [72] + body + [checksum(body), 13, 10]
    feed(uart, port, frame)
    assert uart.mode is None
    assert uart.uart_pub.messages[-1].data == frame
