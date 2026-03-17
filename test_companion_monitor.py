"""
Unit tests for ResourceMonitor state transitions.
ArduPilot GSoC 2026 — Companion Computer Health Monitor

Tests the core state machine logic of ResourceMonitor in isolation —
no MAVLink connection, no hardware, no psutil reads required.

Run with:
    pip install pytest --break-system-packages
    pytest test_companion_monitor.py -v
"""

import pytest
from collections import deque
from enum import Enum, auto


# ─────────────────────────────────────────────────────────────────────────────
# Minimal stubs — isolate ResourceMonitor from MAVLink and global state
# so tests run without a flight controller connected.
# ─────────────────────────────────────────────────────────────────────────────

class SystemState(Enum):
    NOMINAL  = auto()
    WARNING  = auto()
    HIGH     = auto()
    CRITICAL = auto()
    FAILSAFE = auto()
    RECOVERING = auto()

# Capture every send_statustext / send_named_float / trigger_rtl call
_statustext_calls = []
_named_float_calls = []
_rtl_calls = []

def send_statustext(text, severity=4, force=False):
    _statustext_calls.append((text, severity))

def send_named_float(name, value):
    _named_float_calls.append((name, value))

def trigger_rtl(reason=""):
    _rtl_calls.append(reason)

# Paste ResourceMonitor here verbatim so tests are self-contained.
# window_size and critical_timeout are overridable per-test via the fixture.
WINDOW_SIZE      = 3
CRITICAL_TIMEOUT = 3

class ResourceMonitor:
    def __init__(self, name, warning, high, critical, float_key=""):
        self.name      = name
        self.warning   = warning
        self.high      = high
        self.critical  = critical
        self.float_key = float_key if float_key else name[:10]

        self.window           = deque(maxlen=WINDOW_SIZE)
        self.state            = SystemState.NOMINAL
        self.critical_counter = 0

    @property
    def avg(self):
        return sum(self.window) / len(self.window) if self.window else 0.0

    def update(self, value):
        self.window.append(value)
        avg = self.avg
        if avg >= self.critical:
            self._handle_critical(avg)
        elif avg >= self.high:
            self._handle_high(avg)
        elif avg >= self.warning:
            self._handle_warning(avg)
        else:
            self._handle_nominal()
        return self.state

    def _handle_critical(self, avg):
        if self.state != SystemState.CRITICAL:
            self.state = SystemState.CRITICAL
            self.critical_counter = 0
        self.critical_counter += 1
        send_statustext("{} CRITICAL {:.1f}".format(self.name, avg), severity=2)
        send_named_float(self.float_key, avg)
        if self.critical_counter >= CRITICAL_TIMEOUT:
            trigger_rtl(reason=self.name)

    def _handle_high(self, avg):
        if self.state != SystemState.HIGH:
            self.state = SystemState.HIGH
            self.critical_counter = 0
            send_statustext("{} HIGH {:.1f}".format(self.name, avg), severity=3, force=True)
        else:
            send_statustext("{} HIGH {:.1f}".format(self.name, avg), severity=3)
        send_named_float(self.float_key, avg)

    def _handle_warning(self, avg):
        if self.state != SystemState.WARNING:
            self.state = SystemState.WARNING
            self.critical_counter = 0
            send_statustext("{} WARNING {:.1f}".format(self.name, avg), severity=4, force=True)
        send_named_float(self.float_key, avg)

    def _handle_nominal(self):
        if self.state != SystemState.NOMINAL:
            self.state = SystemState.NOMINAL
        self.critical_counter = 0


# ─────────────────────────────────────────────────────────────────────────────
# Fixtures
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture(autouse=True)
def clear_call_logs():
    """Reset all captured call logs before each test."""
    _statustext_calls.clear()
    _named_float_calls.clear()
    _rtl_calls.clear()


@pytest.fixture
def mon():
    """Standard CPU monitor: warning=70, high=80, critical=90."""
    return ResourceMonitor("CPU", warning=70, high=80, critical=90, float_key="CPU_PCT")


# ─────────────────────────────────────────────────────────────────────────────
# State transition tests
# ─────────────────────────────────────────────────────────────────────────────

def test_starts_nominal(mon):
    assert mon.state == SystemState.NOMINAL


def test_below_warning_stays_nominal(mon):
    for _ in range(5):
        mon.update(50.0)
    assert mon.state == SystemState.NOMINAL


def test_enters_warning(mon):
    # Fill window so rolling avg crosses warning threshold
    for _ in range(WINDOW_SIZE):
        mon.update(75.0)
    assert mon.state == SystemState.WARNING


def test_enters_high(mon):
    for _ in range(WINDOW_SIZE):
        mon.update(85.0)
    assert mon.state == SystemState.HIGH


def test_enters_critical(mon):
    for _ in range(WINDOW_SIZE):
        mon.update(95.0)
    assert mon.state == SystemState.CRITICAL


def test_returns_to_nominal_from_warning(mon):
    for _ in range(WINDOW_SIZE):
        mon.update(75.0)
    assert mon.state == SystemState.WARNING
    for _ in range(WINDOW_SIZE):
        mon.update(10.0)
    assert mon.state == SystemState.NOMINAL


def test_critical_counter_increments(mon):
    # Each update() that resolves to CRITICAL increments the counter.
    # WINDOW_SIZE=3 updates all at 95.0: counter reaches 3 after window fills.
    for i in range(WINDOW_SIZE):
        mon.update(95.0)
        assert mon.critical_counter == i + 1
    assert mon.state == SystemState.CRITICAL
    # One more tick -> counter becomes 4 (RTL already fired at 3)
    mon.update(95.0)
    assert mon.critical_counter == 4


def test_rtl_triggers_after_critical_timeout(mon):
    # CRITICAL_TIMEOUT=3, WINDOW_SIZE=3.
    # After CRITICAL_TIMEOUT updates at 95.0 the counter hits threshold -> RTL.
    for _ in range(CRITICAL_TIMEOUT):
        mon.update(95.0)
    assert len(_rtl_calls) == 1
    assert _rtl_calls[0] == "CPU"


def test_rtl_not_triggered_before_timeout(mon):
    # Only 2 ticks in CRITICAL (one fewer than CRITICAL_TIMEOUT=3) — no RTL.
    # update 1: window=[95]    avg=95 -> counter=1
    # update 2: window=[95,95] avg=95 -> counter=2  (still < 3)
    mon.update(95.0)
    mon.update(95.0)
    assert mon.critical_counter == 2
    assert len(_rtl_calls) == 0


def test_statustext_sent_on_warning_entry(mon):
    for _ in range(WINDOW_SIZE):
        mon.update(75.0)
    assert any("WARNING" in t for t, _ in _statustext_calls)


def test_statustext_sent_on_critical_entry(mon):
    for _ in range(WINDOW_SIZE):
        mon.update(95.0)
    assert any("CRITICAL" in t for t, _ in _statustext_calls)


def test_named_float_sent_when_warning(mon):
    for _ in range(WINDOW_SIZE):
        mon.update(75.0)
    assert any(name == "CPU_PCT" for name, _ in _named_float_calls)


def test_named_float_not_sent_when_nominal(mon):
    for _ in range(5):
        mon.update(10.0)
    assert len(_named_float_calls) == 0


def test_rolling_average_smooths_spike(mon):
    """A single spike should not immediately push avg past threshold."""
    mon.update(10.0)
    mon.update(10.0)
    mon.update(95.0)   # single spike in window of 3 → avg = (10+10+95)/3 = 38.3
    assert mon.state == SystemState.NOMINAL


def test_critical_counter_resets_on_recovery(mon):
    for _ in range(WINDOW_SIZE):
        mon.update(95.0)
    assert mon.state == SystemState.CRITICAL
    for _ in range(WINDOW_SIZE):
        mon.update(10.0)
    assert mon.critical_counter == 0
    assert mon.state == SystemState.NOMINAL


def test_warning_to_high_transition(mon):
    for _ in range(WINDOW_SIZE):
        mon.update(75.0)
    assert mon.state == SystemState.WARNING
    for _ in range(WINDOW_SIZE):
        mon.update(85.0)
    assert mon.state == SystemState.HIGH


def test_avg_property(mon):
    mon.update(60.0)
    mon.update(80.0)
    mon.update(100.0)
    assert abs(mon.avg - 80.0) < 0.01


def test_empty_window_avg_is_zero(mon):
    assert mon.avg == 0.0