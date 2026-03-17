"""
Companion Computer Health Monitor & Failsafe
ArduPilot GSoC 2026 Sample – Real-Time Companion-Computer Health Monitoring & Failsafe

Monitors CPU, RAM, Disk, CPU temperature, and GPU (Raspberry Pi via vcgencmd).
Watchdog for critical companion processes — attempts systemctl restart before RTL.
Broadcasts metrics as NAMED_VALUE_FLOAT whenever a threshold is crossed.
Triggers RTL failsafe after sustained CRITICAL usage (configurable seconds).
Recovers automatically after sustained healthy readings (configurable seconds).
Sends MAVLink STATUSTEXT with configurable rate limiting.
Sends periodic HEARTBEAT so ArduPilot can detect companion freeze/crash.
"""

import time
import logging
import psutil
import yaml
import os
import subprocess
import threading
from collections import deque
from enum import Enum, auto
from typing import Optional, Dict, List
from pymavlink import mavutil


# ─────────────────────────────────────────────
# Logging
# ─────────────────────────────────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))
log_path   = os.path.join(script_dir, "companion_health.log")

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(log_path),
        logging.StreamHandler()
    ]
)
log = logging.getLogger(__name__)


# ─────────────────────────────────────────────
# Load config
# ─────────────────────────────────────────────
config_path = os.path.join(script_dir, "configuratioGPU.yaml")

with open(config_path) as f:
    config = yaml.safe_load(f)

sample_interval          = config["sample_interval"]
window_size              = config["rolling_window"]

critical_failsafe_secs   = config["timeouts"]["critical_failsafe"]
recovery_secs            = config["timeouts"]["recovery"]
statustext_interval_crit = config["timeouts"]["statustext_interval_critical"]
statustext_interval_norm = config["timeouts"]["statustext_interval_normal"]
heartbeat_interval       = config["timeouts"]["heartbeat_interval"]

conn_string  = config["mavlink"]["connection"]
sys_id       = config["mavlink"]["system_id"]
comp_id      = config["mavlink"]["component_id"]

cpu_cfg      = config["thresholds"]["cpu"]
ram_cfg      = config["thresholds"]["ram"]
disk_cfg     = config["thresholds"]["disk"]
temp_cfg     = config["thresholds"]["temperature"]
gpu_temp_cfg = config["thresholds"]["gpu_temperature"]

# Watchdog config
watchdog_cfg  = config.get("watchdog", {})
watched_procs = watchdog_cfg.get("processes", [])
max_restarts  = watchdog_cfg.get("max_restart_attempts", 2)

# Convert second-based timeouts to sample counts
critical_timeout = max(1, int(critical_failsafe_secs / sample_interval))
recovery_timeout = max(1, int(recovery_secs          / sample_interval))


# ─────────────────────────────────────────────
# State Machine
# ─────────────────────────────────────────────
class SystemState(Enum):
    NOMINAL    = auto()
    WARNING    = auto()
    HIGH       = auto()
    CRITICAL   = auto()
    FAILSAFE   = auto()
    RECOVERING = auto()


# ─────────────────────────────────────────────
# MAVLink connection
# ─────────────────────────────────────────────
master = mavutil.mavlink_connection(
    conn_string,
    source_system=sys_id,
    source_component=comp_id
)

log.info("Waiting for heartbeat...")
master.wait_heartbeat()
log.info("Connected to vehicle (sysid=%d compid=%d)",
         master.target_system, master.target_component)


# ─────────────────────────────────────────────
# STATUSTEXT rate limiter
# ─────────────────────────────────────────────
_last_statustext_time: Dict[str, float] = {}

def send_statustext(text, severity=4, force=False):
    """
    Send a MAVLink STATUSTEXT with rate limiting.

    severity <= 3 (CRITICAL/ERROR) -> throttled to statustext_interval_critical.
    severity  > 3                  -> throttled to statustext_interval_normal.
    force=True bypasses throttle (used for state-change announcements).

    MAVLink severity: 0=EMERGENCY 1=ALERT 2=CRITICAL 3=ERROR
                      4=WARNING   5=NOTICE 6=INFO     7=DEBUG
    """
    now      = time.monotonic()
    interval = statustext_interval_crit if severity <= 3 else statustext_interval_norm
    last     = _last_statustext_time.get(text, 0.0)

    if not force and (now - last) < interval:
        return

    _last_statustext_time[text] = now
    master.mav.statustext_send(severity, text[:50].encode())
    log.info("STATUSTEXT [sev=%d] %s", severity, text)


# ─────────────────────────────────────────────
# NAMED_VALUE_FLOAT broadcast
# ─────────────────────────────────────────────
def send_named_float(name, value):
    """
    Broadcast a metric as NAMED_VALUE_FLOAT so a GCS (Mission Planner,
    QGroundControl) can graph it in real time.
    name must be <= 10 chars — it is padded/truncated automatically.
    Called only when a monitor crosses WARNING or above.
    """
    time_boot_ms = int((time.monotonic() * 1000) % (2**32))
    name_bytes   = name[:10].encode().ljust(10)
    master.mav.named_value_float_send(time_boot_ms, name_bytes, float(value))
    log.debug("NAMED_VALUE_FLOAT %s=%.2f", name, value)


# ─────────────────────────────────────────────
# HEARTBEAT thread
# ─────────────────────────────────────────────
_heartbeat_stop = threading.Event()

def _heartbeat_loop():
    """
    Sends a MAVLink HEARTBEAT from the companion at a fixed interval.
    ArduPilot uses this to detect a companion freeze or crash independently
    of the health metrics — required by the GSoC spec watchdog criterion.
    """
    while not _heartbeat_stop.wait(timeout=heartbeat_interval):
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )

heartbeat_thread = threading.Thread(target=_heartbeat_loop, daemon=True)
heartbeat_thread.start()
log.info("Heartbeat thread started (interval=%ds)", heartbeat_interval)


# ─────────────────────────────────────────────
# Failsafe / Recovery helpers
# ─────────────────────────────────────────────
previous_mode = None
system_state  = SystemState.NOMINAL


def trigger_rtl(reason=""):
    global previous_mode, system_state

    if system_state == SystemState.FAILSAFE:
        return

    previous_mode = master.flightmode
    system_state  = SystemState.FAILSAFE

    msg = "COMPANION FAILSAFE RTL"
    if reason:
        msg = ("COMPANION FAILSAFE: " + reason)[:50]

    log.warning("FAILSAFE TRIGGERED -> RTL | reason=%s | previous_mode=%s",
                reason or "metric threshold", previous_mode)
    send_statustext(msg, severity=2, force=True)

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0,
        0, 0, 0, 0, 0, 0, 0
    )

    # ── Command ACK handling ────────────────────────────────────────────────
    # Wait up to 3 seconds for the vehicle to acknowledge the RTL command.
    # MAV_RESULT values: 0=ACCEPTED 1=DENIED 2=UNSUPPORTED 3=FAILED 4=IN_PROGRESS
    ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    if ack is None:
        log.error("RTL ACK: no response from vehicle within 3s — command may not have been received")
        send_statustext("RTL NO ACK", severity=2, force=True)
    elif ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        log.info("RTL ACK: ACCEPTED by vehicle (result=%d)", ack.result)
    else:
        log.error("RTL ACK: vehicle rejected command (result=%d) — check flight mode permissions", ack.result)
        send_statustext("RTL REJECTED r={}".format(ack.result), severity=2, force=True)


def recover_mode():
    global system_state

    if system_state != SystemState.RECOVERING:
        return

    restore = previous_mode or "GUIDED"
    log.info("System recovered -> restoring mode: %s", restore)
    send_statustext("COMPANION RECOVERED", severity=6, force=True)

    # Send mode change via MAV_CMD_DO_SET_MODE so we can wait for ACK.
    # master.set_mode() is a helper but swallows the response — we send
    # the command manually here to get the same ACK handling as RTL.
    mode_id = master.mode_mapping().get(restore)

    if mode_id is None:
        log.error("RECOVER ACK: mode '%s' not found in vehicle mode map — defaulting to GUIDED", restore)
        restore  = "GUIDED"
        mode_id  = master.mode_mapping().get("GUIDED", 4)

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
        0, 0, 0, 0, 0
    )

    # ── Command ACK handling for mode restore ──────────────────────────────
    ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    if ack is None:
        log.error("RECOVER ACK: no response within 3s — mode may not have been restored")
        send_statustext("MODE RESTORE NO ACK", severity=3, force=True)
    elif ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        log.info("RECOVER ACK: mode '%s' ACCEPTED by vehicle", restore)
        send_statustext("MODE RESTORED: " + restore[:10], severity=6, force=True)
    else:
        log.error("RECOVER ACK: mode restore rejected (result=%d) — staying in RTL", ack.result)
        send_statustext("MODE RESTORE FAIL r={}".format(ack.result), severity=3, force=True)

    system_state = SystemState.NOMINAL


# ─────────────────────────────────────────────
# Resource Monitor
# ─────────────────────────────────────────────
class ResourceMonitor:
    """
    Tracks a single resource metric using a rolling average window.
    Transitions through NOMINAL -> WARNING -> HIGH -> CRITICAL states
    and fires the global failsafe when CRITICAL persists long enough.
    Broadcasts NAMED_VALUE_FLOAT whenever state enters WARNING or above.
    """

    def __init__(self, name, warning, high, critical, float_key=""):
        self.name      = name
        self.warning   = warning
        self.high      = high
        self.critical  = critical
        self.float_key = float_key if float_key else name[:10]

        self.window           = deque(maxlen=window_size)
        self.state            = SystemState.NOMINAL
        self.critical_counter = 0

    @property
    def avg(self):
        return sum(self.window) / len(self.window) if self.window else 0.0

    def update(self, value):
        self.window.append(value)
        avg = self.avg

        log.debug("%s raw=%.1f avg=%.2f", self.name, value, avg)

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
            log.warning("%s entered CRITICAL (avg=%.1f)", self.name, avg)

        self.critical_counter += 1
        log.info("%s critical counter: %d/%d",
                 self.name, self.critical_counter, critical_timeout)

        send_statustext("{} CRITICAL {:.1f}".format(self.name, avg), severity=2)
        send_named_float(self.float_key, avg)

        if self.critical_counter >= critical_timeout:
            trigger_rtl(reason=self.name)

    def _handle_high(self, avg):
        if self.state != SystemState.HIGH:
            self.state = SystemState.HIGH
            self.critical_counter = 0
            send_statustext("{} HIGH {:.1f}".format(self.name, avg), severity=3, force=True)
            log.warning("%s entered HIGH (avg=%.1f)", self.name, avg)
        else:
            send_statustext("{} HIGH {:.1f}".format(self.name, avg), severity=3)

        send_named_float(self.float_key, avg)

    def _handle_warning(self, avg):
        if self.state != SystemState.WARNING:
            self.state = SystemState.WARNING
            self.critical_counter = 0
            send_statustext("{} WARNING {:.1f}".format(self.name, avg), severity=4, force=True)
            log.info("%s entered WARNING (avg=%.1f)", self.name, avg)

        send_named_float(self.float_key, avg)

    def _handle_nominal(self):
        if self.state != SystemState.NOMINAL:
            log.info("%s returned to NOMINAL", self.name)
            self.state = SystemState.NOMINAL
        self.critical_counter = 0


# ─────────────────────────────────────────────
# Resource readers
# ─────────────────────────────────────────────
def get_cpu():
    return psutil.cpu_percent(interval=None)


def get_ram():
    return psutil.virtual_memory().percent


def get_disk():
    return psutil.disk_usage("/").percent


def get_cpu_temp():
    """
    Read CPU temperature. Tries common sensor keys in priority order
    so it works on Raspberry Pi (cpu_thermal), Jetson, and x86 boards.
    Returns 0.0 if no sensor is available.
    """
    temps = psutil.sensors_temperatures()
    if not temps:
        return 0.0

    for key in ["cpu_thermal", "coretemp", "k10temp", "acpitz"]:
        if key in temps and temps[key]:
            return temps[key][0].current

    first_key = next(iter(temps))
    return temps[first_key][0].current if temps[first_key] else 0.0


# ─────────────────────────────────────────────
# GPU readers — Raspberry Pi (vcgencmd)
#
# Platform notes:
#   RPi    -> vcgencmd is available out of the box.
#             User must be in the 'video' group: sudo usermod -aG video $USER
#   Jetson -> replace _vcgencmd calls with tegrastats parsing. Stub:
#             result = subprocess.run(["tegrastats", "--interval", "1000"],
#                         capture_output=True, text=True, timeout=2)
#             parse "GPU 45%" and "Temp GPU@52C" from result.stdout
#   x86    -> vcgencmd absent; both functions return 0.0 gracefully.
# ─────────────────────────────────────────────
def _vcgencmd(args):
    """Run a vcgencmd command, return stdout string or None on failure."""
    try:
        result = subprocess.run(
            ["vcgencmd"] + args,
            capture_output=True, text=True, timeout=2
        )
        if result.returncode == 0:
            return result.stdout.strip()
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass
    return None


def get_gpu_temp():
    """
    RPi GPU temperature via vcgencmd measure_temp gpu.
    Returns degrees Celsius, or 0.0 if vcgencmd unavailable.
    Output format: "temp=47.2'C"
    """
    out = _vcgencmd(["measure_temp", "gpu"])
    if out:
        try:
            return float(out.split("=")[1].replace("'C", "").strip())
        except (IndexError, ValueError):
            log.warning("Could not parse GPU temp from: %s", out)
    return 0.0


def get_gpu_mem_mb():
    """
    RPi GPU memory split via vcgencmd get_mem gpu.
    Returns reserved GPU memory in MB. This is a STATIC value configured
    in /boot/config.txt (gpu_mem=) — it does not change at runtime.
    Broadcast once at startup as informational STATUSTEXT only.
    Output format: "gpu=128M"
    """
    out = _vcgencmd(["get_mem", "gpu"])
    if out:
        try:
            return float(out.split("=")[1].replace("M", "").strip())
        except (IndexError, ValueError):
            log.warning("Could not parse GPU mem from: %s", out)
    return 0.0


# ─────────────────────────────────────────────
# Critical Services Watchdog
# ─────────────────────────────────────────────
class ServiceWatchdog:
    """
    Monitors a companion process by name.

    Lifecycle:
      1. Process seen alive  -> was_alive = True, restart_count = 0.
      2. Process disappears  -> attempt systemctl restart (up to max_attempts).
      3. Restart succeeds    -> log recovery, reset counter, continue.
      4. All restarts fail   -> trigger RTL failsafe.

    Config entry (configuration.yaml):
      watchdog:
        processes:
          - name: qrtest1          # process name as shown in ps/top
            service: qrtest1.service  # systemd unit name
        max_restart_attempts: 2
    """

    def __init__(self, proc_name, service_name, max_attempts=2):
        self.proc_name     = proc_name
        self.service_name  = service_name
        self.max_attempts  = max_attempts
        self.restart_count = 0
        self.was_alive     = False

    def _is_running(self):
        for proc in psutil.process_iter(["name"]):
            try:
                if proc.info["name"] == self.proc_name:
                    return True
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        return False

    def _attempt_restart(self):
        """
        Run systemctl restart and wait up to 3s for the process to appear.
        Returns True if the process is alive after restart.
        """
        log.warning("Watchdog: systemctl restart %s (attempt %d/%d)",
                    self.service_name, self.restart_count + 1, self.max_attempts)
        try:
            result = subprocess.run(
                ["systemctl", "restart", self.service_name],
                capture_output=True, text=True, timeout=10
            )
            if result.returncode != 0:
                log.error("systemctl restart stderr: %s", result.stderr.strip())
                return False
        except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
            log.error("systemctl restart exception: %s", exc)
            return False

        for _ in range(6):          # poll for 3 seconds (6 x 0.5s)
            time.sleep(0.5)
            if self._is_running():
                log.info("Watchdog: %s is alive after restart.", self.proc_name)
                return True

        log.error("Watchdog: %s did not appear after restart.", self.proc_name)
        return False

    def check(self):
        alive = self._is_running()

        if alive:
            self.was_alive     = True
            self.restart_count = 0
            return

        if not self.was_alive:
            # Never seen — don't act (process may not be expected yet)
            return

        log.error("Watchdog: %s is DEAD", self.proc_name)
        send_statustext("WD DEAD: " + self.proc_name[:10], severity=3, force=True)

        if self.restart_count < self.max_attempts:
            self.restart_count += 1
            success = self._attempt_restart()

            if success:
                send_statustext(self.proc_name[:10] + " RESTARTED", severity=5, force=True)
                self.restart_count = 0
                return

        # Exhausted all restart attempts
        log.error("Watchdog: %s failed all %d restart attempts -> RTL",
                  self.proc_name, self.max_attempts)
        send_statustext("WD RTL: " + self.proc_name[:8], severity=2, force=True)
        trigger_rtl(reason="WD:" + self.proc_name)


# ─────────────────────────────────────────────
# Global health check (uses monitor averages)
# ─────────────────────────────────────────────
def all_monitors_healthy(monitors):
    """
    Returns True only when every monitor's rolling average is below its
    WARNING threshold — consistent with per-monitor rolling logic.
    """
    return all(m.avg < m.warning for m in monitors)


# ─────────────────────────────────────────────
# Create monitors
# ─────────────────────────────────────────────
cpu_monitor  = ResourceMonitor("CPU",      float_key="CPU_PCT",
                               warning=cpu_cfg["warning"],   high=cpu_cfg["high"],   critical=cpu_cfg["critical"])
ram_monitor  = ResourceMonitor("RAM",      float_key="RAM_PCT",
                               warning=ram_cfg["warning"],   high=ram_cfg["high"],   critical=ram_cfg["critical"])
disk_monitor = ResourceMonitor("DISK",     float_key="DISK_PCT",
                               warning=disk_cfg["warning"],  high=disk_cfg["high"],  critical=disk_cfg["critical"])
temp_monitor = ResourceMonitor("CPU_TEMP", float_key="CPU_TEMP",
                               warning=temp_cfg["warning"],  high=temp_cfg["high"],  critical=temp_cfg["critical"])
gpu_monitor  = ResourceMonitor("GPU_TEMP", float_key="GPU_TEMP",
                               warning=gpu_temp_cfg["warning"], high=gpu_temp_cfg["high"], critical=gpu_temp_cfg["critical"])

all_monitors = [cpu_monitor, ram_monitor, disk_monitor, temp_monitor, gpu_monitor]


# ─────────────────────────────────────────────
# Create watchdogs from config
# ─────────────────────────────────────────────
watchdogs = [
    ServiceWatchdog(
        proc_name    = p["name"],
        service_name = p["service"],
        max_attempts = max_restarts
    )
    for p in watched_procs
]

if watchdogs:
    log.info("Watchdog active for: %s", [p["name"] for p in watched_procs])
else:
    log.info("No processes configured for watchdog.")


# ─────────────────────────────────────────────
# Startup broadcasts
# ─────────────────────────────────────────────
gpu_mem = get_gpu_mem_mb()
if gpu_mem > 0:
    log.info("RPi GPU memory split: %.0fMB (static — set in /boot/config.txt)", gpu_mem)
    send_statustext("GPU MEM {:.0f}MB".format(gpu_mem), severity=6, force=True)
    send_named_float("GPU_MEM_MB", gpu_mem)
else:
    log.info("vcgencmd unavailable — GPU memory info skipped.")


# ─────────────────────────────────────────────
# Recovery counter
# ─────────────────────────────────────────────
recovery_counter = 0


# ─────────────────────────────────────────────
# Main monitoring loop
# ─────────────────────────────────────────────
log.info(
    "Starting Companion Health Monitor | "
    "failsafe_after=%ds recovery_after=%ds "
    "statustext_crit=%ds statustext_normal=%ds",
    critical_failsafe_secs, recovery_secs,
    statustext_interval_crit, statustext_interval_norm
)

try:
    while True:
        cpu      = get_cpu()
        ram      = get_ram()
        disk     = get_disk()
        cpu_temp = get_cpu_temp()
        gpu_temp = get_gpu_temp()

        cpu_monitor.update(cpu)
        ram_monitor.update(ram)
        disk_monitor.update(disk)
        temp_monitor.update(cpu_temp)
        gpu_monitor.update(gpu_temp)

        log.info(
            "CPU=%.1f%% RAM=%.1f%% DISK=%.1f%% "
            "CPU_T=%.1fC GPU_T=%.1fC | state=%s",
            cpu_monitor.avg, ram_monitor.avg, disk_monitor.avg,
            temp_monitor.avg, gpu_monitor.avg,
            system_state.name
        )

        # ── Services watchdog ──────────────────
        for wd in watchdogs:
            wd.check()

        # ── Recovery logic ─────────────────────
        if system_state == SystemState.FAILSAFE:
            if all_monitors_healthy(all_monitors):
                recovery_counter += 1
                log.info("Recovery counter: %d/%d", recovery_counter, recovery_timeout)

                if recovery_counter >= recovery_timeout:
                    system_state = SystemState.RECOVERING
                    recover_mode()
                    recovery_counter = 0
            else:
                recovery_counter = 0

        time.sleep(sample_interval)

except KeyboardInterrupt:
    log.info("Monitor stopped by user.")
finally:
    _heartbeat_stop.set()
    heartbeat_thread.join(timeout=2)
    log.info("Heartbeat thread stopped.")