# ArduPilot_Companion_Health_Monitor
Realtime ArduPilot companion computer health monitoring system running alongside Ardupilot flight controller using MAVLink, providing CPU, memory, temperature, and system status telemetry to GCS.


# Problem statement
"There is no standardar way to know the status of Companion computers(Jetson, Raspberry Pi, etc) if they are working fine or facing any issue."


Modern UAV systems frequently use a companion computer (Raspberry Pi, Jetson, etc.) for:Computer vision, Obstacle avoidance, AI inference, Custom autonomy logic

However, ArduPilot currently has no standardized way to monitor the health of the companion computer.
If the companion:Overheats, Crashes, Runs out of memory, Hangs during a mission
The flight controller remains unaware.

# Solution
To build a daemon that runs on the companion computer and:
- Monitors CPU usage
- Monitors RAM usage
- Monitors temperature
- Detects process crashes
- Reports system health to ArduPilot via MAVLink

The flight controller can then:
- Trigger failsafe
- Log warnings
- Notify GCS
- Take autonomous safety action

# Features
On the Companion Computer side :

- Heartbeat publisher — sends a MAVLink heartbeat every N seconds
- System health reporter — CPU %, RAM %, GPU %, disk %, core temperature
- Critical service watchdog — monitors user-defined services (your object detection node, ROS2 nodes, etc.)
- Watchdog timer — if the daemon itself dies, ArduPilot detects the missing heartbeat
- Configurable via a YAML/JSON config file

On the ArduPilot side :

- New MAVLink message definition: COMPANION_HEALTH (or extend SYSTEM_STATUS)
- Heartbeat timeout detector — if companion goes silent for X seconds, trigger failsafe
- Health threshold triggers — if CPU > 95% for Y seconds, trigger failsafe
- Configurable failsafe actions per trigger: RTL / Land / Hold / Warn only
- New ArduPilot parameters: CC_FS_ENABLE, CC_HB_TIMEOUT, CC_CPU_THRESH, CC_FS_ACTION

# Architecture

# tech stack
| #  | Feature                       | Description                                                                 |
|----|-------------------------------|-----------------------------------------------------------------------------|
| 1  | Heartbeat Watchdog            | Companion publishes a MAVLink heartbeat every second. ArduPilot triggers failsafe on silence exceeding `CC_HB_TIMEOUT`. |
| 2  | System Health Reporting       | CPU%, RAM%, GPU%, disk, and core temperature sent via new `COMPANION_HEALTH` MAVLink message every 2s. |
| 3  | Critical Service Watchdog     | User-defined services (ROS2, MAVROS, detection pipelines) monitored via `systemctl`. Status encoded as bitmask in message. |
| 4  | Configurable Failsafe Actions | Per-trigger actions: `RTL` / `Land` / `Hold` / `Warn-only` / `None` — set via ArduPilot parameters. |
| 5  | New ArduPilot Parameter Set   | `CC_FS_*` parameters for enable/disable, thresholds, timeouts, and actions — no recompile required. |
| 6  | GCS Live Dashboard            | MAVProxy module shows real-time companion health in the ground station terminal. |
| 7  | Full SITL Test Suite          | Every failure mode (heartbeat loss, CPU spike, service death, recovery) simulatable in SITL — no hardware needed. |
| 8  | Systemd Service Integration   | Daemon runs as a managed systemd service. If the process itself freezes, systemd restarts it — and ArduPilot detects the heartbeat gap. |
| 9  | YAML Configuration            | Clean, human-readable config for connection settings, thresholds, and which services to monitor. |

# References

- ArduPilot Developer Documentation
- MAVLink Developer Guide
- pymavlink — GitHub
- ArduPilot SITL Setup
- GSoC 2026 ArduPilot Project Ideas

# Contributing
This project is being developed as a GSoC 2026 proposal for the ArduPilot organization. Issues, suggestions, and PRs are very welcome.

- Fork the repo
- Create your feature branch: git checkout -b feature/your-feature
- Commit your changes: git commit -m 'Add: your feature'
- Push to the branch: git push origin feature/your-feature
- Open a Pull Request

#  Motivation & Background
I've built autonomous drones using Pixhawk and ArduPilot, and worked with DroneKit SDK and MAVSDK to implement autonomous mission execution. Through that hands-on experience, I repeatedly encountered the exact problem this project addresses — there is simply no standardized, reliable way for the flight controller to know when the companion computer powering the autonomy stack has degraded or failed.
This project defines that standard — not as a one-off script, but as a proper MAVLink-native, ArduPilot-integrated mechanism that any companion computer can implement and any ArduPilot user can configure.


