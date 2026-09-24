# V2X-using-SIWG917Xbrd2605

Ultra-low-power V2X prototype built with the Silicon Labs SiWx917 (BRD2605A), enabling multi-protocol wireless connectivity for vehicle-to-infrastructure (V2I) and vehicle-to-vehicle (V2V) communication.

---

## Dual-Node V2X Firmware Architecture

This repository contains two operational node builds designed for the Silicon Labs SiWG917 Pro Kit (BRD2605A). They communicate over low-latency UDP broadcast sockets while acquiring continuous on-board motion dynamics via the InvenSense ICM40627 6-axis IMU.

---

### 1. Soft AP Node (`app_soft_ap.c`)
Acts as the central network coordinator, roadside unit (RSU), or infrastructure hub (`V2X_AP_NODE`).

* **Network Subsystem**:
  * Initializes the Wi-Fi subsystem in 802.11 Access Point (AP) mode on 2.4 GHz (Channel 6, WPA2-PSK CCMP).
  * Configures a static IPv4 address (`192.168.10.1`, subnet mask `255.255.255.0`) and manages DHCP leasing for incoming station nodes.
  * Binds to UDP port 5000 and services incoming peer frames through a dedicated, blocking receive task (`v2x_rx_task_entry`).
* **Sensor Acquisition & Telemetry**:
  * Polls the onboard ICM40627 accelerometer over the Ultra-Low-Power (ULP) peripheral bus.
  * Encapsulates raw tri-axial acceleration ($X, Y, Z$ in $g$) into a standardized `v2x_frame_t` payload.
  * Transmits telemetry frames to the global broadcast address (`255.255.255.255:5000`) every 100 ms, applying a pseudo-random jitter of 0–20 ms to mitigate RF channel collisions.

---

### 2. Wi-Fi Station Node (`app_station.c`)
Functions as the mobile endpoint, on-board unit (OBU), or client vehicle node.

* **Network Subsystem**:
  * Configured in Station (STA) client mode to scan, associate, and authenticate with `V2X_AP_NODE`.
  * Dynamically acquires an IP address via DHCP upon association.
  * Binds to UDP port 5000 and directs telemetry packets to the local subnet broadcast address (`192.168.10.255:5000`).
* **Sensor Acquisition & Telemetry**:
  * Runs a synchronized polling loop capturing kinematics from its local ICM40627 IMU.
  * Assembles, timestamps, and seals outgoing `v2x_frame_t` frames on a matching 100 ms baseline schedule.
  * Maintains an independent receiver thread so listening for incoming RSU broadcasts never delays active sensor reads.

---

### 3. Concurrency, Integrity & Reliability
Both implementations are built on FreeRTOS using CMSIS-RTOS2 wrappers and share core reliability mechanisms:

* **Decoupled TX/RX Threads**: Telemetry broadcasting (`v2x_net_task`, stack: 3584 bytes) and packet reception (`v2x_rx_task`, stack: 3072 bytes) run as separate RTOS tasks. Because socket `recvfrom()` calls block until data arrives, thread isolation prevents RX waits from stalling IMU sampling loops.
* **Frame Validation Pipeline**: Incoming packets undergo four-stage validation:
  1. Payload byte-length verification against `sizeof(v2x_frame_t)`.
  2. Magic byte matching (`V2X_FRAME_MAGIC`) and protocol version checks.
  3. CRC / checksum verification via `v2x_validate_frame()`.
  4. Local MAC comparison to silently discard reflected self-broadcasts.
* **Stack Guard**: Features `vApplicationStackOverflowHook` to catch memory pressure early, outputting fault diagnostics over VCOM/UART before halting safely.

---

### 4. Packet Payload Structure (`bsm_packet.h`)

Telemetric frames are exchanged via a fixed-length schema optimized for low overhead:

| Field | Type | Description |
| :--- | :--- | :--- |
| `magic` | `uint32_t` | Frame synchronization identifier (`V2X_FRAME_MAGIC`) |
| `version` | `uint8_t` | Protocol schema version |
| `message_type` | `uint8_t` | Operational frame type (e.g., periodic telemetry, alert) |
| `payload_length`| `uint16_t` | Total size of the frame payload in bytes |
| `sequence` | `uint16_t` | Rolling packet sequence counter |
| `sender_id` | `uint8_t[6]` | Originating node MAC address |
| `timestamp_ms` | `uint32_t` | Local node uptime timestamp in milliseconds |
| `accel_x/y/z` | `float` | 3-axis acceleration readings in units of $g$ |
| `gesture` | `uint8_t` | Extensible gesture classification identifier |
| `emergency` | `uint8_t` | Critical state / emergency flag |
| `confidence` | `float` | Inference / metric confidence score |
| `reserved` | `uint32_t` | Hardware authentication tag / expansion payload |
| `checksum` | `uint16_t` | Data integrity checksum |

---

### 5. Hardware & SDK Environment

* **Target Hardware**: Silicon Labs SiWG917 Pro Kit (`BRD2605A` / `SIWG917M111MGTBA`)
* **Motion Sensor**: InvenSense ICM40627 6-axis IMU (Ultra-Low-Power SSI interface)
* **SDK Platform**: Silicon Labs Simplicity SDK `2025.12.3` / WiSeConnect 3 SDK `4.0.2`
* **RTOS**: FreeRTOS with CMSIS-RTOS2 abstraction

---

### 6. Building and Flashing

Because both node variants share the same build lifecycle entry points (`app_init` and `app_process_action`), select the target role before compiling in Simplicity Studio:

1. Open `v2_X.slcp` in Simplicity Studio v5.
2. Select the target file for compilation:
   * To build the **Access Point (RSU)**: Include `app_soft_ap.c` in the build and exclude `app_station.c` (right-click $\rightarrow$ **Resource Configurations** $\rightarrow$ **Exclude from Build**), or copy/symlink `app_soft_ap.c` to `app.c`.
   * To build the **Station (Vehicle Node)**: Include `app_station.c` and exclude `app_soft_ap.c`, or copy/symlink `app_station.c` to `app.c`.
3. Build the project using the GNU ARM Toolchain (`-u _printf_float` enabled).
4. Flash the generated binary (`.rps` or `.hex`) to the respective BRD2605A radio board.

---

### 7. Future Development

The current architecture provides a hardened real-time foundation for vehicle-to-everything communications. The platform's extensible frame layout (`bsm_packet.h`) and hardware configuration (`v2_X.slcp`) are structured to support three key enhancements currently in active engineering:

* **Edge AI/ML Kinematic & Anomaly Classification**:
  * Integrating lightweight inference pipelines directly on the Cortex-M4 core.
  * Utilizing real-time tri-axial IMU streams from the ICM40627 to classify abrupt lane changes, harsh deceleration/braking events, rollover risk, and collision impulses.
  * Activating pre-allocated frame schema fields (`gesture`, `confidence`, and `emergency`) without breaking backward compatibility.
* **Dynamic Hardware Authentication Tokens (TOTP Tagging)**:
  * Leveraging the SiWG917 internal hardware True Random Number Generator (TRNG) and HMAC-SHA256 crypto acceleration.
  * Injecting rolling cryptographic authentication tags into the 32-bit `reserved` field of `v2x_frame_t`.
  * Mitigating packet replay attacks via a synchronized verification window without introducing network handshake latency.
* **Multi-Node Mesh & Protocol Coexistence**:
  * Extending client station management to support automatic discovery and seamless handoffs across multiple roadside units.
  * Implementing adaptive transmission scheduling to minimize RF contention under dense vehicle cluster environments.
  * Optimizing Ultra-Low-Power (ULP) sleep timer states between transmission deadlines to maximize energy efficiency on battery-powered client nodes.
