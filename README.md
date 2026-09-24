# V2X-using-SIWG917Xbrd2605

Ultra-low-power V2X prototype built with the Silicon Labs SiWx917 (BRD2605A), enabling multi-protocol wireless connectivity for vehicle-to-infrastructure (V2I) and vehicle-to-vehicle (V2V) communication[cite: 3].

---

## Dual-Node V2X Firmware Architecture

This repository contains two operational node builds designed for the Silicon Labs SiWG917 Pro Kit (BRD2605A)[cite: 1, 3]. They communicate over low-latency UDP broadcast sockets while acquiring continuous on-board motion dynamics via the InvenSense ICM40627 6-axis IMU[cite: 1, 3].

---

### 1. Soft AP Node (`app_soft_ap.c`)
Acts as the central network coordinator, roadside unit (RSU), or infrastructure hub (`V2X_AP_NODE`)[cite: 1].

* **Network Subsystem**:
  * Initializes the Wi-Fi subsystem in 802.11 Access Point (AP) mode on 2.4 GHz (Channel 6, WPA2-PSK CCMP)[cite: 1].
  * Configures a static IPv4 address (`192.168.10.1`, subnet mask `255.255.255.0`) and manages DHCP leasing for incoming station nodes[cite: 1].
  * Binds to UDP port 5000 and services incoming peer frames through a dedicated, blocking receive task (`v2x_rx_task_entry`)[cite: 1].
* **Sensor Acquisition & Telemetry**:
  * Polls the onboard ICM40627 accelerometer over the Ultra-Low-Power (ULP) peripheral bus[cite: 1].
  * Encapsulates raw tri-axial acceleration ($X, Y, Z$ in $g$) into a standardized `v2x_frame_t` payload[cite: 1].
  * Transmits telemetry frames to the broadcast address (`255.255.255.255:5000`) every 100 ms, applying a pseudo-random jitter of 0–20 ms to mitigate RF channel collisions[cite: 1].

---

### 2. Wi-Fi Station Node (`app_station.c`)
Functions as the mobile endpoint, on-board unit (OBU), or client vehicle node[cite: 1].

* **Network Subsystem**:
  * Configured in Station (STA) client mode to scan, associate, and authenticate with `V2X_AP_NODE`[cite: 1].
  * Dynamically acquires an IP address via DHCP upon association[cite: 1].
  * Binds to UDP port 5000 and directs telemetry packets to the subnet broadcast address (`192.168.10.255:5000`)[cite: 1].
* **Sensor Acquisition & Telemetry**:
  * Runs a synchronized polling loop capturing kinematics from its local ICM40627 IMU[cite: 1].
  * Assembles, timestamps, and calculates checksums for outgoing `v2x_frame_t` frames on a matching 100 ms baseline schedule[cite: 1].
  * Maintains an independent receiver thread so listening for incoming RSU broadcasts never delays active sensor reads[cite: 1].

---

### 3. Concurrency, Integrity & Reliability
Both implementations are built on FreeRTOS using CMSIS-RTOS2 wrappers and share core reliability mechanisms:

* **Decoupled TX/RX Threads**: Telemetry broadcasting (`v2x_net_task`, stack: 3584 bytes) and packet reception (`v2x_rx_task`, stack: 3072 bytes) run as separate RTOS tasks[cite: 3]. Because socket `recvfrom()` calls block until data arrives, thread isolation prevents RX waits from stalling IMU sampling loops[cite: 3].
* **Frame Validation Pipeline**: Incoming packets undergo four-stage validation:
  1. Payload byte-length verification against `sizeof(v2x_frame_t)` (42 bytes)[cite: 3].
  2. Magic byte matching (`V2X_FRAME_MAGIC` / `0x56325831`) and protocol version checks[cite: 3].
  3. Fletcher-16 checksum integrity verification via `v2x_validate_frame()`[cite: 3].
  4. Local MAC comparison against `s_local_mac` to silently discard reflected self-broadcasts[cite: 3].
* **Stack Guard**: Features `vApplicationStackOverflowHook` to catch memory pressure early, outputting fault diagnostics over VCOM/UART before halting safely[cite: 3].

---

### 4. Packet Payload Structure (`bsm_packet.h`)

Telemetric frames enforce strict 1-byte packing (`#pragma pack(push, 1)`) yielding an exact 42-octet over-the-air payload footprint[cite: 3]:

| Offset (Hex) | Field Identifier | Type | Size | Description |
| :--- | :--- | :--- | :--- | :--- |
| `0x00 - 0x03` | `magic` | `uint32_t` | 4 B | Frame synchronization identifier (`0x56325831`, "V2X1")[cite: 3] |
| `0x04` | `version` | `uint8_t` | 1 B | Protocol schema version (`0x01`)[cite: 3] |
| `0x05` | `message_type` | `uint8_t` | 1 B | Operational frame type (`0x01` = Telemetry, `0x02` = Emergency Alert)[cite: 3] |
| `0x06 - 0x07` | `payload_length`| `uint16_t` | 2 B | Total length of wire frame (42 Bytes)[cite: 3] |
| `0x08 - 0x09` | `sequence` | `uint16_t` | 2 B | Rolling packet sequence counter[cite: 3] |
| `0x0A - 0x0F` | `sender_id[6]` | `uint8_t[6]` | 6 B | Hardware IEEE 802.11 MAC address of transmitting node[cite: 3] |
| `0x10 - 0x13` | `timestamp_ms` | `uint32_t` | 4 B | Node uptime timestamp in milliseconds[cite: 3] |
| `0x14 - 0x17` | `accel_x` | `float` | 4 B | Longitudinal acceleration in $g$ (IEEE 754)[cite: 3] |
| `0x18 - 0x1B` | `accel_y` | `float` | 4 B | Lateral acceleration in $g$ (IEEE 754)[cite: 3] |
| `0x1C - 0x1F` | `accel_z` | `float` | 4 B | Vertical acceleration in $g$ (IEEE 754)[cite: 3] |
| `0x20` | `gesture` | `uint8_t` | 1 B | Motion / anomaly event ID classification code[cite: 3] |
| `0x21` | `emergency` | `uint8_t` | 1 B | Safety Flag (`0` = Normal/Safe, `1` = Hard Deceleration / Crash Alert)[cite: 3] |
| `0x22 - 0x23` | `reserved` | `uint16_t` | 2 B | Boundary alignment / expansion word (cleared to `0x0000`)[cite: 3] |
| `0x24 - 0x27` | `confidence` | `float` | 4 B | Metric confidence / anomaly probability score ($0.000$ to $1.000$)[cite: 3] |
| `0x28 - 0x29` | `checksum` | `uint16_t` | 2 B | Fletcher-16 checksum evaluated over bytes `0x00` to `0x27` (40 bytes)[cite: 3] |

---

### 5. Hardware & SDK Environment

* **Target Hardware**: Silicon Labs SiWG917 Pro Kit (`BRD2605A` / `SIWG917M111MGTBA`)[cite: 3]
* **Motion Sensor**: InvenSense ICM40627 6-axis IMU (Ultra-Low-Power SSI interface)[cite: 3]
* **SDK Platform**: Silicon Labs Simplicity SDK `2025.12.3` / WiSeConnect 3 SDK `4.0.2`[cite: 3]
* **RTOS**: FreeRTOS with CMSIS-RTOS2 abstraction[cite: 3]

---

### 6. Building and Flashing

Because both node variants share the same build lifecycle entry points (`app_init` and `app_process_action`), select the target role before compiling in Simplicity Studio:

1. Open `v2_X.slcp` in Simplicity Studio v5.
2. Select the target file for compilation:
   * To build the **Access Point (RSU)**: Include `app_soft_ap.c` in the build and exclude `app_station.c` (right-click $\rightarrow$ **Resource Configurations** $\rightarrow$ **Exclude from Build**), or copy/symlink `app_soft_ap.c` to `app.c`.
   * To build the **Station (Vehicle Node)**: Include `app_station.c` and exclude `app_soft_ap.c`, or copy/symlink `app_station.c` to `app.c`.
3. Build the project using the GNU ARM Toolchain (`-u _printf_float` enabled)[cite: 3].
4. Flash the generated `.rps` image via Simplicity Commander[cite: 3]:
### 7. Future Development

The current architecture provides a deterministic, low-latency foundation for vehicle-to-everything communications. The extensible frame schema (`bsm_packet.h`) and hardware configuration (`v2_X.slcp`) are structured to support three key enhancements currently in engineering:

* **Kinematic Anomaly & Hazard Detection Engine**:
  * Real-time statistical and dual-counter hysteresis classifiers running directly on the ARM Cortex-M4 core.
  * Continuous tri-axial inertial processing from the ICM40627 to detect hard deceleration (emergency braking), severe lateral jerks (aggressive swerving), rollover risk, and collision shock impulses.
  * Activation of pre-allocated frame schema fields (`gesture` for anomaly class, `confidence` for certainty metric, and `emergency` for priority flagging) without altering wire protocol dimensions.
* **Dynamic Hardware Authentication Tokens (TOTP Tagging)**:
  * Hardware-rooted cryptographic signing leveraging the SiWG917 True Random Number Generator (TRNG) and peripheral HMAC-SHA256 acceleration.
  * Injecting rolling cryptographic authentication tags into a dedicated 32-bit field (`v2x_frame_t` v2) to prevent broadcast spoofing and replay attacks across sliding verification windows.
* **Multi-Node Mesh & Protocol Coexistence**:
  * Extending client station management to support automatic discovery and handoffs across multiple roadside units (RSUs).
  * Implementing adaptive transmission scheduling to minimize RF contention under dense vehicle cluster environments.
  * Optimizing Ultra-Low-Power (ULP) sleep timer states between transmission deadlines to maximize energy efficiency on battery-powered client nodes.
