## Dual-Node V2X Firmware Architecture

This repository contains two operational node builds designed for the Silicon Labs SiWG917 Pro Kit (BRD2605A). They communicate over low-latency UDP broadcast sockets while sampling on-board motion data via the InvenSense ICM40627 6-axis IMU.

---

### 1. Soft AP Node (`app_soft_ap.c`)
This file builds the central hub or roadside unit of the setup (`V2X_AP_NODE`).

* **Network Setup**:
  * Spins up the Wi-Fi subsystem in Access Point mode on 2.4 GHz (Channel 6, WPA2-PSK).
  * Assigns itself a static IP of `192.168.10.1` and manages DHCP for connecting stations.
  * Listens on UDP port 5000 using a dedicated, blocking receive task (`v2x_rx_task_entry`).
* **IMU & Transmission**:
  * Samples the onboard ICM40627 accelerometer over the ULP interface.
  * Packages the latest three-axis acceleration values into a structured `v2x_frame_t` payload.
  * Broadcasts packets to `255.255.255.255:5000` every 100 ms, introducing random 0–20 ms jitter to prevent channel collisions when multiple boards transmit at once.

---

### 2. Wi-Fi Station Node (`app_station.c`)
This file builds the client or vehicle endpoint that connects to the access point.

* **Network Setup**:
  * Configured in Station mode to search for and authenticate with `V2X_AP_NODE`.
  * Pulls an IP address via DHCP from the Soft AP once connected.
  * Binds to UDP port 5000 and directs outgoing packets to the subnet broadcast address (`192.168.10.255:5000`).
* **IMU & Transmission**:
  * Runs its own continuous polling loop to capture local motion from its onboard ICM40627.
  * Assembles, seals, and calculates checksums for outgoing `v2x_frame_t` packets on the same 100 ms schedule.
  * Uses an independent receiver thread so listening for incoming messages never interferes with sensor reads or transmissions.

---

### Concurrency and Reliability
Both implementations are built on FreeRTOS using CMSIS-RTOS2 wrappers and share the same core safety principles:

* **Split TX and RX Paths**: Transmission (`v2x_net_task`, 3584 bytes stack) and reception (`v2x_rx_task`, 3072 bytes stack) run on separate threads. Because `recvfrom()` blocks until data arrives, separating them ensures the sensor read and broadcast cadence remains uninterrupted.
* **Frame Validation**: Inbound frames are checked for magic bytes (`V2X_FRAME_MAGIC`), verified for valid payload length, checked against packet checksums, and matched against the local MAC address to discard self-broadcast echoes.
* **Stack Monitoring**: Includes a `vApplicationStackOverflowHook` that prints a terminal diagnostic and halts safely if memory limits are exceeded during heavy network activity.