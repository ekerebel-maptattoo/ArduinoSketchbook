# MAPTATTOO N2K Gateway — Arduino Sketchbook

This repository contains the Arduino firmware for the **MAPTATTOO N2K BLE Gateway**: a hardware bridge that connects a boat's NMEA 2000 (N2K) network to Bluetooth Low Energy (BLE) clients, primarily the [MAPTATTOO](https://maptattoo.com) GPS tablet app.

---

## Repository Structure

```
gateway_n2k/
    gateway_n2k.ino          ← Production firmware: live N2K → BLE gateway

gateway_n2k_simulation/
    gateway_n2k_simulation.ino  ← Simulator firmware: synthetic data, no N2K bus required

libraries/
    NimBLE-Arduino/          ← NimBLE Arduino library (bundled, used by both sketches)

libraries_DISABLED/
    NimBLE-Arduino_DISABLED/ ← Disabled copy (kept for reference)
```

---

## Sketches

### `gateway_n2k` — Production Gateway

Connects to a real NMEA 2000 network via TWAI (CAN bus) and streams decoded PGN data to up to **3 simultaneous BLE clients**.

**What it does:**
- Listens on the N2K bus at 250 kbps using the ESP32 TWAI (CAN) peripheral
- Decodes 29-bit extended CAN frames into N2K PGN / source address / priority
- Reassembles multi-frame **fast-packet** PGNs (e.g. AIS, GNSS)
- Forwards each PGN payload over BLE as a NOTIFY on a dedicated BLE characteristic
- Streams both **AIS** (Class A/B position, static, AtoN, UTC) and all standard navigation/environmental PGNs
- Throttles non-AIS PGNs to **1 Hz per PGN/source** and AIS to **10 Hz per PGN/source** to prevent BLE notify queue pressure
- Broadcasts a pseudo-PGN (65281) with the current BLE RSSI every second

**Framing — Option A:**
Every BLE notification payload is framed as:
```
byte[0]    = N2K source address (SRC, 0x00–0xFE; 0xFF = gateway-generated)
byte[1..n] = raw N2K PGN payload bytes (single-frame or reassembled fast-packet)
```

---

### `gateway_n2k_simulation` — Simulator (No N2K Required)

Identical BLE service/characteristic layout and Option A framing as the production gateway, but generates all data synthetically. No CAN bus or N2K network is needed.

**What it simulates:**
- **Own vessel**: starts ~5 NM north of Monterey Harbor (`36.688°N 121.891°W`), moving north at 5 kt with slight heading/SOG oscillation
- **15 AIS targets** scattered across Monterey Bay with realistic MMSIs, vessel names, ship types, and movement
- All environmental / engine / navigation PGNs with smooth oscillating values (depth, wind, temperature, pressure, battery, RPM, etc.)
- Simulated "N2K active" state so the RGB LED behaves identically to the production firmware

**Use cases:**
- Load testing the MAPTATTOO app against a realistic data stream without a boat
- BLE protocol development and debugging
- Regression testing after firmware changes
- Demoing the app in a lab environment

**Firmware version tag:** `1.0.2-sim`

---

## Hardware

| Item | Detail |
|---|---|
| MCU | **Seeed XIAO ESP32-C6** |
| CAN/N2K interface | TWAI peripheral — TX: `GPIO16` (`D6`), RX: `GPIO17` (`D7`) |
| CAN bus speed | 250 kbps |
| Buzzer | `GPIO20` — single beep on BLE client connect |
| External RGB LED | Dialight 5218559F (common-anode): R=`GPIO2`, G=`GPIO0`, B=`GPIO1` |
| Antenna | External antenna selected at boot via `GPIO3` (RF switch enable) + `GPIO14` (select external) |
| BLE TX power | +3 dBm (`ESP_PWR_LVL_P3`) |

### RGB LED Status

| Color | Meaning |
|---|---|
| Solid **red** | Powered, no BLE clients, no N2K |
| Solid **green** | N2K traffic active, no BLE clients connected |
| Solid **blue** | BLE client connected **and** N2K active |
| **Blinking blue** (500 ms) | BLE client connected, but no recent N2K traffic |

---

## BLE Protocol

### Service

| Field | Value |
|---|---|
| Service UUID | `8f000000-1a12-4a86-9e26-9f1c9f0e1234` |
| BLE name | `MAPTATTOO N2K` |
| MTU | 247 bytes |
| Max clients | 3 simultaneous |
| Security | No pairing / no bonding required |

### Device Information Service (0x180A)

| Characteristic | UUID | Value |
|---|---|---|
| Manufacturer Name | `0x2A29` | `MAPTATTOO` |
| Model Number | `0x2A24` | `N2K Gateway (XIAO ESP32-C6)` |
| Serial Number | `0x2A25` | Chip MAC address (hex string) |
| Firmware Revision | `0x2A26` | `1.0.2` |

### Per-PGN Characteristics

Each PGN has a dedicated NOTIFY characteristic. The UUID is derived from the PGN:
```
8f00XXXX-1a12-4a86-9e26-9f1c9f0e1234
     ^^^^
     Low 16 bits of the PGN number (hex)
```

A `0x2901` User Description descriptor is attached to each characteristic with a human-readable label.

#### Supported PGNs

| PGN | Description | Type | Throttle |
|---|---|---|---|
| 65281 | RSSI Status *(gateway pseudo-PGN)* | Single | 1 Hz |
| 126992 | System Time | Single | 1 Hz |
| 127245 | Rudder | Single | 1 Hz |
| 127250 | Vessel Heading | Single | 1 Hz |
| 127257 | Attitude | Single | 1 Hz |
| 127488 | Engine Rapid Update | Single | 1 Hz |
| 127489 | Engine Dynamic Parameters | Fast-packet | 1 Hz |
| 127493 | Transmission Dynamic Parameters | Fast-packet | 1 Hz |
| 127505 | Fluid Level | Single | 1 Hz |
| 127506 | DC Detailed Status | Single | 1 Hz |
| 127508 | Battery Status | Single | 1 Hz |
| 128259 | Water Speed | Single | 1 Hz |
| 128267 | Water Depth | Single | 1 Hz |
| 129025 | Position — Rapid Update | Single | 1 Hz |
| 129026 | COG & SOG — Rapid Update | Single | 1 Hz |
| 129029 | GNSS Position Data | Fast-packet | 1 Hz |
| 129038 | AIS Class A Position Report | Fast-packet | 10 Hz |
| 129039 | AIS Class B Position Report | Fast-packet | 10 Hz |
| 129040 | AIS Class B Extended Position | Fast-packet | 10 Hz |
| 129041 | AIS Aids to Navigation | Fast-packet | 10 Hz |
| 129793 | AIS UTC and Date Report | Fast-packet | 10 Hz |
| 129794 | AIS Class A Static and Voyage | Fast-packet | 10 Hz |
| 129809 | AIS Class B Static Data Part A | Fast-packet | 10 Hz |
| 129810 | AIS Class B Static Data Part B | Fast-packet | 10 Hz |
| 130306 | Wind Data | Single | 1 Hz |
| 130310 | Environmental Parameters | Single | 1 Hz |
| 130311 | Environmental Parameters (legacy) | Single | 1 Hz |
| 130312 | Temperature | Single | 1 Hz |
| 130314 | Actual Pressure | Single | 1 Hz |
| 130316 | Temperature Extended Range | Single | 1 Hz |

---

## Building

### Requirements

- **Arduino IDE 2.x** or **arduino-cli**
- **Board package**: `esp32` by Espressif Systems (install via Board Manager)
  - Board target: `Seeed XIAO ESP32-C6` (`XIAO_ESP32C6`)
- **Library**: NimBLE-Arduino — already bundled in `libraries/NimBLE-Arduino/`
  - Place the `libraries/` folder in your Arduino Sketchbook directory, or point the IDE to it

### Build Steps (Arduino IDE)

1. Open `gateway_n2k/gateway_n2k.ino` (production) or `gateway_n2k_simulation/gateway_n2k_simulation.ino` (simulator)
2. Select board: **Tools → Board → esp32 → XIAO_ESP32C6**
3. Select the correct serial port
4. Click **Upload**

### Build Steps (arduino-cli)

```bash
# Install board package (once)
arduino-cli core install esp32:esp32

# Compile production gateway
arduino-cli compile \
  --fqbn esp32:esp32:XIAO_ESP32C6 \
  gateway_n2k/

# Upload
arduino-cli upload \
  --fqbn esp32:esp32:XIAO_ESP32C6 \
  --port /dev/ttyUSB0 \
  gateway_n2k/
```

### Serial Monitor

Both sketches emit diagnostic output at **115200 baud**. Key log lines:

| Prefix | Meaning |
|---|---|
| `SUMMARY ...` | Periodic counters (every 5 s): CAN frames, BLE notify stats, fast-packet stats |
| `HEAP ...` | Free heap / minimum free / largest block (every 60 s) |
| `TWAI status: ...` | TWAI driver state and error counters (every 1 s) |
| `RSSI (connected) = X dBm` | BLE RSSI to connected client(s) (every 1 s) |
| `BLE CONNECT ...` | Client connected (address, handle, count) |
| `BLE DISCONNECT ...` | Client disconnected (address, handle, reason code) |
| `SUBSCRIBE PGN ...` | Client subscribed / unsubscribed to a characteristic |

### Tuning Flags

Both sketches have compile-time boolean flags at the top to enable/disable verbose logging without recompiling everything:

| Flag | Default | Effect |
|---|---|---|
| `LOG_CAN_RX` | `false` | Print every raw incoming CAN frame |
| `LOG_PGN_DECODE` | `false` | Print decoded PGN/SRC/DST for every frame |
| `LOG_NOTIFY_FLOW` | `false` | Print every notify attempt and reason for skip/throttle |
| `LOG_FP_FRAMES` | `false` | Deep fast-packet reassembly debug (very chatty) |
| `LOG_SUBSCRIPTION` | `true` | Log client subscribe/unsubscribe events |
| `LOG_TWAI_STATUS_1HZ` | `true` | TWAI bus health once per second |

---

## Firmware Versions

| Sketch | Version |
|---|---|
| `gateway_n2k` | `1.0.2` |
| `gateway_n2k_simulation` | `1.0.2-sim` |