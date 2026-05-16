# MAPTATTOO N2K Gateway — Changelog

All changes to the simulator are documented here with enough detail to port them
to `gateway_n2k` (the production firmware) when ready.

---

## gateway_n2k_simulation — 1.0.4-sim

### Change 1 — Pause notifications on new client connect

**Why:** When a new BLE client connects it needs 2+ seconds to perform service
discovery and subscribe to characteristics. If notifications are already flowing
(e.g. a second client connects while a first is already streaming), the
connection procedure can be disrupted by the BLE notify queue pressure.

**What was added:**

```cpp
// Near top of file, after global variable declarations
static const uint32_t CONNECT_PAUSE_MS        = 2000;
static uint32_t       gConnectingPauseUntilMs = 0;

static inline bool isConnectPaused(uint32_t nowMs) {
  return nowMs < gConnectingPauseUntilMs;
}
```

In `SrvCB::onConnect`:
```cpp
gConnectingPauseUntilMs = millis() + CONNECT_PAUSE_MS;
Serial.printf("CONNECT PAUSE: notifications paused for %lu ms\n",
              (unsigned long)CONNECT_PAUSE_MS);
```

In `notifyPGN()` — added as the first check (before any map lookups):
```cpp
if (isConnectPaused(millis())) { gBleNotifyThrottled++; return; }
```

In `notifyRSSI_1Hz()` — added after the `getConnectedCount() == 0` guard:
```cpp
if (isConnectPaused(nowMs)) return;
```

**Port to gateway_n2k:** Identical changes apply. The four insertion points are
the same in the production sketch.

**Open question:** Logs show subscriptions can take 70+ seconds (slow service
discovery across 30 characteristics). Consider resetting the pause on each
`onSubscribe` event to keep the pause alive throughout subscription setup:
```cpp
// In CharCB::onSubscribe (or SrvCB if a server-level callback is available):
gConnectingPauseUntilMs = millis() + CONNECT_PAUSE_MS;
```

---

### Change 2 — Skip simulation entirely when no clients connected

**Why:** The simulator was building payloads and calling `notifyPGN()` even
when nobody was connected, wasting CPU cycles on math and map lookups that
would always be dropped by `hasSubscribers()`.

**What was changed in `loop()`:**

Before:
```cpp
simulateOwnship1Hz(now);
simulateAis10Hz(now);
simulateAisStatic30s(now);
```

After:
```cpp
// Only generate and push data when at least one client is connected.
if (isBleConnected()) {
  simulateOwnship1Hz(now);
  simulateAisRealistic(now);   // ← also renamed; see Change 3
}
```

**Port to gateway_n2k:** The equivalent in the production firmware is the CAN
drain loop. The real gateway *must* keep draining the TWAI RX queue even when
no clients are connected (otherwise the hardware RX queue overflows and frames
are lost). Do **not** gate the CAN/TWAI loop on `isBleConnected()`. Only gate
the `notifyPGN()` calls. The production firmware already does this implicitly
via `hasSubscribers()` returning false, so no change is needed there.

---

### Change 3 — Realistic AIS transmission rates (ITU-R M.1371 / IEC 62287)

**Why:** The original simulator called `notifyPGN()` for all 15 AIS targets
every 100 ms (10 Hz), which is unrealistically high and would stress-test the
app with far more AIS traffic than a real busy coastal scene would produce.

**What was removed:**
- `simulateAis10Hz()` — all 15 targets blasted every 100 ms
- `simulateAisStatic30s()` — static data for all targets every 30 s

**What was added:**

A `SimAisTarget` struct (moved to `sim_types.h` to avoid Arduino preprocessor
auto-prototype issues — see Change 4):
```cpp
struct SimAisTarget {
  uint32_t mmsi;  char name[21];
  double lat0, lon0, sogKt, cogDeg, headingDeg;
  uint8_t shipType;  bool classA;
};
static SimAisTarget gAisTargets[15] = { ... };
```

Per-target timestamp arrays:
```cpp
static uint32_t gAisLastDynMs[15]    = {};
static uint32_t gAisLastStaticMs[15] = {};
```

Interval function based on vessel class and speed:
```cpp
static uint32_t aisDynIntervalMs(const SimAisTarget& a) {
  if (a.sogKt < 0.2)  return 180000u;          // anchored/moored: 3 min
  if (a.classA) {
    if (a.sogKt >= 23.0) return  2000u;         // Class A >23 kt: 2 s
    if (a.sogKt >= 14.0) return  6000u;         // Class A 14-23 kt: 6 s
    if (a.sogKt >=  2.0) return 10000u;         // Class A 2-14 kt: 10 s
    return 180000u;
  } else {
    if (a.sogKt >= 14.0) return 15000u;         // Class B >14 kt: 15 s
    if (a.sogKt >=  2.0) return 30000u;         // Class B 2-14 kt: 30 s
    return 180000u;
  }
}
static const uint32_t AIS_STATIC_INTERVAL_MS = 360000u;  // 6 min all types
```

Startup stagger (called from `setup()`) to avoid all 15 targets firing at once
on first connect:
```cpp
static void initAisSchedule() {
  for (uint8_t i = 0; i < 15; ++i) {
    const uint32_t dynInterval = aisDynIntervalMs(gAisTargets[i]);
    gAisLastDynMs[i]    = millis() - (uint32_t)((uint64_t)dynInterval * i / 15);
    gAisLastStaticMs[i] = millis() - (uint32_t)((uint64_t)AIS_STATIC_INTERVAL_MS * i / 15);
  }
}
```

`simulateAisRealistic(uint32_t nowMs)` replaces the two old functions. For each
target it checks elapsed time against `aisDynIntervalMs()` for dynamic reports,
and against `AIS_STATIC_INTERVAL_MS` for static/voyage data.

AtoN (target index 12) is hardcoded to 180 s and only sends PGN 129041.

**Port to gateway_n2k:** The production gateway does not simulate AIS — it
forwards whatever the N2K bus delivers. This change is simulator-only and does
not need to be ported. The production throttle of 100 ms per PGN/SRC for AIS
remains correct as a ceiling for real bus traffic.

---

### Change 4 — SimAisTarget moved to sim_types.h

**Why:** The Arduino IDE/CLI preprocessor inserts auto-generated function
prototypes after the last `#include` statement. `aisDynIntervalMs()` takes a
`const SimAisTarget&` parameter, so its auto-prototype is generated before any
struct definition that appears in the `.ino` body — causing a compile error
regardless of where in the `.ino` the struct is placed.

**Fix:** Define the struct in a companion header:

`gateway_n2k_simulation/sim_types.h`:
```cpp
#pragma once
struct SimAisTarget {
  uint32_t mmsi;  char name[21];
  double lat0, lon0, sogKt, cogDeg, headingDeg;
  uint8_t shipType;  bool classA;
};
```

`gateway_n2k_simulation.ino`:
```cpp
#include <Arduino.h>
#include <NimBLEDevice.h>
#include "sim_types.h"   // ← added; must be in the #include block
```

**Port to gateway_n2k:** Not applicable — the production sketch does not use
`SimAisTarget`.

---

## gateway_n2k_simulation — 1.0.5-sim

### Change 5 — Single fixed AIS source address (SRC_AIS_RECEIVER)

**Why:** The simulator was using `SRC_AIS_BASE + i` (one unique N2K source
address per simulated vessel). In NMEA 2000, the source address identifies
the **talker device on the bus** (the AIS receiver), not the individual vessel.
Vessels are distinguished by the MMSI bytes in the payload. Using a per-target
SRC caused the MAPTATTOO receiver to create a separate `(source, pgn)` map
entry per vessel, and also inflated the `lastTxMs` throttle map unnecessarily.

**What changed:**

Removed `SRC_AIS_BASE = 0x40`. Added:
```cpp
// Fixed N2K source address for ALL AIS PGNs — identifies the AIS receiver
// device on the bus, not the individual vessel.
static const uint8_t SRC_AIS_RECEIVER = 0x01;
```

All `notifyPGN()` calls in `simulateAisRealistic()` now pass `SRC_AIS_RECEIVER`
instead of `SRC_AIS_BASE + i`. The `src` local variable was removed.

**Port to gateway_n2k:** Not applicable. The production gateway correctly uses
the actual N2K source address decoded from the CAN frame (from `decodeId()`),
which on a real bus is the single AIS receiver device's address for all targets.
This bug was introduced by the simulator's artificial per-target SRC assignment.

---

### Change 6 — AIS inter-frame pacing (AIS_MIN_INTER_FRAME_MS)

**Why:** Two or more AIS targets whose dynamic report intervals expire on the
same `loop()` tick would fire `notifyPGN()` back-to-back with < 1 ms between
them. The MAPTATTOO receiver's 10 ms duplicate dedup filter was dropping the
second notification as a presumed BlueZ duplicate — even though the payloads
contained different MMSIs. Confirmed by log analysis showing callbacks 9 ms
apart for MMSI 367100007 and 367100008 with the second one filtered.

**What was added:**

```cpp
static const uint32_t AIS_MIN_INTER_FRAME_MS = 25u;  // > 10 ms dedup window
static uint32_t       gLastAisNotifyMs       = 0;
```

In `simulateAisRealistic()`, before updating `gAisLastDynMs[i]` or calling
`notifyPGN()`, the code checks:
```cpp
if (nowMs - gLastAisNotifyMs < AIS_MIN_INTER_FRAME_MS) {
    // skip this target this tick — will be picked up next loop()
} else {
    gAisLastDynMs[i] = nowMs;
    gLastAisNotifyMs = nowMs;
    // ... build and send ...
}
```

`gAisLastDynMs[i]` is NOT updated when skipping, so the target is retried on
the very next `loop()` iteration (typically < 1 ms later in practice, but with
`gLastAisNotifyMs` now ≥ 25 ms behind, the next target will be allowed through).

The same guard is applied to the static/voyage data block and the AtoN block.

**Port to gateway_n2k:** The production gateway receives frames from the N2K
bus at hardware-limited rates (~0.5 ms per CAN frame minimum). In practice, two
AIS messages from a single AIS receiver arrive with at least the TDMA slot gap
(~26.67 ms). No inter-frame pacing is needed in the production gateway. This
change is simulator-only.

**Memorized for later — Change 7 (not yet implemented):**
Batch multiple AIS targets into a single BLE notification using the receiver's
`parseMultipleAISTargets()` which walks the payload in 18-byte strides. One
`notifyPGN()` call per loop tick carrying N × 18 bytes, immune to BlueZ/kernel
coalescing. Requires coordination with MAPTATTOO app to confirm stride size.

---

## gateway_n2k_simulation — 1.0.6-sim

### Change 8 — Fix AIS dynamic payload field order and SOG units

**Why:** Log analysis showed AIS targets appearing at the correct MMSI but with
lat/lon swapped and wildly wrong SOG (e.g. 841 kts instead of 9 kts). Root cause
was a wrong field layout in `buildAISDynamic()` used by PGN 129038/129039/129040.

**Three bugs fixed in `buildAISDynamic()`:**

1. **LON/LAT swap** — Simulator was sending lat at bytes 5-8 and lon at bytes
   9-12. PGN 129038 standard: **lon first, lat second**.

2. **COG/SOG order** — Simulator was sending SOG at bytes 13-14 and COG at
   bytes 15-16. Standard: **COG first (bytes 13-14), SOG second (bytes 15-16)**.

3. **SOG units** — Simulator used `speedCms()` (m/s × 100). PGN 129038 SOG
   field is **0.01 knots** (centikts). New helper `speedCentikts()` added.

Correct layout after fix:
```
byte 0:       message ID (0x00)
bytes  1-4:   MMSI (u32le)
bytes  5-8:   LON × 1e7 (i32le)
bytes  9-12:  LAT × 1e7 (i32le)
bytes 13-14:  COG (u16le, 1e-4 rad)
bytes 15-16:  SOG (u16le, 0.01 knots)
bytes 17-18:  Heading (u16le, 1e-4 rad)
bytes 19-20:  ROT = 0x7FFF
bytes 21-23:  0xFF padding
```

**Port to gateway_n2k: NOT REQUIRED.**
The production gateway forwards raw N2K PGN payloads directly from the CAN bus
without constructing them. The AIS receiver on the N2K bus sends correctly
formatted PGN 129038/129039/129040 frames — the gateway just passes them through
via `notifyPGN()`. These bugs were introduced only in the simulator's synthetic
payload builders and do not exist in the production firmware.

---

## gateway_n2k — 1.0.2 (production, unchanged)

No changes have been made to the production gateway firmware yet.
The changes above (except Change 2's CAN note) are pending review and
will be ported to `gateway_n2k` in a future version.