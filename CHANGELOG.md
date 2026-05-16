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

## gateway_n2k — 1.0.2 (production, unchanged)

No changes have been made to the production gateway firmware yet.
The changes above (except Change 2's CAN note) are pending review and
will be ported to `gateway_n2k` in a future version.