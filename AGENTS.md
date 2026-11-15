# E25aer GNSS — ESP32 Firmware

Firmware for the E25aer GNSS surveying head (ESP32 + ComNav K902 receiver + HC-12 433 MHz radio). One unified binary serves both roles; the phone app picks ROVER or BASE over BLE (`$ROLE,BASE|ROVER`), persisted in NVS.

This repo is **private**. Public `.bin` artifacts live in `preechA3525/e25aer-gnss-firmware` (releases only, no source).

## Roles

- **ROVER**: GNSS + INS/tilted-pole (`insrtkcompensate`), bubble level from K902 IMUDATA, receives RTCM corrections from BASE via HC-12.
- **BASE**: `inscontrol disable` + smoothing/DR off (clean fix for RTCM), generates RTCM 1074/1084/1094/1124 + 1005/1033, transmits over HC-12 at 115200. Auto-starts ~40 min after boot if the app stays silent (or immediately via `$BCMD,START`).

## Hardware map

- K902 UART: `K902_RX_PIN 16`, `K902_TX_PIN 17`, `115200` baud.
- HC-12 radio: GPIO 25/26/27 area, 115200 transparent mode.
- USB serial: console + flashing (CH9102; no auto-reset — hold BOOT, tap RESET to enter download mode).

## K902 UART ports — important

Field units disagree on which K902 port emits binary logs. **Always mirror binary logs on both `COM1` and `COM2`** (`K902_PORT` is `COM2`). NMEA/commands arrive on the wired port regardless; IMU (`imudatab`) and RTCM (`rtcm*b`) must be logged on both to be safe.

## BLE (Nordic UART)

- Service `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
- RX `...0002` (phone→head commands), TX `...0003` (NMEA/status), tilt `...0004` (binary `[0x1F][pitch_cdeg i16][roll_cdeg i16]` @ ~40 Hz), OTA `...0005`.
- Classic Bluetooth SPP kept for Survey Master compatibility.

## Phone command protocol (ASCII lines over BLE RX)

Incoming bytes are gated by `commandPrefixOk()` — **new `$XXX` commands MUST be added to `kPrefixes` or they are silently dropped** (this bug shipped in 1.2.29).

- `$ROLE,BASE|ROVER` — switch/persist role, retunes K902 INS settings.
- `$BCMD,START,<U north>,<U east>,<elev>,<antH>,<zone>,<N/S>` / `$BCMD,STOP` / `$BCMD,STATUS`
- `$BAVG,1|0` — hold/release base auto-start during app averaging.
- `$ICMD,CALIB[,poleH]`, `$ICMD,HEIGHT,<h>`
- `$TILT,LEVEL|GET|SPD|STA|ALPHA|BLEND|GYRO|MNT|RAW,<v>` — per-head bubble tuning, persisted in NVS (`tSpd`,`tSta`,`tAlp`,`tBld`,`tGyro`,`tMnt`). Replies `$TILT,ACK`/`$TILT,CFG`; `$TILTRAW,...` debug stream.
- `$K902,<raw>` — passthrough to K902. `$LCMD,START|STOP` raw RTCM logging. `$FW`, `$VER`, `$PRTK`, `$BASELINK`, `$STAT`, `$BSTAT` status.

Anything on RX that fails `commandPrefixOk` is forwarded to HC-12 as RTCM.

## Bubble pipeline

K902 `imudatab` binary → `K902::onImu` → gravity low-pass vs `levelRef` (NVS `levelRef`, set by `$TILT,LEVEL` or `$ICMD,CALIB`) → accel pitch/roll → optional gyro complementary fusion (`tiltGyroOn`, `tiltGyroAlpha`) → static/dynamic follow gains (`tiltFollowStatic`/`tiltFollowDynamic`) → quarter-turn mount remap (`tiltMount`) → `shownPitch/shownRoll` → `$TILT,p,r` ASCII fallback + binary notify on char `...0004`.

`ensureBubbleLog()` re-arms IMU logs if NMEA flows but no IMU arrives for >2 s (boot setup can be lost mid-queue).

## Build

```bash
pio run -e esp32dev          # unified firmware (role set by app)
pio run -e esp32dev_base     # BASE-locked variant (FORCE_BASE_ROLE=1; legacy/local use)
```

After every build: read `FW_VERSION` in `src/main.cpp`, copy `.pio/build/<env>/firmware.bin` to repo root as `e25aer_gnss_<FW_VERSION>.bin` (and `e25aer_base_<FW_VERSION>.bin` for the base env).

## Release workflow

1. Bump `FW_VERSION`, build both envs, copy `.bin` files to root.
2. Commit + push to this private repo; tag `v<FW_VERSION>` on the bump commit.
3. Publish to the public repo — **unified binary only** (no `-base` releases since 1.2.29):

```bash
gh release create v<ver> --repo preechA3525/e25aer-gnss-firmware e25aer_gnss_<ver>.bin --latest
```

4. Flash: USB (hold BOOT → RESET → release RESET, keep BOOT) or OTA via the app's Firmware Update screen.

## Gotchas

- `FORCE_BASE_ROLE` builds ignore `$ROLE,ROVER` and report `FW_LABEL = "<ver>-BASE"`; they set role at boot **without writing NVS** — heads coming from a `-BASE` build have no stored role, so after OTA to unified they boot as ROVER until the app sets the role once.
- `maybeAutoStartBase()` only runs when `isBaseRole()`; read-only polls (`$FW`,`$VER`,`$BCMD,STATUS`) don't count as phone activity.
- `unlogall` + `fix none` must both land on base stop or rover coordinates freeze.
- HC-12 `tx=` counter on the status line proves RTCM is actually leaving the radio.
