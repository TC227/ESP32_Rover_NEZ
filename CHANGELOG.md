## 1.2.13 - 2026-11-14

- BASE-only build (`esp32dev_base`) is now standalone: BLE stays up for a 3-minute config window after boot so the app can still send `$BCMD,START` with surveyed coords; if no start arrives and the K902 has a position fix, the head auto-starts base TX using its own GNSS position plus the saved antenna height (`baseAntH` pref, set by `$BCMD,START` or serial `!baseh <m>`, default = pole height). Auto-start also cuts BLE like a manual start. Any explicit stop (`$BCMD,STOP` by phone or rover over HC-12) holds auto-start off until reboot.

## 1.2.12 - 2026-11-14

- `$BCMD,START` now cuts the Bluetooth radio (BLE + Classic SPP) ~600 ms after the `$BSTAT` ack, so a transmitting base head drops the phone link's power draw. RTCM keeps streaming over HC-12 and `$BCMD,STOP` from the rover brings BLE back via `stopBase()`.

## 1.2.11 - 2026-11-14

- `$PRTK` carries diagnostics for the phone log: bubble tilt tx/drop + IMU frame state, base radio state (HC-12 AT/baud/tx/rx/drop, RTCM frames from K902 + age), and the last reset reason.
- `env:esp32dev_base` builds a BASE-only head (`FORCE_BASE_ROLE`): always boots as BASE, ignores `$ROLE,ROVER`, `$FW` reports `1.2.11-BASE`.
- `K902::readUart` is capped per poll so a flooding receiver cannot starve `loop()` into the task watchdog.

## 1.2.10 - 2026-11-13

- Version bump for INS-RTK compensated pole-tip position support.

## 1.2.9 - 2026-11-13

- Remove WiFi hotspot feature. App now uses Bluetooth only.

## 1.2.8 - 2026-11-13

- Enable the K902 `INSRTKCOMPENSATEB` output for tilted-pole surveying.
- Forward compensated pole-tip coordinates and pole-frame tilt/heading only after the K902 reports a valid INS solution.
- Keep IMU and INS logs armed after a receiver restart, and allow up to 85 seconds for field INS alignment.

## 1.2.6 - 2026-11-13

- Rover Wi-Fi beacon stays visible beside BLE. The hotspot is started before Classic Bluetooth, modem sleep is off, and a failed start is retried. Channel 1, name `E25aer_gnss`.

## 1.2.5 - 2026-11-13

- Rover re-sends `log COM2 imudatab` while NMEA is alive but the bubble stream is not. Boot no longer sends `imuaxestype` or `saveconfig`, which restarted the K902 and dropped that log.
- The bubble follows the accelerometer even when the first samples are not perfectly still.

## 1.2.4 - 2026-11-13

- Rover opens a Wi-Fi hotspot named E25aer_gnss so the phone can use Wi-Fi instead of BLE. Corrections stay on HC-12.
- A head saved as BASE does not start the hotspot.

## 1.2.3 - 2026-11-11

- BLE firmware update into the inactive OTA slot, and `$FW` reports 1.2.3.
- HC-12 accepts AT at 115200 or 9600, then locks 115200 channel 1.
- K902 IMU setup stays on imudatab. rawimub at 100 Hz is not logged, so the bubble keeps its frames.
- Drop insrtk commands this K902 firmware rejects.

## 1.2.0 - 2026-11-05

- One firmware for BASE and ROVER. BLE/Classic name `E25aer_gnss`. App selects role with `$ROLE`; stored in NVS.
- HC-12 433 MHz: base TX RTCM after `$BCMD,START`; rover RX or phone NTRIP via `$RCMD,SOURCE`.
- Carrier PCB rev 19: DIA 120 mm, 4× M3 on the X/Y axes, K902 38×38 mm, buzzer GPIO23, SW3 PWR jack.

## 1.1.0

- Gate bubble on static accel and fuse gyro when IMUDATA carries it.

## 1.0.0

- Initial K902 rover BLE firmware with RTCM ingest.
