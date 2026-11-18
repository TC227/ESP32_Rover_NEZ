## 1.2.50 - 2026-11-18

- K902 "Invalid key word" blacklist: when the board rejects a command it echoes the text back; the firmware now remembers rejected commands (persisted in NVS `k902rej`) and `sendCommand` silently drops them, so setup retries stop spamming errors. Field K902s reject `glgsa/gagsa/gbgsa`, `glgsv/gagsv/gbgsv`, `insrtkcompensateb`, `insconfigb`, and `insrtk rodlength` — those logs simply never arm on that board revision instead of retrying forever.
- New `$K902CLR` command clears the rejected-command list (e.g. after swapping the K902 board or flashing new receiver firmware).

## 1.2.28 - 2026-11-15

- BASE keeps IMU logs again (`imudatab` on both COM1 and COM2) so the bubble level works on a base head too; `ensureBubbleLog` no longer skips the BASE role.
- Verified in the field: K902 on this hardware routes binary logs (IMU + RTCM) out COM1, not COM2 — RTCM logs are mirrored on both ports so either wiring works.
- BASE `$FW`/BLE advertisement label was hardcoded `1.2.25-BASE`; now derived from `FW_VERSION` so the app shows the real version.

## 1.2.27 - 2026-11-15

- BASE RTCM log set trimmed to observations only; briefly removed IMU logs (restored in 1.2.28 for the bubble).
- `$K902` passthrough and the `K902 rx=` UART byte counter retained for field diagnosis.

## 1.2.26 - 2026-11-15

- BASE mirrors RTCM + IMU logs on COM1 in addition to COM2 — the field K902 delivers binary output on COM1, which is why base RTCM stayed silent despite accepted `log COM2` commands.
- Status line gains `K902 rx=` (raw UART byte counter) to prove data is actually arriving.

## 1.2.25 - 2026-11-15

- New `$K902,<command>` BLE/serial passthrough forwards a raw command to the receiver (replies `$K902,OK`/`$K902,BUSY`) — field debugging of K902 config without a wired console.

## 1.2.24 - 2026-11-15

- Read-only phone polls (`$FW`, `$VER`, `$BCMD,STATUS`) no longer refresh the auto-start idle timer — an app sitting on the BASE screen polling status was holding auto-start forever. Only state-changing commands count as activity now.

## 1.2.23 - 2026-11-15

- BASE-only build no longer enables K902 INS/DR (`inscontrol disable` is sent instead, and the INS tuning/log commands are skipped): the base only needs a GNSS fix plus RTCM observations. INS mode reported GGA quality 6 and is suspected of suppressing base RTCM output. Rover/unified builds keep INS-RTK unchanged.

## 1.2.22 - 2026-11-15

- Base TX now holds the first HC-12 RTCM burst for 2.5 s after `startBaseArp`: BLE teardown (auto-start path) and the K902 log-start inrush land inside the window, so the supply sees one step instead of BLE+radio+receiver spiking together. Early RTCM frames repeat, so nothing is lost.

## 1.2.21 - 2026-11-15

- BASE auto-start no longer waits forever while a phone is merely connected: it starts once the config window has passed and the app has been silent for 10 minutes. New `$BAVG,1`/`$BAVG,0` command lets the app hold auto-start for the exact duration of a base-point averaging session (auto-expires after 75 min if the app dies).

## 1.2.15 - 2026-11-14

- BASE-only auto-start no longer waits for a navigation fix when a surveyed point was saved via `$BCMD,START` — the K902 receives `fix position` and tracks satellites on its own, so base TX can begin indoors/outdoors before a 3D fix. The live-fix fallback path still requires `hasPosition()`.

## 1.2.14 - 2026-11-14

- BASE-only build rework: config window is now 40 minutes so the app can average a point for up to ~30 min before `$BCMD,START`; auto-start never fires while a phone is connected. Only an explicit `$BCMD,START` persists the surveyed fix (`bLat`/`bLon`/`bArp`) — auto-start reuses it after reboot, otherwise it uses the head's live GNSS position each boot (serial `!baseclr` clears the saved point, `!baseh <m>` sets antenna height).

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
