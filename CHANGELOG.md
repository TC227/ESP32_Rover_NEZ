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
