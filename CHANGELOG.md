## 1.1.0 - 2026-11-05

- One firmware for BASE and ROVER. BLE/Classic name `E25aer_gnss`. App selects role with `$ROLE`; stored in NVS.
- HC-12 433 MHz: base TX RTCM after `$BCMD,START`; rover RX or phone NTRIP via `$RCMD,SOURCE`.
- Carrier PCB rev 19: DIA 120 mm, 4× M3 on the X/Y axes, K902 38×38 mm, buzzer GPIO23, SW3 PWR jack.

## 1.0.0

- Initial K902 rover BLE firmware with RTCM ingest.
