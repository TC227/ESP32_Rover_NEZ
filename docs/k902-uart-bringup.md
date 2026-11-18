# K902 UART bring-up

## Firmware defaults

- ESP32 UART: `Serial2`, 115200 baud, 8 data bits, no parity, one stop bit.
- ESP32 GPIO16 (RX) connects to K902 TX; GPIO17 (TX) connects to K902 RX. Use a common signal ground and verify the interface voltage and connector pinout on the actual board before wiring.
- The firmware names the receiver port `COM2`, but mirrors IMUDATA and, when enabled, INSRTKCOMPENSATE on COM1 because field units differ. Keep the mirrored outputs when testing an existing head.
- Rover startup requests GPGGA at 5 Hz, GPRMC and GPGST at 1 Hz, and IMUDATA at 20 Hz when tilt is enabled. The library sends receiver commands with CRLF.

## Bench checks

1. Identify which K902 port is wired and record the receiver firmware version. On an ESP32-S3 carrier, verify the schematic and board configuration before assuming the ESP32 GPIO assignments apply.
2. Capture traffic passively at 115200 8N1 on the receiver TX line. Record the port, the observed GGA cadence and fix quality, and whether binary frames are present. With an external adapter, leave its TX disconnected during passive capture.
3. If testing the receiver by itself, disconnect the ESP32 TX before attaching the adapter TX. The SinoGNSS v1.3 command manual gives `log gpgga ontime 1` as a GGA example and `log imudatab ontime 0.01` as an IMUDATA example. Use the GGA example for an initial 1 Hz check. Test 20 Hz IMUDATA with the firmware's 0.05-second setting when checking rover behavior.
4. Reconnect the ESP32 and capture startup with a role explicitly selected. Confirm GGA reaches the application. With tilt enabled, verify that the app's bubble responds to movement. Validate correction reception and RTK fix separately with a known correction source.

The 115200 setting above is the host firmware configuration. The manual's COM command uses 115200 in an example. Measure or query the receiver setting if the UART is silent. Bench results are pending.
