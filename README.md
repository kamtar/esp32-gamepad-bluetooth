# esp32-gamepad-bluetooth

ESP32-S3-Zero USB-host to BLE gamepad bridge built with ESP-IDF.

## What it does

- Starts the ESP32-S3 USB host stack and HID host driver
- Reads input reports from a connected USB HID gamepad
- Re-publishes the state as a BLE HID gamepad report
- Advertises as `ESP32-S3 USB Gamepad Bridge`

## Hardware notes

- Board: ESP32-S3-Zero
- USB OTG wiring must support host mode (5V VBUS power + D+/D-)
- Only USB HID gamepads are expected
- Xbox USB gamepads that send 20-byte XInput-style HID reports are mapped explicitly

## Build

```bash
idf.py set-target esp32s3
idf.py build
```

## Flash and monitor

```bash
idf.py -p <PORT> flash monitor
```
