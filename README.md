# Helmet Impact Monitor

A sensor that mounts on the back of a football helmet. It measures the linear acceleration and rotational velocity of every hit and sends a Bluetooth alert when either one passes a threshold from concussion research: **80 g** or **1000 °/s**.

It flags hits that are worth a concussion check. It does not diagnose concussions.

- **Website:** https://oolavuar1.github.io/HelmetImpactMonitor/ (photos, build guide, design decisions, test results and the full design log)
- **Console:** https://oolavuar1.github.io/HelmetImpactMonitor/console.html
- **CAD (Onshape):** https://cad.onshape.com/documents/ed99d0b06df9e477665971e0/w/3d3a22881dab07a6218cb837/e/45c9494ef317be60c1e28623

## What's in this repo

| Folder / file | What it is |
|---|---|
| `HelmetImpactMonitor/` | Arduino firmware for the ESP32 |
| `cad/` | STEP files for the enclosure bottom and top, screw, stack shield, curved base and locking pin |
| `console.html` | Web console: connects over Bluetooth, shows live readings and alerts, and runs the tests |
| `data/` | Raw test data (accuracy drops, alert drops, 91 cm re-test, rotation check) |
| `index.html` and the rest | The project website, served by GitHub Pages |

## Hardware

- Adafruit HUZZAH32 (ESP32 Feather)
- Adafruit ADXL375 accelerometer, ±200 g
- Adafruit ISM330DHCX gyroscope
- 3.7 V 1200 mAh LiPo with JST plug
- 2 red LEDs with built-in resistors
- 22 AWG stranded wire, heat shrink
- 3M VHB tape
- Printed parts in white PETG (0.4 mm nozzle, 4 to 6 walls, PLA support interface)

### Wiring

| Signal | HUZZAH32 pin |
|---|---|
| SDA (both sensors) | SDA |
| SCL (both sensors) | SCL |
| Power (both sensors) | 3V |
| Ground (sensors and LEDs) | GND |
| LEDs (both) | 21 |

The battery plugs into the board's JST port and charges over micro-USB.

## Firmware

1. Install Arduino IDE, the ESP32 board package and the CP210x USB driver.
2. Install the libraries: Adafruit ADXL375, Adafruit LSM6DS, Adafruit Unified Sensor and NimBLE-Arduino.
3. Open `HelmetImpactMonitor/HelmetImpactMonitor.ino`, select the Adafruit HUZZAH32 board and upload.
4. Keep the device still when it powers on. It calibrates both sensors in the first 2 seconds.

`TEST_MODE` lowers the thresholds to 30 m/s² and 500 °/s for bench testing. It must match in the sketch and in `console.html`.

## Console

Open the console in Chrome or Edge on a computer, or Chrome on Android, press **Connect device**, then **start**. iPhone browsers don't support Web Bluetooth. Web Bluetooth needs an https page, so use the GitHub Pages link above on a phone.

## Test results (October 2026)

- 19 accuracy drops from 61 to 152 cm onto grass: impulse ratio 1.23 ± 0.11, all within the 0.85 to 2.15 pass band.
- 25 alert drops: 12 of 12 above 80 g alerted, 0 of 13 below.
- 20 rotation turns at 90° and 180°: gyroscope scale factor 0.989.
- Loop rate raised from 270 to 705 readings per second by running I²C at 400 kHz.

Full results and graphs are on the website's Testing page.

## Status

Still to do: a sharp-twist test of the rotation alert at 1000 °/s, deep sleep while charging, a stronger reprint of the enclosure, and drop tests on a harder surface.

Built by Oliver Comfort, June to October 2026.
