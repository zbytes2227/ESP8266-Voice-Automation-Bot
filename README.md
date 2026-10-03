# ESP8266 Voice Automation Bot

An ESP8266-based voice automation controller using the Voice Recognition V3 module. Recognized voice records can trigger configurable HTTP `GET` or `POST` requests, while a 128×64 OLED provides status and a three-button interface provides local control.

## Features

- Voice Recognition V3 support for up to 80 records.
- Up to 7 trained records loaded into the recognizer at once.
- Automatic loading of enabled records, ordered by priority.
- Per-record HTTP `GET`/`POST` actions, URL, name, body, and enabled state.
- Browser-based configuration portal served by the ESP8266.
- Wi-Fi credentials stored in CRC-protected dual LittleFS slots.
- Record configuration stored in LittleFS as `/cmd/rNN.bin`.
- SSD1306 OLED boot, listening, menu, Wi-Fi, battery, and recognition screens.
- Debounced buttons with auto-repeat for navigation.
- Serial diagnostics and runtime commands at 115200 baud.
- Battery voltage estimation through the ESP8266 ADC.

## Hardware

The sketch uses the following connections:

| Function | ESP8266 pin | Notes |
| --- | --- | --- |
| OLED SDA | D2 | I²C address `0x3C`, then `0x3D` fallback |
| OLED SCL | D1 | I²C |
| VR3 RX | D5 | ESP8266 RX from VR3 TX; use a level divider where required |
| VR3 TX | D6 | ESP8266 TX to VR3 RX |
| Up button | D3 / GPIO0 | Internal pull-up; avoid holding during boot |
| Down button | D7 | Internal pull-up |
| Select button | D0 / GPIO16 | Requires an external 10 kΩ pull-up to 3.3 V |
| Battery sense | A0 | Assumes the configured voltage divider |

The VR3 serial interface uses `9600` baud. Buttons are active-low.

## Requirements

- ESP8266 board supported by the Arduino ESP8266 core.
- Arduino IDE or PlatformIO.
- Libraries:
  - ESP8266 Arduino core libraries: WiFi, WebServer, HTTPClient, LittleFS, DNS server, and WiFi client support.
  - Adafruit GFX Library.
  - Adafruit SSD1306.
  - SoftwareSerial.
  - `VoiceRecognitionV3-ESP8266` library.
- SSD1306 OLED, Voice Recognition V3 module, and the hardware listed above.

## Installation

1. Install the ESP8266 board package and the required libraries.
2. Open `ESP8266_Bot_Voice_v1_CLAUDE.ino` in Arduino IDE. Keep the sketch and folder name aligned with the `.ino` filename.
3. Select the correct ESP8266 board and serial port.
4. Compile and upload the sketch.
5. Open the serial monitor at `115200` baud.

On first boot, or when no saved Wi-Fi connection is available, the device starts an access point named:

```text
BOT-SETUP-<chip-id>
```

The default setup password is `botsetup123`. Join that network and open the displayed AP address in a browser. The Select button skips Wi-Fi setup and continues in offline mode.

## Using the device

After boot, the bot enters the listening loop. Train and manage records through the OLED menu:

1. Press **Select** to open the main menu.
2. Open **Voice Recognition → Train Record** and train a VR3 record.
3. Open the web portal using the displayed ESP8266 IP address.
4. Configure the record name, HTTP method, URL, POST body, priority, and enabled state.
5. Save the record. Enabled and trained records are automatically loaded on the next boot, up to the hardware limit of 7 records.
6. Return to listening mode and speak the trained command.

When a record matches, the bot displays the recognition state, executes the configured request, and shows the HTTP result on the OLED. HTTP actions require an active Wi-Fi connection.

## Web portal

When Wi-Fi is connected, the portal is available at:

```text
http://<device-ip>/
```

Available endpoints:

| Method | Endpoint | Purpose |
| --- | --- | --- |
| `GET` | `/` | Configuration web interface |
| `GET` | `/scan` | Scan for nearby Wi-Fi networks |
| `POST` | `/save` | Save Wi-Fi credentials |
| `GET` | `/api/config` | Return all 80 record configurations |
| `POST` | `/api/record` | Save one record configuration |
| `POST` | `/api/test?r=N` | Test record `N`'s HTTP action |
| `GET` | `/api/status` | Return Wi-Fi, VR, memory, uptime, and record status |
| `POST` | `/api/forget_wifi` | Clear saved Wi-Fi credentials |

Record numbers range from `0` through `79`. The VR3 module can have at most 7 records active at a time.

## Serial commands

At `115200` baud, send one of the following commands:

```text
h  help
s  state
p  ping VR
m  recognizer map
d  deep diagnostics
c  dump active configuration
f  reload configuration from flash
r  reboot
```

Serial output includes boot diagnostics, VR packet information, recognition events, HTTP results, Wi-Fi state, and heap usage.

## Persistent storage

LittleFS is initialized during boot. If mounting fails, the sketch formats the filesystem and retries. The following data is stored in flash:

- `/wifi_a.bin` and `/wifi_b.bin`: CRC-protected, dual-slot Wi-Fi configuration.
- `/cmd/r00.bin` through `/cmd/r79.bin`: CRC-protected per-record action configuration.

Formatting the filesystem removes saved Wi-Fi credentials and record action settings.

## Troubleshooting

- **OLED initialization fails:** verify SDA/SCL wiring, power, ground, and the `0x3C`/`0x3D` address.
- **VR module does not respond:** check D5/D6, common ground, module power, serial wiring, and voltage levels. Use the OLED **Module Test** and serial command `p`.
- **Select button is always pressed:** add the required external 10 kΩ pull-up from D0/GPIO16 to 3.3 V.
- **No HTTP action runs:** confirm the device is connected to Wi-Fi, the record is trained and enabled, and its URL is valid.
- **A command is not loaded:** only trained, enabled records are auto-loaded, and only the highest-priority 7 records fit in the recognizer.

## License

No license file is currently included. Add a license before distributing the project publicly.
