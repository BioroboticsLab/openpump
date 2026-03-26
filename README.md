# 🌱 OpenPump — Single-Pump Quad-Valve

An open-source, sensor-driven plant watering system built on [ESPHome](https://esphome.io/) and the ESP32. A single pump serves up to **four independent zones** via electronically controlled valves, each with its own soil-moisture sensor, adjustable dryness threshold, and cooldown timer — plus real-time **Telegram notifications**.

> **Branch:** `single_pump_quad_valve` — ESPHome YAML configuration.
> The `main` branch contains the original bare-metal ESP-IDF C firmware for a single-sensor setup.

## Features

- **4-zone watering** — one pump, four valves, four capacitive soil-moisture sensors
- **Per-zone thresholds** — adjustable dryness sliders (0–100) via Home Assistant or the built-in web UI
- **Cooldown protection** — 1-hour cooldown per zone prevents overwatering
- **Pump mutex** — only one zone can pump at a time, preventing pressure issues
- **Telegram alerts** — instant notifications when pumping starts/stops
- **Telegram bot commands** — send `status` for a full report, or `value pod 1` to query a single sensor
- **Heartbeat** — periodic uptime report via Telegram (every 2 hours)
- **Status LED** — blinks during boot/Wi-Fi reconnect, solid when connected
- **Wi-Fi events** — Telegram notification on connect; LED feedback on disconnect
- **OTA updates** — flash new firmware over Wi-Fi
- **Fallback AP** — device creates a hotspot if Wi-Fi is unavailable
- **Web dashboard** — built-in web server on port 80

## Hardware

### 3D-Printed Parts

The enclosure consists of **4 main printed parts** and **1 auxiliary part** printed once per plant zone (up to 4×):

| Part | Description |
|------|-------------|
| **Box** | Main enclosure: floor and two short side walls. One side wall has a socket for the buck converter with the DC barrel jack hole directly beneath it. The other side wall has a socket for the breadboard. |
| **Wall 1 (Tube Wall)** | Modular wall with half-circle cable management bridges along the top edge and **5 tube holes** at the bottom. The 3rd (middle) hole is the water intake for the pump; the remaining 4 holes are the valve outlets. |
| **Wall 2 (Relay Wall)** | Modular wall with half-circle cable management bridges along the top edge and sliding sockets for the **4-channel relay module** and the **1-channel relay module**. |
| **Lid** | Slides over the finished assembly to close the enclosure. |
| **Sensor Clamp** *(× 1 per zone, max 4)* | Inserted into a pot or reservoir. Contains a tight slot for the capacitive soil-moisture sensor and a hole to secure the outlet tube in place. |

---

### Electronic Components

| Component | Qty | Link |
|-----------|-----|------|
| ESP32 DevKit | 1 | [AZ-Delivery](https://www.az-delivery.de/products/esp32-nodemcu-module-wlan-wifi-dev-kit-c-development-board-mit-cp2102-und-usb-c-anschluss-esp-32-esp32-wroom-32-kompatibel-mit-arduino) |
| Breadboard (~400 pin, half-size) | 1 | [Conrad](https://www.conrad.de/de/p/velleman-steckplatine-grau-polzahl-gesamt-400-l-x-b-x-h-82-x-8-x-55-mm-1-st-2574925.html) |
| 4-channel relay module | 1 | [Conrad](https://www.conrad.de/de/p/tru-components-tc-9927216-relais-modul-1-st-2481804.html) |
| 1-channel relay module | 1 | [Conrad](https://www.conrad.de/de/p/whadda-wpm406-relais-modul-1-st-2330791.html) |
| Buck converter (12 V → 5 V) | 1 | [AZ-Delivery](https://www.az-delivery.de/products/xl4016e-yh11060d) |
| Capacitive soil-moisture sensor | up to 4 | [AZ-Delivery](https://www.az-delivery.de/products/bodenfeuchte-sensor-modul-v1-2) |
| Peristaltic pump (12 V) | 1 | [Amazon](https://www.amazon.de/Peristaltische-Mini-Peristaltikpumpe-Bioengineering-Silikonschlauch-Aquarienlaboranalytik/dp/B0BTQMRSQ8) |
| Solenoid valve (12 V, NC) | up to 4 | [Amazon](https://www.amazon.de/WITTKOWARE-Mini-Magnetventil-12V-Anschl%C3%BCsse-Positionen/dp/B07JB53SMP) |
| DC barrel jack (2.5 mm) | 1 | [Conrad](https://www.conrad.de/de/p/econ-connect-dce5bp-niedervolt-steckverbinder-chassisbuchse-gerade-2-5-mm-1-st-1303454.html) |
| 12 V / 2 A power supply | 1 | [Conrad](https://www.conrad.de/de/p/dehner-elektronik-sys-1308n-2412-w2e-steckernetzteil-festspannung-12-v-dc-2-a-24-w-stabilisiert-2330693.html) |
| Silicone tube (ID 3.5 mm / OD 4.5 mm) | as needed | any aquarium supplier |
| Spring hose clamps (4 mm) | 8 (2 per valve) | [Amazon](https://amzn.eu/d/0d3o6NUx) |
| T-profile tube connectors (3 mm) | 3 | [Amazon](https://www.amazon.de/BGS-8790-10-Schlauchverbinder-Sortiment-10-mm/dp/B015EKYP7Y) |

---

### Pin Mapping

| GPIO | Function |
|------|----------|
| GPIO32 | Soil-moisture sensor 1 (ADC) |
| GPIO33 | Soil-moisture sensor 2 (ADC) |
| GPIO39 | Soil-moisture sensor 3 (ADC) |
| GPIO35 | Soil-moisture sensor 4 (ADC) |
| GPIO17 | Pump — 1-channel relay (active-HIGH) |
| GPIO22 | Valve 1 — 4-channel relay ch.1 (active-HIGH) |
| GPIO21 | Valve 2 — 4-channel relay ch.2 (active-HIGH) |
| GPIO19 | Valve 3 — 4-channel relay ch.3 (active-HIGH) |
| GPIO18 | Valve 4 — 4-channel relay ch.4 (active-HIGH) |
| GPIO2  | Status LED (active-LOW) |
| 3.3 V pin | Shared power rail for all 4 moisture sensors |

---

### Assembly Instructions

**Before you start:** Print all 4 main parts plus one Sensor Clamp per zone. Have a soldering iron, glue, and the step file open to measure the correct breadboard width for the socket.

**1. Soldering**
1. Solder 2 wires onto the DC barrel jack.
2. Solder 2 wires onto the pump.

**2. Prepare the DC barrel jack**
1. Insert the DC barrel jack into the hole beneath the buck converter socket on the side wall of the box.

**3. Prepare the breadboard**
1. Split the breadboard at the center. Before gluing, measure the correct width using the step file so the breadboard fits snugly into its socket.
2. Glue both halves onto a piece of thin cardboard, leaving a small gap so cables can be routed on both sides of the ESP32.
3. Glue the cardboard-mounted breadboard into its socket on the other side wall of the box.
4. Place the ESP32 onto the breadboard.

**4. Install the buck converter and relay modules**
1. Plug the buck converter into its socket on the box wall.
2. Slide the **4-channel relay module** and **1-channel relay module** down into their sockets in Wall 2 (Relay Wall).

**5. Install Wall 1 (Tube Wall)**
1. Fit Wall 1 before gluing the valves to the floor — this makes positioning much easier.
2. Loosely place the solenoid valves and peristaltic pump inside the box.
3. Route the pump intake tube through the **3rd (middle) hole** of Wall 1. Route the 4 valve outlet tubes through the remaining holes. Secure each valve tube with **2 spring hose clamps**.
4. Use **3 T-profile connectors (3 mm)** to split the single pump output tube into 4 lines, one for each valve inlet.

**6. Wiring — Power circuit**

Start by establishing your two power rails on the breadboard (12 V and 5 V):

1. Connect the two wires from the DC barrel jack to the **12 V + and − rows** on one side of the breadboard.
2. Run two cables from the 12 V rows to the **buck converter input** (+ and −).
3. Run the buck converter output cables to the **opposite side** of the breadboard, establishing the **5 V + and − rows**.

You now have a 12 V rail and a 5 V rail available on the breadboard.

**7. Wiring — Valves (4-channel relay module)**

Repeat for each of the 4 valves:

1. Connect the valve's **red (+) cable** directly to the **12 V + row** on the breadboard.
2. Connect the valve's **black (−) cable** to the **middle terminal (COM)** of the corresponding relay channel.
3. Connect a cable from the **right terminal (NO)** of that relay channel to the **12 V − row** on the breadboard.

Then wire the relay module's control circuit:

| Pin | Connection |
|-----|------------|
| VCC | 5 V + row on breadboard |
| GND | 5 V − row on breadboard |
| IN1 | GPIO22 (Valve 1) |
| IN2 | GPIO21 (Valve 2) |
| IN3 | GPIO19 (Valve 3) |
| IN4 | GPIO18 (Valve 4) |

**8. Wiring — Pump (1-channel relay module)**

1. Connect the pump's **red (+) cable** directly to the **12 V + row** on the breadboard.
2. Connect the pump's **black (−) cable** to the **middle terminal (COM)** of the single relay.
3. Connect a cable from the **right terminal (NO)** of the relay to the **12 V − row** on the breadboard.

Then wire the relay module's control circuit:

| Pin | Connection |
|-----|------------|
| VCC | 5 V + row on breadboard |
| GND | 5 V − row on breadboard |
| IN  | GPIO17 (Pump) |

**9. Wiring — Moisture sensors**

The moisture sensors run on **3.3 V**, not 5 V or 12 V:

1. Run a cable from the ESP32's **3.3 V pin** to a free horizontal row on the breadboard — this becomes your **3.3 V shared row**.
2. For each sensor, plug its **VCC cable** into that same 3.3 V row.
3. Connect each sensor's **GND cable** to the **5 V − row** (common ground).
4. Connect each sensor's **signal (AOUT) cable** to its corresponding GPIO:

| Sensor | GPIO |
|--------|------|
| Sensor 1 | GPIO32 |
| Sensor 2 | GPIO33 |
| Sensor 3 | GPIO39 |
| Sensor 4 | GPIO35 |

**10. Final fixation**
1. Once all wiring is complete and verified, glue or screw the pump into position (screw holes are provided).
2. Glue the valves to there sockets.
3. Slide in the second wall (can be kind of tricky).
4. Slide the **Lid** onto the finished assembly.

**11. Install Sensor Clamps**
1. For each plant zone, insert a Sensor Clamp into the pot or reservoir.
2. Slide the moisture sensor into the tight slot.
3. Route the outlet tube through the hole in the clamp to hold it in place.

> ⚠️ **Relay polarity:** Relays are **active-HIGH** — do **not** set `inverted: true` in the ESPHome config. If the pump or a valve runs when it should be OFF, check that you are using COM/NO and not COM/NC on the relay terminals.

## Getting Started

### Prerequisites

- [ESPHome](https://esphome.io/guides/installing_esphome.html) installed (`pip install esphome`)
- A `secrets.yaml` file in the same directory as `valve.yaml`

### Secrets Setup

Create a `secrets.yaml` file alongside `valve.yaml`:
```yaml
ssid: "Your_WiFi_SSID"
password_wifi: "Your_WiFi_Password"
password_ap: "FallbackHotspotPassword"
telegram_bot_token: "123456:ABC-DEF..."
telegram_chat_id: "-100123456789"
api_key: "your-base64-encryption-key"      # generate with: openssl rand -base64 32
ota_password: "a-secure-ota-password"
```

> **Tip:** Create a Telegram bot via [@BotFather](https://t.me/BotFather) and get your chat ID via [@userinfobot](https://t.me/userinfobot).

### Flash the ESP32
```bash
# Validate the configuration
esphome config valve.yaml

# Compile and upload (first time — via USB)
esphome run valve.yaml

# Subsequent updates can be done OTA
esphome run valve.yaml --device labor.local

# Or use the helper script
./flash.sh                              # default: valve.yaml → labor.local
./flash.sh tests/test_actuators.yaml     # flash a test config
./flash.sh valve.yaml 10.94.201.232      # flash to a specific IP
```

### Home Assistant

The device advertises itself via the ESPHome native API. In Home Assistant, go to **Settings → Devices & Services** — the device `labor` should appear for automatic adoption.

## How It Works

1. **Every 30 seconds**, the system checks each zone sequentially.
2. If a zone's moisture reading is **below its threshold** and the **cooldown has expired** and the **pump is not busy**, watering begins:
   - All other valves are closed.
   - The zone's valve opens.
   - The pump runs for the configured **pump duration** (default 30 s, adjustable 5–120 s via slider).
   - A Telegram message is sent at start and stop.
3. A **1-hour cooldown** prevents the same zone from being watered again too soon.
4. The `pump_busy` flag ensures only one zone can pump at a time.
5. A **heartbeat** message is sent via Telegram every 2 hours with the device's uptime.
6. The **Telegram bot** polls for incoming commands every 5 seconds — send `status` for a full report (all sensors, thresholds, cooldowns, pump state, uptime), or `value pod 1` through `value pod 4` for a single reading.
7. The **status LED** blinks during boot and Wi-Fi reconnect, and stays solid once connected.
8. If a zone stays **dry for 24 hours**, a persistent dry alert is sent via Telegram.

## Sensor Calibration

The sensors output a raw ADC voltage which is scaled (×100) and then linearly mapped to 0–100 % using:
```
y = -0.53125 × x + 111.25
```

This maps approximately 21 → 100 % (wet) and 209 → 0 % (dry). To tune for your sensors, adjust the formula coefficients in the `lambda` filter of each sensor in `valve.yaml`.

## Project Structure
```
openpump/
├── valve.yaml                    # ESPHome configuration (main file)
├── secrets.yaml                  # Wi-Fi & Telegram credentials (not tracked)
├── flash.sh                      # Build + OTA flash helper script
├── tests/
│   ├── secrets.yaml              # → symlink to ../secrets.yaml
│   ├── test_actuators.yaml       # Minimal pump/valve toggle test
│   ├── test_sensors.yaml         # Sensor + actuator test
│   └── test_gpio_scan.yaml       # All ADC1 GPIO scan
├── LICENSE                       # MIT
└── README.md                     # This file
```

## License

Licensed under the [MIT License](LICENSE).
