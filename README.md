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
- **Telegram bot commands** — send `value pot 1` to query live moisture readings
- **Heartbeat** — periodic uptime report via Telegram (every 2 hours)
- **Status LED** — blinks during boot/Wi-Fi reconnect, solid when connected
- **Wi-Fi events** — Telegram notification on connect; LED feedback on disconnect
- **OTA updates** — flash new firmware over Wi-Fi
- **Fallback AP** — device creates a hotspot if Wi-Fi is unavailable
- **Web dashboard** — built-in web server on port 80

## Hardware

### Components

| Component | Qty | Notes |
|-----------|-----|-------|
| ESP32 DevKit | 1 | `esp32dev` board |
| Capacitive soil-moisture sensor | 4 | Analog output |
| 12 V water pump | 1 | Controlled via relay/MOSFET on GPIO5 |
| Solenoid valve (normally closed) | 4 | Controlled via relay/MOSFET |
| Relay module or MOSFET driver | 5 | 1 pump + 4 valves |
| 12 V power supply | 1 | Sized for pump + valves |

### Pin Mapping

| GPIO | Function |
|------|----------|
| GPIO32 | Soil-moisture sensor 1 (ADC) |
| GPIO33 | Soil-moisture sensor 2 (ADC) |
| GPIO34 | Soil-moisture sensor 3 (ADC) |
| GPIO35 | Soil-moisture sensor 4 (ADC) |
| GPIO5 | Pump |
| GPIO18 | Valve 1 |
| GPIO19 | Valve 2 |
| GPIO21 | Valve 3 |
| GPIO22 | Valve 4 |

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
6. The **Telegram bot** polls for incoming commands every 20 seconds — send `value pot 1` through `value pot 4` to get a live moisture reading.
7. The **status LED** blinks during boot and Wi-Fi reconnect, and stays solid once connected.

## Sensor Calibration

The sensors output a raw ADC value which is linearly mapped to 0–100 %. The default calibration assumes:
- **0 → 100 %** (sensor in water)
- **330 → 0 %** (sensor in dry air)

To tune for your sensors, update `cal_wet` and `cal_dry` in the `substitutions` section of `valve.yaml`.

## Project Structure

```
openpump/
├── valve.yaml          # ESPHome configuration (main file)
├── secrets.yaml        # Wi-Fi & Telegram credentials (not tracked)
├── LICENSE             # Apache 2.0
└── README.md           # This file
```

## License

Licensed under the [Apache License 2.0](LICENSE).
