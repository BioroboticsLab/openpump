#Quad_Plant_Watering

Built on an ESP32 and configured with ESPHome in YAML, this setup uses four soil-moisture sensors and four independently controlled pumps, enabling a true quad-pump design where each plant is monitored and watered separately based on its own dryness threshold.

When a plant is detected as too dry, the corresponding pump is activated for a fixed watering cycle. A built-in cooldown period prevents repeated activation and protects plants from overwatering. Each plant operates independently, allowing precise moisture control tailored to different plant requirements.

This branch integrates a Telegram bot–based notification system that sends real-time messages when pumping starts and stops, alerts if a plant remains dry for more than 24 hours, and periodic heartbeat messages reporting device uptime. A status LED provides visual feedback for Wi-Fi connectivity, and fallback access point allow local monitoring and configuration if the primary Wi-Fi connection is unavailable.
