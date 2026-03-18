#!/bin/bash
set -e

DEVICE="${1:-labor.local}"
CONFIG="valve.yaml"

echo "🔧 Compiling $CONFIG..."
python3 -m esphome compile "$CONFIG"

echo ""
echo "📡 Flashing to $DEVICE via OTA..."
python3 -m esphome upload "$CONFIG" --device "$DEVICE"

echo ""
echo "✅ Done! Tailing logs..."
python3 -m esphome logs "$CONFIG" --device "$DEVICE"
