#!/bin/bash
set -e

CONFIG="${1:-valve.yaml}"
DEVICE="${2:-labor.local}"

echo "🔧 Compiling $CONFIG..."
python3 -m esphome compile "$CONFIG"

echo ""
echo "📡 Flashing to $DEVICE via OTA..."
python3 -m esphome upload "$CONFIG" --device "$DEVICE"

echo ""
echo "✅ Done! Tailing logs..."
python3 -m esphome logs "$CONFIG" --device "$DEVICE"
