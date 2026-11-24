#!/usr/bin/env bash
# Buduje wszystkie aplikacje z apps/* i kopiuje firmware do dist/<nazwa>.bin
# Opcjonalnie: ./scripts/build_all.sh /run/media/$USER/SDCARD/apps  -> kopiuje tez na karte SD
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEST_SD="${1:-}"
mkdir -p "$ROOT/dist"
for app in "$ROOT"/apps/*/; do
  name="$(basename "$app")"
  [[ -f "$app/platformio.ini" ]] || continue
  echo "==> $name"
  pio run -d "$app"
  bin="$(ls "$app"/.pio/build/*/firmware.bin | head -n1)"
  cp "$bin" "$ROOT/dist/$name.bin"
  if [[ -n "$DEST_SD" ]]; then
    mkdir -p "$DEST_SD"
    cp "$bin" "$DEST_SD/$name.bin"
  fi
done
[[ -n "$DEST_SD" ]] && sync
echo "Gotowe: $(ls "$ROOT"/dist)"
