#!/usr/bin/env bash
# Build the firmware and write release files.
#   tools/package.sh [project_dir] [out_dir]
# Writes aok-blind-bridge-<version>-factory.bin (flash at 0x0 over USB) and
# aok-blind-bridge-<version>-ota.bin (upload from the web page), plus the same pair
# for the WT32-ETH01 (-eth-) and ESP32-S3 (-s3-) builds.
set -euo pipefail
proj="$(cd "${1:-.}" && pwd)"
out="$(mkdir -p "${2:-dist}" && cd "${2:-dist}" && pwd)"
ver="$(sed -n 's/^#define FW_VERSION "\(.*\)"/\1/p' "$proj/src/main.cpp")"
(cd "$proj" && pio run -e hw395)
b="$proj/.pio/build/hw395"
boot_app0="$(ls ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin)"
python3 -m esptool --chip esp32 merge_bin -o "$out/aok-blind-bridge-$ver-factory.bin" \
  --flash_mode dio --flash_size 4MB \
  0x1000 "$b/bootloader.bin" 0x8000 "$b/partitions.bin" 0xe000 "$boot_app0" 0x10000 "$b/firmware.bin"
cp "$b/firmware.bin" "$out/aok-blind-bridge-$ver-ota.bin"
# WT32-ETH01 (Ethernet) build
(cd "$proj" && pio run -e wt32eth01)
e="$proj/.pio/build/wt32eth01"
python3 -m esptool --chip esp32 merge_bin -o "$out/aok-blind-bridge-$ver-eth-factory.bin" \
  --flash_mode dio --flash_size 4MB \
  0x1000 "$e/bootloader.bin" 0x8000 "$e/partitions.bin" 0xe000 "$boot_app0" 0x10000 "$e/firmware.bin"
cp "$e/firmware.bin" "$out/aok-blind-bridge-$ver-eth-ota.bin"
# ESP32-S3 build (the bootloader sits at 0x0 on the S3)
(cd "$proj" && pio run -e esp32s3)
s3="$proj/.pio/build/esp32s3"
python3 -m esptool --chip esp32s3 merge_bin -o "$out/aok-blind-bridge-$ver-s3-factory.bin" \
  --flash_mode dio --flash_size 4MB \
  0x0 "$s3/bootloader.bin" 0x8000 "$s3/partitions.bin" 0xe000 "$boot_app0" 0x10000 "$s3/firmware.bin"
cp "$s3/firmware.bin" "$out/aok-blind-bridge-$ver-s3-ota.bin"
echo "$ver"
