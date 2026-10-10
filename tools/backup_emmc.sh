#!/bin/bash
# Full byte-for-byte image of the board's eMMC (/dev/mmcblk0, 7.1GB), streamed
# over SSH to the Mac and gzipped here (the board's single core would be the
# bottleneck if it compressed). Run from the standalone macOS Terminal app:
#   bash tools/backup_emmc.sh
# The sudo password is read here without echo and fed to `sudo -S` on the
# board; it is never stored or put on a command line.
#
# Taken from a running system, so the root filesystem is "crash-consistent"
# (like pulling the power): fine for a rollback image, fsck tidies it on
# first boot. Restore = rkdeveloptool from the Mac with the board in maskrom
# mode (BOOT button held while plugging in USB) -- no SD card needed.
set -euo pipefail

BOARD=pico@172.32.0.70
OUT_DIR="$(cd "$(dirname "$0")/.." && pwd)/board_backup"
OUT="$OUT_DIR/emmc-$(date +%Y%m%d-%H%M).img.gz"
mkdir -p "$OUT_DIR"

read -r -s -p "sudo password for $BOARD: " PW
echo
echo "Imaging /dev/mmcblk0 -> $OUT (expect several minutes over USB)..."
printf '%s\n' "$PW" | ssh "$BOARD" 'sudo -S -p "" sh -c "sync; dd if=/dev/mmcblk0 bs=4M status=none"' | gzip -1 > "$OUT"
unset PW

EXPECTED=$(ssh "$BOARD" 'echo $(( $(cat /sys/block/mmcblk0/size) * 512 ))')
echo "Verifying (decompressing to count bytes)..."
RAW_BYTES=$(gzip -dc "$OUT" | wc -c | tr -d ' ')
if [ "$RAW_BYTES" != "$EXPECTED" ]; then
    echo "FAILED: image is $RAW_BYTES bytes, eMMC is $EXPECTED. Do not rely on this backup."
    exit 1
fi
echo "OK: $(du -h "$OUT" | cut -f1) compressed, $RAW_BYTES bytes raw = full eMMC."
shasum -a 256 "$OUT" | tee "$OUT.sha256"
