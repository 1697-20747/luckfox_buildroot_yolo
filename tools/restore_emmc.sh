#!/bin/bash
# Restore a full eMMC image made by tools/backup_emmc.sh, from the Mac, over
# USB, with the board in maskrom mode. No SD card needed.
#
#   1. Unplug the board. Hold BOOT, plug USB back in, release BOOT.
#   2. bash tools/restore_emmc.sh board_backup/emmc-YYYYMMDD-HHMM.img.gz
#
# Needs build/rkdeveloptool (macOS build of rockchip-linux/rkdeveloptool) and
# build/rv1106_download_*.bin (made by the SDK's boot_merger from
# rkbin/RKBOOT/RV1106MINIALL.ini). THIS OVERWRITES THE WHOLE eMMC.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RKDT="$ROOT/build/rkdeveloptool/rkdeveloptool"
LOADER="$(ls "$ROOT"/build/rv1106_download_*.bin | head -1)"
IMG_GZ="${1:?usage: $0 <emmc image .img.gz>}"

[ -f "$IMG_GZ.sha256" ] && { echo "Checking checksum..."; shasum -a 256 -c "$IMG_GZ.sha256"; }

"$RKDT" ld | grep -q Maskrom || { echo "Board not in maskrom mode (hold BOOT while plugging in USB)."; "$RKDT" ld; exit 1; }

RAW="${TMPDIR:-/tmp}/luckfox-restore.img"
echo "Decompressing to $RAW..."
gzip -dc "$IMG_GZ" > "$RAW"

read -r -p "Overwrite the board's ENTIRE eMMC with $(basename "$IMG_GZ")? Type YES: " ok
[ "$ok" = "YES" ] || { echo "Aborted, nothing written."; rm -f "$RAW"; exit 1; }

"$RKDT" db "$LOADER"
sleep 1
"$RKDT" wl 0 "$RAW"
"$RKDT" rd
rm -f "$RAW"
echo "Restore written; the board is rebooting."
