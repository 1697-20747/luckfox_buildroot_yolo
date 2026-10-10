#!/bin/bash
# Flash the Luckfox SDK Buildroot images to the board's eMMC from the Mac, with
# the board in maskrom mode (hold BOOT while plugging in USB). Writes each
# partition image at its offset from the SDK's partition table:
#   32K(env),512K@32K(idblock),256K(uboot),32M(boot),512M(oem),256M(userdata),6G(rootfs)
# which is the same layout the Ubuntu image uses (rootfs ends at sector
# 14222912 in both), so nothing outside these partitions is touched.
#
#   bash tools/flash_buildroot.sh [build/buildroot-image-YYYYMMDD-HHMM]
#
# Uses the same db + wl path verified on this board 2026-10-10 (see
# tools/restore_emmc.sh). To go back to Ubuntu: tools/restore_emmc.sh with
# the board_backup/ image.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RKDT="$ROOT/build/rkdeveloptool/rkdeveloptool"
LOADER="$ROOT/build/rv1106_download_v1.15.108.bin"
DIR="${1:-$ROOT/$(cat "$ROOT/build/LATEST_BUILDROOT_IMAGE")}"

# partition name : start sector (512-byte sectors)
PARTS="env:0 idblock:64 uboot:1088 boot:1600 oem:67136 userdata:1115712 rootfs:1640000"

echo "Images: $DIR"
(cd "$DIR" && shasum -a 256 -c SHA256SUMS)
grep -q '32K(env),512K@32K(idblock),256K(uboot),32M(boot),512M(oem),256M(userdata),6G(rootfs)' "$DIR/.env.txt" \
    || { echo "Partition table in $DIR/.env.txt differs from the offsets in this script. Stopping."; exit 1; }

"$RKDT" ld | grep -q Maskrom || { echo "Board not in maskrom mode (hold BOOT while plugging in USB)."; "$RKDT" ld; exit 1; }

read -r -p "Replace the board's Ubuntu system with this Buildroot image? Type YES: " ok
[ "$ok" = "YES" ] || { echo "Aborted, nothing written."; exit 1; }

"$RKDT" db "$LOADER"
sleep 2
"$RKDT" rfi | grep "Flash Size: .*Sectors"
for p in $PARTS; do
    name=${p%%:*}
    sector=${p##*:}
    echo "== $name.img -> sector $sector"
    "$RKDT" wl "$sector" "$DIR/$name.img"
done
"$RKDT" rd
echo "Flashed; the board is rebooting into Buildroot. Expect usb0 at 172.32.0.70 (ssh root@172.32.0.70)."
