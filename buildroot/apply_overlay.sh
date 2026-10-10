#!/bin/bash
# Copies this repo's Buildroot rootfs overlay into the Luckfox SDK and adds it
# to the Pico Ultra eMMC board config's RK_POST_OVERLAY, so ./build.sh copies it
# over the rootfs last (after Luckfox's own overlays). Run inside the
# luckfox-sdk container with the repo mounted at /work:
#   docker run --rm --platform linux/amd64 -v luckfox-sdk:/sdk -v "$(pwd)":/work \
#     luckfox-sdk bash /work/buildroot/apply_overlay.sh
set -euo pipefail

SDK=/sdk/luckfox-pico
CFG_DIR=$SDK/project/cfg/BoardConfig_IPC
BOARD_MK=$CFG_DIR/BoardConfig-EMMC-Buildroot-RV1106_Luckfox_Pico_Ultra-IPC.mk
NAME=overlay-yolo

rm -rf "$CFG_DIR/overlay/$NAME"
cp -a /work/buildroot/$NAME "$CFG_DIR/overlay/$NAME"
chmod 755 "$CFG_DIR/overlay/$NAME"/etc/init.d/*

if ! grep -q "$NAME" "$BOARD_MK"; then
    sed -i "s/^export RK_POST_OVERLAY=\"\(.*\)\"/export RK_POST_OVERLAY=\"\1 $NAME\"/" "$BOARD_MK"
fi
grep '^export RK_POST_OVERLAY=' "$BOARD_MK"
find "$CFG_DIR/overlay/$NAME" -type f | sed "s|$CFG_DIR/overlay/||"

# Ownership fix: hook buildroot/fix_ownership.py into the SDK's ext4 packer,
# just before `mkfs.ext4 -d $src`. Idempotent (marker line). See the script's
# docstring for why; the firmware step must then run as root.
# The SDK copies mkfs_ext4.sh into its output tree and runs that copy
# (output/out/sysdrv_out/pc/), so hook every copy.
for MKFS in $SDK/sysdrv/tools/pc/e2fsprogs/mkfs_ext4.sh $SDK/sysdrv/out/bin/pc/mkfs_ext4.sh \
            $SDK/output/out/sysdrv_out/pc/mkfs_ext4.sh; do
    [ -f "$MKFS" ] || continue
    cp /work/buildroot/fix_ownership.py "$(dirname "$MKFS")/fix_ownership.py"
    if ! grep -q "luckfox_buildroot_yolo: fix_ownership" "$MKFS"; then
        sed -i '0,/^mkfs.ext4 -d \$src/s||# luckfox_buildroot_yolo: fix_ownership\npython3 "$(dirname "$0")/fix_ownership.py" "$src"\n&|' "$MKFS"
    fi
    echo "hooked: $MKFS"
done
