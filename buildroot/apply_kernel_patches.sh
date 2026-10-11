#!/bin/bash
# Applies buildroot/kernel-patches/*.patch to the Luckfox SDK's kernel tree
# (idempotent: a patch that's already applied is skipped). Run inside the
# luckfox-sdk container with the repo mounted at /work, before ./build.sh
# (idempotent):
#   docker run --rm --platform linux/amd64 -v luckfox-sdk:/sdk -v "$(pwd)":/work \
#     luckfox-sdk bash /work/buildroot/apply_kernel_patches.sh
#
# 0001-mpu6050-i2c2-polled-mode.patch: GY-521/MPU-6050 on I2C2 (header pins
# 32/33, no INT line) -- enables i2c2 with an mpu6050@68 node, builds the
# inv_mpu6050 I2C driver in, and lets that driver run without an IRQ
# (direct mode, polled sysfs reads) instead of failing to probe.
#
# Applied strictly in order and recorded in a stamp file in the SDK
# (.yolo-kernel-patches). A patch not in the stamp is applied with git apply,
# which fails loudly if it doesn't fit. (Per-patch reverse checks broke as
# soon as a later patch touched lines next to an earlier one, 2026-10-11.)
# Runs from the SDK's git root with --directory: `git apply` inside the kernel
# subdirectory silently ignores the patch paths.
set -euo pipefail
cd /sdk/luckfox-pico
DIR=sysdrv/source/kernel
STAMP=.yolo-kernel-patches
touch $STAMP
for p in /work/buildroot/kernel-patches/*.patch; do
    name=$(basename "$p")
    if grep -qx "$name" $STAMP; then
        echo "already applied: $name"
    else
        git apply --directory=$DIR "$p"
        echo "$name" >> $STAMP
        echo "applied: $name"
    fi
done
