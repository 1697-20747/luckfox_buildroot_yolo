#!/bin/bash
# Applies buildroot/kernel-patches/*.patch to the Luckfox SDK's kernel tree
# (idempotent: a patch that's already applied is skipped). Run inside the
# luckfox-sdk container with the repo mounted at /work, before ./build.sh:
#   docker run --rm --platform linux/amd64 -v luckfox-sdk:/sdk -v "$(pwd)":/work \
#     luckfox-sdk bash /work/buildroot/apply_kernel_patches.sh
#
# 0001-mpu6050-i2c2-polled-mode.patch: GY-521/MPU-6050 on I2C2 (header pins
# 32/33, no INT line) -- enables i2c2 with an mpu6050@68 node, builds the
# inv_mpu6050 I2C driver in, and lets that driver run without an IRQ
# (direct mode, polled sysfs reads) instead of failing to probe.
#
# Run from the SDK's git root with --directory: `git apply` run inside the
# kernel subdirectory silently IGNORES the patch's paths (it resolves them
# against the repo root), so both applying and the "already applied" check
# quietly did nothing -- found 2026-10-11.
set -euo pipefail
cd /sdk/luckfox-pico
DIR=sysdrv/source/kernel
for p in /work/buildroot/kernel-patches/*.patch; do
    if git apply --directory=$DIR --reverse --check "$p" 2>/dev/null; then
        echo "already applied: $(basename "$p")"
    else
        git apply --directory=$DIR "$p"
        echo "applied: $(basename "$p")"
    fi
done
