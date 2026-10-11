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
set -euo pipefail
KERNEL=/sdk/luckfox-pico/sysdrv/source/kernel
cd "$KERNEL"
for p in /work/buildroot/kernel-patches/*.patch; do
    if git apply --reverse --check "$p" 2>/dev/null; then
        echo "already applied: $(basename "$p")"
    else
        git apply "$p"
        echo "applied: $(basename "$p")"
    fi
done
