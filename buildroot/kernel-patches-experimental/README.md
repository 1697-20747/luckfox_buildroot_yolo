Kernel patches tried as experiments and NOT applied by apply_kernel_patches.sh.

- 0004-emmc-isp-quiesce-experiment.patch (2026-10-11): ISP/VICAP guarded register writes wait
  (IRQs off, up to 10ms) for the eMMC IDMAC to be idle.
  - Result: 1 hang in 40 persistent-3A restarts, same IDMAC DESC_RD signature. Not a fix.
  - Build manually: git apply --directory=sysdrv/source/kernel, ./build.sh kernel, then reverse-apply.
