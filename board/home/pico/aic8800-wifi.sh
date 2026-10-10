#!/bin/sh
# Loads the AIC8800DC SDIO WiFi/BT driver stack at boot.
# insmod_wifi.sh's own chip-detection doesn't match this board (device-tree
# model has no trailing "W", SDIO ID check looks for an old ID pattern), so
# this loads the known-working module set directly instead.
set -e
cd /oem/usr/ko

insmod cfg80211.ko
insmod libarc4.ko
insmod ctr.ko
insmod ccm.ko
insmod aes_generic.ko
insmod aic8800_bsp.ko
sleep 0.5
insmod aic8800_fdrv.ko
sleep 1
insmod aic8800_btlpm.ko || true
