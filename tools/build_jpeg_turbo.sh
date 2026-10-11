#!/bin/bash
# Cross-builds libjpeg-turbo (static, ARM NEON) with the Luckfox SDK's uclibc
# toolchain, into /sdk/jpeg-turbo in the luckfox-sdk Docker volume, for
# tools/build_app.sh to link statically.
#
# Why (2026-10-11): the Buildroot image's only JPEG library is IJG libjpeg 9
# (no SIMD: 83-93ms per 960x540 frame, 5.4fps). The RV1106 hardware encoder
# (Rockit VENC) was faster but froze the board intermittently -- during
# startup, streaming and teardown -- when used alongside V4L2 capture. This
# is the same NEON-accelerated library the Ubuntu build used (26ms/frame).
#
# Version and sha256 pinned to the SDK's own Buildroot package
# (package/jpeg-turbo/jpeg-turbo.hash, buildroot-2023.02.6).
set -euo pipefail
docker run --rm --platform linux/amd64 -v luckfox-sdk:/sdk luckfox-sdk bash -c '
set -euo pipefail
VER=2.1.4
SHA256=d3ed26a1131a13686dfca4935e520eb7c90ae76fbc45d98bb50a8dc86230342b
TC=/sdk/luckfox-pico/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin/arm-rockchip830-linux-uclibcgnueabihf
SRC=/sdk/jpeg-turbo-src; OUT=/sdk/jpeg-turbo
mkdir -p $SRC && cd $SRC
[ -f libjpeg-turbo-$VER.tar.gz ] || curl -fsSL -o libjpeg-turbo-$VER.tar.gz \
    https://downloads.sourceforge.net/project/libjpeg-turbo/$VER/libjpeg-turbo-$VER.tar.gz
echo "$SHA256  libjpeg-turbo-$VER.tar.gz" | sha256sum -c -
rm -rf libjpeg-turbo-$VER build && tar xzf libjpeg-turbo-$VER.tar.gz && mkdir build && cd build
cmake ../libjpeg-turbo-$VER -DCMAKE_SYSTEM_NAME=Linux -DCMAKE_SYSTEM_PROCESSOR=arm \
    -DCMAKE_C_COMPILER=${TC}-gcc -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_FLAGS="-mcpu=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard" \
    -DENABLE_SHARED=0 -DENABLE_STATIC=1 -DWITH_SIMD=1 -DWITH_TURBOJPEG=0 \
    -DCMAKE_INSTALL_PREFIX=$OUT > /dev/null
make -j"$(nproc)" > /dev/null
rm -rf $OUT && make install > /dev/null
grep -E "WITH_SIMD|NEON" CMakeCache.txt | grep -v "^//" | head -5
ls -la $OUT/lib/libjpeg.a $OUT/include/jpeglib.h
'
