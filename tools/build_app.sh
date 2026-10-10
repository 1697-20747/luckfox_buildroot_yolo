#!/bin/bash
# Cross-compile the YOLO HUD stream server for the Buildroot board (uclibc
# toolchain from the Luckfox SDK in the luckfox-sdk Docker volume) and stage
# everything it needs to run in build/app_src/:
#   luckfox_video_hud_stream_cam, lib/libjpeg.so.9 (not in the Buildroot image),
#   the yolov8n model + labels (from board_backup/), www/ (the web UI).
# Then deploy with tools/deploy_app.sh.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/build/app_src"
mkdir -p "$OUT"

# Rockchip support files that came from the Ubuntu board (rknn_model_zoo's
# drawing utils + font, stb, rga headers, rknn_api.h), the model and labels.
if [ ! -f "$OUT/utils/image_drawing.c" ]; then
    tar -xzf "$ROOT/board_backup/home_pico.tar.gz" -C "$OUT" --strip-components=2 \
        pico/rknn_test/utils pico/rknn_test/3rdparty pico/rknn_test/rknn_api.h \
        pico/rknn_test/coco_80_labels_list.txt pico/rknn_test/yolov8n-optimized-rv1106-1.6.0.rknn
fi
cp "$ROOT"/scripts/luckfox_video_hud_stream_cam.c "$ROOT"/scripts/apex_hud_widgets.[ch] "$OUT"/
rm -rf "$OUT/www" && cp -R "$ROOT/scripts/www" "$OUT/www"

docker run --rm --platform linux/amd64 -v luckfox-sdk:/sdk -v "$OUT":/src luckfox-sdk bash -c '
set -e
S=/sdk/luckfox-pico
TC=$S/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin/arm-rockchip830-linux-uclibcgnueabihf
M=$S/output/out/media_out; A=$S/output/out/app_out
cd /src
${TC}-g++ -O2 -Wall -Wno-unused-function -Wno-unused-variable -Wno-comment \
  luckfox_video_hud_stream_cam.c utils/image_drawing.c apex_hud_widgets.c \
  -I. -Iutils -I3rdparty/stb_image -I$M/include/rga -I$M/include -I$A/include/libjpeg9 \
  -L$M/lib -L$A/lib -Wl,-rpath-link,$M/lib -Wl,-rpath,/oem/usr/lib -Wl,-rpath,\$ORIGIN/lib \
  -DLUCKFOX_HW_JPEG \
  -o luckfox_video_hud_stream_cam -lrknnmrt -lrga -ljpeg -lrockit -lm -lpthread
mkdir -p lib && cp -L $A/lib/libjpeg.so.9 lib/
'
ls -la "$OUT/luckfox_video_hud_stream_cam"
