#!/bin/bash
# Cross-compile the YOLO HUD stream server for the Buildroot board (uclibc
# toolchain from the Luckfox SDK in the luckfox-sdk Docker volume) and stage
# everything it needs to run in build/app_src/:
#   luckfox_video_hud_stream_cam (libjpeg-turbo linked statically -- build it
#   first with tools/build_jpeg_turbo.sh), the yolov8n model + labels (from
#   board_backup/), www/ (the web UI).
# Hardware JPEG (Rockit VENC) is built as a separate opt-in binary,
# luckfox_video_hud_stream_hwjpeg (re-test on the yolo_camd base, 2026-10-11).
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
cp "$ROOT"/scripts/luckfox_video_hud_stream_cam.c "$ROOT"/scripts/apex_hud_widgets.[ch] "$ROOT"/scripts/yolo_supervisor.c "$ROOT"/scripts/yolo_camd.[ch] "$OUT"/
rm -rf "$OUT/www" && cp -R "$ROOT/scripts/www" "$OUT/www"
cp "$ROOT/scripts/app.env" "$OUT/app.env"
# models made by tools/export_yolov8_rknn.sh (e.g. the 800x448 default)
cp "$ROOT"/build/models/*.rknn "$OUT"/ 2>/dev/null || true

docker run --rm --platform linux/amd64 -v luckfox-sdk:/sdk -v "$OUT":/src luckfox-sdk bash -c '
set -e
S=/sdk/luckfox-pico
TC=$S/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin/arm-rockchip830-linux-uclibcgnueabihf
M=$S/output/out/media_out; A=$S/output/out/app_out
cd /src
${TC}-g++ -O2 -Wall -Wno-unused-function -Wno-unused-variable -Wno-comment \
  luckfox_video_hud_stream_cam.c utils/image_drawing.c apex_hud_widgets.c \
  -I. -Iutils -I3rdparty/stb_image -I$M/include/rga -I$M/include -I/sdk/jpeg-turbo/include \
  -L$M/lib -Wl,-rpath-link,$M/lib -Wl,-rpath,/oem/usr/lib \
  -o luckfox_video_hud_stream_cam -lrknnmrt -lrga /sdk/jpeg-turbo/lib/libjpeg.a -lm -lpthread
# Option C: the same server with Rockit VI capture + in-process 3A (see
# LUCKFOX_ROCKIT_VI in the source); selected on the board by app.env
# LUCKFOX_PIPELINE=rockit. Flags match the RV1106 sample Makefile in the SDK.
AIQ=$M/include/rkaiq
${TC}-g++ -O2 -Wall -Wno-unused-function -Wno-unused-variable -Wno-comment \
  -DLUCKFOX_ROCKIT_VI -DISP_HW_V30 -DRV1106 -DUAPI2 \
  luckfox_video_hud_stream_cam.c utils/image_drawing.c apex_hud_widgets.c \
  -I. -Iutils -I3rdparty/stb_image -I$M/include/rga -I$M/include -I/sdk/jpeg-turbo/include \
  -I$AIQ -I$AIQ/uAPI2 -I$AIQ/common -I$AIQ/xcore -I$AIQ/algos -I$AIQ/iq_parser -I$AIQ/iq_parser_v2 \
  -L$M/lib -Wl,-rpath-link,$M/lib -Wl,-rpath,/oem/usr/lib \
  -o luckfox_video_hud_stream_rockit -lrknnmrt -lrga -lrockit -lrkaiq /sdk/jpeg-turbo/lib/libjpeg.a -lm -lpthread
# Hardware JPEG re-test (2026-10-11, on the yolo_camd base): Rockit VENC
# encode, opt-in binary selected by LUCKFOX_PIPELINE=hwjpeg + LUCKFOX_HW_JPEG=1
# in app.env. Software JPEG stays the default.
${TC}-g++ -O2 -Wall -Wno-unused-function -Wno-unused-variable -Wno-comment \
  -DLUCKFOX_HW_JPEG \
  luckfox_video_hud_stream_cam.c utils/image_drawing.c apex_hud_widgets.c \
  -I. -Iutils -I3rdparty/stb_image -I$M/include/rga -I$M/include -I/sdk/jpeg-turbo/include \
  -L$M/lib -Wl,-rpath-link,$M/lib -Wl,-rpath,/oem/usr/lib \
  -o luckfox_video_hud_stream_hwjpeg -lrknnmrt -lrga -lrockit /sdk/jpeg-turbo/lib/libjpeg.a -lm -lpthread
# the watchdog supervisor: static, so it needs nothing from disk once running
${TC}-gcc -O2 -Wall -static -o yolo_supervisor yolo_supervisor.c -lpthread
# the camera stream owner (scripts/yolo_camd.c): static for the same reason
${TC}-gcc -O2 -Wall -static -o yolo_camd yolo_camd.c
'
ls -la "$OUT/luckfox_video_hud_stream_cam" "$OUT/luckfox_video_hud_stream_rockit" "$OUT/luckfox_video_hud_stream_hwjpeg" "$OUT/yolo_supervisor" "$OUT/yolo_camd"
