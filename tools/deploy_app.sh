#!/bin/bash
# Copy the cross-built stream server (tools/build_app.sh) to the board's
# /root/rknn_test and restart it. /usr/bin/yolo-app-start (buildroot overlay)
# starts it at boot from there.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BOARD=root@172.32.0.70
cd "$ROOT/build/app_src"
# COPYFILE_DISABLE: stop macOS tar adding ._* AppleDouble files
COPYFILE_DISABLE=1 tar -cf - luckfox_video_hud_stream_cam luckfox_video_hud_stream_rockit luckfox_video_hud_stream_hwjpeg yolo_supervisor yolo_camd www app.env \
    yolov8n-optimized-rv1106-1.6.0.rknn $(ls yolov8n-*x*-rv1106-1.6.0.rknn 2>/dev/null) coco_80_labels_list.txt \
  | ssh "$BOARD" 'set -e
    # Unpack into a staging dir, then RENAME into place: never write over a
    # file the running server has mapped (its binary, libjpeg). All four
    # board freezes up to 2026-10-11 happened right after a deploy that
    # overwrote them in place. A rename leaves the old inode intact for the
    # old process; the restart picks up the new file.
    APP=/root/rknn_test; NEW=/root/rknn_test.deploy
    rm -rf $NEW && mkdir -p $NEW $APP && tar -xf - -C $NEW && chown -R root:root $NEW
    [ -d $APP/www/docs ] && mv $APP/www/docs $NEW/www/docs   # docs site is deployed separately
    sync
    for f in luckfox_video_hud_stream_cam luckfox_video_hud_stream_rockit luckfox_video_hud_stream_hwjpeg yolo_supervisor yolo_camd app.env yolov8n-optimized-rv1106-1.6.0.rknn coco_80_labels_list.txt $(cd $NEW && ls yolov8n-*x*-rv1106-1.6.0.rknn 2>/dev/null); do
        mv -f $NEW/$f $APP/$f
    done
    rm -rf $APP/lib   # libjpeg is linked statically now; old shared copy unused
    rm -rf $APP/www.old; [ -d $APP/www ] && mv $APP/www $APP/www.old; mv $NEW/www $APP/www; rm -rf $APP/www.old
    rm -rf $NEW; sync'
# Stop the old server only after the new files are safely on disk; the
# boot-time restart loop starts the new binary within 5s. NO_RESTART=1 leaves
# the running server alone (the new files take effect at the next start or
# reboot) -- e.g. when a reboot follows anyway.
if [ "${NO_RESTART:-0}" = 1 ]; then
    echo "NO_RESTART=1: files installed, running server left alone"
    exit 0
fi
ssh "$BOARD" 'killall luckfox_video_hud_stream_cam luckfox_video_hud_stream_rockit luckfox_video_hud_stream_hwjpeg 2>/dev/null; true'
echo "Deployed. If yolo-app-start's restart loop is running it restarts the server within 5s;"
echo "otherwise: ssh $BOARD /usr/bin/yolo-app-start &   (or reboot)"
