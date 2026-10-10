#!/bin/sh
# Swap in the web-UI build of the stream server (keeps a backup), then restart the service.
set -e
cd /home/pico/rknn_test
cp -p luckfox_video_hud_stream_cam luckfox_video_hud_stream_cam.bak-pre-webui
systemctl stop luckfox-video-stream.service
cp luckfox_video_hud_stream_cam_webui luckfox_video_hud_stream_cam
systemctl start luckfox-video-stream.service
sleep 3
systemctl is-active luckfox-video-stream.service
journalctl -u luckfox-video-stream.service -n 8 --no-pager
