# RKNN model conversion image for the Luckfox Pico Ultra (RV1106G3).
#
# Must run --platform linux/amd64 (x86_64 emulation on Apple Silicon) --
# rknn-toolkit2 1.6.0 predates ARM64 host support (that landed in 2.3.0),
# so no arm64 wheel exists for this old, RV1106-runtime-matching version.
# See ../PROJECT_MEMORY.md for why 1.6.0 specifically, not the current
# 2.3.2 (real Luckfox on-device runtime is 1.6.0; a 2.3.2-exported model
# is a format the 1.6.0 runtime can't parse).
#
# Build: docker build --platform linux/amd64 -f docker/convert.Dockerfile -t luckfox-rknn-convert .
# Run:   docker run --rm --platform linux/amd64 -v "$(pwd)":/work -w /work luckfox-rknn-convert bash

FROM python:3.10-slim

# cmake/build-essential: onnxoptimizer (a transitive dep) has no
# prebuilt wheel for this combo and builds from source.
# libgl1/libglib2.0-0: opencv-python's runtime needs libGL.so.1, absent
# from the slim base image.
RUN apt-get update -qq && apt-get install -y -qq \
    cmake build-essential libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# The real rknn-toolkit2 1.6.0 x86_64 wheel -- never published to PyPI
# (pip installability was added in 2.2.0; this predates that), so it's
# fetched from its GitHub tag directly. Filename MUST be exact
# (pip parses wheel metadata from the filename itself) -- don't rename it.
RUN pip install --no-cache-dir --no-input \
    "https://raw.githubusercontent.com/airockchip/rknn-toolkit2/v1.6.0/rknn-toolkit2/packages/rknn_toolkit2-1.6.0%2B81f21f4d-cp310-cp310-linux_x86_64.whl" \
    "protobuf==3.20.3" "onnx==1.14.1" "numpy<2" opencv-python-headless

# Real, QEMU-x86_64-emulation-specific bug found 2026-09-08: old prebuilt
# .so files requesting an executable stack fail to load under Rosetta/QEMU
# emulation on Apple Silicon Docker Desktop ("cannot enable executable
# stack as shared object requires: Invalid argument"). The normal fix
# (`execstack -c`) isn't packaged in this (or even Debian bullseye's, now
# EOL) apt repos -- clear_execstack.py patches the ELF PT_GNU_STACK flag
# directly instead, no extra system package needed.
COPY scripts/clear_execstack.py /tmp/clear_execstack.py
RUN find /usr/local/lib/python3.10/site-packages/onnxruntime -name "*.so" \
    -exec python /tmp/clear_execstack.py {} \;

WORKDIR /work
