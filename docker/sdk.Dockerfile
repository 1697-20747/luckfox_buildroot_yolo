# Luckfox Pico SDK (Buildroot) build environment for the Pico Ultra (RV1106G3).
#
# Must run --platform linux/amd64: the SDK's prebuilt cross toolchain
# (arm-rockchip830-linux-uclibcgnueabihf) is an x86_64 binary, and Luckfox
# only supports Ubuntu 22.04 x86_64 hosts.
#
# The SDK checkout must live in a Docker volume, NOT a bind mount of a Mac
# folder: macOS filesystems are case-insensitive by default and the kernel /
# Buildroot trees contain files differing only in case.
#
# Build: docker build --platform linux/amd64 -f docker/sdk.Dockerfile -t luckfox-sdk .
# Run:   docker run --rm -it --platform linux/amd64 \
#          -v luckfox-sdk:/sdk -v "$(pwd)":/work luckfox-sdk bash
#        (/sdk = case-sensitive SDK volume, /work = this repo, for copying
#         images out and our sources in)

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
# Package list per Luckfox's SDK environment setup docs for Ubuntu 22.04.
RUN apt-get update -qq && apt-get install -y -qq \
    git ssh make gcc gcc-multilib g++-multilib module-assistant expect g++ gawk \
    texinfo libssl-dev bison flex fakeroot cmake unzip gperf autoconf \
    device-tree-compiler libncurses5-dev pkg-config bc python-is-python3 passwd \
    openssl openssh-client vim file cpio rsync ca-certificates locales sudo curl \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8

ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# GNU make 4.4.1 replaces Ubuntu 22.04's make 4.3. Real failure seen
# 2026-10-10: under amd64 emulation on Apple Silicon, make 4.3's
# pipe-fd jobserver repeatedly died with "write jobserver: Bad file
# descriptor" in nested makes (buildroot busybox, freetype, then the
# media samples) -- three different places, so environmental, not one
# bad Makefile. make 4.4 defaults to a named-FIFO jobserver, which
# doesn't depend on inherited file descriptors.
RUN cd /tmp && curl -fsSLO https://ftp.gnu.org/gnu/make/make-4.4.1.tar.gz \
    && echo "dd16fb1d67bfab79a72f5e8390735c49e3e8e70b4945a15ab1f81ddb78658fb3  make-4.4.1.tar.gz" | sha256sum -c - \
    && tar xzf make-4.4.1.tar.gz && cd make-4.4.1 \
    && ./configure --prefix=/usr/local -q && make -s -j"$(nproc)" && make -s install \
    && cd / && rm -rf /tmp/make-4.4.1*

# Buildroot refuses to build as root; build as an ordinary user.
RUN useradd -m -s /bin/bash builder && echo "builder ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/builder
USER builder
WORKDIR /sdk
