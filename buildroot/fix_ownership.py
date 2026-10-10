#!/usr/bin/env python3
"""Normalise file ownership in an SDK staging dir right before mkfs.ext4 packs it.

Why: this SDK runs in Docker as an ordinary user (uid 1000), and `mkfs.ext4 -d`
records the real on-disk owners -- so every file in the flashed rootfs (including
/etc/shadow, /root and setuid busybox) came out owned by uid 1000. Buildroot's
own rootfs.tar (made under fakeroot) holds the intended ownership: everything
root, with a handful of exceptions (dbus helper group, /var/www, setuid bits).

So: chown everything to 0:0, then re-apply owner+mode for every (non-link)
entry in rootfs.tar that exists in the staging dir. Modes matter too: the SDK's
overlays are copied with `rsync --chmod=u=rwX,go=rX`, which left /etc/shadow
0755. Files not in rootfs.tar (SDK media libs, overlays) end up root:root with
their mode unchanged. Must run as root (the firmware step is run with
`docker run -u root`); as non-root it does nothing and says so.

Usage: fix_ownership.py <staging dir>
"""
import os
import sys
import tarfile

ROOTFS_TAR = "/sdk/luckfox-pico/sysdrv/source/buildroot/buildroot-2023.02.6/output/images/rootfs.tar"


def main():
    staging = sys.argv[1]
    if os.geteuid() != 0:
        print(f"fix_ownership: not root, leaving {staging} as-is (files will be uid {os.getuid()})")
        return
    for dirpath, dirnames, filenames in os.walk(staging):
        for name in dirnames + filenames:
            os.lchown(os.path.join(dirpath, name), 0, 0)
    os.lchown(staging, 0, 0)

    restored = 0
    # only the rootfs staging dir corresponds to rootfs.tar
    if os.path.exists(os.path.join(staging, "bin", "busybox")):
        with tarfile.open(ROOTFS_TAR) as tar:
            for m in tar:
                if m.issym() or m.islnk():
                    continue
                path = os.path.join(staging, m.name)
                if os.path.lexists(path) and not os.path.islink(path):
                    os.lchown(path, m.uid, m.gid)
                    os.chmod(path, m.mode)
                    restored += 1
    print(f"fix_ownership: {staging} -> root:root, {restored} owner/mode entries restored from rootfs.tar")


if __name__ == "__main__":
    main()
