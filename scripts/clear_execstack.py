"""Clears the executable-stack (PF_X) bit on an ELF file's PT_GNU_STACK
program header, without needing the (no-longer-packaged) `execstack`
binutils tool. Root cause: QEMU's x86_64 emulation on Apple Silicon
Docker Desktop refuses to load old prebuilt .so files that request an
executable stack ("cannot enable executable stack as shared object
requires: Invalid argument") -- a real, documented emulation quirk, not
an actual security requirement of these libraries.
"""
import struct
import sys

PT_GNU_STACK = 0x6474e551
PF_X = 0x1


def clear_execstack(path: str) -> bool:
    with open(path, "r+b") as f:
        data = bytearray(f.read())
        if data[:4] != b"\x7fELF":
            return False
        ei_class = data[4]  # 1 = 32-bit, 2 = 64-bit
        is64 = ei_class == 2
        if is64:
            e_phoff = struct.unpack_from("<Q", data, 0x20)[0]
            e_phentsize = struct.unpack_from("<H", data, 0x36)[0]
            e_phnum = struct.unpack_from("<H", data, 0x38)[0]
        else:
            e_phoff = struct.unpack_from("<I", data, 0x1C)[0]
            e_phentsize = struct.unpack_from("<H", data, 0x2A)[0]
            e_phnum = struct.unpack_from("<H", data, 0x2C)[0]

        changed = False
        for i in range(e_phnum):
            off = e_phoff + i * e_phentsize
            p_type = struct.unpack_from("<I", data, off)[0]
            if p_type == PT_GNU_STACK:
                flags_off = off + 4  # p_flags follows p_type in both 32/64-bit layouts here
                p_flags = struct.unpack_from("<I", data, flags_off)[0]
                if p_flags & PF_X:
                    struct.pack_into("<I", data, flags_off, p_flags & ~PF_X)
                    changed = True

        if changed:
            f.seek(0)
            f.write(data)
        return changed


if __name__ == "__main__":
    for path in sys.argv[1:]:
        try:
            changed = clear_execstack(path)
            print(f"{'CLEARED' if changed else 'no-op (not exec-stack or no GNU_STACK)'}: {path}")
        except Exception as e:
            print(f"FAILED {path}: {e!r}")
