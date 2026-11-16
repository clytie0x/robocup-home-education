"""Firmware images: seal the application, join it with stage1, check one before writing.

  python tools/image.py seal build/app.raw build/app.bin
  python tools/image.py combine build/stage1.raw build/app.bin build/full.bin
  python tools/image.py check build/app.bin

The layout and the checks are the same as in boot.h / boot.c (stage1 runs them on the
chip before it starts the application). Standard library only, so it runs on the Pi.
"""
import re
import struct
import sys
import zlib

STAGE1_BASE = 0x08000000
APP_BASE = 0x08001000
APP_SPACE = 0x08010000 - APP_BASE
HEADER_OFFSET = 0x100
APP_MAGIC = 0x31505041  # "APP1"
RAM_START, RAM_END = 0x20000000, 0x20005000
VERSION = re.compile(rb"Vversion:\d+,build:[0-9a-z-]+#")


def seal(raw):
    """Write the length into the header and append the CRC-32 of the image."""
    raw = bytearray(raw)
    raw += b"\xff" * (-len(raw) % 4)  # flash is written in whole words
    magic, length = struct.unpack_from("<II", raw, HEADER_OFFSET)
    if magic != APP_MAGIC or length != 0:
        raise ValueError("no empty application header at +0x100: not an application image")
    struct.pack_into("<I", raw, HEADER_OFFSET + 4, len(raw))
    return bytes(raw) + struct.pack("<I", zlib.crc32(raw))


def check(image):
    """None if image is a whole sealed application, else what is wrong with it."""
    if len(image) < HEADER_OFFSET + 12:
        return "too short to be an application"
    magic, length = struct.unpack_from("<II", image, HEADER_OFFSET)
    if magic != APP_MAGIC:
        return "no application header at +0x100 (full.bin or the old firmware? those need the jumper)"
    if length + 4 != len(image) or length + 4 > APP_SPACE:
        return f"length {length} in the header does not match the file ({len(image)} bytes)"
    sp, reset = struct.unpack_from("<II", image, 0)
    if not (RAM_START < sp <= RAM_END and sp % 4 == 0):
        return f"stack pointer {sp:#x} is not in RAM"
    if not (reset & 1 and APP_BASE + HEADER_OFFSET <= reset < APP_BASE + length):
        return f"reset vector {reset:#x} is not inside the application"
    if zlib.crc32(image[:length]) != struct.unpack_from("<I", image, length)[0]:
        return "CRC-32 does not match: the file is damaged"
    return None


def version(image):
    """The exact V# reply the firmware in this image gives."""
    found = VERSION.findall(image)
    if len(found) != 1:
        raise ValueError(f"expected one version string in the image, found {len(found)}")
    return found[0].decode("ascii")


def combine(stage1, app):
    """stage1 padded to 4 KiB, then the application: the image for 0x08000000."""
    if len(stage1) > APP_BASE - STAGE1_BASE:
        raise ValueError(f"stage1 is {len(stage1)} bytes, more than its 4 KiB")
    problem = check(app)
    if problem:
        raise ValueError(problem)
    return stage1 + b"\xff" * (APP_BASE - STAGE1_BASE - len(stage1)) + app


def main(argv):
    command, *paths = argv
    read = lambda p: open(p, "rb").read()
    if command == "seal":
        out = seal(read(paths[0]))
        open(paths[1], "wb").write(out)
    elif command == "combine":
        out = combine(read(paths[0]), read(paths[1]))
        open(paths[2], "wb").write(out)
    elif command == "check":
        image = read(paths[0])
        problem = check(image)
        if problem:
            print(f"{paths[0]}: {problem}")
            return 1
        print(f"{paths[0]}: application OK, {len(image)} bytes, {version(image)}")
    else:
        print(__doc__)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
