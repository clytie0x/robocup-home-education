"""Update the base firmware from the Pi, without the BOOT0 jumper.

  python ~/firmware-new/update.py ~/firmware-new/app.bin
  python ~/firmware-new/update.py --rescue ~/firmware-new/app.bin   (chip already waits in the bootloader)
  python ~/firmware-new/update.py --break ~/firmware-new/app.bin    (application deaf: break escape)
  python ~/firmware-new/update.py --force ~/firmware-new/app.bin    (Uforce#: a broken encoder blocks U#)

Needs stage1 on the chip (written once with the jumper, FLASHING.md section 1), pyserial,
stm32flash, fuser, and image.py next to this file. Steps:

  1. check app.bin (header, length, CRC-32) and that no program holds the port
  2. into the ST ROM bootloader, one of:
       normal    V# (the running firmware answers), S#, then U#: it restarts into the
                 bootloader (only while the wheels stand still)
       --rescue  nothing: the chip already waits there
       --break   the break pattern on the cable while the owner powers the base off and
                 on; stage1 sees it and starts the bootloader (needs nothing from the app)
  3. stm32flash: write app.bin at 0x08001000 with verify, read it back and compare
  4. stm32flash Go to 0x08000000: stage1 checks the new application and starts it
  5. V# must give the version string inside app.bin

A waiting ROM bootloader takes the first byte it gets as its baud-rate probe (0x7F). Any
other byte first (S# from wheels.py, a V#) can upset it until the next power cycle, so
--rescue and --break send nothing before stm32flash's own handshake.

stage1 (0x08000000 to 0x08000FFF) is never written here. On any failure the script says
what state the chip is in and what to do.
"""
import argparse
import os
import subprocess
import sys
import tempfile
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import image  # noqa: E402

PORT = "/dev/rct_ahr_robot"
BAUD = "115200"  # stm32flash's default framing is 8E1, the same as the firmware's
APP_ADDRESS = f"{image.APP_BASE:#010x}"
STAGE1_ADDRESS = f"{image.STAGE1_BASE:#010x}"
BYTE_S = 0.2  # one zero byte at 50 baud 8N1: 180 ms low, 20 ms stop bit (boot.h)

FRESH_START = ("Before trying again: stop every program on the port, power the base off and on, "
               "send nothing, then run with --rescue (or --break if the application starts).")
UNTOUCHED = "Nothing was written: the firmware that ran before still runs."
NO_BOOTLOADER = ("U# was accepted, but the ROM bootloader does not answer. Nothing was written. "
                 "Power the base off and on: stage1 starts the unchanged application. "
                 "If V# still does not answer after that, the chip may be waiting in the bootloader: "
                 + FRESH_START)
IN_BOOTLOADER = ("The chip waits in the ST ROM bootloader, motors off, and the application area may be "
                 "half written. stage1 is untouched. Run again with --rescue and the same or an older "
                 "app.bin, sending nothing else first. If that fails too: " + FRESH_START + " A power "
                 "cycle comes back to the bootloader if the application is broken (stage1 checks its "
                 "CRC) and starts it if it is whole.")
GO_FAILED = ("The new application is written and read back correctly, but the Go command failed. "
             "Power the base off and on: stage1 checks the new application and starts it.")


class Failed(Exception):
    pass


def fail(what, state):
    raise Failed(f"FAILED: {what}\n{state}")


def port_busy(port, run=subprocess.run):
    """fuser exits 0 when some process has the device open."""
    device = os.path.realpath(port)
    result = run(["fuser", device], capture_output=True, text=True)
    return result.returncode == 0, (result.stdout + result.stderr).strip()


def open_port(port):
    import serial

    return serial.Serial(port, 115200, parity=serial.PARITY_EVEN, timeout=0.5, write_timeout=0.2,
                         dsrdtr=False, rtscts=False)


def open_slow_port(port):
    import serial

    return serial.Serial(port, 50, parity=serial.PARITY_NONE, write_timeout=2, dsrdtr=False, rtscts=False)


def ask(link, request):
    link.reset_input_buffer()
    link.write(request)
    return link.read_until(b"#").decode("ascii", "replace").strip()


def stm32flash(run, *args):
    cmd = ["stm32flash", "-b", BAUD, *args]
    print("$", " ".join(cmd), flush=True)
    return run(cmd).returncode


def break_pattern(link, stop, sleep=time.sleep, clock=time.monotonic):
    """Zero bytes at 50 baud until stop() is true. The UART times the pattern itself; the
    loop only keeps two or three bytes queued, so no gap opens between them."""
    link.write(b"\x00" * 3)
    due = clock()
    while not stop():
        due += BYTE_S
        link.write(b"\x00")
        sleep(max(0.0, due - clock()))


def break_escape(port, open_slow=open_slow_port, wait_for_owner=input, sleep=time.sleep):
    link = open_slow(port)
    stop = threading.Event()
    sender = threading.Thread(target=break_pattern, args=(link, stop.is_set, sleep))
    sender.start()
    try:
        wait_for_owner("Break pattern on the cable. Power the base OFF, wait 2 s, power it ON "
                       "(keep the Pi on), wait 3 s, then press Enter ")
    finally:
        stop.set()
        sender.join()
        link.close()
    # The bytes still queued (up to 0.6 s), then stage1's 300 ms of quiet line.
    sleep(1.5)


def update(path, port=PORT, rescue=False, run=subprocess.run, connect=open_port, sleep=time.sleep,
           force=False, escape=False, open_slow=open_slow_port, wait_for_owner=input):
    data = open(path, "rb").read()
    problem = image.check(data)
    if problem:
        fail(f"{path}: {problem}", UNTOUCHED)
    want = image.version(data)
    print(f"{path}: {len(data)} bytes, will report {want}")

    busy, who = port_busy(port, run)
    if busy:
        fail(f"{port} is in use ({who}). Stop ROS and every drive program first.", UNTOUCHED)

    in_bootloader_already = rescue or escape
    if escape:
        break_escape(port, open_slow, wait_for_owner, sleep)
    elif not rescue:
        link = connect(port)
        try:
            before = ask(link, b"V#")
            if not before.startswith("Vversion:"):
                fail(f"no V# reply ({before!r}): the old firmware (no U#, needs the jumper), or an "
                     "application that does not listen (use --break), or the chip already waits in "
                     "the bootloader. " + FRESH_START, UNTOUCHED)
            print(f"running now: {before}")
            for _ in range(3):
                link.write(b"S#")
            sleep(0.5)  # 0.12 m/s at 0.5 m/s^2 stops in 0.24 s
            answer = ask(link, b"Uforce#" if force else b"U#")
            if answer != "Uok#":
                fail(f"U# refused ({answer!r}): a wheel moves or a drive command is active. If the wheels "
                     "really stand still (an encoder counting by itself), --force skips the wheel check.",
                     UNTOUCHED)
        finally:
            link.close()
        sleep(0.5)  # restart and stage1, then the bootloader listens

    # Address and length: stm32flash then erases only the pages the image covers, from
    # page 4 on. Never -S 0x08000000 here: from page 0 to the end it mass-erases stage1.
    if stm32flash(run, "-w", path, "-v", "-S", f"{APP_ADDRESS}:{len(data)}", port) != 0:
        if escape:
            state = ("The break escape did not reach the bootloader, or stm32flash failed in it. The "
                     "application, if it was whole, was not touched: a power cycle starts it. " + FRESH_START)
        elif rescue:
            state = IN_BOOTLOADER
        else:
            state = NO_BOOTLOADER + "\n" + IN_BOOTLOADER
        fail("stm32flash write/verify failed.", state)

    with tempfile.TemporaryDirectory() as tmp:
        readback = os.path.join(tmp, "readback.bin")
        rc = stm32flash(run, "-r", readback, "-S", f"{APP_ADDRESS}:{len(data)}", port)
        if rc != 0 or not os.path.exists(readback):
            fail("stm32flash read-back failed.", IN_BOOTLOADER)
        if open(readback, "rb").read() != data:
            fail("read-back differs from app.bin.", IN_BOOTLOADER)
    print("read-back matches app.bin")

    if stm32flash(run, "-g", STAGE1_ADDRESS, port) != 0:
        fail("stm32flash Go failed.", GO_FAILED)

    sleep(1.0)  # stage1's CRC check and the application's start
    link = connect(port)
    try:
        got = ask(link, b"V#")
    finally:
        link.close()
    if got != want:
        fail(f"after the update V# gives {got!r}, expected {want!r}"
             + (" (came from the bootloader path)" if in_bootloader_already else "") + ".",
             "The image was written and read back correctly. Power the base off and on and ask V# "
             "again; if it stays silent, the chip waits in the bootloader: " + FRESH_START)
    print(f"OK: the base runs {got}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("app_bin")
    ap.add_argument("--port", default=PORT)
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--rescue", action="store_true",
                      help="send nothing first: the chip already waits in the ROM bootloader")
    mode.add_argument("--break", dest="escape", action="store_true",
                      help="break escape: pattern on the cable while the owner power-cycles the base")
    mode.add_argument("--force", action="store_true",
                      help="send Uforce#: skip the wheel-speed check (still needs zero PWM, no drive command)")
    args = ap.parse_args(argv)
    try:
        update(args.app_bin, args.port, args.rescue, force=args.force, escape=args.escape)
    except Failed as e:
        print(e)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
