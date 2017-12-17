#!/usr/bin/env python3

import colorsys
import select
import serial
import sys
import termios
import tty

if len(sys.argv) == 1:
    print("USAGE: {} DEVICE".format(sys.argv[0]))
    sys.exit(1)

# Disable line buffering
old_attr = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin)
try:
    port = serial.Serial(port=sys.argv[1], baudrate=115200)

    def send(payload):
        checksum = (len(payload) + sum(payload)) & 0xff
        port.write(bytes([0x5a, len(payload)] + payload + [checksum]))

    # Hide cursor
    print("\x1b[?25l" + "Keybox test")
    print("Press 1 to open left solenoid, 2 to open right solenoid, Ctrl-C to exit.")

    a = 0
    while True:
        read, _, _ = select.select([sys.stdin, port], [], [], 0.01)

        if port in read:
            r = port.read()
            print("\rStatus bits: " + bin(r[0]).replace("0b", "").zfill(8) + "\r", end="")

        if sys.stdin in read:
            r = sys.stdin.read(1)
            if r == "1":
                send([0x02, 0xe8, 0x03])
            elif r == "2":
                send([0x12, 0xe8, 0x03])

        # Run a color sweep light show on the LEDs

        color = list(map(lambda x: int(x*255), colorsys.hls_to_rgb(a/512, 0.03, 1)))
        send([0x01] + color)

        color = list(map(lambda x: int(x*255), colorsys.hls_to_rgb(((a + 256) % 512)/512, 0.03, 1)))
        send([0x11] + color)

        a += 1
        if a == 512:
            a = 0
except KeyboardInterrupt:
    pass
finally:
    # Reset previous terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
    print("\x1b[?25h")