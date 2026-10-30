"""Convert an X11 `xwd -id <window>` dump (ZPixmap, 24/32 bits per pixel TrueColor) to PNG. Usage: xwd2png.py in.xwd out.png"""
import struct
import sys

import cv2
import numpy as np


def read_xwd(path):
    b = open(path, "rb").read()
    h = struct.unpack(">25I", b[:100])
    header_size, width, height = h[0], h[4], h[5]
    bits_per_pixel, bytes_per_line, ncolors = h[11], h[12], h[19]
    byte_order = h[7]
    if bits_per_pixel not in (24, 32):
        raise SystemExit(f"unsupported bpp {bits_per_pixel}")
    n = bits_per_pixel // 8
    off = header_size + ncolors * 12
    raw = np.frombuffer(b[off:off + bytes_per_line * height], np.uint8).reshape(height, bytes_per_line)
    px = raw[:, :width * n].reshape(height, width, n)
    return px[..., :3] if byte_order == 0 else px[..., n - 3:][..., ::-1]   # LSBFirst: B,G,R(,X) = OpenCV BGR


if __name__ == "__main__":
    cv2.imwrite(sys.argv[2], read_xwd(sys.argv[1]))
