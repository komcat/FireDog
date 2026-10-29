"""Minimal PNG read/write (stdlib only) for the icon generator.

RGBA 8-bit, no Pillow: the build machine has a plain Python and the icons are
32x32, so a hand-rolled encoder is the smaller dependency.
"""
import struct
import zlib


def _chunk(tag, data):
    c = struct.pack(">I", len(data)) + tag + data
    return c + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)


def write_rgba(path, w, h, px):
    """px: list of h rows, each a list of w (r, g, b, a) tuples."""
    raw = bytearray()
    for row in px:
        raw.append(0)   # filter: none
        for r, g, b, a in row:
            raw += bytes((r, g, b, a))
    data = b"\x89PNG\r\n\x1a\n"
    data += _chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0))
    data += _chunk(b"IDAT", zlib.compress(bytes(raw), 9))
    data += _chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(data)


def read_rgba(path):
    """-> (w, h, rows of (r, g, b, a)). Handles RGB and RGBA, 8-bit, all filters."""
    d = open(path, "rb").read()
    i, idat = 8, b""
    w = h = ct = 0
    while i + 8 <= len(d):
        n = struct.unpack(">I", d[i:i + 4])[0]
        tag, c = d[i + 4:i + 8], d[i + 8:i + 8 + n]
        if tag == b"IHDR":
            w, h, _, ct = struct.unpack(">IIBB", c[:10])
        elif tag == b"IDAT":
            idat += c
        elif tag == b"IEND":
            break
        i += 12 + n
    bpp = {6: 4, 2: 3}[ct]
    stride = w * bpp
    raw = zlib.decompress(idat)
    prev, p, rows = bytearray(stride), 0, []
    for _ in range(h):
        f = raw[p]
        line = bytearray(raw[p + 1:p + 1 + stride])
        p += 1 + stride
        for x in range(stride):
            a = line[x - bpp] if x >= bpp else 0
            b = prev[x]
            c = prev[x - bpp] if x >= bpp else 0
            if f == 1:
                line[x] = (line[x] + a) & 255
            elif f == 2:
                line[x] = (line[x] + b) & 255
            elif f == 3:
                line[x] = (line[x] + ((a + b) >> 1)) & 255
            elif f == 4:
                pa, pb, pc = abs(b - c), abs(a - c), abs(a + b - 2 * c)
                pr = a if pa <= pb and pa <= pc else (b if pb <= pc else c)
                line[x] = (line[x] + pr) & 255
        rows.append([tuple(line[x * bpp:(x + 1) * bpp]) + ((255,) if bpp == 3 else ())
                     for x in range(w)])
        prev = line
    return w, h, rows
