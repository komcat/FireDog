"""Generate the recipe COMMAND icons into assets/recipe_icons/.

    python tools/icons/make_command_icons.py                   # write the PNGs
    python tools/icons/make_command_icons.py --preview out.png # also a scaled sheet

Every icon is a 32x32 TILE: a rounded square in its command CATEGORY's colour
(the `category` a CommandSpec registers under; Structure for group / parallel /
loop / if / sub_recipe / stop_recipe) carrying a white glyph outlined in a dark
shade of that colour. The tile says the family, the glyph says the command - so
a long step tree can be scanned by colour before anybody reads a glyph.

Accents mean one thing each: yellow = the point of interest, red = stop / heat,
green = ok / true, cyan = glass / fluid.

The file stems are prefixed by category (st_, mo_, io_, pn_, in_, vi_, al_,
rg_, gr_, nd_, tr_, du_, sy_) and mapped to command names in ONE place,
StepIconFile (src/ui/Widgets.cpp). The toolbar icons (tb_*) are a different
set - make_toolbar_icons.py - and this script never writes them.

Drawn from primitives with no anti-aliasing; stdlib only (pngio.py beside
this). Edit a glyph function and re-run.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pngio import write_rgba  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                   "assets", "recipe_icons")

N = 32

# ---- palette ---------------------------------------------------------------
W = (248, 248, 248)      # glyph main
L = (190, 198, 210)      # glyph shade
D = (38, 42, 50)         # dark detail drawn ON a glyph
Y = (255, 206, 46)       # accent: attention / the point of interest
R = (234, 62, 54)        # accent: stop / heat / laser
G = (92, 222, 104)       # accent: ok / true
C = (96, 214, 255)       # accent: glass / fluid

CATS = {                 # tile colour per command category
    "Structure":    (126, 82, 216),
    "Motion":       (38, 108, 222),
    "IO":           (226, 116, 22),
    "Pneumatic":    (22, 150, 176),
    "Instruments":  (20, 146, 118),
    "Vision":       (58, 150, 52),
    "Align":        (186, 146, 8),
    "Registration": (196, 58, 154),
    "Grip":         (176, 92, 54),
    "Needle":       (204, 48, 60),
    "Tray":         (76, 70, 196),
    "DUT":          (128, 98, 64),
    "System":       (88, 98, 116),
}


def scale(c, f):
    return tuple(max(0, min(255, int(round(v * f)))) for v in c)


class Ic:
    def __init__(self):
        self.g = [[None] * N for _ in range(N)]

    # pixel-index primitives (inclusive)
    def px(self, x, y, c):
        if 0 <= x < N and 0 <= y < N:
            self.g[y][x] = c

    def rect(self, x0, y0, x1, y1, c):
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                self.px(x, y, c)

    def frame(self, x0, y0, x1, y1, c, t=2):
        self.rect(x0, y0, x1, y0 + t - 1, c)
        self.rect(x0, y1 - t + 1, x1, y1, c)
        self.rect(x0, y0, x0 + t - 1, y1, c)
        self.rect(x1 - t + 1, y0, x1, y1, c)

    # continuous primitives: pixel (x,y) covers [x,x+1)x[y,y+1)
    def _each(self, test, c):
        for y in range(N):
            for x in range(N):
                if test(x + 0.5, y + 0.5):
                    self.g[y][x] = c

    def disc(self, cx, cy, r, c):
        self._each(lambda x, y: (x - cx) ** 2 + (y - cy) ** 2 <= r * r, c)

    def ring(self, cx, cy, ro, ri, c, a0=None, a1=None):
        def t(x, y):
            d2 = (x - cx) ** 2 + (y - cy) ** 2
            if not (ri * ri < d2 <= ro * ro):
                return False
            if a0 is None:
                return True
            a = math.degrees(math.atan2(-(y - cy), x - cx)) % 360
            return (a - a0) % 360 <= (a1 - a0) % 360
        self._each(t, c)

    def line(self, x0, y0, x1, y1, c, w=3.0):
        dx, dy = x1 - x0, y1 - y0
        ll = dx * dx + dy * dy or 1e-9
        r = w / 2.0

        def t(x, y):
            u = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / ll))
            px, py = x0 + u * dx, y0 + u * dy
            return (x - px) ** 2 + (y - py) ** 2 <= r * r
        self._each(t, c)

    def lines(self, pts, c, w=3.0):
        for a, b in zip(pts, pts[1:]):
            self.line(a[0], a[1], b[0], b[1], c, w)

    def poly(self, pts, c):
        def t(x, y):
            inside = False
            j = len(pts) - 1
            for i in range(len(pts)):
                xi, yi = pts[i]
                xj, yj = pts[j]
                if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
                    inside = not inside
                j = i
            return inside
        self._each(t, c)

    def arrow(self, x0, y0, x1, y1, c, w=3.0, head=6.0, half=None):
        """Shaft from (x0,y0), head TIP at (x1,y1)."""
        half = half if half is not None else head * 0.62
        L_ = math.hypot(x1 - x0, y1 - y0)
        ux, uy = (x1 - x0) / L_, (y1 - y0) / L_
        bx, by = x1 - ux * head, y1 - uy * head
        self.line(x0, y0, bx + ux * 0.5, by + uy * 0.5, c, w)
        self.poly([(x1, y1), (bx - uy * half, by + ux * half),
                   (bx + uy * half, by - ux * half)], c)

    def mirror_x(self):
        for row in self.g:
            for x in range(N // 2):
                if row[x] is not None and row[N - 1 - x] is None:
                    row[N - 1 - x] = row[x]


# ---- tile + outline ---------------------------------------------------------
def render(glyph, cat):
    base = CATS[cat]
    edge = scale(base, 0.50)
    ink = scale(base, 0.30)                  # glyph outline
    out = [[(0, 0, 0, 0)] * N for _ in range(N)]
    rad = 4.0
    for y in range(N):
        for x in range(N):
            # rounded-square mask
            cx = min(max(x + 0.5, rad), N - rad)
            cy = min(max(y + 0.5, rad), N - rad)
            d = math.hypot(x + 0.5 - cx, y + 0.5 - cy)
            if d > rad:
                continue
            f = 1.16 - 0.30 * (y / (N - 1))      # soft top-lit gradient
            col = scale(base, f)
            if d > rad - 1.0 or x in (0, N - 1) or y in (0, N - 1):
                col = edge
            elif y == 1:
                col = scale(base, 1.40)
            out[y][x] = col + (255,)
    g = glyph.g
    for y in range(N):
        for x in range(N):
            if g[y][x] is not None:
                continue
            near = any(0 <= x + i < N and 0 <= y + j < N and g[y + j][x + i] is not None
                       for i in (-1, 0, 1) for j in (-1, 0, 1))
            if near and out[y][x][3]:
                out[y][x] = ink + (255,)
    for y in range(N):
        for x in range(N):
            if g[y][x] is not None and out[y][x][3]:
                out[y][x] = g[y][x] + (255,)
    return out


# ---- glyphs -----------------------------------------------------------------
ICONS = []          # (cat, file, commands, draw)


def icon(cat, name, cmds):
    def deco(fn):
        ICONS.append((cat, name, cmds, fn))
        return fn
    return deco


# Structure ------------------------------------------------------------------
@icon("Structure", "st_group", "group")
def _(g):
    g.rect(11, 5, 13, 6, W); g.rect(9, 6, 10, 14, W); g.rect(6, 15, 8, 16, W)
    g.rect(9, 17, 10, 25, W); g.rect(11, 25, 13, 26, W)
    g.mirror_x()
    for x in (12, 15, 18):
        g.rect(x, 15, x + 1, 16, Y)


@icon("Structure", "st_parallel", "parallel")
def _(g):
    for y, end in ((8, 27), (16, 27), (24, 27)):
        g.arrow(5, y, end, y, W, w=2.0, head=5, half=3.2)


@icon("Structure", "st_loop", "loop")
def _(g):
    cx, cy, ro, ri = 16, 16.5, 11.0, 7.0
    g.ring(cx, cy, ro, ri, W, 75, 5)
    a = math.radians(5)
    rm = (ro + ri) / 2
    bx, by = cx + rm * math.cos(a), cy - rm * math.sin(a)
    tx, ty = -math.sin(a), -math.cos(a)          # CCW tangent, screen coords
    nx, ny = math.cos(a), -math.sin(a)
    g.poly([(bx + tx * 7, by + ty * 7), (bx + nx * 6, by + ny * 6),
            (bx - nx * 6, by - ny * 6)], W)


@icon("Structure", "st_if", "if")
def _(g):
    g.line(16, 1, 16, 5, W, 2)
    g.poly([(16, 3), (22.5, 9.5), (16, 16), (9.5, 9.5)], W)
    g.arrow(13, 14, 6, 27, G, w=3, head=6, half=4)
    g.arrow(19, 14, 26, 27, R, w=3, head=6, half=4)

@icon("Structure", "st_sub_recipe", "sub_recipe")
def _(g):
    g.frame(11, 4, 27, 27, W, 2)
    for y in (9, 14, 19):
        g.rect(16, y, 24, y + 1, L)
    g.arrow(3, 16, 18, 16, Y, w=3, head=6, half=4.5)


@icon("Structure", "st_stop_recipe", "stop_recipe")
def _(g):
    r = 12.5
    g.poly([(16 + r * math.cos(math.radians(22.5 + 45 * k)),
             16 + r * math.sin(math.radians(22.5 + 45 * k))) for k in range(8)], R)
    g.rect(9, 14, 22, 17, W)


# Motion ---------------------------------------------------------------------
def pin(g, cx, top, c, r=6.5, hole=True):
    cy = top + r
    g.disc(cx, cy, r, c)
    g.poly([(cx - r * 0.8, cy + r * 0.55), (cx + r * 0.8, cy + r * 0.55), (cx, cy + r * 2.0)], c)
    if hole:
        g.disc(cx, cy, 2.6, D)


@icon("Motion", "mo_move_to", "move_to_position, move_to_position_*, move_axis_to, move_lift_off")
def _(g):
    pin(g, 21, 3, W)
    g.arrow(3, 25.5, 17, 25.5, Y, w=3, head=6, half=4.5)


@icon("Motion", "mo_move_relative", "move_relative, move_relative_global")
def _(g):
    g.ring(9, 23, 5.0, 2.2, L)
    g.arrow(12.5, 19.5, 27, 5, W, w=3.2, head=8, half=5)


@icon("Motion", "mo_move_path", "move_path")
def _(g):
    pts = [(6, 25), (11, 11), (20, 21), (26, 7)]
    g.lines(pts, L, 2.4)
    for x, y in pts:
        g.disc(x, y, 3.0, W)
    g.disc(26, 7, 3.4, Y)


@icon("Motion", "mo_scan", "xcan, xcan_to_target, xcan_two_lens, raster_scan, climb_raster")
def _(g):
    ys = (6, 12, 18, 24)
    pts = []
    for i, y in enumerate(ys):
        a, b = (6, 25) if i % 2 == 0 else (25, 6)
        pts += [(a, y), (b, y)]
    g.lines(pts, L, 2.0)
    g.disc(17, 15, 4.2, Y)
    g.disc(17, 15, 1.6, W)


@icon("Motion", "mo_read_position", "read_position, check_position")
def _(g):
    g.arrow(7, 26, 7, 3, W, w=2.4, head=5, half=3.5)
    g.arrow(6, 25, 29, 25, W, w=2.4, head=5, half=3.5)
    for y in range(11, 23, 3):
        g.rect(19, y, 19, y + 1, L)
    for x in range(9, 18, 3):
        g.rect(x, 12, x + 1, 12, L)
    g.disc(19.5, 12.5, 3.6, Y)


@icon("Motion", "mo_velocity", "set_velocity, read_velocity")
def _(g):
    g.ring(16, 21, 12, 8.5, W, 0, 180)
    a = math.radians(40)
    g.line(16, 21, 16 + 9.5 * math.cos(a), 21 - 9.5 * math.sin(a), Y, 3.0)
    g.disc(16, 21, 3, W)
    g.rect(4, 22, 27, 23, W)

@icon("Motion", "mo_stop", "stop_motion")
def _(g):
    g.rect(14, 8, 26, 23, R)
    g.rect(16, 10, 24, 21, scale(R, 1.15))
    for y, x0 in ((10, 6), (15, 3), (20, 6)):
        g.rect(x0, y, 11, y + 1, W)


@icon("Motion", "mo_save_position", "save_position")
def _(g):
    pin(g, 13, 4, W)
    g.rect(19, 21, 28, 24, G); g.rect(22, 17, 25, 28, G)


# IO -------------------------------------------------------------------------
@icon("IO", "io_output", "set_output, toggle_output, clear_latch")
def _(g):
    g.disc(10.5, 16, 7.5, W); g.disc(21.5, 16, 7.5, W); g.rect(10, 9, 21, 22, W)
    g.disc(21.5, 16, 5, D)
    g.disc(21.5, 16, 3.2, Y)


@icon("IO", "io_pulse", "pulse_output")
def _(g):
    g.lines([(3, 23), (10, 23), (10, 9), (21, 9), (21, 23), (29, 23)], W, 3.0)


@icon("IO", "io_wait_input", "wait_input")
def _(g):
    g.ring(9, 16, 6.5, 3.0, W)
    g.disc(9, 16, 3.0, Y)
    g.arrow(28, 16, 16.5, 16, W, w=3, head=6, half=4.5)


# Pneumatic ------------------------------------------------------------------
def cylinder(g):
    g.rect(3, 12, 16, 25, W)
    g.rect(5, 14, 14, 23, L)
    g.rect(9, 14, 10, 23, W)          # piston


@icon("Pneumatic", "pn_extend", "extend_slide")
def _(g):
    cylinder(g)
    g.rect(17, 17, 26, 20, L); g.rect(26, 13, 28, 24, W)
    g.arrow(9, 5, 28, 5, Y, w=2.6, head=5, half=3.6)


@icon("Pneumatic", "pn_retract", "retract_slide")
def _(g):
    cylinder(g)
    g.rect(17, 17, 18, 20, L); g.rect(19, 13, 21, 24, W)
    g.arrow(28, 5, 7, 5, Y, w=2.6, head=5, half=3.6)


# Instruments ----------------------------------------------------------------
@icon("Instruments", "in_meter", "keithley_*")
def _(g):
    g.rect(3, 6, 28, 26, W)
    g.ring(15.5, 22, 10, 8.6, D, 30, 150)
    a = math.radians(62)
    g.line(15.5, 22, 15.5 + 11 * math.cos(a), 22 - 11 * math.sin(a), R, 2.0)
    g.disc(15.5, 22, 2.2, D)


@icon("Instruments", "in_laser", "cld_laser, cld_*_laser_current")
def _(g):
    g.rect(3, 11, 12, 21, W); g.rect(12, 14, 14, 18, L)
    g.line(14, 16, 24, 16, R, 2.0)
    for a in range(0, 360, 45):
        r = math.radians(a)
        g.line(25, 16, 25 + 5 * math.cos(r), 16 + 5 * math.sin(r), Y, 1.6)
    g.disc(25, 16, 2.2, W)


@icon("Instruments", "in_psu", "psu_*")
def _(g):
    g.poly([(20, 2), (7, 18), (15, 18), (11, 30), (25, 13), (17, 13), (22, 2)], Y)


@icon("Instruments", "in_light", "light_*")
def _(g):
    g.disc(16, 12, 8.5, Y)
    g.rect(11, 18, 20, 21, Y)
    g.rect(12, 22, 19, 27, L)
    g.rect(12, 23, 19, 23, D); g.rect(12, 25, 19, 25, D)
    g.rect(14, 10, 17, 11, W)


@icon("Instruments", "in_temperature", "cld_*temperature, cld_tec, wait_for_temperature")
def _(g):
    g.disc(15.5, 6.5, 3.5, W); g.rect(12, 6, 18, 20, W)
    g.disc(15.5, 23.5, 6, W)
    g.disc(15.5, 23.5, 4, R); g.rect(14, 9, 16, 21, R)
    for y in (8, 12, 16):
        g.rect(21, y, 25, y + 1, W)


# Vision ---------------------------------------------------------------------
@icon("Vision", "vi_camera", "capture_view")
def _(g):
    g.rect(3, 10, 28, 26, W); g.rect(9, 6, 17, 10, W)
    g.disc(15.5, 18, 6.5, D)
    g.disc(15.5, 18, 4.0, C)
    g.rect(13, 15, 14, 16, W)
    g.rect(22, 12, 25, 13, Y)


@icon("Vision", "vi_focus", "auto_focus")
def _(g):
    for sx, sy in ((4, 4), (21, 4), (4, 21), (21, 21)):
        x0 = sx if sx < 16 else sx + 3
        y0 = sy if sy < 16 else sy + 3
        g.rect(sx, y0, sx + 6, y0 + 2, W)
        g.rect(x0, sy, x0 + 2, sy + 6, W)
    g.disc(16, 16, 4.0, Y)


@icon("Vision", "vi_detect", "vision_detect, vision_test, pick_point, halcon_status")
def _(g):
    g.line(19, 19, 27, 27, W, 4.2)
    g.ring(13, 13, 9.5, 6.0, W)
    g.disc(13, 13, 6.0, scale(CATS["Vision"], 0.55))
    g.disc(13, 13, 2.8, Y)


@icon("Vision", "vi_mosaic", "mosaic_capture, mosaic_calibrate")
def _(g):
    for i, (x, y) in enumerate(((3, 3), (16, 3), (3, 16), (16, 16))):
        g.rect(x, y, x + 12, y + 12, W if i in (0, 3) else L)
    g.rect(13, 13, 18, 18, Y)


# Align ----------------------------------------------------------------------
@icon("Align", "al_align", "align_run, align_station, align_solve, align_apply, move_local")
def _(g):
    g.ring(16, 16, 9.5, 6.5, W)
    g.rect(15, 2, 16, 10, W); g.rect(15, 22, 16, 29, W)
    g.rect(2, 15, 10, 16, W); g.rect(22, 15, 29, 16, W)
    g.rect(15, 15, 16, 16, Y)


@icon("Align", "al_nominal", "nominal_point, nominal_view, move_to_nominal")
def _(g):
    o = (9, 26)
    a = math.radians(12)
    xd = (math.cos(a), -math.sin(a)); yd = (-math.sin(a), -math.cos(a))
    g.arrow(o[0], o[1], o[0] + 19 * xd[0], o[1] + 19 * xd[1], W, 2.4, 5, 3.5)
    g.arrow(o[0], o[1], o[0] + 19 * yd[0], o[1] + 19 * yd[1], W, 2.4, 5, 3.5)
    g.poly([(20, 6), (25.5, 11.5), (20, 17), (14.5, 11.5)], Y)

# Registration ---------------------------------------------------------------
@icon("Registration", "rg_device_reg", "device_reg*, move_tool_to_camera, move_camera_to_tool, "
                                       "move_to_point, tool_travel_to_camera")
def _(g):
    g.rect(3, 5, 14, 14, W); g.rect(5, 3, 9, 5, W)
    g.disc(8.5, 9.5, 2.8, D)
    g.poly([(17, 18), (29, 18), (23, 29.5)], W)
    g.disc(16.5, 16.5, 1.8, Y); g.disc(20.5, 12.5, 1.5, Y); g.disc(12.5, 20.5, 1.5, Y)

# Grip -----------------------------------------------------------------------
@icon("Grip", "gr_grip", "grip_*")
def _(g):
    g.rect(13, 2, 18, 4, L)
    g.rect(4, 5, 27, 9, W)
    g.rect(6, 10, 8, 26, W); g.rect(9, 18, 10, 26, W)
    g.mirror_x()
    g.disc(15.5, 22, 4.3, Y)

# Needle ---------------------------------------------------------------------
@icon("Needle", "nd_needle", "dispense_run, needle_cal_run, needle_offset*")
def _(g):
    g.rect(10, 2, 21, 3, W); g.rect(15, 4, 16, 7, W)
    g.rect(11, 8, 20, 19, W)
    g.rect(13, 13, 18, 18, C)
    for y in (10, 13, 16):
        g.rect(19, y, 20, y, L)
    g.rect(13, 20, 18, 21, L)
    g.rect(15, 22, 16, 25, L)
    g.disc(15.75, 28, 2.1, Y)


# Tray -----------------------------------------------------------------------
@icon("Tray", "tr_tray", "move_to_cell, move_to_active_cell, tray_*")
def _(g):
    cells = (4, 13, 22)
    for j, y in enumerate(cells):
        for i, x in enumerate(cells):
            k = j * 3 + i
            col = L if k < 3 else W
            if (i, j) == (1, 1):
                col = Y
            g.rect(x, y, x + 5, y + 5, col)


# DUT ------------------------------------------------------------------------
@icon("DUT", "du_dut", "dut_open, dut_close, dut_set")
def _(g):
    for p in (11, 15, 19):
        g.rect(4, p, 8, p + 1, L); g.rect(23, p, 27, p + 1, L)
        g.rect(p, 4, p + 1, 8, L); g.rect(p, 23, p + 1, 27, L)
    g.rect(9, 9, 22, 22, W)
    g.disc(12.5, 12.5, 1.6, D)


@icon("DUT", "du_record", "dut_record, dut_record_all")
def _(g):
    g.rect(6, 5, 25, 29, W)
    g.rect(11, 2, 20, 7, L); g.rect(14, 3, 17, 4, D)
    for y in (11, 16, 21, 25):
        g.rect(9, y, 10, y + 1, G)
        g.rect(13, y, 22 if y != 25 else 18, y + 1, D)


# System ---------------------------------------------------------------------
@icon("System", "sy_delay", "delay")
def _(g):
    g.disc(16, 16, 12, W)
    g.ring(16, 16, 12, 10.6, L)
    for a in range(0, 360, 90):
        r = math.radians(a)
        g.px(int(16 + 8.5 * math.cos(r) - 0.5 + (1 if a == 0 else 0)), int(16 - 8.5 * math.sin(r) - 0.5 + (1 if a == 270 else 0)), D)
    g.line(16, 16, 16, 8, D, 2.0)
    g.line(16, 16, 22, 16, D, 2.0)
    g.disc(16, 16, 1.6, R)


@icon("System", "sy_wait_until", "wait_until")
def _(g):
    g.rect(7, 3, 24, 5, W); g.rect(7, 26, 24, 28, W)
    g.poly([(9, 6), (23, 6), (17, 16), (15, 16)], W)
    g.poly([(15, 15.5), (17, 15.5), (23, 26), (9, 26)], W)
    g.poly([(12, 9), (20, 9), (16.5, 14), (15.5, 14)], Y)
    g.poly([(16, 20), (21, 26), (11, 26)], Y)
    g.rect(15, 15, 16, 20, Y)


@icon("System", "sy_log", "log")
def _(g):
    g.rect(5, 3, 22, 28, W)
    for y, x1 in ((7, 18), (11, 18), (15, 15), (19, 18), (23, 12)):
        g.rect(8, y, x1, y + 1, D)
    g.line(26, 11, 17, 25, Y, 3.4)
    g.poly([(15.4, 24), (18.6, 26), (15, 28.5)], L)


@icon("System", "sy_prompt", "prompt, prompt_input")
def _(g):
    g.rect(3, 3, 28, 22, W)
    g.poly([(7, 22), (15, 22), (7, 29)], W)
    q = [".XXX.", "X...X", "....X", "...X.", "..X..", ".....", "..X.."]
    for j, row in enumerate(q):
        for i, ch in enumerate(row):
            if ch == "X":
                g.rect(11 + 2 * i, 5 + 2 * j, 12 + 2 * i, 6 + 2 * j, D)

@icon("System", "sy_notify", "notify_operator")
def _(g):
    g.rect(15, 3, 16, 5, Y)
    g.disc(15.5, 12, 6.5, Y)
    g.rect(9, 12, 22, 19, Y)
    g.rect(6, 20, 25, 22, Y)
    g.disc(15.5, 25, 2.6, W)
    g.line(26, 6, 29, 3, W, 1.6); g.line(27, 11, 30, 11, W, 1.6)
    g.line(5, 6, 2, 3, W, 1.6); g.line(4, 11, 1, 11, W, 1.6)


@icon("System", "sy_script", "run_script")
def _(g):
    g.rect(3, 5, 28, 26, W)
    g.rect(5, 9, 26, 24, D)
    g.rect(4, 6, 27, 7, L)
    g.lines([(8, 12), (13, 16), (8, 20)], G, 2.6)
    g.rect(15, 19, 22, 20, G)


@icon("System", "sy_variable", "ctx_set, read_topic")
def _(g):
    g.line(4, 10, 13, 22, W, 3.2); g.line(13, 10, 4, 22, W, 3.2)
    g.rect(17, 12, 27, 14, Y); g.rect(17, 18, 27, 20, Y)


@icon("System", "sy_manual_adjust", "manual_adjust")
def _(g):
    for x1, y1 in ((16, 2), (16, 30), (2, 16), (30, 16)):
        g.arrow(16, 16, x1, y1, W, w=3.0, head=6, half=4.5)
    g.disc(16, 16, 3.5, Y)


# The same glyph on another category's tile: the tile must always say the
# category, so a camera step outside Vision gets its own copy.
def alias(cat, name, cmds, src):
    fn = next(f for _, n, _, f in ICONS if n == src)
    ICONS.append((cat, name, cmds, fn))


alias("Instruments", "in_camera", "camera_exposure", "vi_camera")
alias("DUT", "du_capture", "dut_capture_image", "vi_camera")


def main():
    os.makedirs(OUT, exist_ok=True)
    tiles = []
    for cat, name, cmds, fn in ICONS:
        g = Ic()
        fn(g)
        px = render(g, cat)
        tiles.append((cat, px))
        path = os.path.normpath(os.path.join(OUT, name + ".png"))
        write_rgba(path, N, N, px)
        print("wrote", path, "  <-", cmds)

    if "--preview" in sys.argv:
        # One row per category, 6x, nearest-neighbour, on a dark UI grey.
        i = sys.argv.index("--preview")
        path = sys.argv[i + 1] if len(sys.argv) > i + 1 else "command_icons_preview.png"
        s, gap = 6, 12
        cats = list(dict.fromkeys(c for c, _ in tiles))
        per = max(sum(1 for c, _ in tiles if c == k) for k in cats)
        Wd, Hd = gap + per * (N * s + gap), gap + len(cats) * (N * s + gap)
        bg = (30, 30, 32, 255)
        sheet = [[bg] * Wd for _ in range(Hd)]
        col = {k: 0 for k in cats}
        for cat, px in tiles:
            ox = gap + col[cat] * (N * s + gap)
            oy = gap + cats.index(cat) * (N * s + gap)
            col[cat] += 1
            for y in range(N):
                for x in range(N):
                    p = px[y][x]
                    if not p[3]:
                        continue
                    for yy in range(s):
                        row = sheet[oy + y * s + yy]
                        for xx in range(s):
                            row[ox + x * s + xx] = p
        write_rgba(path, Wd, Hd, sheet)
        print("preview", path)


if __name__ == "__main__":
    main()
