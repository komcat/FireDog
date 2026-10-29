"""Generate the editor toolbar icons into assets/recipe_icons/.

    python tools/icons/make_toolbar_icons.py            # write the PNGs
    python tools/icons/make_toolbar_icons.py --preview  # also a scaled sheet

32x32 pixel-art glyphs for the square tool buttons (ui/Widgets
IconToolButton) of the Recipe Editor step toolbar, its recipe list, and the
Program Organizer: tb_move_up, tb_move_down, tb_indent, tb_outdent, tb_delete,
tb_copy, tb_cut, tb_paste, tb_new, tb_duplicate, tb_rename, tb_reload,
tb_show, tb_hide, tb_tag.

Drawn from primitives (rects, polygons, rings) with NO anti-aliasing, then
every glyph gets the same 1 px dark outline - that outline pass is what makes
fifteen shapes from code read as one hand-drawn set. The background is
transparent: the button frame is the tile. Colours come from the existing
recipe icons (move_relative's blue, Stop's red, Save's sheet white).

Stdlib only (pngio.py beside this). Edit a shape here and re-run, or open the
PNG in Piskel and touch it up by hand - the files are plain 32x32 RGBA.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pngio import write_rgba  # noqa: E402

N = 32
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                   "assets", "recipe_icons")


def hexc(s, a=255):
    s = s.lstrip("#")
    return (int(s[0:2], 16), int(s[2:4], 16), int(s[4:6], 16), a)


CLEAR   = (0, 0, 0, 0)
OUTLINE = hexc("#0b0f2a")
BLUE    = hexc("#1f4fe6")
BLUE_HI = hexc("#7c9cf5")
RED     = hexc("#d90707")
RED_HI  = hexc("#ff5a4f")
RED_DK  = hexc("#8f0000")
WHITE   = hexc("#ffffff")
SHEET2  = hexc("#d8e4f0")
LINE    = hexc("#8a94a6")
STEEL   = hexc("#c9ced6")
STEEL_D = hexc("#7d8591")
TAN     = hexc("#b9772f")
TAN_HI  = hexc("#d9a25a")
GOLD    = hexc("#e8c45a")
GREEN   = hexc("#1fa83c")
GREEN_HI = hexc("#7be08f")
ORANGE  = hexc("#f0962a")
ORANGE_HI = hexc("#ffc56b")
PINK    = hexc("#f28fa6")
YELLOW  = hexc("#f5c518")
YELLOW_DK = hexc("#c99400")
GRAPHITE = hexc("#3a3f4b")


class Canvas:
    def __init__(self):
        self.px = [[CLEAR] * N for _ in range(N)]

    def put(self, x, y, c):
        if 0 <= x < N and 0 <= y < N:
            self.px[y][x] = c

    def rect(self, x0, y0, x1, y1, c):
        """Inclusive corners."""
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                self.put(x, y, c)

    def poly(self, pts, c):
        """Fill a polygon: a pixel is in when its centre is (even-odd rule)."""
        ys = [p[1] for p in pts]
        for y in range(int(min(ys)), int(max(ys)) + 1):
            cy = y + 0.5
            xs = []
            for i in range(len(pts)):
                (x0, y0), (x1, y1) = pts[i], pts[(i + 1) % len(pts)]
                if (y0 <= cy < y1) or (y1 <= cy < y0):
                    xs.append(x0 + (cy - y0) * (x1 - x0) / (y1 - y0))
            xs.sort()
            for a, b in zip(xs[0::2], xs[1::2]):
                for x in range(int(a + 0.5), int(b + 0.5)):
                    self.put(x, y, c)

    def bar(self, x0, y0, x1, y1, w, c):
        """A straight stroke of width w from (x0,y0) to (x1,y1)."""
        dx, dy = x1 - x0, y1 - y0
        ln = math.hypot(dx, dy)
        nx, ny = -dy / ln * w / 2, dx / ln * w / 2
        self.poly([(x0 + nx, y0 + ny), (x1 + nx, y1 + ny),
                   (x1 - nx, y1 - ny), (x0 - nx, y0 - ny)], c)

    def arc(self, cx, cy, r_out, r_in, a0, a1, c):
        """Ring between math angles a0..a1 (degrees, counter-clockwise, y up)."""
        for y in range(N):
            for x in range(N):
                px, py = x + 0.5 - cx, cy - (y + 0.5)
                d2 = px * px + py * py
                if not (r_in * r_in <= d2 <= r_out * r_out):
                    continue
                a = math.degrees(math.atan2(py, px)) % 360
                if (a - a0) % 360 <= (a1 - a0) % 360:
                    self.put(x, y, c)

    def stamp(self, other):
        """Lay another (already outlined) canvas over this one."""
        for y in range(N):
            for x in range(N):
                if other.px[y][x][3]:
                    self.px[y][x] = other.px[y][x]

    def ring(self, cx, cy, r_out, r_in, c):
        for y in range(N):
            for x in range(N):
                d2 = (x + 0.5 - cx) ** 2 + (y + 0.5 - cy) ** 2
                if r_in * r_in <= d2 <= r_out * r_out:
                    self.put(x, y, c)

    def outline(self, c=OUTLINE):
        """Every clear pixel touching (8-way) a drawn one becomes the outline."""
        src = [row[:] for row in self.px]
        for y in range(N):
            for x in range(N):
                if src[y][x][3]:
                    continue
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        xx, yy = x + dx, y + dy
                        if 0 <= xx < N and 0 <= yy < N and src[yy][xx][3]:
                            self.px[y][x] = c
                            break
                    else:
                        continue
                    break

    def flip_v(self):
        self.px.reverse()

    def flip_h(self):
        for row in self.px:
            row.reverse()


# ---- Glyphs -----------------------------------------------------------------

def arrow_up():
    c = Canvas()
    c.poly([(16, 3), (28, 15), (21, 15), (21, 28), (11, 28), (11, 15), (4, 15)], BLUE)
    c.poly([(16, 6), (24, 14), (19, 14), (19, 26), (16, 26)], BLUE_HI)   # lit half
    c.outline()
    return c


def arrow_down():
    c = arrow_up()
    c.flip_v()
    return c


def text_lines(c, x0, x1):
    """Three 'step rows' the indent arrows push into / pull out of."""
    for y in (6, 14, 22):
        c.rect(x0, y, x1, y + 3, WHITE)
        c.rect(x0 + 1, y + 1, x1 - 1, y + 2, SHEET2)


def indent():
    c = Canvas()
    text_lines(c, 17, 28)
    c.poly([(3, 12), (11, 12), (11, 7), (17, 15.5), (11, 24), (11, 19), (3, 19)], BLUE)
    c.rect(4, 13, 10, 15, BLUE_HI)
    c.outline()
    return c


def outdent():
    c = Canvas()
    text_lines(c, 3, 14)
    c.poly([(28, 12), (20, 12), (20, 7), (14, 15.5), (20, 24), (20, 19), (28, 19)], BLUE)
    c.rect(21, 13, 27, 15, BLUE_HI)
    c.outline()
    return c


def trash():
    c = Canvas()
    c.rect(12, 3, 19, 5, RED_DK)          # handle
    c.rect(5, 6, 26, 9, RED)              # lid
    c.rect(6, 7, 25, 7, RED_HI)
    c.poly([(7, 11), (25, 11), (23, 29), (9, 29)], RED)   # body
    for x in (11, 15, 19):                # slots
        c.rect(x, 14, x + 1, 26, RED_DK)
    c.rect(8, 12, 9, 26, RED_HI)
    c.outline()
    return c


def sheet(c, x0, y0, x1, y1, fill):
    c.rect(x0, y0, x1, y1, fill)
    for y in range(y0 + 4, y1 - 2, 4):
        c.rect(x0 + 3, y, x1 - 3, y, LINE)


def copy():
    c = Canvas()
    sheet(c, 4, 3, 19, 22, SHEET2)        # behind
    c.outline()
    sheet(c, 12, 10, 27, 29, WHITE)       # front
    # re-outline only the front sheet's edge
    for x in range(11, 29):
        for y in (9, 30):
            c.put(x, y, OUTLINE)
    for y in range(9, 31):
        for x in (11, 28):
            c.put(x, y, OUTLINE)
    return c


def scissors():
    c = Canvas()
    # Two blades that CROSS at the pivot (16, 14): each runs from a tip at the
    # top to the handle on the opposite side, so the pair reads as an X.
    c.poly([(8, 2), (11, 2), (22, 20), (19, 21)], STEEL)
    c.poly([(24, 2), (21, 2), (10, 20), (13, 21)], STEEL)
    c.poly([(9, 2), (10, 2), (16, 12), (15, 12)], STEEL_D)   # edge shading
    c.rect(15, 13, 17, 15, STEEL_D)       # pivot
    # handles
    c.ring(10, 24.5, 6.0, 3.2, RED)
    c.ring(22, 24.5, 6.0, 3.2, RED)
    c.outline()
    return c


def paste():
    c = Canvas()
    c.rect(5, 5, 26, 29, TAN)             # board
    c.rect(6, 6, 7, 28, TAN_HI)
    sheet(c, 9, 9, 22, 27, WHITE)         # paper on it
    c.rect(11, 2, 20, 7, STEEL)           # clip
    c.rect(13, 3, 18, 4, STEEL_D)
    c.rect(14, 6, 17, 7, GOLD)
    c.outline()
    return c


def plus_badge():
    """A green '+' in the lower right, outlined on its own layer."""
    b = Canvas()
    b.rect(20, 16, 24, 29, GREEN)
    b.rect(15, 21, 29, 25, GREEN)
    b.rect(21, 17, 22, 21, GREEN_HI)
    b.rect(16, 22, 20, 23, GREEN_HI)
    b.outline()
    return b


def page(c, x0, y0, x1, y1, fill):
    """A sheet with a folded top-right corner."""
    c.poly([(x0, y0), (x1 - 5, y0), (x1 + 1, y0 + 6), (x1 + 1, y1 + 1), (x0, y1 + 1)], fill)
    c.poly([(x1 - 5, y0), (x1 - 5, y0 + 6), (x1 + 1, y0 + 6)], SHEET2)
    for y in range(y0 + 9, y1 - 1, 4):
        c.rect(x0 + 3, y, x1 - 4, y, LINE)


def new_page():
    c = Canvas()
    page(c, 4, 2, 22, 28, WHITE)
    c.outline()
    c.stamp(plus_badge())
    return c


def duplicate():
    c = Canvas()
    back = Canvas()
    page(back, 3, 1, 17, 21, SHEET2)
    back.outline()
    front = Canvas()
    page(front, 9, 7, 24, 28, WHITE)
    front.outline()
    c.stamp(back)
    c.stamp(front)
    c.stamp(plus_badge())
    return c


def rename():
    c = Canvas()
    field = Canvas()                      # the name field being edited
    field.rect(2, 21, 29, 29, WHITE)
    field.rect(4, 24, 13, 26, LINE)
    field.outline()
    pen = Canvas()                        # a pencil writing into it
    d = (1 / math.sqrt(2), -1 / math.sqrt(2))
    s0, s1 = (12, 19), (24, 7)
    pen.bar(s0[0], s0[1], s1[0], s1[1], 7, YELLOW)
    pen.bar(s0[0] + 1, s0[1] - 2, s1[0] - 1, s1[1] - 4, 2, YELLOW_DK)
    pen.bar(s1[0], s1[1], s1[0] + d[0] * 4, s1[1] + d[1] * 4, 7, PINK)   # eraser
    pen.poly([(s0[0] - 2.5, s0[1] - 2.5), (s0[0] + 2.5, s0[1] + 2.5), (7, 24)], TAN_HI)
    pen.poly([(8.6, 20.6), (10.4, 22.4), (7, 24)], GRAPHITE)              # lead
    pen.outline()
    c.stamp(field)
    c.stamp(pen)
    return c


def reload():
    c = Canvas()
    c.arc(16, 16, 12.5, 8, 75, 15, GREEN)        # gap in the top right
    c.arc(16, 16, 11, 9.5, 100, 200, GREEN_HI)
    # arrowhead at the 15-degree end, pointing the way the arc runs (CCW)
    a = math.radians(15)
    bx, by = 16 + 10.25 * math.cos(a), 16 - 10.25 * math.sin(a)
    tx, ty = -math.sin(a), -math.cos(a)          # travel direction (screen)
    nx, ny = math.cos(a), -math.sin(a)           # outward
    c.poly([(bx + nx * 6.5, by + ny * 6.5), (bx + tx * 7, by + ty * 7),
            (bx - nx * 6.5, by - ny * 6.5)], GREEN)
    c.outline()
    return c


def eye(c):
    c.poly([(2, 16), (8, 9), (16, 7), (24, 9), (30, 16), (24, 23), (16, 25), (8, 23)], WHITE)
    c.ring(16, 16, 6.5, 0, BLUE)
    c.ring(16, 16, 3, 0, OUTLINE)
    c.rect(17, 12, 18, 13, BLUE_HI)


def show():
    c = Canvas()
    eye(c)
    c.outline()
    return c


def hide():
    c = Canvas()
    e = Canvas()
    eye(e)
    for y in range(N):                           # a closed-ish, dimmed eye
        for x in range(N):
            if e.px[y][x] == WHITE:
                e.px[y][x] = SHEET2
    e.outline()
    slash = Canvas()
    slash.bar(5, 28, 27, 4, 4, RED)
    slash.outline()
    c.stamp(e)
    c.stamp(slash)
    return c


def tag():
    c = Canvas()
    a = math.radians(45)

    def rot(x, y):
        x, y = (x - 16) * 0.92, (y - 16) * 0.92
        return (16 + x * math.cos(a) - y * math.sin(a),
                16 + x * math.sin(a) + y * math.cos(a))

    c.poly([rot(*p) for p in [(3, 16), (10, 8), (28, 8), (28, 24), (10, 24)]], ORANGE)
    c.poly([rot(*p) for p in [(11, 10), (26, 10), (26, 12), (11, 12)]], ORANGE_HI)
    c.outline()
    hx, hy = rot(9, 16)
    c.ring(hx, hy, 2.3, 0, OUTLINE)              # the string hole
    return c


ICONS = {
    "tb_move_up":   arrow_up,
    "tb_move_down": arrow_down,
    "tb_indent":    indent,
    "tb_outdent":   outdent,
    "tb_delete":    trash,
    "tb_copy":      copy,
    "tb_cut":       scissors,
    "tb_paste":     paste,
    "tb_new":       new_page,
    "tb_duplicate": duplicate,
    "tb_rename":    rename,
    "tb_reload":    reload,
    "tb_show":      show,
    "tb_hide":      hide,
    "tb_tag":       tag,
}


def main():
    os.makedirs(OUT, exist_ok=True)
    made = {}
    for name, fn in ICONS.items():
        cv = fn()
        made[name] = cv
        write_rgba(os.path.join(OUT, name + ".png"), N, N, cv.px)
        print("wrote", os.path.normpath(os.path.join(OUT, name + ".png")))

    if "--preview" in sys.argv:
        # One row, 8x scale, on a mid-grey frame colour, for eyeballing.
        s, gap = 8, 16
        W = len(made) * (N * s + gap) + gap
        H = N * s + gap * 2
        bg = hexc("#2b3445")
        sheet_px = [[bg] * W for _ in range(H)]
        for i, cv in enumerate(made.values()):
            ox = gap + i * (N * s + gap)
            for y in range(N):
                for x in range(N):
                    p = cv.px[y][x]
                    if not p[3]:
                        continue
                    for yy in range(s):
                        row = sheet_px[gap + y * s + yy]
                        for xx in range(s):
                            row[ox + x * s + xx] = p
        path = sys.argv[sys.argv.index("--preview") + 1] if len(sys.argv) > sys.argv.index("--preview") + 1 \
            else "toolbar_preview.png"
        write_rgba(path, W, H, sheet_px)
        print("preview", path)


if __name__ == "__main__":
    main()
