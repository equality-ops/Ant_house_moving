import argparse
import ast
import os
import sys


ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
MAIN_CAR_DIR = os.path.join(ROOT_DIR, "main_car_1")
if MAIN_CAR_DIR not in sys.path:
    sys.path.insert(0, MAIN_CAR_DIR)

from ant_obstacle_path import FIELD_H, FIELD_W, build_li


def parse_value(text, name):
    try:
        return ast.literal_eval(text)
    except Exception as exc:
        raise ValueError("Cannot parse %s: %s" % (name, text)) from exc


def draw_scene(circle, retangle, start, end, path, r, margin, output, show):
    try:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Polygon, Rectangle
    except ImportError:
        svg_output = _svg_name(output)
        save_svg(circle, retangle, start, end, path, r, margin, svg_output)
        print("matplotlib is not installed. Saved SVG instead:", svg_output)
        print("Install matplotlib for PNG/window display: pip install matplotlib")
        return

    fig, ax = plt.subplots(figsize=(6, 8))
    ax.add_patch(Rectangle((0, 0), FIELD_W, FIELD_H, fill=False,
                           edgecolor="black", linewidth=2))

    safe_r = r + margin
    for c in circle:
        cx, cy = c[0], c[1]
        ax.add_patch(Circle((cx, cy), safe_r, fill=True,
                            color="#ffb3b3", alpha=0.35, linewidth=0))
        ax.add_patch(Circle((cx, cy), r, fill=False,
                            edgecolor="#cc2222", linewidth=2))
        ax.plot(cx, cy, "x", color="#cc2222")

    rects = normalize_rectangles_for_draw(retangle)
    for rect in rects:
        ax.add_patch(Polygon(rect, closed=True, fill=True,
                             facecolor="#ffd580", edgecolor="#c77700",
                             linewidth=2, alpha=0.7))

    ax.plot(start[0], start[1], "o", color="#157f3b", markersize=9, label="start")
    ax.plot(end[0], end[1], "o", color="#1f4fbf", markersize=9, label="end")

    if path:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        ax.plot(xs, ys, "-o", color="#222222", linewidth=2.5,
                markersize=5, label="path")
        for i, p in enumerate(path):
            ax.text(p[0] + 2, p[1] + 2, str(i), fontsize=9, color="#222222")
    else:
        ax.text(FIELD_W / 2, FIELD_H / 2, "No path found",
                ha="center", va="center", fontsize=16, color="#cc2222")

    ax.set_xlim(-10, FIELD_W + 10)
    ax.set_ylim(-10, FIELD_H + 10)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title("Obstacle Path")
    ax.legend(loc="upper right")

    if output:
        fig.savefig(output, dpi=160, bbox_inches="tight")
        print("Saved figure:", output)
    if show:
        plt.show()
    plt.close(fig)


def _svg_name(output):
    if not output:
        return os.path.join(ROOT_DIR, "path_visualization.svg")
    base, _ = os.path.splitext(output)
    return base + ".svg"


def _svg_y(y):
    return FIELD_H - y


def save_svg(circle, retangle, start, end, path, r, margin, output):
    scale = 2.0
    pad = 20.0
    width = FIELD_W * scale + pad * 2
    height = FIELD_H * scale + pad * 2

    def sx(x):
        return pad + x * scale

    def sy(y):
        return pad + _svg_y(y) * scale

    lines = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="%s" height="%s" viewBox="0 0 %s %s">' %
        (width, height, width, height),
        '<rect width="100%" height="100%" fill="white"/>',
        '<rect x="%s" y="%s" width="%s" height="%s" fill="none" stroke="black" stroke-width="2"/>' %
        (sx(0), sy(FIELD_H), FIELD_W * scale, FIELD_H * scale),
    ]

    safe_r = r + margin
    for c in circle:
        cx, cy = c[0], c[1]
        lines.append('<circle cx="%s" cy="%s" r="%s" fill="#ffb3b3" opacity="0.35"/>' %
                     (sx(cx), sy(cy), safe_r * scale))
        lines.append('<circle cx="%s" cy="%s" r="%s" fill="none" stroke="#cc2222" stroke-width="2"/>' %
                     (sx(cx), sy(cy), r * scale))
        lines.append('<text x="%s" y="%s" font-size="12" fill="#cc2222">x</text>' %
                     (sx(cx) - 4, sy(cy) + 4))

    for rect in normalize_rectangles_for_draw(retangle):
        pts = []
        for p in rect:
            pts.append("%s,%s" % (sx(p[0]), sy(p[1])))
        lines.append('<polygon points="%s" fill="#ffd580" opacity="0.7" stroke="#c77700" stroke-width="2"/>' %
                     " ".join(pts))

    if path:
        pts = []
        for p in path:
            pts.append("%s,%s" % (sx(p[0]), sy(p[1])))
        lines.append('<polyline points="%s" fill="none" stroke="#222222" stroke-width="3"/>' %
                     " ".join(pts))
        for i, p in enumerate(path):
            lines.append('<circle cx="%s" cy="%s" r="4" fill="#222222"/>' % (sx(p[0]), sy(p[1])))
            lines.append('<text x="%s" y="%s" font-size="12" fill="#222222">%d</text>' %
                         (sx(p[0]) + 5, sy(p[1]) - 5, i))
    else:
        lines.append('<text x="%s" y="%s" text-anchor="middle" font-size="20" fill="#cc2222">No path found</text>' %
                     (sx(FIELD_W / 2), sy(FIELD_H / 2)))

    lines.append('<circle cx="%s" cy="%s" r="6" fill="#157f3b"/>' % (sx(start[0]), sy(start[1])))
    lines.append('<text x="%s" y="%s" font-size="12" fill="#157f3b">start</text>' %
                 (sx(start[0]) + 8, sy(start[1]) + 4))
    lines.append('<circle cx="%s" cy="%s" r="6" fill="#1f4fbf"/>' % (sx(end[0]), sy(end[1])))
    lines.append('<text x="%s" y="%s" font-size="12" fill="#1f4fbf">end</text>' %
                 (sx(end[0]) + 8, sy(end[1]) + 4))

    lines.append("</svg>")
    with open(output, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


def normalize_rectangles_for_draw(retangle):
    if not retangle:
        return []
    if len(retangle) == 4 and len(retangle[0]) == 2 and isinstance(retangle[0][0], (int, float)):
        return [retangle]
    return retangle


def main():
    parser = argparse.ArgumentParser(
        description="Call ant_obstacle_path.py and visualize the planned path."
    )
    parser.add_argument("--circle", default="[(50, 100), (150, 220)]",
                        help='Circle centers, e.g. "[(50,50),(150,250)]"')
    parser.add_argument("--retangle", default="[[(80, 100), (80, 200), (180, 200), (180, 100)],[(50, 300), (50, 250), (100, 250), (100, 300)]]",
                        help='Rectangles, e.g. "[[(40,40),(60,40),(60,60),(40,60)]]"')
    parser.add_argument("--start", default="(50,50)",
                        help='Start point, e.g. "(10,50)"')
    parser.add_argument("--end", default="(200, 250)",
                        help='End point, e.g. "(220,260)"')
    parser.add_argument("--r", type=float, default=10.0,
                        help="Circle obstacle radius.")
    parser.add_argument("--margin", type=float, default=2.0,
                        help="Safety margin used by path planner.")
    parser.add_argument("--output", default="path_visualization.png",
                        help="Output image path. Use empty string to skip saving.")
    parser.add_argument("--no-show", action="store_true",
                        help="Do not open the matplotlib window.")
    args = parser.parse_args()

    circle = parse_value(args.circle, "circle")
    retangle = parse_value(args.retangle, "retangle")
    start = parse_value(args.start, "start")
    end = parse_value(args.end, "end")

    path = build_li(circle, retangle, start, end, r=20, margin=args.margin)
    print("li =", path)

    output = args.output if args.output else None
    draw_scene(circle, retangle, start, end, path, 20, args.margin,
               output, not args.no_show)


if __name__ == "__main__":
    main()
