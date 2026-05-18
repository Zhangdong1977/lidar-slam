#!/usr/bin/env python3
"""
Diagnose Nav2 InflationLayer cost distribution and RViz2 color mapping.

Captures a costmap frame, prints cost statistics, and explains the
OccupancyGrid 0-100 scaling that causes the "cyan core / red periphery"
visual pattern observed in RViz2.

CRITICAL FINDING: Nav2 costmap_2d_ros converts internal 0-255 costs to
the OccupancyGrid 0-100 convention when publishing on the /costmap topic.
This means:
  - Internal cost 253 (INSCRIBED_INFLATED) → published 99 → palette[99] = CYAN
  - Internal cost 252 (MAX_NON_OBSTACLE)   → published 98 → palette[98] = (249,0,6) ≈ RED
  - The cyan↔red boundary at published 99↔98 corresponds to the
    inscribed_radius_ boundary (0.30m from the robot footprint geometry).
    This boundary is NOT affected by cost_scaling_factor.

Usage:
    python3 scripts/diagnose_inflation.py                    # local costmap
    python3 scripts/diagnose_inflation.py --global            # global costmap
    python3 scripts/diagnose_inflation.py -t /other/costmap   # custom topic
    python3 scripts/diagnose_inflation.py --all-frames        # don't skip empty frames
"""

import argparse
import math
import sys

import numpy as np

try:
    from PIL import Image as PILImage
except ImportError:
    PILImage = None

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


# ── RViz2 exact costmap palette (from palette_builder.cpp, Jazzy) ──────────
def build_costmap_palette():
    pal = np.zeros(256 * 4, dtype=np.uint8)
    pal[0:4] = [0, 0, 0, 0]
    for i in range(1, 99):
        v = (255 * i) // 100
        pal[4 * i : 4 * i + 4] = [v, 0, 255 - v, 255]
    pal[4 * 99 : 4 * 99 + 4] = [0, 255, 255, 255]        # cyan
    pal[4 * 100 : 4 * 100 + 4] = [255, 0, 255, 255]       # magenta
    for i in range(101, 128):
        pal[4 * i : 4 * i + 4] = [0, 255, 0, 255]         # green
    for i in range(128, 255):
        g = (255 * (i - 128)) // (254 - 128)
        pal[4 * i : 4 * i + 4] = [255, g, 0, 255]         # red→yellow
    pal[4 * 255 : 4 * 255 + 4] = [0x70, 0x89, 0x86, 255]  # grayish teal
    return pal


def palette_rgb(pal, cost):
    return pal[4 * cost], pal[4 * cost + 1], pal[4 * cost + 2]


# ── OccupancyGrid scaling ───────────────────────────────────────────────────
# Nav2 costmap_2d_ros publishes costs as nav_msgs/OccupancyGrid,
# which uses the ROS standard 0-100 convention for int8 data.
# Internal Nav2 costs (0-255) are scaled: published = int(255 * internal / 100)
# For display we invert: internal ≈ int(published * 255 / 100)

def published_to_internal(pub):
    return int(pub * 255 / 100)

def internal_to_published(internal):
    return int(internal * 100 / 255)


# ── Nav2 cost constants ─────────────────────────────────────────────────────
FREE_SPACE = 0
LETHAL_OBSTACLE = 254
INSCRIBED_INFLATED_OBSTACLE = 253
MAX_NON_OBSTACLE = 252
NO_INFORMATION = 255


class InflationDiagnoser(Node):
    def __init__(self, topic, require_nonzero=True):
        super().__init__("inflation_diagnoser")
        self.costmap = None
        self.info = None
        self.best_costmap = None
        self.best_info = None
        self.best_nonzero = 0
        self.frames_seen = 0
        self.require_nonzero = require_nonzero
        self.sub = self.create_subscription(OccupancyGrid, topic, self.callback, 1)
        self.get_logger().info(f"Listening on {topic} (require_nonzero={require_nonzero})...")

    def callback(self, msg):
        self.frames_seen += 1
        raw = np.array(msg.data, dtype=np.int8)
        cm = raw.view(np.uint8).reshape(msg.info.height, msg.info.width)
        nnz = int(np.count_nonzero(cm))

        if nnz > self.best_nonzero:
            self.best_nonzero = nnz
            self.best_costmap = cm
            self.best_info = msg.info

        if self.costmap is not None:
            return
        if self.require_nonzero and nnz == 0:
            return

        self.costmap = cm
        self.info = msg.info
        self.get_logger().info(
            f"Captured {self.info.width}x{self.info.height} costmap "
            f"at {self.info.resolution:.3f} m/pixel ({nnz} non-zero cells)"
        )


def back_calc_csf(costmap, resolution, inscribed_radius=0.30):
    """
    Back-calculate cost_scaling_factor from published OccupancyGrid cost values.

    The costmap publishes 0-100 values. We convert to internal 0-255 and
    apply the Nav2 formula to estimate csf.
    """
    height, width = costmap.shape
    # Use published 99+ as anchors (maps to internal 253+)
    anchor_mask = costmap >= 99
    anchor_ys, anchor_xs = np.where(anchor_mask)
    if len(anchor_ys) == 0:
        print("  WARNING: no anchor cells (cost >= 99 pub / >= 253 internal)")
        return

    valid = (costmap > FREE_SPACE) & (costmap < 99)
    sample_ys, sample_xs = np.where(valid)
    if len(sample_ys) < 10:
        print("  WARNING: too few inflated cells to back-calculate csf")
        return

    n_sample = min(2000, len(sample_ys))
    indices = np.random.choice(len(sample_ys), n_sample, replace=False)
    sample_ys = sample_ys[indices]
    sample_xs = sample_xs[indices]

    min_excess = inscribed_radius * 0.5  # require at least 0.15m past inscribed
    csfs = []
    for sy, sx in zip(sample_ys, sample_xs):
        dist_cells = np.min(np.sqrt((anchor_ys - sy) ** 2 + (anchor_xs - sx) ** 2))
        distance_m = dist_cells * resolution
        excess = distance_m - inscribed_radius
        if excess < min_excess:
            continue
        pub_cost = float(costmap[sy, sx])
        internal_cost = pub_cost * 255.0 / 100.0  # invert OccupancyGrid scaling
        ratio = internal_cost / MAX_NON_OBSTACLE
        if ratio <= 0.01 or ratio >= 1.0:
            continue
        csf = -math.log(ratio) / excess
        csfs.append(csf)

    if csfs:
        arr = np.array(csfs)
        clipped = arr[(arr > 0.01) & (arr < 100.0)]
        print(f"\n  Back-calculated effective cost_scaling_factor "
              f"({len(csfs)} raw samples, {len(clipped)} clipped):")
        if len(clipped) > 0:
            print(f"    Median:  {np.median(clipped):.4f}")
            print(f"    P25-P75: [{np.percentile(clipped, 25):.4f}, "
                  f"{np.percentile(clipped, 75):.4f}]")
    else:
        print("  WARNING: no valid samples for back-calculation")


def main():
    parser = argparse.ArgumentParser(description="Diagnose Nav2 InflationLayer")
    parser.add_argument("-t", "--topic", default="/local_costmap/costmap")
    parser.add_argument("--global", action="store_true", dest="use_global")
    parser.add_argument("-o", "--output", default=None)
    parser.add_argument("-r", "--inscribed-radius", type=float, default=0.30)
    parser.add_argument("--resolution", type=float, default=None)
    parser.add_argument("--all-frames", action="store_true",
                        help="Accept empty costmaps (don't require non-zero data)")
    parser.add_argument("--print-costs", action="store_true")
    args = parser.parse_args()

    if args.use_global:
        args.topic = "/global_costmap/costmap"

    rclpy.init()
    node = InflationDiagnoser(args.topic, require_nonzero=not args.all_frames)

    import time
    deadline = time.time() + 10.0
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    while rclpy.ok() and node.costmap is None and time.time() < deadline:
        executor.spin_once(timeout_sec=0.5)
    executor.remove_node(node)
    node.destroy_node()
    rclpy.shutdown()

    # Use best available costmap (fallback to best_costmap if costmap is None)
    if node.costmap is None and node.best_costmap is not None:
        print(f"  NOTE: No non-zero frame received. Using best frame "
              f"({node.best_nonzero} non-zero cells out of {node.frames_seen} frames).")
        print(f"  Move the robot near obstacles for meaningful data.\n")
        node.costmap = node.best_costmap
        node.info = node.best_info

    if node.costmap is None:
        print("ERROR: No costmap received within 10s. Is Nav2 running?")
        sys.exit(1)

    costmap = node.costmap
    info = node.info
    resolution = args.resolution if args.resolution is not None else info.resolution

    # ── Statistics ───────────────────────────────────────────────────
    unique, counts = np.unique(costmap, return_counts=True)
    total_cells = costmap.size

    print("=" * 60)
    print("Cost Distribution — OccupancyGrid (published 0-100 scale)")
    print("=" * 60)
    print(f"  Resolution:  {resolution:.3f} m/pixel")
    print(f"  Dimensions:  {info.width} x {info.height} ({total_cells} cells)")
    print(f"  Inscribed radius: {args.inscribed_radius:.3f} m "
          f"({args.inscribed_radius / resolution:.0f} cells)")

    # Category breakdown on published scale
    cat_free = int(np.sum(costmap == 0))
    # published 0 -> internal 0 (free)
    # published 1-98 -> internal 1-252 (inflated gradient)
    # published 99 -> internal ~253 (inscribed inflated)
    # published 100 -> internal ~255 (lethal / over-range)
    cat_inflated = int(np.sum((costmap > 0) & (costmap < 99)))
    cat_cyan = int(np.sum(costmap == 99))
    cat_magenta = int(np.sum(costmap == 100))
    cat_unknown = int(np.sum(costmap >= 101))

    print(f"\n  Published category breakdown:")
    print(f"    FREE (0):               {cat_free:>8} cells ({100*cat_free/total_cells:5.1f}%)")
    print(f"    GRADIENT (1-98):        {cat_inflated:>8} cells ({100*cat_inflated/total_cells:5.1f}%)")
    print(f"    CYAN ZONE (99):         {cat_cyan:>8} cells ({100*cat_cyan/total_cells:5.1f}%)  ← maps to internal 253")
    print(f"    MAGENTA ZONE (100):     {cat_magenta:>8} cells ({100*cat_magenta/total_cells:5.1f}%)  ← maps to internal 254+")
    print(f"    OTHER (101+):           {cat_unknown:>8} cells ({100*cat_unknown/total_cells:5.1f}%)")

    print(f"\n  OccupancyGrid scaling table (key transition points):")
    print(f"    {'Internal':>10} → {'Published':>11} → {'Palette Color':>20}")
    print(f"    {'─'*10}   {'─'*11}   {'─'*20}")
    for internal in [0, 1, 128, 200, 250, 252, 253, 254, 255]:
        pub = internal_to_published(internal)
        r, g, b = palette_rgb(build_costmap_palette(), pub)
        note = ""
        if internal == 252: note = "← MAX_NON_OBSTACLE = RED"
        elif internal == 253: note = "← INSCRIBED = CYAN"
        elif internal == 254: note = "← LETHAL"
        elif internal == 255: note = "← NO_INFORMATION"
        print(f"    {internal:>10} → {pub:>11} → ({r:3d},{g:3d},{b:3d}) {note}")

    # Interesting cost range
    interesting = costmap[(costmap > FREE_SPACE) & (costmap <= 100)]
    if len(interesting) > 0:
        print(f"\n  Published cost stats (1-100):")
        print(f"    Count: {len(interesting)}")
        print(f"    Min:   {interesting.min()}")
        print(f"    Max:   {interesting.max()}")
        print(f"    Mean:  {interesting.mean():.1f}  → internal ~{interesting.mean()*255/100:.0f}")
        print(f"    Median:{np.median(interesting):.0f}  → internal ~{np.median(interesting)*255/100:.0f}")

    # Top cost values
    print(f"\n  Top 15 published cost values:")
    top_indices = np.argsort(counts)[::-1][:15]
    for idx in top_indices:
        pub_val = int(unique[idx])
        cnt = int(counts[idx])
        r, g, b = palette_rgb(build_costmap_palette(), pub_val)
        internal_est = published_to_internal(pub_val)
        label = {
            0: "FREE_SPACE",
            99: "*** CYAN (internal 253, inscribed) ***",
            100: "*** MAGENTA (internal 254+) ***",
        }.get(pub_val, "")
        if 1 <= pub_val <= 98:
            label = f"gradient (internal ~{internal_est})"
        elif 101 <= pub_val <= 127:
            label = "GREEN (illegal)"
        elif pub_val >= 128:
            label = "RED/YELLOW"
        print(f"    pub={pub_val:3d}  → ({r:3d},{g:3d},{b:3d})  "
              f"internal≈{internal_est:3d}  n={cnt:>8} ({100*cnt/total_cells:5.1f}%)  {label}")

    # ── Color boundary analysis ───────────────────────────────────────
    if cat_cyan > 0:
        print(f"\n  *** CYAN/RED BOUNDARY ANALYSIS ***")
        print(f"  The cyan (99) ↔ red (98) color boundary in RViz2 is at")
        print(f"  published cost 99↔98, which corresponds to internal cost")
        print(f"  253↔252 = inscribed_radius ({args.inscribed_radius:.2f}m).")
        print(f"  This boundary is determined by the robot footprint geometry,")
        print(f"  NOT by cost_scaling_factor. To expand the cyan zone you")
        print(f"  would need to increase inscribed_radius (larger footprint).")
        print(f"  Ratio cyan:red ≈ {args.inscribed_radius:.2f} : "
              f"({resolution * max(costmap.shape):.1f} - {args.inscribed_radius:.2f})")

    # ── Back-calculate cost_scaling_factor ───────────────────────────
    print(f"\n  Back-calculating effective cost_scaling_factor...")
    back_calc_csf(costmap, resolution, args.inscribed_radius)

    # ── Cost profile (optional) ───────────────────────────────────────
    if args.print_costs and cat_cyan > 0:
        anchor_ys, anchor_xs = np.where(costmap >= 99)
        print(f"\n  Cost profile from a CYAN (99) anchor:")
        ay, ax = anchor_ys[len(anchor_ys) // 2], anchor_xs[len(anchor_xs) // 2]
        print(f"  Anchor at pixel ({ax}, {ay}):")
        max_dx = min(60, info.width - ax - 1)
        print(f"  {'dist(m)':>8}  {'pub':>4}  {'internal':>9}  {'color(R,G,B)':>16}")
        for dx in range(0, max_dx):
            dist_m = dx * resolution
            c = int(costmap[ay, ax + dx])
            internal_c = published_to_internal(c)
            r, g, b = palette_rgb(build_costmap_palette(), c)
            marker = ""
            if c == 99 and dx > 1:
                marker = " ← cyan→red boundary"
            print(f"  {dist_m:8.3f}  {c:>4}  {internal_c:>9}  ({r:3d},{g:3d},{b:3d}){marker}")
    elif args.print_costs:
        print(f"\n  (No CYAN cells found — cannot print cost profile. Run near obstacles.)")

    # ── Render RGB image ─────────────────────────────────────────────
    pal = build_costmap_palette()
    rgb = np.zeros((info.height, info.width, 3), dtype=np.uint8)
    for i in range(256):
        mask = costmap == i
        rgb[mask, 0] = pal[4 * i]
        rgb[mask, 1] = pal[4 * i + 1]
        rgb[mask, 2] = pal[4 * i + 2]
    rgb[costmap == FREE_SPACE] = [255, 255, 255]

    output_path = args.output
    if output_path is None:
        topic_slug = args.topic.replace("/", "_").strip("_")
        output_path = f"/tmp/{topic_slug}_diagnostic.png"
    if PILImage is not None:
        PILImage.fromarray(rgb, "RGB").save(output_path)
        print(f"\n  RGB diagnostic image saved to: {output_path}")
    else:
        npy_path = output_path.rsplit(".", 1)[0] + ".npy"
        np.save(npy_path, rgb)
        print(f"\n  Pillow not available. Raw RGB array saved to: {npy_path}")


if __name__ == "__main__":
    main()
