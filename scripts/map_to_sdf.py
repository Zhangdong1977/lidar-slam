#!/usr/bin/env python3
"""Convert an occupancy grid map (PGM + YAML) to a Gazebo SDF world file.

Reads a ROS map (PGM image + YAML metadata) and generates walls as static
box models in SDF 1.9 format. Uses row/column scanning with segment merging
to produce a reasonable number of wall models.

Usage:
    python3 scripts/map_to_sdf.py --map maps/real_map_big_fix.yaml --output worlds/real_map_big_fix.sdf
"""

import argparse
import os
import sys
from xml.etree.ElementTree import Element, SubElement, ElementTree, indent

import numpy as np
from PIL import Image

try:
    import yaml
except ImportError:
    # Fallback: parse YAML manually for simple map files
    yaml = None


def load_map_yaml(yaml_path):
    """Load map YAML metadata."""
    if yaml:
        with open(yaml_path) as f:
            return yaml.safe_load(f)
    # Simple fallback parser
    meta = {}
    with open(yaml_path) as f:
        for line in f:
            line = line.strip()
            if ":" in line:
                key, val = line.split(":", 1)
                key = key.strip()
                val = val.strip()
                if key == "resolution":
                    meta[key] = float(val)
                elif key == "origin":
                    meta[key] = [float(x) for x in val.strip("[]").split(",")]
                elif key in ("negate", "occupied_thresh", "free_thresh"):
                    meta[key] = float(val)
                elif key == "image":
                    meta[key] = val
                elif key == "mode":
                    meta[key] = val
    return meta


def load_map_image(yaml_path, meta):
    """Load the PGM image relative to the YAML file."""
    img_name = meta["image"]
    img_path = os.path.join(os.path.dirname(yaml_path), img_name)
    img = Image.open(img_path)
    return np.array(img)


def threshold_map(arr, meta):
    """Create binary wall mask from image array.

    PGM values: 0=occupied(wall), 205=unknown, 255=free (with negate=1).
    We exclude 205 (unknown) and pick dark pixels as walls.
    """
    negate = meta.get("negate", 0)
    occupied_thresh = meta.get("occupied_thresh", 0.65)

    if negate:
        # negate=1: dark pixels (0) = occupied/wall
        threshold = int(255 * (1 - occupied_thresh))
        return (arr < threshold) & (arr != 205)
    else:
        # standard: dark pixels (0) = occupied/wall
        threshold = int(255 * (1 - occupied_thresh))
        return (arr < threshold) & (arr != 205)


def extract_horizontal_segments(mask, min_length=3):
    """Extract horizontal wall segments from binary mask.

    Returns list of (row, col_start, col_end) tuples.
    col_end is inclusive.
    """
    segments = []
    h, w = mask.shape
    for row in range(h):
        col = 0
        while col < w:
            if mask[row, col]:
                start = col
                while col < w and mask[row, col]:
                    col += 1
                length = col - start
                if length >= min_length:
                    segments.append((row, start, col - 1))
            else:
                col += 1
    return segments


def extract_vertical_segments(mask, min_length=3):
    """Extract vertical wall segments from binary mask.

    Returns list of (col, row_start, row_end) tuples.
    row_end is inclusive.
    """
    segments = []
    h, w = mask.shape
    for col in range(w):
        row = 0
        while row < h:
            if mask[row, col]:
                start = row
                while row < h and mask[row, col]:
                    row += 1
                length = row - start
                if length >= min_length:
                    segments.append((col, start, row - 1))
            else:
                row += 1
    return segments


def merge_horizontal_segments(segments, max_gap=2, max_span=6):
    """Merge horizontal segments that are close in row and overlap in columns.

    max_span limits how many rows a merged segment can span, preventing
    flood-fill merging of connected wall networks.
    Returns list of (row_center, col_start, col_end, row_span) tuples.
    """
    if not segments:
        return []

    segments = sorted(segments, key=lambda s: (s[0], s[1]))

    merged = []
    used = set()

    for i, (row, cs, ce) in enumerate(segments):
        if i in used:
            continue
        row_min = row
        row_max = row
        col_min = cs
        col_max = ce

        changed = True
        while changed:
            changed = False
            for j, (r2, cs2, ce2) in enumerate(segments):
                if j in used:
                    continue
                if r2 < row_min - max_gap or r2 > row_max + max_gap:
                    continue
                # Don't exceed max_span in row direction
                new_row_min = min(row_min, r2)
                new_row_max = max(row_max, r2)
                if new_row_max - new_row_min > max_span:
                    continue
                # Check column overlap (with some tolerance)
                overlap_start = max(col_min, cs2)
                overlap_end = min(col_max, ce2)
                if overlap_start <= overlap_end + 2:
                    row_min, row_max = new_row_min, new_row_max
                    col_min = min(col_min, cs2)
                    col_max = max(col_max, ce2)
                    used.add(j)
                    changed = True

        used.add(i)
        row_center = (row_min + row_max) / 2.0
        row_span = row_max - row_min + 1
        merged.append((row_center, col_min, col_max, row_span))

    return merged


def merge_vertical_segments(segments, max_gap=2, max_span=6):
    """Merge vertical segments that are close in column and overlap in rows.

    max_span limits how many columns a merged segment can span.
    Returns list of (col_center, row_start, row_end, col_span) tuples.
    """
    if not segments:
        return []

    segments = sorted(segments, key=lambda s: (s[0], s[1]))

    merged = []
    used = set()

    for i, (col, rs, re) in enumerate(segments):
        if i in used:
            continue
        col_min = col
        col_max = col
        row_min = rs
        row_max = re

        changed = True
        while changed:
            changed = False
            for j, (c2, rs2, re2) in enumerate(segments):
                if j in used:
                    continue
                if c2 < col_min - max_gap or c2 > col_max + max_gap:
                    continue
                new_col_min = min(col_min, c2)
                new_col_max = max(col_max, c2)
                if new_col_max - new_col_min > max_span:
                    continue
                overlap_start = max(row_min, rs2)
                overlap_end = min(row_max, re2)
                if overlap_start <= overlap_end + 2:
                    col_min, col_max = new_col_min, new_col_max
                    row_min = min(row_min, rs2)
                    row_max = max(row_max, re2)
                    used.add(j)
                    changed = True

        used.add(i)
        col_center = (col_min + col_max) / 2.0
        col_span = col_max - col_min + 1
        merged.append((col_center, row_min, row_max, col_span))

    return merged


def pixel_to_world(col, row, meta):
    """Convert pixel coordinates to world coordinates."""
    res = meta["resolution"]
    ox, oy = meta["origin"][0], meta["origin"][1]
    h = meta["_img_height"]
    x = ox + col * res
    y = oy + (h - 1 - row) * res
    return x, y


def segments_to_sdf_walls(h_segs, v_segs, meta, wall_height=2.5, wall_thickness=0.1):
    """Convert merged segments to SDF model elements."""
    res = meta["resolution"]
    models = []

    # Horizontal segments -> walls oriented along X axis
    for row_center, col_start, col_end, row_span in h_segs:
        x_start, y_start = pixel_to_world(col_start, row_center, meta)
        x_end, y_end = pixel_to_world(col_end, row_center, meta)
        x_center = (x_start + x_end) / 2.0
        y_center = (y_start + y_end) / 2.0
        length = (col_end - col_start + 1) * res
        thickness = max(row_span * res, wall_thickness)
        models.append({
            "name": f"wall_h_{len(models):03d}",
            "pose": (x_center, y_center, wall_height / 2, 0, 0, 0),
            "size": (length, thickness, wall_height),
        })

    h_count = len(models)

    # Vertical segments -> walls oriented along Y axis
    for col_center, row_start, row_end, col_span in v_segs:
        x_start, y_start = pixel_to_world(col_center, row_start, meta)
        x_end, y_end = pixel_to_world(col_center, row_end, meta)
        x_center = (x_start + x_end) / 2.0
        y_center = (y_start + y_end) / 2.0
        length = (row_end - row_start + 1) * res
        thickness = max(col_span * res, wall_thickness)
        models.append({
            "name": f"wall_v_{len(models) - h_count:03d}",
            "pose": (x_center, y_center, wall_height / 2, 0, 0, 0),
            "size": (thickness, length, wall_height),
        })

    return models


def build_sdf(models, world_name="real_map_big_fix"):
    """Build the complete SDF XML tree."""
    sdf = Element("sdf", version="1.9")
    world = SubElement(sdf, "world", name=world_name)

    # Physics
    physics = SubElement(world, "physics", name="1ms", type="ignored")
    SubElement(physics, "max_step_size").text = "0.001"
    SubElement(physics, "real_time_factor").text = "1.0"

    # Plugins
    for plugin_file, plugin_name in [
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
    ]:
        SubElement(world, "plugin", filename=plugin_file, name=plugin_name)

    sensors_plugin = SubElement(
        world, "plugin", filename="gz-sim-sensors-system",
        name="gz::sim::systems::Sensors"
    )
    SubElement(sensors_plugin, "render_engine").text = "ogre2"

    SubElement(
        world, "plugin", filename="gz-sim-contact-system",
        name="gz::sim::systems::Contact"
    )

    # Sun
    light = SubElement(world, "light", type="directional", name="sun")
    SubElement(light, "cast_shadows").text = "true"
    SubElement(light, "pose").text = "0 0 10 0 0 0"
    SubElement(light, "diffuse").text = "0.8 0.8 0.8 1"
    SubElement(light, "specular").text = "0.2 0.2 0.2 1"
    att = SubElement(light, "attenuation")
    SubElement(att, "range").text = "1000"
    SubElement(att, "constant").text = "0.9"
    SubElement(att, "linear").text = "0.01"
    SubElement(att, "quadratic").text = "0.001"
    SubElement(light, "direction").text = "-0.5 0.1 -0.9"

    # Ground plane
    ground = SubElement(world, "model", name="ground_plane")
    SubElement(ground, "static").text = "true"
    ground_link = SubElement(ground, "link", name="link")
    gc = SubElement(ground_link, "collision", name="collision")
    SubElement(SubElement(gc, "geometry"), "plane").text = ""
    plane_c = gc.find("geometry/plane")
    plane_c.clear()
    SubElement(plane_c, "normal").text = "0 0 1"
    SubElement(plane_c, "size").text = "100 100"
    gv = SubElement(ground_link, "visual", name="visual")
    SubElement(SubElement(gv, "geometry"), "plane").text = ""
    plane_v = gv.find("geometry/plane")
    plane_v.clear()
    SubElement(plane_v, "normal").text = "0 0 1"
    SubElement(plane_v, "size").text = "100 100"
    gm = SubElement(gv, "material")
    SubElement(gm, "ambient").text = "0.8 0.8 0.8 1"
    SubElement(gm, "diffuse").text = "0.8 0.8 0.8 1"

    # Wall models
    for m in models:
        model_el = SubElement(world, "model", name=m["name"])
        SubElement(model_el, "static").text = "true"
        x, y, z, rr, rp, ry = m["pose"]
        SubElement(model_el, "pose").text = f"{x:.4f} {y:.4f} {z:.4f} {rr} {rp} {ry}"
        link = SubElement(model_el, "link", name="link")
        sx, sy, sz = m["size"]
        size_str = f"{sx:.4f} {sy:.4f} {sz:.4f}"

        coll = SubElement(link, "collision", name="collision")
        SubElement(SubElement(coll, "geometry"), "box").text = ""
        SubElement(coll.find("geometry/box"), "size").text = size_str

        vis = SubElement(link, "visual", name="visual")
        SubElement(SubElement(vis, "geometry"), "box").text = ""
        SubElement(vis.find("geometry/box"), "size").text = size_str
        mat = SubElement(vis, "material")
        SubElement(mat, "ambient").text = "0.7 0.7 0.7 1"
        SubElement(mat, "diffuse").text = "0.7 0.7 0.7 1"

    indent(sdf, space="  ")
    return sdf


def main():
    parser = argparse.ArgumentParser(
        description="Convert occupancy grid map to Gazebo SDF world file"
    )
    parser.add_argument("--map", required=True, help="Path to map YAML file")
    parser.add_argument("--output", required=True, help="Output SDF file path")
    parser.add_argument("--wall-height", type=float, default=2.5)
    parser.add_argument("--wall-thickness", type=float, default=0.1)
    parser.add_argument("--min-pixels", type=int, default=3,
                        help="Minimum segment length in pixels")
    parser.add_argument("--merge-gap", type=int, default=2,
                        help="Max pixel gap for merging parallel segments")
    args = parser.parse_args()

    print(f"Loading map metadata from {args.map}...")
    meta = load_map_yaml(args.map)
    print(f"  resolution: {meta['resolution']} m/px")
    print(f"  origin: {meta['origin']}")
    print(f"  negate: {meta.get('negate', 0)}")

    print("Loading map image...")
    arr = load_map_image(args.map, meta)
    meta["_img_height"] = arr.shape[0]
    print(f"  image size: {arr.shape[1]}x{arr.shape[0]} pixels")
    print(f"  world size: {arr.shape[1]*meta['resolution']:.1f}x{arr.shape[0]*meta['resolution']:.1f} m")

    print("Thresholding...")
    mask = threshold_map(arr, meta)
    wall_pixels = np.sum(mask)
    print(f"  wall pixels: {wall_pixels} ({100*wall_pixels/mask.size:.1f}%)")

    print("Extracting horizontal segments...")
    h_segs_raw = extract_horizontal_segments(mask, min_length=args.min_pixels)
    print(f"  raw segments: {len(h_segs_raw)}")

    print("Extracting vertical segments...")
    v_segs_raw = extract_vertical_segments(mask, min_length=args.min_pixels)
    print(f"  raw segments: {len(v_segs_raw)}")

    print("Merging horizontal segments...")
    h_segs = merge_horizontal_segments(h_segs_raw, max_gap=args.merge_gap)
    print(f"  merged: {len(h_segs)}")

    print("Merging vertical segments...")
    v_segs = merge_vertical_segments(v_segs_raw, max_gap=args.merge_gap)
    print(f"  merged: {len(v_segs)}")

    print("Converting to SDF walls...")
    models = segments_to_sdf_walls(
        h_segs, v_segs, meta,
        wall_height=args.wall_height,
        wall_thickness=args.wall_thickness
    )
    print(f"  total wall models: {len(models)}")

    print("Building SDF XML...")
    sdf = build_sdf(models, world_name=os.path.splitext(os.path.basename(args.output))[0])

    tree = ElementTree(sdf)
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    tree.write(args.output, encoding="unicode", xml_declaration=True)
    print(f"Written to {args.output}")

    # File size
    size = os.path.getsize(args.output)
    print(f"  file size: {size/1024:.1f} KB")


if __name__ == "__main__":
    main()
