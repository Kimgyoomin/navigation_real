#!/usr/bin/env python3

import math
from pathlib import Path

import numpy as np
import yaml
from PIL import Image


# ============================================================
# Input
#
# 중요:
#   여기에는 현재 RViz에서 PCD와 맞는 YAML을 넣는다.
#   지금 네 /map origin이 [-62.470326, 9.196763, -1.379025]
#   으로 나오는 그 YAML이다.
# ============================================================

map_dir = Path("/home/ams4976/ros2_ws/src/pongbot_navigation/maps")
src_yaml_path = map_dir / "building_1f_map_flip_y.yaml"


# ============================================================
# Output
# ============================================================

dst_image_name = "building_1f_map_nav2_yaw0.pgm"
dst_yaml_name = "building_1f_map_nav2_yaw0.yaml"

dst_image_path = map_dir / dst_image_name
dst_yaml_path = map_dir / dst_yaml_name


def rot2d(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([
        [c, -s],
        [s,  c],
    ], dtype=float)


def yaw_from_quaternion(q):
    # YAML origin is usually [x, y, yaw], so this function is not used
    # unless you extend this script later.
    z = q["z"]
    w = q["w"]
    return 2.0 * math.atan2(z, w)


# ============================================================
# 1. Load source YAML and image
# ============================================================

with open(src_yaml_path, "r") as f:
    src_yaml = yaml.safe_load(f)

src_image_path = map_dir / src_yaml["image"]

resolution = float(src_yaml["resolution"])
src_origin = src_yaml["origin"]

src_origin_x = float(src_origin[0])
src_origin_y = float(src_origin[1])
src_yaw = float(src_origin[2])

src_img = Image.open(src_image_path).convert("L")
src = np.array(src_img, dtype=np.uint8)

src_h, src_w = src.shape

print("")
print("Source map:")
print(f"  yaml      : {src_yaml_path}")
print(f"  image     : {src_image_path}")
print(f"  size      : {src_w} x {src_h}")
print(f"  resolution: {resolution}")
print(f"  origin    : [{src_origin_x}, {src_origin_y}, {src_yaw}]")
print(f"  yaw deg   : {math.degrees(src_yaw):.3f}")


# ============================================================
# 2. Decide unknown fill value
#
# ROS map_saver often uses:
#   occupied: 0
#   free    : 254
#   unknown : 205
#
# But to be safer, use the most common border pixel as outside fill.
# ============================================================

border_pixels = np.concatenate([
    src[0, :],
    src[-1, :],
    src[:, 0],
    src[:, -1],
])

counts = np.bincount(border_pixels, minlength=256)
unknown_value = int(np.argmax(counts))

print(f"  outside fill value: {unknown_value}")


# ============================================================
# 3. Compute world-space bounding box of rotated source map
#
# Source local coordinates:
#   q_local = [x, y]
#
# Source world coordinates:
#   p_world = R(src_yaw) * q_local + src_origin
#
# The output map will have yaw=0, so its image canvas must cover
# the rotated source map's world-space bounding box.
# ============================================================

R = rot2d(src_yaw)
R_inv = R.T

src_origin_xy = np.array([src_origin_x, src_origin_y], dtype=float)

src_corners_local = np.array([
    [0.0, 0.0],
    [src_w * resolution, 0.0],
    [0.0, src_h * resolution],
    [src_w * resolution, src_h * resolution],
], dtype=float)

world_corners = (R @ src_corners_local.T).T + src_origin_xy

min_xy = np.floor(world_corners.min(axis=0) / resolution) * resolution
max_xy = np.ceil(world_corners.max(axis=0) / resolution) * resolution

dst_w = int(math.ceil((max_xy[0] - min_xy[0]) / resolution))
dst_h = int(math.ceil((max_xy[1] - min_xy[1]) / resolution))

dst_origin_x = float(min_xy[0])
dst_origin_y = float(min_xy[1])

print("")
print("Destination map:")
print(f"  size  : {dst_w} x {dst_h}")
print(f"  origin: [{dst_origin_x:.6f}, {dst_origin_y:.6f}, 0.0]")


# ============================================================
# 4. Resample source image into yaw=0 destination image
#
# Destination world coordinate:
#   p_world = dst_origin + [x_cell, y_cell]
#
# Back-project into source local coordinate:
#   q_src = R(-src_yaw) * (p_world - src_origin)
#
# Note:
#   PGM row 0 is top.
#   ROS map y=0 is bottom.
# ============================================================

dst = np.full((dst_h, dst_w), unknown_value, dtype=np.uint8)

xs_world = dst_origin_x + (np.arange(dst_w, dtype=float) + 0.5) * resolution

c = math.cos(src_yaw)
s = math.sin(src_yaw)

for row_dst_top in range(dst_h):
    row_dst_bottom = dst_h - 1 - row_dst_top
    y_world = dst_origin_y + (row_dst_bottom + 0.5) * resolution

    dx = xs_world - src_origin_x
    dy = y_world - src_origin_y

    # R(-yaw) * [dx, dy]
    qx = c * dx + s * dy
    qy = -s * dx + c * dy

    cols_src = np.floor(qx / resolution).astype(np.int64)
    rows_src_bottom = np.floor(qy / resolution).astype(np.int64)
    rows_src_top = src_h - 1 - rows_src_bottom

    valid = (
        (cols_src >= 0) &
        (cols_src < src_w) &
        (rows_src_top >= 0) &
        (rows_src_top < src_h)
    )

    dst[row_dst_top, valid] = src[rows_src_top[valid], cols_src[valid]]


# ============================================================
# 5. Save output image and YAML
# ============================================================

Image.fromarray(dst).save(dst_image_path)

dst_yaml = dict(src_yaml)
dst_yaml["image"] = dst_image_name
dst_yaml["origin"] = [dst_origin_x, dst_origin_y, 0.0]
dst_yaml["resolution"] = resolution

with open(dst_yaml_path, "w") as f:
    yaml.safe_dump(dst_yaml, f, sort_keys=False)

print("")
print("Created:")
print(f"  {dst_image_path}")
print(f"  {dst_yaml_path}")
print("")
print("Use this map for Nav2:")
print(f"  map_yaml: {dst_yaml_path}")
print("")
