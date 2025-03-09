import math
import xml.etree.ElementTree as ET
from PIL import Image

# Replace this string with your SDF, ensuring no blank lines before <?xml
sdf_data = r"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="world_trial">
    <!-- SDF snippet -->
    <model name="box1">
      <pose>-3 0 7 0 0 0</pose>
      <link name="box_link1">
        <collision name="box_collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <model name="box2">
      <pose>2 -7 1 0 0 0</pose>
      <link name="box_link2">
        <collision name="box_collision">
          <geometry>
            <box>
              <size>1 5 1</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <model name="sph1">
      <pose>9 2 0 0 0 0</pose>
      <link name="sph_link1">
        <collision name="sph_collision">
          <geometry>
            <sphere>
              <radius>5</radius>
            </sphere>
          </geometry>
        </collision>
      </link>
    </model>

    <model name="sph2">
      <pose>11 -13 0 0 0 0</pose>
      <link name="sph_link2">
        <collision name="sph_collision2">
          <geometry>
            <sphere>
              <radius>3</radius>
            </sphere>
          </geometry>
        </collision>
      </link>
    </model>

    <model name="sph3">
      <pose>11 17 0 0 0 0</pose>
      <link name="sph_link3">
        <collision name="sph_collision3">
          <geometry>
            <sphere>
              <radius>3</radius>
            </sphere>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
"""

# 1) Parse the SDF, ignoring height and yaw
root = ET.fromstring(sdf_data)

obstacles = []
for model in root.findall(".//model"):
    pose_elem = model.find("pose")
    if pose_elem is not None:
        pose_vals = pose_elem.text.strip().split()
        mx, my = float(pose_vals[0]), float(pose_vals[1])
    else:
        mx, my = 0.0, 0.0
    
    coll = model.find(".//collision")
    if not coll:
        continue

    geom = coll.find("geometry")
    if not geom:
        continue

    box_elem = geom.find("box")
    sphere_elem = geom.find("sphere")

    if box_elem is not None:
        size_elem = box_elem.find("size")
        sx, sy, sz = map(float, size_elem.text.strip().split())
        # We'll place a 2D rectangle of width sx and height sy at (mx,my).
        obstacles.append({
            "type": "rect",
            "center": (mx, my),
            "half_size": (sx/2.0, sy/2.0)
        })

    elif sphere_elem is not None:
        r_elem = sphere_elem.find("radius")
        r = float(r_elem.text.strip())
        obstacles.append({
            "type": "sphere",
            "center": (mx, my),
            "radius": r
        })

# 2) Create a 2D grid covering x=-20..20, y=-20..20 with resolution=0.05
x_min, x_max = -20.0, 20.0
y_min, y_max = -20.0, 20.0
resolution = 0.05  # meters/pixel

width  = int((x_max - x_min) / resolution)  # 800
height = int((y_max - y_min) / resolution)  # 800

img = Image.new("L", (width, height), 255)  # 255=free, 0=occupied
pixels = img.load()

def in_rectangle(px, py, cx, cy, half_w, half_h):
    return (abs(px - cx) <= half_w) and (abs(py - cy) <= half_h)

def in_sphere(px, py, cx, cy, r):
    return math.hypot(px - cx, py - cy) <= r

for ix in range(width):
    for iy in range(height):
        # Convert pixel (ix, iy) -> world coords
        wx = x_min + (ix + 0.5) * resolution
        wy = y_min + (iy + 0.5) * resolution
        
        occupied = False
        for obs in obstacles:
            if obs["type"] == "rect":
                hw, hh = obs["half_size"]
                if in_rectangle(wx, wy, obs["center"][0], obs["center"][1], hw, hh):
                    occupied = True
                    break
            elif obs["type"] == "sphere":
                if in_sphere(wx, wy, obs["center"][0], obs["center"][1], obs["radius"]):
                    occupied = True
                    break
        
        if occupied:
            # Mark black=0. Flip Y to match top-down format
            pixels[ix, height - iy - 1] = 0

# 3) Save the PNG
img.save("my_costmap_rectangles.png")
print("Created my_costmap_rectangles.png, size =", img.size)
