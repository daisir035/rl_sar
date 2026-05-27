#!/usr/bin/env python3
"""
Unified MuJoCo Terrain Generator for rl_sar Sim2Sim.

Supports all MUJOCO_ROBOTS. Generates scene XML files with various terrain types:
  flat, stairs, suspend_stairs, slope, rough_ground, obstacles,
  perlin_hfield, image_hfield, mixed, extreme

Usage:
  python terrain_generator.py <robot> <preset> [--seed N] [--output name]
  python terrain_generator.py 0315 mixed --seed 42 --output my_terrain
"""

import argparse
import os
import sys
import xml.etree.ElementTree as xml_et
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ROBOTS = {"0315", "b2", "b2w", "d1", "g1", "go2", "go2w"}

TERRAIN_PRESETS = [
    "flat",
    "stairs",
    "suspend_stairs",
    "slope",
    "rough_ground",
    "obstacles",
    "perlin_hfield",
    "image_hfield",
    "mixed",
    "extreme",
]


def euler_to_quat(roll, pitch, yaw):
    cx, sx = np.cos(roll / 2), np.sin(roll / 2)
    cy, sy = np.cos(pitch / 2), np.sin(pitch / 2)
    cz, sz = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array([
        cx * cy * cz + sx * sy * sz,
        sx * cy * cz - cx * sy * sz,
        cx * sy * cz + sx * cy * sz,
        cx * cy * sz - sx * sy * cz,
    ], dtype=np.float64)


def euler_to_rot(roll, pitch, yaw):
    rot_x = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]], dtype=np.float64)
    rot_y = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]], dtype=np.float64)
    rot_z = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]], dtype=np.float64)
    return rot_z @ rot_y @ rot_x


def rot2d(x, y, yaw):
    return x * np.cos(yaw) - y * np.sin(yaw), x * np.sin(yaw) + y * np.cos(yaw)


def rot3d(pos, euler):
    return euler_to_rot(euler[0], euler[1], euler[2]) @ np.array(pos, dtype=np.float64)


def list_to_str(vec):
    return " ".join(str(float(s)) for s in vec)


class TerrainGenerator:
    """Generate MuJoCo scene XML with procedural terrain."""

    def __init__(self, robot_name, base_scene_path=None):
        if robot_name not in ROBOTS:
            raise ValueError(f"Robot '{robot_name}' not supported. Choose from: {ROBOTS}")
        self.robot_name = robot_name
        self.mjcf_dir = os.path.join(PROJECT_ROOT, "src", "rl_sar_zoo", f"{robot_name}_description", "mjcf")

        if base_scene_path and os.path.isfile(base_scene_path):
            self.scene = xml_et.parse(base_scene_path)
        else:
            # Use the robot's default scene.xml as base
            default_scene = os.path.join(self.mjcf_dir, "scene.xml")
            if not os.path.isfile(default_scene):
                raise FileNotFoundError(f"Default scene not found: {default_scene}")
            self.scene = xml_et.parse(default_scene)

        self.root = self.scene.getroot()
        self.worldbody = self.root.find("worldbody")
        self.asset = self.root.find("asset")
        if self.worldbody is None:
            raise RuntimeError("Parsed XML has no <worldbody>")
        if self.asset is None:
            self.asset = xml_et.SubElement(self.root, "asset")

        # Clear existing non-floor geoms from worldbody to build fresh terrain
        self._clear_existing_terrain()

    def _clear_existing_terrain(self):
        """Remove all geom elements except the floor plane and lights."""
        to_remove = []
        for child in self.worldbody:
            if child.tag == "geom":
                # Keep the floor plane
                if child.get("name") == "floor":
                    continue
                to_remove.append(child)
        for child in to_remove:
            self.worldbody.remove(child)

        # Clear existing hfield assets
        if self.asset is not None:
            to_remove = []
            for child in self.asset:
                if child.tag == "hfield":
                    to_remove.append(child)
            for child in to_remove:
                self.asset.remove(child)

    # ── Primitive builders ──────────────────────────────────────

    def add_box(self, pos, size, euler=(0, 0, 0), rgba=None):
        geo = xml_et.SubElement(self.worldbody, "geom")
        geo.attrib["pos"] = list_to_str(pos)
        geo.attrib["type"] = "box"
        geo.attrib["size"] = list_to_str(0.5 * np.array(size))
        geo.attrib["quat"] = list_to_str(euler_to_quat(euler[0], euler[1], euler[2]))
        if rgba:
            geo.attrib["rgba"] = list_to_str(rgba)
        return geo

    def add_geometry(self, pos, size, geo_type="box", euler=(0, 0, 0), rgba=None):
        geo = xml_et.SubElement(self.worldbody, "geom")
        geo.attrib["pos"] = list_to_str(pos)
        geo.attrib["type"] = geo_type
        geo.attrib["size"] = list_to_str(0.5 * np.array(size)) if geo_type == "box" else list_to_str(size)
        geo.attrib["quat"] = list_to_str(euler_to_quat(euler[0], euler[1], euler[2]))
        if rgba:
            geo.attrib["rgba"] = list_to_str(rgba)
        return geo

    # ── Composite terrain types ─────────────────────────────────

    def add_stairs(self, init_pos=(1.0, 0.0, 0.0), yaw=0.0, width=0.2, height=0.15, length=1.5, num=10):
        local_pos = [0.0, 0.0, -0.5 * height]
        for i in range(num):
            local_pos[0] += width
            local_pos[2] += height
            x, y = rot2d(local_pos[0], local_pos[1], yaw)
            self.add_box([x + init_pos[0], y + init_pos[1], local_pos[2]], [width, length, height], [0, 0, yaw])

    def add_suspend_stairs(self, init_pos=(1.0, 0.0, 0.0), yaw=0.0, width=0.2, height=0.15, length=1.5, gap=0.1, num=10):
        local_pos = [0.0, 0.0, -0.5 * height]
        for i in range(num):
            local_pos[0] += width
            local_pos[2] += height
            x, y = rot2d(local_pos[0], local_pos[1], yaw)
            self.add_box([x + init_pos[0], y + init_pos[1], local_pos[2]], [width, length, abs(height - gap)], [0, 0, yaw])

    def add_slope(self, pos=(2.0, 0.0, 0.5), size=(3.0, 1.5, 0.1), pitch=-0.5, yaw=0.0):
        self.add_box(pos, size, [0.0, pitch, yaw])

    def add_rough_ground(self, init_pos=(1.0, 0.0, 0.0), euler=(0, 0, 0), nums=(10, 10),
                         box_size=(0.5, 0.5, 0.5), separation=(0.2, 0.2),
                         box_size_rand=(0.05, 0.05, 0.05), box_euler_rand=(0.2, 0.2, 0.2),
                         separation_rand=(0.05, 0.05)):
        local_pos = [0.0, 0.0, -0.5 * box_size[2]]
        new_sep = np.array(separation) + np.array(separation_rand) * np.random.uniform(-1.0, 1.0, 2)
        for i in range(nums[0]):
            local_pos[0] += new_sep[0]
            local_pos[1] = 0.0
            for j in range(nums[1]):
                new_box_size = np.array(box_size) + np.array(box_size_rand) * np.random.uniform(-1.0, 1.0, 3)
                new_box_euler = np.array(euler) + np.array(box_euler_rand) * np.random.uniform(-1.0, 1.0, 3)
                new_sep = np.array(separation) + np.array(separation_rand) * np.random.uniform(-1.0, 1.0, 2)
                local_pos[1] += new_sep[1]
                pos = rot3d(local_pos, euler) + np.array(init_pos)
                self.add_box(pos, new_box_size, new_box_euler, rgba=(0.5, 0.45, 0.4, 1))

    def add_obstacles(self, count=15, area=((2.0, 6.0), (-2.0, 2.0)), height_range=(0.05, 0.3)):
        """Add random box and cylinder obstacles."""
        for _ in range(count):
            x = np.random.uniform(area[0][0], area[0][1])
            y = np.random.uniform(area[1][0], area[1][1])
            h = np.random.uniform(height_range[0], height_range[1])
            geo_type = np.random.choice(["box", "cylinder"])
            yaw = np.random.uniform(-0.5, 0.5)
            if geo_type == "box":
                sx = np.random.uniform(0.1, 0.3)
                sy = np.random.uniform(0.1, 0.25)
                self.add_box((x, y, h * 0.5), (sx, sy, h), (0, 0, yaw), rgba=(0.5, 0.4, 0.35, 1))
            else:
                r = np.random.uniform(0.08, 0.18)
                self.add_geometry((x, y, h * 0.5), (r, h), geo_type="cylinder", euler=(0, 0, yaw), rgba=(0.6, 0.55, 0.5, 1))

    def add_perlin_hfield(self, pos=(0.0, 0.0, 0.0), euler=(0, 0, 0), size=(4.0, 3.0),
                          height_scale=0.2, negative_height=0.2, image_size=(256, 256),
                          smooth=100.0, octaves=6, persistence=0.5, lacunarity=2.0,
                          name="perlin_terrain"):
        try:
            from noise import pnoise2
        except ImportError:
            print("Warning: noise library not installed. Install with: pip install noise")
            return

        img_h, img_w = image_size
        terrain = np.zeros((img_h, img_w), dtype=np.uint8)
        for y in range(img_h):
            for x in range(img_w):
                val = pnoise2(x / smooth, y / smooth, octaves=octaves,
                              persistence=persistence, lacunarity=lacunarity)
                terrain[y, x] = int((val + 1) / 2 * 255)

        img_path = os.path.join(self.mjcf_dir, f"{name}.png")
        try:
            from PIL import Image
            Image.fromarray(terrain, mode="L").save(img_path)
        except ImportError:
            import cv2
            cv2.imwrite(img_path, terrain)

        hfield = xml_et.SubElement(self.asset, "hfield")
        hfield.attrib["name"] = name
        hfield.attrib["size"] = list_to_str([size[0] / 2.0, size[1] / 2.0, height_scale, negative_height])
        hfield.attrib["file"] = f"{name}.png"

        geo = xml_et.SubElement(self.worldbody, "geom")
        geo.attrib["type"] = "hfield"
        geo.attrib["hfield"] = name
        geo.attrib["pos"] = list_to_str(pos)
        geo.attrib["quat"] = list_to_str(euler_to_quat(euler[0], euler[1], euler[2]))

    def add_image_hfield(self, pos=(0.0, 0.0, 0.0), euler=(0, 0, 0), size=(2.0, 1.6),
                         height_scale=0.02, negative_height=0.1, input_img=None,
                         image_scale=(1.0, 1.0), invert_gray=False, name="image_terrain"):
        if input_img is None or not os.path.isfile(input_img):
            print(f"Warning: input image not found: {input_img}")
            return

        try:
            import cv2
            img = cv2.imread(input_img)
            w = int(img.shape[1] * image_scale[0])
            h = int(img.shape[0] * image_scale[1])
            resized = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
            gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
            if invert_gray:
                gray = 255 - gray
            img_path = os.path.join(self.mjcf_dir, f"{name}.png")
            cv2.imwrite(img_path, gray)
        except Exception as e:
            print(f"Warning: failed to process image: {e}")
            return

        hfield = xml_et.SubElement(self.asset, "hfield")
        hfield.attrib["name"] = name
        hfield.attrib["size"] = list_to_str([size[0] / 2.0, size[1] / 2.0, height_scale, negative_height])
        hfield.attrib["file"] = f"{name}.png"

        geo = xml_et.SubElement(self.worldbody, "geom")
        geo.attrib["type"] = "hfield"
        geo.attrib["hfield"] = name
        geo.attrib["pos"] = list_to_str(pos)
        geo.attrib["quat"] = list_to_str(euler_to_quat(euler[0], euler[1], euler[2]))

    # ── Preset generators ───────────────────────────────────────

    def generate_flat(self):
        """Flat ground only (base scene already has floor)."""
        pass

    def generate_stairs(self):
        self.add_stairs(init_pos=(1.2, 0.0, 0.0), yaw=0.0, width=0.25, height=0.12, length=2.0, num=12)

    def generate_suspend_stairs(self):
        self.add_suspend_stairs(init_pos=(1.2, 0.0, 0.0), yaw=0.0, width=0.25, height=0.12, length=2.0, gap=0.08, num=12)

    def generate_slope(self):
        self.add_slope(pos=(2.0, 0.0, 0.4), size=(4.0, 2.0, 0.1), pitch=-0.6, yaw=0.0)
        self.add_slope(pos=(4.0, 0.0, 0.6), size=(3.0, 1.5, 0.1), pitch=0.5, yaw=0.3)

    def generate_rough_ground(self):
        self.add_rough_ground(init_pos=(1.0, -2.0, 0.0), euler=(0, 0, 0), nums=(12, 8),
                              box_size=(0.5, 0.5, 0.3), separation=(0.25, 0.25),
                              box_size_rand=(0.1, 0.1, 0.1), box_euler_rand=(0.3, 0.3, 0.3),
                              separation_rand=(0.08, 0.08))

    def generate_obstacles(self):
        self.add_obstacles(count=20, area=((1.5, 6.0), (-2.5, 2.5)), height_range=(0.05, 0.35))

    def generate_perlin_hfield(self):
        self.add_perlin_hfield(pos=(2.0, 0.0, 0.0), size=(5.0, 4.0), height_scale=0.3, negative_height=0.2,
                               image_size=(512, 512), smooth=80.0, octaves=8, name="generated_perlin")

    def generate_image_hfield(self):
        # Look for any PNG in the mjcf dir that could be used
        candidates = [f for f in os.listdir(self.mjcf_dir) if f.endswith(".png") and "perlin" not in f]
        if candidates:
            img = os.path.join(self.mjcf_dir, candidates[0])
        else:
            print("Warning: no source image found for image_hfield, falling back to perlin")
            self.generate_perlin_hfield()
            return
        self.add_image_hfield(pos=(1.5, 0.0, 0.0), size=(3.0, 2.5), input_img=img, name="generated_image")

    def generate_mixed(self):
        """Mix of stairs, slope, obstacles, and a small perlin field."""
        self.add_stairs(init_pos=(1.0, 2.0, 0.0), yaw=0.0, width=0.2, height=0.12, length=1.5, num=10)
        self.add_slope(pos=(3.0, -1.0, 0.3), size=(2.5, 1.5, 0.08), pitch=-0.4, yaw=0.2)
        self.add_obstacles(count=12, area=((2.5, 5.5), (-2.0, 1.0)), height_range=(0.05, 0.25))
        try:
            self.add_perlin_hfield(pos=(-1.0, 0.0, 0.0), size=(3.0, 3.0), height_scale=0.2, negative_height=0.15,
                                   image_size=(256, 256), smooth=100.0, name="mixed_perlin")
        except Exception:
            pass

    def generate_extreme(self):
        """Large-scale mixed terrain for stress testing."""
        # Big obstacles
        self.add_box((3.0, 0.0, 0.3), (2.0, 1.0, 0.6), (0, 0, 0.3), rgba=(0.5, 0.45, 0.4, 1))
        self.add_box((4.5, 1.5, 0.2), (1.5, 2.0, 0.4), (0, 0, -0.2), rgba=(0.5, 0.45, 0.4, 1))
        # Spheres/cylinders
        self.add_geometry((3.5, 2.0, 0.2), (0.4, 0.4, 0.4), "sphere", rgba=(0.55, 0.5, 0.45, 1))
        self.add_geometry((5.0, 2.5, 0.25), (0.5, 0.5), "cylinder", rgba=(0.6, 0.55, 0.5, 1))
        # Stairs
        self.add_stairs(init_pos=(2.0, 4.0, 0.0), yaw=0.0, width=0.25, height=0.12, length=2.0, num=15)
        self.add_suspend_stairs(init_pos=(2.0, 7.0, 0.0), yaw=0.0, width=0.25, height=0.12, length=2.0, gap=0.08, num=15)
        # Slopes
        self.add_slope(pos=(-2.0, 3.0, 0.4), size=(4.0, 2.0, 0.1), pitch=-0.6, yaw=0.3)
        # Rough ground
        self.add_rough_ground(init_pos=(-4.0, -3.0, 0.0), euler=(0, 0, 0), nums=(15, 12),
                              box_size=(0.6, 0.6, 0.4), separation=(0.3, 0.3),
                              box_size_rand=(0.1, 0.1, 0.1), box_euler_rand=(0.3, 0.3, 0.3),
                              separation_rand=(0.1, 0.1))
        self.add_rough_ground(init_pos=(6.0, -4.0, 0.0), euler=(0, 0, 0.5), nums=(12, 10),
                              box_size=(0.5, 0.5, 0.35), separation=(0.25, 0.25),
                              box_size_rand=(0.08, 0.08, 0.08), box_euler_rand=(0.2, 0.2, 0.2),
                              separation_rand=(0.08, 0.08))
        # Large Perlin fields
        try:
            self.add_perlin_hfield(pos=(0.0, -6.0, 0.0), size=(8.0, 6.0), height_scale=0.4, negative_height=0.3,
                                   image_size=(512, 512), smooth=80.0, octaves=8, name="extreme_perlin1")
            self.add_perlin_hfield(pos=(8.0, 2.0, 0.0), size=(6.0, 6.0), height_scale=0.5, negative_height=0.4,
                                   image_size=(512, 512), smooth=50.0, octaves=10, persistence=0.6,
                                   lacunarity=2.2, name="extreme_perlin2")
        except Exception:
            pass

    # ── Public API ──────────────────────────────────────────────

    def generate(self, preset, seed=None):
        if seed is not None:
            np.random.seed(seed)

        method = getattr(self, f"generate_{preset}", None)
        if method is None:
            raise ValueError(f"Unknown preset '{preset}'. Available: {TERRAIN_PRESETS}")
        method()
        return self

    def save(self, output_name=None):
        if output_name is None:
            output_name = "scene_generated"
        if not output_name.endswith(".xml"):
            output_name += ".xml"
        output_path = os.path.join(self.mjcf_dir, output_name)
        # Pretty-print XML with indentation
        xml_et.indent(self.scene, space="  ")
        self.scene.write(output_path, encoding="unicode")
        return output_path


def main():
    parser = argparse.ArgumentParser(description="Generate MuJoCo terrain scenes for rl_sar sim2sim")
    parser.add_argument("robot", choices=sorted(ROBOTS), help="Robot name")
    parser.add_argument("preset", choices=TERRAIN_PRESETS, help="Terrain preset")
    parser.add_argument("--seed", type=int, default=None, help="Random seed")
    parser.add_argument("--output", type=str, default="scene_generated", help="Output XML filename (without .xml)")
    args = parser.parse_args()

    tg = TerrainGenerator(args.robot)
    tg.generate(args.preset, seed=args.seed)
    path = tg.save(args.output)
    print(f"Generated terrain scene: {path}")


if __name__ == "__main__":
    main()
