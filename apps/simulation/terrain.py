"""Build repeatable G1 obstacle courses with real gaps in the walking surface."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


TERRAINS = ("original", "mixed", "platforms", "stones", "gaps", "stairs", "hurdles")
SETTINGS = Path(__file__).with_name("terrain.json")
COLORS = {
    "ground": "0.27 0.32 0.36 1",
    "platforms": "0.78 0.55 0.25 1",
    "stones": "0.35 0.65 0.46 1",
    "gaps": "0.32 0.53 0.74 1",
    "stairs": "0.63 0.48 0.77 1",
    "hurdles": "0.80 0.34 0.27 1",
}


class Course:
    def __init__(self, world, settings, y=0.0, prefix="", add_start=True):
        self.world, self.settings = world, settings
        self.y, self.prefix = y, prefix
        self.half_width = settings["lane_width"] / 2
        self.pit_depth = settings["pit_depth"]
        self.cursor = settings["start_length"]
        self.sections = []
        if add_start:
            self.ground(-4.0, self.cursor, "start")

    def box(self, name, start, end, top=0, color="ground"):
        # Every tile reaches down to the pit floor. No plane exists at walking height.
        bottom = -self.pit_depth
        appearance = {"material": "terrain_sandstone_ground"} if top == 0 else {"rgba": COLORS[color]}
        ET.SubElement(self.world, "geom", name=f"terrain_{self.prefix}{name}", type="box",
                      pos=f"{(start + end) / 2} {self.y} {(top + bottom) / 2}",
                      size=f"{(end - start) / 2} {self.half_width} {(top - bottom) / 2}",
                      friction="1 0.005 0.0001", **appearance)

    def ground(self, start, end, name, color="ground"):
        self.box(name, start, end, color=color)

    def platforms(self):
        cfg = self.settings["platforms"]
        for index, height in enumerate(cfg["heights"]):
            start = self.cursor
            self.box(f"platform_{index}", start, start + cfg["length"], height, "platforms")
            self.cursor += cfg["length"]
            self.ground(self.cursor, self.cursor + cfg["spacing"], f"platform_exit_{index}")
            self.cursor += cfg["spacing"]

    def stones(self):
        cfg = self.settings["stones"]
        for index, (height, radius) in enumerate(zip(cfg["heights"], cfg["radii"])):
            x = self.cursor + cfg["stride"] / 2
            y = self.y + cfg["lateral_offset"] * (1 if index % 2 else -1)
            ET.SubElement(self.world, "geom", name=f"terrain_{self.prefix}stone_{index}", type="cylinder",
                          pos=f"{x} {y} {(height - self.pit_depth) / 2}",
                          size=f"{radius} {(height + self.pit_depth) / 2}",
                          rgba=COLORS["stones"], friction="1 0.005 0.0001")
            self.cursor += cfg["stride"]

    def gaps(self):
        cfg = self.settings["gaps"]
        for index, width in enumerate(cfg["widths"]):
            # Deliberately leave this x interval empty across the whole lane width.
            self.cursor += width
            self.ground(self.cursor, self.cursor + cfg["landing_length"],
                        f"gap_landing_{index}", "gaps")
            self.cursor += cfg["landing_length"]

    def stairs(self):
        cfg = self.settings["stairs"]
        for flight, rise in enumerate(cfg["rises"]):
            heights = [rise * step for step in range(1, cfg["steps"] + 1)]
            for index, height in enumerate(heights + heights[-2::-1]):
                self.box(f"stairs_{flight}_{index}", self.cursor,
                         self.cursor + cfg["tread_length"], height, "stairs")
                self.cursor += cfg["tread_length"]
            self.ground(self.cursor, self.cursor + cfg["landing_length"], f"stairs_exit_{flight}")
            self.cursor += cfg["landing_length"]

    def hurdles(self):
        cfg = self.settings["hurdles"]
        for index, height in enumerate(cfg["heights"]):
            self.box(f"hurdle_{index}", self.cursor, self.cursor + cfg["thickness"], height, "hurdles")
            self.cursor += cfg["thickness"]
            self.ground(self.cursor, self.cursor + cfg["spacing"], f"hurdle_exit_{index}")
            self.cursor += cfg["spacing"]

    def build(self, selection):
        start = self.cursor
        getattr(self, selection)()
        self.sections.append((selection, start, self.cursor, self.y))
        end = self.cursor + self.settings["transition_length"]
        self.ground(self.cursor, end, f"after_{selection}")
        self.cursor = end
        self.ground(self.cursor, self.cursor + 5, "finish")
        return self.sections


def generate_scene(root, selection, destination):
    """Copy scene/robot definitions into a temporary MJCF and replace only terrain."""
    if selection not in TERRAINS or selection == "original":
        raise ValueError(f"Cannot generate terrain: {selection}")
    robot_dir = Path(root) / "unitree_robots/g1"
    scene = ET.parse(robot_dir / "scene_29dof.xml").getroot()
    # Inline the robot so compiler/mesh paths do not depend on the temporary directory.
    for include in list(scene.findall("include")):
        scene.remove(include)
    robot = ET.parse(robot_dir / "g1_29dof.xml").getroot()
    for child in reversed(list(robot)):
        if child.tag == "compiler":
            for attribute in ("meshdir", "texturedir"):
                if attribute in child.attrib:
                    child.set(attribute, str((robot_dir / child.get(attribute)).resolve()))
        scene.insert(0, child)
    for world in scene.findall("worldbody"):
        for floor in list(world.findall("geom")):
            if floor.get("type") == "plane" or floor.get("name") == "floor":
                world.remove(floor)
    # Soft daylight sky, with a matching horizon instead of the original black gradient.
    for asset in scene.findall("asset"):
        for texture in list(asset.findall("texture")):
            if texture.get("type") == "skybox":
                asset.remove(texture)
    visual = scene.find("visual")
    if visual is None:
        visual = ET.SubElement(scene, "visual")
    rgba = visual.find("rgba")
    if rgba is None:
        rgba = ET.SubElement(visual, "rgba")
    rgba.set("haze", "0.86 0.89 0.91 1")
    assets = ET.SubElement(scene, "asset")
    ET.SubElement(assets, "texture", name="terrain_daylight_sky", type="skybox", builtin="gradient",
                  rgb1="0.55 0.72 0.88", rgb2="0.92 0.92 0.88", width="512", height="3072")
    ET.SubElement(assets, "texture", name="terrain_sandstone_texture", type="2d", builtin="flat",
                  mark="random", rgb1="0.66 0.59 0.47", markrgb="0.58 0.51 0.40", random="0.035",
                  width="512", height="512")
    ET.SubElement(assets, "material", name="terrain_sandstone_ground", texture="terrain_sandstone_texture",
                  texuniform="true", texrepeat="0.5 0.5", reflectance="0", specular="0.04", shininess="0.05")
    world = ET.SubElement(scene, "worldbody")
    settings = json.loads(SETTINGS.read_text())
    def validate_numbers(values):
        for value in values.values():
            if isinstance(value, dict):
                validate_numbers(value)
                continue
            numbers = value if isinstance(value, list) else [value]
            if not numbers or any(not isinstance(number, (int, float)) or
                                  not math.isfinite(number) or number <= 0 for number in numbers):
                raise ValueError("Terrain dimensions must be finite positive numbers")

    validate_numbers(settings)
    if len(settings["stones"]["heights"]) != len(settings["stones"]["radii"]):
        raise ValueError("Each stepping stone requires a height and radius")
    if not isinstance(settings["stairs"]["steps"], int):
        raise ValueError("Stair step count must be an integer")
    if settings["lane_spacing"] < settings["lane_width"]:
        raise ValueError("lane_spacing must be at least lane_width to keep courses separate")
    ET.SubElement(world, "geom", name="terrain_pit_floor", type="plane",
                  pos=f"0 0 {-settings['pit_depth']}", size="0 0 .05",
                  material="terrain_sandstone_ground", friction="1 0.005 0.0001")
    if selection == "mixed":
        categories = ("platforms", "stones", "gaps", "stairs", "hurdles")
        # All lanes start at the same X; alternating Y keeps platforms at the spawn heading.
        offsets = [((index + 1) // 2) * settings["lane_spacing"] * (1 if index % 2 else -1)
                   for index in range(len(categories))]
        plaza_settings = {**settings, "lane_width": max(offsets) - min(offsets) + settings["lane_width"]}
        Course(world, plaza_settings, prefix="plaza_")
        sections = []
        for category, y in zip(categories, offsets):
            sections.extend(Course(world, settings, y=y, prefix=f"{category}_", add_start=False).build(category))
    else:
        sections = Course(world, settings).build(selection)
    scene.set("model", f"G1 {selection} course")
    ET.indent(scene, space="  ")
    ET.ElementTree(scene).write(destination, encoding="utf-8", xml_declaration=True)
    return sections
