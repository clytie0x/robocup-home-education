"""Build a MuJoCo scene from this repo's SO-100 URDF.

MuJoCo cannot load the URDF as it is: the mesh paths are package:// URIs.
The fix used on 20 Nov is a <compiler meshdir> under <robot> and the prefix
stripped. The saved MJCF is then given actuators, finger pads, the deck, the
floor and a bag. Meshes stay in arm/so100_moveit_config/meshes. They are not copied.

MuJoCo 4.0.1 rejects the old <composite> cable ("unrecognized element").
Each handle is a dim-1 flexcomp rod, the cable that replaced composite.
"""

import math
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from . import constants as C
from .paths import kinematics_type, mesh_dir, urdf_path

# Contact bits. Adjacent arm meshes overlap, so the arm does not collide with itself.
_FLOOR = 1
_ARM = 2
_PAD = 4
_BAG = 8
_ROPE = 16


def _arm_xml():
    """URDF compiled once to MJCF, with package:// meshes resolved."""
    text = urdf_path().read_text()
    text = text.replace("package://so100_moveit_config/meshes/", "")
    compiler = (
        '<mujoco><compiler meshdir="%s" balanceinertia="true" '
        'discardvisual="false" fusestatic="false"/></mujoco>' % mesh_dir().resolve()
    )
    robot = text.find("<robot")
    end = text.find(">", robot)
    text = text[: end + 1] + "\n" + compiler + text[end + 1 :]
    model = mujoco.MjModel.from_xml_string(text)
    with tempfile.NamedTemporaryFile("w", suffix=".xml", delete=False) as handle:
        path = handle.name
    mujoco.mj_saveLastXML(path, model)
    return ET.parse(path).getroot()


def _quat_about_x(deg):
    half = math.radians(deg) / 2.0
    return "%.6f %.6f 0 0" % (math.cos(half), math.sin(half))


def _arch_points(y):
    """Handle arch in the bag frame. The rise is HANDLE_TOP - RIM_H.

    The body origin is at mid-height of the 0.27 m bag, so the rim is at local z +0.135.
    """
    n = 7
    rise = C.handle_rise()
    rim_local = C.RIM_H / 2.0
    pts = []
    elements = []
    for i in range(n):
        t = i / (n - 1)
        pts.extend((
            -0.04 + 0.08 * t,
            y,
            rim_local + rise * math.sin(math.pi * t),
        ))
    for i in range(n - 1):
        elements.extend((i, i + 1))
    point = " ".join("%.5f" % v for v in pts)
    element = " ".join(str(i) for i in elements)
    return point, element, n - 1


def _add_handle(bag, name, y, kind):
    point, element, last = _arch_points(y)
    attrib = {
        "name": name,
        "type": "direct",
        "dim": "1",
        "mass": "0.004",
        "point": point,
        "element": element,
    }
    if kind == "gift":
        attrib["radius"] = "%.4f" % C.RIBBON_HALF[1]
    else:
        attrib["radius"] = "%.4f" % C.ROPE_RADIUS
    flex = ET.SubElement(bag, "flexcomp", attrib)
    ET.SubElement(flex, "contact", {
        "condim": "4", "selfcollide": "none",
        "contype": str(_ROPE), "conaffinity": str(_PAD),
    })
    ET.SubElement(flex, "edge", {"equality": "true", "solref": "0.02 1"})
    elastic = {"young": "%.1e" % C.ROPE_YOUNG, "damping": "0.02"}
    if kind == "gift":
        elastic["section"] = "box"
        elastic["sectionsize"] = "%.4f %.4f" % C.RIBBON_HALF
    else:
        elastic["section"] = "circle"
    ET.SubElement(flex, "elasticity", elastic)
    ET.SubElement(flex, "pin", {"id": "0"})
    ET.SubElement(flex, "pin", {"id": str(last)})


def _add_bag(world, kin, kind):
    """Rigid bag. The upper front is a plate the jaws can pinch; the inside is empty.

    At pitch -60 deg the jaws close on a diagonal (30 deg from horizontal), so one
    finger is inside the bag and one is outside. A solid box the size of the whole
    bag would fill that space and the inner finger would hit it. The lower box is
    only the bottom 12 cm. The plate is tilted onto the closing axis and is
    PINCH_THICKNESS deep, so the real close command (jaw 0, gap 1.6 cm) stalls on it.
    That thickness stands in for paper that bunches into the jaws. It was not measured.
    """
    wall = np.array(kin.from_user(C.NOMINAL_WALL, 0.0))
    forward = np.array(kin.from_user(1.0, 0.0)) - np.array(kin.from_user(0.0, 0.0))
    centre = wall + C.HALF_DEPTH * forward
    origin_z = C.FLOOR_Z + C.RIM_H / 2.0
    bag = ET.SubElement(world, "body", {
        "name": "bag",
        "pos": "%.4f %.4f %.4f" % (centre[0], centre[1], origin_z),
    })
    ET.SubElement(bag, "freejoint", {"name": "bag_free"})
    ET.SubElement(bag, "geom", {
        "name": "bag_wall", "type": "box",
        "pos": "0 0 -0.075", "size": "0.09 0.055 0.060",
        "mass": "%.4f" % (C.BAG_MASS - 0.02),
        "rgba": "0.96 0.96 0.96 1" if kind == "paper" else "0.85 0.55 0.62 1",
        "friction": "0.8 0.005 0.0001",
        "contype": str(_BAG), "conaffinity": str(_ARM + _PAD),
    })
    half = C.PINCH_THICKNESS / 2.0
    ET.SubElement(bag, "geom", {
        "name": "rim", "type": "box",
        "pos": "0 0.068 0.050",
        "size": "0.07 %.4f 0.028" % half,
        "quat": _quat_about_x(-30.0),
        "mass": "0.02",
        "rgba": "0.93 0.90 0.75 1",
        "friction": "2 0.1 0.02",
        "condim": "6", "solref": "0.01 1", "solimp": "0.95 0.99 0.001",
        "contype": str(_BAG), "conaffinity": str(_ARM + _PAD),
    })
    # Front rope sits just outside the plate. Back rope sits on the back rim.
    _add_handle(bag, "rope_front", 0.078, kind)
    _add_handle(bag, "rope_back", -0.062, kind)


def _add_world(world, kin):
    corners = np.array([
        kin.from_user(forward, left)
        for forward in (-C.DECK_BACK, C.DECK_FRONT)
        for left in (-C.DECK_RIGHT, C.DECK_LEFT)
    ])
    low, high = corners.min(axis=0), corners.max(axis=0)
    centre = (low + high) / 2.0
    half = (high - low) / 2.0
    deck_half_z = (C.DECK_Z - C.FLOOR_Z) / 2.0
    deck_z = (C.DECK_Z + C.FLOOR_Z) / 2.0
    ET.SubElement(world, "geom", {
        "name": "floor", "type": "plane",
        "pos": "0 0 %.4f" % C.FLOOR_Z, "size": "2 2 0.1",
        "rgba": "0.62 0.62 0.58 1", "friction": "1 0.005 0.0001",
        "contype": str(_FLOOR), "conaffinity": str(_ARM + _BAG),
    })
    ET.SubElement(world, "geom", {
        "name": "deck", "type": "box",
        "pos": "%.4f %.4f %.4f" % (centre[0], centre[1], deck_z),
        "size": "%.4f %.4f %.4f" % (half[0], half[1], deck_half_z),
        "rgba": "0.25 0.25 0.28 1",
        "contype": str(_FLOOR), "conaffinity": str(_ARM),
    })


def _add_pads(root):
    """Capsules would be round. The pads are thin boxes on the inner faces measured
    from Fixed_Jaw.stl and Moving_Jaw.stl: fixed face near gripper x = 0.008,
    moving face near jaw-local x = -0.012, both at the fingertip."""
    bodies = {body.get("name"): body for body in root.iter("body")}
    pad = {
        "type": "box", "mass": "0.001", "rgba": "0.12 0.12 0.12 1",
        "friction": "2 0.1 0.01", "condim": "6",
        "contype": str(_PAD), "conaffinity": str(_BAG + _ROPE),
    }
    ET.SubElement(bodies["gripper"], "geom", dict(pad, **{
        "name": "pad_fixed", "pos": "0.0095 -0.098 0", "size": "0.002 0.014 0.008",
    }))
    ET.SubElement(bodies["jaw"], "geom", dict(pad, **{
        "name": "pad_jaw", "pos": "-0.0105 -0.078 0", "size": "0.002 0.012 0.008",
    }))
    for body in root.iter("body"):
        for geom in list(body):
            if geom.tag != "geom" or geom.get("type") != "mesh":
                continue
            if geom.get("contype") == "0":
                continue
            if body.get("name") in ("gripper", "jaw"):
                geom.set("contype", "0")
                geom.set("conaffinity", "0")
                continue
            geom.set("name", "col_%s" % body.get("name"))
            geom.set("contype", str(_ARM))
            geom.set("conaffinity", str(_FLOOR + _BAG))


def _add_actuators(root):
    # PLACEHOLDER gains. See constants.ACTUATOR_*.
    act = ET.SubElement(root, "actuator")
    note = ET.Comment(
        " PLACEHOLDER position actuators. kp/kv/forcerange are not a measured "
        "STS3215 torque. Joint speed is capped in the executor at the MoveIt limits. "
    )
    act.append(note)
    for name in C.ALL_JOINTS:
        ET.SubElement(act, "position", {
            "name": "act_%s" % name, "joint": name,
            "kp": "%.1f" % C.ACTUATOR_KP, "kv": "%.1f" % C.ACTUATOR_KV,
            "forcerange": "%.1f %.1f" % (-C.ACTUATOR_FORCE, C.ACTUATOR_FORCE),
        })


def build_xml(kind="paper"):
    if kind not in ("paper", "gift"):
        raise ValueError("bag kind must be paper or gift, got %r" % kind)
    root = _arm_xml()
    kin = kinematics_type()(urdf_path().read_text())
    _add_pads(root)
    world = root.find("worldbody")
    _add_world(world, kin)
    _add_bag(world, kin, kind)
    _add_actuators(root)
    ET.SubElement(root, "option", {
        "timestep": "%.4f" % C.PHYSICS_DT,
        "integrator": "discrete",
        "solver": "Newton",
        "iterations": "40",
        "tolerance": "1e-8",
    })
    return ET.tostring(root, encoding="unicode"), kin


def load_scene(kind="paper"):
    xml, kin = build_xml(kind)
    model = mujoco.MjModel.from_xml_string(xml)
    return model, kin


def joint_id(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)


def geom_id(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)


def actuator_id(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_%s" % name)


def qpos_of(model, data, name):
    joint = joint_id(model, name)
    return float(data.qpos[model.jnt_qposadr[joint]])


def qvel_of(model, data, name):
    joint = joint_id(model, name)
    return float(data.qvel[model.jnt_dofadr[joint]])


def bag_bottom(model, data):
    """World z of the lowest corner of the lower box, then height above the floor plane."""
    gid = geom_id(model, "bag_wall")
    centre = data.geom_xpos[gid]
    rotation = data.geom_xmat[gid].reshape(3, 3)
    half = model.geom_size[gid]
    lowest = math.inf
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                corner = centre + rotation @ (half * np.array((sx, sy, sz)))
                lowest = min(lowest, float(corner[2]))
    floor = float(model.geom_pos[geom_id(model, "floor")][2])
    return lowest - floor


def set_controls(model, data, targets):
    for name, value in targets.items():
        data.ctrl[actuator_id(model, name)] = float(value)
