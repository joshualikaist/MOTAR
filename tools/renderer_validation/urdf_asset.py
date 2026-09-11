"""Turn a URDF's visual geometry into a MeshScene, using only the standard library and numpy.

Scope, conventions and refusals are fixed in docs/renderer_urdf_loader_v1_contract.md. Nothing
here imports aerial_gym, a simulator, a physics engine or a URDF library: the point of the
prototype is that it shares no code with the system under study except audited ray kernels.

URDF rgba is carried through unconverted. That is a declared choice, not a claim that a display
colour equals a linear one; the raw values are kept so a conversion can be added as its own step.
"""
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ElementTree

import numpy as np

from .scene import MeshScene, frozen_array, integer_array

SPHERE_SEGMENTS = 16
SPHERE_RINGS = 8
CYLINDER_SEGMENTS = 16
DEFAULT_MATERIAL_NAME = "__urdf_default__"
DEFAULT_MATERIAL_RGBA = (0.5, 0.5, 0.5, 1.0)
SUPPORTED_GEOMETRY = ("box", "sphere", "cylinder")


class UrdfError(ValueError):
    """Every refusal in this module. Callers should not have to guess which failure occurred."""


def _floats(text, count, what):
    parts = (text or "").split()
    if len(parts) != count:
        raise UrdfError(f"{what} needs {count} numbers, got {len(parts)}: {text!r}")
    try:
        values = [float(p) for p in parts]
    except ValueError as error:
        raise UrdfError(f"{what} is not numeric: {text!r}") from error
    if not all(math.isfinite(v) for v in values):
        raise UrdfError(f"{what} is not finite: {text!r}")
    return values


def rotation_from_rpy(roll, pitch, yaw):
    """URDF fixed-axis roll-pitch-yaw: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]], dtype=np.float64)


def origin_transform(element):
    """A URDF <origin> as a 4x4 homogeneous transform; a missing origin is the identity."""
    if element is None:
        return np.eye(4)
    node = element if element.tag == "origin" else element.find("origin")
    if node is None:
        return np.eye(4)
    transform = np.eye(4)
    transform[:3, :3] = rotation_from_rpy(*_floats(node.get("rpy", "0 0 0"), 3, "origin rpy"))
    transform[:3, 3] = _floats(node.get("xyz", "0 0 0"), 3, "origin xyz")
    return transform


def _quad(indices, a, b, c, d):
    """Two triangles for a planar quad whose corners run counter-clockwise seen from outside."""
    indices.append((a, b, c))
    indices.append((a, c, d))


def box_mesh(size):
    sx, sy, sz = (v / 2.0 for v in size)
    points = [(-sx, -sy, -sz), (sx, -sy, -sz), (sx, sy, -sz), (-sx, sy, -sz),
              (-sx, -sy, sz), (sx, -sy, sz), (sx, sy, sz), (-sx, sy, sz)]
    faces = []
    _quad(faces, 0, 3, 2, 1)   # -Z
    _quad(faces, 4, 5, 6, 7)   # +Z
    _quad(faces, 0, 1, 5, 4)   # -Y
    _quad(faces, 2, 3, 7, 6)   # +Y
    _quad(faces, 0, 4, 7, 3)   # -X
    _quad(faces, 1, 2, 6, 5)   # +X
    return np.array(points, dtype=np.float64), faces


def sphere_mesh(radius, segments=SPHERE_SEGMENTS, rings=SPHERE_RINGS):
    """A UV sphere with triangle fans at the poles, so no degenerate triangle is emitted."""
    points = [(0.0, 0.0, radius)]
    for ring in range(1, rings):
        polar = math.pi * ring / rings
        for segment in range(segments):
            azimuth = 2.0 * math.pi * segment / segments
            points.append((radius * math.sin(polar) * math.cos(azimuth),
                           radius * math.sin(polar) * math.sin(azimuth),
                           radius * math.cos(polar)))
    south = len(points)
    points.append((0.0, 0.0, -radius))
    index = lambda ring, segment: 1 + (ring - 1) * segments + segment % segments
    faces = [(0, index(1, s), index(1, s + 1)) for s in range(segments)]
    for ring in range(1, rings - 1):
        for s in range(segments):
            _quad(faces, index(ring, s), index(ring + 1, s),
                  index(ring + 1, s + 1), index(ring, s + 1))
    faces += [(south, index(rings - 1, s + 1), index(rings - 1, s)) for s in range(segments)]
    return np.array(points, dtype=np.float64), faces


def cylinder_mesh(radius, length, segments=CYLINDER_SEGMENTS):
    """URDF convention: the axis is +Z and the origin is the centre of the cylinder."""
    half = length / 2.0
    ring = [(radius * math.cos(2.0 * math.pi * s / segments),
             radius * math.sin(2.0 * math.pi * s / segments)) for s in range(segments)]
    points = [(x, y, half) for x, y in ring] + [(x, y, -half) for x, y in ring]
    top, bottom = len(points), len(points) + 1
    points += [(0.0, 0.0, half), (0.0, 0.0, -half)]
    faces = []
    for s in range(segments):
        nxt = (s + 1) % segments
        _quad(faces, s, segments + s, segments + nxt, nxt)
        faces.append((top, s, nxt))
        faces.append((bottom, segments + nxt, segments + s))
    return np.array(points, dtype=np.float64), faces


def geometry_mesh(node):
    children = [child for child in node if child.tag in ("box", "sphere", "cylinder", "mesh")]
    if len(children) != 1:
        tags = [child.tag for child in node]
        raise UrdfError(f"<geometry> needs exactly one supported shape, found {tags}")
    shape = children[0]
    if shape.tag == "mesh":
        raise UrdfError("<mesh> is out of scope for URDF loader v1; see the contract document")
    if shape.tag == "box":
        size = _floats(shape.get("size"), 3, "box size")
        if min(size) <= 0.0:
            raise UrdfError(f"box size must be positive: {size}")
        return box_mesh(size), ("box", {"size": size})
    if shape.tag == "sphere":
        radius = _floats(shape.get("radius"), 1, "sphere radius")[0]
        if radius <= 0.0:
            raise UrdfError(f"sphere radius must be positive: {radius}")
        return sphere_mesh(radius), ("sphere", {"radius": radius})
    radius = _floats(shape.get("radius"), 1, "cylinder radius")[0]
    length = _floats(shape.get("length"), 1, "cylinder length")[0]
    if radius <= 0.0 or length <= 0.0:
        raise UrdfError(f"cylinder radius and length must be positive: {radius}, {length}")
    return cylinder_mesh(radius, length), ("cylinder", {"radius": radius, "length": length})


def link_world_transforms(links, joints):
    """Chain fixed-joint origins from the single root. Refuses cycles, orphans and extra roots."""
    children = {}
    for joint in joints:
        parent, child = joint["parent"], joint["child"]
        for name in (parent, child):
            if name not in links:
                raise UrdfError(f"joint {joint['name']!r} refers to unknown link {name!r}")
        if child in children:
            raise UrdfError(f"link {child!r} has more than one parent joint")
        children[child] = joint
    roots = [name for name in links if name not in children]
    if len(roots) != 1:
        raise UrdfError(f"exactly one root link is required, found {sorted(roots)}")
    transforms, order = {roots[0]: np.eye(4)}, [roots[0]]
    remaining = [joint for joint in joints]
    while remaining:
        progressed = [joint for joint in remaining if joint["parent"] in transforms]
        if not progressed:
            unreached = sorted(joint["child"] for joint in remaining)
            raise UrdfError(f"links unreachable from the root (cycle or split tree): {unreached}")
        for joint in progressed:
            transforms[joint["child"]] = transforms[joint["parent"]] @ joint["origin"]
            order.append(joint["child"])
            remaining.remove(joint)
    return transforms, order


@dataclass(frozen=True)
class UrdfAsset:
    mesh: MeshScene
    robot_name: str
    link_names: tuple
    material_names: tuple
    material_rgba: np.ndarray
    shapes: tuple
    source_path: str
    source_sha256: str
    default_material_visuals: int

    def __post_init__(self):
        object.__setattr__(self, "material_rgba", frozen_array(self.material_rgba, np.float64))
        if self.material_rgba.shape != (len(self.material_names), 4):
            raise UrdfError("One rgba row per material is required")
        if self.mesh.material_count > len(self.material_names):
            raise UrdfError("A face references a material that was never declared")

    @property
    def face_link(self):
        return self.mesh.face_instance

    def as_dict(self):
        return {"robot_name": self.robot_name, "link_names": list(self.link_names),
                "material_names": list(self.material_names),
                "material_rgba": self.material_rgba.tolist(),
                "shapes": [{"link": link, "kind": kind, "parameters": parameters,
                            "triangles": count} for link, kind, parameters, count in self.shapes],
                "source_path": self.source_path, "source_sha256": self.source_sha256,
                "default_material_visuals": self.default_material_visuals,
                "tessellation": {"sphere_segments": SPHERE_SEGMENTS, "sphere_rings": SPHERE_RINGS,
                                 "cylinder_segments": CYLINDER_SEGMENTS},
                "colour_note": "URDF rgba carried through unconverted; alpha ignored",
                "triangles": int(len(self.mesh.triangles)),
                "links_with_visual_geometry": int(len(set(self.mesh.face_instance.tolist())))}


def load_urdf_asset(path):
    """Read one URDF file into a MeshScene. Every unsupported construct raises UrdfError."""
    path = Path(path)
    raw = path.read_bytes()
    try:
        root = ElementTree.fromstring(raw)
    except ElementTree.ParseError as error:
        raise UrdfError(f"{path} is not well-formed XML: {error}") from error
    if root.tag != "robot":
        raise UrdfError(f"{path} root element is <{root.tag}>, expected <robot>")

    materials, rgba = {}, []

    def material_index(name, values):
        if name not in materials:
            materials[name] = len(rgba)
            rgba.append(values)
        return materials[name]

    for node in root.findall("material"):
        name = node.get("name")
        colour = node.find("color")
        if name and colour is not None:
            material_index(name, _floats(colour.get("rgba"), 4, f"material {name} rgba"))

    links = {}
    for node in root.findall("link"):
        name = node.get("name")
        if not name:
            raise UrdfError("every <link> needs a name")
        if name in links:
            raise UrdfError(f"duplicate link name {name!r}")
        links[name] = node

    joints = []
    for node in root.findall("joint"):
        kind = node.get("type")
        name = node.get("name") or "<unnamed>"
        if kind != "fixed":
            raise UrdfError(f"joint {name!r} is {kind!r}; loader v1 supports fixed joints only")
        parent, child = node.find("parent"), node.find("child")
        if parent is None or child is None:
            raise UrdfError(f"joint {name!r} needs <parent> and <child>")
        joints.append({"name": name, "parent": parent.get("link"), "child": child.get("link"),
                       "origin": origin_transform(node)})

    transforms, order = link_world_transforms(links, joints)
    vertices, triangles, face_material, face_link, shapes = [], [], [], [], []
    defaulted = 0
    for instance, link_name in enumerate(order):
        for visual in links[link_name].findall("visual"):
            geometry = visual.find("geometry")
            if geometry is None:
                raise UrdfError(f"link {link_name!r} has a <visual> with no <geometry>")
            (points, faces), (kind, parameters) = geometry_mesh(geometry)
            node = visual.find("material")
            if node is None:
                defaulted += 1
                index = material_index(DEFAULT_MATERIAL_NAME, list(DEFAULT_MATERIAL_RGBA))
            else:
                name = node.get("name") or ""
                colour = node.find("color")
                if colour is not None:
                    index = material_index(name or f"__anonymous_{len(rgba)}__",
                                           _floats(colour.get("rgba"), 4, f"material {name} rgba"))
                elif name in materials:
                    index = materials[name]
                else:
                    raise UrdfError(f"link {link_name!r} references undeclared material {name!r}")
            transform = transforms[link_name] @ origin_transform(visual)
            placed = points @ transform[:3, :3].T + transform[:3, 3]
            offset = len(vertices)
            vertices.extend(placed.tolist())
            triangles.extend([[offset + a, offset + b, offset + c] for a, b, c in faces])
            face_material.extend([index] * len(faces))
            face_link.extend([instance] * len(faces))
            shapes.append((link_name, kind, parameters, len(faces)))
    if not triangles:
        raise UrdfError(f"{path} declares no visual geometry this loader can use")
    return UrdfAsset(MeshScene(vertices, triangles, face_material, face_link),
                     root.get("name") or path.stem, tuple(order),
                     tuple(materials), np.array(rgba, dtype=np.float64), tuple(shapes),
                     str(path), hashlib.sha256(raw).hexdigest(), defaulted)


def outward_normal_violations(mesh, asset=None):
    """Triangles whose winding points into their own shape, per link centroid. Zero is required."""
    points = mesh.vertices[mesh.triangles]
    normals = np.cross(points[:, 1] - points[:, 0], points[:, 2] - points[:, 0])
    centroids = points.mean(axis=1)
    violations = 0
    for instance in sorted(set(mesh.face_instance.tolist())):
        selected = mesh.face_instance == instance
        centre = mesh.vertices[np.unique(mesh.triangles[selected])].mean(axis=0)
        violations += int((np.sum(normals[selected] * (centroids[selected] - centre), axis=1) <= 0).sum())
    return violations


def open_edges(mesh):
    """Edges not shared by exactly two triangles. A closed surface has none."""
    counts = {}
    for triangle in mesh.triangles.tolist():
        for i in range(3):
            edge = (triangle[i], triangle[(i + 1) % 3])
            counts[tuple(sorted(edge))] = counts.get(tuple(sorted(edge)), 0) + 1
    return sorted(edge for edge, count in counts.items() if count != 2)


def signed_volume(mesh, instance=None):
    """Divergence-theorem volume of one link's closed surface, for a convergence check."""
    selected = slice(None) if instance is None else (mesh.face_instance == instance)
    points = mesh.vertices[mesh.triangles[selected]].astype(np.float64)
    return float(np.sum(np.einsum("ij,ij->i", points[:, 0],
                                  np.cross(points[:, 1], points[:, 2]))) / 6.0)
