#!/usr/bin/env python3
"""Deterministic contracts for the PGM-derived Hanyang Gazebo wall model."""

import hashlib
import importlib.util
import sys
import tempfile
from decimal import Decimal
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[3]
GENERATOR_PATH = ROOT / "scripts" / "generate_hanyang_gazebo_model.py"
MAP_YAML = ROOT / "data" / "hanyang_aegimun_loop" / "map.yaml"
MAP_PGM = MAP_YAML.with_name("map.pgm")
MODEL_DIR = ROOT / "src" / "wheelchair_gazebo" / "models" / "hanyang_aegimun_occupancy_walls"
MODEL_SDF = MODEL_DIR / "model.sdf"
MODEL_MESH = MODEL_DIR / "meshes" / "occupancy_walls.obj"
MODEL_URI = "model://hanyang_aegimun_occupancy_walls"
MESH_URI = MODEL_URI + "/meshes/occupancy_walls.obj"
WORLD_NAMES = (
    "empty.world", "road_free_space.world", "sidewalk_obstacles.world",
    "static_dynamic_obstacles.world", "wheelchair_rc_scenarios.world",
)
PACKAGE = ROOT / "src" / "wheelchair_gazebo"
SDF_SHA256 = "d2e1e1742e7d09c59d78cf28ee2ad38f43721772de9535cab793515ccc90be6f"
MESH_SHA256 = "50730156c13b3c0967256129329fdb978a51d1fc5634248610f1ebe388fe725e"


def load_generator():
    spec = importlib.util.spec_from_file_location("hanyang_gazebo_model_generator_test", str(GENERATOR_PATH))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


generator = load_generator()


def source_contract():
    yaml_data = MAP_YAML.read_bytes()
    metadata = generator.parse_simple_yaml(yaml_data)
    pgm_data = MAP_PGM.read_bytes()
    width, height, raster = generator.read_pgm(pgm_data)
    occupied, occupied_count = generator.occupied_cells(raster, width, height)
    segments = generator.boundary_segments(occupied, width, height)
    return yaml_data, metadata, pgm_data, width, height, raster, occupied, occupied_count, segments


def generated_lines():
    yaml_data, metadata, pgm_data, width, height, _, _, occupied_count, segments = source_contract()
    resolution = generator.decimal_value(metadata["resolution"], "resolution")
    origin = generator.parse_origin(metadata["origin"])
    sdf = generator.sdf_lines(
        hashlib.sha256(yaml_data).hexdigest(), hashlib.sha256(pgm_data).hexdigest(),
        segments, width, height, resolution, origin, occupied_count,
    )
    return sdf, generator.obj_lines(segments, resolution, origin)


def test_committed_pgm_identity_and_coalesced_boundary_are_exact():
    _, metadata, pgm_data, width, height, raster, occupied, occupied_count, segments = source_contract()
    expected_edges = set()
    for row, column in occupied:
        bottom = height - row - 1
        if row == height - 1 or (row + 1, column) not in occupied:
            expected_edges.add(("h", bottom, column))
        if row == 0 or (row - 1, column) not in occupied:
            expected_edges.add(("h", bottom + 1, column))
        if column == 0 or (row, column - 1) not in occupied:
            expected_edges.add(("v", column, bottom))
        if column == width - 1 or (row, column + 1) not in occupied:
            expected_edges.add(("v", column + 1, bottom))

    actual_edges = set()
    for x0, y0, x1, y1 in segments:
        if y0 == y1:
            actual_edges.update(("h", y0, x) for x in range(x0, x1))
        else:
            actual_edges.update(("v", x0, y) for y in range(y0, y1))

    assert hashlib.sha256(pgm_data).hexdigest() == generator.EXPECTED_PGM_SHA256
    assert (width, height) == (3220, 2361)
    assert set(raster) == {0, 205, 254}
    assert occupied_count == len(occupied) == 210223
    assert len(segments) == 32712
    assert actual_edges == expected_edges
    assert metadata["resolution"] == "0.1"
    assert metadata["origin"] == "[-60.1, -148.4, 0.0]"


def test_assets_are_byte_identical_and_repeatable():
    with tempfile.TemporaryDirectory() as directory:
        temporary = Path(directory)
        sdf_lines, obj_lines = generated_lines()
        first = temporary / "first"
        generator.write_atomic_pair(first / "model.sdf", sdf_lines, first / "meshes" / "occupancy_walls.obj", obj_lines)
        sdf_lines, obj_lines = generated_lines()
        second = temporary / "second"
        generator.write_atomic_pair(second / "model.sdf", sdf_lines, second / "meshes" / "occupancy_walls.obj", obj_lines)
        generated_sdf = (first / "model.sdf").read_bytes()
        generated_mesh = (first / "meshes" / "occupancy_walls.obj").read_bytes()
        assert generated_sdf == (second / "model.sdf").read_bytes() == MODEL_SDF.read_bytes()
        assert generated_mesh == (second / "meshes" / "occupancy_walls.obj").read_bytes() == MODEL_MESH.read_bytes()
        assert hashlib.sha256(generated_sdf).hexdigest() == SDF_SHA256
        assert hashlib.sha256(generated_mesh).hexdigest() == MESH_SHA256


def test_sdf_has_one_identity_mesh_collision_and_visual():
    root = ET.parse(MODEL_SDF).getroot()
    model = root.find("model")
    assert model is not None and model.attrib["name"] == generator.MODEL_NAME
    assert model.findtext("static") == "true"
    assert model.findtext("pose") == "0 0 0 0 0 0"
    assert not model.findall(".//plugin")
    link = model.find("link")
    assert link is not None and link.findtext("pose") == "0 0 0 0 0 0"
    assert len(link.findall("collision")) == len(link.findall("visual")) == 1
    assert len(model.findall(".//mesh")) == 2
    assert not model.findall(".//box")
    assert [item.text for item in model.findall(".//mesh/uri")] == [MESH_URI, MESH_URI]


def test_mesh_is_exact_coalesced_boundary_with_paired_quad_faces():
    vertices = []
    faces = []
    for line in MODEL_MESH.read_text(encoding="ascii").splitlines():
        fields = line.split()
        if fields and fields[0] == "v":
            vertices.append(tuple(Decimal(value) for value in fields[1:]))
        elif fields and fields[0] == "f":
            faces.append(tuple(int(value) for value in fields[1:]))

    _, _, _, _, _, _, _, _, segments = source_contract()
    assert len(vertices) == 65424
    assert len(faces) == 65424
    assert len(faces) == len(segments) * 2
    assert all(len(face) == 4 and all(1 <= index <= len(vertices) for index in face) for face in faces)
    mesh_segments = []
    for front, back in zip(faces[::2], faces[1::2]):
        assert back == tuple(reversed(front))
        points = [vertices[index - 1] for index in front]
        assert {point[2] for point in points} == {Decimal("0"), Decimal("2")}
        xs = {point[0] for point in points}
        ys = {point[1] for point in points}
        assert len(xs) == 1 or len(ys) == 1
        if len(xs) == 1:
            x = int((next(iter(xs)) - Decimal("-60.1")) / Decimal("0.1"))
            y0, y1 = sorted(int((value - Decimal("-148.4")) / Decimal("0.1")) for value in ys)
            mesh_segments.append((x, y0, x, y1))
        else:
            y = int((next(iter(ys)) - Decimal("-148.4")) / Decimal("0.1"))
            x0, x1 = sorted(int((value - Decimal("-60.1")) / Decimal("0.1")) for value in xs)
            mesh_segments.append((x0, y, x1, y))
    assert len(mesh_segments) == len(set(mesh_segments)) == len(segments)
    assert set(mesh_segments) == set(segments)


def test_generator_refuses_arbitrary_output_targets():
    with tempfile.TemporaryDirectory() as directory:
        arbitrary = Path(directory) / "model.sdf"
        try:
            generator.ensure_safe_output(ROOT, arbitrary, MODEL_SDF)
        except generator.GenerationError as error:
            assert "unsafe output path" in str(error)
        else:
            raise AssertionError("arbitrary output target was accepted")

def test_every_qualification_world_includes_one_identity_map_model():
    for name in WORLD_NAMES:
        world = ET.parse(PACKAGE / "worlds" / name).getroot().find("world")
        includes = [item for item in world.findall("include") if item.findtext("uri") == MODEL_URI]
        assert len(includes) == 1, name
        assert includes[0].find("pose") is None and includes[0].find("scale") is None


def test_catkin_installs_and_exports_the_model_directory():
    cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
    package_xml = ET.parse(PACKAGE / "package.xml").getroot()
    export = package_xml.find("export")
    gazebo = export.find("gazebo_ros") if export is not None else None
    assert "install(DIRECTORY config launch worlds models" in cmake
    assert gazebo is not None and gazebo.attrib.get("gazebo_model_path") == "${prefix}/models"
    assert (MODEL_DIR / "model.config").is_file()
