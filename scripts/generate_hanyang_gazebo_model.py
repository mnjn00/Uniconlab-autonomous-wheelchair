#!/usr/bin/env python3
"""Generate the Gazebo collision model for the committed Hanyang occupancy map."""

import argparse
import hashlib
import math
import os
from decimal import Decimal, InvalidOperation
from pathlib import Path
import re
import tempfile
from typing import Dict, Iterable, List, Sequence, Set, Tuple


EXPECTED_PGM_SHA256 = "c89d791f71fe3d1705ae04724acf8ff6ba0ccc351fc162fe996982f9469a0278"
EXPECTED_WIDTH = 3220
EXPECTED_HEIGHT = 2361
EXPECTED_RESOLUTION = Decimal("0.1")
EXPECTED_ORIGIN = (Decimal("-60.1"), Decimal("-148.4"), Decimal("0"))
EXPECTED_SOURCE_OCCUPIED_POINTS = 16629
EXPECTED_FINAL_OCCUPIED_CELLS = 210223
EXPECTED_PIXEL_VALUES = {0, 205, 254}
WALL_HEIGHT = Decimal("2.0")
MODEL_NAME = "hanyang_aegimun_occupancy_walls"


class GenerationError(ValueError):
    """Raised when the committed inputs do not satisfy the model contract."""


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def parse_simple_yaml(data: bytes) -> Dict[str, str]:
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise GenerationError("map YAML is not UTF-8") from exc

    values: Dict[str, str] = {}
    for line_number, line in enumerate(text.splitlines(), 1):
        content = line.split("#", 1)[0].strip()
        if not content:
            continue
        match = re.fullmatch(r"([A-Za-z_][A-Za-z0-9_]*):\s*(.*?)\s*", content)
        if match is None or not match.group(2):
            raise GenerationError("unsupported map YAML syntax on line {}".format(line_number))
        key, value = match.groups()
        if key in values:
            raise GenerationError("duplicate map YAML key: {}".format(key))
        values[key] = value
    return values


def decimal_value(value: str, field: str) -> Decimal:
    try:
        parsed = Decimal(value)
    except InvalidOperation as exc:
        raise GenerationError("invalid numeric {}: {}".format(field, value)) from exc
    if not parsed.is_finite():
        raise GenerationError("nonfinite geometry in {}".format(field))
    return parsed


def parse_origin(value: str) -> Tuple[Decimal, Decimal, Decimal]:
    match = re.fullmatch(r"\[\s*([^,]+),\s*([^,]+),\s*([^,]+)\s*\]", value)
    if match is None:
        raise GenerationError("origin must be a three-element inline sequence")
    return tuple(decimal_value(item.strip(), "origin") for item in match.groups())  # type: ignore


def read_pgm(data: bytes) -> Tuple[int, int, bytes]:
    position = 0
    length = len(data)

    def token() -> bytes:
        nonlocal position
        while position < length:
            if data[position] in b" \t\r\n\v\f":
                position += 1
            elif data[position] == ord("#"):
                newline = data.find(b"\n", position)
                if newline < 0:
                    raise GenerationError("unterminated PGM header comment")
                position = newline + 1
            else:
                break
        start = position
        while position < length and data[position] not in b" \t\r\n\v\f#":
            position += 1
        if start == position:
            raise GenerationError("truncated PGM header")
        return data[start:position]

    magic = token()
    if magic != b"P5":
        raise GenerationError("unsupported PGM mode: expected binary P5")
    try:
        width = int(token())
        height = int(token())
        maxval = int(token())
    except ValueError as exc:
        raise GenerationError("invalid PGM dimensions or maxval") from exc
    if width <= 0 or height <= 0:
        raise GenerationError("PGM dimensions must be positive")
    if maxval != 255:
        raise GenerationError("unsupported PGM maxval: {}".format(maxval))
    if position >= length or data[position] not in b" \t\r\n\v\f":
        raise GenerationError("PGM header is missing the raster separator")
    if data[position:position + 2] == b"\r\n":
        position += 2
    else:
        position += 1
    raster = data[position:]
    expected_size = width * height
    if len(raster) != expected_size:
        raise GenerationError(
            "bad PGM raster size: expected {}, got {}".format(expected_size, len(raster))
        )
    return width, height, raster


def occupied_cells(
    raster: bytes, width: int, height: int
) -> Tuple[Set[Tuple[int, int]], int]:
    occupied = {
        (row, column)
        for row in range(height)
        for column in range(width)
        if raster[row * width + column] == 0
    }
    return occupied, len(occupied)


def boundary_segments(
    occupied: Set[Tuple[int, int]], width: int, height: int
) -> List[Tuple[int, int, int, int]]:
    horizontal = set()
    vertical = set()
    for row, column in occupied:
        bottom = height - row - 1
        if row == height - 1 or (row + 1, column) not in occupied:
            horizontal.add((bottom, column))
        if row == 0 or (row - 1, column) not in occupied:
            horizontal.add((bottom + 1, column))
        if column == 0 or (row, column - 1) not in occupied:
            vertical.add((column, bottom))
        if column == width - 1 or (row, column + 1) not in occupied:
            vertical.add((column + 1, bottom))

    segments = []
    for y in range(height + 1):
        start = None
        for x in range(width + 1):
            if (y, x) in horizontal:
                if start is None:
                    start = x
            elif start is not None:
                segments.append((start, y, x, y))
                start = None
    for x in range(width + 1):
        start = None
        for y in range(height + 1):
            if (x, y) in vertical:
                if start is None:
                    start = y
            elif start is not None:
                segments.append((x, start, x, y))
                start = None
    return segments


def format_decimal(value: Decimal) -> str:
    if not value.is_finite():
        raise GenerationError("nonfinite generated geometry")
    if value == 0:
        return "0"
    result = format(value, "f")
    if "." in result:
        result = result.rstrip("0").rstrip(".")
    return result


def sdf_lines(
    yaml_sha256: str,
    pgm_sha256: str,
    segments: Sequence[Tuple[int, int, int, int]],
    width: int,
    height: int,
    resolution: Decimal,
    origin: Tuple[Decimal, Decimal, Decimal],
    occupied_count: int,
) -> Iterable[str]:
    mesh_uri = "model://{}/meshes/occupancy_walls.obj".format(MODEL_NAME)
    yield '<?xml version="1.0"?>\n'
    yield '<!-- Generated by scripts/generate_hanyang_gazebo_model.py; do not edit. -->\n'
    yield '<!-- map_yaml_sha256={} -->\n'.format(yaml_sha256)
    yield '<!-- map_pgm_sha256={} -->\n'.format(pgm_sha256)
    yield '<!-- dimensions={}x{} resolution={} origin=[{},{},{}] -->\n'.format(
        width, height, format_decimal(resolution),
        format_decimal(origin[0]), format_decimal(origin[1]), format_decimal(origin[2]),
    )
    yield '<!-- occupied_value=0 free_value=254 unknown_value=205 source_occupied_points={} final_occupied_cells={} boundary_segments={} boundary_faces={} wall_height={} -->\n'.format(
        EXPECTED_SOURCE_OCCUPIED_POINTS, occupied_count, len(segments), len(segments) * 2,
        format_decimal(WALL_HEIGHT)
    )
    yield '<sdf version="1.6">\n'
    yield '  <model name="{}">\n'.format(MODEL_NAME)
    yield '    <static>true</static>\n'
    yield '    <pose>0 0 0 0 0 0</pose>\n'
    yield '    <link name="occupancy_walls">\n'
    yield '      <pose>0 0 0 0 0 0</pose>\n'
    yield '      <collision name="occupancy_walls_collision">\n'
    yield '        <geometry><mesh><uri>{}</uri></mesh></geometry>\n'.format(mesh_uri)
    yield '      </collision>\n'
    yield '      <visual name="occupancy_walls_visual">\n'
    yield '        <geometry><mesh><uri>{}</uri></mesh></geometry>\n'.format(mesh_uri)
    yield '        <material><ambient>0.45 0.45 0.45 1</ambient><diffuse>0.55 0.55 0.55 1</diffuse></material>\n'
    yield '      </visual>\n'
    yield '    </link>\n'
    yield '  </model>\n'
    yield '</sdf>\n'


def obj_lines(
    segments: Sequence[Tuple[int, int, int, int]],
    resolution: Decimal,
    origin: Tuple[Decimal, Decimal, Decimal],
) -> Iterable[str]:
    yield "# Generated by scripts/generate_hanyang_gazebo_model.py; do not edit.\n"
    yield "# Exact coalesced occupied-cell boundary; paired quad faces.\n"
    vertex_indices = {}
    vertices = []
    faces = []
    for x0, y0, x1, y1 in segments:
        face = []
        for grid_x, grid_y, top in (
            (x0, y0, 0),
            (x1, y1, 0),
            (x1, y1, 1),
            (x0, y0, 1),
        ):
            key = (grid_x, grid_y, top)
            index = vertex_indices.get(key)
            if index is None:
                index = len(vertices) + 1
                vertex_indices[key] = index
                x = origin[0] + Decimal(grid_x) * resolution
                y = origin[1] + Decimal(grid_y) * resolution
                z = origin[2] + (WALL_HEIGHT if top else Decimal(0))
                vertices.append((x, y, z))
            face.append(index)
        faces.append(tuple(face))
    for x, y, z in vertices:
        yield "v {} {} {}\n".format(
            format_decimal(x), format_decimal(y), format_decimal(z)
        )
    for face in faces:
        yield "f {} {} {} {}\n".format(*face)
        yield "f {} {} {} {}\n".format(*reversed(face))


def ensure_safe_output(repo_root: Path, output: Path, expected_output: Path) -> None:
    unresolved = output if output.is_absolute() else repo_root / output
    if unresolved.is_symlink():
        raise GenerationError("unsafe output path: output is a symbolic link")
    resolved = unresolved.resolve(strict=False)
    expected = expected_output.resolve(strict=False)
    try:
        common = Path(os.path.commonpath((str(repo_root.resolve()), str(resolved))))
    except ValueError as exc:
        raise GenerationError("unsafe output path") from exc
    if common != repo_root.resolve() or resolved != expected:
        raise GenerationError("unsafe output path: must be {}".format(expected))
    current = resolved.parent
    while current != repo_root.resolve():
        if current.exists() and current.is_symlink():
            raise GenerationError("unsafe output path: symbolic-link parent")
        current = current.parent


def write_atomic_pair(
    sdf_output: Path,
    sdf_content: Iterable[str],
    mesh_output: Path,
    mesh_content: Iterable[str],
) -> None:
    if sdf_output.parent != mesh_output.parent.parent:
        raise GenerationError("SDF and mesh outputs must share one model directory")
    mesh_output.parent.mkdir(parents=True, exist_ok=True)
    temporary_names = []
    replacements = []
    try:
        for output, lines, prefix in (
            (sdf_output, sdf_content, ".model.sdf."),
            (mesh_output, mesh_content, ".occupancy_walls.obj."),
        ):
            with tempfile.NamedTemporaryFile(
                mode="w", encoding="ascii", newline="\n", dir=str(output.parent),
                prefix=prefix, delete=False
            ) as temporary:
                temporary_names.append(temporary.name)
                for line in lines:
                    temporary.write(line)
                temporary.flush()
                os.fsync(temporary.fileno())
            replacements.append((temporary.name, output))
        for temporary_name, output in replacements:
            os.replace(temporary_name, str(output))
            temporary_names.remove(temporary_name)
        for directory in (mesh_output.parent, sdf_output.parent):
            directory_fd = os.open(str(directory), os.O_RDONLY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
    finally:
        for temporary_name in temporary_names:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    default_yaml = repo_root / "data" / "hanyang_aegimun_loop" / "map.yaml"
    default_output = (
        repo_root / "src" / "wheelchair_gazebo" / "models" /
        MODEL_NAME / "model.sdf"
    )
    default_mesh_output = default_output.parent / "meshes" / "occupancy_walls.obj"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-yaml", type=Path, default=default_yaml)
    parser.add_argument("--output", type=Path, default=default_output)
    parser.add_argument("--mesh-output", type=Path, default=default_mesh_output)
    args = parser.parse_args()

    map_yaml = args.map_yaml if args.map_yaml.is_absolute() else repo_root / args.map_yaml
    if map_yaml.resolve(strict=False) != default_yaml.resolve(strict=False):
        raise GenerationError("map YAML must be the committed Hanyang map")
    ensure_safe_output(repo_root, args.output, default_output)
    ensure_safe_output(repo_root, args.mesh_output, default_mesh_output)
    output = args.output if args.output.is_absolute() else repo_root / args.output
    mesh_output = (
        args.mesh_output if args.mesh_output.is_absolute() else repo_root / args.mesh_output
    )

    yaml_data = map_yaml.read_bytes()
    metadata = parse_simple_yaml(yaml_data)
    required = {
        "image", "mode", "resolution", "origin", "negate", "occupied_thresh", "free_thresh"
    }
    missing = sorted(required.difference(metadata))
    if missing:
        raise GenerationError("missing map YAML keys: {}".format(", ".join(missing)))
    if metadata["mode"] != "trinary":
        raise GenerationError("unsupported map mode: {}".format(metadata["mode"]))
    if metadata["image"] != "map.pgm":
        raise GenerationError("unexpected map image: {}".format(metadata["image"]))
    if metadata["negate"] != "0":
        raise GenerationError("unsupported map negate value: {}".format(metadata["negate"]))

    resolution = decimal_value(metadata["resolution"], "resolution")
    origin = parse_origin(metadata["origin"])
    occupied_thresh = decimal_value(metadata["occupied_thresh"], "occupied_thresh")
    free_thresh = decimal_value(metadata["free_thresh"], "free_thresh")
    if resolution != EXPECTED_RESOLUTION or resolution <= 0:
        raise GenerationError("bad map resolution")
    if origin[2] != 0:
        raise GenerationError("nonzero map yaw is unsupported")
    if origin != EXPECTED_ORIGIN:
        raise GenerationError("bad map origin")
    if occupied_thresh != Decimal("0.65") or free_thresh != Decimal("0.25"):
        raise GenerationError("bad map occupancy thresholds")

    pgm_path = map_yaml.parent / metadata["image"]
    if pgm_path.resolve(strict=False) != (default_yaml.parent / "map.pgm").resolve(strict=False):
        raise GenerationError("unsafe map image path")
    pgm_data = pgm_path.read_bytes()
    pgm_sha256 = sha256_bytes(pgm_data)
    if pgm_sha256 != EXPECTED_PGM_SHA256:
        raise GenerationError("bad committed PGM SHA-256: {}".format(pgm_sha256))
    width, height, raster = read_pgm(pgm_data)
    if (width, height) != (EXPECTED_WIDTH, EXPECTED_HEIGHT):
        raise GenerationError("bad PGM dimensions: {}x{}".format(width, height))
    pixel_values = set(raster)
    if pixel_values != EXPECTED_PIXEL_VALUES:
        raise GenerationError("bad PGM pixel semantics: {}".format(sorted(pixel_values)))

    occupied, occupied_count = occupied_cells(raster, width, height)
    segments = boundary_segments(occupied, width, height)
    if occupied_count != EXPECTED_FINAL_OCCUPIED_CELLS:
        raise GenerationError("bad occupied-cell count: {}".format(occupied_count))
    if not segments:
        raise GenerationError("occupied-cell boundary is empty")

    write_atomic_pair(
        output,
        sdf_lines(
            sha256_bytes(yaml_data), pgm_sha256, segments, width, height,
            resolution, origin, occupied_count
        ),
        mesh_output,
        obj_lines(segments, resolution, origin),
    )
    model_sha256 = sha256_bytes(output.read_bytes())
    mesh_sha256 = sha256_bytes(mesh_output.read_bytes())
    print("output={}".format(output.relative_to(repo_root)))
    print("mesh_output={}".format(mesh_output.relative_to(repo_root)))
    print("source_occupied_points={}".format(EXPECTED_SOURCE_OCCUPIED_POINTS))
    print("final_occupied_cells={}".format(occupied_count))
    print("boundary_segments={}".format(len(segments)))
    print("vertices={}".format(len(segments) * 2))
    print("faces={}".format(len(segments) * 2))
    print("model_sha256={}".format(model_sha256))
    print("mesh_sha256={}".format(mesh_sha256))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (GenerationError, OSError) as error:
        raise SystemExit("error: {}".format(error))
