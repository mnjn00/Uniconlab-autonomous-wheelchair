"""Cryptographic binding for route, safety band, and drivable mask."""

import hashlib
import json
from pathlib import Path

import yaml


def sha256(path):
    payload = Path(path).read_bytes()
    if Path(path).suffix == ".json":
        payload = payload.rstrip(b"\r\n")
    return hashlib.sha256(payload).hexdigest()


def mask_image_path(mask_yaml):
    mask_yaml = Path(mask_yaml)
    metadata = yaml.safe_load(mask_yaml.read_text(encoding="utf-8"))
    return mask_yaml.parent / metadata["image"]


def validate_asset_binding(route_path, band_path, mask_yaml=None):
    route = json.loads(Path(route_path).read_text(encoding="utf-8"))
    band = json.loads(Path(band_path).read_text(encoding="utf-8"))
    binding = route.get("asset_binding")
    if not isinstance(binding, dict):
        raise ValueError("route has no asset_binding")
    if binding.get("safety_band_sha256") != sha256(band_path):
        raise ValueError("route/safety-band SHA-256 mismatch")
    if band.get("route_id") != binding.get("route_id"):
        raise ValueError("route/safety-band identity mismatch")
    if mask_yaml is not None:
        yaml_sha = sha256(mask_yaml)
        if binding.get("drivable_mask_yaml_sha256") != yaml_sha:
            raise ValueError("route/drivable-mask metadata SHA-256 mismatch")
        if band.get("drivable_mask_yaml_sha256") != yaml_sha:
            raise ValueError(
                "safety-band/drivable-mask metadata SHA-256 mismatch"
            )
        image_sha = sha256(mask_image_path(mask_yaml))
        if binding.get("drivable_mask_sha256") != image_sha:
            raise ValueError("route/drivable-mask SHA-256 mismatch")
        if band.get("drivable_mask_sha256") != image_sha:
            raise ValueError("safety-band/drivable-mask SHA-256 mismatch")
    return binding
