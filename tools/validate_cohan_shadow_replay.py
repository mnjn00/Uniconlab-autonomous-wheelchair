#!/usr/bin/env python3
"""Validate captured advisory CoHAN/HATEB replay evidence."""

from __future__ import annotations

import argparse
import importlib
import json
import sys
from pathlib import Path
from typing import Callable, cast

ROOT = Path(__file__).parents[1]
SCRIPTS = ROOT / "src" / "static_livox_localization" / "scripts"
sys.path.insert(0, str(SCRIPTS))
try:
    contract = importlib.import_module("shadow_qa_contract")
    validate_human_aware_replay = cast(
        Callable[[dict[str, object]], dict[str, object]],
        contract.validate_human_aware_replay,
    )
finally:
    _ = sys.path.pop(0)


def main():
    parser = argparse.ArgumentParser()
    _ = parser.add_argument("--input", required=True)
    _ = parser.add_argument("--output", required=True)
    args = parser.parse_args()
    input_path = Path(cast(str, args.input))
    output_path = Path(cast(str, args.output))
    replay = cast(
        dict[str, object],
        json.loads(input_path.read_text(encoding="utf-8")),
    )
    evidence: dict[str, object] = {
        "status": "PASS",
        **validate_human_aware_replay(replay),
    }
    rendered = json.dumps(evidence, indent=2, sort_keys=True) + "\n"
    _ = output_path.write_text(rendered, encoding="utf-8")
    print(rendered, end="")


if __name__ == "__main__":
    main()
