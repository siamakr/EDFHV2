#!/usr/bin/env python3
"""
Reads config.yaml and generates src/Config.h with a cfg:: namespace.

Type rules (automatic, no FIELD_MAP needed):
  - YAML int   -> constexpr int
  - YAML float -> constexpr double  (use a trailing dot in YAML for floats, e.g. 5.0)
  - YAML bool  -> constexpr bool

Special key suffix conventions (applied automatically):
  - key ends with _deg  -> constant renamed *_RAD, value converted to radians
  - key ends with _usec -> constexpr int
  - key ends with _msec -> constexpr double
  - key ends with _sec  -> constexpr double

Usage:
    python3 scripts/gen_config.py
    (also called automatically by PlatformIO pre-build hook)
"""

import yaml
import math
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent
YAML_PATH    = PROJECT_ROOT / "config.yaml"
OUTPUT_PATH  = PROJECT_ROOT / "src" / "Config.h"


def _section_comment(section_key: str) -> str:
    """Turn a snake_case section name into a readable comment."""
    return "// --- " + section_key.replace("_", " ").title() + " ---"


def _cpp_name(key: str) -> str:
    """Return the C++ constant name for a YAML leaf key."""
    name = key.upper()
    if name.endswith("_DEG"):
        name = name[:-4] + "_RAD"
    return name


def _cpp_type_and_literal(key: str, value) -> tuple[str, str]:
    """
    Infer the C++ type and produce a correctly suffixed literal.
    Degree keys are converted to radians at generation time.
    """
    # Convert degrees -> radians
    if key.endswith("_deg") and isinstance(value, (int, float)):
        rad = math.radians(float(value))
        return "double", repr(rad)

    if isinstance(value, bool):
        return "bool", "true" if value else "false"

    if isinstance(value, int):
        return "int", str(value)

    if isinstance(value, float):
        return "double", repr(value)

    # Fallback for strings (rarely needed in embedded configs)
    return "const char*", f'"{value}"'


def _walk(data: dict, lines: list, indent: str = "    "):
    """Recursively walk the YAML dict, emitting one section comment per nested dict."""
    for section_key, section_val in data.items():
        if isinstance(section_val, dict):
            lines.append("")
            lines.append(f"{indent}{_section_comment(section_key)}")
            _walk(section_val, lines, indent)
        else:
            cpp_type, literal = _cpp_type_and_literal(section_key, section_val)
            name = _cpp_name(section_key)
            lines.append(f"{indent}constexpr {cpp_type} {name} = {literal};")


def generate(yaml_path: Path, output_path: Path):
    with open(yaml_path) as f:
        data = yaml.safe_load(f)

    lines = [
        "// AUTO-GENERATED — do not edit by hand.",
        f"// Source: {yaml_path.name}  |  Script: scripts/gen_config.py",
        "#pragma once",
        "",
        "#include <math.h>",
        "",
        "namespace cfg {",
    ]

    _walk(data, lines)

    lines += [
        "",
        "    // --- Derived constants ---",
        "    constexpr double D2R = M_PI / 180.0;",
        "    constexpr double R2D = 180.0 / M_PI;",
        "",
        "}  // namespace cfg",
        "",
    ]

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines))
    print(f"Generated {output_path}")


if __name__ == "__main__":
    generate(YAML_PATH, OUTPUT_PATH)
