#!/usr/bin/env python3
"""
Generate ESP Security 2 (SRP6a) salt/verifier from username + password.

This script uses the ESP-IDF reference Python implementation to ensure
compatibility with ESP provisioning Security 2.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys
from pathlib import Path


def _format_c_array(name: str, data: bytes, indent: int = 4, wrap: int = 16) -> str:
    pad = " " * indent
    lines = []
    for i in range(0, len(data), wrap):
        chunk = data[i : i + wrap]
        line = ", ".join(f"0x{b:02x}" for b in chunk)
        lines.append(f"{pad}{line}")
    body = ",\n".join(lines)
    return f"static const char {name}[] = {{\n{body}\n}};"


def _resolve_idf_path(arg_value: str | None) -> Path:
    raw = arg_value or os.environ.get("IDF_PATH", "")
    if not raw:
        raise RuntimeError(
            "IDF_PATH not set. Pass --idf-path or set IDF_PATH environment variable."
        )

    p = Path(raw).expanduser().resolve()
    if not p.exists():
        raise RuntimeError(f"IDF_PATH does not exist: {p}")
    return p


def _load_generator(idf_path: Path):
    protocomm_py = idf_path / "components" / "protocomm" / "python"
    local_ctrl_scripts = idf_path / "examples" / "protocols" / "esp_local_ctrl" / "scripts"
    esp_prov_tools = idf_path / "tools" / "esp_prov"

    if not protocomm_py.exists() or (not local_ctrl_scripts.exists() and not esp_prov_tools.exists()):
        raise RuntimeError(
            "Could not find expected ESP-IDF Python paths:\n"
            f"  {protocomm_py}\n"
            f"  {local_ctrl_scripts}\n"
            f"  {esp_prov_tools}\n"
            "Make sure --idf-path points to a full ESP-IDF checkout."
        )

    sys.path.insert(0, str(protocomm_py))
    if local_ctrl_scripts.exists():
        sys.path.insert(1, str(local_ctrl_scripts))
    if esp_prov_tools.exists():
        sys.path.insert(1, str(esp_prov_tools))

    try:
        # Imported from ESP-IDF scripts to guarantee SRP6a compatibility with device side.
        from security.srp6a import generate_salt_and_verifier  # type: ignore
        return generate_salt_and_verifier
    except Exception:
        srp_candidates = [
            local_ctrl_scripts / "security" / "srp6a.py",
            esp_prov_tools / "security" / "srp6a.py",
        ]

        for srp_file in srp_candidates:
            if not srp_file.exists():
                continue

            parent_dir = srp_file.parent.parent
            if str(parent_dir) not in sys.path:
                sys.path.insert(1, str(parent_dir))

            spec = importlib.util.spec_from_file_location("sec2_srp6a_cli", srp_file)
            if spec is None or spec.loader is None:
                continue

            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            fn = getattr(module, "generate_salt_and_verifier", None)
            if callable(fn):
                return fn

        raise RuntimeError("Failed to import Security2 generator from ESP-IDF (security.srp6a).")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate Security2 salt/verifier from username + password (ESP-IDF compatible)."
    )
    parser.add_argument("username", help="Security2 username (e.g. wifiprov)")
    parser.add_argument("password", help="Security2 password / PoP")
    parser.add_argument(
        "--salt-len",
        type=int,
        default=16,
        help="Salt length in bytes (default: 16)",
    )
    parser.add_argument(
        "--idf-path",
        default=None,
        help="Path to ESP-IDF (if IDF_PATH is not set)",
    )
    parser.add_argument(
        "--json-out",
        default=None,
        help="Optional output JSON path",
    )
    parser.add_argument(
        "--print-c",
        action="store_true",
        help="Print C arrays for provisioning.c",
    )
    parser.add_argument(
        "--print-uart",
        action="store_true",
        help="Print ready-to-send UART commands for communication.c",
    )
    args = parser.parse_args()

    if args.salt_len <= 0:
        raise RuntimeError("--salt-len must be > 0")

    idf_path = _resolve_idf_path(args.idf_path)
    gen = _load_generator(idf_path)

    salt, verifier = gen(args.username, args.password, len_s=args.salt_len)

    salt_hex = salt.hex()
    verifier_hex = verifier.hex()

    print(f"username: {args.username}")
    print(f"salt_len: {len(salt)}")
    print(f"verifier_len: {len(verifier)}")
    print(f"salt_hex: {salt_hex}")
    print(f"verifier_hex: {verifier_hex}")

    if args.print_c:
        print()
        print(_format_c_array("sec2_salt", salt))
        print()
        print(_format_c_array("sec2_verifier", verifier))

    if args.print_uart:
        print()
        print("UART commands:")
        print(f"$btSalt>{salt_hex}")
        print(f"$btVerifi>{verifier_hex}")

    if args.json_out:
        out = {
            "username": args.username,
            "salt_len": len(salt),
            "verifier_len": len(verifier),
            "salt_hex": salt_hex,
            "verifier_hex": verifier_hex,
        }
        out_path = Path(args.json_out).expanduser().resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(out, indent=2), encoding="utf-8")
        print(f"json_written: {out_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
