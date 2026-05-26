#!/usr/bin/env python3
"""
Desktop helper for generating device credentials and HMAC hashes.

Features:
- Generate random device IDs in the form LG_xxxxxxxx or LH_xxxxxxxx.
- Generate random 32-byte printable ASCII secrets for secret_hash and pop_hash.
- Accept existing secrets in ASCII or hex form.
- Compute HMAC-SHA256 hashes using a pepper provided in hex.
- Show raw values in both ASCII and hex.
"""

from __future__ import annotations

import hmac
import hashlib
import secrets
import string
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, ttk

DEVICE_ID_SUFFIX_LEN = 8
RANDOM_SECRET_LEN = 32
SAFE_ASCII_ALPHABET = string.ascii_letters + string.digits


def normalize_hex(value: str) -> str:
    return "".join(value.split())


def parse_hex_bytes(value: str) -> bytes:
    normalized = normalize_hex(value)
    if len(normalized) == 0:
        return b""
    return bytes.fromhex(normalized)


def bytes_to_ascii_display(value: bytes) -> str:
    if not value:
        return ""
    try:
        text = value.decode("ascii")
    except UnicodeDecodeError:
        return "<non-ascii>"

    if any(ord(ch) < 32 or ord(ch) > 126 for ch in text):
        return "<non-printable-ascii>"
    return text


def ascii_to_bytes(value: str) -> bytes:
    return value.encode("ascii")


def random_ascii(length: int) -> str:
    return "".join(secrets.choice(SAFE_ASCII_ALPHABET) for _ in range(length))


@dataclass
class CredentialResult:
    raw_bytes: bytes
    ascii_value: str
    hex_value: str
    hash_hex: str


class CredentialSection(ttk.LabelFrame):
    def __init__(self, master: tk.Misc, title: str):
        super().__init__(master, text=title, padding=10)

        self.var_format = tk.StringVar(value="ascii")
        self.var_input = tk.StringVar()
        self.var_ascii = tk.StringVar()
        self.var_hex = tk.StringVar()
        self.var_hash = tk.StringVar()

        ttk.Label(self, text="Input format").grid(row=0, column=0, sticky="w", pady=(0, 6))
        format_box = ttk.Combobox(
            self,
            textvariable=self.var_format,
            values=("ascii", "hex"),
            state="readonly",
            width=10,
        )
        format_box.grid(row=0, column=1, sticky="w", pady=(0, 6))

        ttk.Label(self, text="Input value").grid(row=1, column=0, sticky="nw")
        entry = ttk.Entry(self, textvariable=self.var_input, width=60)
        entry.grid(row=1, column=1, columnspan=3, sticky="ew", pady=(0, 6))

        ttk.Button(self, text="Generate random 32B", command=self.generate_random).grid(
            row=2, column=1, sticky="w", pady=(0, 10)
        )
        ttk.Button(self, text="Use hex as input", command=self.use_hex_input).grid(
            row=2, column=2, sticky="w", pady=(0, 10)
        )
        ttk.Button(self, text="Use ASCII as input", command=self.use_ascii_input).grid(
            row=2, column=3, sticky="w", pady=(0, 10)
        )

        ttk.Label(self, text="ASCII").grid(row=3, column=0, sticky="w")
        ttk.Entry(self, textvariable=self.var_ascii, width=60, state="readonly").grid(
            row=3, column=1, columnspan=3, sticky="ew", pady=(0, 6)
        )

        ttk.Label(self, text="Hex").grid(row=4, column=0, sticky="w")
        ttk.Entry(self, textvariable=self.var_hex, width=60, state="readonly").grid(
            row=4, column=1, columnspan=3, sticky="ew", pady=(0, 6)
        )

        ttk.Label(self, text="HMAC-SHA256").grid(row=5, column=0, sticky="w")
        ttk.Entry(self, textvariable=self.var_hash, width=60, state="readonly").grid(
            row=5, column=1, columnspan=3, sticky="ew"
        )

        self.columnconfigure(1, weight=1)
        self.var_format.trace_add("write", self._on_change)
        self.var_input.trace_add("write", self._on_change)

    def _on_change(self, *_args: object) -> None:
        self.var_ascii.set("")
        self.var_hex.set("")
        self.var_hash.set("")

    def set_hash(self, hash_hex: str) -> None:
        self.var_hash.set(hash_hex)

    def get_raw_bytes(self) -> bytes:
        value = self.var_input.get()
        if not value:
            return b""

        if self.var_format.get() == "ascii":
            return ascii_to_bytes(value)
        return parse_hex_bytes(value)

    def resolve(self, pepper_hex: str) -> CredentialResult:
        raw_bytes = self.get_raw_bytes()
        hash_hex = hmac.new(
            bytes.fromhex(normalize_hex(pepper_hex)),
            raw_bytes,
            hashlib.sha256,
        ).hexdigest()
        return CredentialResult(
            raw_bytes=raw_bytes,
            ascii_value=bytes_to_ascii_display(raw_bytes),
            hex_value=raw_bytes.hex(),
            hash_hex=hash_hex,
        )

    def refresh_views(self, pepper_hex: str) -> None:
        result = self.resolve(pepper_hex)
        self.var_ascii.set(result.ascii_value)
        self.var_hex.set(result.hex_value)
        self.var_hash.set(result.hash_hex)

    def generate_random(self) -> None:
        random_value = random_ascii(RANDOM_SECRET_LEN)
        self.var_format.set("ascii")
        self.var_input.set(random_value)

    def use_hex_input(self) -> None:
        if self.var_hex.get():
            self.var_format.set("hex")
            self.var_input.set(self.var_hex.get())

    def use_ascii_input(self) -> None:
        ascii_value = self.var_ascii.get()
        if ascii_value and not ascii_value.startswith("<"):
            self.var_format.set("ascii")
            self.var_input.set(ascii_value)


class App(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("HMAC Device Tool")
        self.geometry("980x720")

        self.var_device_type = tk.StringVar(value="LG")
        self.var_device_id = tk.StringVar()
        self.var_pepper = tk.StringVar()
        self.var_status = tk.StringVar(value="Ready")

        outer = ttk.Frame(self, padding=12)
        outer.pack(fill="both", expand=True)

        device_frame = ttk.LabelFrame(outer, text="Device", padding=10)
        device_frame.pack(fill="x", pady=(0, 10))

        ttk.Label(device_frame, text="Type").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            device_frame,
            textvariable=self.var_device_type,
            values=("LG", "LH"),
            state="readonly",
            width=8,
        ).grid(row=0, column=1, sticky="w")

        ttk.Label(device_frame, text="Device ID").grid(row=0, column=2, sticky="w", padx=(12, 0))
        ttk.Entry(device_frame, textvariable=self.var_device_id, width=30).grid(
            row=0, column=3, sticky="ew"
        )
        ttk.Button(device_frame, text="Generate ID", command=self.generate_device_id).grid(
            row=0, column=4, padx=(8, 0)
        )

        ttk.Label(device_frame, text="Pepper (hex)").grid(row=1, column=0, sticky="w", pady=(10, 0))
        ttk.Entry(device_frame, textvariable=self.var_pepper, width=80).grid(
            row=1, column=1, columnspan=4, sticky="ew", pady=(10, 0)
        )

        device_frame.columnconfigure(3, weight=1)

        self.secret_section = CredentialSection(outer, "secret_hash input")
        self.secret_section.pack(fill="x", pady=(0, 10))

        self.pop_section = CredentialSection(outer, "pop_hash input")
        self.pop_section.pack(fill="x", pady=(0, 10))

        button_row = ttk.Frame(outer)
        button_row.pack(fill="x", pady=(0, 10))
        ttk.Button(button_row, text="Generate all", command=self.generate_all).pack(side="left")
        ttk.Button(button_row, text="Calculate hashes", command=self.calculate_all).pack(side="left", padx=(8, 0))
        ttk.Button(button_row, text="Clear", command=self.clear_all).pack(side="left", padx=(8, 0))

        output_frame = ttk.LabelFrame(outer, text="Summary", padding=10)
        output_frame.pack(fill="both", expand=True)

        self.txt_output = tk.Text(output_frame, wrap="word", height=16)
        self.txt_output.pack(fill="both", expand=True)

        status_label = ttk.Label(outer, textvariable=self.var_status)
        status_label.pack(anchor="w")

        self.generate_device_id()

    def generate_device_id(self) -> None:
        suffix = random_ascii(DEVICE_ID_SUFFIX_LEN)
        self.var_device_id.set(f"{self.var_device_type.get()}_{suffix}")
        self.var_status.set("Generated device ID")

    def validate_pepper(self) -> bytes:
        pepper = normalize_hex(self.var_pepper.get())
        if not pepper:
            raise ValueError("Pepper must not be empty")
        if len(pepper) % 2 != 0:
            raise ValueError("Pepper hex length must be even")
        return bytes.fromhex(pepper)

    def generate_all(self) -> None:
        self.generate_device_id()
        self.secret_section.generate_random()
        self.pop_section.generate_random()
        self.var_status.set("Generated device ID, secret, and POP")
        self.calculate_all()

    def clear_all(self) -> None:
        self.var_device_id.set("")
        self.var_pepper.set("")
        for section in (self.secret_section, self.pop_section):
            section.var_format.set("ascii")
            section.var_input.set("")
            section.var_ascii.set("")
            section.var_hex.set("")
            section.var_hash.set("")
        self.txt_output.delete("1.0", tk.END)
        self.var_status.set("Cleared")

    def calculate_all(self) -> None:
        try:
            self.validate_pepper()
            pepper_hex = self.var_pepper.get()

            secret_result = self.secret_section.resolve(pepper_hex)
            pop_result = self.pop_section.resolve(pepper_hex)

            self.secret_section.var_ascii.set(secret_result.ascii_value)
            self.secret_section.var_hex.set(secret_result.hex_value)
            self.secret_section.set_hash(secret_result.hash_hex)

            self.pop_section.var_ascii.set(pop_result.ascii_value)
            self.pop_section.var_hex.set(pop_result.hex_value)
            self.pop_section.set_hash(pop_result.hash_hex)

            summary = [
                f"device_id: {self.var_device_id.get()}",
                "",
                "secret input:",
                f"  ascii: {secret_result.ascii_value}",
                f"  hex:   {secret_result.hex_value}",
                f"  hash:  {secret_result.hash_hex}",
                "",
                "pop input:",
                f"  ascii: {pop_result.ascii_value}",
                f"  hex:   {pop_result.hex_value}",
                f"  hash:  {pop_result.hash_hex}",
            ]

            self.txt_output.delete("1.0", tk.END)
            self.txt_output.insert("1.0", "\n".join(summary))
            self.var_status.set("Hashes calculated")
        except Exception as exc:
            self.var_status.set(f"Error: {exc}")
            messagebox.showerror("Calculation error", str(exc))


def main() -> None:
    app = App()
    app.mainloop()


if __name__ == "__main__":
    main()
