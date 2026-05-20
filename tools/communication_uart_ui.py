#!/usr/bin/env python3
"""
Full UART desktop tool for communication.c commands.

Covers:
- $ protocol commands (set/get):
    devID, devPW, wifiSsid, wifiPw, fbEmail, fbPw, btSalt, btVerifi, selPos
- Security2 salt/verifier generation from username/password (ESP-IDF SRP6a)
- Legacy one-byte and multi-byte commands handled in testFunction()
- Raw ASCII and raw hex send for full flexibility
"""

from __future__ import annotations

import os
import json
import importlib.util
import queue
import re
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

import serial
from serial.tools import list_ports


def parse_byte(value: str) -> int:
    s = value.strip().lower()
    if s.startswith("0x"):
        v = int(s, 16)
    else:
        v = int(s, 10)
    if v < 0 or v > 255:
        raise ValueError("Byte out of range (0..255)")
    return v


def _load_workspace_settings() -> dict:
    """Load VS Code workspace settings.json if available."""
    script_dir = Path(__file__).resolve().parent
    workspace_root = script_dir.parent
    settings_path = workspace_root / ".vscode" / "settings.json"

    if not settings_path.exists():
        return {}

    try:
        return json.loads(settings_path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def detect_idf_path() -> str:
    """Prefer workspace ESP-IDF settings, fallback to environment variable."""
    settings = _load_workspace_settings()

    candidates = [
        settings.get("idf.espIdfPathWin"),
        settings.get("idf.currentSetup"),
        settings.get("idf.espIdfPath"),
        os.environ.get("IDF_PATH"),
    ]

    for candidate in candidates:
        if not candidate:
            continue
        p = Path(str(candidate)).expanduser()
        if p.exists():
            return str(p)
    return ""


class UartTransport:
    def __init__(self):
        self.ser: serial.Serial | None = None
        self._lock = threading.Lock()
        self._rx_thread: threading.Thread | None = None
        self._running = False

        self.lines: queue.Queue[str] = queue.Queue()
        self.log: queue.Queue[str] = queue.Queue()

    @property
    def connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def connect(self, port: str, baud: int):
        self.disconnect()
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
        self._running = True
        self._rx_thread = threading.Thread(target=self._reader, daemon=True)
        self._rx_thread.start()
        self._log(f"Connected: {port} @ {baud}")

    def disconnect(self):
        self._running = False
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self._log("Disconnected")

    def _log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.log.put(f"[{ts}] {msg}")

    def _reader(self):
        buf = bytearray()
        while self._running and self.ser is not None:
            try:
                data = self.ser.read(256)
                if not data:
                    continue
                buf.extend(data)

                while b"\n" in buf:
                    idx = buf.index(0x0A)
                    raw = bytes(buf[:idx]).rstrip(b"\r")
                    del buf[: idx + 1]
                    line = raw.decode("utf-8", errors="replace")
                    self.lines.put(line)
                    self._log(f"RX: {line}")
            except Exception as exc:
                self._log(f"RX error: {exc}")
                break

    def clear_rx(self):
        while True:
            try:
                self.lines.get_nowait()
            except queue.Empty:
                break

    def send_bytes(self, payload: bytes):
        if not self.connected or self.ser is None:
            raise RuntimeError("UART not connected")
        with self._lock:
            self.ser.write(payload)
            self.ser.flush()
        self._log(f"TX bytes: {payload.hex()}")

    def send_ascii(self, text: str):
        self.send_bytes(text.encode("utf-8"))
        self._log(f"TX: {text}")

    def wait_match(self, pattern: re.Pattern[str], timeout_s: float = 2.0) -> str:
        end = time.time() + timeout_s
        while time.time() < end:
            try:
                line = self.lines.get(timeout=0.05)
            except queue.Empty:
                continue
            m = pattern.search(line)
            if m:
                return line[m.start() :]
        raise TimeoutError("No matching response")


def resolve_idf_paths(idf_path_raw: str):
    idf_path = Path(idf_path_raw).expanduser().resolve()
    protocomm_py = idf_path / "components" / "protocomm" / "python"
    local_ctrl_scripts = idf_path / "examples" / "protocols" / "esp_local_ctrl" / "scripts"
    esp_prov_tools = idf_path / "tools" / "esp_prov"

    if not idf_path.exists():
        raise RuntimeError(f"IDF path does not exist: {idf_path}")
    if not protocomm_py.exists():
        raise RuntimeError(f"Missing path: {protocomm_py}")

    # At least one provider for package "security" must exist.
    if not local_ctrl_scripts.exists() and not esp_prov_tools.exists():
        raise RuntimeError(
            "Could not find a Security2 Python provider in ESP-IDF. Checked:\n"
            f"  {local_ctrl_scripts}\n"
            f"  {esp_prov_tools}"
        )

    return protocomm_py, local_ctrl_scripts, esp_prov_tools


def generate_sec2(username: str, password: str, salt_len: int, idf_path_raw: str):
    protocomm_py, local_ctrl_scripts, esp_prov_tools = resolve_idf_paths(idf_path_raw)

    if str(protocomm_py) not in sys.path:
        sys.path.insert(0, str(protocomm_py))
    if local_ctrl_scripts.exists() and str(local_ctrl_scripts) not in sys.path:
        sys.path.insert(1, str(local_ctrl_scripts))
    if esp_prov_tools.exists() and str(esp_prov_tools) not in sys.path:
        sys.path.insert(1, str(esp_prov_tools))

    try:
        from security.srp6a import generate_salt_and_verifier  # type: ignore
    except Exception:
        # Some IDF versions import optional security modules in security/__init__.py.
        # Fallback: load srp6a.py directly.
        srp_candidates = [
            local_ctrl_scripts / "security" / "srp6a.py",
            esp_prov_tools / "security" / "srp6a.py",
        ]
        generate_salt_and_verifier = None

        for srp_file in srp_candidates:
            if not srp_file.exists():
                continue
            parent_dir = srp_file.parent.parent
            if str(parent_dir) not in sys.path:
                sys.path.insert(1, str(parent_dir))

            spec = importlib.util.spec_from_file_location("sec2_srp6a", srp_file)
            if spec is None or spec.loader is None:
                continue

            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            generate_salt_and_verifier = getattr(module, "generate_salt_and_verifier", None)
            if callable(generate_salt_and_verifier):
                break

        if not callable(generate_salt_and_verifier):
            raise RuntimeError(
                "Failed to import Security2 generator from ESP-IDF (security.srp6a)."
            )

    salt, verifier = generate_salt_and_verifier(username, password, len_s=salt_len)
    return bytes(salt), bytes(verifier)


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("HP Firmware Communication Tool")
        self.geometry("1180x840")

        self.uart = UartTransport()

        self.var_port = tk.StringVar()
        self.var_baud = tk.StringVar(value="115200")

        self.var_idf = tk.StringVar(value=detect_idf_path())
        self.var_user = tk.StringVar(value="wifiprov")
        self.var_pass = tk.StringVar(value="abcd1234")
        self.var_salt_len = tk.StringVar(value="16")

        self.var_sel_ch = tk.StringVar(value="0")
        self.var_sel_angle = tk.StringVar(value="40")

        self.fields: dict[str, tk.StringVar] = {
            "devID": tk.StringVar(),
            "devPW": tk.StringVar(),
            "wifiSsid": tk.StringVar(),
            "wifiPw": tk.StringVar(),
            "fbEmail": tk.StringVar(),
            "fbPw": tk.StringVar(),
            "btSalt": tk.StringVar(),
            "btVerifi": tk.StringVar(),
        }

        self.var_raw_ascii = tk.StringVar()
        self.var_raw_hex = tk.StringVar()

        self.var_b1 = tk.StringVar(value="0")
        self.var_b2 = tk.StringVar(value="0")

        self._build_ui()
        self._refresh_ports()
        self.after(100, self._pump_log)

    def _build_ui(self):
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)

        top = ttk.LabelFrame(root, text="Connection")
        top.pack(fill=tk.X, pady=5)

        ttk.Label(top, text="Port").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.cmb_port = ttk.Combobox(top, textvariable=self.var_port, state="readonly", width=18)
        self.cmb_port.grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(top, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, padx=5, pady=5)

        ttk.Label(top, text="Baud").grid(row=0, column=3, padx=5, pady=5, sticky="w")
        ttk.Entry(top, textvariable=self.var_baud, width=10).grid(row=0, column=4, padx=5, pady=5)

        ttk.Button(top, text="Connect", command=self._connect).grid(row=0, column=5, padx=5, pady=5)
        ttk.Button(top, text="Disconnect", command=self._disconnect).grid(row=0, column=6, padx=5, pady=5)

        notebook = ttk.Notebook(root)
        notebook.pack(fill=tk.BOTH, expand=True, pady=6)

        tab_dollar = ttk.Frame(notebook)
        tab_legacy = ttk.Frame(notebook)
        tab_raw = ttk.Frame(notebook)

        notebook.add(tab_dollar, text="$ Protocol")
        notebook.add(tab_legacy, text="Legacy Commands")
        notebook.add(tab_raw, text="Raw")

        self._build_dollar_tab(tab_dollar)
        self._build_legacy_tab(tab_legacy)
        self._build_raw_tab(tab_raw)

        logf = ttk.LabelFrame(root, text="Log")
        logf.pack(fill=tk.BOTH, expand=True, pady=4)

        self.txt_log = tk.Text(logf, height=12, wrap="word", state="disabled")
        self.txt_log.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

    def _build_dollar_tab(self, parent: ttk.Frame):
        sec = ttk.LabelFrame(parent, text="Sec2 Generator")
        sec.pack(fill=tk.X, padx=8, pady=8)

        ttk.Label(sec, text="IDF_PATH").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        ttk.Entry(sec, textvariable=self.var_idf, width=90).grid(row=0, column=1, columnspan=5, padx=5, pady=5, sticky="we")
        ttk.Button(sec, text="Auto", command=self._autofill_idf_path).grid(row=0, column=6, padx=5, pady=5)

        ttk.Label(sec, text="Username").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        ttk.Entry(sec, textvariable=self.var_user, width=20).grid(row=1, column=1, padx=5, pady=5, sticky="w")
        ttk.Label(sec, text="Password").grid(row=1, column=2, padx=5, pady=5, sticky="w")
        ttk.Entry(sec, textvariable=self.var_pass, width=20).grid(row=1, column=3, padx=5, pady=5, sticky="w")
        ttk.Label(sec, text="Salt bytes").grid(row=1, column=4, padx=5, pady=5, sticky="w")
        ttk.Entry(sec, textvariable=self.var_salt_len, width=8).grid(row=1, column=5, padx=5, pady=5, sticky="w")

        ttk.Button(sec, text="Generate btSalt/btVerifi", command=self._generate_sec2).grid(row=2, column=0, columnspan=2, padx=5, pady=6, sticky="w")
        ttk.Button(sec, text="Send Both to Device", command=self._send_both_sec2).grid(row=2, column=2, padx=5, pady=6, sticky="w")
        ttk.Button(sec, text="Verify Device == Generated", command=self._verify_sec2).grid(row=2, column=3, padx=5, pady=6, sticky="w")

        fields = ttk.LabelFrame(parent, text="Set/Get Fields")
        fields.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        ids = ["devID", "devPW", "wifiSsid", "wifiPw", "fbEmail", "fbPw", "btSalt", "btVerifi"]
        for idx, ident in enumerate(ids):
            ttk.Label(fields, text=ident).grid(row=idx, column=0, padx=5, pady=4, sticky="w")
            width = 92 if ident != "btVerifi" else 100
            ttk.Entry(fields, textvariable=self.fields[ident], width=width).grid(row=idx, column=1, padx=5, pady=4, sticky="we")
            ttk.Button(fields, text="Set", command=lambda i=ident: self._set_id(i)).grid(row=idx, column=2, padx=5, pady=4)
            ttk.Button(fields, text="Get", command=lambda i=ident: self._get_id(i)).grid(row=idx, column=3, padx=5, pady=4)

        sel = ttk.LabelFrame(parent, text="Selector")
        sel.pack(fill=tk.X, padx=8, pady=8)

        ttk.Label(sel, text="Channel (0..3)").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        ttk.Entry(sel, textvariable=self.var_sel_ch, width=10).grid(row=0, column=1, padx=5, pady=5)
        ttk.Label(sel, text="Angle").grid(row=0, column=2, padx=5, pady=5, sticky="w")
        ttk.Entry(sel, textvariable=self.var_sel_angle, width=10).grid(row=0, column=3, padx=5, pady=5)
        ttk.Button(sel, text="Set selPos", command=self._set_selpos).grid(row=0, column=4, padx=5, pady=5)
        ttk.Button(sel, text="Get selPos", command=self._get_selpos).grid(row=0, column=5, padx=5, pady=5)

    def _autofill_idf_path(self):
        detected = detect_idf_path()
        self.var_idf.set(detected)
        if detected:
            self.uart._log(f"Auto IDF_PATH: {detected}")
        else:
            self.uart._log("Auto IDF_PATH: not found")

    def _build_legacy_tab(self, parent: ttk.Frame):
        fr1 = ttk.LabelFrame(parent, text="Simple one-byte commands")
        fr1.pack(fill=tk.X, padx=8, pady=8)

        simple = [
            ("p", "Periphery log"), ("u", "Batt volt"), ("v", "ESP volt"), ("c", "Selector cali"),
            ("t", "Temperature"), ("w", "Tank ADC"), ("z", "Deep sleep"), ("h", "Humidity"),
            ("f", "Level ml"), ("g", "Level %"), ("x", "Unused"), ("2", "Storage read test"),
            ("a", "ADC cal 150"), ("b", "ADC cal 850"), ("d", "Analyser dump"), ("m", "MiFlora read"),
        ]

        for i, (cmd, title) in enumerate(simple):
            ttk.Button(fr1, text=f"{cmd} ({title})", command=lambda c=cmd: self._send_legacy(bytes([ord(c)]))).grid(
                row=i // 4, column=i % 4, padx=5, pady=5, sticky="we"
            )

        fr2 = ttk.LabelFrame(parent, text="Parameterized legacy commands")
        fr2.pack(fill=tk.X, padx=8, pady=8)

        ttk.Label(fr2, text="Param1 (byte dec/hex)").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        ttk.Entry(fr2, textvariable=self.var_b1, width=12).grid(row=0, column=1, padx=5, pady=5)
        ttk.Label(fr2, text="Param2 (byte dec/hex)").grid(row=0, column=2, padx=5, pady=5, sticky="w")
        ttk.Entry(fr2, textvariable=self.var_b2, width=12).grid(row=0, column=3, padx=5, pady=5)

        ttk.Button(fr2, text="o + p1", command=lambda: self._send_with_params("o", 1)).grid(row=1, column=0, padx=5, pady=5)
        ttk.Button(fr2, text="i + p1", command=lambda: self._send_with_params("i", 1)).grid(row=1, column=1, padx=5, pady=5)
        ttk.Button(fr2, text="y + p1", command=lambda: self._send_with_params("y", 1)).grid(row=1, column=2, padx=5, pady=5)
        ttk.Button(fr2, text="1 + p1", command=lambda: self._send_with_params("1", 1)).grid(row=1, column=3, padx=5, pady=5)

        ttk.Button(fr2, text="r + p1 + p2", command=lambda: self._send_with_params("r", 2)).grid(row=2, column=0, padx=5, pady=5)
        ttk.Button(fr2, text="s + p1 + p2", command=lambda: self._send_with_params("s", 2)).grid(row=2, column=1, padx=5, pady=5)

        fr3 = ttk.LabelFrame(parent, text="Log commands")
        fr3.pack(fill=tk.X, padx=8, pady=8)

        ttk.Button(fr3, text="lrP", command=lambda: self._send_legacy(b"lrP")).grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(fr3, text="lcP", command=lambda: self._send_legacy(b"lcP")).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(fr3, text="lrW", command=lambda: self._send_legacy(b"lrW")).grid(row=0, column=2, padx=5, pady=5)
        ttk.Button(fr3, text="lcW", command=lambda: self._send_legacy(b"lcW")).grid(row=0, column=3, padx=5, pady=5)
        ttk.Button(fr3, text="lrE", command=lambda: self._send_legacy(b"lrE")).grid(row=0, column=4, padx=5, pady=5)
        ttk.Button(fr3, text="lcE", command=lambda: self._send_legacy(b"lcE")).grid(row=0, column=5, padx=5, pady=5)

    def _build_raw_tab(self, parent: ttk.Frame):
        fr1 = ttk.LabelFrame(parent, text="Raw ASCII")
        fr1.pack(fill=tk.X, padx=8, pady=8)

        ttk.Entry(fr1, textvariable=self.var_raw_ascii, width=100).grid(row=0, column=0, padx=5, pady=5, sticky="we")
        ttk.Button(fr1, text="Send ASCII", command=self._send_raw_ascii).grid(row=0, column=1, padx=5, pady=5)

        fr2 = ttk.LabelFrame(parent, text="Raw HEX bytes (e.g. 24 62 74 53 61 6c 74 3c)")
        fr2.pack(fill=tk.X, padx=8, pady=8)

        ttk.Entry(fr2, textvariable=self.var_raw_hex, width=100).grid(row=0, column=0, padx=5, pady=5, sticky="we")
        ttk.Button(fr2, text="Send HEX", command=self._send_raw_hex).grid(row=0, column=1, padx=5, pady=5)

    def _append_log(self, line: str):
        self.txt_log.configure(state="normal")
        self.txt_log.insert(tk.END, line + "\n")
        self.txt_log.see(tk.END)
        self.txt_log.configure(state="disabled")

    def _pump_log(self):
        while not self.uart.log.empty():
            self._append_log(self.uart.log.get_nowait())
        self.after(100, self._pump_log)

    def _refresh_ports(self):
        ports = sorted(p.device for p in list_ports.comports())
        self.cmb_port["values"] = ports
        if ports and not self.var_port.get():
            self.var_port.set(ports[0])
        self.uart._log("Ports: " + (", ".join(ports) if ports else "none"))

    def _connect(self):
        try:
            port = self.var_port.get().strip()
            baud = int(self.var_baud.get().strip())
            self.uart.connect(port, baud)
        except Exception as exc:
            messagebox.showerror("Connect failed", str(exc))

    def _disconnect(self):
        self.uart.disconnect()

    def _query_dollar(self, ident: str, direction: str, value: str = "") -> str:
        if direction not in (">", "<"):
            raise ValueError("direction must be > or <")

        cmd = f"${ident}{direction}{value}" if direction == ">" else f"${ident}<"
        self.uart.clear_rx()
        # Send newline for $ protocol so echoed command is separated from the real response.
        self.uart.send_ascii(cmd + "\n")
        pat = re.compile(rf"{re.escape(ident)}(OK|ERR)(:.*)?$")
        return self.uart.wait_match(pat, timeout_s=4.0)

    def _set_id(self, ident: str):
        try:
            value = self.fields[ident].get()
            if ident == "btSalt" and len(value) != 32:
                raise ValueError("btSalt must be 32 hex chars")
            if ident == "btVerifi" and len(value) != 768:
                raise ValueError("btVerifi must be 768 hex chars")
            resp = self._query_dollar(ident, ">", value)
            self.uart._log(f"CMD result: {resp}")
        except Exception as exc:
            messagebox.showerror("Set failed", str(exc))

    def _get_id(self, ident: str):
        try:
            resp = self._query_dollar(ident, "<")
            self.uart._log(f"CMD result: {resp}")
            if ":" in resp:
                self.fields[ident].set(resp.split(":", 1)[1])
        except Exception as exc:
            messagebox.showerror("Get failed", str(exc))

    def _set_selpos(self):
        try:
            ch = int(self.var_sel_ch.get().strip())
            angle = int(self.var_sel_angle.get().strip())
            # Firmware responds with "selPosOK/ERR" (without channel suffix).
            ident = f"selPos{ch}"
            cmd = f"${ident}>{angle}\n"
            self.uart.clear_rx()
            self.uart.send_ascii(cmd)
            resp = self.uart.wait_match(re.compile(r"selPos(OK|ERR)$"), timeout_s=4.0)
            self.uart._log(f"CMD result: {resp}")
        except Exception as exc:
            messagebox.showerror("selPos set failed", str(exc))

    def _get_selpos(self):
        try:
            self.uart.clear_rx()
            self.uart.send_ascii("$selPos<\n")
            # For GET we require the full payload: selPosOK:a,b,c,d
            resp = self.uart.wait_match(
                re.compile(r"selPos(OK:[0-9]+,[0-9]+,[0-9]+,[0-9]+|ERR)$"),
                timeout_s=4.0,
            )
            self.uart._log(f"CMD result: {resp}")
            if ":" in resp:
                vals = resp.split(":", 1)[1]
                messagebox.showinfo("selPos", vals)
            else:
                messagebox.showwarning("selPos", "Device returned ERR")
        except Exception as exc:
            messagebox.showerror("selPos get failed", str(exc))

    def _generate_sec2(self):
        try:
            username = self.var_user.get().strip()
            password = self.var_pass.get()
            salt_len = int(self.var_salt_len.get().strip())
            idf = self.var_idf.get().strip()

            if not username or not password:
                raise ValueError("Username and password required")

            salt, verifier = generate_sec2(username, password, salt_len, idf)
            self.fields["btSalt"].set(salt.hex())
            self.fields["btVerifi"].set(verifier.hex())
            self.uart._log(f"Generated sec2: salt={len(salt)} bytes, verifier={len(verifier)} bytes")
        except Exception as exc:
            messagebox.showerror("Generate failed", str(exc))

    def _send_both_sec2(self):
        self._set_id("btSalt")
        self._set_id("btVerifi")

    def _verify_sec2(self):
        try:
            gen_salt = self.fields["btSalt"].get().strip().lower()
            gen_ver = self.fields["btVerifi"].get().strip().lower()
            if not gen_salt or not gen_ver:
                raise ValueError("Generate values first")

            r1 = self._query_dollar("btSalt", "<")
            r2 = self._query_dollar("btVerifi", "<")

            dev_salt = r1.split(":", 1)[1].strip().lower() if ":" in r1 else ""
            dev_ver = r2.split(":", 1)[1].strip().lower() if ":" in r2 else ""

            ok_salt = (dev_salt == gen_salt)
            ok_ver = (dev_ver == gen_ver)

            if ok_salt and ok_ver:
                messagebox.showinfo("Verify", "Device values match generated values")
            else:
                messagebox.showwarning("Verify", f"Mismatch\nSalt: {ok_salt}\nVerifier: {ok_ver}")
        except Exception as exc:
            messagebox.showerror("Verify failed", str(exc))

    def _send_legacy(self, data: bytes):
        try:
            self.uart.send_bytes(data)
        except Exception as exc:
            messagebox.showerror("Legacy send failed", str(exc))

    def _send_with_params(self, cmd_char: str, param_count: int):
        try:
            b1 = parse_byte(self.var_b1.get())
            payload = bytearray([ord(cmd_char), b1])
            if param_count == 2:
                b2 = parse_byte(self.var_b2.get())
                payload.append(b2)
            self._send_legacy(bytes(payload))
        except Exception as exc:
            messagebox.showerror("Param send failed", str(exc))

    def _send_raw_ascii(self):
        try:
            s = self.var_raw_ascii.get()
            self.uart.send_ascii(s)
        except Exception as exc:
            messagebox.showerror("Raw ASCII failed", str(exc))

    def _send_raw_hex(self):
        try:
            s = self.var_raw_hex.get().strip()
            if not s:
                return
            b = bytes.fromhex(s)
            self.uart.send_bytes(b)
        except Exception as exc:
            messagebox.showerror("Raw HEX failed", str(exc))


if __name__ == "__main__":
    app = App()
    app.mainloop()
