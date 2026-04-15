#!/usr/bin/env python3
"""
Desktop UI for ESP32 Security 2 provisioning helpers.

Features:
- Generate SRP6a salt/verifier from username + password using ESP-IDF Python code.
- Connect to a UART port and send btSalt / btVerifi commands.
- Read back values from device and compare with generated values.
"""

from __future__ import annotations

import os
import queue
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk, messagebox

import serial
from serial.tools import list_ports


class UartClient:
    def __init__(self) -> None:
        self._ser: serial.Serial | None = None
        self._lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def connect(self, port: str, baudrate: int = 115200, timeout: float = 2.0) -> None:
        self.disconnect()
        self._ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)

    def disconnect(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def send_line(self, line: str) -> str:
        if not self.is_connected or self._ser is None:
            raise RuntimeError("UART not connected")

        with self._lock:
            self._ser.reset_input_buffer()
            self._ser.write((line + "\n").encode("utf-8"))
            self._ser.flush()
            response = self._ser.readline().decode("utf-8", errors="replace").strip()
            if not response:
                raise TimeoutError("No UART response")
            return response


def resolve_idf_paths(idf_path_raw: str):
    idf_path = Path(idf_path_raw).expanduser().resolve()
    protocomm_py = idf_path / "components" / "protocomm" / "python"
    local_ctrl_scripts = idf_path / "examples" / "protocols" / "esp_local_ctrl" / "scripts"

    if not idf_path.exists():
        raise RuntimeError(f"IDF path does not exist: {idf_path}")
    if not protocomm_py.exists():
        raise RuntimeError(f"Missing path: {protocomm_py}")
    if not local_ctrl_scripts.exists():
        raise RuntimeError(f"Missing path: {local_ctrl_scripts}")

    return idf_path, protocomm_py, local_ctrl_scripts


def generate_sec2(username: str, password: str, salt_len: int, idf_path_raw: str):
    _, protocomm_py, local_ctrl_scripts = resolve_idf_paths(idf_path_raw)

    if str(protocomm_py) not in sys.path:
        sys.path.insert(0, str(protocomm_py))
    if str(local_ctrl_scripts) not in sys.path:
        sys.path.insert(1, str(local_ctrl_scripts))

    from security.srp6a import generate_salt_and_verifier  # type: ignore

    salt, verifier = generate_salt_and_verifier(username, password, len_s=salt_len)
    return bytes(salt), bytes(verifier)


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ESP32 Sec2 UART Tool")
        self.geometry("980x760")

        self.uart = UartClient()
        self.log_queue: queue.Queue[str] = queue.Queue()

        self.var_port = tk.StringVar()
        self.var_baud = tk.StringVar(value="115200")
        self.var_idf = tk.StringVar(value=os.environ.get("IDF_PATH", ""))
        self.var_user = tk.StringVar(value="wifiprov")
        self.var_pass = tk.StringVar(value="abcd1234")
        self.var_salt_len = tk.StringVar(value="16")

        self.salt_hex = ""
        self.verifier_hex = ""

        self._build_ui()
        self._refresh_ports()
        self.after(100, self._pump_log)

    def _build_ui(self):
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)

        conn = ttk.LabelFrame(root, text="UART")
        conn.pack(fill=tk.X, pady=5)

        ttk.Label(conn, text="Port").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.cmb_port = ttk.Combobox(conn, textvariable=self.var_port, width=18, state="readonly")
        self.cmb_port.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        ttk.Button(conn, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, padx=5, pady=5)

        ttk.Label(conn, text="Baud").grid(row=0, column=3, padx=5, pady=5, sticky="w")
        ttk.Entry(conn, textvariable=self.var_baud, width=10).grid(row=0, column=4, padx=5, pady=5, sticky="w")

        ttk.Button(conn, text="Connect", command=self._connect_uart).grid(row=0, column=5, padx=5, pady=5)
        ttk.Button(conn, text="Disconnect", command=self._disconnect_uart).grid(row=0, column=6, padx=5, pady=5)

        sec = ttk.LabelFrame(root, text="Security 2 Generator")
        sec.pack(fill=tk.X, pady=5)

        ttk.Label(sec, text="IDF_PATH").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        ttk.Entry(sec, textvariable=self.var_idf, width=88).grid(row=0, column=1, columnspan=5, padx=5, pady=5, sticky="we")

        ttk.Label(sec, text="Username").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        ttk.Entry(sec, textvariable=self.var_user, width=24).grid(row=1, column=1, padx=5, pady=5, sticky="w")
        ttk.Label(sec, text="Password").grid(row=1, column=2, padx=5, pady=5, sticky="w")
        ttk.Entry(sec, textvariable=self.var_pass, width=24, show="*").grid(row=1, column=3, padx=5, pady=5, sticky="w")
        ttk.Label(sec, text="Salt bytes").grid(row=1, column=4, padx=5, pady=5, sticky="w")
        ttk.Entry(sec, textvariable=self.var_salt_len, width=8).grid(row=1, column=5, padx=5, pady=5, sticky="w")

        ttk.Button(sec, text="Generate Salt/Verifier", command=self._generate).grid(row=2, column=0, columnspan=2, padx=5, pady=8, sticky="w")
        ttk.Button(sec, text="Copy Salt Hex", command=lambda: self._copy_to_clipboard(self.salt_hex)).grid(row=2, column=2, padx=5, pady=8)
        ttk.Button(sec, text="Copy Verifier Hex", command=lambda: self._copy_to_clipboard(self.verifier_hex)).grid(row=2, column=3, padx=5, pady=8)

        out = ttk.LabelFrame(root, text="Generated Hex")
        out.pack(fill=tk.BOTH, expand=True, pady=5)

        ttk.Label(out, text="Salt").pack(anchor="w", padx=5)
        self.txt_salt = tk.Text(out, height=3, wrap="word")
        self.txt_salt.pack(fill=tk.X, padx=5, pady=4)

        ttk.Label(out, text="Verifier").pack(anchor="w", padx=5)
        self.txt_ver = tk.Text(out, height=10, wrap="word")
        self.txt_ver.pack(fill=tk.BOTH, expand=True, padx=5, pady=4)

        actions = ttk.LabelFrame(root, text="UART Actions")
        actions.pack(fill=tk.X, pady=5)

        ttk.Button(actions, text="Send btSalt", command=self._send_btsalt).grid(row=0, column=0, padx=5, pady=6)
        ttk.Button(actions, text="Send btVerifi", command=self._send_btverifier).grid(row=0, column=1, padx=5, pady=6)
        ttk.Button(actions, text="Send Both", command=self._send_both).grid(row=0, column=2, padx=5, pady=6)
        ttk.Button(actions, text="Read btSalt", command=self._read_btsalt).grid(row=0, column=3, padx=5, pady=6)
        ttk.Button(actions, text="Read btVerifi", command=self._read_btverifier).grid(row=0, column=4, padx=5, pady=6)
        ttk.Button(actions, text="Verify Device Values", command=self._verify_device_values).grid(row=0, column=5, padx=5, pady=6)

        log_frame = ttk.LabelFrame(root, text="Log")
        log_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        self.txt_log = tk.Text(log_frame, height=10, wrap="word", state="disabled")
        self.txt_log.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

    def _log(self, msg: str):
        stamp = time.strftime("%H:%M:%S")
        self.log_queue.put(f"[{stamp}] {msg}")

    def _pump_log(self):
        updated = False
        while not self.log_queue.empty():
            line = self.log_queue.get_nowait()
            self.txt_log.configure(state="normal")
            self.txt_log.insert(tk.END, line + "\n")
            self.txt_log.see(tk.END)
            self.txt_log.configure(state="disabled")
            updated = True
        if updated:
            self.update_idletasks()
        self.after(100, self._pump_log)

    def _refresh_ports(self):
        ports = sorted(p.device for p in list_ports.comports())
        self.cmb_port["values"] = ports
        if ports and not self.var_port.get():
            self.var_port.set(ports[0])
        self._log(f"Ports: {', '.join(ports) if ports else 'none'}")

    def _connect_uart(self):
        port = self.var_port.get().strip()
        if not port:
            messagebox.showerror("UART", "Please select a port")
            return
        try:
            baud = int(self.var_baud.get().strip())
            self.uart.connect(port=port, baudrate=baud)
            self._log(f"Connected UART {port} @ {baud}")
        except Exception as exc:
            messagebox.showerror("UART connect failed", str(exc))

    def _disconnect_uart(self):
        self.uart.disconnect()
        self._log("UART disconnected")

    def _generate(self):
        username = self.var_user.get().strip()
        password = self.var_pass.get()
        idf_path = self.var_idf.get().strip()

        if not username or not password:
            messagebox.showerror("Generate", "Username and password are required")
            return

        try:
            salt_len = int(self.var_salt_len.get().strip())
            if salt_len <= 0:
                raise ValueError("salt bytes must be > 0")
        except Exception as exc:
            messagebox.showerror("Generate", f"Invalid salt length: {exc}")
            return

        try:
            salt, verifier = generate_sec2(username, password, salt_len, idf_path)
            self.salt_hex = salt.hex()
            self.verifier_hex = verifier.hex()

            self.txt_salt.delete("1.0", tk.END)
            self.txt_salt.insert(tk.END, self.salt_hex)
            self.txt_ver.delete("1.0", tk.END)
            self.txt_ver.insert(tk.END, self.verifier_hex)

            self._log(f"Generated sec2: salt={len(salt)} bytes, verifier={len(verifier)} bytes")
        except Exception as exc:
            messagebox.showerror("Generate failed", str(exc))

    def _copy_to_clipboard(self, value: str):
        if not value:
            return
        self.clipboard_clear()
        self.clipboard_append(value)
        self._log("Copied to clipboard")

    def _uart_cmd(self, cmd: str) -> str:
        self._log(f"TX: {cmd}")
        resp = self.uart.send_line(cmd)
        self._log(f"RX: {resp}")
        return resp

    def _send_btsalt(self):
        if len(self.salt_hex) != 32:
            messagebox.showerror("btSalt", "Salt hex must be 32 chars (16 bytes)")
            return
        self._uart_cmd(f"$btSalt>{self.salt_hex}")

    def _send_btverifier(self):
        if len(self.verifier_hex) != 768:
            messagebox.showerror("btVerifi", "Verifier hex must be 768 chars (384 bytes)")
            return
        self._uart_cmd(f"$btVerifi>{self.verifier_hex}")

    def _send_both(self):
        self._send_btsalt()
        self._send_btverifier()

    def _read_btsalt(self) -> str:
        return self._uart_cmd("$btSalt<")

    def _read_btverifier(self) -> str:
        return self._uart_cmd("$btVerifi<")

    def _verify_device_values(self):
        if not self.salt_hex or not self.verifier_hex:
            messagebox.showerror("Verify", "Generate values first")
            return

        r_salt = self._read_btsalt()
        r_ver = self._read_btverifier()

        ok_salt = r_salt.lower() == f"btsaltok:{self.salt_hex}".lower()
        ok_ver = r_ver.lower() == f"btverifiok:{self.verifier_hex}".lower()

        if ok_salt and ok_ver:
            messagebox.showinfo("Verify", "Device salt and verifier match generated values")
        else:
            messagebox.showwarning(
                "Verify",
                f"Mismatch detected\nSalt match: {ok_salt}\nVerifier match: {ok_ver}",
            )


if __name__ == "__main__":
    app = App()
    app.mainloop()
