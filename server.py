"""
Modulux ESP32-S3 Compile & Upload Server
รันก่อนใช้งานเว็บ IDE: python server.py

ต้องติดตั้ง:
  1. arduino-cli  (https://arduino.github.io/arduino-cli/)
  2. ESP32 core:  arduino-cli core install esp32:esp32
  3. pyserial:    pip install pyserial
"""

import http.server
import json
import os
import subprocess
import tempfile
import shutil
import sys
import importlib

HAS_SERIAL = False
SERIAL_LIST_PORTS = None
SERIAL_INSTALL_ATTEMPTED = False


def ensure_pyserial():
    """Load pyserial dynamically and auto-install once if missing."""
    global HAS_SERIAL, SERIAL_LIST_PORTS, SERIAL_INSTALL_ATTEMPTED

    if HAS_SERIAL and SERIAL_LIST_PORTS is not None:
        return True

    try:
        SERIAL_LIST_PORTS = importlib.import_module("serial.tools.list_ports")
        HAS_SERIAL = True
        return True
    except Exception:
        HAS_SERIAL = False

    if SERIAL_INSTALL_ATTEMPTED:
        return False

    SERIAL_INSTALL_ATTEMPTED = True
    print("[INFO] pyserial ไม่ได้ติดตั้ง — กำลังติดตั้งอัตโนมัติ...")
    try:
        subprocess.run([sys.executable, "-m", "pip", "install", "pyserial", "-q"], check=True)
        SERIAL_LIST_PORTS = importlib.import_module("serial.tools.list_ports")
        HAS_SERIAL = True
        print("[INFO] ติดตั้ง pyserial สำเร็จ")
        return True
    except Exception:
        HAS_SERIAL = False
        SERIAL_LIST_PORTS = None
        print("[WARNING] ติดตั้ง pyserial ไม่สำเร็จ — จะใช้ PowerShell แทน")
        return False


def list_ports_fallback():
    """Windows: ค้น COM Port ผ่าน PowerShell WMI (ไม่ต้องใช้ pyserial)"""
    try:
        cmd = [
            "powershell", "-NoProfile", "-Command",
            "Get-WMIObject Win32_SerialPort | Select-Object DeviceID,Description | ConvertTo-Json"
        ]
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=8)
        if r.returncode != 0 or not r.stdout.strip():
            return []
        raw = json.loads(r.stdout.strip())
        if isinstance(raw, dict):
            raw = [raw]
        ports = []
        for item in raw:
            dev = item.get("DeviceID", "") or ""
            desc = item.get("Description", "") or ""
            if dev:
                ports.append({"port": dev, "desc": desc})
        return sorted(ports, key=lambda x: x["port"])
    except Exception as e:
        print(f"[WARNING] PowerShell WMI ล้มเหลว: {e}")
        return []


def list_ports_arduino_cli():
    """Fallback: ใช้ arduino-cli board list"""
    try:
        r = subprocess.run(
            ["arduino-cli", "board", "list", "--json"],
            capture_output=True, text=True, timeout=10
        )
        if r.returncode != 0:
            return []
        data = json.loads(r.stdout)
        ports = []
        for item in data.get("detected_ports", []):
            addr = item.get("port", {}).get("address", "")
            label = item.get("port", {}).get("label", "")
            if addr:
                ports.append({"port": addr, "desc": label or addr})
        return sorted(ports, key=lambda x: x["port"])
    except Exception:
        return []


def get_all_ports():
    """รวม port จากทุกวิธี (pyserial → arduino-cli → PowerShell)"""
    result = []
    seen = set()

    if ensure_pyserial():
        for p in SERIAL_LIST_PORTS.comports():
            if p.device not in seen:
                seen.add(p.device)
                desc = p.description or p.device
                result.append({"port": p.device, "desc": desc})

    if not result and HAS_CLI:
        for p in list_ports_arduino_cli():
            if p["port"] not in seen:
                seen.add(p["port"])
                result.append(p)

    if not result:
        for p in list_ports_fallback():
            if p["port"] not in seen:
                seen.add(p["port"])
                result.append(p)

    return sorted(result, key=lambda x: x["port"])

PORT = int(os.environ.get("MODULUX_PORT", "8765"))
FQBN = "esp32:esp32:esp32s3"   # Board: ESP32-S3 Wroom-1

# ตรวจสอบ arduino-cli
def check_arduino_cli():
    try:
        r = subprocess.run(["arduino-cli", "version"], capture_output=True, text=True, timeout=5)
        return r.returncode == 0
    except FileNotFoundError:
        return False

HAS_CLI = check_arduino_cli()
ensure_pyserial()
if not HAS_CLI:
    print("[ERROR] ไม่พบ arduino-cli — ดาวน์โหลดได้ที่ https://arduino.github.io/arduino-cli/")
    print("        และรัน: arduino-cli core install esp32:esp32")


class ModuluxHandler(http.server.BaseHTTPRequestHandler):

    def log_message(self, format, *args):
        pass  # ปิด default log

    def _set_cors(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def do_OPTIONS(self):
        self.send_response(200)
        self._set_cors()
        self.end_headers()

    # ----- GET /ports -----
    def do_GET(self):
        if self.path == "/ports":
            ports = get_all_ports()
            data = json.dumps(ports).encode()
            self.send_response(200)
            self._set_cors()
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", len(data))
            self.end_headers()
            self.wfile.write(data)

        elif self.path == "/status":
            status = {
                "arduino_cli": HAS_CLI,
                "pyserial": HAS_SERIAL,
                "fqbn": FQBN
            }
            data = json.dumps(status).encode()
            self.send_response(200)
            self._set_cors()
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", len(data))
            self.end_headers()
            self.wfile.write(data)

        else:
            self.send_response(404)
            self.end_headers()

    # ----- POST /upload -----
    def do_POST(self):
        if self.path != "/upload":
            self.send_response(404)
            self.end_headers()
            return

        length = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(length))
        code = body.get("code", "")
        port = body.get("port", "")

        self.send_response(200)
        self._set_cors()
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Transfer-Encoding", "chunked")
        self.end_headers()

        def emit(line):
            text = (line + "\n").encode("utf-8")
            chunk = f"{len(text):X}\r\n".encode() + text + b"\r\n"
            try:
                self.wfile.write(chunk)
                self.wfile.flush()
            except BrokenPipeError:
                pass

        if not HAS_CLI:
            emit("[ERROR] arduino-cli ไม่ได้ติดตั้ง")
            emit("[END]")
            self._end_chunked()
            return

        if not port:
            emit("[ERROR] กรุณาเลือก COM Port ก่อน")
            emit("[END]")
            self._end_chunked()
            return

        tmpdir = tempfile.mkdtemp(prefix="modulux_")
        sketch_dir = os.path.join(tmpdir, "sketch")
        os.makedirs(sketch_dir)
        sketch_file = os.path.join(sketch_dir, "sketch.ino")

        with open(sketch_file, "w", encoding="utf-8") as f:
            f.write(code)

        try:
            # ---- Compile ----
            emit(f"[COMPILE] กำลัง Compile สำหรับ ESP32-S3 Wroom-1...")
            emit(f"[COMPILE] FQBN: {FQBN}")

            proc = subprocess.Popen(
                ["arduino-cli", "compile", "--fqbn", FQBN, sketch_dir,
                 "--warnings", "none"],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace"
            )
            for line in proc.stdout:
                line = line.rstrip()
                if line:
                    emit("[COMPILE] " + line)
            proc.wait()

            if proc.returncode != 0:
                emit("[ERROR] ❌ Compile ล้มเหลว — ตรวจสอบ ESP32 core: arduino-cli core install esp32:esp32")
                emit("[END]")
                self._end_chunked()
                return

            emit("[COMPILE] ✅ Compile สำเร็จ!")

            # ---- Upload ----
            emit(f"[UPLOAD] กำลัง Upload ไปยัง {port}...")

            proc = subprocess.Popen(
                ["arduino-cli", "upload", "--fqbn", FQBN, "-p", port, sketch_dir],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace"
            )
            for line in proc.stdout:
                line = line.rstrip()
                if line:
                    emit("[UPLOAD] " + line)
            proc.wait()

            if proc.returncode != 0:
                emit("[ERROR] ❌ Upload ล้มเหลว — ตรวจสอบ COM Port และการเชื่อมต่อบอร์ด")
            else:
                emit("[DONE] ✅ Upload สำเร็จ! บอร์ดกำลัง Restart...")

        except Exception as e:
            emit(f"[ERROR] {e}")

        finally:
            shutil.rmtree(tmpdir, ignore_errors=True)
            emit("[END]")
            self._end_chunked()

    def _end_chunked(self):
        try:
            self.wfile.write(b"0\r\n\r\n")
            self.wfile.flush()
        except:
            pass


if __name__ == "__main__":
    server = http.server.HTTPServer(("localhost", PORT), ModuluxHandler)
    print("=" * 50)
    print(f"  Modulux ESP32-S3 Server")
    print(f"  http://localhost:{PORT}")
    print(f"  arduino-cli : {'✅ พบแล้ว' if HAS_CLI else '❌ ไม่พบ'}")
    print(f"  pyserial    : {'✅ พบแล้ว' if HAS_SERIAL else '❌ ไม่พบ'}")
    print("=" * 50)
    print("กด Ctrl+C เพื่อหยุด server")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nServer หยุดทำงาน")
        sys.exit(0)
