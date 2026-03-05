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
import socket
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

# ค้นหา Local IP ของเครื่องนี้บน LAN
def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"

LOCAL_IP = get_local_ip()
# หา path ของ index.html (อยู่ในโฟลเดอร์เดียวกับ server.py)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
INDEX_PATH = os.path.join(SCRIPT_DIR, "index.html")
LAST_CODE_PATH = os.path.join(SCRIPT_DIR, "last_code.ino")


def add_fqbn_option(fqbn, key, value):
    """Append FQBN menu option safely if it does not already exist."""
    if f"{key}=" in fqbn:
        return fqbn
    # Base FQBN has 3 parts: package:architecture:board
    # If menu options already exist, they are appended as the 4th part and comma-separated.
    if fqbn.count(":") >= 3:
        return f"{fqbn},{key}={value}"
    return f"{fqbn}:{key}={value}"

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

    # ----- GET / (serve index.html to any device on LAN) -----
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            try:
                with open(INDEX_PATH, "rb") as f:
                    data = f.read()
                self.send_response(200)
                self._set_cors()
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", len(data))
                self.end_headers()
                self.wfile.write(data)
            except FileNotFoundError:
                self.send_response(404)
                self.end_headers()
                self.wfile.write(b"index.html not found")

        elif self.path == "/ports":
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

        elif self.path == "/info":
            info = {
                "lan_ip": LOCAL_IP,
                "port": PORT,
                "url": f"http://{LOCAL_IP}:{PORT}"
            }
            data = json.dumps(info).encode()
            self.send_response(200)
            self._set_cors()
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", len(data))
            self.end_headers()
            self.wfile.write(data)

        elif self.path == "/readcode":
            if os.path.exists(LAST_CODE_PATH):
                with open(LAST_CODE_PATH, "rb") as f:
                    data = f.read()
                self.send_response(200)
                self._set_cors()
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Content-Length", len(data))
                self.end_headers()
                self.wfile.write(data)
            else:
                msg = "// ยังไม่มีโค้ดที่เคย Upload".encode("utf-8")
                self.send_response(404)
                self._set_cors()
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Content-Length", len(msg))
                self.end_headers()
                self.wfile.write(msg)

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
        code     = body.get("code", "")
        port     = body.get("port", "")

        # บันทึกโค้ดล่าสุดที่ส่งมา Upload (ใช้สำหรับ /readcode)
        try:
            with open(LAST_CODE_PATH, "w", encoding="utf-8") as f:
                f.write(code)
        except Exception as e:
            print(f"[WARNING] บันทึก last_code.ino ไม่ได้: {e}")
        fqbn_req = body.get("fqbn", FQBN).strip() or FQBN
        # ESP32-S3 default is CDC disabled on boot; force enable so Web Serial Monitor can read output.
        if fqbn_req.startswith("esp32:esp32:esp32s3"):
            fqbn_req = add_fqbn_option(fqbn_req, "USBMode", "hwcdc")
            fqbn_req = add_fqbn_option(fqbn_req, "CDCOnBoot", "cdc")
        board_name = body.get("board_name", fqbn_req)

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

        # ---- แผนที่ #include → ชื่อ library ใน arduino-cli ----
        LIB_MAP = {
            "DHT.h":               "DHT sensor library",
            "Adafruit_Sensor.h":   "Adafruit Unified Sensor",
            "LiquidCrystal_I2C.h": "LiquidCrystal I2C",
            "LiquidCrystal.h":     "LiquidCrystal",
            "Adafruit_NeoPixel.h": "Adafruit NeoPixel",
            "ESP32Servo.h":        "ESP32Servo",
            "Wire.h":              None,   # built-in
            "Arduino.h":           None,   # built-in
            "SPI.h":               None,   # built-in
        }

        import re as _re
        includes = _re.findall(r'#include\s+[<"]([^>"]+)[>"]', code)
        for inc in includes:
            lib = LIB_MAP.get(inc)
            if lib is None:
                continue  # built-in หรือไม่รู้จัก
            # ตรวจว่า install แล้วหรือยัง
            chk = subprocess.run(
                ["arduino-cli", "lib", "list"],
                capture_output=True, text=True, timeout=10
            )
            if lib.lower() in chk.stdout.lower():
                continue  # มีแล้ว
            emit(f"[LIB] ติดตั้ง library: {lib} ...")
            r = subprocess.run(
                ["arduino-cli", "lib", "install", lib],
                capture_output=True, text=True, timeout=60
            )
            if r.returncode == 0:
                emit(f"[LIB] ✅ ติดตั้ง {lib} สำเร็จ")
            else:
                emit(f"[LIB] ⚠️ ติดตั้ง {lib} ไม่สำเร็จ: {r.stderr.strip()}")

        try:
            # ---- Compile ----
            emit(f"[COMPILE] กำลัง Compile สำหรับ {board_name}...")
            emit(f"[COMPILE] FQBN: {fqbn_req}")

            proc = subprocess.Popen(
                ["arduino-cli", "compile", "--fqbn", fqbn_req, sketch_dir,
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
                emit(f"[ERROR] ❌ Compile ล้มเหลว — ตรวจสอบ core ของ {fqbn_req.split(':')[0]}:{fqbn_req.split(':')[1] if ':' in fqbn_req else ''}")
                emit("[END]")
                self._end_chunked()
                return

            emit("[COMPILE] ✅ Compile สำเร็จ!")

            # ---- Upload ----
            emit(f"[UPLOAD] กำลัง Upload ไปยัง {port}...")

            proc = subprocess.Popen(
                ["arduino-cli", "upload", "--fqbn", fqbn_req, "-p", port, sketch_dir],
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
    # ฟังทุก interface (0.0.0.0) เพื่อให้อุปกรณ์อื่นในวง LAN เข้าถึงได้
    server = http.server.HTTPServer(("", PORT), ModuluxHandler)
    print("=" * 55)
    print(f"  🤖 Modulux Coding Server")
    print(f"  arduino-cli : {'✅ พบแล้ว' if HAS_CLI else '❌ ไม่พบ'}")
    print(f"  pyserial    : {'✅ พบแล้ว' if HAS_SERIAL else '❌ ไม่พบ'}")
    print("=" * 55)
    print(f"  🖥️  เครื่องนี้      : http://localhost:{PORT}")
    print(f"  🌐 คอมอื่นใน LAN  : http://{LOCAL_IP}:{PORT}")
    print(f"  📱 โทรศัพท์ใน WiFi : http://{LOCAL_IP}:{PORT}")
    print("=" * 55)
    print("  (ต้องอยู่ WiFi เดียวกัน)")
    print("  กด Ctrl+C เพื่อหยุด server")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nServer หยุดทำงาน")
        sys.exit(0)
