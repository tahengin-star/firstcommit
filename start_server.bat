@echo off
title 🤖 Modulux Coding Server
color 0B
chcp 65001 >nul

echo.
echo  ╔══════════════════════════════════════════════════╗
echo  ║        🤖  Modulux Coding for Kids               ║
echo  ╚══════════════════════════════════════════════════╝
echo.

REM ── ตรวจสอบ arduino-cli ──────────────────────────────
where arduino-cli >nul 2>&1
if errorlevel 1 (
    if exist "C:\Program Files\Arduino CLI\arduino-cli.exe" (
        set "PATH=C:\Program Files\Arduino CLI;%PATH%"
    ) else (
        echo  [!] arduino-cli ไม่พบ
        echo      ดาวน์โหลดจาก: https://arduino.github.io/arduino-cli/
        echo      แล้วรัน: arduino-cli core install esp32:esp32
        echo.
    )
) else (
    echo  [OK] arduino-cli พบแล้ว
)

REM ── หยุด Server เก่าที่ค้างอยู่ ──────────────────────
echo  กำลังหยุด Server เก่า (ถ้ามี)...
for /f "tokens=5" %%a in ('netstat -ano 2^>nul ^| findstr ":8765 "') do (
    taskkill /F /PID %%a >nul 2>&1
)
timeout /T 2 /NOBREAK >nul

REM ── เริ่ม Server ──────────────────────────────────────
echo  กำลังเริ่ม Server...
echo.

REM แสดง LAN IP ปัจจุบัน
for /f "tokens=2 delims=:" %%a in ('ipconfig ^| findstr /c:"IPv4"') do (
    set LAN_IP=%%a
    goto :found_ip
)
:found_ip
set LAN_IP=%LAN_IP: =%

echo  ┌──────────────────────────────────────────────────┐
echo  │  🖥️  เครื่องนี้       : http://localhost:8765      │
echo  │  🌐 คอมอื่นใน LAN   : http://%LAN_IP%:8765  │
echo  │  📱 โทรศัพท์ (WiFi) : http://%LAN_IP%:8765  │
echo  │                                                  │
echo  │  (ต้องอยู่ WiFi เดียวกัน)                         │
echo  └──────────────────────────────────────────────────┘
echo.

REM เริ่ม server แล้วรอ 2 วินาทีก่อนเปิด browser
start /B powershell -ExecutionPolicy Bypass -File "%~dp0server.ps1" -Port 8765
timeout /T 2 /NOBREAK >nul

REM เปิด browser 1 ครั้ง
start "" http://localhost:8765

echo  Server กำลังทำงาน ปิดหน้าต่างนี้เพื่อหยุด Server
pause
