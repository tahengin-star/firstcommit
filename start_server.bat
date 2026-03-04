@echo off
title Modulux ESP32-S3 Server
color 0A
echo ==================================================
echo   Modulux ESP32-S3 Blockly IDE Server
echo ==================================================
echo.

REM ตรวจสอบ arduino-cli
where arduino-cli >nul 2>&1
if errorlevel 1 (
    if exist "C:\Program Files\Arduino CLI\arduino-cli.exe" (
        set "PATH=C:\Program Files\Arduino CLI;%PATH%"
        echo [OK] พบ arduino-cli ที่ C:\Program Files\Arduino CLI
    ) else (
        echo [!] arduino-cli ไม่พบ
        echo     ดาวน์โหลดจาก: https://arduino.github.io/arduino-cli/
        echo     แล้ว add ไปใน PATH
        echo.
        echo     หลังติดตั้งแล้วรัน:
        echo     arduino-cli core install esp32:esp32
        echo.
    )
) else (
    echo [OK] arduino-cli พบแล้ว
)

echo.
echo กำลังเริ่ม Server ที่ http://localhost:8765 ...
echo เปิดไฟล์ index.html ในเบราว์เซอร์ก่อนแล้วกด Refresh
echo กด Ctrl+C เพื่อหยุด
echo.

powershell -ExecutionPolicy Bypass -File "%~dp0server.ps1" -Port 8765
pause
