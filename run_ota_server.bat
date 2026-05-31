@echo off
chcp 65001 >nul
echo ============================================
echo ESP32 LoRa Broker - OTA Server
 echo ============================================
echo.

REM Vérifier que Python est installé
python --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python is not installed or not in PATH
    echo Please install Python 3.7+
    pause
    exit /b 1
)

REM Vérifier que le firmware existe
echo Checking firmware.bin...
if not exist ".pio\build\ttgo-lora32-v1\firmware.bin" (
    echo ERROR: firmware.bin not found!
    echo Please build the project first:
    echo   pio run
    pause
    exit /b 1
)

echo.
echo Starting OTA Server on port 8000...
echo Press Ctrl+C to stop the server
echo.
echo Configure your ESP32 credentials.h with:
echo   #define OTA_SERVER "<your-pc-ip>"
echo   #define OTA_PORT 8000
echo   #define OTA_PATH "/firmware.bin"
echo.
echo To find your PC IP, run: ipconfig
ping -n 3 127.0.0.1 >nul

python ota_server.py --port 8000

pause
