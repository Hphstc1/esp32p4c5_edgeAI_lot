@echo off
setlocal EnableExtensions
title ESP32-P4 Flash
set MSYSTEM=
set MSYS=
set MSYS2_ARG_CONV_EXCL=*
set MSYS_NO_PATHCONV=1

set "IDF_PYTHON_ENV_PATH=D:\esp32\Espressif\python_env\idf5.4_py3.12_env"
set "BUILD=D:\esp32\esp32p4c5_edgeAI_lot\build"
set "MODEL=D:\esp32\PestYOLO\pest15_320.espdl"

echo ========================================
echo   ESP32-P4 Flash (FW + Model)
echo ========================================
echo.

:: --- Get port ---
if not "%1"=="" (
    set "PORT=%1"
    echo Port: %PORT% (manual)
    goto :check_files
)

echo Auto-detecting COM port...
set "PORT="
for /f "tokens=*" %%d in ('python -c "import serial.tools.list_ports; [print(p.device) for p in serial.tools.list_ports.comports()]" 2^>nul') do set "PORT=%%d"
if not "%PORT%"=="" (
    echo Port: %PORT% (auto)
    goto :check_files
)

echo.
echo ERROR: No COM port found and none specified.
echo.
echo Please run with port number:
echo   flash_p4.bat COM3
echo.
echo If unsure, check Device Manager ^> Ports ^(COM ^& LPT^)
echo or run in PowerShell: [System.IO.Ports.SerialPort]::GetPortNames()
pause
exit /b 1

:: --- Check files ---
:check_files
echo.
echo === Checking files ===
set MISSING=0
if not exist "%BUILD%\bootloader\bootloader.bin" (
    echo   MISSING: bootloader\bootloader.bin
    set MISSING=1
)
if not exist "%BUILD%\partition_table\partition-table.bin" (
    echo   MISSING: partition_table\partition-table.bin
    set MISSING=1
)
if not exist "%BUILD%\p4_face_stream.bin" (
    echo   MISSING: p4_face_stream.bin
    set MISSING=1
)
if not exist "%MODEL%" (
    echo   MISSING: pest15_320.espdl
    set MISSING=1
)
if %MISSING%==1 (
    echo.
    echo Run build_p4.bat first, and make sure pest15_320.espdl is in D:\esp32\PestYOLO\
    pause
    exit /b 1
)
echo   All files OK
echo.

:: --- Flash ---
set "PY=D:\esp32\Espressif\python_env\idf5.4_py3.12_env\Scripts\python.exe"

echo [1/4] bootloader @ 0x2000
"%PY%" -m esptool --chip esp32p4 -p %PORT% -b 460800 --before default_reset --after no_reset write_flash --verify --flash_mode dio --flash_size 16MB --flash_freq 80m 0x2000 "%BUILD%\bootloader\bootloader.bin"
if errorlevel 1 goto :err

echo [2/4] partition table @ 0x8000
"%PY%" -m esptool --chip esp32p4 -p %PORT% -b 460800 --before no_reset --after no_reset write_flash --verify --flash_mode dio --flash_size 16MB --flash_freq 80m 0x8000 "%BUILD%\partition_table\partition-table.bin"
if errorlevel 1 goto :err

echo [3/4] firmware @ 0x10000
"%PY%" -m esptool --chip esp32p4 -p %PORT% -b 460800 --before no_reset --after no_reset write_flash --verify --flash_mode dio --flash_size 16MB --flash_freq 80m 0x10000 "%BUILD%\p4_face_stream.bin"
if errorlevel 1 goto :err

echo [4/4] pest model @ 0x910000
"%PY%" -m esptool --chip esp32p4 -p %PORT% -b 460800 --before no_reset --after hard_reset write_flash --verify --flash_mode dio --flash_size 16MB --flash_freq 80m 0x910000 "%MODEL%"
if errorlevel 1 goto :err

echo.
echo ========================================
echo   FLASH OK (verified)
echo.
echo   1. Open serial terminal: 115200-8-N-1
echo   2. Press RST button on board
echo   3. Watch for: ESP-ROM:esp32p4
echo ========================================
pause
exit /b 0

:err
echo.
echo ========================================
echo   FLASH FAILED
echo   Check: port=%PORT%, cable, boot mode
echo ========================================
pause
exit /b 1
