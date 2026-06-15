@echo off
setlocal EnableExtensions
title ESP32-P4 Build
set MSYSTEM=
set MSYS=
set MSYS2_ARG_CONV_EXCL=*
set MSYS_NO_PATHCONV=1

set "IDF_PATH=D:\esp32\Espressif\frameworks\esp-idf-v5.4.4"
set "ESP_ROM_ELF_DIR=D:\esp32\Espressif\tools\esp-rom-elfs\20241011"
set "IDF_PYTHON_ENV_PATH=D:\esp32\Espressif\python_env\idf5.4_py3.12_env"
set "PATH=D:\esp32\Espressif\tools\cmake\3.30.2\bin;D:\esp32\Espressif\tools\ninja\1.12.1;D:\esp32\Espressif\tools\riscv32-esp-elf\esp-14.2.0_20260121\riscv32-esp-elf\bin;%IDF_PYTHON_ENV_PATH%\Scripts;%PATH%"

echo ========================================
echo   ESP32-P4 Firmware Build
echo ========================================
echo.
echo Project: D:\esp32\esp32p4c5_edgeAI_lot
echo IDF:     %IDF_PATH%
echo Python:  %IDF_PYTHON_ENV_PATH%
echo.

python -c "import os,sys;[os.environ.pop(k,None) for k in ('MSYSTEM','MSYS','MSYS_NO_PATHCONV','MSYS2_ARG_CONV_EXCL')];sys.path.insert(0,os.path.join(os.environ['IDF_PATH'],'tools'));import idf;sys.argv=['idf.py','-C','D:/esp32/esp32p4c5_edgeAI_lot','build'];idf.main()"
set ERR=%ERRORLEVEL%
echo.
if %ERR%==0 (
    echo ========================================
    echo   BUILD OK - p4_face_stream.bin
    echo ========================================
) else (
    echo ========================================
    echo   BUILD FAILED (code: %ERR%)
    echo ========================================
)
pause
exit /b %ERR%
