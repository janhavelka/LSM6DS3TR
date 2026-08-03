@echo off
setlocal

if defined PLATFORMIO_CORE_DIR (
    set "PIO_CORE=%PLATFORMIO_CORE_DIR%"
) else (
    set "PIO_CORE=%USERPROFILE%\.platformio"
)

set "PIO_EXE=%PIO_CORE%\penv\Scripts\pio.exe"

if not exist "%PIO_EXE%" (
    >&2 echo PlatformIO was not found at: "%PIO_EXE%". Check PLATFORMIO_CORE_DIR or the current user's PlatformIO installation; do not install another Core automatically.
    exit /b 1
)

"%PIO_EXE%" %*
exit /b %ERRORLEVEL%
