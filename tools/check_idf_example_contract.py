#!/usr/bin/env python3
from __future__ import annotations

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]

ARDUINO_CLI = ROOT / "examples" / "01_basic_bringup_cli" / "main.cpp"
IDF_MAIN_DIR = ROOT / "examples" / "idf" / "basic" / "main"
IDF_CMAKE = IDF_MAIN_DIR / "CMakeLists.txt"
IDF_README = ROOT / "examples" / "idf" / "basic" / "README.md"

MANDATORY_COMMANDS = [
    "help",
    "version",
    "ver",
    "scan",
    "begin",
    "drv",
    "drv1",
    "cfg",
    "settings",
    "refresh",
    "verbose",
    "read",
    "raw",
    "accel",
    "gyro",
    "temp",
    "status",
    "tsread",
    "steps",
    "fifo",
    "fifo_read",
    "stream",
    "odrxl",
    "odrg",
    "fsxl",
    "fsg",
    "apm",
    "gpm",
    "gsleep",
    "reset",
    "boot",
    "alpf2",
    "aslope",
    "a6d",
    "glpf1",
    "ghpf",
    "ghpfmode",
    "ts",
    "tshr",
    "tsreset",
    "pedo",
    "sigmot",
    "tilt",
    "wtilt",
    "stepreset",
    "ofswt",
    "offset",
    "fifo_mode",
    "fifo_odr",
    "fifo_xl",
    "fifo_g",
    "fifo_th",
    "fifo_temp",
    "fifo_step",
    "fifo_stop",
    "fifo_high",
    "cal",
    "calxl",
    "calg",
    "biasxl",
    "biasg",
    "biasreset",
    "probe",
    "recover",
    "whoami",
    "id",
    "wusrc",
    "tapsrc",
    "6dsrc",
    "funcsrc1",
    "funcsrc2",
    "wtstatus",
    "shub",
    "selftest",
    "stress",
    "stress_mix",
    "rreg",
    "wreg",
    "dump",
]


def fail(message: str) -> None:
    print(f"IDF example contract FAILED: {message}")
    raise SystemExit(1)


def require(path: pathlib.Path, label: str) -> str:
    if not path.exists():
        fail(f"missing {label}: {path.as_posix()}")
    return path.read_text(encoding="utf-8", errors="replace")


def main() -> int:
    arduino_text = require(ARDUINO_CLI, "shared Arduino CLI source")
    cmake_text = require(IDF_CMAKE, "IDF main CMake")
    readme_text = require(IDF_README, "IDF README")
    compat_text = require(IDF_MAIN_DIR / "ArduinoCompat.cpp", "IDF Arduino compatibility source")
    require(IDF_MAIN_DIR / "Arduino.h", "IDF Arduino compatibility header")
    require(IDF_MAIN_DIR / "Wire.h", "IDF Wire compatibility header")

    if "../../../01_basic_bringup_cli/main.cpp" not in cmake_text:
        fail("IDF example must compile the shared Arduino bringup CLI source")

    required_components = [
        "LSM6DS3TR",
        "esp_driver_i2c",
        "esp_driver_gpio",
        "esp_timer",
        "freertos",
    ]
    for component in required_components:
        if component not in cmake_text:
            fail(f"IDF CMake missing required component '{component}'")

    required_i2c_symbols = [
        "driver/i2c_master.h",
        "i2c_new_master_bus",
        "i2c_master_bus_add_device",
        "i2c_master_transmit",
        "i2c_master_transmit_receive",
        "i2c_master_receive",
        "i2c_master_probe",
    ]
    for symbol in required_i2c_symbols:
        if symbol not in compat_text and symbol not in require(IDF_MAIN_DIR / "Wire.h", "IDF Wire header"):
            fail(f"IDF I2C compatibility layer missing '{symbol}'")

    for command in MANDATORY_COMMANDS:
        if re.search(rf"\b{re.escape(command)}\b", arduino_text) is None:
            fail(f"shared CLI source missing mandatory command '{command}'")

    stale_phrases = [
        " ".join(("minimal", "IDF", "example")),
        " ".join(("minimal", "ESP-IDF", "example")),
        " ".join(("periodic", "sampler")),
    ]
    combined_text = "\n".join([readme_text, cmake_text, compat_text])
    for phrase in stale_phrases:
        if phrase.lower() in combined_text.lower():
            fail(f"stale wording still present: '{phrase}'")

    print("IDF example contract PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
