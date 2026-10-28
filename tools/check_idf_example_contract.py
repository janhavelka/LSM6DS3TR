#!/usr/bin/env python3
"""Enforce native ESP-IDF CLI parity and framework-boundary contracts."""

from __future__ import annotations

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
ARDUINO_MAIN = ROOT / "examples" / "01_basic_bringup_cli" / "main.cpp"
IDF_MAIN_DIR = ROOT / "examples" / "idf" / "basic" / "main"
PROFILE_HELPER = ROOT / "examples" / "common" / "ProfileCli.h"

COMMANDS = {
    "help", "?", "version", "ver", "status", "diag", "job", "result",
    "bind", "unbind", "cancel", "scan", "addr", "freq", "profile", "cfg",
    "settings", "probe", "configure", "sample", "reset", "boot", "recover",
    "reconcile", "powerdown", "selftest", "calxl", "calg", "purge", "fifo", "counterreset", "events", "rreg",
    "wreg", "dump", "stress", "stress_mix",
}

PROFILE_FIELDS = {
    "xl_odr", "xl_fs", "xl_power", "xl_lpf2", "xl_filter", "xl_hpcf", "xl_input", "xl_analog", "g_ftype", "xl_slope_hp", "xl_6d_lpf",
    "g_odr", "g_fs", "g_power", "g_lpf1", "g_hpf", "g_hpf_mode",
    "g_sleep", "bdu", "offset_weight", "offset", "fifo",
    "fifo_mode", "fifo_odr", "fifo_g_dec", "fifo_xl_dec", "fifo_watermark",
    "fifo_fourth", "fifo_fourth_dec", "fifo_trigger", "timestamp_resolution",
    "interrupts", "irq", "irq_polarity", "irq_drive", "irq_drdy", "irq_latch", "irq_combine", "int1", "int2", "orientation",
}

REQUIRED_IDF_TOKENS = (
    'extern "C" void app_main',
    '#include "driver/i2c_master.h"',
    '#include "ProfileCli.h"',
    "i2c_new_master_bus",
    "i2c_master_bus_add_device",
    "i2c_master_bus_rm_device",
    "i2c_master_probe",
    "i2c_master_transmit",
    "i2c_master_transmit_receive",
    "esp_timer_get_time",
    "vTaskDelay",
    "getchar()",
    "O_NONBLOCK",
    "clearerr(stdin)",
    "char input[",
    "OperationTiming",
    "startFifoRead",
    "startCounterReset",
    "startEventRead",
    "EventReadRequest",
    "profile_cli::eventRequest",
    "result.events.requestedMask",
    "result.events.mayHaveAcknowledgedMask",
    "result.events.acknowledgedMask",
    "result.events.historyMayBeLost",
    "printEventObservation",
    "timestamp_end_status=unavailable",
    "pad_level=unobserved",
    "observations=sequential",
    "expected events [ready|fifo|embedded|orientation|all]",
    "TaskHandle_t interruptOwnerTask = nullptr",
    "interruptOwnerTask = xTaskGetCurrentTaskHandle()",
    "xTaskNotifyFromISR(interruptOwnerTask, 1U, eSetBits, &higherPriorityTaskWoken)",
    "xTaskNotifyWait(0U, 1U, &notificationBits, 0)",
    "bool interruptNotificationPending = false",
    "serviceInterruptNotification(nowMs())",

    "result.counters.epoch",
    "result.counters.timestampResetMayHaveOccurred",
    "result.counters.stepResetMayHaveOccurred",
    "result.registerBankMayBeUnknown",
    "record.kind == FifoRecordKind::TEMPERATURE",
    "record.kind == FifoRecordKind::TIMESTAMP_STEP",
    "record.counterEpoch",
    "record.timestampTicks",
    "record.stepCount",
    "wraps=unexpanded",
    "freshness=unverified",
    "expected counterreset <timestamp|steps|both>",
    "FifoMotionRecord fifoRecords[FIFO_RECORD_CAPACITY]",
    "request.stopCollection = count == 3U",
    "result.token == fifoBufferToken",
    "OperationToken",
    "DeviceProfile stagedProfile",
    "profile_cli::setField",
    "profile_cli::sampleRequest",
    "SampleQuality::FILTER_UNVERIFIED",
    "ConfigurationState::FILTER_UNQUALIFIED",
    "raw.qualifiedMask",
    "record.qualifiedMask",
    "events.qualifiedMask",
    "raw.filters",
    "filter_context",
    "result.configuration.referenceDiscardConfirmed",
    "configure_budget_ms=",
    "profile_cli::callbackBudgetMs",
    "profile_cli::sampleBudgetMs",
    "maximumCalibrationTransactions(request.samples)",
    "sample gyro unqualified",
    "device.poll(now, POLL_TRANSACTION_BUDGET)",
    "device.takeResult",
    "device.cancelActiveJob",
    "SessionKind::SCAN",
    "SessionKind::STRESS",
    "SessionKind::STRESS_MIX",
    "validateSessionResult",
    "result.state != OperationState::SUCCEEDED",
    "result.hardwareStateMayHaveChanged",
    "last_error present=",
    "bus_init code=",
    "mismatch present=",
    "job poll token=",
    "scan summary attempted=",
    "transport_fail_delta=",
    "calibrationTimeoutMs",
    "odrPeriodUs(odr)",
    "fflush(stdout)",
)

FORBIDDEN_PATTERNS = (
    r"ArduinoCompat",
    r"IdfArduinoCompat",
    r"Arduino\.h",
    r"Wire\.h",
    r"\bString\b",
    r"\bSerial\b",
    r"\bTwoWire\b",
    r"01_basic_bringup_cli/main\.cpp",
    r"driver/i2c\.h",
    r"i2c_cmd_link",
    r"i2c_driver_install",
    r"std::vector",
    r"std::string",
    r"\bmalloc\s*\(",
    r"\bcalloc\s*\(",
    r"\brealloc\s*\(",
)


def fail(message: str) -> None:
    print(f"IDF example contract FAILED: {message}")
    raise SystemExit(1)


def read(path: pathlib.Path, label: str) -> str:
    if not path.exists():
        fail(f"missing {label}: {path.as_posix()}")
    return path.read_text(encoding="utf-8", errors="replace")


def handled_commands(text: str) -> set[str]:
    return set(re.findall(r'strcmp\(command,\s*"([^"]+)"\)', text))


def function_body(text: str, signature: str, label: str) -> str:
    """Extract a balanced definition body, ignoring forward declarations."""
    match = re.search(rf"\b{re.escape(signature)}\s*\([^)]*\)\s*\{{", text)
    if match is None:
        fail(f"{label} definition not found")
    start = match.end() - 1
    depth = 0
    for index in range(start, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[start:index + 1]
    fail(f"{label} definition is unbalanced")
    return ""


def main() -> int:
    main_text = read(IDF_MAIN_DIR / "main.cpp", "native ESP-IDF main")
    cmake_text = read(IDF_MAIN_DIR / "CMakeLists.txt", "ESP-IDF main CMake")
    arduino_text = read(ARDUINO_MAIN, "Arduino owner-safe example")
    helper_text = read(PROFILE_HELPER, "shared typed profile helper")
    combined = main_text + "\n" + cmake_text

    for token in REQUIRED_IDF_TOKENS:
        if token not in combined:
            fail(f"required native ESP-IDF token missing: {token}")
    for component in (
        "LSM6DS3TR", "esp_driver_i2c", "esp_timer", "freertos", "heap",
        "spi_flash",
    ):
        if re.search(rf"\b{re.escape(component)}\b", cmake_text) is None:
            fail(f"IDF CMake missing required component '{component}'")
    if '"../../../common"' not in cmake_text:
        fail("IDF example must include only the narrow shared-example helper path")
    if '"../../../.."' in cmake_text:
        fail("IDF example must not add the repository root as an include path")
    for pattern in FORBIDDEN_PATTERNS:
        if re.search(pattern, combined):
            fail(f"forbidden Arduino/dynamic-allocation token present: {pattern}")
    for stale in ("ArduinoCompat.cpp", "Arduino.h", "Wire.h"):
        if (IDF_MAIN_DIR / stale).exists():
            fail(f"stale compatibility file remains: {stale}")

    irq_hook = re.search(r"void IRAM_ATTR onImuInterrupt\([^)]*\)\s*\{([^}]+)\}", main_text)
    expected_irq_body = (
        "if (interruptOwnerTask == nullptr) return; "
        "BaseType_t higherPriorityTaskWoken = pdFALSE; "
        "(void)xTaskNotifyFromISR(interruptOwnerTask, 1U, eSetBits, &higherPriorityTaskWoken); "
        "if (higherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();"
    )
    if irq_hook is None or " ".join(irq_hook.group(1).split()) != expected_irq_body:
        fail("native ISR hook must only publish the application-owned notification")
    if "gpio_isr_handler_add(" in main_text:
        fail("native diagnostic example must not attach an unconfigured fixture GPIO")

    idf_commands = handled_commands(main_text)
    arduino_commands = handled_commands(arduino_text)
    if idf_commands != COMMANDS:
        fail(
            "native CLI command mismatch: "
            f"missing={sorted(COMMANDS - idf_commands)} "
            f"extra={sorted(idf_commands - COMMANDS)}"
        )
    if arduino_commands != idf_commands:
        fail(
            "Arduino/ESP-IDF command mismatch: "
            f"arduino_only={sorted(arduino_commands - idf_commands)} "
            f"idf_only={sorted(idf_commands - arduino_commands)}"
        )
    for field in PROFILE_FIELDS:
        if f'"{field}"' not in helper_text or field not in main_text:
            fail(f"typed profile field '{field}' is not shared and documented")

    if "POLL_TRANSACTION_BUDGET = 1" not in main_text:
        fail("native CLI must demonstrate one-transaction driver polling")
    if "serviced < INPUT_CHARS_PER_LOOP" not in main_text:
        fail("native console input work must be bounded per loop")
    if not re.search(
        r"if \(c == '\\n'\).*?inputOverflow = false;\s*return;",
        main_text,
        re.DOTALL,
    ):
        fail("native console must return after servicing one complete command")
    if not re.search(
        r"serviceInput\(\);\s*serviceOperation\(nowMs\(\)\);", main_text
    ):
        fail("native owner loop must sample time after command admission")
    if not re.search(
        r"busInitializationStatus = configureI2c\(\);\s*"
        r"printStatus\(busInitializationStatus\);\s*"
        r"if \(busInitializationStatus\.ok\(\)\) \{.*?\}\s*cliLoop\(\);",
        main_text,
        re.DOTALL,
    ):
        fail("native CLI must remain available after an I2C initialization failure")

    generic_mapper = function_body(
        main_text, "Status mapEspError", "generic native I2C error mapper"
    )
    if "I2C_NACK" in generic_mapper or "I2C_BUSY" in generic_mapper:
        fail("generic native I2C errors must not invent NACK or busy context")
    probe_mapper = function_body(
        main_text, "Status mapEspProbeError", "native address-probe error mapper"
    )
    if "ESP_ERR_NOT_FOUND" not in probe_mapper or "I2C_NACK_ADDR" not in probe_mapper:
        fail("only the address-probe mapper may classify NOT_FOUND as NACK")
    if not re.search(
        r"Status probeAddress\(.*?return mapEspProbeError\(\s*"
        r"i2c_master_probe\(",
        main_text,
        re.DOTALL,
    ):
        fail("native address probing must use its context-specific error mapper")
    if not re.search(
        r"bool ownerMutationBlocked\(\).*?session\.kind != SessionKind::NONE.*?"
        r"device\.operationActive\(\).*?device\.resultPending\(\)",
        main_text,
        re.DOTALL,
    ):
        fail("all native owner/session states must gate mutations")

    replace_start = main_text.find("HandleReplacement replaceDeviceHandle")
    normal_replace = main_text.find(
        "i2c_master_dev_handle_t previousDevice", replace_start
    )
    add_index = main_text.find("replacement.status = addDevice", normal_replace)
    remove_index = main_text.find(
        "replacement.status = i2c_master_bus_rm_device(previousDevice)",
        replace_start,
    )
    commit_index = main_text.find("i2c.device = candidate", remove_index)
    if min(replace_start, normal_replace, add_index, remove_index, commit_index) < 0:
        fail("native I2C handle replacement evidence is incomplete")
    if not add_index < remove_index < commit_index:
        fail("candidate handle must be added before old-handle removal and commit")
    if "replacement.originalPreserved = true" not in main_text:
        fail("failed handle replacement must expose preserved-owner evidence")
    if "i2c.orphanDevice = candidate" not in main_text:
        fail("failed candidate cleanup must remain tracked for a bounded retry")

    for marker in (
        "frequency != 100000U && frequency != 400000U",
        "address != 0x6AU && address != 0x6BU",
        "result.state != OperationState::SUCCEEDED",
        "result.hardwareStateMayHaveChanged",
        "Session job reported zero transactions",
        'kind == SessionKind::STRESS_MIX\n             ? "mixed"',
        "lastResultAvailable = false",
        "(periodMs + 1U) * static_cast<uint64_t>(request.samples) * 3U + 5000U",
        "expected sample [all|accel|gyro|temp] [ready|direct|unqualified]",
        "expected profile [show|validate|defaults|apply|set <field> <value...>]",
        "expected selftest [5..100]",
        "expected calxl [samples [x y z [max_p2p_g]]]",
        "expected calg [samples [max_p2p_dps]]",
        "Stress: stress_mix [1..10000]",
    ):
        if marker not in main_text:
            fail(f"native CLI safety/arity marker missing: {marker}")

    print("IDF example contract PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
