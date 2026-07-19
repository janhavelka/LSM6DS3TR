/// @file FakeBus.h
/// @brief Fixed-memory LSM6DS3TR transport fake with deterministic fault injection.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

#include "LSM6DS3TR/CommandTable.h"
#include "LSM6DS3TR/Config.h"

namespace LSM6DS3TRTest {

using LSM6DS3TR::DriverConfig;
using LSM6DS3TR::Err;
using LSM6DS3TR::SensorAddress;
using LSM6DS3TR::Status;
namespace cmd = LSM6DS3TR::cmd;

enum class TransferKind : uint8_t {
  WRITE,
  WRITE_READ,
};

struct Transfer {
  TransferKind kind = TransferKind::WRITE_READ;
  uint8_t address = 0;
  uint8_t startReg = 0;
  uint8_t firstValue = 0;
  uint16_t txLength = 0;
  uint16_t rxLength = 0;
  uint32_t timeoutMs = 0;
  uint64_t atMs = 0;
  Err result = Err::OK;
  bool writeEffectApplied = false;
};

struct TransferFault {
  uint32_t call = 0;
  Status status = Status::Error(Err::I2C_ERROR, "injected transfer failure");
  bool applyWriteEffect = false;
  bool consumed = false;
};

struct FakeBus {
  static constexpr size_t MAX_TRACE = 512;
  static constexpr size_t MAX_FAULTS = 8;

  uint64_t nowMs = 1000;
  uint32_t transferCalls = 0;
  uint32_t readCalls = 0;
  uint32_t writeCalls = 0;
  Transfer trace[MAX_TRACE] = {};
  size_t traceCount = 0;
  TransferFault faults[MAX_FAULTS] = {};
  size_t faultCount = 0;

  uint8_t regs[256] = {};
  uint32_t notReadyStatusReads = 0;
  uint8_t corruptReadRegister = 0;
  uint8_t corruptReadValue = 0;
  uint32_t corruptReadRemaining = 0;
  bool alternateAccelInt16Extremes = false;
  uint32_t accelDataReads = 0;
  bool holdResetBit = false;
  bool holdBootBit = false;
  uint8_t resetBitReadsRemaining = 0;
  uint8_t bootBitReadsRemaining = 0;
  bool resetRegistersOnSoftwareReset = false;
  bool resetRegistersOnBoot = false;
  bool simulateSelfTest = true;
  bool negativeSelfTestStimulus = false;
  uint32_t commandInaccessibleMs = 0;
  uint64_t inaccessibleUntilMs = 0;

  uint16_t fifoUnreadWords = 0;
  uint16_t fifoPattern = 0;
  uint16_t fifoNextWord = 0;
  bool fifoOverrun = false;
  uint16_t fifoConcurrentArrivalWords = 0;
  uint16_t fifoDataReads = 0;
  bool fifoArrivalInjected = false;

  FakeBus() { resetRegisters(); }

  void resetRegisters() {
    std::memset(regs, 0, sizeof(regs));
    regs[cmd::REG_WHO_AM_I] = cmd::WHO_AM_I_VALUE;
    regs[cmd::REG_STATUS_REG] =
        static_cast<uint8_t>(cmd::MASK_XLDA | cmd::MASK_GDA | cmd::MASK_TDA);
    setRawSample(6400, 1000, -500, 0, 0, 0, 16384);
    syncFifoRegisters();
  }

  void clearTrace() {
    transferCalls = 0;
    readCalls = 0;
    writeCalls = 0;
    traceCount = 0;
    std::memset(trace, 0, sizeof(trace));
    faultCount = 0;
    std::memset(faults, 0, sizeof(faults));
  }

  void addFault(uint32_t call, const Status& status, bool applyWriteEffect = false) {
    if (faultCount >= MAX_FAULTS) {
      return;
    }
    faults[faultCount++] = TransferFault{call, status, applyWriteEffect, false};
  }

  TransferFault* faultForCurrentCall() {
    for (size_t i = 0; i < faultCount; ++i) {
      if (!faults[i].consumed && faults[i].call == transferCalls) {
        faults[i].consumed = true;
        return &faults[i];
      }
    }
    return nullptr;
  }

  Transfer* beginTransfer(TransferKind kind, uint8_t address, uint8_t startReg,
                          size_t txLength, size_t rxLength, uint32_t timeoutMs,
                          uint8_t firstValue = 0) {
    transferCalls++;
    if (kind == TransferKind::WRITE) {
      writeCalls++;
    } else {
      readCalls++;
    }
    if (traceCount >= MAX_TRACE) {
      return nullptr;
    }
    Transfer& transfer = trace[traceCount++];
    transfer.kind = kind;
    transfer.address = address;
    transfer.startReg = startReg;
    transfer.firstValue = firstValue;
    transfer.txLength = static_cast<uint16_t>(txLength);
    transfer.rxLength = static_cast<uint16_t>(rxLength);
    transfer.timeoutMs = timeoutMs;
    transfer.atMs = nowMs;
    return &transfer;
  }

  static void finishTransfer(Transfer* transfer, const Status& status,
                             bool effectApplied = false) {
    if (transfer == nullptr) {
      return;
    }
    transfer->result = status.code;
    transfer->writeEffectApplied = effectApplied;
  }

  void setRawSample(int16_t temperature, int16_t gx, int16_t gy, int16_t gz,
                    int16_t ax, int16_t ay, int16_t az) {
    setI16(cmd::REG_OUT_TEMP_L, temperature);
    setI16(cmd::REG_OUTX_L_G, gx);
    setI16(cmd::REG_OUTY_L_G, gy);
    setI16(cmd::REG_OUTZ_L_G, gz);
    setI16(cmd::REG_OUTX_L_XL, ax);
    setI16(cmd::REG_OUTY_L_XL, ay);
    setI16(cmd::REG_OUTZ_L_XL, az);
  }

  void setFifo(uint16_t unreadWords, uint16_t pattern, uint16_t firstWord,
               bool overrun = false) {
    fifoUnreadWords = unreadWords;
    fifoPattern = pattern;
    fifoNextWord = firstWord;
    fifoOverrun = overrun;
    syncFifoRegisters();
  }

  void syncFifoRegisters() {
    regs[cmd::REG_FIFO_STATUS1] = static_cast<uint8_t>(fifoUnreadWords & 0xFFu);
    regs[cmd::REG_FIFO_STATUS2] =
        static_cast<uint8_t>((fifoUnreadWords >> 8) & cmd::MASK_DIFF_FIFO_HI);
    if (fifoUnreadWords == 0u) {
      regs[cmd::REG_FIFO_STATUS2] |= cmd::MASK_FIFO_EMPTY;
    }
    if (fifoOverrun) {
      regs[cmd::REG_FIFO_STATUS2] |= cmd::MASK_FIFO_OVER_RUN;
    }
    regs[cmd::REG_FIFO_STATUS3] = static_cast<uint8_t>(fifoPattern & 0xFFu);
    regs[cmd::REG_FIFO_STATUS4] = static_cast<uint8_t>((fifoPattern >> 8) & 0x03u);
    setI16(cmd::REG_FIFO_DATA_OUT_L, static_cast<int16_t>(fifoNextWord));
  }

  void applyWrite(const uint8_t* data, size_t length) {
    const uint8_t startReg = data[0];
    for (size_t i = 1; i < length; ++i) {
      const uint8_t reg = static_cast<uint8_t>(startReg + i - 1u);
      const uint8_t value = data[i];
      if (reg != cmd::REG_CTRL3_C) {
        regs[reg] = value;
        if (reg == cmd::REG_CTRL5_C && simulateSelfTest) {
          if ((value & cmd::MASK_ST_XL) != 0u) {
            if (negativeSelfTestStimulus) {
              setRawSample(6400, 1000, -500, 0, -2000, -2000, 14384);
            } else {
              setRawSample(6400, 1000, -500, 0, 2000, 2000, 18384);
            }
          } else if ((value & cmd::MASK_ST_G) != 0u) {
            if (negativeSelfTestStimulus) {
              setRawSample(6400, -2000, -4500, -3000, 0, 0, 16384);
            } else {
              setRawSample(6400, 4000, 3500, 3000, 0, 0, 16384);
            }
          } else {
            setRawSample(6400, 1000, -500, 0, 0, 0, 16384);
          }
        }
        continue;
      }

      const bool softwareReset = (value & cmd::MASK_SW_RESET) != 0u;
      const bool boot = (value & cmd::MASK_BOOT) != 0u;
      const bool resetImage = (softwareReset && resetRegistersOnSoftwareReset) ||
                              (boot && resetRegistersOnBoot);
      if ((softwareReset || boot) && commandInaccessibleMs != 0u) {
        inaccessibleUntilMs = nowMs + commandInaccessibleMs;
      }
      if (resetImage) {
        resetRegisters();
      }

      uint8_t stored = resetImage ? 0u : value;
      if (!holdResetBit && resetBitReadsRemaining == 0u) {
        stored = static_cast<uint8_t>(stored & ~cmd::MASK_SW_RESET);
      }
      if (!holdBootBit && bootBitReadsRemaining == 0u) {
        stored = static_cast<uint8_t>(stored & ~cmd::MASK_BOOT);
      }
      regs[reg] = stored;
    }
  }

  void setI16(uint8_t lowRegister, int16_t value) {
    const uint16_t bits = static_cast<uint16_t>(value);
    regs[lowRegister] = static_cast<uint8_t>(bits & 0xFFu);
    regs[static_cast<uint8_t>(lowRegister + 1u)] = static_cast<uint8_t>(bits >> 8);
  }
};

inline Status fakeWrite(uint8_t address, const uint8_t* data, size_t length,
                        uint32_t timeoutMs, void* user) {
  FakeBus& bus = *static_cast<FakeBus*>(user);
  if (data == nullptr || length < 2u) {
    return Status::Error(Err::INVALID_PARAM, "invalid fake write arguments");
  }

  Transfer* transfer = bus.beginTransfer(TransferKind::WRITE, address, data[0],
                                         length, 0u, timeoutMs, data[1]);
  if (bus.inaccessibleUntilMs != 0u && bus.nowMs < bus.inaccessibleUntilMs) {
    const Status status =
        Status::Error(Err::I2C_TIMEOUT, "device temporarily inaccessible", -900);
    FakeBus::finishTransfer(transfer, status);
    return status;
  }

  TransferFault* fault = bus.faultForCurrentCall();
  if (fault != nullptr && !fault->applyWriteEffect) {
    FakeBus::finishTransfer(transfer, fault->status);
    return fault->status;
  }

  bus.applyWrite(data, length);
  if (fault != nullptr) {
    FakeBus::finishTransfer(transfer, fault->status, true);
    return fault->status;
  }

  const Status status = Status::Ok();
  FakeBus::finishTransfer(transfer, status, true);
  return status;
}

inline Status fakeWriteRead(uint8_t address, const uint8_t* txData, size_t txLength,
                            uint8_t* rxData, size_t rxLength, uint32_t timeoutMs,
                            void* user) {
  FakeBus& bus = *static_cast<FakeBus*>(user);
  if (txData == nullptr || txLength == 0u || (rxLength > 0u && rxData == nullptr)) {
    return Status::Error(Err::INVALID_PARAM, "invalid fake read arguments");
  }

  const uint8_t startReg = txData[0];
  Transfer* transfer = bus.beginTransfer(TransferKind::WRITE_READ, address, startReg,
                                         txLength, rxLength, timeoutMs);
  if (bus.inaccessibleUntilMs != 0u && bus.nowMs < bus.inaccessibleUntilMs) {
    const Status status =
        Status::Error(Err::I2C_TIMEOUT, "device temporarily inaccessible", -901);
    FakeBus::finishTransfer(transfer, status);
    return status;
  }

  TransferFault* fault = bus.faultForCurrentCall();
  if (fault != nullptr) {
    FakeBus::finishTransfer(transfer, fault->status);
    return fault->status;
  }

  if (startReg == cmd::REG_STATUS_REG && bus.notReadyStatusReads > 0u) {
    bus.notReadyStatusReads--;
    std::memset(rxData, 0, rxLength);
  } else {
    const bool countdownReset = startReg == cmd::REG_CTRL3_C &&
                                bus.resetBitReadsRemaining > 0u;
    const bool countdownBoot = startReg == cmd::REG_CTRL3_C &&
                               bus.bootBitReadsRemaining > 0u;
    if (countdownReset) {
      bus.regs[cmd::REG_CTRL3_C] = static_cast<uint8_t>(
          bus.regs[cmd::REG_CTRL3_C] | cmd::MASK_SW_RESET);
    }
    if (countdownBoot) {
      bus.regs[cmd::REG_CTRL3_C] =
          static_cast<uint8_t>(bus.regs[cmd::REG_CTRL3_C] | cmd::MASK_BOOT);
    }
    if (startReg == cmd::REG_FIFO_STATUS1 && bus.fifoDataReads > 0u &&
        bus.fifoConcurrentArrivalWords > 0u && !bus.fifoArrivalInjected) {
      bus.fifoUnreadWords = static_cast<uint16_t>(
          bus.fifoUnreadWords + bus.fifoConcurrentArrivalWords);
      bus.fifoArrivalInjected = true;
    }
    if (startReg == cmd::REG_FIFO_STATUS1) {
      bus.syncFifoRegisters();
    }
    if (startReg == cmd::REG_DATA_START_ACCEL &&
        bus.alternateAccelInt16Extremes) {
      const int16_t value = (bus.accelDataReads++ & 1u) == 0u
                                ? std::numeric_limits<int16_t>::min()
                                : std::numeric_limits<int16_t>::max();
      bus.setI16(cmd::REG_OUTX_L_XL, value);
    }
    for (size_t i = 0; i < rxLength; ++i) {
      rxData[i] = bus.regs[static_cast<uint8_t>(startReg + i)];
    }
    if (countdownReset && --bus.resetBitReadsRemaining == 0u &&
        !bus.holdResetBit) {
      bus.regs[cmd::REG_CTRL3_C] = static_cast<uint8_t>(
          bus.regs[cmd::REG_CTRL3_C] & ~cmd::MASK_SW_RESET);
    }
    if (countdownBoot && --bus.bootBitReadsRemaining == 0u &&
        !bus.holdBootBit) {
      bus.regs[cmd::REG_CTRL3_C] =
          static_cast<uint8_t>(bus.regs[cmd::REG_CTRL3_C] & ~cmd::MASK_BOOT);
    }
    if (bus.corruptReadRemaining > 0u && bus.corruptReadRegister >= startReg &&
        static_cast<size_t>(bus.corruptReadRegister - startReg) < rxLength) {
      rxData[bus.corruptReadRegister - startReg] = bus.corruptReadValue;
      bus.corruptReadRemaining--;
    }
  }

  if (startReg == cmd::REG_FIFO_DATA_OUT_L && rxLength >= 2u &&
      bus.fifoUnreadWords > 0u) {
    bus.fifoUnreadWords--;
    bus.fifoDataReads++;
    bus.fifoPattern = static_cast<uint16_t>((bus.fifoPattern + 1u) & 0x03FFu);
    bus.fifoNextWord++;
    bus.syncFifoRegisters();
  }

  const Status status = Status::Ok();
  FakeBus::finishTransfer(transfer, status);
  return status;
}

inline DriverConfig makeDriverConfig(FakeBus& bus) {
  DriverConfig config;
  config.i2cWrite = fakeWrite;
  config.i2cWriteRead = fakeWriteRead;
  config.i2cUser = &bus;
  config.address = SensorAddress::SA0_GND;
  config.i2cTimeoutMs = 10;
  return config;
}

}  // namespace LSM6DS3TRTest
