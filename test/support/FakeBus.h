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
  bool effectApplied = false;
};

struct TransferFault {
  uint32_t call = 0;
  Status status = Status::Error(Err::I2C_ERROR, "injected transfer failure");
  bool applyEffect = false;
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
  // Separate physical bank storage: main-bank aliases are never reused for
  // embedded parameters. Tests can independently poison the policy bytes.
  uint8_t bankA[256] = {};
  uint32_t timestampTicks = 0;
  uint16_t stepCount = 0;
  uint32_t timestampResetWrites = 0;
  uint32_t stepResetAssertions = 0;
  uint32_t bankWritesWhileSensorsActive = 0;
  uint32_t fifoFourthTriggers = 0;
  uint32_t notReadyStatusReads = 0;
  uint32_t selfTestNotReadyStatusReads = 0;
  uint32_t notReadyPeriod = 0;
  uint32_t statusReadIndex = 0;
  uint8_t corruptReadRegister = 0;
  uint8_t corruptReadValue = 0;
  uint32_t corruptReadRemaining = 0;
  bool corruptReadBankOnly = false;
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
  // Independent destructive FIFO storage. Tests specify the wire pattern
  // modulus explicitly; the fake never calls the production schema helper.
  uint16_t fifoWords[2048] = {};
  uint16_t fifoHead = 0;
  uint16_t fifoPatternWords = 1024;
  uint16_t fifoEmptyReadAttempts = 0;
  uint16_t fifoStatusReads = 0;
  uint16_t fifoPatternJumpAfterRead = 0;
  uint16_t fifoPatternJump = 0;
  uint16_t fifoOverflowOnDataRead = 0;
  uint16_t fifoOverflowArrivalWords = 0;
  bool fifoClearOverrunOnDataRead = false;
  bool fifoStoppedOnFull = false;
  bool fifoFullSmart = false;
  bool fifoOverrun = false;
  bool forceFifoEmptyClear = false;
  bool forceFifoEmptySet = false;
  bool forceFifoEmptySetAfterDataRead = false;
  uint16_t fifoConcurrentArrivalWords = 0;
  uint16_t fifoDataReads = 0;
  bool fifoArrivalInjected = false;

  FakeBus() { resetRegisters(); }

  void resetRegisters() {
    std::memset(regs, 0, sizeof(regs));
    std::memset(bankA, 0, sizeof(bankA));
    bankA[0x0F] = 0x10;
    bankA[0x14] = 0x6E;
    timestampTicks = 0;
    stepCount = 0;
    fifoFourthTriggers = 0;
    fifoUnreadWords = 0;
    fifoHead = 0;
    fifoPattern = 0;
    fifoOverrun = false;
    fifoStoppedOnFull = false;
    regs[cmd::REG_WHO_AM_I] = cmd::WHO_AM_I_VALUE;
    regs[cmd::REG_STATUS_REG] =
        static_cast<uint8_t>(cmd::MASK_XLDA | cmd::MASK_GDA | cmd::MASK_TDA);
    setRawSample(6400, 1000, -500, 0, 0, 0, 16384);
    syncFifoRegisters();
  }

  void setCounters(uint32_t ticks, uint16_t steps) {
    timestampTicks = ticks & 0xFFFFFFUL;
    stepCount = steps;
    regs[cmd::REG_TIMESTAMP0] = static_cast<uint8_t>(timestampTicks);
    regs[cmd::REG_TIMESTAMP1] = static_cast<uint8_t>(timestampTicks >> 8U);
    regs[cmd::REG_TIMESTAMP2] = static_cast<uint8_t>(timestampTicks >> 16U);
    regs[0x4B] = static_cast<uint8_t>(stepCount);
    regs[0x4C] = static_cast<uint8_t>(stepCount >> 8U);
  }

  // Explicit physical inputs, independent of the production decoder. Disabling
  // either engine preserves its counter; an asserted pedometer reset holds zero.
  void advanceCounters(uint32_t ticks, uint16_t steps) {
    if ((regs[cmd::REG_CTRL10_C] & 0x20U) != 0U &&
        ((regs[cmd::REG_CTRL1_XL] & 0xF0U) != 0U ||
         (regs[cmd::REG_CTRL2_G] & 0xF0U) != 0U))
      timestampTicks = (timestampTicks + ticks) & 0xFFFFFFUL;
    if ((regs[cmd::REG_CTRL10_C] & 0x16U) == 0x14U &&
        (regs[cmd::REG_CTRL1_XL] & 0xF0U) >= 0x20U)
      stepCount = static_cast<uint16_t>(stepCount + steps);
    setCounters(timestampTicks, stepCount);
  }

  // AN5130 Rev1 Tables73/74: literal wire construction from counter/event
  // inputs. This supports fourth-only fake captures, not a second schema owner.
  void captureFourthDataset(bool stepEvent, int16_t temperature = 0) {
    if ((regs[cmd::REG_FIFO_CTRL5] & 7U) == 0U) return;
    const bool wantsStep = (regs[cmd::REG_FIFO_CTRL2] & 0x40U) != 0U;
    if (stepEvent != wantsStep) return;
    const uint8_t decimation = static_cast<uint8_t>((regs[cmd::REG_FIFO_CTRL4] >> 3U) & 7U);
    static constexpr uint8_t DIVISORS[] = {0, 1, 2, 3, 4, 8, 16, 32};
    if (decimation == 0U || fifoFourthTriggers++ % DIVISORS[decimation] != 0U) return;
    if ((regs[cmd::REG_FIFO_CTRL2] & 0x80U) != 0U) {
      appendFifoWord(static_cast<uint16_t>(((timestampTicks >> 8U) & 0xFFU) |
                                         ((timestampTicks >> 8U) & 0xFF00U)));
      appendFifoWord(static_cast<uint16_t>(0x00A5U | ((timestampTicks & 0xFFU) << 8U)));
      appendFifoWord(stepCount);
    } else if ((regs[cmd::REG_FIFO_CTRL2] & 0x08U) != 0U) {
      appendFifoWord(0xBBAAU);
      appendFifoWord(static_cast<uint16_t>(temperature));
      appendFifoWord(0xDDCCU);
    }
  }

  void clearTrace() {
    transferCalls = 0;
    readCalls = 0;
    writeCalls = 0;
    traceCount = 0;
    for (Transfer& transfer : trace) transfer = {};
    faultCount = 0;
    for (TransferFault& fault : faults) fault = {};
  }

  void addFault(uint32_t call, const Status& status, bool applyEffect = false) {
    if (faultCount >= MAX_FAULTS) {
      return;
    }
    faults[faultCount++] = TransferFault{call, status, applyEffect, false};
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
    transfer->effectApplied = effectApplied;
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
    fifoHead = 0;
    for (uint16_t i = 0; i < unreadWords && i < 2048U; ++i) {
      fifoWords[i] = static_cast<uint16_t>(firstWord + i);
    }
    fifoOverrun = overrun;
    fifoDataReads = 0;
    fifoStatusReads = 0;
    fifoArrivalInjected = false;
    fifoStoppedOnFull = unreadWords == 2048U &&
                        (regs[cmd::REG_FIFO_CTRL5] & 7U) == 1U;
    syncFifoRegisters();
  }

  void setFifoWords(const int16_t* words, uint16_t count, uint16_t pattern,
                    uint16_t patternWords) {
    setFifo(count, pattern, 0);
    fifoPatternWords = patternWords;
    for (uint16_t i = 0; i < count && i < 2048U; ++i) {
      fifoWords[i] = static_cast<uint16_t>(words[i]);
    }
    syncFifoRegisters();
  }

  void appendFifoWord(uint16_t value) {
    if (fifoStoppedOnFull) return;
    if (fifoUnreadWords == 2048U) {
      fifoOverrun = true;
      if ((regs[cmd::REG_FIFO_CTRL5] & 7U) == 1U) {
        fifoStoppedOnFull = true;
        return;
      }
      fifoHead = static_cast<uint16_t>((fifoHead + 1U) % 2048U);
      fifoPattern = static_cast<uint16_t>((fifoPattern + 1U) % fifoPatternWords);
      --fifoUnreadWords;
    }
    fifoWords[(fifoHead + fifoUnreadWords) % 2048U] = value;
    ++fifoUnreadWords;
    if (fifoUnreadWords == 2048U && (regs[cmd::REG_FIFO_CTRL5] & 7U) == 1U) {
      fifoStoppedOnFull = true;
    }
    syncFifoRegisters();
  }

  uint16_t consumeFifoWord() {
    if (fifoOverflowOnDataRead == fifoDataReads + 1U) {
      for (uint16_t i = 0; i < fifoOverflowArrivalWords; ++i) {
        appendFifoWord(static_cast<uint16_t>(0x7000U + i));
      }
    }
    if (fifoUnreadWords == 0U) {
      ++fifoEmptyReadAttempts;
      return 0;
    }
    const uint16_t value = fifoWords[fifoHead];
    fifoHead = static_cast<uint16_t>((fifoHead + 1U) % 2048U);
    --fifoUnreadWords;
    ++fifoDataReads;
    fifoPattern = static_cast<uint16_t>((fifoPattern + 1U) % fifoPatternWords);
    if (fifoPatternJumpAfterRead != 0U &&
        fifoDataReads == fifoPatternJumpAfterRead) {
      fifoPattern = static_cast<uint16_t>((fifoPattern + fifoPatternJump) %
                                          fifoPatternWords);
    }
    fifoNextWord = static_cast<uint16_t>(value + 1U);
    if (fifoClearOverrunOnDataRead) fifoOverrun = false;
    syncFifoRegisters();
    return value;
  }

  void syncFifoRegisters() {
    const uint16_t reportedUnread =
        fifoOverrun && fifoUnreadWords == 2048U ? 0U : fifoUnreadWords;
    regs[cmd::REG_FIFO_STATUS1] = static_cast<uint8_t>(reportedUnread & 0xFFu);
    regs[cmd::REG_FIFO_STATUS2] =
        static_cast<uint8_t>((reportedUnread >> 8) & 0x07u);
    if ((fifoUnreadWords == 0u && !forceFifoEmptyClear) || forceFifoEmptySet ||
        (forceFifoEmptySetAfterDataRead && fifoDataReads > 0u)) {
      regs[cmd::REG_FIFO_STATUS2] |= cmd::MASK_FIFO_EMPTY;
    }
    if (fifoOverrun) {
      regs[cmd::REG_FIFO_STATUS2] |= cmd::MASK_FIFO_OVER_RUN;
    }
    if (fifoFullSmart || fifoUnreadWords > 2045U) {
      regs[cmd::REG_FIFO_STATUS2] |= 0x20U;
    }
    const uint16_t threshold = static_cast<uint16_t>(regs[cmd::REG_FIFO_CTRL1]) |
        static_cast<uint16_t>((regs[cmd::REG_FIFO_CTRL2] & 7U) << 8U);
    if (fifoUnreadWords >= threshold) regs[cmd::REG_FIFO_STATUS2] |= 0x80U;
    regs[cmd::REG_FIFO_STATUS3] = static_cast<uint8_t>(fifoPattern & 0xFFu);
    regs[cmd::REG_FIFO_STATUS4] = static_cast<uint8_t>((fifoPattern >> 8) & 0x03u);
    setI16(cmd::REG_FIFO_DATA_OUT_L, static_cast<int16_t>(fifoNextWord));
  }

  void applyWrite(const uint8_t* data, size_t length) {
    const uint8_t startReg = data[0];
    for (size_t i = 1; i < length; ++i) {
      const uint8_t reg = static_cast<uint8_t>(startReg + i - 1u);
      const uint8_t value = data[i];
      if (reg != cmd::REG_FUNC_CFG_ACCESS &&
          (regs[cmd::REG_FUNC_CFG_ACCESS] & 0x80U) != 0U) {
        bankA[reg] = value;
        continue;
      }
      if (reg == cmd::REG_TIMESTAMP2 && value == 0xAAU) {
        ++timestampResetWrites;
        setCounters(0, stepCount);
        continue;
      }
      if (reg == cmd::REG_CTRL10_C && (value & 0x02U) != 0U) {
        ++stepResetAssertions;
        setCounters(timestampTicks, 0);
      }
      if (reg != cmd::REG_CTRL3_C) {
        regs[reg] = value;
        if (reg == cmd::REG_FIFO_CTRL5 && (value & 7U) == 0U) {
          fifoUnreadWords = 0;
          fifoHead = 0;
          fifoPattern = 0;
          fifoOverrun = false;
          fifoStoppedOnFull = false;
          syncFifoRegisters();
        }
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
      if (softwareReset) setCounters(0, 0);

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
  if (fault != nullptr && !fault->applyEffect) {
    FakeBus::finishTransfer(transfer, fault->status);
    return fault->status;
  }

  if ((bus.regs[cmd::REG_FUNC_CFG_ACCESS] & 0x80U) != 0U &&
      data[0] != cmd::REG_FUNC_CFG_ACCESS &&
      ((bus.regs[cmd::REG_CTRL1_XL] & 0xF0U) != 0U ||
       (bus.regs[cmd::REG_CTRL2_G] & 0xF0U) != 0U)) {
    ++bus.bankWritesWhileSensorsActive;
    const Status status = Status::Error(Err::I2C_ERROR, "bank policy changed with active sensor");
    FakeBus::finishTransfer(transfer, status);
    return status;
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
  if (fault != nullptr && !fault->applyEffect) {
    FakeBus::finishTransfer(transfer, fault->status);
    return fault->status;
  }

  if (startReg == cmd::REG_FIFO_DATA_OUT_L && rxLength >= 2U) {
    // AN5130 sections 4.7.1 and 8.4: FIFO output addresses round from
    // 0x3F back to 0x3E. Every received L/H pair destructively pops one word.
    uint8_t received[32] = {};
    if (rxLength > sizeof(received) || (rxLength & 1U) != 0U) {
      const Status status = Status::Error(Err::INVALID_PARAM, "invalid FIFO burst");
      FakeBus::finishTransfer(transfer, status);
      return status;
    }
    for (size_t i = 0; i < rxLength; i += 2U) {
      const uint16_t word = bus.consumeFifoWord();
      received[i] = static_cast<uint8_t>(word);
      received[i + 1U] = static_cast<uint8_t>(word >> 8U);
    }
    if (fault != nullptr) {
      // Simulate consumption without any trustworthy bytes reaching the host.
      FakeBus::finishTransfer(transfer, fault->status, true);
      return fault->status;
    }
    std::memcpy(rxData, received, rxLength);
    FakeBus::finishTransfer(transfer, Status::Ok(), true);
    return Status::Ok();
  }

  const bool selfTestActive =
      (bus.regs[cmd::REG_CTRL5_C] & (cmd::MASK_ST_XL | cmd::MASK_ST_G)) != 0u;
  if (startReg == cmd::REG_STATUS_REG && bus.notReadyPeriod > 1u &&
      bus.statusReadIndex++ % bus.notReadyPeriod != bus.notReadyPeriod - 1u) {
    std::memset(rxData, 0, rxLength);
  } else if (startReg == cmd::REG_STATUS_REG && bus.notReadyStatusReads > 0u) {
    bus.notReadyStatusReads--;
    std::memset(rxData, 0, rxLength);
  } else if (startReg == cmd::REG_STATUS_REG && selfTestActive &&
             bus.selfTestNotReadyStatusReads > 0u) {
    bus.selfTestNotReadyStatusReads--;
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
      for (uint16_t i = 0; i < bus.fifoConcurrentArrivalWords; ++i) {
        bus.appendFifoWord(static_cast<uint16_t>(bus.fifoNextWord + i));
      }
      bus.fifoArrivalInjected = true;
    }
    if (startReg == cmd::REG_FIFO_STATUS1) {
      ++bus.fifoStatusReads;
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
      const uint8_t reg = static_cast<uint8_t>(startReg + i);
      rxData[i] = reg != cmd::REG_FUNC_CFG_ACCESS &&
          (bus.regs[cmd::REG_FUNC_CFG_ACCESS] & 0x80U) != 0U
          ? bus.bankA[reg] : bus.regs[reg];
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
    if (bus.corruptReadRemaining > 0u &&
        (!bus.corruptReadBankOnly || (bus.regs[cmd::REG_FUNC_CFG_ACCESS] & 0x80U) != 0U) &&
        bus.corruptReadRegister >= startReg &&
        static_cast<size_t>(bus.corruptReadRegister - startReg) < rxLength) {
      rxData[bus.corruptReadRegister - startReg] = bus.corruptReadValue;
      bus.corruptReadRemaining--;
    }
  }

  if (fault != nullptr) {
    FakeBus::finishTransfer(transfer, fault->status, true);
    return fault->status;
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
