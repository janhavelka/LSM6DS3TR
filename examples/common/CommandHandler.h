/**
 * @file CommandHandler.h
 * @brief Simple serial command parser helpers for interactive examples.
 */

#pragma once

#include <Arduino.h>

#include "examples/common/Log.h"

namespace cmd {

inline bool readLine(char* buffer, size_t bufSize) {
  static char cmdBuf[128];
  static size_t cmdLen = 0;

  while (LOG_SERIAL.available()) {
    int c = LOG_SERIAL.read();
    if (c < 0) {
      break;
    }

    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        strncpy(buffer, cmdBuf, bufSize - 1);
        buffer[bufSize - 1] = '\0';
        cmdLen = 0;
        return true;
      }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = static_cast<char>(c);
    }
  }

  return false;
}

inline bool parseInt(const char* cmdLine, const char* keyword, int* outValue) {
  const size_t kwLen = strlen(keyword);
  if (strncmp(cmdLine, keyword, kwLen) != 0) {
    return false;
  }

  const char* valueStr = cmdLine + kwLen;
  while (*valueStr == ' ' || *valueStr == '\t') {
    valueStr++;
  }
  if (*valueStr == '\0') {
    return false;
  }

  *outValue = atoi(valueStr);
  return true;
}

inline bool match(const char* cmdLine, const char* keyword) {
  return strncasecmp(cmdLine, keyword, strlen(keyword)) == 0;
}

}  // namespace cmd
