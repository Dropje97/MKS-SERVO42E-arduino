\
#pragma once
#include <stdint.h>
#include <stddef.h>

namespace MKS {
  // CRC used in the manual examples:
  // CRC = (sum(bytes_without_crc) + 1) & 0xFF
  inline uint8_t crc8_sum_plus1(const uint8_t *buf, size_t len) {
    uint16_t s = 0;
    for (size_t i = 0; i < len; i++) s += buf[i];
    return (uint8_t)((s + 1) & 0xFF);
  }
}
