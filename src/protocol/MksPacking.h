#pragma once
#include <stdint.h>

namespace MKS {
  inline void put_u16_be(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)((v >> 8) & 0xFF);
    p[1] = (uint8_t)(v & 0xFF);
  }

  inline void put_u16_le(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
  }

  inline uint16_t get_u16_be(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] << 8) | (uint16_t)p[1];
  }

  inline uint16_t get_u16_le(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
  }

  inline void put_u24_be(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)((v >> 16) & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)(v & 0xFF);
  }

  inline void put_i24_be(uint8_t *p, int32_t v) {
    if (v > 8388607) {
      v = 8388607;
    }
    if (v < -8388608) {
      v = -8388608;
    }
    uint32_t u = (uint32_t)(v & 0x00FFFFFF);
    p[0] = (uint8_t)((u >> 16) & 0xFF);
    p[1] = (uint8_t)((u >> 8) & 0xFF);
    p[2] = (uint8_t)(u & 0xFF);
  }

  inline void put_u32_le(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
  }

  inline void put_u32_be(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)((v >> 24) & 0xFF);
    p[1] = (uint8_t)((v >> 16) & 0xFF);
    p[2] = (uint8_t)((v >> 8) & 0xFF);
    p[3] = (uint8_t)(v & 0xFF);
  }

  inline uint32_t get_u32_be(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
  }

  inline uint32_t get_u32_le(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
  }

  inline int64_t get_i48_be(const uint8_t *p) {
    uint64_t u = ((uint64_t)p[0] << 40) | ((uint64_t)p[1] << 32) | ((uint64_t)p[2] << 24) | ((uint64_t)p[3] << 16) | ((uint64_t)p[4] << 8) | (uint64_t)p[5];
    if (u & 0x0000800000000000ULL) {
      u |= 0xFFFF000000000000ULL;
    }
    return (int64_t)u;
  }

  // int24 in 3 bytes little-endian (sign-extended)
  inline void put_i24_le(uint8_t *p, int32_t v) {
    // Clamp to [-8388608, 8388607]
    if (v > 8388607) {
      v = 8388607;
    }
    if (v < -8388608) {
      v = -8388608;
    }
    uint32_t u = (uint32_t)(v & 0x00FFFFFF);
    p[0] = (uint8_t)(u & 0xFF);
    p[1] = (uint8_t)((u >> 8) & 0xFF);
    p[2] = (uint8_t)((u >> 16) & 0xFF);
  }

  inline int32_t get_i24_le(const uint8_t *p) {
    uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
    // sign extend
    if (u & 0x00800000) {
      u |= 0xFF000000;
    }
    return (int32_t)u;
  }
}
