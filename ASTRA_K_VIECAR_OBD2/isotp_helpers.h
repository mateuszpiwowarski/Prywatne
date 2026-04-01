#pragma once
#include <Arduino.h>

struct IsoTpBuf {
  static const uint16_t CAP = 160;
  uint16_t n = 0;
  uint8_t b[CAP];
};

static inline void isoClear(IsoTpBuf& p) { p.n = 0; }

static inline void isoAppend(IsoTpBuf& p, const uint8_t* src, int len) {
  while (len-- > 0 && p.n < IsoTpBuf::CAP) p.b[p.n++] = *src++;
}
