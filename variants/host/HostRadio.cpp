// Airtime and packet score: the two numbers the firmware computes about the
// air rather than reads from the radio.
//
// Out of line because both are transcriptions of a formula that lives somewhere
// else — RadioLib's getTimeOnAir() and MeshCore's packetScoreInt — and a
// transcription wants to be in one obvious place where it can be diffed against
// its source, not inlined in a header.
#include "HostRadio.h"

// RadioLib's getTimeOnAir(), truncated to milliseconds, which is exactly what
// MeshCore's RadioLibWrapper::getEstAirtimeFor() returns on real hardware.
//
// This has to be the firmware's own formula rather than a good approximation of
// it. CSMA backoff, the duty-cycle budget and the send timeout are all built on
// this number, so if the channel occupies the air for a different length of time
// than the firmware believes it did, the two desynchronise silently and every
// collision result after that is fiction.
uint32_t HostRadio::getEstAirtimeFor(int len) {
  float symbolMs = (float)((uint32_t)1 << g_sf) / g_bwKHz;
  float sfCoeff1 = 4.25f, sfCoeff2 = 8.0f;
  if (g_sf == 5 || g_sf == 6) {
    sfCoeff1 = 6.25f;
    sfCoeff2 = 0.0f;
  }
  // The chip enables low data rate optimisation itself once a symbol reaches
  // 16 ms — SF11 and SF12 at 125 kHz.
  int sfDivisor = (symbolMs >= 16.0f) ? 4 * (g_sf - 2) : 4 * g_sf;
  int bits = 8 * len + 16 /* CRC */ - 4 * g_sf + (int)sfCoeff2 + 20 /* explicit header */;
  if (bits < 0) bits = 0;
  int coded = (bits + sfDivisor - 1) / sfDivisor;
  // MeshCore's own preamble rule, from RadioLibWrappers.h.
  int preamble = g_sf <= 8 ? 32 : 16;
  float symbols = (float)preamble + sfCoeff1 + 8.0f + (float)(coded * (g_cr + 4));
  return (uint32_t)(symbolMs * symbols);
}

// Ranking for the delayed-flood decision.
//
// MeshCore's own, from RadioLibWrapper::packetScoreInt, and it has to be:
// Dispatcher::calcRxDelay computes (10^(0.85 - score) - 1) * airtime, which
// assumes a score in [0,1]. An early placeholder returned snr*100 - len, and a
// score of 675 makes that expression negative, so every node relayed the instant
// it decoded. Staggering by how well each node heard the packet is the whole of
// MeshCore's flood design, and a stub quietly removed it while the simulation
// still looked plausible.
float HostRadio::packetScore(float snr, int len) {
  if (g_sf < 7 || g_sf > 12) return 0.0f;
  // Semtech's per-SF demodulator floor, the same table MeshCore uses.
  static const float threshold[] = {-7.5f, -10.0f, -12.5f, -15.0f, -17.5f, -20.0f};
  const float floorDB = threshold[g_sf - 7];
  if (snr < floorDB) return 0.0f;  // no chance of success

  float v = ((snr - floorDB) / 10.0f) * (1.0f - (float)len / 256.0f);
  if (v < 0.0f) v = 0.0f;
  if (v > 1.0f) v = 1.0f;
  return v;
}
