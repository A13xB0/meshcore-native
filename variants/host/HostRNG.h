// Seeded randomness for identity generation.
//
// On a board this is RadioNoiseListener, sampling the RSSI of an idle channel.
// That is the correct source on hardware and the wrong one here: the same
// scenario replayed with the same seed has to produce the same public keys, or
// no two runs can be compared and the event logs cannot be diffed.
#pragma once

#include <Mesh.h>

#include <cstddef>
#include <cstdint>

class HostRNG : public mesh::RNG {
 public:
  explicit HostRNG(uint64_t seed = 4417) : state_(seed ? seed : 4417) {}

  void random(uint8_t* dest, size_t sz) override {
    for (size_t i = 0; i < sz; i++) {
      state_ = state_ * 6364136223846793005ULL + 1442695040888963407ULL;
      dest[i] = (uint8_t)(state_ >> 33);
    }
  }

 private:
  uint64_t state_;
};
