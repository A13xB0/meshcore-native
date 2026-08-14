// The Arduino platform, for a host.
//
// Ours rather than MeshCore's test mock, because the mock brings its own
// minimal Stream and two definitions of Print cannot coexist. Owning both
// headers keeps the contract in one place.
//
// millis() is the simulator's clock, not the wall clock. Every timing decision
// the firmware makes — CSMA, retransmit delay, duty-cycle refill — reads from
// here, so a node that consulted real time would run at real speed and could
// not be stepped, replayed or made deterministic.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "Stream.h"

using std::isnan;

// Set by the bridge on every tick.
extern uint32_t g_sim_millis;

inline uint32_t millis() { return g_sim_millis; }
inline uint32_t micros() { return g_sim_millis * 1000; }

// A firmware that busy-waits would hang the simulation: simulated time only
// advances when the bridge says so, so a delay that spun until millis() moved
// would spin forever. Nothing in the repeater's main path calls it.
inline void delay(uint32_t) {}
inline void delayMicroseconds(uint32_t) {}
inline void yield() {}

inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return 0; }
inline int analogRead(int) { return 0; }

// Constants rather than macros, for the same reason min and max are templates
// in HostArduino.h: a macro takes the name everywhere, including inside other
// people's headers. Windows has a struct called INPUT, and `#define INPUT 0`
// turned its declaration into `} 0,*P0,*LP0;` - which the compiler reports as
// a missing semicolon in winuser.h, a file nobody here has read.
//
// MeshCore uses these as plain integers, so a constant serves it identically.
constexpr int HIGH = 1;
constexpr int LOW = 0;
constexpr int INPUT = 0;
constexpr int OUTPUT = 1;
constexpr int INPUT_PULLUP = 2;

// SPI, which nothing here uses.
//
// MeshCore's std_init takes an SPIClass* so a board can hand it the bus its
// radio is wired to. There is no bus: RadioLib reaches the virtual chip through
// SimHal, and this exists only so the signature resolves. Passing one in would
// be harmless and pointless, which is why the variant passes nullptr.
class SPIClass {
 public:
  void begin() {}
  void begin(int, int, int) {}
  void end() {}
  void setPins(int, int, int) {}
};
extern SPIClass SPI;
