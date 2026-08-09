// The host variant.
//
// Every MeshCore board provides a target.h/target.cpp pair declaring the same
// handful of globals; this is that pair, implemented for a machine instead of a
// board. Nothing here is role-specific, and that is the whole point: a MeshCore
// application is just an application, and what a node *is* — repeater,
// companion, room server, sensor, or whatever ships next — is decided by which
// application you build against this variant, not by anything the variant or
// the simulator knows.
#include "target.h"

#include <Arduino.h>

#include <Wire.h>

#include "HostRNG.h"
#include "InternalFileSystem.h"

HostBoard board;
HostRadio radio_driver;
HostRTCClock rtc_clock;
HostSensorManager sensors;

// The platform singletons MeshCore's nRF52 build expects to find already
// defined by its board support.
HostSerial Serial;
Adafruit_LittleFS InternalFS;
TwoWire Wire;

// A real variant also defines `radio`, the RadioLib driver instance, and wraps
// it. There is no SX1262 here — the driver *is* the wrapper, because the thing
// on the other side is a socket to the simulator's channel rather than an SPI
// bus.

// The simulator's clock. Read by millis(); written by the bridge on every tick.
uint32_t g_sim_millis = 0;

// LoRa parameters, set by the bridge before the application starts so that the
// airtime calculation and the packet score see the same numbers the channel is
// modulating with.
int g_sf = 10;
float g_bwKHz = 250.0f;
int g_cr = 1;

// Seeded from the simulator rather than from radio noise. A run has to be
// reproducible from its seed, and a hardware RNG is the one thing that cannot
// be — so the seam that would ordinarily be RadioNoiseListener is the seam
// where determinism gets injected.
uint32_t g_identity_seed = 4417;

bool radio_init() { return true; }

mesh::LocalIdentity radio_new_identity() {
  HostRNG rng(g_identity_seed);
  return mesh::LocalIdentity(&rng);
}
