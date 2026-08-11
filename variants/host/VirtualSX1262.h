#pragma once

// A virtual SX1262: what answers RadioLib's SPI transactions.
//
// Everything above this line is the real software stack - MeshCore's
// Dispatcher, its CustomSX1262 driver, and RadioLib itself. This is where that
// stack meets the simulator, and it is deliberately the only place that knows
// both.
//
// The interesting part is not the command table, it is the IRQ flags. MeshCore
// decides whether the channel is busy by reading PREAMBLE_DETECTED and
// HEADER_VALID and timing how long they have been set - that is what 1.17's
// listen-before-talk rewrite changed, and it cannot be exercised by a shim that
// never sets the bits. So the flags are driven from the engine's view of what
// is on the air at this node, at the simulated instant each becomes true.
//
// What is modelled, and what is not:
//   * modelled: the command set RadioLib issues for a LoRa link, the IRQ
//     register, the data buffer, RSSI/SNR from the engine, CAD.
//   * not modelled: BUSY (see SimHal), calibration timing, the analogue front
//     end. A virtual chip is a model of a chip; the stack above it is real, the
//     silicon is not.

#include <stdint.h>
#include <string.h>

#include <deque>
#include <vector>

class VirtualSX1262 {
 public:
  VirtualSX1262();

  // ---- the simulator's side, called by the bridge ----

  // Frames the firmware has handed to the radio, waiting to go on the air.
  bool hasPendingTx = false;
  std::vector<uint8_t> pendingTx;

  // Frames the engine has delivered to this node.
  std::deque<std::vector<uint8_t>> inbox;

  // The engine says our waveform has left the antenna. The node cannot know
  // this: how long a transmission occupied the channel is a property of the
  // samples the engine generated.
  void transmitFinished();

  // Is another station on the air here, loud enough to detect? The engine is
  // the only thing that can answer, and this is where that answer becomes
  // something a driver can read.
  void setChannelBusy(bool busy);

  // What the engine measured for the last frame it delivered.
  void setLastSignal(float rssiDbm, float snrDb) { rssi_ = rssiDbm; snr_ = snrDb; }

  // Advance internal timers to this simulated instant.
  void tick(uint64_t nowMs);

  bool irqAsserted() const { return (irq_ & irqMask_) != 0; }

  // ---- what the firmware's channel decisions look like from below ----
  //
  // MeshCore decides whether to defer by reading the IRQ register. Counting
  // those reads, and how long the busy flags were up, is the only way to tell
  // "the mesh is genuinely busy" from "our chip cries busy too readily" - and
  // the second is a fault in the simulator that would look exactly like a
  // finding about the firmware.
  uint32_t irqReads() const { return irqReads_; }
  uint32_t busyReads() const { return busyReads_; }
  uint32_t busyMs() const { return busyMs_; }
  uint32_t preambleRaises() const { return preambleRaises_; }
  uint32_t spuriousRaises() const { return spuriousRaises_; }

  // Latch a flag once raised, as a misbehaving chip does.
  //
  // This is the fault MeshCore 1.17 exists to survive: a preamble or header
  // flag that sets and never clears, so a driver that trusts it believes the
  // channel is busy for ever and stops transmitting. 1.16 trusts it; 1.17 times
  // it out. Without a way to reproduce the fault, the difference between them
  // cannot be observed at all - which is exactly what twelve runs showed.
  void setStuckIrqMs(uint32_t ms) { stuckIrqMs_ = ms; }

  // Airtime, from the parameters the firmware actually programmed.
  uint32_t estAirtimeMs(int lenBytes) const;

  int spreadingFactor() const { return sf_; }
  float bandwidthKHz() const { return bwKHz_; }
  int codingRate() const { return cr_; }

  // ---- RadioLib's side ----

  void spiTransfer(const uint8_t* out, size_t len, uint8_t* in);

 private:
  void runCommand(const uint8_t* out, size_t len, uint8_t* in);
  void applyModulation(const uint8_t* p);
  void applyPacketParams(const uint8_t* p);
  void startRx();
  void startTx();
  void startCad();
  void deliverPending();

  // Chip state.
  uint8_t buffer_[256] = {0};
  uint8_t rxLen_ = 0;
  uint16_t irq_ = 0;      // IRQ status register
  uint16_t irqMask_ = 0;  // what is allowed to raise DIO1
  uint8_t mode_ = 0;      // 0 standby, 1 rx, 2 tx, 3 cad
  uint8_t txBase_ = 0, rxBase_ = 0;
  // How many bytes the firmware said the next transmission is, from
  // SetPacketParams. The buffer is 256 bytes and only this many are on the air.
  uint8_t txLenForSend_ = 0;
  uint8_t regs_[0x1000] = {0};

  // Modem parameters, as programmed by the firmware.
  int sf_ = 10;
  float bwKHz_ = 250;
  int cr_ = 5;
  uint32_t preambleSyms_ = 16;
  uint32_t freqHz_ = 869525000;

  // What the air is doing here, from the engine.
  bool channelBusy_ = false;
  uint64_t busySinceMs_ = 0;
  bool preambleRaised_ = false;
  bool headerRaised_ = false;

  float rssi_ = -100, snr_ = 0;
  uint64_t nowMs_ = 0;

  // Instrumentation.
  uint32_t irqReads_ = 0;
  uint32_t busyReads_ = 0;
  uint32_t busyMs_ = 0;
  uint32_t preambleRaises_ = 0;
  uint64_t lastBusyTickMs_ = 0;

  // Fault injection: how long a raised flag refuses to clear. 0 is a chip that
  // behaves.
  uint32_t stuckIrqMs_ = 0;
  uint64_t nextSpuriousMs_ = 0;
  uint32_t spuriousRaises_ = 0;
};
