// The radio the host repeater build talks to.
//
// It is the same seam as the native node's BridgeRadio — transmissions go to
// the simulator over a socket and receptions come back — plus the handful of
// extra methods the repeater application calls on its driver. Those come from
// RadioLibWrapper on a real board, and the repeater uses them for statistics,
// transmit power and the RNG seed rather than for anything the physics depends
// on.
#pragma once

#include <Mesh.h>
#include <stdint.h>
#include <string.h>

#include <deque>
#include <vector>

// Set by the bridge before the firmware starts, so the same LoRa parameters
// reach the airtime calculation and the packet score.
extern int g_sf;
extern float g_bwKHz;
extern int g_cr;

class HostRadio : public mesh::Radio {
 public:
  // What the bridge fills and drains.
  std::deque<std::vector<uint8_t>> inbox;
  std::vector<uint8_t> pendingTx;
  bool hasPendingTx = false;

  // ---- mesh::Radio ----

  bool startSendRaw(const uint8_t* bytes, int len) override {
    pendingTx.assign(bytes, bytes + len);
    hasPendingTx = true;
    onAir_ = true;
    sending_ = true;
    packetsSent_++;
    return true;
  }

  // The engine says when the waveform ended. The node cannot know: how long a
  // transmission occupied the channel is a property of the samples the engine
  // generated.
  bool isSendComplete() override { return !onAir_; }
  void onSendFinished() override { sending_ = false; }
  void transmitFinished() { onAir_ = false; }

  int recvRaw(uint8_t* dest, int max) override {
    if (inbox.empty()) return 0;
    auto& f = inbox.front();
    int n = (int)f.size() < max ? (int)f.size() : max;
    memcpy(dest, f.data(), (size_t)n);
    inbox.pop_front();
    packetsRecv_++;
    return n;
  }

  uint32_t getEstAirtimeFor(int len) override;
  float packetScore(float snr, int len) override;

  int getNoiseFloor() const override { return noiseFloor_; }
  bool isInRecvMode() const override { return !sending_; }

  // Is another station on the air right now, loud enough for this node to hear?
  //
  // This is what MeshCore's listen-before-talk asks before transmitting
  // (Dispatcher::checkSend). The base class answers false, and left
  // unimplemented that is what every node believed: the channel is permanently
  // clear, nobody ever defers, and the entire CAD path is unreachable. Two
  // firmware versions differing only in their listen-before-talk code then
  // produce bit-identical timing - which is not the firmware behaving
  // identically, but the simulator declining to ask the question.
  //
  // Only the engine can answer it: it knows which waveforms overlap this
  // node's antenna and how strong each one arrives. It sends that in; this
  // reports it.
  bool isReceiving() override { return channelBusy_; }

  // Set from the bridge once per tick, before the node is advanced.
  void setChannelBusy(bool busy) { channelBusy_ = busy; }

  // ---- what the repeater application calls on its driver ----

  void setParams(float freq, float bw, uint8_t sf, uint8_t cr) {
    freqMHz_ = freq;
    g_bwKHz = bw;
    g_sf = sf;
    g_cr = cr;
  }
  void setTxPower(uint8_t dbm) { txPower_ = dbm; }

  float getLastRSSI() const { return lastRSSI_; }
  float getLastSNR() const { return lastSNR_; }
  void setLastSignal(float rssi, float snr) { lastRSSI_ = rssi; lastSNR_ = snr; }

  uint32_t getPacketsRecv() const { return packetsRecv_; }
  uint32_t getPacketsSent() const { return packetsSent_; }
  uint32_t getPacketsRecvErrors() const { return 0; }
  void resetStats() { packetsRecv_ = packetsSent_ = 0; }

  // The seed comes from the simulator rather than from radio noise, because a
  // run has to be reproducible from its seed and a hardware RNG is the one
  // thing that cannot be.
  uint32_t getRngSeed() const { return seed_; }
  void setRngSeed(uint32_t s) { seed_ = s; }

  bool getRxBoostedGainMode() const { return boosted_; }
  bool setRxBoostedGainMode(bool en) { boosted_ = en; return true; }

  void configSideDetectors(int, int) {}

  // What a receiver reads off the chip between packets. The engine owns the
  // real figure; this is the last one it reported.
  float getCurrentRSSI() { return lastRSSI_; }
  float getCurrentSNR() { return lastSNR_; }
  void idle() {}
  void standby() {}
  void triggerNoiseFloorCalibrate(int) {}
  void resetAGC() {}
  uint32_t getEstAirtimeFor(uint8_t, int len) { return getEstAirtimeFor(len); }

  void begin() override {}
  void loop() override {}

 private:
  bool sending_ = false;
  bool channelBusy_ = false;
  bool onAir_ = false;
  bool boosted_ = false;
  int noiseFloor_ = -114;
  float freqMHz_ = 869.525f;
  uint8_t txPower_ = 22;
  float lastRSSI_ = -100, lastSNR_ = 0;
  uint32_t packetsRecv_ = 0, packetsSent_ = 0;
  uint32_t seed_ = 4417;
};
