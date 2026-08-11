#include "VirtualSX1262.h"

#include <math.h>

// Opcodes, from the SX1262 datasheet. Only the ones RadioLib issues for a LoRa
// link are handled; anything else is acknowledged and ignored, which is the
// right default - an unhandled command should cost detail, not wedge a driver.
namespace {
constexpr uint8_t kSetSleep = 0x84;
constexpr uint8_t kSetStandby = 0x80;
constexpr uint8_t kSetTx = 0x83;
constexpr uint8_t kSetRx = 0x82;
constexpr uint8_t kSetCad = 0xC5;
constexpr uint8_t kSetRfFrequency = 0x86;
constexpr uint8_t kSetPacketType = 0x8A;
constexpr uint8_t kGetPacketType = 0x11;
constexpr uint8_t kSetTxParams = 0x8E;
constexpr uint8_t kSetModulationParams = 0x8B;
constexpr uint8_t kSetPacketParams = 0x8C;
constexpr uint8_t kSetCadParams = 0x88;
constexpr uint8_t kSetBufferBase = 0x8F;
constexpr uint8_t kWriteBuffer = 0x0E;
constexpr uint8_t kReadBuffer = 0x1E;
constexpr uint8_t kWriteRegister = 0x0D;
constexpr uint8_t kReadRegister = 0x1D;
constexpr uint8_t kSetDioIrqParams = 0x08;
constexpr uint8_t kGetIrqStatus = 0x12;
constexpr uint8_t kClearIrqStatus = 0x02;
constexpr uint8_t kGetRxBufferStatus = 0x13;
constexpr uint8_t kGetPacketStatus = 0x14;
constexpr uint8_t kGetStatus = 0xC0;
constexpr uint8_t kGetDeviceErrors = 0x17;
constexpr uint8_t kClearDeviceErrors = 0x07;
constexpr uint8_t kGetRssiInst = 0x15;

// IRQ bits.
constexpr uint16_t kIrqTxDone = 1 << 0;
constexpr uint16_t kIrqRxDone = 1 << 1;
constexpr uint16_t kIrqPreambleDetected = 1 << 2;
constexpr uint16_t kIrqSyncWordValid = 1 << 3;
constexpr uint16_t kIrqHeaderValid = 1 << 4;
constexpr uint16_t kIrqHeaderErr = 1 << 5;
constexpr uint16_t kIrqCrcErr = 1 << 6;
constexpr uint16_t kIrqCadDone = 1 << 7;
constexpr uint16_t kIrqCadDetected = 1 << 8;
constexpr uint16_t kIrqTimeout = 1 << 9;

// How far into a transmission a receiver locks onto the preamble, and how much
// later the header is demodulated. Both are in symbols and become milliseconds
// through the current modem settings, because that is what makes them behave
// like a radio rather than like a constant: at SF12 a preamble takes an age and
// at SF7 it is gone in a blink, and MeshCore's listen-before-talk times exactly
// that.
constexpr double kPreambleSymbols = 4.0;
constexpr double kHeaderSymbols = 12.0;
}  // namespace

VirtualSX1262::VirtualSX1262() {
  // The chip has to introduce itself. RadioLib reads sixteen bytes at
  // REG_VERSION_STRING and compares the first six against the chip type; a chip
  // that answers zeroes is retried ten times and then reported as not present,
  // which is exactly how this first failed to boot.
  //
  // The expected string for an SX1262 is "SX1261". That is not a mistake here
  // or in RadioLib: the part genuinely reports its family that way, and a
  // virtual chip that answered "SX1262" would be the one telling the lie.
  static const char kVersion[] = "SX1261";
  memcpy(&regs_[0x0320], kVersion, 6);

  // Receiver gain, which MeshCore reads back to report whether boosted mode is
  // on. Power-on default is the non-boosted value.
  regs_[0x08AC] = 0x94;
}

void VirtualSX1262::tick(uint64_t nowMs) {
  nowMs_ = nowMs;
  if (mode_ == 1) deliverPending();

  // Preamble and header detection, in receive mode only. A node cannot hear
  // anything while its own transmitter is keyed - that is half duplex, and the
  // engine already reports the channel as clear to a node that is transmitting.
  if (mode_ != 1) return;

  const double symbolMs = (double)(1u << sf_) / (bwKHz_ > 0 ? bwKHz_ : 250.0);
  if (channelBusy_) {
    const double sinceMs = (double)(nowMs_ - busySinceMs_);
    if (!preambleRaised_ && sinceMs >= kPreambleSymbols * symbolMs) {
      irq_ |= kIrqPreambleDetected;
      preambleRaised_ = true;
    }
    if (!headerRaised_ && sinceMs >= kHeaderSymbols * symbolMs) {
      irq_ |= kIrqHeaderValid | kIrqSyncWordValid;
      headerRaised_ = true;
    }
  }
}

void VirtualSX1262::setChannelBusy(bool busy) {
  if (busy && !channelBusy_) busySinceMs_ = nowMs_;
  if (!busy) {
    // The air went quiet. The flags stay set until the firmware clears them,
    // which is deliberate: a real SX1262 latches them, and coping with a flag
    // that outlives the signal is precisely what MeshCore's driver does.
    preambleRaised_ = false;
    headerRaised_ = false;
  }
  channelBusy_ = busy;
}

void VirtualSX1262::transmitFinished() {
  if (mode_ == 2) {
    irq_ |= kIrqTxDone;
    mode_ = 0;
  }
}

void VirtualSX1262::deliverPending() {
  if (inbox.empty()) return;
  auto& f = inbox.front();
  rxLen_ = (uint8_t)(f.size() > 255 ? 255 : f.size());
  memcpy(&buffer_[rxBase_], f.data(), rxLen_);
  inbox.pop_front();
  irq_ |= kIrqRxDone;
  // A delivered frame implies its preamble and header were seen, whatever the
  // detector had got to. Without this a short packet can arrive before the
  // detection thresholds elapse, and the driver sees a payload it never saw
  // start.
  irq_ |= kIrqPreambleDetected | kIrqHeaderValid | kIrqSyncWordValid;
}

void VirtualSX1262::startRx() { mode_ = 1; }

void VirtualSX1262::startTx() {
  mode_ = 2;
  pendingTx.assign(&buffer_[txBase_], &buffer_[txBase_] + txLenForSend_);
  hasPendingTx = true;
}

void VirtualSX1262::startCad() {
  mode_ = 3;
  // CAD answers in one go: the chip listens for a couple of symbols and reports.
  irq_ |= kIrqCadDone;
  if (channelBusy_) irq_ |= kIrqCadDetected;
  mode_ = 0;
}

uint32_t VirtualSX1262::estAirtimeMs(int lenBytes) const {
  // Semtech's own airtime formula, from the parameters the firmware programmed.
  const double bwHz = bwKHz_ * 1000.0;
  const double tSym = (double)(1u << sf_) / bwHz;
  const int de = (sf_ >= 11) ? 1 : 0;
  const int crc = 1, header = 0, crDen = cr_;
  double num = 8.0 * lenBytes - 4.0 * sf_ + 28 + 16 * crc - 20 * header;
  double den = 4.0 * (sf_ - 2 * de);
  double payloadSyms = 8 + fmax(ceil(num / den) * crDen, 0.0);
  double t = (preambleSyms_ + 4.25 + payloadSyms) * tSym;
  return (uint32_t)(t * 1000.0 + 0.5);
}

void VirtualSX1262::applyModulation(const uint8_t* p) {
  sf_ = p[0];
  // Bandwidth is an index in the datasheet's table; only the values MeshCore
  // uses are mapped, and anything else keeps the current setting rather than
  // silently becoming zero and making airtime infinite.
  switch (p[1]) {
    case 0x00: bwKHz_ = 7.81f; break;
    case 0x08: bwKHz_ = 10.42f; break;
    case 0x01: bwKHz_ = 15.63f; break;
    case 0x09: bwKHz_ = 20.83f; break;
    case 0x02: bwKHz_ = 31.25f; break;
    case 0x0A: bwKHz_ = 41.67f; break;
    case 0x03: bwKHz_ = 62.5f; break;
    case 0x04: bwKHz_ = 125.0f; break;
    case 0x05: bwKHz_ = 250.0f; break;
    case 0x06: bwKHz_ = 500.0f; break;
    default: break;
  }
  cr_ = p[2] ? (4 + p[2]) : cr_;
}

void VirtualSX1262::applyPacketParams(const uint8_t* p) {
  preambleSyms_ = ((uint32_t)p[0] << 8) | p[1];
  txLenForSend_ = p[3];
}

void VirtualSX1262::spiTransfer(const uint8_t* out, size_t len, uint8_t* in) {
  if (len == 0) return;
  memset(in, 0, len);
  runCommand(out, len, in);
}

void VirtualSX1262::runCommand(const uint8_t* out, size_t len, uint8_t* in) {
  const uint8_t op = out[0];
  // Byte 1 of every reply is the status byte. RadioLib reads data from
  // buffIn[cmdLen + 1], so a one-byte command puts its first data byte at
  // in[2] - which is where the datasheet puts it too.
  auto status = [&](uint8_t v) { if (len > 1) in[1] = v; };
  status(0x22);  // standby, command completed

  switch (op) {
    case kSetStandby: mode_ = 0; break;
    case kSetSleep: mode_ = 0; break;
    case kSetRx: startRx(); break;
    case kSetTx: startTx(); break;
    case kSetCad: startCad(); break;

    case kSetRfFrequency:
      if (len >= 5) {
        uint32_t raw = ((uint32_t)out[1] << 24) | ((uint32_t)out[2] << 16) |
                       ((uint32_t)out[3] << 8) | out[4];
        // The datasheet's PLL step: freq = raw * 32e6 / 2^25.
        freqHz_ = (uint32_t)((double)raw * 32000000.0 / 33554432.0);
      }
      break;

    case kSetModulationParams: if (len >= 4) applyModulation(&out[1]); break;
    case kSetPacketParams:     if (len >= 7) applyPacketParams(&out[1]); break;
    case kSetBufferBase:       if (len >= 3) { txBase_ = out[1]; rxBase_ = out[2]; } break;

    case kWriteBuffer:
      if (len >= 2) {
        uint8_t offset = out[1];
        for (size_t i = 2; i < len && (size_t)offset + (i - 2) < sizeof(buffer_); i++) {
          buffer_[offset + (i - 2)] = out[i];
        }
      }
      break;

    case kReadBuffer:
      if (len >= 3) {
        uint8_t offset = out[1];
        for (size_t i = 3; i < len; i++) {
          size_t idx = (size_t)offset + (i - 3);
          in[i] = idx < sizeof(buffer_) ? buffer_[idx] : 0;
        }
      }
      break;

    case kWriteRegister:
      if (len >= 3) {
        uint16_t addr = ((uint16_t)out[1] << 8) | out[2];
        for (size_t i = 3; i < len && (size_t)addr + (i - 3) < sizeof(regs_); i++) {
          regs_[addr + (i - 3)] = out[i];
        }
      }
      break;

    case kReadRegister:
      if (len >= 4) {
        uint16_t addr = ((uint16_t)out[1] << 8) | out[2];
        for (size_t i = 4; i < len; i++) {
          size_t idx = (size_t)addr + (i - 4);
          in[i] = idx < sizeof(regs_) ? regs_[idx] : 0;
        }
      }
      break;

    case kSetDioIrqParams:
      if (len >= 9) {
        irqMask_ = ((uint16_t)out[1] << 8) | out[2];
      }
      break;

    case kGetIrqStatus:
      // [op][nop][status][irq hi][irq lo]
      if (len >= 4) in[2] = (uint8_t)(irq_ >> 8);
      if (len >= 5) in[3] = (uint8_t)(irq_ & 0xFF);
      break;

    case kClearIrqStatus:
      if (len >= 3) {
        uint16_t clear = ((uint16_t)out[1] << 8) | out[2];
        irq_ &= (uint16_t)~clear;
        if (clear & kIrqPreambleDetected) preambleRaised_ = false;
        if (clear & kIrqHeaderValid) headerRaised_ = false;
      }
      break;

    case kGetRxBufferStatus:
      // [op][nop][status][payload len][start ptr]
      if (len >= 4) in[2] = rxLen_;
      if (len >= 5) in[3] = rxBase_;
      break;

    case kGetPacketStatus:
      // RSSI and SNR as the datasheet encodes them, from what the engine
      // measured - the one place a virtual chip can be exactly right.
      if (len >= 4) in[2] = (uint8_t)(-rssi_ * 2);
      if (len >= 5) in[3] = (uint8_t)(int8_t)(snr_ * 4);
      if (len >= 6) in[4] = (uint8_t)(-rssi_ * 2);
      break;

    case kGetRssiInst:
      if (len >= 3) in[2] = (uint8_t)(-rssi_ * 2);
      break;

    case kGetStatus: break;
    case kGetPacketType: if (len >= 3) in[2] = 0x01; break;  // LoRa
    case kGetDeviceErrors: break;
    case kClearDeviceErrors: break;
    default: break;  // acknowledged and ignored
  }
}
