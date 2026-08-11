// The streaming path must answer exactly as the buffer path does.
//
// There is one chip model and two ways of clocking it: the in-process HAL hands
// over a whole SPI buffer, and an emulated MCU clocks single bytes with the chip
// select marking the boundaries. If those two ever disagree, a node running
// under emulation is a different radio from a node running natively, and every
// comparison between them is measuring the difference between our two code
// paths rather than anything about MeshCore.
//
//   c++ -std=c++17 -I variants/host variants/host/streaming_test.cpp \
//       variants/host/VirtualSX1262.cpp -o /tmp/streaming_test && /tmp/streaming_test

#include "VirtualSX1262.h"

#include <cstdio>
#include <cstring>
#include <vector>

static int failures = 0;

// Run one command both ways, against two chips in the same state, and compare
// every byte that comes back.
static void bothWays(const char* what, std::vector<uint8_t> cmd) {
  VirtualSX1262 buffered;
  VirtualSX1262 streamed;

  std::vector<uint8_t> viaBuffer(cmd.size(), 0);
  buffered.spiTransfer(cmd.data(), cmd.size(), viaBuffer.data());

  std::vector<uint8_t> viaStream;
  streamed.beginTransaction();
  for (uint8_t b : cmd) viaStream.push_back(streamed.transferByte(b));
  streamed.endTransaction();

  for (size_t i = 0; i < cmd.size(); i++) {
    if (viaBuffer[i] != viaStream[i]) {
      printf("FAIL %-22s byte %zu: buffered 0x%02x, streamed 0x%02x\n",
             what, i, viaBuffer[i], viaStream[i]);
      failures++;
      return;
    }
  }
  printf("ok   %-22s %zu bytes\n", what, cmd.size());
}

int main() {
  // The version string RadioLib probes with. If this one disagrees the driver
  // decides there is no chip, which is the failure the whole exercise began
  // with.
  bothWays("ReadRegister version", {0x1D, 0x03, 0x20, 0x00, 0, 0, 0, 0, 0, 0});
  bothWays("GetStatus", {0xC0, 0x00, 0x00});
  bothWays("GetIrqStatus", {0x12, 0x00, 0x00, 0x00});
  bothWays("GetRxBufferStatus", {0x13, 0x00, 0x00, 0x00});
  bothWays("GetPacketStatus", {0x14, 0x00, 0x00, 0x00, 0x00});
  bothWays("GetDeviceErrors", {0x17, 0x00, 0x00, 0x00});

  // Writes and mode changes answer with a status byte and nothing else.
  bothWays("SetStandby", {0x80, 0x00});
  bothWays("SetPacketType LoRa", {0x8A, 0x01});
  bothWays("SetModulationParams", {0x8B, 0x07, 0x04, 0x01, 0x00});
  bothWays("SetBufferBase", {0x8F, 0x00, 0x00});
  bothWays("WriteRegister", {0x0D, 0x08, 0xAC, 0x94});

  // State written one way must be readable the other way, or the two paths
  // agree only on a chip nobody has configured.
  {
    VirtualSX1262 chip;
    const uint8_t write[] = {0x0D, 0x08, 0xAC, 0x5A};
    uint8_t sink[4] = {0};
    chip.spiTransfer(write, sizeof(write), sink);

    chip.beginTransaction();
    const uint8_t read[] = {0x1D, 0x08, 0xAC, 0x00, 0x00};
    uint8_t got = 0;
    for (size_t i = 0; i < sizeof(read); i++) got = chip.transferByte(read[i]);
    chip.endTransaction();

    if (got != 0x5A) {
      printf("FAIL %-22s wrote 0x5A through the buffer path, streamed back 0x%02x\n",
             "write then stream-read", got);
      failures++;
    } else {
      printf("ok   %-22s 0x5A survived the crossing\n", "write then stream-read");
    }
  }

  // And the other direction: a command sent byte by byte has to actually take
  // effect, not merely be answered politely.
  {
    VirtualSX1262 chip;
    const uint8_t write[] = {0x0D, 0x08, 0xAD, 0xC3};
    chip.beginTransaction();
    for (uint8_t b : write) chip.transferByte(b);
    chip.endTransaction();

    const uint8_t read[] = {0x1D, 0x08, 0xAD, 0x00, 0x00};
    uint8_t out[5] = {0};
    chip.spiTransfer(read, sizeof(read), out);
    if (out[4] != 0xC3) {
      printf("FAIL %-22s streamed write did not take effect, read 0x%02x\n",
             "stream-write then read", out[4]);
      failures++;
    } else {
      printf("ok   %-22s streamed write took effect\n", "stream-write then read");
    }
  }

  printf("\n%s\n", failures ? "FAILURES" : "all paths agree");
  return failures ? 1 : 0;
}
