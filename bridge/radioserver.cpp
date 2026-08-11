// The radio model, offered on a socket for an emulated MCU to clock.
//
// A native node calls VirtualSX1262 in process, through SimHal. An emulated one
// cannot: the firmware is inside QEMU, and its SPI controller reaches out over
// a socket to whatever is modelling the chip. This is that end.
//
// It is deliberately the same chip object either way. Writing a second model
// for the emulated path would give two things to keep in agreement, and the
// first time they drifted every comparison between a native node and an
// emulated one would be measuring our own code rather than MeshCore's.
//
// The protocol is the one QEMU's sx1262 device speaks, and it is small because
// it sits on the hot path of every SPI byte:
//
//   0x01  chip select asserted     -> beginTransaction()
//   0x02  chip select released     -> endTransaction()
//   0x03  one byte out, one back   -> transferByte()
//   0x04  read the BUSY line       -> one byte, 0 or 1
//
// Usage:
//   radioserver /run/user/1000/meshbench-radio-7.sock

#include "VirtualSX1262.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <string>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace {

enum : uint8_t {
  kCsAssert = 0x01,
  kCsRelease = 0x02,
  kXfer = 0x03,
  kReadBusy = 0x04,
};

// Read exactly n bytes, or say the peer has gone.
bool readAll(int fd, void* buf, size_t n) {
  auto* p = static_cast<uint8_t*>(buf);
  while (n > 0) {
    ssize_t got = ::read(fd, p, n);
    if (got <= 0) return false;
    p += got;
    n -= (size_t)got;
  }
  return true;
}

bool writeAll(int fd, const void* buf, size_t n) {
  auto* p = static_cast<const uint8_t*>(buf);
  while (n > 0) {
    ssize_t put = ::write(fd, p, n);
    if (put <= 0) return false;
    p += put;
    n -= (size_t)put;
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    fprintf(stderr, "usage: %s <socket path>\n", argv[0]);
    return 2;
  }
  const char* path = argv[1];

  // A broken pipe is an emulator that has exited, which is ordinary. Let the
  // read fail and tidy up rather than dying on a signal.
  ::signal(SIGPIPE, SIG_IGN);

  ::unlink(path);

  int srv = ::socket(AF_UNIX, SOCK_STREAM, 0);
  if (srv < 0) {
    perror("socket");
    return 1;
  }
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  if (strlen(path) >= sizeof(addr.sun_path)) {
    fprintf(stderr, "radioserver: socket path too long: %s\n", path);
    return 1;
  }
  strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);

  if (::bind(srv, (sockaddr*)&addr, sizeof(addr)) < 0) {
    perror("bind");
    return 1;
  }
  if (::listen(srv, 1) < 0) {
    perror("listen");
    return 1;
  }
  printf("radioserver: listening on %s\n", path);
  fflush(stdout);

  VirtualSX1262 chip;

  int fd = ::accept(srv, nullptr, nullptr);
  if (fd < 0) {
    perror("accept");
    return 1;
  }
  printf("radioserver: emulator connected\n");
  fflush(stdout);

  uint64_t transactions = 0, bytes = 0;

  for (;;) {
    uint8_t tag = 0;
    if (!readAll(fd, &tag, 1)) break;

    switch (tag) {
      case kCsAssert:
        chip.beginTransaction();
        break;

      case kCsRelease:
        chip.endTransaction();
        transactions++;
        break;

      case kXfer: {
        uint8_t out = 0;
        if (!readAll(fd, &out, 1)) goto done;
        uint8_t in = chip.transferByte(out);
        bytes++;
        if (!writeAll(fd, &in, 1)) goto done;
        break;
      }

      case kReadBusy: {
        // Never busy for now. BUSY is asserted by the chip while it digests a
        // command, and modelling that needs the simulated clock this process
        // does not yet have - see the note below about time.
        uint8_t busy = 0;
        if (!writeAll(fd, &busy, 1)) goto done;
        break;
      }

      default:
        fprintf(stderr, "radioserver: unknown tag 0x%02x; the stream has "
                        "desynchronised, closing\n", tag);
        goto done;
    }
  }

done:
  printf("radioserver: %llu transactions, %llu bytes\n",
         (unsigned long long)transactions, (unsigned long long)bytes);
  ::close(fd);
  ::close(srv);
  ::unlink(path);
  return 0;
}

// Not here yet, and both are the same missing thing: simulated time.
//
//   * BUSY always reads clear. A real chip raises it while it works, and the
//     driver waits on it. Answering truthfully means knowing what time it is.
//   * Nothing connects this chip to the RF engine, so it transmits into
//     nowhere and never receives. VirtualSX1262 already has pendingTx and an
//     inbox for exactly that; they need the bridge on the other side.
//
// Both arrive with the lockstep link, which is what gives an emulated node the
// same clock every native node already runs on.
