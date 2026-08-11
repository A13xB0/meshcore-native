// The radio model, between an emulated MCU and the RF engine.
//
// A native node reaches VirtualSX1262 in process through SimHal. An emulated one
// cannot: its firmware is inside QEMU, so the chip has to sit outside and be
// reachable from both sides at once.
//
//   QEMU  --- SPI transactions --->  this  --- frames and ticks --->  engine
//
// Deliberately the same chip object as the native path. A second model would be
// a second thing to keep in agreement, and the first time the two drifted every
// comparison between a native node and an emulated one would be measuring our
// own code rather than MeshCore's.
//
// One thing is different from a native node and it matters. There, the bridge
// owns the firmware's execution: a tick runs loop() a millisecond at a time and
// nothing else happens in between. Here the firmware runs inside an emulator on
// its own schedule, so SPI transactions arrive whenever QEMU feels like it,
// while ticks arrive from the engine. Both mutate the chip, so both take a lock,
// and the ordering between them is not reproducible the way a native node's is.
// See the note at the bottom.
//
// Usage:
//   radioserver <spi socket> [--bridge host:port]

#include "VirtualSX1262.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <mutex>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace {

// The QEMU side. Small because it is on the hot loop of every SPI byte.
enum : uint8_t {
  kCsAssert = 0x01,
  kCsRelease = 0x02,
  kXfer = 0x03,
  kReadBusy = 0x04,
};

// The engine side, shared with the simulator's Go half and with bridge/main.cpp.
constexpr uint8_t kFrame = 0x01;
constexpr uint8_t kTick = 0x02;
constexpr uint8_t kAck = 0x03;
constexpr uint8_t kTxDone = 0x04;
// Console traffic reaches an emulated node over the emulator's own serial
// port, so these two are named here only to be ignored deliberately rather
// than to fall through to the unknown-message path.
constexpr uint8_t kConsoleIn = 0x06;
constexpr uint8_t kChannelBusy = 0x08;
constexpr uint8_t kRadioStats = 0x09;

VirtualSX1262 gChip;
std::mutex gChipMu;          // QEMU and the engine both reach the chip
uint32_t gSimMillis = 0;

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

bool writeMsg(int fd, uint8_t kind, const uint8_t* p, size_t n) {
  uint8_t hdr[3] = {kind, (uint8_t)(n >> 8), (uint8_t)n};
  if (!writeAll(fd, hdr, 3)) return false;
  return n == 0 || writeAll(fd, p, n);
}

// Anything the firmware handed its radio goes out to the engine now.
//
// Transmission reaches the channel immediately and is *not* immediately
// complete: the chip stays in transmit until the engine sends kTxDone, exactly
// as a native node does, because that is what stops a node talking over itself.
void drainTx(int bridgeFd) {
  if (bridgeFd < 0 || !gChip.hasPendingTx) return;
  gChip.hasPendingTx = false;
  writeMsg(bridgeFd, kFrame, gChip.pendingTx.data(), gChip.pendingTx.size());
}

int dialBridge(const std::string& addr) {
  auto colon = addr.rfind(':');
  if (colon == std::string::npos) {
    fprintf(stderr, "radioserver: --bridge wants host:port, got %s\n", addr.c_str());
    return -1;
  }
  std::string host = addr.substr(0, colon), port = addr.substr(colon + 1);

  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* res = nullptr;
  if (getaddrinfo(host.c_str(), port.c_str(), &hints, &res) != 0) {
    fprintf(stderr, "radioserver: cannot resolve %s\n", addr.c_str());
    return -1;
  }
  int fd = -1;
  for (addrinfo* a = res; a; a = a->ai_next) {
    fd = ::socket(a->ai_family, a->ai_socktype, a->ai_protocol);
    if (fd < 0) continue;
    if (::connect(fd, a->ai_addr, a->ai_addrlen) == 0) break;
    ::close(fd);
    fd = -1;
  }
  freeaddrinfo(res);
  if (fd < 0) {
    fprintf(stderr, "radioserver: cannot reach the engine at %s\n", addr.c_str());
    return -1;
  }
  // Frames are small and latency is the whole game here: a tick that waits on
  // Nagle is a node that answers late for no reason.
  int one = 1;
  ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  return fd;
}

// One message from the engine.
bool serviceBridge(int fd) {
  uint8_t hdr[3];
  if (!readAll(fd, hdr, 3)) return false;
  const uint8_t kind = hdr[0];
  const size_t n = ((size_t)hdr[1] << 8) | hdr[2];
  std::vector<uint8_t> payload(n);
  if (n && !readAll(fd, payload.data(), n)) return false;

  std::lock_guard<std::mutex> lock(gChipMu);
  switch (kind) {
    case kFrame:
      // A packet the channel delivered. Only CRC-passing frames arrive here,
      // exactly as on hardware: everything else was recorded and withheld.
      gChip.inbox.push_back(std::move(payload));
      break;

    case kTxDone:
      gChip.transmitFinished();
      break;

    case kChannelBusy:
      if (n >= 1) gChip.setChannelBusy(payload[0] != 0);
      break;

    case kTick: {
      if (n != 4) break;
      uint32_t at = ((uint32_t)payload[0] << 24) | ((uint32_t)payload[1] << 16) |
                    ((uint32_t)payload[2] << 8) | payload[3];
      // A millisecond at a time, as a native node does. Stepping rather than
      // jumping is what keeps the chip's own timeouts behaving: a preamble flag
      // that should clear after 66 ms does not, if time arrives in 500 ms
      // lumps.
      while (gSimMillis < at) {
        gSimMillis++;
        gChip.tick(gSimMillis);
        drainTx(fd);
      }
      gChip.tick(gSimMillis);
      drainTx(fd);

      uint32_t st[4] = {gChip.irqReads(), gChip.busyReads(), gChip.busyMs(),
                        gChip.spuriousRaises()};
      uint8_t sb[16];
      for (int k = 0; k < 4; k++) {
        sb[k * 4 + 0] = (uint8_t)(st[k] >> 24);
        sb[k * 4 + 1] = (uint8_t)(st[k] >> 16);
        sb[k * 4 + 2] = (uint8_t)(st[k] >> 8);
        sb[k * 4 + 3] = (uint8_t)st[k];
      }
      writeMsg(fd, kRadioStats, sb, sizeof(sb));
      if (!writeMsg(fd, kAck, payload.data(), 4)) return false;
      break;
    }

    case kConsoleIn:
      // Console input belongs to the firmware's serial port, and an emulated
      // node's serial port is the emulator's, not this socket. Ignoring it is
      // the whole handling: what matters is that it is not fatal. Treating it
      // as unknown killed the radio model the moment anything typed at the
      // fleet, and the node then reported "radio init failed: -2" - chip not
      // found - which points at wiring rather than at a console.
      break;

    default:
      // Skipped rather than fatal. The framing is length-prefixed and the
      // payload has already been read, so an unrecognised kind costs nothing
      // and cannot desynchronise the stream - whereas exiting takes the node
      // down for a message it did not need.
      fprintf(stderr, "radioserver: ignoring engine message 0x%02x (%zu bytes)\n",
              kind, n);
      break;
  }
  return true;
}

// One message from the emulator.
bool serviceQemu(int fd, uint64_t* transactions, uint64_t* bytes) {
  uint8_t tag = 0;
  if (!readAll(fd, &tag, 1)) return false;

  std::lock_guard<std::mutex> lock(gChipMu);
  switch (tag) {
    case kCsAssert:
      gChip.beginTransaction();
      return true;

    case kCsRelease:
      gChip.endTransaction();
      (*transactions)++;
      return true;

    case kXfer: {
      uint8_t out = 0;
      if (!readAll(fd, &out, 1)) return false;
      uint8_t in = gChip.transferByte(out);
      (*bytes)++;
      return writeAll(fd, &in, 1);
    }

    case kReadBusy: {
      // Always clear, which is what the native path does too: SimHal holds BUSY
      // low and VirtualSX1262 does not model the time a real chip spends
      // digesting a command. Answering differently here would make an emulated
      // node a different radio from a native one, which is the one thing this
      // whole arrangement exists to avoid.
      uint8_t busy = 0;
      return writeAll(fd, &busy, 1);
    }

    default:
      fprintf(stderr, "radioserver: unknown emulator tag 0x%02x\n", tag);
      return false;
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    fprintf(stderr, "usage: %s <spi socket> [--bridge host:port]\n", argv[0]);
    return 2;
  }
  const char* path = argv[1];
  std::string bridgeAddr;
  for (int i = 2; i < argc - 1; i++) {
    if (strcmp(argv[i], "--bridge") == 0) bridgeAddr = argv[i + 1];
  }

  // A broken pipe is an emulator that has exited, which is ordinary. Let the
  // read fail and tidy up rather than dying on a signal.
  ::signal(SIGPIPE, SIG_IGN);

  // Two ways in, because there are two emulators. QEMU is native and takes a
  // Unix socket; Renode runs on Mono, whose Unix domain socket support has been
  // unreliable for long enough that betting an emulated node on it is a poor
  // trade for one path separator. A leading colon asks for TCP on loopback.
  const bool useTcp = path[0] == ':';
  int srv = -1;
  if (useTcp) {
    const int port = atoi(path + 1);
    srv = ::socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) {
      perror("socket");
      return 1;
    }
    int on = 1;
    ::setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    sockaddr_in in{};
    in.sin_family = AF_INET;
    in.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    in.sin_port = htons((uint16_t)port);
    if (::bind(srv, (sockaddr*)&in, sizeof(in)) < 0) {
      perror("bind");
      return 1;
    }
    if (::listen(srv, 1) < 0) {
      perror("listen");
      return 1;
    }
    // The chosen port is printed because port 0 means "any", which is what a
    // harness starting several nodes at once wants: it reads the number back
    // rather than picking one and hoping.
    socklen_t len = sizeof(in);
    if (::getsockname(srv, (sockaddr*)&in, &len) == 0) {
      printf("radioserver: listening on 127.0.0.1:%d\n", ntohs(in.sin_port));
    }
  } else {
    ::unlink(path);
    srv = ::socket(AF_UNIX, SOCK_STREAM, 0);
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
  }
  fflush(stdout);

  int bridgeFd = -1;
  if (!bridgeAddr.empty()) {
    bridgeFd = dialBridge(bridgeAddr);
    if (bridgeFd < 0) return 1;
    printf("radioserver: joined the engine at %s\n", bridgeAddr.c_str());
    fflush(stdout);
  } else {
    // Worth saying. Without the engine this chip transmits into nowhere and
    // never receives, so the firmware comes up and then waits for ever on a
    // transmission that cannot complete - which looks like a hang rather than
    // like a missing argument.
    printf("radioserver: no --bridge, so this node is deaf and mute\n");
    fflush(stdout);
  }

  int qemuFd = ::accept(srv, nullptr, nullptr);
  if (qemuFd < 0) {
    perror("accept");
    return 1;
  }
  printf("radioserver: emulator connected\n");
  fflush(stdout);

  uint64_t transactions = 0, bytes = 0;
  for (;;) {
    pollfd fds[2];
    int n = 0;
    fds[n++] = {qemuFd, POLLIN, 0};
    if (bridgeFd >= 0) fds[n++] = {bridgeFd, POLLIN, 0};

    if (::poll(fds, n, -1) < 0) break;

    if (fds[0].revents & (POLLIN | POLLHUP)) {
      if (!serviceQemu(qemuFd, &transactions, &bytes)) break;
    }
    if (bridgeFd >= 0 && (fds[1].revents & (POLLIN | POLLHUP))) {
      if (!serviceBridge(bridgeFd)) {
        fprintf(stderr, "radioserver: the engine went away\n");
        break;
      }
    }
  }

  printf("radioserver: %llu transactions, %llu bytes\n",
         (unsigned long long)transactions, (unsigned long long)bytes);
  if (bridgeFd >= 0) ::close(bridgeFd);
  ::close(qemuFd);
  ::close(srv);
  if (!useTcp) ::unlink(path);
  return 0;
}

// What this is not, and it is worth being exact.
//
// A native node runs in lockstep: the engine supplies the clock, the bridge runs
// loop() one millisecond at a time, and the same seed gives the same answer
// every time. Here the engine still supplies the clock to the *chip*, so IRQ
// timing and channel state are engine-relative - but the *firmware* runs inside
// an emulator on wall time, so the instant at which it reads a register is not
// reproducible.
//
// The consequence is narrow and real: an emulated node can transmit and receive
// and take part in a mesh, and two runs of one seed will not produce identical
// ledgers. Mixing emulated and native nodes in a measurement therefore costs the
// determinism the native path has. Fixing it means QEMU's -icount with the
// engine driving virtual time, which is the same contract the native bridge
// already implements.
