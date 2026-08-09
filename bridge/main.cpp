// The bridge: main() for a MeshCore application built against the host variant.
//
// A board's runtime calls setup() once and loop() forever, driven by a crystal.
// This calls the same two functions, driven by a socket — the simulator owns the
// clock, the antenna and the console, and this is the only file that knows that.
//
// Nothing here is role-aware, and that is deliberate. A node is a node: a radio
// at a place running an application. Which application — repeater, companion,
// room server, sensor, or something MeshCore has not shipped yet — is settled by
// what gets linked alongside this file, not by anything this file or the
// simulator decides. When upstream adds a new example directory, it builds.
// Sockets, on the three families of desktop this ships for.
//
// Winsock is not a POSIX socket layer with different headers: it needs
// initialising, its handles are not file descriptors, and closesocket is not
// close. Confining that to five lines here is cheaper than the alternative,
// which is a Windows build that silently is not tested.
#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  using sock_t = SOCKET;
  #define BAD_SOCK INVALID_SOCKET
  #define CLOSE_SOCK closesocket
  static int sockRead(sock_t s, void* p, size_t n) { return recv(s, (char*)p, (int)n, 0); }
  static int sockWrite(sock_t s, const void* p, size_t n) { return send(s, (const char*)p, (int)n, 0); }
#else
  #include <arpa/inet.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <sys/socket.h>
  #include <unistd.h>
  using sock_t = int;
  #define BAD_SOCK (-1)
  #define CLOSE_SOCK close
  static int sockRead(sock_t s, void* p, size_t n) { return (int)read(s, p, n); }
  static int sockWrite(sock_t s, const void* p, size_t n) { return (int)write(s, p, n); }
#endif

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

#include "target.h"

// The application's entry points, as on any Arduino target.
void setup();
void loop();

// Defined by the host variant.
extern uint32_t g_sim_millis;
extern int g_sf;
extern float g_bwKHz;
extern int g_cr;
extern uint32_t g_identity_seed;

namespace {

// The wire protocol, shared with the simulator's Go side.
constexpr uint8_t kFrame = 0x01;       // a packet, either direction
constexpr uint8_t kTick = 0x02;        // advance simulated time to N ms
constexpr uint8_t kAck = 0x03;         // this node has caught up to N ms
constexpr uint8_t kTxDone = 0x04;      // the waveform has left the antenna
constexpr uint8_t kOriginate = 0x05;   // send a message of the node's own
constexpr uint8_t kConsoleIn = 0x06;   // bytes typed at the node's UART
constexpr uint8_t kConsoleOut = 0x07;  // bytes the node printed

sock_t gFd = BAD_SOCK;
std::deque<uint8_t> gConsoleIn;
std::vector<char> gConsoleOut;

bool readAll(sock_t fd, uint8_t* p, size_t n) {
  while (n) {
    int r = sockRead(fd, p, n);
    if (r <= 0) return false;
    p += r;
    n -= (size_t)r;
  }
  return true;
}

bool writeMsg(sock_t fd, uint8_t kind, const uint8_t* p, size_t n) {
  uint8_t hdr[3] = {kind, (uint8_t)(n >> 8), (uint8_t)n};
  if (sockWrite(fd, hdr, 3) != 3) return false;
  while (n) {
    int w = sockWrite(fd, p, n);
    if (w <= 0) return false;
    p += w;
    n -= (size_t)w;
  }
  return true;
}

// The console seam. MeshCore's applications carry their own CLI on Serial, and
// carrying it over the bridge is what makes clicking a node in the simulator
// reach a real command interface rather than a mock of one.
void consoleWrite(const char* p, size_t n) { gConsoleOut.insert(gConsoleOut.end(), p, p + n); }

int consoleRead() {
  if (gConsoleIn.empty()) return -1;
  int c = gConsoleIn.front();
  gConsoleIn.pop_front();
  return c;
}

void flushConsole() {
  if (gConsoleOut.empty()) return;
  writeMsg(gFd, kConsoleOut, (const uint8_t*)gConsoleOut.data(), gConsoleOut.size());
  gConsoleOut.clear();
}

// Anything the application handed its radio goes out now.
//
// Transmission reaches the wire immediately and is *not* immediately complete:
// isSendComplete() stays false until the engine sends kTxDone. The node cannot
// time its own transmission, because how long the signal occupied the channel is
// a property of the samples the engine generated. Computing it here from the
// airtime estimate would replace the simulation with the formula.
void drainTx() {
  if (!radio_driver.hasPendingTx) return;
  radio_driver.hasPendingTx = false;
  writeMsg(gFd, kFrame, radio_driver.pendingTx.data(), radio_driver.pendingTx.size());
}

sock_t connectTo(const std::string& addr) {
#ifdef _WIN32
  WSADATA wsa;
  if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return BAD_SOCK;
#endif
  auto colon = addr.rfind(':');
  if (colon == std::string::npos) {
    fprintf(stderr, "bridge: --bridge wants host:port, got %s\n", addr.c_str());
    return BAD_SOCK;
  }
  std::string host = addr.substr(0, colon);
  int port = atoi(addr.c_str() + colon + 1);

  sock_t fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd == BAD_SOCK) return BAD_SOCK;
  sockaddr_in sa{};
  sa.sin_family = AF_INET;
  sa.sin_port = htons((uint16_t)port);
  if (inet_pton(AF_INET, host.c_str(), &sa.sin_addr) != 1) {
    CLOSE_SOCK(fd);
    fprintf(stderr, "bridge: cannot parse address %s\n", host.c_str());
    return BAD_SOCK;
  }
  if (connect(fd, (sockaddr*)&sa, sizeof sa) != 0) {
    CLOSE_SOCK(fd);
    return BAD_SOCK;
  }
  // Nagle would coalesce a frame with the tick that follows it, which is exactly
  // the latency the lockstep round trip exists to avoid paying.
  int one = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof one);
  return fd;
}

}  // namespace

int main(int argc, char** argv) {
  std::string bridge;
  int printAirtimeFor = -1;
  for (int i = 1; i < argc; i++) {
    auto next = [&]() { return i + 1 < argc ? argv[++i] : ""; };
    if (!strcmp(argv[i], "--bridge")) bridge = next();
    else if (!strcmp(argv[i], "--seed")) g_identity_seed = (uint32_t)strtoul(next(), nullptr, 10);
    else if (!strcmp(argv[i], "--sf")) g_sf = atoi(next());
    else if (!strcmp(argv[i], "--bw-khz")) g_bwKHz = (float)atof(next());
    else if (!strcmp(argv[i], "--cr")) g_cr = atoi(next());
    else if (!strcmp(argv[i], "--print-airtime")) printAirtimeFor = atoi(next());
  }

  // A self-report, so the simulator can check that this transcription of the
  // airtime formula still agrees with its own. Two copies of a formula that
  // nothing compares are two formulas.
  if (printAirtimeFor >= 0) {
    printf("%u\n", radio_driver.getEstAirtimeFor(printAirtimeFor));
    return 0;
  }
  if (bridge.empty()) {
    fprintf(stderr,
            "usage: %s --bridge host:port [--seed N] [--sf N] [--bw-khz F] [--cr N]\n"
            "A MeshCore node with its radio and console on a socket.\n",
            argv[0]);
    return 2;
  }

  gFd = connectTo(bridge);
  if (gFd == BAD_SOCK) {
    fprintf(stderr, "bridge: cannot reach the simulator at %s\n", bridge.c_str());
    return 1;
  }
  Serial.attach(consoleWrite, consoleRead);

  setup();
  drainTx();
  flushConsole();

  uint8_t hdr[3];
  for (;;) {
    if (!readAll(gFd, hdr, 3)) break;
    uint16_t n = (uint16_t)((hdr[1] << 8) | hdr[2]);
    std::vector<uint8_t> payload(n);
    if (n && !readAll(gFd, payload.data(), n)) break;

    switch (hdr[0]) {
      case kFrame:
        // Queued, not delivered. The application collects it from recvRaw() on
        // its next loop, exactly as it would drain a real radio's FIFO.
        radio_driver.inbox.push_back(std::move(payload));
        break;

      case kTxDone:
        radio_driver.transmitFinished();
        break;

      case kConsoleIn:
        gConsoleIn.insert(gConsoleIn.end(), payload.begin(), payload.end());
        break;

      case kOriginate:
        // The application owns the mesh instance, not this file, so there is no
        // generic way to make it author a packet — and fabricating one here is
        // exactly what does not work: MeshCore drops what is not a valid packet,
        // correctly, and nothing relays.
        //
        // Said out loud rather than ignored. A silently dropped request looks
        // identical to a message that was sent and reached nobody, which is the
        // single most misleading thing this bridge could do. Originate traffic
        // through the node's own CLI instead — that is what the console is for.
        fprintf(stderr,
                "bridge: this application cannot be asked to originate; "
                "send through its CLI on the console instead\n");
        break;

      case kTick: {
        if (n != 4) break;
        uint32_t at = ((uint32_t)payload[0] << 24) | ((uint32_t)payload[1] << 16) |
                      ((uint32_t)payload[2] << 8) | payload[3];
        // One loop per millisecond of simulated time. Stepping rather than
        // jumping is what keeps timeouts, retries and duty-cycle refill behaving
        // as they do on hardware: a node that sees time move in 500 ms jumps
        // takes different branches.
        while (g_sim_millis < at) {
          g_sim_millis++;
          loop();
          drainTx();
        }
        loop();
        drainTx();
        flushConsole();
        if (!writeMsg(gFd, kAck, payload.data(), 4)) goto done;
        break;
      }

      default:
        goto done;
    }
  }
done:
  fprintf(stderr, "bridge: closed after %u ms\n", g_sim_millis);
  CLOSE_SOCK(gFd);
  return 0;
}
