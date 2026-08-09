# meshcore-native

Real [MeshCore](https://github.com/meshcore-dev/MeshCore) firmware, compiled for
desktop architectures, with its radio and its console on a socket.

These are not models of MeshCore. They are MeshCore's own applications —
`examples/simple_repeater`, `examples/companion_radio` and the rest — built
unmodified against a host *variant*, so the forwarding policy, the CSMA timing,
the CLI and the preferences are the firmware's and not a reimplementation of it.
A simulator that decides for itself which packets get relayed is answering a
different question from the one anybody asked.

## A node is a node

There is no list of supported node types here, and that is deliberate. A
MeshCore node is a radio in a place running an application; whether it is a
repeater, a companion, a room server or a sensor is settled entirely by which
application is linked. So the pipeline reads `examples/` at build time and builds
whatever it finds. When upstream ships a new kind of node, it appears in the next
nightly release without this repository changing.

The same goes for versions: "every previous release" is not a fixed list, it is
whatever upstream has tagged by the time the workflow runs.

## What gets released

| release | tracks |
|---|---|
| `main` | upstream's `main` branch, rebuilt when it moves |
| `dev` | upstream's `dev` branch, marked pre-release |
| `<tag>` | each upstream tag, built once and never rebuilt |

Each release carries one binary per role per platform, named
`meshcore-<role>-<os>-<arch>`:

| | x86 | x64 | arm64 |
|---|---|---|---|
| Linux | ✓ | ✓ | ✓ |
| Windows | ✓ | ✓ | ✓ |
| macOS | — | ✓ | ✓ |

macOS has had no 32-bit userland since Catalina, so there is nothing to build
there.

A release body contains a `meshcore-commit:` line. That is not decoration — the
next run reads it back to decide whether the ref still needs building, which is
what stops a nightly schedule recompiling three tags that have not changed since
they were cut.

## Running one

```
meshcore-simple_repeater-linux-amd64 --bridge 127.0.0.1:9000 --seed 4417 --sf 10 --bw-khz 250 --cr 1
```

The binary connects to the simulator and then does nothing on its own clock. The
simulator owns time: it sends a tick, the node runs `loop()` once per simulated
millisecond and acknowledges. That is what makes a run reproducible from its
seed, and it is why `delay()` here does nothing — a firmware that busy-waited for
`millis()` to move would wait forever.

`--print-airtime N` reports the firmware's own `getEstAirtimeFor(N)` and exits,
so a caller can check that its channel model and this build still agree about how
long a packet occupies the air. Two copies of a formula that nothing compares are
two formulas.

## The host variant

`variants/host/` is a MeshCore board like any other: it provides `target.h`,
the four globals every variant declares, and the platform underneath them.
Nothing in it is role-aware.

What it stands in for, and why it answers the way it does:

- **The radio** is a socket. Transmission reaches the wire immediately and is
  *not* immediately complete — `isSendComplete()` stays false until the simulator
  says the waveform ended, because how long a signal occupied the channel is a
  property of the samples the simulator generated, not of a formula this end
  could evaluate.
- **The clock** is the simulator's. `millis()` reads a variable the bridge writes.
- **The filesystem** is real files under a directory the simulator owns, with the
  block geometry of an nRF52840's `InternalFS`. The identity, preferences and ACL
  a repeater writes are genuinely persisted, so a CLI that changes a setting
  changes something.
- **Randomness** is seeded. Identity generation on hardware samples radio noise,
  which is correct there and the one thing that cannot be replayed.
- **Sensors and I²C** report absent. That is a case the firmware already handles,
  and it is a truthful answer rather than a plausible invented reading.

`roles.d/<role>.flags` is optional per-role build flags. Anything without a file
there builds with the defaults, which is the point.

## Building locally

```
MESHCORE=path/to/MeshCore CRYPTO=path/to/arduinolibs/libraries/Crypto \
  ./build.sh simple_repeater out
```

Cross-compiling is `TARGET_OS`, `TARGET_ARCH`, `CXX`, `CC` and `EXTRA_FLAGS`.

Sources that will not compile for a host are dropped rather than failing the
build — a helper that wants an SPI bus has nothing to say here. If dropping one
leaves the link short of a symbol, that role does not build on that platform for
that version of MeshCore, and the build says so instead of papering over it.

## Licence

MIT. MeshCore is MIT, this links MeshCore, and that is why these builds are
released from here rather than from the simulator that consumes them — see
`NOTICE.md`.
