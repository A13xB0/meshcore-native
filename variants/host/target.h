// The host "board" MeshCore's applications are built against.
//
// Every MeshCore variant provides a target.h declaring these globals and two
// functions; this is that same contract, implemented for a machine rather than
// a board. It exists so the real applications - with their own forwarding
// policy, CLI and preferences - compile and run here unmodified.
//
// The radio is MeshCore's own CustomSX1262 over real RadioLib, on a virtual
// chip. It used to be HostRadio, a hand-written stand-in for the whole driver
// stack: that made every listen-before-talk decision ours rather than the
// firmware's, and two versions differing only in their driver produced
// bit-identical results. See docs/virtual-sx1262.md in meshcoresim.
#pragma once

#include <Mesh.h>
#include <InternalFileSystem.h>
#include <helpers/radiolib/CustomSX1262Wrapper.h>

#include "HostBoard.h"
#include "HostSensors.h"
#include "SimHal.h"

extern HostBoard board;
extern CustomSX1262 radio;
extern CustomSX1262Wrapper radio_driver;
extern HostRTCClock rtc_clock;
extern HostSensorManager sensors;

bool radio_init();
mesh::LocalIdentity radio_new_identity();
