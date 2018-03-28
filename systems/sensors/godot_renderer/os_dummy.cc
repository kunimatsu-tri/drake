#include "drake/systems/sensors/godot_renderer/os_dummy.h"

RWLock *RWLockDummy::create() {
  return memnew(RWLockDummy);
};

void RWLockDummy::make_default() {
  RWLock::create_func = &RWLockDummy::create;
};

IP *IPDummy::create() {
  return memnew(IPDummy);
};

void IPDummy::make_default() {
  IP::_create = &IPDummy::create;
};
