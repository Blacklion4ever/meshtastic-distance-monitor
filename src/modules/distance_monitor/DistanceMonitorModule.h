#pragma once

#include "concurrency/OSThread.h"
#include "mesh/SinglePortModule.h"

#include "DistanceMonitorConfig.h"

/**
 * Distance monitor core module.
 *
 * Initial implementation:
 * - registers as a Meshtastic module;
 * - runs periodically;
 * - reports its role and version through debug logs.
 *
 * Radio messages, positioning and alarms will be added incrementally.
 */
class DistanceMonitorModule : public SinglePortModule, private concurrency::OSThread
{
public:
  DistanceMonitorModule();

  void setup() override;

protected:
  int32_t runOnce() override;

private:
  static const char *roleName();
  int32_t getDistNodeNum();
};