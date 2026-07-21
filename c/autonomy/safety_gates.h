#ifndef SAFETY_GATES_H
#define SAFETY_GATES_H

#include "autonomy_types.h"

typedef struct {
  uint64_t transient_failure_since_ms;
  SafetyReason last_reason;
} SafetyMonitor;

SafetyResult evaluate_safety(const MissionContext *context,const AutonomyConfig *config,
                             SafetyMonitor *monitor);
const char *safety_reason_name(SafetyReason reason);

#endif
