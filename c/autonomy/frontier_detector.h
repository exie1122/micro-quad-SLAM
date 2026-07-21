#ifndef FRONTIER_DETECTOR_H
#define FRONTIER_DETECTOR_H

#include "autonomy_types.h"

typedef struct {
  uint8_t visited[AUTONOMY_MAX_CELLS];
  uint32_t queue[AUTONOMY_MAX_CELLS];
} FrontierWorkspace;

bool frontier_detect(const OccupancyGrid *grid, const AutonomyConfig *config,
                     uint64_t now_ms, FrontierWorkspace *workspace, FrontierSet *result);
const char *frontier_rejection_name(FrontierRejection reason);

#endif
