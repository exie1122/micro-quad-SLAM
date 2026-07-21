#ifndef LOCAL_PATH_PLANNER_H
#define LOCAL_PATH_PLANNER_H

#include "autonomy_types.h"

typedef enum {
  PLAN_OK = 0,
  PLAN_INVALID_INPUT,
  PLAN_START_BLOCKED,
  PLAN_GOAL_BLOCKED,
  PLAN_SEARCH_LIMIT,
  PLAN_PATH_LIMIT,
  PLAN_NO_PATH,
  PLAN_INSUFFICIENT_CLEARANCE,
  PLAN_TOO_LONG,
  PLAN_TOO_COMPLEX
} PlanStatus;

typedef struct {
  int32_t parent[AUTONOMY_MAX_CELLS];
  uint32_t queue[AUTONOMY_MAX_CELLS];
  uint8_t visited[AUTONOMY_MAX_CELLS];
} PlannerWorkspace;

PlanStatus local_path_plan(const OccupancyGrid *grid, GridPoint start, GridPoint goal,
                           const AutonomyConfig *config, PlannerWorkspace *workspace,
                           LocalPath *path);
bool local_path_next_subgoal(const LocalPath *path, GridPoint current, float max_step_m,
                             GridPoint *subgoal);
bool local_path_segment_valid(const OccupancyGrid *grid, GridPoint start, GridPoint end);

#endif
