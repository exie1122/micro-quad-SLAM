#ifndef COMMAND_SMOOTHER_H
#define COMMAND_SMOOTHER_H

#include "autonomy_types.h"

typedef struct {MotionCommand last;uint64_t last_update_ms;} CommandSmoother;

bool command_smoother_update(CommandSmoother *s,const AutonomyConfig *c,
                             const MotionCommand *requested,uint64_t now_ms,
                             bool horizontal_motion_allowed,bool landing,
                             MotionCommand *output,bool *clamped);
bool command_limit_subgoal(GridPoint current,GridPoint proposed,float max_step_m,GridPoint *limited);

#endif
