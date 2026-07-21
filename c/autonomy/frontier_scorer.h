#ifndef FRONTIER_SCORER_H
#define FRONTIER_SCORER_H

#include "autonomy_types.h"

bool frontier_score_candidate(FrontierCandidate *candidate,const AutonomyConfig *config);
int frontier_select_best(FrontierCandidate *candidates,uint16_t count,const AutonomyConfig *config);

#endif
