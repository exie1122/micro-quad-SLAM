#include "frontier_scorer.h"
#include <math.h>

bool frontier_score_candidate(FrontierCandidate *f,const AutonomyConfig *c){
  if(!f||!c||!f->reachable||!isfinite(f->path_distance_m)||f->path_distance_m<0.0f||
     !isfinite(f->expected_unknown_gain)||!isfinite(f->heading_change_deg)||
     !isfinite(f->obstacle_risk)||!isfinite(f->revisit_penalty)||
     !isfinite(f->uncertainty)||!isfinite(f->path_complexity)) return false;
  f->score=c->information_gain_weight*f->expected_unknown_gain-
           c->distance_weight*f->path_distance_m-
           c->turn_weight*fabsf(f->heading_change_deg)-
           c->risk_weight*f->obstacle_risk-
           c->revisit_weight*f->revisit_penalty-
           c->confidence_weight*f->uncertainty-
           c->path_complexity_weight*f->path_complexity;
  return isfinite(f->score);
}

int frontier_select_best(FrontierCandidate *f,uint16_t n,const AutonomyConfig *c){
  int best=-1;
  for(uint16_t i=0;i<n;++i){
    if(!frontier_score_candidate(&f[i],c))continue;
    if(best<0||f[i].score>f[best].score+1e-6f||
       (fabsf(f[i].score-f[best].score)<1e-6f&&
        (f[i].path_distance_m<f[best].path_distance_m-1e-6f||
         (fabsf(f[i].path_distance_m-f[best].path_distance_m)<1e-6f&&f[i].id<f[best].id)))) best=(int)i;
  }
  return best;
}
