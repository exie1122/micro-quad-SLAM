#ifndef PROGRESS_MONITOR_H
#define PROGRESS_MONITOR_H
#include "autonomy_types.h"
typedef enum {PROGRESS_OK=0,PROGRESS_REACHED,PROGRESS_STALLED,PROGRESS_DIVERGED,PROGRESS_EXCESS_SPEED,PROGRESS_INVALID} ProgressResult;
typedef struct {GridPoint start,target;float best_distance_m;uint64_t started_ms,last_gain_ms;bool active;} ProgressMonitor;
void progress_monitor_start(ProgressMonitor *monitor,GridPoint start,GridPoint target,uint64_t now_ms);
ProgressResult progress_monitor_update(ProgressMonitor *monitor,const VehiclePose *pose,uint64_t now_ms,const AutonomyConfig *config);
#endif
