#ifndef RUN_LOGGER_H
#define RUN_LOGGER_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "autonomy_types.h"
#include "mission_state_machine.h"

typedef struct {FILE *events;FILE *frontiers;FILE *telemetry;char directory[512];uint64_t start_ms;} RunLogger;
bool run_logger_open(RunLogger *logger,const char *directory,uint64_t start_ms);
void run_logger_event(RunLogger *logger,uint64_t timestamp_ms,const char *event,const char *reason,
                      MissionState state,SafetyResult safety,const MotionCommand *command);
void run_logger_frontier(RunLogger *logger,uint64_t timestamp_ms,const FrontierCandidate *frontier);
void run_logger_frontier_rejection(RunLogger *logger,uint64_t timestamp_ms,uint32_t cluster_id,
                                   const char *reason);
void run_logger_telemetry(RunLogger *logger,const MissionContext *context,MissionState state,
                          bool armed,uint32_t mode,const LocalPath *path,const GridPoint *subgoal);
void run_logger_close(RunLogger *logger);

#endif
