#ifndef FRONTIER_HISTORY_H
#define FRONTIER_HISTORY_H
#include "autonomy_types.h"
#define FRONTIER_HISTORY_CAPACITY 32u
typedef struct {GridPoint goal;uint16_t attempts;uint16_t failures;uint64_t last_attempt_ms;bool valid;} FrontierHistoryEntry;
typedef struct {FrontierHistoryEntry entries[FRONTIER_HISTORY_CAPACITY];uint16_t next;} FrontierHistory;
float frontier_history_penalty(const FrontierHistory *history,GridPoint goal,uint64_t now_ms,float match_radius_m);
void frontier_history_record(FrontierHistory *history,GridPoint goal,uint64_t now_ms,bool failed,float match_radius_m);
#endif
