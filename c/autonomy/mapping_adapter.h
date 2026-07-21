#ifndef MAPPING_ADAPTER_H
#define MAPPING_ADAPTER_H

#include "autonomy_types.h"
#include "recorded_log.h"

typedef struct {OccupancyGrid grid;uint32_t rejected_records;} MappingAdapter;

bool mapping_adapter_init(MappingAdapter *adapter,CellState *storage,uint16_t width,uint16_t height,
                          float resolution_m,float center_x_m,float center_y_m,bool synthetic);
bool mapping_adapter_apply_sclog3(MappingAdapter *adapter,const Sclog3Record *record);
bool mapping_adapter_live_available(void);

#endif
