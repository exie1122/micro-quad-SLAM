#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include "autonomy_types.h"

typedef enum {
  GRID_OK = 0,
  GRID_INVALID_POINTER,
  GRID_INVALID_DIMENSIONS,
  GRID_INVALID_RESOLUTION,
  GRID_INVALID_ORIGIN,
  GRID_INVALID_FRAME,
  GRID_STALE
} GridStatus;

GridStatus occupancy_grid_validate(const OccupancyGrid *grid, uint64_t now_ms, uint64_t max_age_ms);
bool occupancy_grid_in_bounds(const OccupancyGrid *grid, int x, int y);
bool occupancy_grid_world_to_cell(const OccupancyGrid *grid, float world_x_m, float world_y_m,
                                  int *cell_x, int *cell_y);
bool occupancy_grid_cell_to_world(const OccupancyGrid *grid, int cell_x, int cell_y,
                                  float *world_x_m, float *world_y_m);
CellState occupancy_grid_get(const OccupancyGrid *grid, int x, int y);
bool occupancy_grid_set(OccupancyGrid *grid, int x, int y, CellState value);
bool occupancy_grid_is_plannable(CellState value);
float occupancy_grid_clearance_m(const OccupancyGrid *grid, int cell_x, int cell_y,
                                 float search_limit_m);
bool occupancy_grid_inflate(const OccupancyGrid *source, OccupancyGrid *destination,
                            float clearance_radius_m);

#endif
