#include "occupancy_grid.h"
#include <math.h>
#include <string.h>

static size_t cell_index(const OccupancyGrid *g, int x, int y) {
  return (size_t)y * g->width + (size_t)x;
}

GridStatus occupancy_grid_validate(const OccupancyGrid *g, uint64_t now_ms, uint64_t max_age_ms) {
  if (!g || !g->cells || !g->valid) return GRID_INVALID_POINTER;
  size_t count = (size_t)g->width * g->height;
  if (!g->width || !g->height || count > AUTONOMY_MAX_CELLS) return GRID_INVALID_DIMENSIONS;
  if (!isfinite(g->resolution_m) || g->resolution_m <= 0.0f || g->resolution_m > 2.0f)
    return GRID_INVALID_RESOLUTION;
  if (!isfinite(g->origin_x_m) || !isfinite(g->origin_y_m)) return GRID_INVALID_ORIGIN;
  if (g->frame != FRAME_LOCAL_NED && g->frame != FRAME_LOCAL_ENU) return GRID_INVALID_FRAME;
  if (!g->timestamp_ms || now_ms < g->timestamp_ms || now_ms - g->timestamp_ms > max_age_ms)
    return GRID_STALE;
  for (size_t i = 0; i < count; ++i) {
    if (g->cells[i] < CELL_UNKNOWN || g->cells[i] > CELL_ROBOT) return GRID_INVALID_POINTER;
  }
  return GRID_OK;
}

bool occupancy_grid_in_bounds(const OccupancyGrid *g, int x, int y) {
  return g && x >= 0 && y >= 0 && x < (int)g->width && y < (int)g->height;
}

bool occupancy_grid_world_to_cell(const OccupancyGrid *g, float wx, float wy, int *x, int *y) {
  if (!g || !x || !y || !isfinite(wx) || !isfinite(wy) ||
      !isfinite(g->resolution_m) || g->resolution_m <= 0.0f) return false;
  int cx = (int)floorf((wx - g->origin_x_m) / g->resolution_m);
  int cy = (int)floorf((wy - g->origin_y_m) / g->resolution_m);
  if (!occupancy_grid_in_bounds(g, cx, cy)) return false;
  *x = cx; *y = cy;
  return true;
}

bool occupancy_grid_cell_to_world(const OccupancyGrid *g, int x, int y, float *wx, float *wy) {
  if (!g || !wx || !wy || !occupancy_grid_in_bounds(g, x, y)) return false;
  *wx = g->origin_x_m + ((float)x + 0.5f) * g->resolution_m;
  *wy = g->origin_y_m + ((float)y + 0.5f) * g->resolution_m;
  return isfinite(*wx) && isfinite(*wy);
}

CellState occupancy_grid_get(const OccupancyGrid *g, int x, int y) {
  if (!occupancy_grid_in_bounds(g, x, y)) return CELL_OCCUPIED;
  CellState v = g->cells[cell_index(g, x, y)];
  return v >= CELL_UNKNOWN && v <= CELL_ROBOT ? v : CELL_OCCUPIED;
}

bool occupancy_grid_set(OccupancyGrid *g, int x, int y, CellState v) {
  if (!occupancy_grid_in_bounds(g, x, y) || v < CELL_UNKNOWN || v > CELL_ROBOT) return false;
  g->cells[cell_index(g, x, y)] = v;
  return true;
}

bool occupancy_grid_is_plannable(CellState v) {
  return v == CELL_FREE || v == CELL_FRONTIER || v == CELL_ROBOT;
}

float occupancy_grid_clearance_m(const OccupancyGrid *g, int cx, int cy, float limit_m) {
  if (!g || !occupancy_grid_in_bounds(g, cx, cy) || !isfinite(limit_m) || limit_m <= 0.0f)
    return 0.0f;
  int radius = (int)ceilf(limit_m / g->resolution_m);
  float best = limit_m;
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      int x = cx + dx, y = cy + dy;
      CellState v = occupancy_grid_get(g, x, y);
      if (v != CELL_OCCUPIED && v != CELL_INFLATED_OBSTACLE &&
          occupancy_grid_in_bounds(g, x, y)) continue;
      float d = hypotf((float)dx, (float)dy) * g->resolution_m;
      if (d < best) best = d;
    }
  }
  return best;
}

bool occupancy_grid_inflate(const OccupancyGrid *src, OccupancyGrid *dst, float radius_m) {
  if (!src || !dst || !src->cells || !dst->cells || src->cells == dst->cells ||
      src->width != dst->width || src->height != dst->height ||
      !isfinite(radius_m) || radius_m < src->resolution_m) return false;
  size_t count = (size_t)src->width * src->height;
  if (count > AUTONOMY_MAX_CELLS) return false;
  memcpy(dst->cells, src->cells, count * sizeof(CellState));
  dst->width = src->width; dst->height = src->height;
  dst->resolution_m = src->resolution_m;
  dst->origin_x_m = src->origin_x_m; dst->origin_y_m = src->origin_y_m;
  dst->timestamp_ms = src->timestamp_ms; dst->sequence = src->sequence;
  dst->frame = src->frame; dst->valid = src->valid; dst->synthetic = src->synthetic;
  int radius = (int)ceilf(radius_m / src->resolution_m);
  float radius_cells = radius_m / src->resolution_m;
  for (int y = 0; y < (int)src->height; ++y) {
    for (int x = 0; x < (int)src->width; ++x) {
      if (occupancy_grid_get(src, x, y) != CELL_OCCUPIED) continue;
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          if (hypotf((float)dx, (float)dy) > radius_cells) continue;
          int nx = x + dx, ny = y + dy;
          if (!occupancy_grid_in_bounds(dst, nx, ny)) continue;
          size_t i = cell_index(dst, nx, ny);
          if (dst->cells[i] != CELL_OCCUPIED) dst->cells[i] = CELL_INFLATED_OBSTACLE;
        }
      }
    }
  }
  return true;
}
