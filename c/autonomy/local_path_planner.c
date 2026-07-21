#include "local_path_planner.h"
#include "occupancy_grid.h"
#include <math.h>
#include <string.h>

static uint32_t idx(const OccupancyGrid *g, int x, int y) { return (uint32_t)y * g->width + (uint32_t)x; }

PlanStatus local_path_plan(const OccupancyGrid *g, GridPoint start, GridPoint goal,
                           const AutonomyConfig *c, PlannerWorkspace *w, LocalPath *out) {
  if (out) memset(out, 0, sizeof(*out));
  if (!g || !c || !w || !out || !g->cells || !autonomy_config_validate(c)) return PLAN_INVALID_INPUT;
  size_t cells = (size_t)g->width * g->height;
  if (!cells || cells > AUTONOMY_MAX_CELLS) return PLAN_INVALID_INPUT;
  int sx, sy, gx, gy;
  if (!occupancy_grid_world_to_cell(g, start.x_m, start.y_m, &sx, &sy) ||
      !occupancy_grid_world_to_cell(g, goal.x_m, goal.y_m, &gx, &gy)) return PLAN_INVALID_INPUT;
  if (!occupancy_grid_is_plannable(occupancy_grid_get(g, sx, sy))) return PLAN_START_BLOCKED;
  if (!occupancy_grid_is_plannable(occupancy_grid_get(g, gx, gy))) return PLAN_GOAL_BLOCKED;
  memset(w->visited, 0, cells);
  for (size_t i = 0; i < cells; ++i) w->parent[i] = -1;
  uint32_t head = 0, tail = 0, explored = 0;
  uint32_t s = idx(g, sx, sy), target = idx(g, gx, gy);
  w->queue[tail++] = s; w->visited[s] = 1;
  static const int dx[4] = {1,0,-1,0};
  static const int dy[4] = {0,1,0,-1};
  bool found = false;
  while (head < tail) {
    uint32_t cur = w->queue[head++];
    if (++explored > c->planner_max_nodes) return PLAN_SEARCH_LIMIT;
    if (cur == target) { found = true; break; }
    int x = (int)(cur % g->width), y = (int)(cur / g->width);
    if (hypotf((float)(x-sx), (float)(y-sy)) * g->resolution_m > c->planner_search_radius_m) continue;
    for (int d = 0; d < 4; ++d) {
      int nx=x+dx[d], ny=y+dy[d];
      if (!occupancy_grid_in_bounds(g,nx,ny)) continue;
      uint32_t ni=idx(g,nx,ny);
      if (w->visited[ni] || !occupancy_grid_is_plannable(occupancy_grid_get(g,nx,ny))) continue;
      w->visited[ni]=1; w->parent[ni]=(int32_t)cur;
      if (tail >= cells) return PLAN_SEARCH_LIMIT;
      w->queue[tail++]=ni;
    }
  }
  if (!found) return PLAN_NO_PATH;
  uint32_t rev[AUTONOMY_MAX_PATH_POINTS];
  uint16_t count=0; int32_t cur=(int32_t)target;
  while (cur >= 0) {
    if (count >= c->planner_max_path_points || count >= AUTONOMY_MAX_PATH_POINTS) return PLAN_PATH_LIMIT;
    rev[count++]=(uint32_t)cur;
    if ((uint32_t)cur == s) break;
    cur=w->parent[cur];
  }
  if (!count || rev[count-1] != s) return PLAN_NO_PATH;
  float clearance_limit = autonomy_required_clearance_m(c, g->resolution_m) + g->resolution_m;
  out->minimum_clearance_m = clearance_limit;
  int prev_dx=0, prev_dy=0;
  for (uint16_t i=0; i<count; ++i) {
    uint32_t ci=rev[count-1-i]; int x=(int)(ci%g->width), y=(int)(ci/g->width);
    occupancy_grid_cell_to_world(g,x,y,&out->points[i].x_m,&out->points[i].y_m);
    float clear=occupancy_grid_clearance_m(g,x,y,clearance_limit);
    if (clear < out->minimum_clearance_m) out->minimum_clearance_m=clear;
    if (i>0) {
      out->distance_m += g->resolution_m;
      int ddx=x-(int)(rev[count-i]%g->width), ddy=y-(int)(rev[count-i]/g->width);
      if (i>1 && (ddx!=prev_dx || ddy!=prev_dy)) out->turn_count++;
      prev_dx=ddx; prev_dy=ddy;
    }
  }
  out->count=count;
  /* Clearance is enforced by planning only on the separately inflated grid.
     Requiring the full inflation radius again here would double-count it. */
  if (out->distance_m > c->planner_max_path_m) return PLAN_TOO_LONG;
  if (out->turn_count > c->planner_max_turns) return PLAN_TOO_COMPLEX;
  out->valid=true;
  return PLAN_OK;
}

bool local_path_segment_valid(const OccupancyGrid *g, GridPoint a, GridPoint b) {
  if (!g || !g->cells) return false;
  float distance=hypotf(b.x_m-a.x_m,b.y_m-a.y_m);
  int steps=(int)ceilf(distance/(g->resolution_m*0.5f)); if (steps<1) steps=1;
  for (int i=0;i<=steps;++i) {
    float u=(float)i/(float)steps; int x,y;
    if (!occupancy_grid_world_to_cell(g,a.x_m+u*(b.x_m-a.x_m),a.y_m+u*(b.y_m-a.y_m),&x,&y) ||
        !occupancy_grid_is_plannable(occupancy_grid_get(g,x,y))) return false;
  }
  return true;
}

bool local_path_next_subgoal(const LocalPath *p, GridPoint current, float max_step, GridPoint *out) {
  if (!p || !p->valid || !p->count || !out || !isfinite(max_step) || max_step<=0.0f) return false;
  GridPoint best=p->points[0]; bool found=false;
  for (uint16_t i=0;i<p->count;++i) {
    float d=hypotf(p->points[i].x_m-current.x_m,p->points[i].y_m-current.y_m);
    if (d<=max_step+1e-5f) { best=p->points[i]; found=true; } else break;
  }
  if (!found) return false;
  *out=best; return true;
}
