#include "frontier_detector.h"
#include "occupancy_grid.h"
#include <float.h>
#include <math.h>
#include <string.h>

static uint32_t cell_index(const OccupancyGrid *g, int x, int y) {
  return (uint32_t)y*g->width+(uint32_t)x;
}

static bool is_frontier(const OccupancyGrid *g,int x,int y) {
  if (occupancy_grid_get(g,x,y)!=CELL_FREE) return false;
  for(int dy=-1;dy<=1;++dy) for(int dx=-1;dx<=1;++dx) {
    if ((!dx&&!dy)||!occupancy_grid_in_bounds(g,x+dx,y+dy)) continue;
    if (occupancy_grid_get(g,x+dx,y+dy)==CELL_UNKNOWN) return true;
  }
  return false;
}

static void reject_cluster(FrontierSet *r,uint32_t id,FrontierRejection why) {
  if(r->rejected_count<AUTONOMY_MAX_REJECTIONS) {
    r->rejected[r->rejected_count].cluster_id=id;
    r->rejected[r->rejected_count++].reason=why;
  } else r->truncated=true;
}

bool frontier_detect(const OccupancyGrid *g,const AutonomyConfig *c,uint64_t now,
                     FrontierWorkspace *w,FrontierSet *r) {
  if (!g||!c||!w||!r) return false;
  memset(r,0,sizeof(*r));
  size_t cells=(size_t)g->width*g->height;
  if(cells>AUTONOMY_MAX_CELLS || occupancy_grid_validate(g,now,(uint64_t)(c->map_max_age_s*1000.0f))!=GRID_OK)
    return false;
  memset(w->visited,0,cells);
  uint32_t cluster_id=0;
  float required=autonomy_required_clearance_m(c,g->resolution_m);
  static const int nx8[8]={1,1,0,-1,-1,-1,0,1};
  static const int ny8[8]={0,1,1,1,0,-1,-1,-1};
  for(int y=0;y<(int)g->height;++y) for(int x=0;x<(int)g->width;++x) {
    uint32_t first=cell_index(g,x,y);
    if(w->visited[first]||!is_frontier(g,x,y)) continue;
    cluster_id++;
    uint32_t head=0,tail=0; w->queue[tail++]=first; w->visited[first]=1;
    double sumx=0.0,sumy=0.0; uint32_t unknown_edges=0;
    while(head<tail) {
      uint32_t cur=w->queue[head++]; int cx=(int)(cur%g->width),cy=(int)(cur/g->width);
      sumx+=cx; sumy+=cy;
      for(int d=0;d<8;++d) {
        int ax=cx+nx8[d],ay=cy+ny8[d];
        if(!occupancy_grid_in_bounds(g,ax,ay)) continue;
        if(occupancy_grid_get(g,ax,ay)==CELL_UNKNOWN) unknown_edges++;
        uint32_t ai=cell_index(g,ax,ay);
        if(!w->visited[ai]&&is_frontier(g,ax,ay)) {
          if(tail>=cells){r->truncated=true;break;}
          w->visited[ai]=1;w->queue[tail++]=ai;
        }
      }
    }
    if(tail<c->minimum_frontier_cells){reject_cluster(r,cluster_id,FRONTIER_REJECT_TOO_SMALL);continue;}
    float centroid_x=(float)(sumx/(double)tail),centroid_y=(float)(sumy/(double)tail);
    float best_dist=FLT_MAX,best_clear=0.0f;int best_x=-1,best_y=-1;
    for(uint32_t i=0;i<tail;++i){
      int cx=(int)(w->queue[i]%g->width),cy=(int)(w->queue[i]/g->width);
      float clear=occupancy_grid_clearance_m(g,cx,cy,required+g->resolution_m);
      if(clear+1e-5f<required)continue;
      float d=hypotf((float)cx-centroid_x,(float)cy-centroid_y);
      if(d<best_dist-1e-6f || (fabsf(d-best_dist)<1e-6f &&
          (cy<best_y || (cy==best_y&&cx<best_x)))){
        best_dist=d;best_clear=clear;best_x=cx;best_y=cy;
      }
    }
    if(best_x<0){reject_cluster(r,cluster_id,FRONTIER_REJECT_NO_SAFE_REPRESENTATIVE);continue;}
    if(r->accepted_count>=AUTONOMY_MAX_FRONTIERS){r->truncated=true;continue;}
    FrontierCandidate *f=&r->accepted[r->accepted_count++]; memset(f,0,sizeof(*f));
    f->id=cluster_id;f->cell_count=tail;f->clearance_m=best_clear;
    occupancy_grid_cell_to_world(g,(int)lroundf(centroid_x),(int)lroundf(centroid_y),
                                 &f->centroid.x_m,&f->centroid.y_m);
    occupancy_grid_cell_to_world(g,best_x,best_y,&f->goal.x_m,&f->goal.y_m);
    f->expected_unknown_gain=(float)unknown_edges;
  }
  return true;
}

const char *frontier_rejection_name(FrontierRejection r){
  switch(r){
    case FRONTIER_ACCEPTED:return "accepted";
    case FRONTIER_REJECT_TOO_SMALL:return "too_small";
    case FRONTIER_REJECT_CLEARANCE:return "insufficient_clearance";
    case FRONTIER_REJECT_NO_SAFE_REPRESENTATIVE:return "no_safe_representative";
    case FRONTIER_REJECT_OUT_OF_BOUNDS:return "out_of_bounds";
    case FRONTIER_REJECT_UNREACHABLE:return "unreachable";
    case FRONTIER_REJECT_PATH_TOO_LONG:return "path_too_long";
    case FRONTIER_REJECT_PATH_TOO_COMPLEX:return "path_too_complex";
    case FRONTIER_REJECT_STALE_MAP:return "stale_map";
    default:return "unknown";
  }
}
