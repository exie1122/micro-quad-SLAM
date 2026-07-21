#include "mapping_adapter.h"
#include "occupancy_grid.h"
#include <math.h>
#include <string.h>

static bool set_world(MappingAdapter*a,float x,float y,CellState v){int cx,cy;if(!occupancy_grid_world_to_cell(&a->grid,x,y,&cx,&cy))return false;CellState old=occupancy_grid_get(&a->grid,cx,cy);if(v==CELL_FREE&&(old==CELL_OCCUPIED||old==CELL_INFLATED_OBSTACLE))return true;return occupancy_grid_set(&a->grid,cx,cy,v);}
static float column_distance(const uint8_t *grid,int col){uint16_t values[8];int n=0;for(int row=0;row<8;++row){size_t z=(size_t)(row*8+col)*2;uint16_t mm=(uint16_t)grid[z]|((uint16_t)grid[z+1]<<8);if(mm>=100&&mm<=4000)values[n++]=mm;}if(!n)return NAN;for(int i=1;i<n;++i){uint16_t v=values[i];int j=i-1;while(j>=0&&values[j]>v){values[j+1]=values[j];--j;}values[j+1]=v;}return (float)values[n/2]*0.001f;}
bool mapping_adapter_init(MappingAdapter*a,CellState*storage,uint16_t w,uint16_t h,float res,float cx,float cy,bool synth){
  if(!a||!storage||!w||!h||(size_t)w*h>AUTONOMY_MAX_CELLS||!isfinite(res)||res<=0.0f) return false;
  memset(a,0,sizeof(*a));
  a->grid.cells=storage;a->grid.width=w;a->grid.height=h;a->grid.resolution_m=res;a->grid.origin_x_m=cx-0.5f*w*res;a->grid.origin_y_m=cy-0.5f*h*res;a->grid.frame=FRAME_LOCAL_NED;a->grid.synthetic=synth;a->grid.valid=true;
  for(size_t i=0;i<(size_t)w*h;++i) storage[i]=CELL_UNKNOWN;
  return true;
}
bool mapping_adapter_apply_sclog3(MappingAdapter*a,const Sclog3Record*r){
  if(!a||!r||r->magic!=SCLOG3_MAGIC||!isfinite(r->x_m)||!isfinite(r->y_m)||!isfinite(r->yaw_deg)||!r->host_ms){if(a)a->rejected_records++;return false;}
  const float base[4]={0.0f,90.0f,180.0f,-90.0f};
  for(int sensor=0;sensor<4;++sensor)for(int col=0;col<8;++col){
    float d=column_distance(r->grid_raw+sensor*128,col);if(!isfinite(d))continue;
    float offset=(((float)col+0.5f)/8.0f-0.5f)*45.0f;float yaw=(r->yaw_deg+base[sensor]+offset)*0.01745329252f;
    float free_limit=d-0.08f;if(free_limit<0.0f)free_limit=0.0f;
    for(float s=0.0f;s<=free_limit;s+=a->grid.resolution_m*0.5f)set_world(a,r->x_m+cosf(yaw)*s,r->y_m+sinf(yaw)*s,CELL_FREE);
    if(d<=3.5f)set_world(a,r->x_m+cosf(yaw)*d,r->y_m+sinf(yaw)*d,CELL_OCCUPIED);
  }
  a->grid.timestamp_ms=r->host_ms;a->grid.sequence++;return true;
}
bool mapping_adapter_live_available(void){return false;}
