#include "command_smoother.h"
#include <math.h>
#include <string.h>

static float clampf(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}
static bool finite_command(const MotionCommand *c){return c&&isfinite(c->vx_mps)&&isfinite(c->vy_mps)&&isfinite(c->vz_mps)&&isfinite(c->yaw_rate_dps);}

bool command_limit_subgoal(GridPoint cur,GridPoint p,float max_step,GridPoint *out){
  if(!out||!isfinite(cur.x_m)||!isfinite(cur.y_m)||!isfinite(p.x_m)||!isfinite(p.y_m)||!isfinite(max_step)||max_step<=0.0f)return false;
  float dx=p.x_m-cur.x_m,dy=p.y_m-cur.y_m,d=hypotf(dx,dy);*out=p;
  if(d>max_step){float k=max_step/d;out->x_m=cur.x_m+dx*k;out->y_m=cur.y_m+dy*k;}return true;
}

bool command_smoother_update(CommandSmoother *s,const AutonomyConfig *c,const MotionCommand *req,
                             uint64_t now,bool horizontal,bool landing,MotionCommand *out,bool *was_clamped){
  if(!s||!c||!out||!was_clamped){return false;} *was_clamped=false;
  if(!finite_command(req)){memset(out,0,sizeof(*out));out->source_timestamp_ms=now;out->valid=false;return false;}
  float dt=s->last_update_ms&&now>=s->last_update_ms?(float)(now-s->last_update_ms)*0.001f:1.0f/c->command_rate_hz;
  float max_dt=2.0f/c->command_rate_hz;if(dt>max_dt){dt=max_dt;*was_clamped=true;}if(dt<0.001f)dt=0.001f;
  MotionCommand target=*req;
  bool stale=!req->source_timestamp_ms||now<req->source_timestamp_ms||
             now-req->source_timestamp_ms>(uint64_t)(c->command_timeout_s*1000.0f);
  if(stale||!horizontal){target.vx_mps=0.0f;target.vy_mps=0.0f;*was_clamped=true;}
  if(landing){target.vx_mps=0.0f;target.vy_mps=0.0f;target.yaw_rate_dps=0.0f;}
  float h=hypotf(target.vx_mps,target.vy_mps);
  if(h>c->max_horizontal_speed_mps){float k=c->max_horizontal_speed_mps/h;target.vx_mps*=k;target.vy_mps*=k;*was_clamped=true;}
  float vz=clampf(target.vz_mps,-c->max_vertical_speed_mps,c->max_vertical_speed_mps);
  float yaw=clampf(target.yaw_rate_dps,-c->max_yaw_rate_dps,c->max_yaw_rate_dps);
  if(vz!=target.vz_mps||yaw!=target.yaw_rate_dps) *was_clamped=true;
  target.vz_mps=vz;target.yaw_rate_dps=yaw;
  float max_delta=c->max_acceleration_mps2*dt;
  float dvx=target.vx_mps-s->last.vx_mps,dvy=target.vy_mps-s->last.vy_mps,dh=hypotf(dvx,dvy);
  if(dh>max_delta){float k=max_delta/dh;target.vx_mps=s->last.vx_mps+dvx*k;target.vy_mps=s->last.vy_mps+dvy*k;*was_clamped=true;}
  float dvz=clampf(target.vz_mps-s->last.vz_mps,-max_delta,max_delta);
  if(fabsf(dvz-(target.vz_mps-s->last.vz_mps))>1e-6f) *was_clamped=true;
  target.vz_mps=s->last.vz_mps+dvz;
  /* Safety states are an immediate horizontal stop, not a ramped setpoint. */
  if(!horizontal){target.vx_mps=0.0f;target.vy_mps=0.0f;}
  target.source_timestamp_ms=now;target.valid=true;*out=target;s->last=target;s->last_update_ms=now;return true;
}
