#include "recorded_log.h"
#include <stddef.h>
#include <string.h>

_Static_assert(sizeof(Sclog3Record)==SCLOG3_RECORD_SIZE,"SCLOG3 layout must remain 642 bytes");

ReplayStatus replay_reader_open(ReplayReader*r,const char*path){
  if(!r||!path) return REPLAY_IO_ERROR;
  memset(r,0,sizeof(*r));
  r->file=fopen(path,"rb");if(!r->file)return REPLAY_IO_ERROR;
  unsigned char header[7];if(fread(header,1,7,r->file)!=7||memcmp(header,"SCLOG3\n",7)!=0){fclose(r->file);r->file=NULL;return REPLAY_BAD_HEADER;}
  r->offset=7;snprintf(r->path,sizeof(r->path),"%s",path);return REPLAY_OK;
}

ReplayStatus replay_reader_next(ReplayReader*r,Sclog3Record*out){
  if(!r||!r->file||!out)return REPLAY_IO_ERROR;
  unsigned char buf[SCLOG3_RECORD_SIZE];size_t n=fread(buf,1,sizeof(buf),r->file);
  if(n==0)return feof(r->file)?REPLAY_EOF:REPLAY_IO_ERROR;
  if(n<sizeof(buf)){r->warnings++;return REPLAY_EOF;}
  memcpy(out,buf,sizeof(*out));r->offset+=sizeof(buf);
  if(out->magic!=SCLOG3_MAGIC){
    r->warnings++;
    /* Bounded byte-wise resynchronization. No incompatible bytes are interpreted. */
    unsigned char window[4];memcpy(window,buf,4);
    for(uint32_t scanned=0;scanned<1024u*1024u;++scanned){
      int ch=fgetc(r->file);if(ch==EOF)return REPLAY_EOF;r->offset++;
      memmove(window,window+1,3);window[3]=(unsigned char)ch;
      uint32_t magic;memcpy(&magic,window,4);
      if(magic==SCLOG3_MAGIC){
        memcpy(buf,window,4);
        if(fread(buf+4,1,sizeof(buf)-4,r->file)!=sizeof(buf)-4)return REPLAY_EOF;
        r->offset+=sizeof(buf)-4;memcpy(out,buf,sizeof(*out));break;
      }
    }
    if(out->magic!=SCLOG3_MAGIC)return REPLAY_IO_ERROR;
  }
  if(r->previous_timestamp_ms&&out->host_ms<r->previous_timestamp_ms)r->timestamp_resets++;
  r->previous_timestamp_ms=out->host_ms;r->records++;return REPLAY_OK;
}
void replay_reader_close(ReplayReader*r){if(r&&r->file)fclose(r->file);if(r)r->file=NULL;}
