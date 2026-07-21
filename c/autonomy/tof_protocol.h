#ifndef TOF_PROTOCOL_H
#define TOF_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define TOF_SENSOR_COUNT 4u
#define TOF_ZONES_PER_SENSOR 64u
#define TOF_LEGACY_A5_FRAME_SIZE 518u
#define TOF_CONTROL_A6_FRAME_SIZE 7u
#define TOF_INVALID_MM UINT16_MAX

typedef enum {
  TOF_PROTOCOL_NONE = 0,
  /* Exact unversioned 518-byte frame emitted by arduino/tof_esp32.ino. */
  TOF_PROTOCOL_LEGACY_A5_V0
} TofProtocol;

typedef enum {
  TOF_DIRECTION_FRONT = 0,
  TOF_DIRECTION_RIGHT,
  TOF_DIRECTION_BACK,
  TOF_DIRECTION_LEFT
} TofDirection;

typedef struct {
  TofProtocol protocol;
  uint32_t sensor_uptime_ms;
  uint32_t sequence;
  uint64_t received_monotonic_ms;
  uint16_t range_mm[TOF_SENSOR_COUNT][TOF_ZONES_PER_SENSOR];
  uint16_t valid_zone_count;
  bool valid;
} TofFrame;

const char *tof_protocol_name(TofProtocol protocol);
TofProtocol tof_protocol_from_name(const char *name);

#endif
