#ifndef LIDAR_HPP
#define LIDAR_HPP

#include <Arduino.h>
#include "lgfx.hpp"

#define LIDARSerial Serial1

typedef struct
{
  uint16_t x;
  uint16_t y;
} Point_t;

typedef enum
{
  STATE_WAIT_HEADER = 0,
  STATE_READ_HEADER,
  STATE_READ_PAYLOAD,
  STATE_READ_DONE
} State_t;

typedef struct {
  uint8_t header0;
  uint8_t header1;
  uint8_t header2;
  uint8_t header3;
  uint16_t rotation_speed;
  uint16_t angle_begin;
  uint16_t distance_0;
  uint8_t reserved_0;
  uint16_t distance_1;
  uint8_t reserved_1;
  uint16_t distance_2;
  uint8_t reserved_2;
  uint16_t distance_3;
  uint8_t reserved_3;
  uint16_t distance_4;
  uint8_t reserved_4;
  uint16_t distance_5;
  uint8_t reserved_5;
  uint16_t distance_6;
  uint8_t reserved_6;
  uint16_t distance_7;
  uint8_t reserved_7;
  uint16_t distance_8;
  uint8_t reserved_8;
  uint16_t distance_9;
  uint8_t reserved_9;
  uint16_t distance_10;
  uint8_t reserved_10;
  uint16_t distance_11;
  uint8_t reserved_11;
  uint16_t distance_12;
  uint8_t reserved_12;
  uint16_t distance_13;
  uint8_t reserved_13;
  uint16_t distance_14;
  uint8_t reserved_14;
  uint16_t distance_15;
  uint8_t reserved_15;
  uint16_t angle_end;
  uint16_t crc;
} __attribute__((packed)) LidarPacket_t;

const uint8_t header[] = { 0x55, 0xaa, 0x23, 0x10 };

uint16_t convertDegree(uint16_t input)
{
  return (input - 40960) / 64;
}

uint16_t convertSpeed(uint16_t input)
{
  return input / 64;
}

void remapDegrees(uint16_t minAngle, uint16_t maxAngle, uint16_t *map)
{
  int16_t delta = maxAngle - minAngle;
  if (maxAngle < minAngle) {
    delta += 360;
  }

  if ((map == NULL) || (delta < 0)) {
    return;
  }
  for (int32_t cnt = 0; cnt < 16; cnt++)
  {
    map[cnt] = minAngle + (delta * cnt / 15);
    if (map[cnt] >= 360) {
      map[cnt] -= 360;
    }
  }
}

void plotDistanceMap(WaveshareRoundLCD* lcd, uint16_t* degrees, uint16_t* distances)
{
  int32_t i;
  uint32_t x, y;
  static Point_t pointCloud[360];      // 360度分の点群

  for (i = 0; i < 16; i++) {
    lcd->drawPixel(pointCloud[degrees[i]].x, pointCloud[degrees[i]].y, lcd->color888(0,0,0));
    if (distances[i] < 10000) {
      x = cos((1.f * PI * degrees[i]) / 180.0f) * (distances[i] / 20.0f) + lcd->width() / 2;
      y = sin((1.f * PI * degrees[i]) / 180.0f) * (distances[i] / 20.0f) + lcd->height() / 2;

      lcd->drawPixel(x, y, lcd->color888(255,255,255));

      pointCloud[degrees[i]].x = x;
      pointCloud[degrees[i]].y = y;

    }
  }
}

#endif