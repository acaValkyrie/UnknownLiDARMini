#include <Arduino.h>
#include <LovyanGFX.hpp>
#include "lgfx.hpp"
#include "lidar.hpp"

WaveshareRoundLCD lcd;

void setup(void) {
  lcd.init();

  lcd.setCursor(0,0);
  lcd.print("start!\n");

  LIDARSerial.begin(230400, SERIAL_8N1, 20, 21);

  delay(10);
}

void loop() {
  static State_t state;
  static uint32_t counter;
  static uint8_t payload[64];

  if (LIDARSerial.available())
  {
    uint8_t data = LIDARSerial.read();
    switch (state)
    {
      case STATE_WAIT_HEADER:
        if (data == header[0]) {
          counter++;
          payload[0] = data;
          state = STATE_READ_HEADER;
        } else {
          //printf("?? (%02X) Please do LiDAR power cycle\n", data);
          LIDARSerial.flush();
        }
        break;
      case STATE_READ_HEADER:
        if (data == header[counter]) {
          payload[counter] = data;
          counter++;
          if (counter == sizeof(header)) {
            state = STATE_READ_PAYLOAD;
          }
        } else {
          counter = 0;
          state = STATE_WAIT_HEADER;
        }
        break;
      case STATE_READ_PAYLOAD:
        payload[counter] = data;
        counter++;
        if (counter == sizeof(LidarPacket_t)) {
          state = STATE_READ_DONE;
        }
        break;
      case STATE_READ_DONE:
        LidarPacket_t* packet = (LidarPacket_t*)payload;
        {
          uint16_t degree_begin;
          uint16_t degree_end;
          degree_begin = convertDegree(packet->angle_begin);
          degree_end = convertDegree(packet->angle_end);
          if ((degree_begin < 360) && (degree_end < 360)) {
            printf("%3drpm %5d - %5d\n", convertSpeed(packet->rotation_speed), convertDegree(packet->angle_begin), convertDegree(packet->angle_end));
            uint16_t map[16];
            uint16_t distances[16];
            remapDegrees(degree_begin, degree_end, map);
            distances[0] = packet->distance_0 & 0x3FFF;
            distances[1] = packet->distance_1 & 0x3FFF;
            distances[2] = packet->distance_2 & 0x3FFF;
            distances[3] = packet->distance_3 & 0x3FFF;
            distances[4] = packet->distance_4 & 0x3FFF;
            distances[5] = packet->distance_5 & 0x3FFF;
            distances[6] = packet->distance_6 & 0x3FFF;
            distances[7] = packet->distance_7 & 0x3FFF;
            distances[8] = packet->distance_8 & 0x3FFF;
            distances[9] = packet->distance_9 & 0x3FFF;
            distances[10] = packet->distance_10 & 0x3FFF;
            distances[11] = packet->distance_11 & 0x3FFF;
            distances[12] = packet->distance_12 & 0x3FFF;
            distances[13] = packet->distance_13 & 0x3FFF;
            distances[14] = packet->distance_14 & 0x3FFF;
            distances[15] = packet->distance_15 & 0x3FFF;
            plotDistanceMap(&lcd, map, distances);
          }
        }
        lcd.setCursor(0, 0);
        lcd.printf("Speed : %d rpm  \n", convertSpeed(packet->rotation_speed));
        state = STATE_WAIT_HEADER;
        counter = 0;
        state = STATE_WAIT_HEADER;
        break;
    }
  }
}
