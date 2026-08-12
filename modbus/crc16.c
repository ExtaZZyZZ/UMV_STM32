/*
 * crc16.c
  *  Created on: 12 авг. 2026 г.
 *      Author: i.yakovlev
 */

#include "crc16.h"

uint16_t crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint16_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x0001u) ? (crc >> 1) ^ 0xA001u : (crc >> 1);
        }
    }
    return crc;
}

