#ifndef MODBUS_CRC16_H
#define MODBUS_CRC16_H

#include <stdint.h>

uint16_t crc16_modbus(const uint8_t *data, uint16_t len);

#endif /* MODBUS_CRC16_H */