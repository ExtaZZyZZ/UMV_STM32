#ifndef MODBUS_REGFILE_H
#define MODBUS_REGFILE_H

#include <stdint.h>
#include <stdbool.h>
#include "board_registers.h"

void regfile_init(void);

bool regfile_read_input(uint16_t addr, uint16_t *out);
bool regfile_read_holding(uint16_t addr, uint16_t *out);
bool regfile_write_holding(uint16_t addr, uint16_t value);

void     regfile_set_input(ireg_t reg, uint16_t value);
uint16_t regfile_get_holding(hreg_t reg);

#endif /* MODBUS_REGFILE_H */
