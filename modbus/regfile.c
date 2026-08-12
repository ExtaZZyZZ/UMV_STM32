#include "regfile.h"
#include <string.h>

static uint16_t input_regs[IREG_COUNT];     // 22 числа телеметрии
static uint16_t holding_regs[HREG_COUNT];   //  5 чисел команд

void regfile_init(void) 
{
	memset(input_regs, 0, sizeof(input_regs));
	memset(holding_regs, 0, sizeof(holding_regs));
	input_regs[IREG_PROTO_VER] = PROTOCOL_VERSION;
	input_regs[IREG_LAYOUT_CRC] = LAYOUT_CRC;

}

bool regfile_read_input(uint16_t addr, uint16_t *out)
{
    if (addr >= IREG_COUNT) {
        return false;
    } 
    *out = input_regs[addr];
    return true;
}

bool regfile_read_holding(uint16_t addr, uint16_t *out)
{
    if (addr >= HREG_COUNT) {
        return false;
    }
    *out = holding_regs[addr];
    return true;
}

bool regfile_write_holding(uint16_t addr, uint16_t value)
{
    if (addr >= HREG_COUNT){
        return false;
    }
    holding_regs[addr] = value;
    return true;
}

void regfile_set_input(ireg_t reg, uint16_t value)
{
	input_regs[reg] = value;
}

uint16_t regfile_get_holding(hreg_t reg)
{
	return holding_regs[reg];
}
