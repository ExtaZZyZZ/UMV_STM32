#ifndef CORE_WDG_H
#define CORE_WDG_H
#include <stdint.h>

void wdg_init(void);
uint8_t wdg_tripped(void);
void wdg_step(void);
#endif