#ifndef APP_RELEASE_H
#define APP_RELEASE_H

#include <stdint.h>

void release_init(void);
void release_step(void);
uint16_t release_cmd_start(uint16_t unit);

#endif /* APP_RELEASE_H */