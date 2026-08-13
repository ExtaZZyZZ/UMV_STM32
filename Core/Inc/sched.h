
#ifndef CORE_SCHED_H
#define CORE_SCHED_H

#include <stdint.h>

void sched_init(void);

void sched_step(void);


uint32_t sched_max_lag(void);    
uint32_t sched_late_ticks(void); 

#endif 
