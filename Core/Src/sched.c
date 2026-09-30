
#include "sched.h"
#include "main.h"
#include "uart_link.h"
#include "telemetry.h"
#include "command.h"
#include "inputs.h"
#include "outputs.h"
#include "axis.h"
#include "encoder.h"
#include "wdg.h"
static uint32_t serviced;     
static uint32_t max_lag;      
static uint32_t late_ticks;   

typedef struct {
    uint16_t divisor;
    uint16_t offset;
    void (*step)(void);
} sched_task_t;

static const sched_task_t TASKS[] = {
   
    {  1u,    0u, uart_link_task     },  // частота работы modbus 1000 Гц
    {  1u,    0u, command_pulse_step },  // снятие импульса замка, 1000 Гц
    {  1u,    0u, axis_failsafe_step },  // снятие моста при потере связи, 1000 Гц
    {  1u,    0u, inputs_step        },  // дребезг концевиков, 1000 Гц
    {  1u,    0u, encoder_step       },  // обмен с энкодером, 1000 Гц
    {  5u,    2u, axis_step          },  // автоматы осей, 200 Гц
    { 20u,    0u, command_step       },  // исполнение команд, 50 Гц
    { 20u,    5u, outputs_step       },  // раскладка маски по ногам, 50 Гц
    { 20u,   10u, telemetry_step     },  // обновление телеметрии, 50 Гц
    { 100u,  15u, wdg_step           },  // перезапуск вотчдога 10 Гц
};

#define TASK_COUNT (sizeof(TASKS) / sizeof(TASKS[0]))

static void sched_dispatch(uint32_t tick)
{
    for (unsigned i = 0; i < TASK_COUNT; i++) {
        const sched_task_t *t = &TASKS[i];
        if ((tick % t->divisor) == t->offset) {
            t->step();
        }
    }
}

void sched_init(void)
{
    serviced   = HAL_GetTick();
    max_lag    = 0u;
    late_ticks = 0u;
}

void sched_step(void)
{
    const uint32_t lag = HAL_GetTick() - serviced;
    if (lag == 0u) {
        return;                 
    }

    if (lag > max_lag)  { max_lag = lag; }
    if (lag > 1u)       { late_ticks++;  }

    serviced++;
    sched_dispatch(serviced);
}

uint32_t sched_max_lag(void)    { return max_lag; }
uint32_t sched_late_ticks(void) { return late_ticks; }