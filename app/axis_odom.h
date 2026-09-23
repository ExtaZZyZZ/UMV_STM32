
#ifndef APP_AXIS_ODOM_H
#define APP_AXIS_ODOM_H

#include <stdint.h>


static inline int32_t odom_step_to_x10(uint16_t step, uint16_t step_mm)
{
    return (int32_t)((uint32_t)step * step_mm * 10u);
}


static inline uint16_t odom_x10_to_step(int32_t x10, uint16_t step_mm)
{
    const int32_t d = (int32_t)step_mm * 10;

    if (x10 <= 0 || d <= 0) {
        return 0u;
    }
    return (uint16_t)((x10 + d / 2) / d);
}


static inline int32_t odom_x10_from_ms(uint32_t run_ms, uint32_t step_travel_ms,
                                       uint16_t step_mm)
{
    if (step_travel_ms == 0u) {
        return 0;
    }
    return (int32_t)((run_ms * step_mm * 10u) / step_travel_ms);
}


static inline uint32_t odom_ms_from_x10(int32_t x10, uint32_t step_travel_ms,
                                        uint16_t step_mm)
{
    if (x10 <= 0 || step_mm == 0u) {
        return 0u;
    }
    return ((uint32_t)x10 * step_travel_ms) / ((uint32_t)step_mm * 10u);
}


#endif /* APP_AXIS_ODOM_H */
