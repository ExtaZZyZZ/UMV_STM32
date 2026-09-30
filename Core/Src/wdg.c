#include "wdg.h"
#include "main.h"
static uint8_t status_wdg = 0;
extern IWDG_HandleTypeDef hiwdg;
void wdg_init(void) 
{
    status_wdg = __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST);
    __HAL_RCC_CLEAR_RESET_FLAGS();
    __HAL_DBGMCU_FREEZE_IWDG();
}
uint8_t wdg_tripped() 
{
    return status_wdg;
}
void wdg_step(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}