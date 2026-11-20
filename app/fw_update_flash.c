#include "fw_update_flash.h"
#include "sysflash/sysflash.h"
#include "stm32g4xx_hal.h"

#define BANK_BYTES 0x40000u

_Static_assert(BOARD_FLASH_BASE == FLASH_BASE && BOARD_PAGE_SIZE == FLASH_PAGE_SIZE,
               "Unexpected STM32 flash geometry");
_Static_assert(BOARD_SLOT1_OFF == BANK_BYTES &&
               BOARD_SLOT1_SIZE >= BOARD_PAGE_SIZE &&
               BOARD_SLOT1_SIZE % BOARD_PAGE_SIZE == 0u &&
               BOARD_SLOT1_SIZE <= BANK_BYTES,
               "Secondary slot must contain whole pages in bank 2");

static volatile enum {
    ERASE_UNINITIALIZED,
    ERASE_IDLE,
    ERASE_ACTIVE,
    ERASE_DONE,
    ERASE_FAILED
} erase_state;

static bool layout_supported(void)
{
    return __HAL_RCC_SYSCFG_IS_CLK_ENABLED() &&
           FLASH_SIZE == 2u * BANK_BYTES &&
           (FLASH->OPTR & (FLASH_OPTR_DBANK | FLASH_OPTR_BFB2)) == FLASH_OPTR_DBANK &&
           (SYSCFG->MEMRMP & SYSCFG_MEMRMP_FB_MODE) == 0u;
}

bool fw_update_flash_init(void)
{
    if (!layout_supported()) {
        return false;
    }
    if (erase_state != ERASE_UNINITIALIZED) {
        return erase_state == ERASE_IDLE;
    }

    HAL_NVIC_SetPriority(FLASH_IRQn, 1u, 0u); /* UART interrupts have priority 0. */
    HAL_NVIC_ClearPendingIRQ(FLASH_IRQn);
    erase_state = ERASE_IDLE;
    HAL_NVIC_EnableIRQ(FLASH_IRQn);
    return true;
}

bool fw_update_flash_start_erase(uint32_t page_off)
{
    if (erase_state != ERASE_IDLE || page_off % BOARD_PAGE_SIZE != 0u ||
        page_off > BOARD_SLOT1_SIZE - BOARD_PAGE_SIZE || !layout_supported() ||
        pFlash.Lock != HAL_UNLOCKED || __HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {
        return false;
    }

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Banks = FLASH_BANK_2,
        .Page = page_off / BOARD_PAGE_SIZE,
        .NbPages = 1u
    };
    if (HAL_FLASH_Unlock() != HAL_OK) {
        return false;
    }

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_SR_ERRORS | FLASH_FLAG_EOP);
    HAL_NVIC_ClearPendingIRQ(FLASH_IRQn);
    erase_state = ERASE_ACTIVE;
    if (HAL_FLASHEx_Erase_IT(&erase) != HAL_OK) {
        erase_state = ERASE_IDLE;
        if (pFlash.Lock == HAL_UNLOCKED) {
            (void)HAL_FLASH_Lock();
        }
        return false;
    }
    return true;
}

void FLASH_IRQHandler(void)
{
    HAL_FLASH_IRQHandler();
}

void HAL_FLASH_EndOfOperationCallback(uint32_t value)
{
    /* The HAL reports UINT32_MAX on the final page of an erase request. */
    if (erase_state == ERASE_ACTIVE && value == UINT32_MAX) {
        erase_state = ERASE_DONE;
    }
}

void HAL_FLASH_OperationErrorCallback(uint32_t value)
{
    (void)value;
    if (erase_state == ERASE_ACTIVE) {
        erase_state = ERASE_FAILED;
    }
}

fw_update_erase_result_t fw_update_flash_poll_erase(void)
{
    if (erase_state == ERASE_ACTIVE) {
        return FW_UPDATE_ERASE_BUSY;
    }
    if (erase_state != ERASE_DONE && erase_state != ERASE_FAILED) {
        return FW_UPDATE_ERASE_ERROR;
    }

    /* The HAL releases pFlash after the terminal callback returns. */
    if (pFlash.Lock != HAL_UNLOCKED) {
        return FW_UPDATE_ERASE_BUSY;
    }
    bool erased = erase_state == ERASE_DONE;
    HAL_StatusTypeDef locked = HAL_FLASH_Lock();
    erase_state = ERASE_IDLE;
    return erased && locked == HAL_OK ? FW_UPDATE_ERASE_OK : FW_UPDATE_ERASE_ERROR;
}

void fw_update_flash_reset(void)
{
    NVIC_SystemReset();
}
