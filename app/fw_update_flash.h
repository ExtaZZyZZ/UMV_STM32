#ifndef APP_FW_UPDATE_FLASH_H
#define APP_FW_UPDATE_FLASH_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FW_UPDATE_ERASE_BUSY,
    FW_UPDATE_ERASE_OK,
    FW_UPDATE_ERASE_ERROR
} fw_update_erase_result_t;

bool fw_update_flash_init(void);
bool fw_update_flash_start_erase(uint32_t page_off);
fw_update_erase_result_t fw_update_flash_poll_erase(void);
void fw_update_flash_reset(void);

#endif /* APP_FW_UPDATE_FLASH_H */
