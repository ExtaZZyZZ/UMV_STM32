/*
 * flash_map_backend.c
 *
 *  Created on: 19 нояб. 2026 г.
 *      Author: i.yakovlev
 */


#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stm32g4xx_hal.h>
#include "flash_map_backend/flash_map_backend.h"
#include "sysflash/sysflash.h"

#define BOOT_FLASH_BANK_BYTES 0x40000u
enum {
    PRIMARY_SLOT = 0,
    SECONDARY_SLOT,
    SLOT_COUNT
};

_Static_assert(BOARD_SLOT0_OFF % BOARD_PAGE_SIZE == 0u &&
               BOARD_SLOT0_SIZE % BOARD_PAGE_SIZE == 0u &&
               BOARD_SLOT1_OFF % BOARD_PAGE_SIZE == 0u &&
               BOARD_SLOT1_SIZE % BOARD_PAGE_SIZE == 0u,
               "Flash slots must use whole pages");

static const struct flash_area slot_areas[SLOT_COUNT] = {
    [PRIMARY_SLOT] = {
        .fa_id = FLASH_AREA_IMAGE_PRIMARY(0),
        .fa_device_id = 0u,
        .fa_off = BOARD_SLOT0_OFF,
        .fa_size = BOARD_SLOT0_SIZE,
    },
    [SECONDARY_SLOT] = {
        .fa_id = FLASH_AREA_IMAGE_SECONDARY(0),
        .fa_device_id = 0u,
        .fa_off = BOARD_SLOT1_OFF,
        .fa_size = BOARD_SLOT1_SIZE,
    },
};

static int slot_index_from_id(int area_id)
{
    for (int slot = 0; slot < SLOT_COUNT; ++slot) {
        if (slot_areas[slot].fa_id == area_id) {
            return slot;
        }
    }

    return -1;
}

static int check_slot_range(const struct flash_area *area, uint32_t off,
                            uint32_t len)
{
    if (area != &slot_areas[PRIMARY_SLOT] &&
        area != &slot_areas[SECONDARY_SLOT]) {
        return -EINVAL;
    }

    if (off > area->fa_size || len > area->fa_size - off) {
        return -ERANGE;
    }

    return 0;
}

int flash_area_open(uint8_t id, const struct flash_area **area_out)
{
    if (area_out == NULL) {
        return -EINVAL;
    }

    int slot = slot_index_from_id(id);
    if (slot < 0) {
        *area_out = NULL;
        return -EINVAL;
    }

    *area_out = &slot_areas[slot];
    return 0;
}

void flash_area_close(const struct flash_area *area)
{
    (void)area;
}

int flash_area_read(const struct flash_area *area, uint32_t off,
                    void *dst, uint32_t len)
{
    int rc = check_slot_range(area, off, len);
    if (rc != 0 || len == 0u) {
        return rc;
    }

    if (dst == NULL) {
        return -EINVAL;
    }

    uintptr_t address = (uintptr_t)BOARD_FLASH_BASE + area->fa_off + off;
    memcpy(dst, (const void *)address, len);
    return 0;
}

uint32_t flash_area_align(const struct flash_area *area)
{
    (void)area;
    return 8u;
}

uint8_t flash_area_erased_val(const struct flash_area *area)
{
    (void)area;
    return 0xFFu;
}

int flash_area_get_sectors(int area_id, uint32_t *count,
                           struct flash_sector *sectors)
{
    int slot = slot_index_from_id(area_id);
    if (slot < 0 || count == NULL || sectors == NULL) {
        return -EINVAL;
    }

    uint32_t capacity = *count;
    uint32_t page_count = slot_areas[slot].fa_size / BOARD_PAGE_SIZE;
    *count = page_count;

    if (capacity < page_count) {
        return -ENOMEM;
    }

    for (uint32_t page = 0; page < page_count; ++page) {
        sectors[page].fs_off = page * BOARD_PAGE_SIZE;
        sectors[page].fs_size = BOARD_PAGE_SIZE;
    }

    return 0;
}

int flash_area_get_sector(const struct flash_area *area, uint32_t off,
                          struct flash_sector *sector)
{
    if (sector == NULL) {
        return -EINVAL;
    }

    int rc = check_slot_range(area, off, 1u);
    if (rc != 0) {
        return rc;
    }

    sector->fs_off = off - off % BOARD_PAGE_SIZE;
    sector->fs_size = BOARD_PAGE_SIZE;
    return 0;
}

int flash_area_id_from_multi_image_slot(int image_index, int slot)
{
    if (image_index != 0 || slot < 0 || slot >= SLOT_COUNT) {
        return -EINVAL;
    }

    return slot_areas[slot].fa_id;
}

int flash_area_id_from_image_slot(int slot)
{
    return flash_area_id_from_multi_image_slot(0, slot);
}

int flash_area_id_to_multi_image_slot(int image_index, int area_id)
{
    if (image_index != 0) {
        return -1;
    }

    return slot_index_from_id(area_id);
}

_Static_assert(BOARD_FLASH_BASE == FLASH_BASE &&
               BOARD_PAGE_SIZE == FLASH_PAGE_SIZE,
               "STM32 Flash geometry mismatch");
_Static_assert(BOARD_SLOT1_OFF == BOOT_FLASH_BANK_BYTES &&
               BOARD_SLOT1_SIZE <= BOOT_FLASH_BANK_BYTES,
               "Flash slots must stay within their banks");

static int flash_layout_supported(void)
{
    return __HAL_RCC_SYSCFG_IS_CLK_ENABLED() &&
           FLASH_SIZE == 2u * BOOT_FLASH_BANK_BYTES &&
           (FLASH->OPTR & (FLASH_OPTR_DBANK | FLASH_OPTR_BFB2)) ==
               FLASH_OPTR_DBANK &&
           (SYSCFG->MEMRMP & SYSCFG_MEMRMP_FB_MODE) == 0u;
}

int flash_area_write(const struct flash_area *area, uint32_t off,
                     const void *src, uint32_t len)
{
    int rc = check_slot_range(area, off, len);
    if (rc != 0 || len == 0u) {
        return rc;
    }

    const uint32_t word_size = sizeof(uint64_t);
    if (src == NULL || off % word_size != 0u || len % word_size != 0u) {
        return -EINVAL;
    }

    if (!flash_layout_supported()) {
        return -ENOTSUP;
    }

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return -EIO;
    }

    const uint8_t *bytes = src;
    uint32_t address = BOARD_FLASH_BASE + area->fa_off + off;

    for (uint32_t pos = 0; pos < len; pos += word_size) {
        uint64_t wanted;
        uint64_t current;
        uint32_t word_address = address + pos;

        memcpy(&wanted, bytes + pos, sizeof(wanted));
        memcpy(&current, (const void *)(uintptr_t)word_address,
               sizeof(current));

        if (current == wanted) {
            continue;
        }

        if (current != UINT64_MAX ||
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, word_address,
                              wanted) != HAL_OK) {
            rc = -EIO;
            break;
        }
    }

    if (HAL_FLASH_Lock() != HAL_OK) {
        return -EIO;
    }

    return rc;
}

int flash_area_erase(const struct flash_area *area, uint32_t off, uint32_t len)
{
    int rc = check_slot_range(area, off, len);
    if (rc != 0 || len == 0u) {
        return rc;
    }

    if (off % BOARD_PAGE_SIZE != 0u || len % BOARD_PAGE_SIZE != 0u) {
        return -EINVAL;
    }

    if (!flash_layout_supported()) {
        return -ENOTSUP;
    }

    uint32_t device_off = area->fa_off + off;
    uint32_t bank_off = device_off % BOOT_FLASH_BANK_BYTES;

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Banks = device_off < BOOT_FLASH_BANK_BYTES ? FLASH_BANK_1 : FLASH_BANK_2,
        .Page = bank_off / BOARD_PAGE_SIZE,
        .NbPages = len / BOARD_PAGE_SIZE,
    };
    uint32_t page_error = UINT32_MAX;

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return -EIO;
    }

    HAL_StatusTypeDef erase_status = HAL_FLASHEx_Erase(&erase, &page_error);
    HAL_StatusTypeDef lock_status = HAL_FLASH_Lock();

    return erase_status == HAL_OK && lock_status == HAL_OK ? 0 : -EIO;
}
