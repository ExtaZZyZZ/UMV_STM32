/*
 * fw_update.c
 *
 *  Created on: 19 нояб. 2026 г.
 *      Author: i.yakovlev
 */

#include "fw_update.h"

#include <string.h>
#include "axis.h"
#include "bootutil/bootutil_public.h"
#include "bootutil/image.h"
#include "flash_map_backend/flash_map_backend.h"
#include "fw_update_flash.h"
#include "sysflash/sysflash.h"

#define HEADER_SIZE 0x200u
#define TRAILER_PAGES 2u
#define IMAGE_LIMIT (BOARD_SLOT1_SIZE - TRAILER_PAGES * BOARD_PAGE_SIZE)
#define FLASH_ALIGN 8u
#define DATA_LIMIT 232u
#define WRITE_SLICE (2u * FLASH_ALIGN)
#define CRC_SLICE 128u
#define CONFIRM_MS 30000u
#define TRIAL_MS 120000u
#define RESET_ACK_MS 50u
#define INACTIVE_MS 30000u

_Static_assert(IMAGE_LIMIT == 0x32800u, "Unexpected upload image limit");
_Static_assert(BOARD_SLOT1_OFF + BOARD_SLOT1_SIZE <= BOARD_CALIB_OFF,
               "Upload overlaps encoder calibration");
_Static_assert(FW_UPDATE_HEADER_WORDS + DATA_LIMIT / 2u <= 123u,
               "Upload frame exceeds FC16 capacity");

enum {
    UPDATE_IDLE = 0,
    UPDATE_ERASING,
    UPDATE_RECEIVING,
    UPDATE_VERIFIED,
    UPDATE_ERROR,
    UPDATE_VERIFYING,
    UPDATE_REBOOT_PENDING,
};

enum {
    ERR_NONE = 0,
    ERR_STATE,
    ERR_SIZE,
    ERR_OFFSET,
    ERR_FLASH,
    ERR_CRC,
    ERR_IMAGE,
    ERR_CALIBRATION,
    ERR_BOOT,
    ERR_ARGUMENT,
};

enum {
    OP_BEGIN = 1,
    OP_DATA,
    OP_VERIFY,
    OP_INSTALL,
    OP_ROLLBACK,
    OP_ABORT,
};

typedef struct {
    const struct flash_area *secondary;
    struct image_version running_version;
    struct image_version secondary_version;
    uint16_t state;
    uint16_t error;
    bool ready;
    bool running_valid;
    bool secondary_valid;
    bool confirmed;
    bool trial;
    bool confirm_attempted;
    bool link_was_up;
    uint32_t started_at;
    uint32_t link_since;
    uint32_t last_request;
    uint32_t image_size;
    uint32_t wire_size;
    uint32_t payload_erase_end;
    uint32_t erase_off;
    bool erasing_trailer;
    bool erase_in_flight;
    bool abort_after_erase;
    uint32_t next_offset;
    uint32_t data_offset;
    uint16_t data_len;
    uint16_t data_pos;
    uint8_t data[DATA_LIMIT];
    uint8_t header_prefix[FLASH_ALIGN];
    fw_update_reply_t data_reply;
    uint32_t expected_crc;
    uint32_t crc;
    uint32_t crc_pos;
} update_t;

static update_t update;

static fw_update_reply_t reject(uint16_t error)
{
    update.error = error;
    return FW_UPDATE_REJECTED;
}

static fw_update_reply_t fail(uint16_t error)
{
    update.state = UPDATE_ERROR;
    update.error = error;
    return FW_UPDATE_FAILED;
}

static uint32_t align_up(uint32_t size, uint32_t alignment)
{
    return (size + alignment - 1u) & ~(alignment - 1u);
}

static bool read_tlv_info(const struct flash_area *area, uint32_t off,
                          uint32_t limit, uint16_t magic, uint16_t *len)
{
    struct image_tlv_info info;
    if (off > limit || limit - off < sizeof(info) ||
        flash_area_read(area, off, &info, sizeof(info)) != 0 ||
        info.it_magic != magic || info.it_tlv_tot < sizeof(info) ||
        info.it_tlv_tot > limit - off) {
        return false;
    }
    *len = info.it_tlv_tot;
    return true;
}

static bool read_image_version(const struct flash_area *area, uint32_t limit,
                                const uint8_t *prefix,
                                struct image_version *version)
{
    struct image_header header;
    if (limit < HEADER_SIZE ||
        flash_area_read(area, 0u, &header, sizeof(header)) != 0) {
        return false;
    }
    if (prefix != NULL) {
        memcpy(&header, prefix, FLASH_ALIGN);
    }
    if (header.ih_magic != IMAGE_MAGIC || header.ih_hdr_size != HEADER_SIZE ||
        header.ih_flags != 0u || header.ih_load_addr != 0u ||
        header.ih_img_size < 8u || header.ih_img_size > limit - HEADER_SIZE) {
        return false;
    }

    uint32_t off = HEADER_SIZE + header.ih_img_size;
    uint16_t len;
    if (header.ih_protect_tlv_size != 0u) {
        if (!read_tlv_info(area, off, limit, IMAGE_TLV_PROT_INFO_MAGIC, &len) ||
            len != header.ih_protect_tlv_size) {
            return false;
        }
        off += len;
    }
    if (!read_tlv_info(area, off, limit, IMAGE_TLV_INFO_MAGIC, &len) ||
        (prefix != NULL && off + len != limit)) {
        return false;
    }
    *version = header.ih_ver;
    return true;
}

static void refresh_secondary(void)
{
    update.secondary_version = (struct image_version){0};
    update.secondary_valid = read_image_version(update.secondary, IMAGE_LIMIT,
                                                NULL, &update.secondary_version);
}

void fw_update_init(uint32_t now)
{
    const struct flash_area *primary;
    struct boot_swap_state swap;

    memset(&update, 0, sizeof(update));
    update.started_at = now;
    if (flash_area_open(FLASH_AREA_IMAGE_PRIMARY(0), &primary) != 0 ||
        flash_area_open(FLASH_AREA_IMAGE_SECONDARY(0), &update.secondary) != 0 ||
        boot_read_swap_state_by_id(FLASH_AREA_IMAGE_PRIMARY(0), &swap) != 0) {
        (void)fail(ERR_BOOT);
        return;
    }

    update.running_valid = read_image_version(primary, IMAGE_LIMIT, NULL,
                                              &update.running_version);
    flash_area_close(primary);
    if (swap.magic == BOOT_MAGIC_GOOD && swap.copy_done == BOOT_FLAG_SET &&
        swap.image_ok == BOOT_FLAG_UNSET) {
        update.trial = true;
    } else if ((swap.magic == BOOT_MAGIC_GOOD && swap.image_ok == BOOT_FLAG_SET) ||
               (swap.magic == BOOT_MAGIC_UNSET && swap.copy_done == BOOT_FLAG_UNSET &&
                (swap.image_ok == BOOT_FLAG_UNSET || swap.image_ok == BOOT_FLAG_SET))) {
        /* MCUboot can set image_ok on a factory image after rejecting a candidate. */
        update.confirmed = true;
    }

    if (!update.running_valid || (!update.confirmed && !update.trial)) {
        (void)fail(ERR_BOOT);
        return;
    }
    if (!fw_update_flash_init()) {
        (void)fail(ERR_FLASH);
        return;
    }
    refresh_secondary();
    update.ready = true;
}

bool fw_update_transfer_active(void)
{
    return update.state == UPDATE_ERASING || update.state == UPDATE_RECEIVING ||
           update.state == UPDATE_VERIFYING || update.state == UPDATE_VERIFIED ||
           update.state == UPDATE_REBOOT_PENDING;
}

static void write_version(uint16_t regs[5], const struct image_version *version)
{
    regs[0] = version->iv_major;
    regs[1] = version->iv_minor;
    regs[2] = version->iv_revision;
    regs[3] = (uint16_t)version->iv_build_num;
    regs[4] = (uint16_t)(version->iv_build_num >> 16);
}

void fw_update_read_status(uint16_t regs[FW_UPDATE_IREG_COUNT])
{
    regs[0] = update.state;
    regs[1] = update.error;
    regs[2] = (uint16_t)update.next_offset;
    regs[3] = (uint16_t)(update.next_offset >> 16);
    write_version(&regs[4], &update.running_version);
    regs[9] = (update.running_valid ? 1u : 0u) |
              (update.confirmed ? 2u : 0u) |
              (update.secondary_valid ? 4u : 0u) |
              (update.trial ? 8u : 0u);
    write_version(&regs[10], &update.secondary_version);
    regs[15] = 1u; /* Wire version of this register block */
}

static void finish_abort(void)
{
    update.state = UPDATE_IDLE;
    update.error = ERR_NONE;
    update.next_offset = 0u;
    update.data_len = 0u;
    update.abort_after_erase = false;
    refresh_secondary();
}

static void abort_upload(void)
{
    if (update.erase_in_flight) {
        update.abort_after_erase = true;
    } else {
        finish_abort();
    }
}

static fw_update_reply_t begin_upload(uint32_t size)
{
    if (!update.confirmed ||
        (update.state != UPDATE_IDLE && update.state != UPDATE_ERROR) ||
        boot_swap_type_multi(0) != BOOT_SWAP_TYPE_NONE) {
        return reject(ERR_STATE);
    }
    if (axis_calibration_active()) {
        return reject(ERR_CALIBRATION);
    }
    if (size < HEADER_SIZE + 8u + sizeof(struct image_tlv_info) || size > IMAGE_LIMIT) {
        return reject(ERR_SIZE);
    }

    update.image_size = size;
    update.wire_size = align_up(size, FLASH_ALIGN);
    update.payload_erase_end = align_up(update.wire_size, BOARD_PAGE_SIZE);
    /* Disarm old trailer before touching any image page. */
    update.erase_off = IMAGE_LIMIT;
    update.erasing_trailer = true;
    update.secondary_valid = false;
    update.secondary_version = (struct image_version){0};
    update.next_offset = 0u;
    update.data_len = 0u;
    update.state = UPDATE_ERASING;
    return FW_UPDATE_OK;
}

static void step_erase(void)
{
    if (!update.erase_in_flight) {
        if (fw_update_flash_start_erase(update.erase_off)) {
            update.erase_in_flight = true;
        } else {
            (void)fail(ERR_FLASH);
        }
        return;
    }

    const fw_update_erase_result_t result = fw_update_flash_poll_erase();
    if (result == FW_UPDATE_ERASE_BUSY) {
        return;
    }
    update.erase_in_flight = false;
    if (result != FW_UPDATE_ERASE_OK) {
        update.abort_after_erase = false;
        (void)fail(ERR_FLASH);
        return;
    }
    if (update.abort_after_erase) {
        finish_abort();
        return;
    }
    update.erase_off += BOARD_PAGE_SIZE;
    if (update.erasing_trailer && update.erase_off == BOARD_SLOT1_SIZE) {
        update.erasing_trailer = false;
        update.erase_off = 0u;
    } else if (!update.erasing_trailer && update.erase_off == update.payload_erase_end) {
        update.state = UPDATE_RECEIVING;
    }
}

static fw_update_reply_t start_data(uint32_t off, const uint8_t *data, uint16_t len)
{
    if (update.state != UPDATE_RECEIVING || update.data_reply == FW_UPDATE_DEFERRED) {
        return reject(ERR_STATE);
    }
    if (len == 0u || len > DATA_LIMIT || len % FLASH_ALIGN != 0u) {
        return reject(ERR_SIZE);
    }
    if (update.data_len == len && update.data_offset == off &&
        update.next_offset > off && memcmp(update.data, data, len) == 0) {
        return FW_UPDATE_OK;
    }
    if (off != update.next_offset || off >= update.wire_size ||
        len > update.wire_size - off) {
        return reject(ERR_OFFSET);
    }

    for (uint32_t i = 0u; i < len; ++i) {
        if (off + i >= update.image_size && data[i] != 0xffu) {
            return reject(ERR_ARGUMENT);
        }
    }
    memcpy(update.data, data, len);
    update.data_offset = off;
    update.data_len = len;
    update.data_pos = 0u;
    if (off == 0u) {
        /* Publish the header only once the complete image has been verified. */
        memcpy(update.header_prefix, data, FLASH_ALIGN);
        update.data_pos = FLASH_ALIGN;
    }
    update.data_reply = FW_UPDATE_DEFERRED;
    return FW_UPDATE_DEFERRED;
}

static void step_data(void)
{
    uint16_t len = update.data_len - update.data_pos;
    if (len > WRITE_SLICE) {
        len = WRITE_SLICE;
    }
    if (len != 0u &&
        flash_area_write(update.secondary, update.data_offset + update.data_pos,
                         &update.data[update.data_pos], len) != 0) {
        update.data_reply = fail(ERR_FLASH);
        return;
    }
    update.data_pos += len;
    if (update.data_pos == update.data_len) {
        update.next_offset = update.data_offset + update.data_len;
        update.data_reply = FW_UPDATE_OK;
    }
}

fw_update_reply_t fw_update_data_reply(void)
{
    return update.data_reply;
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0u; i < len; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0u; bit < 8u; ++bit) {
            crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
        }
    }
    return crc;
}

static fw_update_reply_t start_verify(uint32_t expected_crc)
{
    if (update.state != UPDATE_RECEIVING || update.next_offset != update.wire_size) {
        return reject(ERR_STATE);
    }
    update.expected_crc = expected_crc;
    update.crc = crc32_update(UINT32_MAX, update.header_prefix, FLASH_ALIGN);
    update.crc_pos = FLASH_ALIGN;
    update.state = UPDATE_VERIFYING;
    return FW_UPDATE_OK;
}

static void step_verify(void)
{
    uint8_t bytes[CRC_SLICE];
    uint32_t len = update.image_size - update.crc_pos;
    if (len > sizeof(bytes)) {
        len = sizeof(bytes);
    }
    if (flash_area_read(update.secondary, update.crc_pos, bytes, len) != 0) {
        (void)fail(ERR_FLASH);
        return;
    }
    update.crc = crc32_update(update.crc, bytes, len);
    update.crc_pos += len;
    if (update.crc_pos != update.image_size) {
        return;
    }
    if ((update.crc ^ UINT32_MAX) != update.expected_crc) {
        (void)fail(ERR_CRC);
        return;
    }
    struct image_version version;
    if (!read_image_version(update.secondary, update.image_size,
                            update.header_prefix, &version)) {
        (void)fail(ERR_IMAGE);
        return;
    }
    update.state = UPDATE_VERIFIED;
}

static fw_update_reply_t schedule_test(void)
{
    struct boot_swap_state swap;
    if (boot_read_swap_state_by_id(FLASH_AREA_IMAGE_SECONDARY(0), &swap) != 0 ||
        swap.magic != BOOT_MAGIC_UNSET || swap.image_ok != BOOT_FLAG_UNSET ||
        swap.copy_done != BOOT_FLAG_UNSET ||
        boot_set_pending_multi(0, 0) != 0 ||
        boot_swap_type_multi(0) != BOOT_SWAP_TYPE_TEST) {
        return fail(ERR_BOOT);
    }
    update.state = UPDATE_REBOOT_PENDING;
    return FW_UPDATE_OK;
}

static fw_update_reply_t install(void)
{
    if (update.state != UPDATE_VERIFIED || !update.confirmed) {
        return reject(ERR_STATE);
    }
    if (flash_area_write(update.secondary, 0u, update.header_prefix, FLASH_ALIGN) != 0) {
        return fail(ERR_FLASH);
    }
    refresh_secondary();
    return schedule_test();
}

static fw_update_reply_t rollback(void)
{
    if (update.state != UPDATE_IDLE) {
        return reject(ERR_STATE);
    }
    if (axis_calibration_active()) {
        return reject(ERR_CALIBRATION);
    }
    if (update.trial) {
        update.state = UPDATE_REBOOT_PENDING;
        return FW_UPDATE_OK;
    }
    if (!update.confirmed || !update.secondary_valid) {
        return reject(ERR_STATE);
    }
    return schedule_test();
}

fw_update_reply_t fw_update_request(uint16_t op, uint32_t arg,
                                    const uint8_t *data, uint16_t len,
                                    uint32_t now)
{
    if (!update.ready || update.state == UPDATE_REBOOT_PENDING ||
        update.data_reply == FW_UPDATE_DEFERRED) {
        return FW_UPDATE_REJECTED;
    }
    if ((op != OP_DATA && len != 0u) || (op >= OP_INSTALL && arg != 0u)) {
        return reject(ERR_ARGUMENT);
    }

    fw_update_reply_t result;
    switch (op) {
    case OP_BEGIN:
        result = begin_upload(arg);
        break;
    case OP_DATA:
        result = start_data(arg, data, len);
        break;
    case OP_VERIFY:
        result = start_verify(arg);
        break;
    case OP_INSTALL:
        result = install();
        break;
    case OP_ROLLBACK:
        result = rollback();
        break;
    case OP_ABORT:
        abort_upload();
        result = FW_UPDATE_OK;
        break;
    default:
        return reject(ERR_ARGUMENT);
    }

    if (result == FW_UPDATE_OK || result == FW_UPDATE_DEFERRED) {
        update.last_request = now;
        update.error = ERR_NONE;
    }
    return result;
}

static void step_trial(uint32_t now, bool link_up)
{
    if (now - update.started_at >= TRIAL_MS) {
        update.state = UPDATE_REBOOT_PENDING;
        fw_update_flash_reset();
        return;
    }
    if (!update.ready || update.confirm_attempted) {
        return;
    }
    if (!link_up) {
        update.link_was_up = false;
        return;
    }
    if (!update.link_was_up) {
        update.link_since = now;
        update.link_was_up = true;
        return;
    }
    if (now - update.link_since >= CONFIRM_MS) {
        update.confirm_attempted = true;
        if (boot_set_confirmed_multi(0) == 0) {
            update.confirmed = true;
            update.trial = false;
        } else {
            (void)fail(ERR_BOOT);
        }
    }
}

void fw_update_step(uint32_t now, bool link_up, bool tx_idle)
{
    if (update.state == UPDATE_REBOOT_PENDING) {
        if (tx_idle || now - update.last_request >= RESET_ACK_MS) {
            fw_update_flash_reset();
        }
        return;
    }
    if (update.trial) {
        step_trial(now, link_up);
        return;
    }
    if (fw_update_transfer_active() && !update.abort_after_erase &&
        update.data_reply != FW_UPDATE_DEFERRED &&
        now - update.last_request >= INACTIVE_MS) {
        abort_upload();
    }

    switch (update.state) {
    case UPDATE_ERASING:
        step_erase();
        break;
    case UPDATE_RECEIVING:
        if (update.data_reply == FW_UPDATE_DEFERRED) {
            step_data();
        }
        break;
    case UPDATE_VERIFYING:
        step_verify();
        break;
    default:
        break;
    }
}
