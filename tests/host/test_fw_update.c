#include "test_support.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "axis.h"
#include "bootutil/bootutil_public.h"
#include "flash_map_backend/flash_map_backend.h"
#include "fw_update.h"
#include "fw_update_flash.h"
#include "regfile.h"
#include "slave.h"
#include "sysflash/sysflash.h"

enum {
    RAW_SIZE = 695,
    WIRE_SIZE = 696,
    BLOCK_SIZE = 232,
    STATUS_WORDS = 16,
    BASE = 0x0100,
    MAX_IMAGE = 0x32800,
    TRAILER_START = 0x32800,
    BEGIN = 1,
    DATA = 2,
    VERIFY = 3,
    INSTALL = 4,
    ROLLBACK = 5,
    ABORT = 6,
    IDLE = 0,
    ERASING = 1,
    RECEIVING = 2,
    VERIFIED = 3,
    ERROR = 4,
    VERIFYING = 5,
    REBOOT_PENDING = 6,
    ACTIVE_VALID = 1,
    CONFIRMED = 2,
    SECONDARY_VALID = 4,
    TRIAL = 8,
};

static const struct flash_area primary = {
    .fa_id = FLASH_AREA_IMAGE_PRIMARY(0),
    .fa_off = BOARD_SLOT0_OFF,
    .fa_size = BOARD_SLOT0_SIZE,
};
static const struct flash_area secondary = {
    .fa_id = FLASH_AREA_IMAGE_SECONDARY(0),
    .fa_off = BOARD_SLOT1_OFF,
    .fa_size = BOARD_SLOT1_SIZE,
};
static const uint32_t image_crc32 = 0x1b24f3cau;
static uint8_t flash[BOARD_CALIB_OFF + BOARD_PAGE_SIZE];
static uint8_t image[WIRE_SIZE];
static uint32_t now_ms;
static bool calibration_active;
static unsigned resets, writes, erase_count, unsafe_accesses;
static uint32_t erased_pages[3];
static bool erase_pending, erase_wait;
static uint32_t erase_offset;
static mb_slave_t slave;
static uint8_t reply[ADU_MAX_LEN];
static uint16_t reply_len;

bool axis_calibration_active(void)
{
    return calibration_active;
}

bool fw_update_flash_init(void)
{
    erase_pending = false;
    return true;
}

bool fw_update_flash_start_erase(uint32_t page_off)
{
    if (erase_pending || page_off % BOARD_PAGE_SIZE != 0u ||
        page_off > secondary.fa_size - BOARD_PAGE_SIZE) {
        ++unsafe_accesses;
        return false;
    }

    if (erase_count < sizeof(erased_pages) / sizeof(erased_pages[0])) {
        erased_pages[erase_count] = page_off;
    }
    ++erase_count;
    erase_offset = page_off;
    erase_wait = true;
    erase_pending = true;
    return true;
}

fw_update_erase_result_t fw_update_flash_poll_erase(void)
{
    if (!erase_pending) {
        return FW_UPDATE_ERASE_ERROR;
    }
    if (erase_wait) {
        erase_wait = false;
        return FW_UPDATE_ERASE_BUSY;
    }

    memset(flash + secondary.fa_off + erase_offset, 0xff, BOARD_PAGE_SIZE);
    erase_pending = false;
    return FW_UPDATE_ERASE_OK;
}

void fw_update_flash_reset(void)
{
    ++resets;
}

static bool range_valid(const struct flash_area *area, uint32_t off, uint32_t length)
{
    return (area == &primary || area == &secondary) &&
           off <= area->fa_size && length <= area->fa_size - off;
}

int flash_area_open(uint8_t id, const struct flash_area **area)
{
    if (id == primary.fa_id) {
        *area = &primary;
    } else if (id == secondary.fa_id) {
        *area = &secondary;
    } else {
        return -1;
    }
    return 0;
}

void flash_area_close(const struct flash_area *area)
{
    (void)area;
}

uint32_t flash_area_align(const struct flash_area *area)
{
    (void)area;
    return 8u;
}

uint8_t flash_area_erased_val(const struct flash_area *area)
{
    (void)area;
    return 0xffu;
}

int flash_area_read(const struct flash_area *area, uint32_t off, void *data,
                    uint32_t length)
{
    if (!range_valid(area, off, length) || (area == &secondary && erase_pending)) {
        ++unsafe_accesses;
        return -1;
    }
    memcpy(data, flash + area->fa_off + off, length);
    return 0;
}

int flash_area_write(const struct flash_area *area, uint32_t off, const void *data,
                     uint32_t length)
{
    if (erase_pending || off % 8u != 0u || length % 8u != 0u ||
        !range_valid(area, off, length)) {
        ++unsafe_accesses;
        return -1;
    }
    ++writes;
    const uint8_t *src = data;
    uint8_t *dest = flash + area->fa_off + off;
    for (uint32_t i = 0; i < length; i += 8u) {
        static const uint8_t erased[8] = {
            0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu,
        };
        if (memcmp(dest + i, src + i, 8u) == 0) {
            continue;
        }
        if (memcmp(dest + i, erased, 8u) != 0) {
            ++unsafe_accesses;
            return -1;
        }
        memcpy(dest + i, src + i, 8u);
    }
    return 0;
}

static void wr_le16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
}

static void wr_le32(uint8_t *out, uint32_t value)
{
    wr_le16(out, (uint16_t)value);
    wr_le16(out + 2, (uint16_t)(value >> 16));
}

static void make_image(void)
{
    memset(image, 0, sizeof(image));
    wr_le32(image, 0x96f3b83du);
    wr_le16(image + 8, 0x200u);
    wr_le32(image + 12, 32u);
    image[20] = 2u;
    image[21] = 3u;
    wr_le16(image + 22, 0x1234u);
    wr_le32(image + 24, 0xfedcba98u);

    wr_le32(image + 512, 0x2001f000u);
    wr_le32(image + 516, 0x0800c209u);
    for (unsigned i = 0; i < 24u; ++i) {
        image[520u + i] = (uint8_t)(0x80u + i);
    }

    /* Well-formed synthetic TLVs. The bootloader authenticates real images. */
    wr_le16(image + 544, 0x6907u);
    wr_le16(image + 546, 151u);
    wr_le16(image + 548, 0x01u);
    wr_le16(image + 550, 32u);
    memset(image + 552, 0xa5, 32);
    wr_le16(image + 584, 0x10u);
    wr_le16(image + 586, 32u);
    memset(image + 588, 0xcd, 32);
    wr_le16(image + 620, 0x22u);
    wr_le16(image + 622, 71u);
    const uint8_t sig_header[] = {0x30u, 0x45u, 0x02u, 0x21u, 0x00u};
    memcpy(image + 624, sig_header, sizeof(sig_header));
    memset(image + 629, 0x81, 32);
    image[661] = 0x02u;
    image[662] = 0x20u;
    memset(image + 663, 0x42, 32);
    image[RAW_SIZE] = 0xffu;
}

static uint32_t tick(void) { return now_ms; }
static void no_interrupts(void) {}

static void capture_reply(const uint8_t *data, uint16_t len)
{
    CHECK(reply_len == 0u && len <= sizeof(reply));
    if (len <= sizeof(reply)) {
        memcpy(reply, data, len);
        reply_len = len;
    }
}

static const mb_hw_ops_t operations = {
    .uart_transmit = capture_reply,
    .critical_enter = no_interrupts,
    .critical_exit = no_interrupts,
};

static void start_app(void)
{
    now_ms = 0u;
    fw_update_init(now_ms);
    mb_slave_init(&slave, &operations, tick, 1u);
}

static void set_up(bool trial)
{
    erase_pending = false;
    memset(flash, 0xff, sizeof(flash));
    memset(flash + BOARD_CALIB_OFF, 0x5a, BOARD_PAGE_SIZE);
    memcpy(flash + primary.fa_off, image, RAW_SIZE);
    flash[primary.fa_off + 20u] = 1u;
    memcpy(flash + secondary.fa_off, image, RAW_SIZE);
    if (trial) {
        /* Simulate the trailer left after MCUboot swaps a trial into primary. */
        CHECK(boot_set_pending(0) == 0);
        memcpy(flash + primary.fa_off + primary.fa_size - BOOT_MAGIC_SZ,
               flash + secondary.fa_off + secondary.fa_size - BOOT_MAGIC_SZ,
               BOOT_MAGIC_SZ);
        flash[primary.fa_off + primary.fa_size - BOOT_MAGIC_SZ - 2u * BOOT_MAX_ALIGN] = 1u;
        memset(flash + secondary.fa_off + TRAILER_START, 0xff,
               secondary.fa_size - TRAILER_START);
        CHECK(boot_swap_type() == BOOT_SWAP_TYPE_REVERT);
    } else {
        /* A previous attempt can leave non-erased data in both trailer pages. */
        flash[secondary.fa_off + TRAILER_START] = 0x00u;
        flash[secondary.fa_off + TRAILER_START + BOARD_PAGE_SIZE] = 0x00u;
    }
    calibration_active = false;
    resets = writes = erase_count = unsafe_accesses = 0u;
    start_app();
}

static void send_frame(uint8_t *frame, uint16_t length)
{
    reply_len = 0u;
    length = adu_wrap(frame, length);
    for (uint16_t i = 0; i < length; ++i) {
        mb_slave_on_rx_byte(&slave, frame[i]);
    }
    now_ms += 3u;
    modbus_handler_task(&slave);
}

static void request(uint16_t op, uint32_t arg, const uint8_t *data, uint16_t length)
{
    const uint16_t qty = (uint16_t)(4u + length / 2u);
    uint8_t frame[ADU_MAX_LEN] = {1u, 0x10u};
    wr_u16be(frame + 2, BASE);
    wr_u16be(frame + 4, qty);
    frame[6] = (uint8_t)(qty * 2u);
    wr_u16be(frame + 7, op);
    wr_u16be(frame + 9, (uint16_t)arg);
    wr_u16be(frame + 11, (uint16_t)(arg >> 16));
    wr_u16be(frame + 13, length);
    if (length != 0u) {
        memcpy(frame + 15, data, length);
    }
    send_frame(frame, (uint16_t)(15u + length));
}

static void check_ack(uint16_t quantity)
{
    CHECK(reply_len == 8u && reply[1] == 0x10u);
    CHECK(rd_u16be(reply + 2) == BASE && rd_u16be(reply + 4) == quantity);
    CHECK(adu_check(reply, reply_len, 1u, NULL) == ADU_OK);
}

static void check_rejected(void)
{
    CHECK(reply_len == 5u && reply[1] == 0x90u);
    CHECK(adu_check(reply, reply_len, 1u, NULL) == ADU_OK);
}

static void status(uint16_t out[STATUS_WORDS])
{
    uint8_t frame[8] = {1u, 4u};
    wr_u16be(frame + 2, BASE);
    wr_u16be(frame + 4, STATUS_WORDS);
    send_frame(frame, 6u);
    CHECK(reply_len == 37u && reply[1] == 4u && reply[2] == 32u);
    CHECK(adu_check(reply, reply_len, 1u, NULL) == ADU_OK);
    for (unsigned i = 0; i < STATUS_WORDS; ++i) {
        out[i] = rd_u16be(reply + 3u + i * 2u);
    }
}

static void run_step(bool link_up, bool uart_idle)
{
    ++now_ms;
    fw_update_step(now_ms, link_up, uart_idle);
    modbus_handler_task(&slave);
}

static void wait_state(uint16_t expected, uint16_t waiting)
{
    uint16_t regs[STATUS_WORDS];
    status(regs);
    for (unsigned i = 0; i < 50u && regs[0] == waiting; ++i) {
        run_step(false, true);
        status(regs);
    }
    CHECK(regs[0] == expected);
}

static void begin_update(void)
{
    request(BEGIN, RAW_SIZE, NULL, 0u);
    check_ack(4u);
    CHECK(fw_update_transfer_active());
    wait_state(RECEIVING, ERASING);
}

static void send_data(uint32_t off)
{
    request(DATA, off, image + off, BLOCK_SIZE);
    CHECK(reply_len == 0u);
    for (unsigned i = 0; i < 100u && reply_len == 0u; ++i) {
        run_step(false, true);
    }
    check_ack(4u + BLOCK_SIZE / 2u);
}

static bool header_erased(void)
{
    static const uint8_t erased_word[] = {
        0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu,
    };
    return memcmp(flash + secondary.fa_off, erased_word, sizeof(erased_word)) == 0;
}

static void check_calibration_untouched(void)
{
    for (uint32_t i = 0; i < BOARD_PAGE_SIZE; ++i) {
        if (flash[BOARD_CALIB_OFF + i] != 0x5au) {
            CHECK(false);
            return;
        }
    }
    CHECK(unsafe_accesses == 0u);
}

static void test_receive_and_install(void)
{
    set_up(false);
    uint16_t regs[STATUS_WORDS];
    status(regs);
    CHECK(regs[0] == IDLE && regs[1] == 0u && regs[15] == 1u);
    CHECK(regs[4] == 1u && regs[10] == 2u);
    CHECK(regs[5] == 3u && regs[6] == 0x1234u);
    CHECK(regs[7] == 0xba98u && regs[8] == 0xfedcu);
    CHECK(regs[9] == (ACTIVE_VALID | CONFIRMED | SECONDARY_VALID));

    request(BEGIN, MAX_IMAGE + 1u, NULL, 0u);
    check_rejected();
    calibration_active = true;
    request(BEGIN, RAW_SIZE, NULL, 0u);
    check_rejected();
    calibration_active = false;
    CHECK(erase_count == 0u);

    begin_update();
    CHECK(erase_count == 3u && erased_pages[2] == 0u);
    CHECK(erased_pages[0] >= TRAILER_START && erased_pages[1] >= TRAILER_START);
    CHECK(erased_pages[0] != erased_pages[1]);
    CHECK(boot_swap_type() == BOOT_SWAP_TYPE_NONE);

    send_data(0u);
    status(regs);
    CHECK(regs[2] == BLOCK_SIZE && regs[3] == 0u);
    CHECK(header_erased());
    unsigned count = writes;
    request(DATA, 0u, image, BLOCK_SIZE);
    check_ack(4u + BLOCK_SIZE / 2u);
    CHECK(writes == count);

    send_data(BLOCK_SIZE);
    send_data(2u * BLOCK_SIZE);
    status(regs);
    CHECK(regs[2] == WIRE_SIZE && regs[3] == 0u);
    CHECK(header_erased());

    request(VERIFY, image_crc32, NULL, 0u);
    check_ack(4u);
    wait_state(VERIFIED, VERIFYING);
    CHECK(header_erased());
    CHECK(boot_swap_type() == BOOT_SWAP_TYPE_NONE);

    request(INSTALL, 0u, NULL, 0u);
    check_ack(4u);
    status(regs);
    CHECK(regs[0] == REBOOT_PENDING);
    CHECK(memcmp(flash + secondary.fa_off, image, WIRE_SIZE) == 0);
    CHECK(boot_swap_type() == BOOT_SWAP_TYPE_TEST);
    for (unsigned i = 0; i < 5u; ++i) {
        run_step(false, false);
    }
    CHECK(resets == 0u);
    for (unsigned i = 0; i < 100u && resets == 0u; ++i) {
        run_step(false, true);
    }
    CHECK(resets == 1u);
    check_calibration_untouched();
}

static void test_incomplete_and_bad_crc(void)
{
    set_up(false);
    begin_update();
    send_data(0u);
    const unsigned count = writes;
    uint8_t changed[BLOCK_SIZE];
    memcpy(changed, image, sizeof(changed));
    changed[8] ^= 1u;
    request(DATA, 0u, changed, BLOCK_SIZE);
    check_rejected();
    CHECK(writes == count);
    CHECK(header_erased());
    start_app();
    uint16_t regs[STATUS_WORDS];
    status(regs);
    CHECK(regs[0] == IDLE && (regs[9] & SECONDARY_VALID) == 0u);
    CHECK(boot_swap_type() == BOOT_SWAP_TYPE_NONE);
    request(ROLLBACK, 0u, NULL, 0u);
    check_rejected();
    CHECK(resets == 0u);

    begin_update();
    for (uint32_t off = 0; off < WIRE_SIZE; off += BLOCK_SIZE) {
        send_data(off);
    }
    request(VERIFY, image_crc32 ^ 1u, NULL, 0u);
    check_ack(4u);
    wait_state(ERROR, VERIFYING);
    request(INSTALL, 0u, NULL, 0u);
    check_rejected();
    CHECK(header_erased() && resets == 0u);
    request(ABORT, 0u, NULL, 0u);
    check_ack(4u);
    status(regs);
    CHECK(regs[0] == IDLE);
    check_calibration_untouched();
}

static void test_trial(void)
{
    set_up(true);
    uint16_t regs[STATUS_WORDS];
    status(regs);
    CHECK((regs[9] & (CONFIRMED | TRIAL)) == TRIAL);
    request(BEGIN, RAW_SIZE, NULL, 0u);
    check_rejected();
    CHECK(erase_count == 0u);

    fw_update_step(100u, true, true);
    fw_update_step(30099u, true, true);
    CHECK(boot_swap_type() == BOOT_SWAP_TYPE_REVERT);
    fw_update_step(30100u, false, true);
    fw_update_step(30200u, true, true);
    fw_update_step(60199u, true, true);
    CHECK(boot_swap_type() == BOOT_SWAP_TYPE_REVERT);
    fw_update_step(60200u, true, true);
    CHECK(boot_swap_type() == BOOT_SWAP_TYPE_NONE);
    status(regs);
    CHECK((regs[9] & (CONFIRMED | TRIAL)) == CONFIRMED);
    CHECK(resets == 0u);
    check_calibration_untouched();

    set_up(true);
    fw_update_step(119999u, false, false);
    CHECK(resets == 0u);
    fw_update_step(120001u, false, false);
    CHECK(resets == 1u);
    CHECK(boot_swap_type() == BOOT_SWAP_TYPE_REVERT);
    check_calibration_untouched();
}

static void test_factory_after_rejected_signature(void)
{
    set_up(false);
    /* BOOT_SWAP_TYPE_FAIL records image_ok even if the primary has no magic. */
    flash[primary.fa_off + primary.fa_size - BOOT_MAGIC_SZ - BOOT_MAX_ALIGN] = 1u;
    start_app();
    uint16_t regs[STATUS_WORDS];
    status(regs);
    CHECK(regs[0] == IDLE && (regs[9] & CONFIRMED) != 0u);
    request(BEGIN, RAW_SIZE, NULL, 0u);
    check_ack(4u);
}

void test_fw_update_run(void)
{
    make_image();
    test_receive_and_install();
    test_incomplete_and_bad_crc();
    test_trial();
    test_factory_after_rejected_signature();
}
