/* СГЕНЕРИРОВАНО protocol/generate.py — НЕ РЕДАКТИРОВАТЬ.
 *
 * Источник: protocol/board1_registers.yaml в репозитории pi4-fleet.
 * Этот файл вендорится копией; правки вносить в YAML и перегенерировать.
 */
#ifndef BOARD_REGISTERS_H
#define BOARD_REGISTERS_H

#include <stdint.h>

#define PROTOCOL_VERSION 7u
#define BOARD_ID 1u
#define BOARD_NAME "payload-board-1"

/* Отдаётся в регистре layout_crc. Мастер сверяет со своим значением и
 * отказывается работать при несовпадении. */
#define LAYOUT_CRC 0x80EDu

#define MODBUS_SLAVE_ADDRESS 1u
#define MODBUS_BAUD 115200u
#define MODBUS_PARITY 'E'
#define MODBUS_STOP_BITS 1u

#define POLL_INTERVAL_MS 50u
#define COMMAND_TIMEOUT_MS 100u
#define COMMAND_RETRIES 3u
#define LINK_TIMEOUT_MS 500u
#define SETTLE_AFTER_OUTPUT_MS 150u
#define FRAME_GAP_MS 1u

typedef enum {
    OUT_SPOTLIGHT = 0,  /* Фары дальнего света */
    OUT_LIGHTBAR = 1,  /* Светодиодная балка */
    OUT_SGU_LIGHT = 2,  /* СГУ «КОЛУМБ» синий */
    OUT_UNDERWATER = 3,  /* Подводные огни */
    OUT_FLASHER_YELLOW = 4,  /* Проблесковые желтые тарелки */
    OUT_FLASHER_BLUE = 5,  /* Пробелсковые синие тарелки */
} out_t;

typedef enum {
    IN_RELEASE1_DEPLOYED = 0,  /* Концевик системы сброса ПС №1 */
    IN_RELEASE2_DEPLOYED = 1,  /* Концевик системы сброса ПС №2 */
    IN_RELEASE1_LOCK_OPEN = 2,  /* Концевик замка ПС №1 */
    IN_RELEASE2_LOCK_OPEN = 3,  /* Концевик замка ПС №2 */
} in_t;

typedef enum {
    HREG_CMD_SEQ = 0,  /* Номер команды */
    HREG_CMD_CODE = 1,  /* Код команды */
    HREG_CMD_ARG0 = 2,  /* Аргумент 0 */
    HREG_CMD_ARG1 = 3,  /* Аргумент 1 */
    HREG_OUTPUTS = 4,  /* Битовая маска дискретных выходов */
} hreg_t;

typedef enum {
    IREG_ACK_SEQ = 0,  /* Номер последней исполненной команды */
    IREG_ACK_RESULT = 1,  /* Результат последней команды */
    IREG_DO_ACTUAL = 2,  /* Фактическое состояние дискретных выходов */
    IREG_DI_BITS = 3,  /* Дискретные входы */
    IREG_FLAGS = 4,  /* Флаги состояния */
    IREG_AXIS0_STATE = 5,  /* Состояние оси 0 */
    IREG_AXIS0_STEP = 6,  /* Ступень оси 0 */
    IREG_AXIS0_POS_MM_X10 = 7,  /* Положение оси 0, мм x10 */
    IREG_AXIS0_FAULT = 8,  /* Код неисправности оси 0 */
    IREG_AXIS1_STATE = 9,  /* Состояние оси 1 */
    IREG_AXIS1_STEP = 10,  /* Ступень оси 1 */
    IREG_AXIS1_POS_MM_X10 = 11,  /* Положение оси 1, мм x10 */
    IREG_AXIS1_FAULT = 12,  /* Код неисправности оси 1 */
    IREG_UPTIME_LO = 13,  /* Наработка, мс (младшее слово) */
    IREG_UPTIME_HI = 14,  /* Наработка, мс (старшее слово) */
    IREG_CRC_ERR = 15,  /* Счётчик кадров с битым CRC */
    IREG_FRAME_ERR = 16,  /* Счётчик ошибок кадрирования */
    IREG_ENC_ERR = 17,  /* Счётчик отказов обмена с энкодером */
    IREG_PROTO_VER = 18,  /* Версия протокола в прошивке */
    IREG_LAYOUT_CRC = 19,  /* CRC регистровой карты */
    IREG_FW_BUILD_LO = 20,  /* Идентификатор сборки прошивки (младшее слово) */
    IREG_FW_BUILD_HI = 21,  /* Идентификатор сборки прошивки (старшее слово) */
} ireg_t;

typedef enum {
    ACK_RESULT_OK = 0,
    ACK_RESULT_BAD_ARG = 1,
    ACK_RESULT_REFUSED_SAFETY = 2,
    ACK_RESULT_REFUSED_BUSY = 3,
    ACK_RESULT_REFUSED_LINK_LOST = 4,
    ACK_RESULT_UNKNOWN_CMD = 5,
} ack_result_t;

typedef enum {
    AXIS_FAULT_NONE = 0,
    AXIS_FAULT_STALL = 1,
    AXIS_FAULT_TIMEOUT = 2,
    AXIS_FAULT_ENCODER_LOST = 3,
    AXIS_FAULT_OVER_TRAVEL = 4,
} axis_fault_t;

typedef enum {
    AXIS_STATE_IDLE = 0,
    AXIS_STATE_MOVING = 1,
    AXIS_STATE_HOMING = 2,
    AXIS_STATE_FAULT = 3,
    AXIS_STATE_UNKNOWN = 4,
} axis_state_t;

typedef enum {
    CMD_CODE_NOP = 0,
    CMD_CODE_RELEASE = 1,
    CMD_CODE_AXIS_GOTO = 2,
    CMD_CODE_AXIS_STOP = 3,
    CMD_CODE_AXIS_HOME = 4,
} cmd_code_t;

typedef enum {
    STATUS_FLAG_LINK_OK = 0,
    STATUS_FLAG_WDT_TRIPPED = 1,
    STATUS_FLAG_ENCODER_OK = 2,
} status_flag_t;

#define OUT_BIT_COUNT 6u
#define IN_BIT_COUNT 4u
#define HREG_COUNT 5u
#define IREG_COUNT 22u

/* Границы блоков, каждый пишется одной транзакцией. */
#define COMMAND_BLOCK_START 0x0000u
#define COMMAND_BLOCK_COUNT 4u

/* Статические параметры осей. */
typedef struct {
    uint8_t  index;
    uint16_t stroke_mm;
    uint16_t step_mm;
    uint8_t  max_step;
    uint16_t speed_mm_s_x10;
    uint8_t  has_encoder;
    uint32_t step_travel_ms;
} axis_spec_t;

#define AXIS_COUNT 2u

static const axis_spec_t AXIS_SPECS[AXIS_COUNT] = {
    { .index = 0, .stroke_mm = 150, .step_mm = 1, .max_step = 150,
      .speed_mm_s_x10 = 40, .has_encoder = 1, .step_travel_ms = 250 },  /* shaft1 */
    { .index = 1, .stroke_mm = 100, .step_mm = 100, .max_step = 1,
      .speed_mm_s_x10 = 40, .has_encoder = 0, .step_travel_ms = 25000 },  /* shaft2 */
};

/* Системы сброса ПС. Импульсом, не уровнем: катушка не должна остаться
   под током. Номер устройства приходит в cmd_arg0, длительность в cmd_arg1. */
typedef struct {
    uint8_t  index;
    uint8_t  deployed_bit;
    uint8_t  lock_open_bit;
    uint16_t pulse_ms;
    uint16_t pulse_ms_max;
} release_spec_t;

#define RELEASE_COUNT 2u

static const release_spec_t RELEASE_SPECS[RELEASE_COUNT] = {
    { .index = 0, .deployed_bit = 0, .lock_open_bit = 2,
      .pulse_ms = 500, .pulse_ms_max = 2000 },  /* release1 */
    { .index = 1, .deployed_bit = 1, .lock_open_bit = 3,
      .pulse_ms = 500, .pulse_ms_max = 2000 },  /* release2 */
};

#endif /* BOARD_REGISTERS_H */
