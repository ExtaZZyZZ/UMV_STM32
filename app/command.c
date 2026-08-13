/*
 * command.c — исполнение команд с рукопожатием по cmd_seq.
 *
 * У Modbus RTU нет ни номера транзакции, ни корреляции запроса с ответом.
 * Для сброса спасательного круга это неприемлемо: MQTT-сторона идёт с QoS 2,
 * повтор кадра на линии возможен, а круг должен уйти один раз.
 * Идемпотентность возвращается тем, что команда исполняется, только если
 * cmd_seq отличается от последнего ИСПОЛНЕННОГО.
 */

#include "command.h"

#include "main.h"
#include "board_registers.h"
#include "regfile.h"
#include "uart_link.h"

/* Предел длительности импульса замка. В протоколе не задан — это ограничение
 * со стороны прошивки: катушка eboxlock рассчитана на кратковременное
 * включение, и мастер не должен иметь возможности попросить «подать на минуту».
 * Номинал 500 мс, четырёхкратный запас. Величину стоит подтвердить по железу. */
#define BUOY_PULSE_MAX_MS 2000u

static uint16_t last_seq;        /* последний ИСПОЛНЕННЫЙ номер команды */

static uint8_t  buoy_active;
static uint32_t buoy_started_ms;
static uint32_t buoy_duration_ms;

/* Единственное место модуля, зависящее от железа. Номер ноги и активный
 * уровень берутся из схемы «МТТКХ_ Полезная нагрузка.pdf»; пока они не
 * подтверждены, функция пустая, а вся логика вокруг неё уже настоящая. */
static void buoy_coil_set(uint8_t on)
{
    (void)on;
}

static uint8_t link_is_up(void)
{
    const mb_slave_t *s = uart_link_slave();
    return s->link_ever_valid && ((HAL_GetTick() - s->last_valid_ms) < LINK_TIMEOUT_MS);
}

static uint16_t do_buoy_release(uint16_t duration_ms)
{
    if (duration_ms == 0u || duration_ms > BUOY_PULSE_MAX_MS) {
        return ACK_RESULT_BAD_ARG;
    }

    /* Только запускаем. Снятие делает command_pulse_step на 1 кГц и делает
     * его безусловно — катушка не должна остаться под током, даже если связь
     * оборвётся через миллисекунду после подачи. */
    buoy_started_ms  = HAL_GetTick();
    buoy_duration_ms = duration_ms;
    buoy_active      = 1u;
    buoy_coil_set(1u);

    return ACK_RESULT_OK;
}

static uint16_t do_axis(uint16_t code, uint16_t axis, uint16_t step)
{
    if (axis >= AXIS_COUNT) {
        return ACK_RESULT_BAD_ARG;
    }
    if (code == CMD_CODE_AXIS_GOTO && step > AXIS_SPECS[axis].max_step) {
        return ACK_RESULT_BAD_ARG;
    }

    /* README §7: команды AXIS_* отклоняются с REFUSED_LINK_LOST до первой
     * валидной транзакции. На практике сработать почти не может — сама
     * команда и есть валидная транзакция, — но проверка дешёвая и по спецификации. */
    if (!link_is_up()) {
        return ACK_RESULT_REFUSED_LINK_LOST;
    }

    /* Осей ещё нет. Отвечаем честным отказом, а не молчанием и не ложным OK:
     * мастер сразу видит, что движение не выполнено. Здесь появится вызов
     * axis_goto / axis_stop / axis_home, когда будет axis.c. */
    return ACK_RESULT_REFUSED_SAFETY;
}

static uint16_t command_execute(uint16_t code, uint16_t arg0, uint16_t arg1)
{
    switch (code) {
        case CMD_CODE_NOP:
            return ACK_RESULT_OK;

        case CMD_CODE_BUOY_RELEASE:
            return do_buoy_release(arg0);

        case CMD_CODE_AXIS_GOTO:
        case CMD_CODE_AXIS_STOP:
        case CMD_CODE_AXIS_HOME:
            return do_axis(code, arg0, arg1);

        default:
            return ACK_RESULT_UNKNOWN_CMD;
    }
}

void command_init(void)
{
    last_seq         = 0u;
    buoy_active      = 0u;
    buoy_started_ms  = 0u;
    buoy_duration_ms = 0u;
    buoy_coil_set(0u);
}

void command_step(void)
{
    const uint16_t seq = regfile_get_holding(HREG_CMD_SEQ);

    if (seq == 0u) {
        return;              
    }
    if (seq == last_seq) {
        return;             
    }

    const uint16_t result = command_execute(regfile_get_holding(HREG_CMD_CODE),
                                            regfile_get_holding(HREG_CMD_ARG0),
                                            regfile_get_holding(HREG_CMD_ARG1));


    last_seq = seq;


    regfile_set_input(IREG_ACK_RESULT, result);
    regfile_set_input(IREG_ACK_SEQ,    seq);
}

void command_pulse_step(void)
{
    if (!buoy_active) {
        return;
    }

    /* Разность беззнаковых — переполнение счётчика тиков переживает. */
    if ((HAL_GetTick() - buoy_started_ms) >= buoy_duration_ms) {
        buoy_coil_set(0u);
        buoy_active = 0u;
    }
}

uint8_t command_buoy_active(void)
{
    return buoy_active;
}
