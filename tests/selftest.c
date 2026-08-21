/*
 * selftest.c — прогон golden-векторов из vendor/crc_vectors.json.
 *
 * Все числа ниже переписаны из этого файла и менять их руками нельзя:
 * источник правды — protocol/board1_registers.yaml на стороне Pi.
 */

#include "selftest.h"

#include "main.h"
#include "crc16.h"
#include "pdu.h"
#include "regfile.h"
#include "slave.h"
#include "command.h"
#include "outputs.h"

#include <string.h>

extern UART_HandleTypeDef huart1;

#define SELFTEST_SLAVE_ID 1u

static unsigned st_passed;
static unsigned st_failed;



void st_puts(const char *s)
{
    HAL_UART_Transmit(&huart1, (const uint8_t *)s, (uint16_t)strlen(s), 100u);
}

static void st_hex16(uint16_t v)
{
    static const char digits[] = "0123456789ABCDEF";
    char buf[5];
    buf[0] = digits[(v >> 12) & 0xFu];
    buf[1] = digits[(v >> 8) & 0xFu];
    buf[2] = digits[(v >> 4) & 0xFu];
    buf[3] = digits[v & 0xFu];
    buf[4] = '\0';
    st_puts(buf);
}

static void st_udec(unsigned v)
{
    char buf[11];
    unsigned i = 10u;
    buf[i] = '\0';
    do {
        buf[--i] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v != 0u);
    st_puts(&buf[i]);
}

/* sub — необязательный суффикс имени, чтобы три проверки одного кадра
 * не печатались тремя одинаковыми строками. */
static void st_eq16s(const char *name, const char *sub, uint16_t got, uint16_t want)
{
    if (got == want) {
        st_passed++;
        st_puts("PASS ");
    } else {
        st_failed++;
        st_puts("FAIL ");
    }
    st_puts(name);
    if (sub != 0) {
        st_puts(sub);
    }
    if (got != want) {
        st_puts(" got=");
        st_hex16(got);
        st_puts(" want=");
        st_hex16(want);
    }
    st_puts("\r\n");
}

static void st_eq16(const char *name, uint16_t got, uint16_t want)
{
    st_eq16s(name, 0, got, want);
}

/* -------------------------------------------------- вектора: crc_vectors */

static const uint8_t V_ZERO[]  = { 0x00 };
static const uint8_t V_FF[]    = { 0xFF };
static const uint8_t V_ASCII[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
static const uint8_t V_0_15[]  = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                   0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };

typedef struct {
    const char    *name;
    const uint8_t *data;
    uint16_t       len;
    uint16_t       want;
} crc_case_t;

static const crc_case_t CRC_CASES[] = {
    /* len 0 — указатель валиден, но не разыменуется: проверяем, что цикл
     * не выполняется ни разу и остаётся начальное значение. */
    { "crc:empty",           V_ZERO,   0u, 0xFFFFu },
    { "crc:single_zero",     V_ZERO,   1u, 0x40BFu },
    { "crc:single_ff",       V_FF,     1u, 0x00FFu },
    { "crc:ascii_123456789", V_ASCII,  9u, 0x4B37u },
    { "crc:all_bytes_0_15",  V_0_15,  16u, 0xE7B4u },
};

/* ------------------------------------------------ вектора: frame_vectors */

static const uint8_t F_READ_STATE[] = {
    0x01, 0x04, 0x00, 0x00, 0x00, 0x16, 0x71, 0xC4
};
static const uint8_t F_WRITE_OUTPUTS[] = {
    0x01, 0x06, 0x00, 0x04, 0x00, 0x0F, 0x88, 0x0F
};
/* seq=1, RELEASE, arg0=0 (ПС №1), arg1=500 мс. */
static const uint8_t F_RELEASE[] = {
    0x01, 0x10, 0x00, 0x00, 0x00, 0x04, 0x08,
    0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x01, 0xF4, 0x9B, 0x6D
};
/* Байт в байт тот же кадр — повтор после таймаута. На уровне кадрирования он
 * неотличим от первого, и это правильно: дедуп по cmd_seq делает command.c. */
static const uint8_t F_RELEASE_REPEAT[] = {
    0x01, 0x10, 0x00, 0x00, 0x00, 0x04, 0x08,
    0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x01, 0xF4, 0x9B, 0x6D
};
/* Та же команда, другой номер устройства: seq=2, arg0=1 (ПС №2). */
static const uint8_t F_RELEASE_SECOND[] = {
    0x01, 0x10, 0x00, 0x00, 0x00, 0x04, 0x08,
    0x00, 0x02, 0x00, 0x01, 0x00, 0x01, 0x01, 0xF4, 0xF9, 0xAD
};
static const uint8_t F_AXIS_GOTO[] = {
    0x01, 0x10, 0x00, 0x00, 0x00, 0x04, 0x08,
    0x00, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x38, 0xBB
};
static const uint8_t F_EXCEPTION[] = {
    0x01, 0x84, 0x02, 0xC2, 0xC1
};

typedef struct {
    const char    *name;
    const uint8_t *adu;
    uint16_t       len;
    uint16_t       pdu_len;
} frame_case_t;

static const frame_case_t FRAME_CASES[] = {
    { "read_state",      F_READ_STATE,     8u,  5u },
    { "write_outputs",   F_WRITE_OUTPUTS,  8u,  5u },
    { "release",         F_RELEASE,        17u, 14u },
    { "release_repeat",  F_RELEASE_REPEAT, 17u, 14u },
    { "release_second",  F_RELEASE_SECOND, 17u, 14u },
    { "axis_goto",       F_AXIS_GOTO,     17u, 14u },
    { "exception",       F_EXCEPTION,      5u,  2u },
};

/* Статический, а не на стеке: 256 байт при стеке по умолчанию 1 КиБ. */
static uint8_t st_scratch[ADU_MAX_LEN];

/* ------------------------------------------------------------------ прогон */

static void run_crc_cases(void)
{
    for (unsigned i = 0; i < sizeof(CRC_CASES) / sizeof(CRC_CASES[0]); i++) {
        const crc_case_t *c = &CRC_CASES[i];
        st_eq16(c->name, crc16_modbus(c->data, c->len), c->want);
    }
}

static void run_frame_cases(void)
{
    for (unsigned i = 0; i < sizeof(FRAME_CASES) / sizeof(FRAME_CASES[0]); i++) {
        const frame_case_t *f = &FRAME_CASES[i];
        const uint16_t body = (uint16_t)(f->len - 2u);   /* адрес + PDU, без CRC */

        /* 1. кадр принимается */
        uint16_t pdu_len = 0;
        st_eq16s(f->name, "/status",
                 (uint16_t)adu_check(f->adu, f->len, SELFTEST_SLAVE_ID, &pdu_len),
                 (uint16_t)ADU_OK);

        /* 2. длина PDU — без адреса и без CRC */
        st_eq16s(f->name, "/len", pdu_len, f->pdu_len);

        /* 3. сборка даёт те же два байта CRC, что лежат в эталонном кадре */
        memcpy(st_scratch, f->adu, body);
        (void)adu_wrap(st_scratch, body);
        st_eq16s(f->name, "/wrap",
                 (uint16_t)((uint16_t)st_scratch[body] | ((uint16_t)st_scratch[body + 1u] << 8)),
                 (uint16_t)((uint16_t)f->adu[body]     | ((uint16_t)f->adu[body + 1u]     << 8)));
    }
}

static void run_reject_cases(void)
{
    /* Короче структурного минимума. */
    static const uint8_t short_frame[] = { 0x01, 0x04, 0x00 };
    st_eq16("rej:short",
            (uint16_t)adu_check(short_frame, sizeof(short_frame), SELFTEST_SLAVE_ID, 0),
            (uint16_t)ADU_ERR_SHORT);


    memcpy(st_scratch, F_READ_STATE, sizeof(F_READ_STATE));
    st_scratch[3] ^= 0x01u;
    st_eq16("rej:bad_crc",
            (uint16_t)adu_check(st_scratch, sizeof(F_READ_STATE), SELFTEST_SLAVE_ID, 0),
            (uint16_t)ADU_ERR_CRC);


    memcpy(st_scratch, F_READ_STATE, sizeof(F_READ_STATE) - 2u);
    st_scratch[0] = 0x02u;
    (void)adu_wrap(st_scratch, sizeof(F_READ_STATE) - 2u);
    st_eq16("rej:foreign_address",
            (uint16_t)adu_check(st_scratch, sizeof(F_READ_STATE), SELFTEST_SLAVE_ID, 0),
            (uint16_t)ADU_ERR_ADDRESS);


    memcpy(st_scratch, F_READ_STATE, sizeof(F_READ_STATE));
    st_scratch[0] = 0x02u;
    st_eq16("rej:crc_checked_before_address",
            (uint16_t)adu_check(st_scratch, sizeof(F_READ_STATE), SELFTEST_SLAVE_ID, 0),
            (uint16_t)ADU_ERR_CRC);
}

static void run_endian_cases(void)
{
    /* Количество регистров в запросе FC04: байты 00 16 -> 0x0016. */
    st_eq16("endian:rd_u16be", rd_u16be(&F_READ_STATE[4]), 0x0016u);

    uint8_t buf[2];
    wr_u16be(buf, 0x1234u);
    st_eq16("endian:wr_u16be",
            (uint16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]), 0x1234u);
}

static void run_regfile_cases(void)
{
    uint16_t v;

    regfile_init();

    /* Два регистра, ради которых файл сейчас и существует: Pi читает их при
     * подключении и при несовпадении не отправляет ни одной команды. */
    v = 0;
    st_eq16s("regfile:proto_ver", "/ok",
             (uint16_t)regfile_read_input(IREG_PROTO_VER, &v), 1u);
    st_eq16("regfile:proto_ver", v, (uint16_t)PROTOCOL_VERSION);

    v = 0;
    st_eq16s("regfile:layout_crc", "/ok",
             (uint16_t)regfile_read_input(IREG_LAYOUT_CRC, &v), 1u);
    st_eq16("regfile:layout_crc", v, (uint16_t)LAYOUT_CRC);

    /* Вся остальная телеметрия после init — нули. */
    v = 0xFFFFu;
    (void)regfile_read_input(IREG_ACK_SEQ, &v);
    st_eq16("regfile:rest_is_zero", v, 0u);

    /* Границы: последний допустимый адрес и первый за границей. */
    st_eq16("regfile:input_last",   (uint16_t)regfile_read_input(IREG_COUNT - 1u, &v), 1u);
    st_eq16("regfile:input_over",   (uint16_t)regfile_read_input(IREG_COUNT, &v),      0u);
    st_eq16("regfile:holding_last", (uint16_t)regfile_read_holding(HREG_COUNT - 1u, &v), 1u);
    st_eq16("regfile:holding_over", (uint16_t)regfile_read_holding(HREG_COUNT, &v),      0u);

    /* Запись команды кладёт число и ничего не исполняет. */
    st_eq16("regfile:write_ok",   (uint16_t)regfile_write_holding(HREG_OUTPUTS, 0x000Fu), 1u);
    st_eq16("regfile:write_back", regfile_get_holding(HREG_OUTPUTS), 0x000Fu);
    st_eq16("regfile:write_over", (uint16_t)regfile_write_holding(HREG_COUNT, 0x1234u),   0u);

    /* Главная проверка файла: два пространства независимы. Адрес 0x0000 в
     * holding — это cmd_seq, в input — ack_seq, и это разные числа. */
    (void)regfile_write_holding(HREG_CMD_SEQ, 0x00AAu);
    v = 0xFFFFu;
    (void)regfile_read_input(IREG_ACK_SEQ, &v);
    st_eq16("regfile:spaces_independent", v, 0u);

    /* Публикация состояния со стороны приложения. */
    regfile_set_input(IREG_DO_ACTUAL, 0x00FFu);
    v = 0;
    (void)regfile_read_input(IREG_DO_ACTUAL, &v);
    st_eq16("regfile:set_input", v, 0x00FFu);

    /* Вернуть исходное состояние, чтобы тесты не зависели от порядка. */
    regfile_init();
}

/* ------------------------------------------------- заглушки железа слейва */

/* Поддельные часы: время становится обычной переменной, которой можно
 * управлять. Иначе проверку границы кадра пришлось бы ловить в микросекундах. */
static uint32_t fake_ms;
static uint32_t fake_now_ms(void) { return fake_ms; }

/* Поддельный передатчик: вместо провода складывает ответ сюда. */
static uint8_t  cap_buf[ADU_MAX_LEN];
static uint16_t cap_len;

static void fake_transmit(const uint8_t *data, uint16_t len)
{
    cap_len = (len <= sizeof(cap_buf)) ? len : (uint16_t)sizeof(cap_buf);
    memcpy(cap_buf, data, cap_len);
}

/* Прерываний в тесте нет, защищаться не от чего. */
static void fake_critical(void) { }

static const mb_hw_ops_t FAKE_OPS = {
    .uart_transmit  = fake_transmit,
    .critical_enter = fake_critical,
    .critical_exit  = fake_critical,
};

static mb_slave_t test_slave;

/* Скармливает кадр байт за байтом, выдерживает паузу, прогоняет разбор. */
static void feed(const uint8_t *adu, uint16_t len, uint32_t gap_ticks)
{
    cap_len = 0;
    for (uint16_t i = 0; i < len; i++) {
        mb_slave_on_rx_byte(&test_slave, adu[i]);
    }
    fake_ms += gap_ticks;
    modbus_handler_task(&test_slave);
}

/* Собирает запрос вида «функция, начальный адрес, количество». */
static uint16_t make_req(uint8_t *buf, uint8_t fc, uint16_t start, uint16_t qty)
{
    buf[0] = SELFTEST_SLAVE_ID;
    buf[1] = fc;
    wr_u16be(&buf[2], start);
    wr_u16be(&buf[4], qty);
    return adu_wrap(buf, 6u);
}

/* Эталонный ответ на read_state_request сразу после regfile_init.
 * Не перечисленные элементы — нули, за это отвечает инициализация массива. */
static const uint8_t EXPECT_READ_STATE[49] = {
    [0]  = 0x01, [1]  = 0x04, [2]  = 0x2C,   /* адрес, функция, 44 байта данных */
    [40] = 0x03,                             /* регистр 18: proto_ver  = 3      */
    [41] = 0xF7, [42] = 0xC9,                /* регистр 19: layout_crc = 0xF7C9 */
    [47] = 0x70, [48] = 0x89,                /* CRC 0x8970, младшим байтом вперёд */
};

static void run_slave_cases(void)
{
    uint16_t len;

    regfile_init();
    fake_ms = 1000u;
    mb_slave_init(&test_slave, &FAKE_OPS, fake_now_ms, SELFTEST_SLAVE_ID);

    /* До первой валидной транзакции связи ещё не было — не путать с
     * «связь потеряна». Иначе плата отрапортует LINK_OK сразу после подачи
     * питания и примет команду движения, не зная, есть ли мастер вообще. */
    st_eq16("slave:link_not_yet", test_slave.link_ever_valid, 0u);

    /* Пауза в один тик кадр не закрывает: при часах с шагом 1 мс реально могло
     * пройти почти ноль, и следующий байт того же кадра ещё в пути. */
    feed(F_READ_STATE, sizeof(F_READ_STATE), 1u);
    st_eq16("slave:gap1_silent", cap_len, 0u);

    /* Второй тик — тишина заведомо дольше миллисекунды, кадр закончился. */
    fake_ms += 1u;
    modbus_handler_task(&test_slave);
    st_eq16("slave:gap2_replies", cap_len, 49u);
    st_eq16("slave:reply_bytes",
            (uint16_t)(memcmp(cap_buf, EXPECT_READ_STATE, sizeof(EXPECT_READ_STATE)) == 0), 1u);

    /* Валидная транзакция — доказательство, что мастер жив. */
    st_eq16("slave:link_alive", (uint16_t)(test_slave.last_valid_ms == fake_ms), 1u);

    /* Неизвестная функция: исключение 0x01, старший бит в коде функции взведён. */
    len = make_req(st_scratch, 0x07u, 0u, 1u);
    feed(st_scratch, len, 2u);
    st_eq16("slave:unknown_fc/len",  cap_len, 5u);
    st_eq16("slave:unknown_fc/code", cap_buf[1], 0x87u);
    st_eq16("slave:unknown_fc/ex",   cap_buf[2], 0x01u);

    /* Нулевое количество — запрос бессмыслен сам по себе. */
    len = make_req(st_scratch, 0x04u, 0u, 0u);
    feed(st_scratch, len, 2u);
    st_eq16("slave:qty_zero", cap_buf[2], 0x03u);

    /* Количество разумное, но блок кончается раньше. */
    len = make_req(st_scratch, 0x04u, 20u, 5u);
    feed(st_scratch, len, 2u);
    st_eq16("slave:out_of_range", cap_buf[2], 0x02u);

    /* Битый CRC: молчание и +1 к своему счётчику. */
    uint16_t crc_before = test_slave.crc_err;
    memcpy(st_scratch, F_READ_STATE, sizeof(F_READ_STATE));
    st_scratch[3] ^= 0x01u;
    feed(st_scratch, sizeof(F_READ_STATE), 2u);
    st_eq16("slave:bad_crc/silent",  cap_len, 0u);
    st_eq16("slave:bad_crc/counter", test_slave.crc_err, (uint16_t)(crc_before + 1u));

    /* Чужой адрес: тоже молчание, но счётчик другой. */
    uint16_t frame_before = test_slave.frame_err;
    memcpy(st_scratch, F_READ_STATE, sizeof(F_READ_STATE) - 2u);
    st_scratch[0] = 0x02u;
    (void)adu_wrap(st_scratch, sizeof(F_READ_STATE) - 2u);
    feed(st_scratch, sizeof(F_READ_STATE), 2u);
    st_eq16("slave:foreign/silent",  cap_len, 0u);
    st_eq16("slave:foreign/counter", test_slave.frame_err, (uint16_t)(frame_before + 1u));

    /* FC03 отдаёт другое адресное пространство: пять командных регистров. */
    (void)regfile_write_holding(HREG_OUTPUTS, 0x000Fu);
    len = make_req(st_scratch, 0x03u, 0u, HREG_COUNT);
    feed(st_scratch, len, 2u);
    st_eq16("slave:fc03/len",     cap_len, (uint16_t)(3u + HREG_COUNT * 2u + 2u));
    st_eq16("slave:fc03/outputs", rd_u16be(&cap_buf[3 + HREG_OUTPUTS * 2]), 0x000Fu);

    /* FC06 на эталонном кадре: маска четырёх ходовых огней в регистр outputs.
     * Ответ обязан быть эхом запроса байт в байт. */
    regfile_init();
    feed(F_WRITE_OUTPUTS, sizeof(F_WRITE_OUTPUTS), 2u);
    st_eq16("slave:fc06/echo",
            (uint16_t)(cap_len == sizeof(F_WRITE_OUTPUTS) &&
                       memcmp(cap_buf, F_WRITE_OUTPUTS, sizeof(F_WRITE_OUTPUTS)) == 0), 1u);
    st_eq16("slave:fc06/stored", regfile_get_holding(HREG_OUTPUTS), 0x000Fu);

    /* FC06 за границей блока — исключение 0x02. */
    len = make_req(st_scratch, 0x06u, HREG_COUNT, 0x0001u);
    feed(st_scratch, len, 2u);
    st_eq16("slave:fc06/out_of_range", cap_buf[2], 0x02u);

    /* FC16 на эталонном кадре release: cmd_seq=1, cmd_code=1, arg0=0, arg1=500.
     * Ответ подтверждает блок и данные не возвращает — 8 байт. */
    regfile_init();
    feed(F_RELEASE, sizeof(F_RELEASE), 2u);
    st_eq16("slave:fc16/len",      cap_len, 8u);
    st_eq16("slave:fc16/start",    rd_u16be(&cap_buf[2]), 0x0000u);
    st_eq16("slave:fc16/qty",      rd_u16be(&cap_buf[4]), 0x0004u);
    st_eq16("slave:fc16/cmd_seq",  regfile_get_holding(HREG_CMD_SEQ),  1u);
    st_eq16("slave:fc16/cmd_code", regfile_get_holding(HREG_CMD_CODE), 1u);
    st_eq16("slave:fc16/cmd_arg0", regfile_get_holding(HREG_CMD_ARG0), 500u);

    /* Счётчик байт не сходится с количеством — запрос испорчен. */
    memcpy(st_scratch, F_RELEASE, sizeof(F_RELEASE));
    st_scratch[6] = 0x06u;                                  /* было 8 */
    (void)adu_wrap(st_scratch, sizeof(F_RELEASE) - 2u); /* пересчитать CRC */
    feed(st_scratch, sizeof(F_RELEASE), 2u);
    st_eq16("slave:fc16/bad_count", cap_buf[2], 0x03u);

    /* Блок не помещается в пространство holding. */
    memcpy(st_scratch, F_RELEASE, sizeof(F_RELEASE));
    wr_u16be(&st_scratch[2], HREG_COUNT - 1u);              /* start у самого края */
    (void)adu_wrap(st_scratch, sizeof(F_RELEASE) - 2u);
    feed(st_scratch, sizeof(F_RELEASE), 2u);
    st_eq16("slave:fc16/out_of_range", cap_buf[2], 0x02u);

    regfile_init();
}

/* Собирает кадр FC16 на командный блок: seq, code, arg0, arg1. */
static uint16_t make_cmd(uint8_t *buf, uint16_t seq, uint16_t code,
                         uint16_t arg0, uint16_t arg1)
{
    buf[0] = SELFTEST_SLAVE_ID;
    buf[1] = 0x10u;
    wr_u16be(&buf[2], 0u);                 /* начальный адрес блока */
    wr_u16be(&buf[4], COMMAND_BLOCK_COUNT);
    buf[6] = (uint8_t)(COMMAND_BLOCK_COUNT * 2u);
    wr_u16be(&buf[7],  seq);
    wr_u16be(&buf[9],  code);
    wr_u16be(&buf[11], arg0);
    wr_u16be(&buf[13], arg1);
    return adu_wrap(buf, 15u);
}

static void run_command_cases(void)
{
    uint16_t v;
    uint16_t len;

    regfile_init();
    command_init();
    mb_slave_init(&test_slave, &FAKE_OPS, fake_now_ms, SELFTEST_SLAVE_ID);

    /* Эталонный кадр из crc_vectors.json: seq=1, RELEASE, ПС №1, 500 мс. */
    feed(F_RELEASE, sizeof(F_RELEASE), 2u);
    command_step();

    (void)regfile_read_input(IREG_ACK_SEQ, &v);
    st_eq16("cmd:release/ack_seq", v, 1u);
    (void)regfile_read_input(IREG_ACK_RESULT, &v);
    st_eq16("cmd:release/ack_result", v, (uint16_t)ACK_RESULT_OK);
    st_eq16("cmd:release/pulse_on", command_release_active(0u), 1u);

    /* Второе устройство ещё не тронуто: импульс подан ровно одному. Иначе
     * сброс одного ПС уводил бы оба. */
    st_eq16("cmd:release/other_idle", command_release_active(1u), 0u);

    /* ГЛАВНАЯ ПРОВЕРКА ПРОТОКОЛА. Тот же кадр байт в байт — повтор после
     * таймаута. ПС обязано сброситься один раз.
     *
     * Ставим метку в ack_result: если дубликат исполнится, он её затрёт
     * значением OK. Сравнивать сами квитанции бесполезно — у повтора тот же
     * номер и тот же результат, они одинаковы в обоих случаях. */
    regfile_set_input(IREG_ACK_RESULT, 0xBEEFu);
    feed(F_RELEASE_REPEAT, sizeof(F_RELEASE_REPEAT), 2u);
    command_step();

    (void)regfile_read_input(IREG_ACK_RESULT, &v);
    st_eq16("cmd:repeat_not_executed", v, 0xBEEFu);

    /* Новый номер — исполняется. */
    len = make_cmd(st_scratch, 2u, CMD_CODE_NOP, 0u, 0u);
    feed(st_scratch, len, 2u);
    command_step();
    (void)regfile_read_input(IREG_ACK_SEQ, &v);
    st_eq16("cmd:new_seq/ack", v, 2u);
    (void)regfile_read_input(IREG_ACK_RESULT, &v);
    st_eq16("cmd:new_seq/result", v, (uint16_t)ACK_RESULT_OK);

    /* Неизвестный код. */
    len = make_cmd(st_scratch, 3u, 0x00FFu, 0u, 0u);
    feed(st_scratch, len, 2u);
    command_step();
    (void)regfile_read_input(IREG_ACK_RESULT, &v);
    st_eq16("cmd:unknown_code", v, (uint16_t)ACK_RESULT_UNKNOWN_CMD);

    /* Номер оси вне диапазона — проверка аргументов настоящая уже сейчас. */
    len = make_cmd(st_scratch, 4u, CMD_CODE_AXIS_GOTO, AXIS_COUNT, 0u);
    feed(st_scratch, len, 2u);
    command_step();
    (void)regfile_read_input(IREG_ACK_RESULT, &v);
    st_eq16("cmd:axis_bad_index", v, (uint16_t)ACK_RESULT_BAD_ARG);

    /* Ось корректная, но связи нет: в тестах uart_link ещё не поднят, и
     * настоящий слейв обнулён — ровно случай «мастер ни разу не выходил». */
    len = make_cmd(st_scratch, 5u, CMD_CODE_AXIS_STOP, 0u, 0u);
    feed(st_scratch, len, 2u);
    command_step();
    (void)regfile_read_input(IREG_ACK_RESULT, &v);
    st_eq16("cmd:axis_link_lost", v, (uint16_t)ACK_RESULT_REFUSED_LINK_LOST);

    /* Нулевая длительность импульса замка. */
    len = make_cmd(st_scratch, 6u, CMD_CODE_RELEASE, 0u, 0u);
    feed(st_scratch, len, 2u);
    command_step();
    (void)regfile_read_input(IREG_ACK_RESULT, &v);
    st_eq16("cmd:release_zero_len", v, (uint16_t)ACK_RESULT_BAD_ARG);

    /* Эталонный кадр второго устройства: номер обязан доехать до нужной
     * катушки. Перепутанный arg0 сбросил бы не то ПС, и снаружи это выглядело
     * бы исправной работой. */
    feed(F_RELEASE_SECOND, sizeof(F_RELEASE_SECOND), 2u);
    command_step();
    (void)regfile_read_input(IREG_ACK_RESULT, &v);
    st_eq16("cmd:release2/ack_result", v, (uint16_t)ACK_RESULT_OK);
    st_eq16("cmd:release2/pulse_on", command_release_active(1u), 1u);

    /* Номер устройства вне диапазона. */
    len = make_cmd(st_scratch, 7u, CMD_CODE_RELEASE, RELEASE_COUNT, 500u);
    feed(st_scratch, len, 2u);
    command_step();
    (void)regfile_read_input(IREG_ACK_RESULT, &v);
    st_eq16("cmd:release_bad_unit", v, (uint16_t)ACK_RESULT_BAD_ARG);

    /* Длительность выше потолка из карты. */
    len = make_cmd(st_scratch, 8u, CMD_CODE_RELEASE, 0u,
                   (uint16_t)(RELEASE_SPECS[0].pulse_ms_max + 1u));
    feed(st_scratch, len, 2u);
    command_step();
    (void)regfile_read_input(IREG_ACK_RESULT, &v);
    st_eq16("cmd:release_too_long", v, (uint16_t)ACK_RESULT_BAD_ARG);

    regfile_init();
    command_init();
}

static void run_outputs_cases(void)
{
    uint16_t v;

    regfile_init();
    outputs_init();

    /* После инициализации всё погашено. */
    outputs_step();
    (void)regfile_read_input(IREG_DO_ACTUAL, &v);
    st_eq16("out:idle_off", v, 0u);

    /* Обычная маска: фары и балка. */
    (void)regfile_write_holding(HREG_OUTPUTS, 0x0003u);
    outputs_step();
    (void)regfile_read_input(IREG_DO_ACTUAL, &v);
    st_eq16("out:mask_mirrored", v, 0x0003u);
    st_eq16("out:actual_matches", outputs_actual(), 0x0003u);

    /* Мастер прислал мусор во все шестнадцать бит. Отдать это же значение в
     * do_actual нельзя: Pi увидел бы включёнными выходы, которых нет. */
    (void)regfile_write_holding(HREG_OUTPUTS, 0xFFFFu);
    outputs_step();
    (void)regfile_read_input(IREG_DO_ACTUAL, &v);
    st_eq16("out:extra_bits_dropped", v, (uint16_t)((1u << OUT_BIT_COUNT) - 1u));

    /* Failsafe: при потере связи outputs не трогается. Проверяем бездействие —
     * маска обязана пережить любое число вызовов без мастера на линии. */
    (void)regfile_write_holding(HREG_OUTPUTS, 0x0005u);
    outputs_step();
    for (unsigned i = 0; i < 50u; i++) {
        outputs_step();
    }
    (void)regfile_read_input(IREG_DO_ACTUAL, &v);
    st_eq16("out:survives_link_loss", v, 0x0005u);

    regfile_init();
    outputs_init();
}

unsigned selftest_run(void)
{
    st_passed = 0;
    st_failed = 0;

    st_puts("\r\n--- selftest ---\r\n");
    run_crc_cases();
    run_frame_cases();
    run_reject_cases();
    run_endian_cases();
    run_regfile_cases();
    run_slave_cases();
    run_command_cases();
    run_outputs_cases();

    st_puts("selftest: ");
    st_udec(st_passed);
    st_puts(" passed, ");
    st_udec(st_failed);
    st_puts(" failed\r\n\r\n");

    return st_failed;
}
