/*
 * pdu.c — кадрирование Modbus RTU.
 */

#include "pdu.h"
#include "crc16.h"

uint16_t rd_u16be(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

void wr_u16be(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0x00FFu);
}

adu_status_t adu_check(const uint8_t *adu, uint16_t len, uint8_t slave_id, uint16_t *pdu_len)
{
    if (len < ADU_MIN_LEN) {
        return ADU_ERR_SHORT;
    }

    /* CRC на проводе лежит младшим байтом вперёд. */
    const uint16_t on_wire = (uint16_t)((uint16_t)adu[len - 2] | ((uint16_t)adu[len - 1] << 8));
    if (crc16_modbus(adu, (uint16_t)(len - 2)) != on_wire) {
        return ADU_ERR_CRC;
    }

    /* Адрес проверяется только после CRC — см. комментарий в pdu.h. */
    if (adu[0] != slave_id) {
        return ADU_ERR_ADDRESS;
    }

    if (pdu_len != 0) {
        *pdu_len = (uint16_t)(len - 3u);   /* минус адрес и минус CRC */
    }
    return ADU_OK;
}

uint16_t adu_wrap(uint8_t *buf, uint16_t len)
{
    const uint16_t crc = crc16_modbus(buf, len);
    buf[len]     = (uint8_t)(crc & 0x00FFu);
    buf[len + 1] = (uint8_t)(crc >> 8);
    return (uint16_t)(len + 2u);
}
