#!/usr/bin/env python3
"""Мастер Modbus RTU для проверки платы №1 с компьютера.

Замена modpoll на время отладки: показывает сырые байты в обе стороны,
проверяет CRC и расшифровывает регистры по именам.

    python tools/mbpoll.py                     телеметрия, 22 регистра (FC04)
    python tools/mbpoll.py holding             командные регистры (FC03)
    python tools/mbpoll.py outputs 15          маска выходов (FC06)
    python tools/mbpoll.py --port COM9 ...     другой порт

Параметры линии берутся из протокола: 115200 8E1, адрес слейва 1.
"""
import argparse
import sys

import serial

BAUD = 115200
SLAVE = 1

IREG_NAMES = [
    "ack_seq", "ack_result", "do_actual", "di_bits", "flags",
    "axis0_state", "axis0_step", "axis0_pos_mm_x10", "axis0_fault",
    "axis1_state", "axis1_step", "axis1_pos_mm_x10", "axis1_fault",
    "uptime_lo", "uptime_hi", "crc_err", "frame_err", "enc_err",
    "proto_ver", "layout_crc", "fw_build_lo", "fw_build_hi",
]
HREG_NAMES = ["cmd_seq", "cmd_code", "cmd_arg0", "cmd_arg1", "outputs"]

EXCEPTIONS = {
    0x01: "Illegal Function",
    0x02: "Illegal Data Address",
    0x03: "Illegal Data Value",
}


def crc16(data: bytes) -> int:
    """Тот же алгоритм, что в modbus/crc16.c."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def wrap(body: bytes) -> bytes:
    return body + crc16(body).to_bytes(2, "little")


def transact(port: str, request: bytes, expect: int) -> bytes | None:
    """Отправляет кадр и читает ответ. Печатает обе стороны обмена."""
    with serial.Serial(port, BAUD, bytesize=8, parity=serial.PARITY_EVEN,
                       stopbits=1, timeout=0.5) as link:
        link.reset_input_buffer()
        link.write(request)
        print(f"-> {request.hex(' ').upper()}")
        reply = link.read(expect)

    if not reply:
        print("<- (тишина)")
        print()
        print("Плата не ответила. Возможные причины:")
        print("  - TX платы (PC4) не подключён к RX переходника;")
        print("  - порт занят другой программой — закрой терминал;")
        print("  - расходятся параметры линии: нужно 115200 8E1.")
        return None

    print(f"<- {reply.hex(' ').upper()}")

    if len(reply) >= 4 and crc16(reply[:-2]) != int.from_bytes(reply[-2:], "little"):
        print("CRC ответа не сошёлся — байты искажаются на линии.")
        return None

    if len(reply) >= 3 and reply[1] & 0x80:
        code = reply[2]
        print(f"Исключение {code:#04x}: {EXCEPTIONS.get(code, 'неизвестное')}")
        return None

    return reply


def show_registers(reply: bytes, names: list[str]) -> None:
    count = reply[2] // 2
    print()
    for i in range(count):
        value = int.from_bytes(reply[3 + i * 2:5 + i * 2], "big")
        name = names[i] if i < len(names) else f"reg{i}"
        mark = ""
        if name == "proto_ver":
            mark = "  <- версия протокола, ожидается 2"
        elif name == "layout_crc":
            mark = "  <- CRC карты регистров, ожидается 12207 (0x2FAF)"
        print(f"  [{i:2d}] {name:<18} {value:6d}  0x{value:04X}{mark}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", nargs="?", default="state",
                        choices=["state", "holding", "outputs"])
    parser.add_argument("value", nargs="?", type=lambda s: int(s, 0))
    parser.add_argument("--port", default="COM8")
    args = parser.parse_args()

    if args.command == "state":
        request = wrap(bytes([SLAVE, 0x04, 0x00, 0x00, 0x00, 22]))
        reply = transact(args.port, request, 3 + 22 * 2 + 2)
        if reply:
            show_registers(reply, IREG_NAMES)

    elif args.command == "holding":
        request = wrap(bytes([SLAVE, 0x03, 0x00, 0x00, 0x00, 5]))
        reply = transact(args.port, request, 3 + 5 * 2 + 2)
        if reply:
            show_registers(reply, HREG_NAMES)

    else:
        if args.value is None:
            print("Нужна маска: python tools/mbpoll.py outputs 15")
            return 2
        body = bytes([SLAVE, 0x06, 0x00, 0x04]) + args.value.to_bytes(2, "big")
        reply = transact(args.port, wrap(body), 8)
        if reply:
            echo = "совпало" if reply == wrap(body) else "НЕ СОВПАЛО"
            print(f"\nЭхо запроса: {echo}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
