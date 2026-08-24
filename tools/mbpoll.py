#!/usr/bin/env python3
"""Мастер Modbus RTU для проверки платы №1 с компьютера.

Замена modpoll на время отладки: показывает сырые байты в обе стороны,
проверяет CRC и расшифровывает регистры по именам.

    python tools/mbpoll.py                     телеметрия, 22 регистра (FC04)
    python tools/mbpoll.py holding             командные регистры (FC03)
    python tools/mbpoll.py outputs 15          маска выходов (FC06)
    python tools/mbpoll.py watch               непрерывный опрос 20 Гц
    python tools/mbpoll.py release 0 500       сброс ПС №1, импульс 500 мс
    python tools/mbpoll.py axis 1 home         хоминг оси 1 с замером времени
    python tools/mbpoll.py axis 0 goto 75      ось 0 на ступень 75
    python tools/mbpoll.py axis 0 stop         аварийный останов оси
    python tools/mbpoll.py --port COM9 ...     другой порт

Параметры линии берутся из протокола: 115200 8E1, адрес слейва 1.
Ожидаемые proto_ver и layout_crc читаются из vendor/board_registers.h —
того же файла, из которого собрана прошивка, чтобы они не разъезжались.
"""
import argparse
import re
import sys
import time
from pathlib import Path

import serial

BAUD = 115200
SLAVE = 1

VENDOR_HEADER = Path(__file__).resolve().parent.parent / "vendor" / "board_registers.h"

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

ACK_RESULTS = {
    0: "OK",
    1: "BAD_ARG",
    2: "REFUSED_SAFETY",
    3: "REFUSED_BUSY",
    4: "REFUSED_LINK_LOST",
    5: "UNKNOWN_CMD",
}

CMD_RELEASE = 1
CMD_AXIS_GOTO = 2
CMD_AXIS_STOP = 3
CMD_AXIS_HOME = 4

AXIS_STATES = {0: "IDLE", 1: "MOVING", 2: "HOMING", 3: "FAULT", 4: "UNKNOWN"}
AXIS_FAULTS = {
    0: "NONE", 1: "STALL", 2: "TIMEOUT", 3: "ENCODER_LOST", 4: "OVER_TRAVEL",
}

# Ось занимает четыре подряд идущих входных регистра, начиная с axis0_state.
AXIS_BLOCK_START = 5
AXIS_BLOCK_LEN = 4
AXIS_COUNT = 2


def expected() -> tuple[int | None, int | None]:
    """Достаёт PROTOCOL_VERSION и LAYOUT_CRC из вендоренного заголовка.

    Зашивать их сюда константами уже пробовали: значения остались от версии 2,
    и инструмент принялся ругаться на исправную плату. Заголовок лежит в этом
    же репозитории и собран вместе с прошивкой, так что он и есть источник.
    """
    try:
        text = VENDOR_HEADER.read_text(encoding="utf-8")
    except OSError:
        return None, None

    def find(name: str) -> int | None:
        m = re.search(rf"^#define\s+{name}\s+(0[xX][0-9a-fA-F]+|\d+)u?\s*$",
                      text, re.MULTILINE)
        return int(m.group(1), 0) if m else None

    return find("PROTOCOL_VERSION"), find("LAYOUT_CRC")


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
    want_ver, want_crc = expected()
    print()
    for i in range(count):
        value = int.from_bytes(reply[3 + i * 2:5 + i * 2], "big")
        name = names[i] if i < len(names) else f"reg{i}"
        mark = ""
        if name == "proto_ver":
            mark = _verdict(value, want_ver, "версия протокола")
        elif name == "layout_crc":
            mark = _verdict(value, want_crc, "CRC карты регистров", hexed=True)
        print(f"  [{i:2d}] {name:<18} {value:6d}  0x{value:04X}{mark}")

    if want_ver is None or want_crc is None:
        print()
        print(f"Не прочитался {VENDOR_HEADER} — сверить карту не с чем.")


def _verdict(got: int, want: int | None, what: str, hexed: bool = False) -> str:
    if want is None:
        return f"  <- {what}"
    if got == want:
        return f"  <- {what}, сошлось"
    shown = f"0x{want:04X}" if hexed else str(want)
    return f"  <- {what}: РАЗОШЛОСЬ, прошивка собрана под {shown}"


def _poll_regs(link: serial.Serial) -> list[int] | None:
    """Читает 22 входных регистра на уже открытом порту."""
    link.reset_input_buffer()
    link.write(wrap(bytes([SLAVE, 0x04, 0x00, 0x00, 0x00, 22])))
    reply = link.read(49)
    if len(reply) != 49 or crc16(reply[:-2]) != int.from_bytes(reply[-2:], "little"):
        return None
    return [int.from_bytes(reply[3 + i * 2:5 + i * 2], "big") for i in range(22)]


def _signed(value: int) -> int:
    """Регистры ступени и положения объявлены i16: 0xFFFF это -1, а не 65535."""
    return value - 0x10000 if value & 0x8000 else value


def follow_axis(link: serial.Serial, index: int) -> None:
    """Следит за осью до остановки и печатает, сколько занял ход.

    Ради этого команда и добавлена. Выдержки хода не покрыты автотестами ни
    на одной стороне: `step_travel_ms` не попадает в Python-артефакт, а
    selftest на плате проверяет CRC и кадры, но не время. Секундомер —
    единственный способ узнать, что число в карте похоже на правду.

    ВАЖНО: цикл не наблюдатель, а участник. Опрос мастера — это heartbeat
    платы, и пока он идёт, ход живёт. Оборвать цикл значит оборвать движение:
    через LINK_TIMEOUT_MS плата снимет ход по failsafe.
    """
    base = AXIS_BLOCK_START + AXIS_BLOCK_LEN * index
    waiting_since = time.monotonic()
    move_started: float | None = None
    print()

    try:
        while True:
            regs = _poll_regs(link)
            if regs is None:
                print("\n   плата замолчала посреди хода")
                return

            state = regs[base]
            step = _signed(regs[base + 1])
            pos = _signed(regs[base + 2])
            fault = regs[base + 3]
            moving = state in (1, 2)          # MOVING или HOMING

            if moving and move_started is None:
                move_started = time.monotonic()

            if move_started is not None:
                elapsed = time.monotonic() - move_started
                print(f"\r   {elapsed:6.1f} с  {AXIS_STATES.get(state, state):<8}"
                      f"  ступень {step:<5d} положение {pos / 10:6.1f} мм",
                      end="", flush=True)

                if not moving:
                    print()
                    print(f"   итог: {AXIS_STATES.get(state, state)}, "
                          f"неисправность {AXIS_FAULTS.get(fault, fault)}, "
                          f"ход занял {elapsed:.1f} с")
                    return

                if elapsed > 180.0:
                    print("\n   три минуты без остановки — перестаю следить")
                    return

            elif time.monotonic() - waiting_since > 2.0:
                print("   команда принята, но ось не тронулась за две секунды")
                print(f"   состояние {AXIS_STATES.get(state, state)}, "
                      f"неисправность {AXIS_FAULTS.get(fault, fault)}")
                return

            time.sleep(0.1)

    except KeyboardInterrupt:
        print()
        print("   ПРЕРВАНО — и ход прерван вместе с опросом.")
        print("   Мастер замолчал, через link_timeout_ms плата считает связь")
        print("   потерянной и снимает движение (axis_failsafe_step). Положение")
        print("   становится UNKNOWN, следующий goto вернёт REFUSED_SAFETY.")
        print(f"   Вернуть опору: python tools/mbpoll.py axis {index} home")


def send_command(port: str, code: int, arg0: int, arg1: int, twice: bool,
                 follow: int | None = None) -> int:
    """Отправляет команду с рукопожатием — так же, как это делает сервис на Pi.

    Порядок по протоколу: увеличить cmd_seq, записать блок одной FC16,
    опрашивать телеметрию, пока ack_seq не совпадёт с отправленным.
    """
    with serial.Serial(port, BAUD, bytesize=8, parity=serial.PARITY_EVEN,
                       stopbits=1, timeout=0.3) as link:
        regs = _poll_regs(link)
        if regs is None:
            print("Плата не отвечает на опрос телеметрии.")
            return 1

        seq = (regs[0] + 1) & 0xFFFF or 1        # 0 зарезервирован под «команд не было»
        body = (bytes([SLAVE, 0x10, 0x00, 0x00, 0x00, 4, 8])
                + seq.to_bytes(2, "big") + code.to_bytes(2, "big")
                + arg0.to_bytes(2, "big") + arg1.to_bytes(2, "big"))
        frame = wrap(body)

        attempts = 2 if twice else 1
        acked: int | None = None
        for n in range(attempts):
            label = "повтор того же кадра" if n else "команда"
            print(f"\n{label}: seq={seq} code={code} arg0={arg0} arg1={arg1}")
            print(f"-> {frame.hex(' ').upper()}")
            link.reset_input_buffer()
            link.write(frame)
            reply = link.read(8)
            print(f"<- {reply.hex(' ').upper() if reply else '(тишина)'}")

            for _ in range(5):
                time.sleep(0.05)
                regs = _poll_regs(link)
                if regs and regs[0] == seq:
                    acked = regs[1]
                    result = ACK_RESULTS.get(acked, f"неизвестно ({acked})")
                    print(f"   ack_seq={regs[0]}  ack_result={acked} {result}")
                    break
            else:
                print("   квитанция не пришла")

        # Следим только за принятой командой: гоняться за осью после отказа
        # незачем, там и движения не будет.
        if follow is not None and acked == 0:
            follow_axis(link, follow)

    if twice:
        print("\nОба кадра байт в байт одинаковы, квитанция после повтора та же.")
        print("Так это и должно выглядеть со стороны мастера: повтор безопасен,")
        print("номер и результат не меняются.")
        print("Заметь: сама по себе одинаковая квитанция ещё не доказывает, что")
        print("исполнение было однократным — при переисполнении она выглядела бы")
        print("так же. Однократность доказывает selftest (cmd:repeat_not_executed),")
        print("а на железе её будет видно по катушке, когда появится пин.")
    return 0


def watch(port: str) -> None:
    """Непрерывный опрос 20 Гц — так же, как это делает сервис на Pi.

    Отдельными запусками скрипта LINK_OK не увидеть: старт интерпретатора
    занимает больше 500 мс, и таймаут связи успевает истечь между ними.
    """
    request = wrap(bytes([SLAVE, 0x04, 0x00, 0x00, 0x00, 22]))
    print("Опрос 20 Гц. Ctrl+C для выхода.\n")

    with serial.Serial(port, BAUD, bytesize=8, parity=serial.PARITY_EVEN,
                       stopbits=1, timeout=0.2) as link:
        polls = errors = 0
        while True:
            link.reset_input_buffer()
            link.write(request)
            reply = link.read(49)
            polls += 1

            if len(reply) != 49 or crc16(reply[:-2]) != int.from_bytes(reply[-2:], "little"):
                errors += 1
            else:
                reg = [int.from_bytes(reply[3 + i * 2:5 + i * 2], "big") for i in range(22)]
                uptime = (reg[14] << 16) | reg[13]
                flags = reg[4]
                link_ok = "LINK_OK" if flags & 1 else "-------"
                print(f"\ruptime {uptime / 1000:8.1f} с   flags 0x{flags:04X} {link_ok}   "
                      f"crc_err {reg[15]:<4d} frame_err {reg[16]:<4d} "
                      f"do_actual 0x{reg[2]:02X}   опросов {polls}, сбоев {errors}",
                      end="", flush=True)

            time.sleep(0.05)


def _number(text: str, what: str) -> int | None:
    try:
        return int(text, 0)
    except ValueError:
        print(f"{what} должно быть числом, получено {text!r}.")
        return None


def cmd_release(args: argparse.Namespace) -> int:
    """RELEASE: номер устройства в arg0, длительность импульса в arg1.

    Раньше здесь длительность уезжала в arg0, а в arg1 шёл ноль — раскладка
    от BUOY_RELEASE, до того как в карте v4 появилось второе устройство.
    Плата читала это как «сбросить ПС №500 импульсом 0 мс».
    """
    if not args.params:
        print("Нужен номер устройства: python tools/mbpoll.py release 0 [мс]")
        print("В arg0 едет индекс: 0 — система сброса ПС №1, 1 — №2.")
        return 2

    device = _number(args.params[0], "Номер устройства")
    if device is None:
        return 2
    pulse = 500
    if len(args.params) > 1:
        pulse = _number(args.params[1], "Длительность")
        if pulse is None:
            return 2

    print(f"Сброс ПС №{device + 1}: arg0={device} (индекс), arg1={pulse} мс")
    return send_command(args.port, CMD_RELEASE, device, pulse, args.twice)


def cmd_axis(args: argparse.Namespace) -> int:
    if len(args.params) < 2:
        print("Нужны номер оси и действие:")
        print("  python tools/mbpoll.py axis 1 home")
        print("  python tools/mbpoll.py axis 0 goto 75")
        print("  python tools/mbpoll.py axis 0 stop")
        return 2

    index = _number(args.params[0], "Номер оси")
    if index is None:
        return 2
    if not 0 <= index < AXIS_COUNT:
        print(f"Осей всего {AXIS_COUNT}, номера 0..{AXIS_COUNT - 1}.")
        return 2

    action = args.params[1]
    if action == "home":
        return send_command(args.port, CMD_AXIS_HOME, index, 0, args.twice,
                            follow=index)
    if action == "stop":
        return send_command(args.port, CMD_AXIS_STOP, index, 0, args.twice)
    if action == "goto":
        if len(args.params) < 3:
            print("goto нужна целевая ступень: python tools/mbpoll.py axis 0 goto 75")
            return 2
        step = _number(args.params[2], "Ступень")
        if step is None:
            return 2
        return send_command(args.port, CMD_AXIS_GOTO, index, step, args.twice,
                            follow=index)

    print(f"Неизвестное действие {action!r}: бывают home, goto и stop.")
    return 2


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("command", nargs="?", default="state",
                        choices=["state", "holding", "outputs", "watch",
                                 "release", "buoy", "axis"])
    parser.add_argument("params", nargs="*",
                        help="аргументы команды, примеры в описании выше")
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--twice", action="store_true",
                        help="послать тот же кадр дважды — проверка защиты от повтора")
    args = parser.parse_args()

    if args.command in ("release", "buoy"):
        if args.command == "buoy":
            print("Команда называется release с карты v4; buoy оставлен как алиас.")
        return cmd_release(args)

    if args.command == "axis":
        return cmd_axis(args)

    if args.command == "watch":
        try:
            watch(args.port)
        except KeyboardInterrupt:
            print()
        return 0

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
        if not args.params:
            print("Нужна маска: python tools/mbpoll.py outputs 15")
            return 2
        mask = _number(args.params[0], "Маска")
        if mask is None:
            return 2
        body = bytes([SLAVE, 0x06, 0x00, 0x04]) + mask.to_bytes(2, "big")
        reply = transact(args.port, wrap(body), 8)
        if reply:
            echo = "совпало" if reply == wrap(body) else "НЕ СОВПАЛО"
            print(f"\nЭхо запроса: {echo}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
