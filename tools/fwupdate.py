#!/usr/bin/env python3
"""Обновление платы №1 через Modbus RTU и MCUboot. Для DATA порт должен иметь одного мастера."""

import argparse
from dataclasses import dataclass
import hashlib
from pathlib import Path
import struct
import sys
import time
import zlib

import serial

from mbpoll import BAUD, SLAVE, crc16, expected, wrap


BASE = 0x0100
STATUS_WORDS = 16
WIRE_VERSION = 1
MAX_IMAGE = 0x32800
CHUNK = 232
POLL_S = 0.075
# Не подтверждать пробную прошивку одним только входящим трафиком при неисправном TX.
RECOVERY_POLL_S = 0.7
BOOT_TIMEOUT_S = 150
IDLE, ERASING, RECEIVING, VERIFIED, ERROR, VERIFYING, REBOOT_PENDING = range(7)
BEGIN, DATA, VERIFY, INSTALL, ROLLBACK, ABORT = range(1, 7)
STATE_NAMES = ("IDLE", "ERASING", "RECEIVING", "VERIFIED", "ERROR", "VERIFYING", "REBOOT_PENDING")
ERROR_NAMES = ("none", "state", "size", "offset", "flash", "crc", "image", "calibration", "boot", "argument")

Version = tuple[int, int, int, int]


class UpdateError(Exception):
    pass


class TransportError(UpdateError):
    """Доставка или ответ не подтверждены. Побочные эффекты команды возможны."""


class ModbusError(UpdateError):
    """Корректная квитанция с исключением устройства."""


def version_text(version: Version | None) -> str:
    return "нет" if version is None else f"{version[0]}.{version[1]}.{version[2]}+{version[3]}"


def check_image(raw: bytes) -> Version:
    """Проверяет структуру и SHA-256 образа перед стиранием слота."""
    if not 32 <= len(raw) <= MAX_IMAGE:
        raise UpdateError(f"Размер образа MCUboot .bin вне диапазона 32..{MAX_IMAGE} байт")
    magic, load, header, protected, size, flags = struct.unpack_from("<IIHHII", raw)
    if magic != 0x96F3B83D or header != 0x200 or size < 8 or load or flags:
        raise UpdateError("Ожидался обычный образ MCUboot для flash с заголовком 0x200")
    pos = header + size
    hash_end = pos + protected
    digest = None
    blocks = [(0x6908, protected)] if protected else []
    blocks.append((0x6907, None))
    for want_magic, want_size in blocks:
        if pos + 4 > len(raw):
            raise UpdateError("Отсутствует заголовок TLV")
        tlv_magic, total = struct.unpack_from("<HH", raw, pos)
        end = pos + total
        if tlv_magic != want_magic or total < 4 or end > len(raw) or (want_size is not None and total != want_size):
            raise UpdateError("Повреждён заголовок TLV или длина секции")
        pos += 4
        while pos < end:
            if pos + 4 > end:
                raise UpdateError("Обрезан заголовок записи TLV")
            kind, length = struct.unpack_from("<HH", raw, pos)
            pos += 4
            if pos + length > end:
                raise UpdateError("Запись TLV выходит за пределы секции")
            if want_magic == 0x6907 and kind == 0x10:
                if digest is not None:
                    raise UpdateError("В файле более одной записи SHA-256")
                digest = raw[pos:pos + length]
            pos += length
    if digest != hashlib.sha256(raw[:hash_end]).digest():
        raise UpdateError("Отсутствует или не совпадает SHA-256 образа")
    if pos != len(raw):
        raise UpdateError("Нужен образ MCUboot .bin без --pad. Последняя секция TLV должна заканчиваться в конце файла")
    return struct.unpack_from("<BBHI", raw, 20)


@dataclass(frozen=True)
class Status:
    state: int
    error: int
    offset: int
    current: Version | None
    confirmed: bool
    trial: bool
    secondary: Version | None

    @classmethod
    def decode(cls, words):
        if words[15] != WIRE_VERSION:
            raise UpdateError(f"Версия протокола обновления {words[15]}, ожидалась {WIRE_VERSION}")
        flags = words[9]
        def version(i):
            return words[i], words[i + 1], words[i + 2], words[i + 3] | (words[i + 4] << 16)
        if words[0] >= len(STATE_NAMES):
            raise UpdateError(f"Неизвестное состояние обновления {words[0]}")
        return cls(words[0], words[1], words[2] | (words[3] << 16),
                   version(4) if flags & 1 else None, bool(flags & 2), bool(flags & 8),
                   version(10) if flags & 4 else None)

    def __str__(self):
        error = ERROR_NAMES[self.error] if self.error < len(ERROR_NAMES) else str(self.error)
        return (f"{STATE_NAMES[self.state]}, ошибка={error}, смещение={self.offset}, "
                f"прошивка={version_text(self.current)}, подтверждена={self.confirmed}, "
                f"пробная={self.trial}, слот 2={version_text(self.secondary)}")


class Board:
    def __init__(self, port):
        self.port = port
        self.link = None

    def close(self):
        if self.link is not None:
            link, self.link = self.link, None
            link.close()

    def exchange(self, body, reply_size):
        frame = wrap(body)
        try:
            if self.link is None:
                self.link = serial.Serial(self.port, BAUD, bytesize=8, parity=serial.PARITY_EVEN,
                                          stopbits=1, timeout=0.22, write_timeout=1)
            self.link.reset_input_buffer()
            if self.link.write(frame) != len(frame):
                raise TransportError("Кадр UART отправлен не полностью")
            header = self.link.read(2)
            if len(header) != 2 or header[0] != SLAVE or header[1] not in (body[1], body[1] | 0x80):
                raise TransportError("Таймаут или неожиданный адрес/код ответа Modbus")
            length = 5 if header[1] & 0x80 else reply_size
            reply = header + self.link.read(length - 2)
        except OSError as exc:
            try:
                self.close()
            except OSError:
                pass  # Отключённому USB-UART иногда не удаётся даже штатное закрытие.
            raise TransportError(f"{self.port}: {exc}") from exc
        if len(reply) != length or crc16(reply[:-2]) != int.from_bytes(reply[-2:], "little"):
            raise TransportError("Неполный ответ или ошибка CRC16")
        if reply[1] & 0x80:
            raise ModbusError(f"Modbus FC{body[1]:02X}: исключение 0x{reply[2]:02X}")
        return reply

    def read_words(self, start, count):
        request = struct.pack(">BBHH", SLAVE, 4, start, count)
        reply = self.exchange(request, 5 + 2 * count)
        if reply[2] != count * 2:
            raise TransportError("FC04 вернул другое число байтов")
        return struct.unpack_from(f">{count}H", reply, 3)

    def status(self):
        return Status.decode(self.read_words(BASE, STATUS_WORDS))

    def command(self, op, arg=0, data=b""):
        count = 4 + len(data) // 2
        request = struct.pack(">BBHHBHHHH", SLAVE, 16, BASE, count, count * 2,
                              op, arg & 0xFFFF, arg >> 16, len(data)) + data
        try:
            reply = self.exchange(request, 8)
        except ModbusError as exc:
            try:
                reason = self.status()
            except UpdateError:
                raise exc
            raise UpdateError(f"{exc}; состояние платы: {reason}") from exc
        if reply[2:6] != request[2:6]:
            raise TransportError("Квитанция FC16 относится к другому адресу или размеру")


def read_status(board, deadline):
    while True:
        try:
            return board.status()
        except TransportError:
            if time.monotonic() >= deadline:
                raise
            time.sleep(POLL_S)


def wait_state(board, wanted, waiting, timeout=20):
    deadline = time.monotonic() + timeout
    while True:
        st = read_status(board, deadline)
        if st.state == wanted and not st.error:
            return st
        if st.state not in waiting or st.error or time.monotonic() >= deadline:
            raise UpdateError(f"Ожидалось {STATE_NAMES[wanted]}, плата сообщает {st}")
        time.sleep(POLL_S)


def start_phase(board, op, arg, wanted, waiting):
    for attempt in range(3):
        try:
            board.command(op, arg)
        except TransportError:
            st = read_status(board, time.monotonic() + 3)
            if st.state == wanted and not st.error:
                return st
            # Повторяем только исходное состояние операции. ABORT идемпотентен.
            not_started = ((op == BEGIN and st.state == IDLE) or
                           (op == VERIFY and st.state == RECEIVING) or
                           (op == ABORT and st.state != REBOOT_PENDING))
            if not_started and attempt < 2:
                continue
        return wait_state(board, wanted, waiting)


def write_chunk(board, offset, data):
    end = offset + len(data)
    for _ in range(3):
        try:
            board.command(DATA, offset, data)
        except TransportError:
            pass
        st = read_status(board, time.monotonic() + 3)
        if st.state != RECEIVING or st.error:
            raise UpdateError(f"Запись блока {offset}: {st}")
        if st.offset == end:
            return
        if st.offset != offset:
            raise UpdateError(f"Блок {offset}..{end}: неожиданное смещение {st.offset}")
        # Слот на прежнем смещении: повторяем только тот же блок.
    raise UpdateError(f"После трёх попыток плата ожидает блок {offset}")


def wait_boot(board, before, target):
    deadline = time.monotonic() + BOOT_TIMEOUT_S
    saw_trial = False
    saw_wrong_trial = False
    last = str(before)
    while time.monotonic() < deadline:
        try:
            st = board.status()
        except UpdateError as exc:
            last = str(exc)
            time.sleep(RECOVERY_POLL_S)
            continue
        last = str(st)
        if st.state == ERROR:
            raise UpdateError(f"Ошибка платы после сброса: {st}")
        if st.state == IDLE and st.current == target:
            if st.trial and not st.confirmed and not saw_trial:
                print(f"Пробный запуск {version_text(target)}. Ожидание подтверждения...", flush=True)
                saw_trial = True
            if st.confirmed and not st.trial and (target != before.current or saw_trial):
                return st
        elif st.trial:
            # LINK_OK у STM32 определяется приёмом. Частый опрос подтвердил бы чужую версию.
            saw_wrong_trial = True
            time.sleep(RECOVERY_POLL_S)
            continue
        elif (saw_trial or saw_wrong_trial) and st.state == IDLE and st.current and st.confirmed:
            raise UpdateError(f"Плата вернулась на {version_text(st.current)}")
        time.sleep(POLL_S)
    raise UpdateError(f"За {BOOT_TIMEOUT_S} с не подтверждена новая загрузка {version_text(target)}: {last}")


def reboot(board, op, before, target):
    try:
        board.command(op)
    except TransportError:
        # MCU мог перезагрузиться до получения квитанции. Повторная операция опасна.
        pass
    return wait_boot(board, before, target)


def flash(board, raw, target):
    initial = read_status(board, time.monotonic() + 3)
    if not initial.current or not initial.confirmed or initial.trial or initial.state == REBOOT_PENDING:
        raise UpdateError(f"Для загрузки нужен подтверждённый текущий образ: {initial}")
    if initial.state != IDLE:
        start_phase(board, ABORT, 0, IDLE, {ERASING, VERIFYING})
    start_phase(board, BEGIN, len(raw), RECEIVING, {ERASING})
    for offset in range(0, len(raw), CHUNK):
        block = raw[offset:offset + CHUNK]
        write_chunk(board, offset, block + b"\xff" * (-len(block) % 8))
    start_phase(board, VERIFY, zlib.crc32(raw), VERIFIED, {VERIFYING})
    print(f"CRC32 проверена. Установка версии {version_text(target)}...", flush=True)
    return reboot(board, INSTALL, initial, target)


def rollback(board):
    initial = read_status(board, time.monotonic() + 3)
    if initial.state != IDLE or not initial.current or not initial.secondary:
        raise UpdateError(f"Нет доступной пары прошивок для отката из текущего состояния: {initial}")
    if initial.trial and initial.current == initial.secondary:
        raise UpdateError("Версии обоих слотов совпадают. По статусу нельзя подтвердить откат пробного образа")
    return reboot(board, ROLLBACK, initial, initial.secondary)


def report_map(board):
    wanted = expected()
    if None not in wanted:
        try:
            current = board.read_words(18, 2)
        except UpdateError as exc:
            print(f"Предупреждение: новая версия подтверждена, проверить карту не удалось: {exc}", file=sys.stderr)
            return
        if current != wanted:
            print(f"Предупреждение: карта платы v{current[0]} CRC 0x{current[1]:04X}, "
                  f"в vendor v{wanted[0]} CRC 0x{wanted[1]:04X}.", file=sys.stderr)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_subparsers(dest="command", required=True)
    for command, help_text in (("status", "состояние и версии слотов"), ("flash", "установить образ MCUboot с SHA-256"),
                               ("rollback", "загрузить резервную версию")):
        action = actions.add_parser(command, help=help_text)
        action.add_argument("--port", default="COM8", help="USB-UART или UART платы (по умолчанию COM8)")
        if command == "flash":
            action.add_argument("image", type=Path, help="образ MCUboot .bin с SHA-256")
    args = parser.parse_args(argv)
    board = Board(args.port)
    try:
        if args.command == "flash":
            with args.image.open("rb") as image:
                raw = image.read(MAX_IMAGE + 1)
            result = flash(board, raw, check_image(raw))
        elif args.command == "rollback":
            result = rollback(board)
        else:
            result = read_status(board, time.monotonic() + 3)
        print(result)
        if args.command != "status":
            report_map(board)
        return 0
    except (UpdateError, OSError) as exc:
        print(f"Ошибка: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("Прервано. Проверьте состояние платы командой status.", file=sys.stderr)
        return 130
    finally:
        board.close()


if __name__ == "__main__":
    sys.exit(main())

