"""Tests for the maintenance PC updater; no serial device or signing key is required."""

from dataclasses import replace
import hashlib
import struct
import unittest
from unittest.mock import Mock, patch
import zlib

import fwupdate as fw
import serial


OLD = (1, 0, 0, 0)
NEW = (1, 2, 3, 0x10203)
BASE_STATUS = fw.Status(fw.IDLE, 0, 0, OLD, True, False, None)


def image(version=NEW, protected=b"", payload=bytes(range(23)), *,
          include_hash=True, extra_tlvs=b""):
    header = struct.pack("<IIHHIIBBHII", 0x96F3B83D, 0, 0x200, len(protected), len(payload),
                         0, *version, 0)
    data = header.ljust(0x200, b"\xff") + payload + protected
    tlvs = struct.pack("<HH", 0x10, 32) + hashlib.sha256(data).digest() if include_hash else b""
    tlvs += extra_tlvs
    return data + struct.pack("<HH", 0x6907, len(tlvs) + 4) + tlvs


class SerialStub:
    def __init__(self, reply):
        self.reply = reply
        self.sent = b""

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.sent = data
        return len(data)

    def read(self, count):
        data, self.reply = self.reply[:count], self.reply[count:]
        return data


class Clock:
    def __init__(self):
        self.now = 0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Tests(unittest.TestCase):
    def test_framing_and_arg_word_order(self):
        b = fw.Board("unused")
        b.link = SerialStub(bytes.fromhex("011001000008c033"))
        b.command(fw.DATA, 0x00234567, bytes.fromhex("0102030405067778"))
        self.assertEqual(b.link.sent.hex(), "011001000008100002456700230008010203040506777852db")

    def test_status_layout_and_rtu_crc(self):
        words = (2, 0, 0x5678, 0x1234, 1, 2, 3, 0x203, 1, 15, 4, 5, 6, 0xA2, 0xB1, 1)
        b = fw.Board("unused")
        b.link = SerialStub(fw.wrap(bytes([1, 4, 32]) + struct.pack(">16H", *words)))
        self.assertEqual(b.status(),
                         fw.Status(2, 0, 0x12345678, NEW, True, True, (4, 5, 6, 0xB100A2)))
        self.assertEqual(b.link.sent.hex(), "010401000010f03a")

    def test_reject_bad_ack_and_old_board(self):
        for reply in (b"\x01", bytes.fromhex("011001000008c032"),
                      fw.wrap(bytes.fromhex("021001000004")), fw.wrap(bytes.fromhex("010601000004")),
                      fw.wrap(bytes.fromhex("011001000005"))):
            with self.subTest(reply=reply):
                b = fw.Board("unused")
                b.link = SerialStub(reply)
                with self.assertRaises(fw.TransportError):
                    b.command(fw.BEGIN, 8)
        b.link = SerialStub(bytes.fromhex("018402c2c1"))
        with self.assertRaisesRegex(fw.UpdateError, "0x02"):
            b.status()

    def test_image_accepts_sha256_only(self):
        self.assertEqual(fw.check_image(image()), NEW)

    def test_image_accepts_legacy_signature_metadata(self):
        signature = bytes.fromhex("3006020101020101")
        key_hash = struct.pack("<HH", 0x01, 32) + bytes(32)
        signature_tlv = struct.pack("<HH", 0x22, len(signature)) + signature
        self.assertEqual(fw.check_image(image(extra_tlvs=key_hash + signature_tlv)), NEW)

    def test_image_rejects_missing_invalid_or_duplicate_hash(self):
        raw = image()
        tampered = bytearray(raw)
        tampered[0x200] ^= 1
        invalid_hash = image(include_hash=False, extra_tlvs=struct.pack("<HHB", 0x10, 1, 0))
        for broken in (image(include_hash=False), invalid_hash, tampered):
            with self.subTest(size=len(broken)), self.assertRaisesRegex(fw.UpdateError, "SHA-256"):
                fw.check_image(bytes(broken))
        with self.assertRaisesRegex(fw.UpdateError, "SHA-256"):
            fw.check_image(image(extra_tlvs=raw[-36:]))

    def test_image_hash_covers_protected_tlvs(self):
        protected = struct.pack("<HHHHI", 0x6908, 12, 0x50, 4, 3)
        raw = image(protected=protected)
        self.assertEqual(fw.check_image(raw), NEW)
        tampered = bytearray(raw)
        tampered[0x200 + 23 + 8] ^= 1
        with self.assertRaisesRegex(fw.UpdateError, "SHA-256"):
            fw.check_image(bytes(tampered))
        wrong_size = struct.pack("<HHHHI", 0x6908, 11, 0x50, 4, 3)
        with self.assertRaisesRegex(fw.UpdateError, "TLV"):
            fw.check_image(image(protected=wrong_size))

    def test_image_rejects_invalid_headers_and_tlv_bounds(self):
        raw = image()
        bad_flags = bytearray(raw)
        struct.pack_into("<I", bad_flags, 16, 0x20)
        overrun_tlv = image(extra_tlvs=struct.pack("<HHB", 0x22, 4, 0))
        for broken in (raw[:30], raw[:-1], raw + b"trailer", raw + b"\xff",
                       image(payload=b"x"), bad_flags, overrun_tlv):
            with self.subTest(size=len(broken)), self.assertRaises(fw.UpdateError):
                fw.check_image(bytes(broken))

    def test_lost_data_ack_is_resolved_by_reported_offset(self):
        for first_offset, writes in ((24, 1), (16, 2)):
            with self.subTest(first_offset=first_offset):
                board = Mock()
                board.command.side_effect = [fw.TransportError("lost ACK"), None]
                st = replace(BASE_STATUS, state=fw.RECEIVING)
                board.status.side_effect = [replace(st, offset=first_offset), replace(st, offset=24)]
                fw.write_chunk(board, 16, b"abcdefgh")
                self.assertEqual(board.command.call_count, writes)
                for called in board.command.call_args_list:
                    self.assertEqual(called.args, (fw.DATA, 16, b"abcdefgh"))
        board = Mock()
        board.status.return_value = replace(BASE_STATUS, state=fw.RECEIVING, offset=8)
        with self.assertRaisesRegex(fw.UpdateError, "8"):
            fw.write_chunk(board, 16, b"abcdefgh")

    def test_lost_install_ack_does_not_confirm_the_old_version(self):
        board = Mock()
        board.command.side_effect = fw.TransportError("reboot")
        board.status.return_value = BASE_STATUS
        with patch.object(fw, "time", Clock()), patch.object(fw, "BOOT_TIMEOUT_S", 1):
            for target in (OLD, NEW):
                with self.subTest(target=target), self.assertRaises(fw.UpdateError):
                    fw.reboot(board, fw.INSTALL, BASE_STATUS, target)
        self.assertEqual(board.command.call_count, 2)

    def test_same_version_requires_fresh_trial_then_confirmation(self):
        board = Mock()
        board.status.side_effect = [
            BASE_STATUS, replace(BASE_STATUS, confirmed=False, trial=True), BASE_STATUS
        ]
        with patch.object(fw, "time", Clock()):
            self.assertEqual(fw.wait_boot(board, BASE_STATUS, OLD), BASE_STATUS)

    def test_disconnected_serial_is_reopened_and_unknown_trials_expire(self):
        device = fw.Board("test COM")
        failed = Mock()
        failed.write.side_effect = serial.SerialException("USB disconnected")
        device.link = failed
        words = (0, 0, 0, 0, 1, 2, 3, 0x203, 1, 3, 0, 0, 0, 0, 0, 1)
        good = SerialStub(fw.wrap(bytes([1, 4, 32]) + struct.pack(">16H", *words)))
        clock = Clock()
        with patch.object(fw.serial, "Serial", return_value=good) as reopen, patch.object(fw, "time", clock):
            self.assertEqual(fw.wait_boot(device, BASE_STATUS, NEW).current, NEW)
        failed.close.assert_called_once()
        self.assertEqual(reopen.call_args.args[0], "test COM")
        self.assertGreater(clock.now, 0.5)

        other = replace(BASE_STATUS, current=(9, 9, 9, 0), trial=True, confirmed=False)
        board = Mock()
        board.status.side_effect = [other, BASE_STATUS]
        clock = Clock()
        with patch.object(fw, "time", clock), self.assertRaisesRegex(fw.UpdateError, "вернулась"):
            fw.wait_boot(board, BASE_STATUS, NEW)
        self.assertGreater(clock.now, 0.5)

    def test_lost_begin_request_and_lost_rollback_ack(self):
        board = Mock()
        board.command.side_effect = [fw.TransportError("request"), None]
        receiving = replace(BASE_STATUS, state=fw.RECEIVING)
        board.status.side_effect = [BASE_STATUS, receiving]
        self.assertEqual(fw.start_phase(board, fw.BEGIN, 513, fw.RECEIVING, {fw.ERASING}), receiving)
        self.assertEqual(board.command.call_count, 2)

        board = Mock()
        board.command.side_effect = fw.TransportError("ACK lost during reset")
        current = replace(BASE_STATUS, current=NEW, secondary=OLD)
        restored = replace(BASE_STATUS, secondary=NEW)
        board.status.side_effect = [current, fw.ModbusError("not ready"), restored]
        with patch.object(fw, "time", Clock()):
            self.assertEqual(fw.rollback(board), restored)
        board.command.assert_called_once_with(fw.ROLLBACK)

    def test_flash_uses_raw_crc_and_last_chunk_ff_padding(self):
        raw = image()
        board = Mock()
        pos = 0
        writes = []
        for pos in range(0, len(raw), fw.CHUNK):
            chunk = raw[pos:pos + fw.CHUNK]
            writes.append(chunk.ljust((len(chunk) + 7) & ~7, b"\xff"))
        receiving = replace(BASE_STATUS, state=fw.RECEIVING)
        verified = replace(BASE_STATUS, state=fw.VERIFIED)
        new = replace(BASE_STATUS, current=NEW, secondary=OLD)
        accepted = []
        pos = 0
        for data in writes:
            pos += len(data)
            accepted.append(replace(receiving, offset=pos))
        board.status.side_effect = [BASE_STATUS, replace(BASE_STATUS, state=fw.ERASING), receiving,
                                    *accepted, verified, new]
        with patch.object(fw, "time", Clock()):
            self.assertEqual(fw.flash(board, raw, NEW), new)
        actual = [c.args for c in board.command.call_args_list]
        self.assertEqual(actual[0], (fw.BEGIN, len(raw)))
        self.assertEqual(actual[-2:], [(fw.VERIFY, zlib.crc32(raw)), (fw.INSTALL,)])
        self.assertEqual([c[2] for c in actual[1:-2]], writes)
        self.assertEqual(b"".join(writes)[:len(raw)], raw)


if __name__ == "__main__":
    unittest.main()

