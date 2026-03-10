from datetime import datetime

import navisar.pixhawk.gps_output as gps_output
from navisar.pixhawk.gps_output import UbxSerialEmitter


class _FakeSerial:
    def __init__(self, incoming=b""):
        self._incoming = bytearray(incoming)
        self.writes = []

    @property
    def in_waiting(self):
        return len(self._incoming)

    def read(self, size=1):
        chunk = bytes(self._incoming[:size])
        del self._incoming[:size]
        return chunk

    def write(self, payload):
        self.writes.append(bytes(payload))
        return len(payload)


def test_ubx_cfg_messages_receive_ack(monkeypatch):
    cfg = UbxSerialEmitter._create_ubx_message(0x06, 0x01, b"\x01")
    fake_serial = _FakeSerial(incoming=cfg)
    monkeypatch.setattr(gps_output.serial, "Serial", lambda *args, **kwargs: fake_serial)

    emitter = UbxSerialEmitter(
        port="/dev/null",
        baud=115200,
        rate_hz=5,
        fix_type=3,
        min_sats=10,
        max_sats=20,
        update_s=5,
    )

    emitter.ready(10.0)

    assert fake_serial.writes[-1] == UbxSerialEmitter._create_ack_ack(0x06, 0x01)


def test_duplicate_itow_is_not_emitted_twice(monkeypatch):
    fake_serial = _FakeSerial()
    monkeypatch.setattr(gps_output.serial, "Serial", lambda *args, **kwargs: fake_serial)

    class _FixedDatetime(datetime):
        @classmethod
        def utcnow(cls):
            return cls(2026, 3, 10, 12, 0, 0, 123000)

    monkeypatch.setattr(gps_output._dt, "datetime", _FixedDatetime)

    emitter = UbxSerialEmitter(
        port="/dev/null",
        baud=115200,
        rate_hz=5,
        fix_type=3,
        min_sats=10,
        max_sats=20,
        update_s=5,
    )

    first = emitter.send(12.0, 77.0, 1.0, 0.1, 0.0)
    writes_after_first = len(fake_serial.writes)
    second = emitter.send(12.0, 77.0, 1.0, 0.1, 0.0)

    assert first == second
    assert writes_after_first == 6
    assert len(fake_serial.writes) == 6
