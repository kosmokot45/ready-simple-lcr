# fake_lcr_meter.py
import serial
import time
import struct
import random
from typing import Dict, Any
import math

# Эмулируем состояние прибора
import numpy as np
import struct
import random

import re


def normalize_com_port(port):
    if re.match(r"^COM\d+$", port, re.IGNORECASE):
        num = int(port[3:])
        if num >= 10:
            return f"\\\\.\\{port}"
    return port


SERIAL_PORT = normalize_com_port("COM6")
# Настройки
# SERIAL_PORT = r"\\.\COM11"
BAUDRATE = 9600

# Команды
CMD_GET_NAME = 64
CMD_AV_ON = 65
CMD_AV_OFF = 66
CMD_SET_FREQ = 67
CMD_SET_BIAS = 70
CMD_RESET = 71
CMD_MEASURE = 72



# Параметры модели Рэндлса (настройте под себя)
R1 = 100.0  # Ом
R2 = 1000.0  # Ом
C = 0.00000001  # Фарад (1 мкФ)


class FakeLCR:
    def __init__(self):
        self.av_enabled = False
        self.frequency = 1000.0  # Гц
        self.bias_voltage = 0.0  # мВ
        self.device_name = b"E728"

    def _impedance_randles(self, freq_hz):
        """Вычисляет Re, Im по модели Рэндлса на заданной частоте."""
        if freq_hz <= 0:
            freq_hz = 1.0
        w = 2 * np.pi * freq_hz

        denom = 1.0 + (w * R2 * C) ** 2
        Re = R1 + R2 / denom
        Im = -(w * R2 * R2 * C) / denom  # отрицательная — ёмкостной

        Z_mag = np.sqrt(Re**2 + Im**2)
        phase_rad = np.arctan2(Im, Re)

        return Z_mag, phase_rad

    def handle_command(self, data: bytes) -> bytes:
        if len(data) < 2 or data[0] != 0xAA:
            return b""

        cmd = data[1]

        if cmd == CMD_GET_NAME:
            return self._response(cmd, self.device_name)
        elif cmd in (CMD_AV_ON, CMD_AV_OFF, CMD_RESET):
            if cmd == CMD_AV_ON:
                self.av_enabled = True
            elif cmd == CMD_AV_OFF:
                self.av_enabled = False
            elif cmd == CMD_RESET:
                self.av_enabled = False
                self.frequency = 1000.0
                self.bias_voltage = 0.0
            return self._response(cmd)
        elif cmd == CMD_SET_FREQ:
            if len(data) >= 6:
                # Частота передаётся как uint32 (big-endian) — в Гц напрямую
                freq = struct.unpack(">I", data[2:6])[0]
                self.frequency = float(freq)
            return self._response(cmd)
        elif cmd == CMD_SET_BIAS:
            if len(data) >= 4:
                bias_raw = struct.unpack("<h", data[2:4])[0]
                self.bias_voltage = bias_raw * 10  # в мВ
            return self._response(cmd)
        elif cmd == CMD_MEASURE:
            z_mag, phase_rad = self._impedance_randles(self.frequency)

            # Шум (по желанию)
            z_mag *= 1.0 + random.uniform(-0.01, 0.01)
            phase_rad += random.uniform(-0.01, 0.01)

            # Флаги и параметры (как в реальном логе)
            flags = 0x99
            mode = 3  # Z
            speed = 1  # normal
            diap = 1  # диапазон

            # Смещение
            bias_val = int(self.bias_voltage / 10)
            U0, U1 = struct.pack("<h", bias_val)

            # Частота (big-endian uint32, в Гц)
            freq_bytes = struct.pack(">I", int(self.frequency))
            f0, f1, f2, f3 = freq_bytes

            # |Z| и фаза (big-endian float)
            z_bytes = struct.pack(">f", z_mag)
            fi_bytes = struct.pack(">f", phase_rad)
            z0, z1, z2, z3 = z_bytes
            fi0, fi1, fi2, fi3 = fi_bytes

            # 22 байта полезной нагрузки (без AA 48)
            payload = bytes(
                [flags, mode, speed, diap, U0, U1, 0, 0, f0, f1, f2, f3, z0, z1, z2, z3, fi0, fi1, fi2, fi3]
            )
            return payload
        else:
            print(f"Неизвестная команда: {cmd}")
            return b""

    def _response(self, cmd: int, data: bytes = b"") -> bytes:
        return bytes([0xAA, cmd]) + data


def main():

    import serial.tools.list_ports

    print([p.device for p in serial.tools.list_ports.comports()])

    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        print(f"Фейковый LCR-метр запущен на {SERIAL_PORT}")
        fake_meter = FakeLCR()

        while True:
            if ser.in_waiting > 0:
                # Читаем заголовок (минимум 2 байта)
                header = ser.read(2)
                if len(header) < 2 or header[0] != 0xAA:
                    continue

                cmd = header[1]
                params = b""
                # Читаем параметры, ели они есть
                if cmd == CMD_SET_FREQ and ser.in_waiting >= 4:
                    params = ser.read(4)
                elif cmd == CMD_SET_BIAS and ser.in_waiting >= 2:
                    params = ser.read(2)

                full_command = header + params
                response = fake_meter.handle_command(full_command)
                if response:
                    ser.write(response)
                    print(f"← Ответ: {list(response)}")

            time.sleep(0.01)  # Не нагружаем CPU

    except serial.SerialException as e:
        print(f"Ошибка порта {SERIAL_PORT}: {e}")
    except KeyboardInterrupt:
        print("\nФейковый прибор остановлен.")


if __name__ == "__main__":
    main()
