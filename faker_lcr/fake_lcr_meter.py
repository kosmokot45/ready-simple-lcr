# fake_lcr_meter.py
import serial
import time
import struct
import random
from typing import Dict, Any


import re


def normalize_com_port(port):
    if re.match(r"^COM\d+$", port, re.IGNORECASE):
        num = int(port[3:])
        if num >= 10:
            return f"\\\\.\\{port}"
    return port


SERIAL_PORT = normalize_com_port("COM11")
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


# Эмулируем состояние прибора
class FakeLCR:
    def __init__(self):
        self.av_enabled = False
        self.frequency = 1000.0
        self.bias_voltage = 0.0  # в мВ
        self.device_name = b"E728"

    def handle_command(self, data: bytes) -> bytes:
        if len(data) < 2 or data[0] != 0xAA:
            return b""

        cmd = data[1]

        if cmd == CMD_GET_NAME:
            return self._response(cmd, self.device_name)

        elif cmd == CMD_AV_ON:
            self.av_enabled = True
            return self._response(cmd)

        elif cmd == CMD_AV_OFF:
            self.av_enabled = False
            return self._response(cmd)

        elif cmd == CMD_SET_FREQ:
            if len(data) >= 6:
                freq_bytes = data[2:6]
                # accept both big-endian and little-endian encodings
                try:
                    self.frequency = struct.unpack(">I", freq_bytes)[0] / 100.0
                except struct.error:
                    try:
                        self.frequency = struct.unpack("<I", freq_bytes)[0] / 100.0
                    except struct.error:
                        pass
            return self._response(cmd)

        elif cmd == CMD_SET_BIAS:
            if len(data) >= 4:
                bias_raw = struct.unpack("<h", data[2:4])[0]
                self.bias_voltage = bias_raw * 10  # *10 → мВ
            return self._response(cmd)

        elif cmd == CMD_RESET:
            self.av_enabled = False
            self.frequency = 1000
            self.bias_voltage = 0
            return self._response(cmd)

        elif cmd == CMD_MEASURE:
            # Эмулируем данные
            z_mag = self._simulate_impedance()
            phase_rad = self._simulate_phase()
            flags = 0x80  # бит 7 = завершено
            mode = 3  # Z
            speed = 1  # норма
            diap = 5  # 100 Ом
            freq = int(self.frequency)

            # Смещение: int16 → два байта (little-endian)
            Uсм_val = int(self.bias_voltage / 10)
            Uсм_bytes = struct.pack("<h", Uсм_val)
            Uсм0, Uсм1 = Uсм_bytes[0], Uсм_bytes[1]

            # Частота: uint32 → 4 байта (service expects freq*100, big-endian)
            freq_int = int(freq * 100)
            freq_bytes = struct.pack(">I", freq_int)
            f0, f1, f2, f3 = freq_bytes[0], freq_bytes[1], freq_bytes[2], freq_bytes[3]

            # |Z|: float → 4 байта big-endian
            z_bytes = struct.pack(">f", z_mag)
            z0, z1, z2, z3 = z_bytes[0], z_bytes[1], z_bytes[2], z_bytes[3]

            # Фаза: float → 4 байта big-endian
            fi_bytes = struct.pack(">f", phase_rad)
            fi0, fi1, fi2, fi3 = fi_bytes[0], fi_bytes[1], fi_bytes[2], fi_bytes[3]

            # Собираем 22 байта полезной нагрузки (без заголовка) в формате,
            # который ожидает `service.parse_measurement`:
            # [flags(1), mode(1), speed(1), range(1), Uсм0(1), Uсм1(1), pad0(1), pad1(1), freq(4), z(4), fi(4), pad(2)]
            data = bytes(
                [
                    flags,
                    mode,
                    speed,
                    diap,
                    Uсм0,
                    Uсм1,
                    0,
                    0,  # padding to match expected offsets
                    f0,
                    f1,
                    f2,
                    f3,
                    z0,
                    z1,
                    z2,
                    z3,
                    fi0,
                    fi1,
                    fi2,
                    fi3,
                    0,
                    0,  # final padding to reach 22 bytes
                ]
            )

            # For measurement command we return payload *without* the 0xAA,cmd header
            return data

        else:
            print(f"Неизвестная команда: {cmd}")
            return b""

    def _simulate_impedance(self) -> float:
        base = 1000.0
        noise = random.uniform(-50, 50)
        if self.frequency < 100:
            drift = 200
        elif self.frequency > 10000:
            drift = -100
        else:
            drift = 0
        return max(base + noise + drift, 1.0)

    def _simulate_phase(self) -> float:
        phase_deg = 0.0
        if self.frequency > 5000:
            phase_deg = random.uniform(-5, -1)
        elif self.frequency < 100:
            phase_deg = random.uniform(1, 5)
        else:
            phase_deg = random.uniform(-2, 2)
        return phase_deg * (3.14159265 / 180)  # в радианы

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
                # Читаем параметры, если они есть
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
