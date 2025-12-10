import csv
import io
import os
import math
import logging
import struct
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, List, Optional, Tuple, Union

import serial
import serial.tools.list_ports
from flask import Flask, jsonify, render_template, request, send_file

app = Flask(__name__)
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


DEFAULT_BAUDRATE = 9600
MEASUREMENT_INTERVAL = 0.1
MAX_MEASUREMENTS = 1000

# Command codes
CMD_GET_NAME = 64
CMD_AV_ON = 65
CMD_AV_OFF = 66
CMD_SET_FREQ = 67
CMD_SET_BIAS = 70
CMD_RESET = 71
CMD_MEASURE = 72

LOG_FILE_PATH = "lcr_protocol_log.csv"

# Текстовые имена команд
CMD_NAMES = {
    64: "CMD_GET_NAME",
    65: "CMD_AV_ON",
    66: "CMD_AV_OFF",
    67: "CMD_SET_FREQ",
    70: "CMD_SET_BIAS",
    71: "CMD_RESET",
    72: "CMD_MEASURE",
}


def log_protocol(command_code, command_bytes, response_obj, response_bytes):
    """
    Записывает обмен с прибором в CSV-лог.

    :param command_code: int, код команды (64, 65, ...)
    :param command_bytes: bytes, отправленные байты
    :param response_obj: объект ответа (str, dict, bool)
    :param response_bytes: bytes, полученные байты
    """
    try:
        # Преобразуем байты в читаемый hex
        cmd_hex = " ".join(f"{b:02X}" for b in command_bytes)
        resp_hex = " ".join(f"{b:02X}" for b in response_bytes) if response_bytes else ""

        # Текст команды
        cmd_text = CMD_NAMES.get(command_code, f"CMD_{command_code}")

        # Текст ответа
        if isinstance(response_obj, dict):
            # Измерение — форматируем кратко
            mode = response_obj.get("mode", "?")
            val = response_obj.get("value")
            unit = response_obj.get("unit", "")
            if val is not None:
                resp_text = f"{mode} = {val} {unit}"
            else:
                resp_text = "Measurement data"
        elif isinstance(response_obj, str):
            resp_text = response_obj
        elif response_obj is True:
            resp_text = "OK"
        elif response_obj is False:
            resp_text = "ERROR"
        else:
            resp_text = str(response_obj)

        # Убедимся, что файл существует с заголовком
        file_exists = os.path.isfile(LOG_FILE_PATH)
        with open(LOG_FILE_PATH, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(["Timestamp", "Command", "Command Bytes", "Response", "Response Bytes"])
            writer.writerow([datetime.now().isoformat(), cmd_text, cmd_hex, resp_text, resp_hex])
    except Exception as e:
        logger.error(f"Ошибка записи протокола: {e}")


@dataclass
class DeviceConfig:
    frequency: float = 1000.0
    mode: str = "Z"
    speed: str = "normal"
    range: str = "auto"
    bias_voltage: float = 0.0  # в мВ
    frequency_start: float = 1000.0
    frequency_end: float = 1000.0
    frequency_step: float = 100.0
    sweep_enabled: bool = False
    # If >0, number of points in the sweep. When >1, frequency_step is computed
    # as (frequency_end - frequency_start) / (points - 1)
    points: int = 0
    # sweep_once: bool = False


class MeterState:
    def __init__(self):
        self.connection = None
        self.measurements = []
        self.is_measuring = False
        self.config = DeviceConfig()
        self.device_info = {"name": None, "id": None}
        # internal sweep state: keep track of current frequency while sweeping
        self._sweep_current: Optional[float] = None
        self._sweep_direction: int = 1
        # If using points-based sweep, track current index (0..points-1)
        self._sweep_index = 0
        self._sweep_current: Optional[float] = None
        self._manual_sweep_active = False  # флаг для режима (2)


state = MeterState()


def connect_to_meter(port: str, baudrate: int = DEFAULT_BAUDRATE) -> Tuple[bool, str]:
    """Установить последовательное соединение с измерителем LCR"""
    try:
        logger.info(f"Попытка подключиться к {port} @ {baudrate} baud")
        if state.connection and state.connection.is_open:
            logger.info("Закрываем предыдущее соединение")
            state.connection.close()
        state.connection = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=2,
            write_timeout=2,
        )
        logger.info(f"Порт {port} открыт")
        name = send_command(CMD_GET_NAME)
        logger.info(f"Имя устройства: {name}")
        if name:
            state.device_info["name"] = name
            state.device_info["id"] = "E7-28"
            return True, f"Подключено к {port}"
        else:
            disconnect_meter()
            return False, "Устройство не отвечает"

    except serial.SerialException as e:
        logger.error(f"Ошибка подключения: {str(e)}")
        return False, f"Ошибка подключения: {str(e)}"
    except Exception as e:
        logger.error(f"Неопределенная ошибка подключения: {str(e)}")
        return False, f"Неопределенная ошибка подключения: {str(e)}"


def disconnect_meter() -> Tuple[bool, str]:
    """Закрыть последовательное соединение"""
    try:
        if state.connection and state.connection.is_open:
            state.connection.close()
        state.connection = None
        state.device_info = {"name": None, "id": None}
        return True, "Подключение закрыто"
    except Exception as e:
        logger.error(f"Закрыть последовательное соединение: {str(e)}")
        return False, f"Закрыть последовательное соединение: {str(e)}"


def send_command(command: int, params: bytes = b"") -> Union[bool, str, Dict, None]:
    """Отправка команды и получение ответа с логированием протокола"""
    if not state.connection or not state.connection.is_open:
        logger.warning("Нет подключения")
        return None

    try:
        packet = bytes([0xAA, command]) + params
        state.connection.write(packet)
        time.sleep(0.1)
        response_bytes = None

        # Получаем "сырые" байты ответа для лога
        if command == CMD_GET_NAME:
            name_bytes = b""
            while True:
                b = state.connection.read(1)
                if b == b"" or b == b"\x00":
                    break
                name_bytes += b
            response_bytes = name_bytes
            response_obj = name_bytes.decode("ascii", errors="ignore")
        elif command == CMD_MEASURE:
            data = state.connection.read(22)
            response_bytes = data
            response_obj = parse_measurement(data) if len(data) >= 22 else None
        else:
            header = state.connection.read(2)
            response_bytes = header
            response_obj = len(header) == 2 and header[0] == 0xAA and header[1] == command

        # Логируем обмен
        log_protocol(command, packet, response_obj, response_bytes)

        return response_obj

    except Exception as e:
        logger.error(f"Ошибка отправки команды {command}: {str(e)}")
        return None


def handle_response(command: int) -> Union[bool, str, Dict, None]:
    """Обработка ответа от устройства"""
    try:
        if command == CMD_GET_NAME:
            name_bytes = b""
            while True:
                b = state.connection.read(1)
                if b == b"" or b == b"\x00":
                    break
                name_bytes += b
            return name_bytes.decode("ascii", errors="ignore")

        elif command == CMD_MEASURE:
            data = state.connection.read(22)
            if len(data) < 22:
                return None
            return parse_measurement(data)

        else:
            # Control commands return header echo
            header = state.connection.read(2)
            return len(header) == 2 and header[0] == 0xAA and header[1] == command

    except Exception as e:
        logger.error(f"Ошибка обработки ответа: {str(e)}")
        return None


def parse_measurement(data: bytes) -> Optional[Dict]:
    """Парсинг 22 байт данных измерения"""
    if len(data) < 22:
        return None

    try:
        flags = data[0]
        mode_code = data[1]
        speed_code = data[2]
        range_code = data[3]
        # Смещение (int16)
        bias_mv = struct.unpack("<h", data[4:6])[0] * 10
        # Частота (uint32 big-endian, freq*100)
        freq = struct.unpack(">I", data[8:12])[0] / 100.0
        # |Z| (float big-endian)
        z_mag = struct.unpack(">f", data[12:16])[0]

        # Фаза (float big-endian, радианы → градусы)
        phase_rad = struct.unpack(">f", data[16:20])[0]
        phase_deg = phase_rad * 57.2957795

        R_val = z_mag * math.cos(phase_rad)
        X_val = z_mag * math.sin(phase_rad)

        mode_map = {0: "L", 1: "C", 2: "R", 3: "Z", 4: "Y", 5: "Q", 6: "D", 7: "θ"}
        speed_map = {0: "fast", 1: "normal", 2: "average"}

        # Вычисление значения в зависимости от режима
        value, unit = None, "Ω"
        if mode_code == 0:  # L
            value = z_mag / (2 * 3.14159265 * freq) if freq != 0 else 0
            unit = "H"
        elif mode_code == 1:  # C
            value = 1 / (z_mag * 2 * 3.14159265 * freq) if z_mag != 0 and freq != 0 else 0
            unit = "F"
        elif mode_code in (2, 3):  # R или Z
            value = z_mag
            unit = "Ω"
        elif mode_code == 7:  # θ
            value = phase_deg
            unit = "°"
        else:
            value = z_mag
            unit = "Ω"

        measurement = {
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "mode_code": mode_code,
            "mode": mode_map.get(mode_code, "unknown"),
            "frequency": round(freq, 2),
            "z_mag": round(z_mag, 6),
            "phase_rad": round(phase_rad, 6),
            "phase_deg": round(phase_deg, 4),
            "R": round(R_val, 6),
            "X": round(X_val, 6),
            "bias_mv": bias_mv,
            "speed": speed_map.get(speed_code, "unknown"),
            "range": f"{10**(7-range_code)} Ω" if range_code < 8 else "auto",
            "flags": f"{bin(flags)}",
            "value": round(value, 6) if value is not None else None,
            "unit": unit,
        }

        return measurement

    except Exception as e:
        logger.error(f"Ошибка анализа измерения: {str(e)}")
        return None


def get_available_ports() -> List[str]:
    """Получить список доступных последовательных портов"""
    a = [port.device for port in serial.tools.list_ports.comports()]
    logger.info(f" Список портов COM - {a}")
    return a


def measurement_worker():
    """Фоновый воркер с поддержкой:
    - мониторинга (start == end)
    - однократного прохода (start != end, sweep_enabled=False)
    - внешнего sweep (sweep_enabled=True)
    """
    while True:
        if not (state.is_measuring and state.connection and state.connection.is_open):
            time.sleep(0.5)
            continue

        try:
            # Режим 1: прибор сам делает sweep (мы только читаем)
            if state.config.sweep_enabled:
                measurement = send_command(CMD_MEASURE)
                if measurement:
                    state.measurements.append(measurement)
                    if len(state.measurements) > MAX_MEASUREMENTS:
                        state.measurements = state.measurements[-MAX_MEASUREMENTS:]
                time.sleep(MEASUREMENT_INTERVAL)
                continue

            # Режим 2: однократный проход по частотам (start != end)
            if state.config.frequency_start != state.config.frequency_end:
                # Инициализация при первом входе
                if not hasattr(state, "_pass_initialized") or not state._pass_initialized:
                    state._pass_initialized = True
                    state._current_index = 0
                    state._current_freq = state.config.frequency_start

                    # Вычисляем точки, если задано points
                    if state.config.points and state.config.points > 1:
                        span = state.config.frequency_end - state.config.frequency_start
                        state.config.frequency_step = span / (state.config.points - 1)
                        state._total_points = state.config.points
                    else:
                        # Рассчитываем количество точек по шагу
                        if state.config.frequency_step <= 0:
                            state.config.frequency_step = 1  # защита от деления на ноль
                        span = state.config.frequency_end - state.config.frequency_start
                        state._total_points = int(span // state.config.frequency_step) + 1

                # Проверка завершения
                if state._current_index >= state._total_points:
                    state.is_measuring = False
                    state._pass_initialized = False
                    logger.info("Однократный проход завершён")
                    continue

                # Устанавливаем частоту
                freq = state.config.frequency_start + state._current_index * state.config.frequency_step
                freq = min(freq, state.config.frequency_end)  # не выйти за end
                state.config.frequency = freq
                send_command(CMD_SET_FREQ, struct.pack(">I", int(freq * 100)))
                time.sleep(0.1)

                # Измеряем
                measurement = send_command(CMD_MEASURE)
                if measurement:
                    state.measurements.append(measurement)
                    if len(state.measurements) > MAX_MEASUREMENTS:
                        state.measurements = state.measurements[-MAX_MEASUREMENTS:]

                state._current_index += 1
                time.sleep(MEASUREMENT_INTERVAL)
                continue

            # Режим 3: мониторинг на одной частоте (start == end)
            freq = state.config.frequency_start
            if state.config.frequency != freq:
                state.config.frequency = freq
                send_command(CMD_SET_FREQ, struct.pack(">I", int(freq * 100)))
                time.sleep(0.1)
            measurement = send_command(CMD_MEASURE)
            if measurement:
                state.measurements.append(measurement)
                if len(state.measurements) > MAX_MEASUREMENTS:
                    state.measurements = state.measurements[-MAX_MEASUREMENTS:]
            time.sleep(MEASUREMENT_INTERVAL)

        except Exception as e:
            logger.error(f"Ошибка в measurement_worker: {e}")
            time.sleep(0.5)


@app.route("/")
def index():
    """Главная страница"""
    return render_template("index.html", ports=get_available_ports())


@app.route("/connect", methods=["POST"])
def connect():
    """Обработка запроса на соединение"""
    port = request.form.get("port")
    baudrate_str = request.form.get("baudrate", "").strip()

    if not port:
        return jsonify({"success": False, "message": "Порт не выбран"})

    try:
        # Парсим baudrate
        baudrate = int(baudrate_str) if baudrate_str.isdigit() else DEFAULT_BAUDRATE
    except ValueError:
        baudrate = DEFAULT_BAUDRATE

    try:
        logger.info(port)
        success, message = connect_to_meter(port, baudrate)
        logger.info(port)
        response = {"success": success, "message": message}

        if success:
            response["device_info"] = state.device_info

        return jsonify(response)
    except Exception as e:
        logger.error(e)
        return jsonify({"success": False, "message": e})


@app.route("/disconnect", methods=["POST"])
def disconnect():
    """Обработка запроса на отключение"""
    success, message = disconnect_meter()
    return jsonify({"success": success, "message": message})


@app.route("/av_on", methods=["POST"])
def av_on():
    """Включить АВП (активный режим)"""
    result = send_command(CMD_AV_ON)
    return jsonify({"success": result is not None})


@app.route("/av_off", methods=["POST"])
def av_off():
    """Выключить АВП"""
    result = send_command(CMD_AV_OFF)
    return jsonify({"success": result is not None})


@app.route("/reset", methods=["POST"])
def reset():
    """Сброс устройства"""
    result = send_command(CMD_RESET)
    return jsonify({"success": result is not None})


@app.route("/config", methods=["GET", "POST"])
def handle_config():
    """Получить или обновить конфигурацию"""
    if request.method == "GET":
        return jsonify(
            {
                "frequency": state.config.frequency,
                "mode": state.config.mode,
                "speed": state.config.speed,
                "range": state.config.range,
                "bias_voltage": state.config.bias_voltage,
                "frequency_start": state.config.frequency_start,
                "frequency_end": state.config.frequency_end,
                "frequency_step": state.config.frequency_step,
                "points": state.config.points,
                "sweep_enabled": state.config.sweep_enabled,
            }
        )
    else:
        try:
            state.config.frequency = float(request.form.get("frequency", 1000))
            state.config.mode = request.form.get("mode", "Z")
            state.config.speed = request.form.get("speed", "normal")
            state.config.range = request.form.get("range", "auto")
            state.config.bias_voltage = float(request.form.get("bias_voltage", 0))

            # Sweep parameters
            try:
                state.config.frequency_start = float(request.form.get("frequency_start", state.config.frequency_start))
            except (TypeError, ValueError):
                state.config.frequency_start = state.config.frequency_start

            try:
                state.config.frequency_end = float(request.form.get("frequency_end", state.config.frequency_end))
            except (TypeError, ValueError):
                state.config.frequency_end = state.config.frequency_end

            try:
                state.config.frequency_step = float(request.form.get("frequency_step", state.config.frequency_step))
            except (TypeError, ValueError):
                state.config.frequency_step = state.config.frequency_step

            try:
                points_val = request.form.get("points", "")
                state.config.points = int(points_val) if str(points_val).strip() != "" else state.config.points
            except (TypeError, ValueError):
                state.config.points = state.config.points

            sweep_val = str(request.form.get("sweep_enabled", "")).lower()
            state.config.sweep_enabled = sweep_val in ("1", "true", "on", "yes")

            # If points > 1, compute frequency_step
            if state.config.points and state.config.points > 1:
                span = float(state.config.frequency_end) - float(state.config.frequency_start)
                state.config.frequency_step = span / (state.config.points - 1)

            if state.connection and state.connection.is_open:
                # Set frequency
                freq_bytes = struct.pack(">I", int(state.config.frequency))
                send_command(CMD_SET_FREQ, freq_bytes)

                # Set bias voltage
                bias_int16 = int(state.config.bias_voltage / 10)
                bias_bytes = struct.pack("<h", bias_int16)
                send_command(CMD_SET_BIAS, bias_bytes)

                # Initialize sweep if enabled
                if state.config.sweep_enabled and state.config.frequency_start <= state.config.frequency_end:
                    state._sweep_current = float(state.config.frequency_start)
                    state._sweep_index = 0

            return jsonify({"success": True, "message": "Настройки применены"})
        except Exception as e:
            return jsonify({"success": False, "message": str(e)})


@app.route("/measurements", methods=["POST"])
def control_measurements():
    """Запуск или остановка измерений"""
    global state

    action = request.form.get("action")
    if action == "start":
        state.is_measuring = True
        state.measurements.clear()
        state._manual_sweep_active = False
        state._sweep_index = 0
        state._sweep_current = None
        if hasattr(state, "_pass_initialized"):
            delattr(state, "_pass_initialized")
        return jsonify({"success": True, "message": "Запуск измерения"})
    elif action == "stop":
        state.is_measuring = False
        return jsonify({"success": True, "message": "Остановка измерения"})
    else:
        return jsonify({"success": False, "message": "Неверное действие"})


@app.route("/measurements", methods=["GET"])
def get_measurements():
    """Получить данные измерений"""
    return jsonify({"measurements": state.measurements[-100:], "is_measuring": state.is_measuring})


@app.route("/export", methods=["GET"])
def export_data():
    """Экспорт данных в CSV"""
    if not state.measurements:
        return jsonify({"success": False, "message": "Нет данных для экспорта"})

    output = io.StringIO()
    fieldnames = [
        "timestamp",
        "mode",
        "mode_code",
        "value",
        "unit",
        "frequency",
        "z_mag",
        "phase_deg",
        "phase_rad",
        "bias_mv",
        "speed",
        "range",
        "flags",
    ]

    writer = csv.DictWriter(output, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(state.measurements)

    output.seek(0)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"lcr_measurements_{timestamp}.csv"
    return send_file(
        io.BytesIO(output.getvalue().encode("utf-8")),
        mimetype="text/csv",
        as_attachment=True,
        download_name=filename,
    )
