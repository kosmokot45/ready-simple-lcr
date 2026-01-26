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

import threading

app = Flask(__name__)
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

DEFAULT_BAUDRATE = 9600
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
    frequency_end: float = 100000.0
    frequency_step: float = 100.0  # Шаг по умолчанию 100 Гц
    sweep_enabled: bool = False
    points: int = 100


class MeterState:
    def __init__(self):
        self.connection = None
        self.measurements = []
        self.is_measuring = False
        self.config = DeviceConfig()
        self.device_info = {"name": None, "id": None}
        self._last_measurement_id = 0
        self._measurements_lock = threading.Lock()
        self._stop_event = threading.Event()  # Событие для остановки
        self.current_frequency = None  # Текущая установленная частота


state = MeterState()


def connect_to_meter(port: str, baudrate: int = DEFAULT_BAUDRATE) -> Tuple[bool, str]:
    """Установить последовательное соединение с измерителем LCR"""
    try:
        logger.info(f"Попытка подключиться к {port} @ {baudrate} baud")
        if state.connection and state.connection.is_open:
            logger.info("Закрываем предыдущее соединение")
            state.connection.close()
            time.sleep(0.5)  # Даем время на закрытие порта
        
        # Открываем порт с увеличенными тайм-аутами
        state.connection = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=2,  # Увеличена задержка для чтения
            write_timeout=2,  # Увеличена задержка для записи
        )
        logger.info(f"Порт {port} открыт с параметрами: {baudrate} baud, timeout=2s")
        
        # Даем прибору время на инициализацию
        time.sleep(0.5)
        
        # Очищаем буферы перед попыткой подключения
        state.connection.reset_input_buffer()
        state.connection.reset_output_buffer()
        logger.info("Буферы порта очищены")
        
        # Пытаемся получить имя устройства с несколькими попытками
        name = None
        for attempt in range(3):
            logger.info(f"Попытка получить имя устройства {attempt + 1}/3")
            name = send_command(CMD_GET_NAME)
            if name:
                logger.info(f"Имя устройства получено: {name}")
                break
            time.sleep(0.5)  # Между попытками
        
        if name:
            state.device_info["name"] = name
            state.device_info["id"] = "E7-28"
            return True, f"Подключено к {port}"
        else:
            logger.warning("Устройство не отвечает на запрос имени")
            disconnect_meter()
            return False, "Устройство не отвечает на запрос имени. Проверьте подключение и питание прибора."

    except serial.SerialException as e:
        logger.error(f"Ошибка SerialException подключения: {str(e)}")
        state.connection = None
        return False, f"Ошибка подключения к порту: {str(e)}"
    except Exception as e:
        logger.error(f"Неопределенная ошибка подключения: {str(e)}")
        state.connection = None
        return False, f"Ошибка подключения: {str(e)}"


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
        logger.debug(f"Отправка пакета: {packet.hex()} ({len(packet)} байт)")
        
        # Отправляем команду
        bytes_written = state.connection.write(packet)
        logger.debug(f"Отправлено байт: {bytes_written}")
        
        # Даем прибору время на обработку команды
        if command == CMD_GET_NAME:
            time.sleep(0.2)  # Для получения имени больше времени
        else:
            time.sleep(0.1)  # Для остальных команд

        response_bytes = None
        response_obj = None

        # Получаем "сырые" байты ответа для лога
        if command == CMD_GET_NAME:
            name_bytes = b""
            start_time = time.time()
            timeout = 3  # 3 секунды на получение имени
            
            while time.time() - start_time < timeout:
                if state.connection.in_waiting > 0:
                    b = state.connection.read(1)
                    logger.debug(f"Получен байт: {b.hex() if b else 'пусто'}")
                    
                    if b == b"" or b == b"\x00":
                        # Конец строки или пустой байт
                        break
                    name_bytes += b
                else:
                    time.sleep(0.05)  # Небольшая задержка перед повторной проверкой
            
            response_bytes = name_bytes
            response_obj = name_bytes.decode("ascii", errors="ignore").strip() if name_bytes else None
            logger.info(f"Получено имя устройства: '{response_obj}'")
            
        elif command == CMD_MEASURE:
            # Ждем данные измерения
            time.sleep(0.15)
            available = state.connection.in_waiting
            logger.debug(f"Доступно байт для чтения: {available}")
            
            if available >= 22:
                data = state.connection.read(22)
            else:
                data = state.connection.read(max(available, 20))
                
            logger.debug(f"Получено {len(data)} байт: {data.hex()}")
            response_bytes = data
            response_obj = parse_measurement(data) if len(data) >= 20 else None
            
        else:
            # Для остальных команд ожидаем хедер ответа (0xAA + код команды)
            time.sleep(0.15)
            header = state.connection.read(2)
            response_bytes = header
            
            if len(header) >= 2:
                logger.debug(f"Получен хедер ответа: {header.hex()}")
                response_obj = header[0] == 0xAA and header[1] == command
            else:
                logger.warning(f"Неполный хедер ответа: {header.hex() if header else 'пусто'}")
                response_obj = False

        # Логируем обмен
        log_protocol(command, packet, response_obj, response_bytes)

        return response_obj

    except Exception as e:
        logger.error(f"Ошибка отправки команды {command}: {str(e)}", exc_info=True)
        return None


def parse_measurement(data: bytes) -> Optional[Dict]:
    """Парсинг 22 байт данных измерения"""
    if len(data) < 20:
        logger.warning(f"Недостаточно данных: {len(data)} байт")
        return None

    try:
        # Логируем сырые данные для отладки
        logger.debug(f"Сырые данные: {data.hex()}")

        # Распаковываем структуру данных
        # Согласно протоколу E7-28:
        # Байт 0: флаги
        # Байт 1: код режима
        # Байт 2: код скорости
        # Байт 3: код диапазона
        # Байты 4-5: смещение (int16, little-endian)
        # Байты 6-7: резерв
        # Байты 8-11: частота (uint32, big-endian) в Гц
        # Байты 12-15: модуль импеданса |Z| (float, big-endian) в Омах
        # Байты 16-19: фаза (float, big-endian) в радианах

        flags = data[0]
        mode_code = data[1]
        speed_code = data[2]
        range_code = data[3]

        # Смещение (int16 little-endian, в мВ)
        bias_mv = struct.unpack("<h", data[4:6])[0]

        # Частота (uint32 big-endian) - уже в Гц, не нужно делить на 100
        freq = struct.unpack(">I", data[8:12])[0]

        # |Z| (float big-endian) в Омах
        z_mag = struct.unpack(">f", data[12:16])[0]

        # Фаза (float big-endian, радианы)
        phase_rad = struct.unpack(">f", data[16:20])[0]
        phase_deg = phase_rad * 180.0 / math.pi  # Конвертируем в градусы

        # Вычисляем R и X
        R_val = z_mag * math.cos(phase_rad)
        X_val = z_mag * math.sin(phase_rad)

        # Отладочная информация
        logger.debug(f"Частота: {freq} Гц, |Z|: {z_mag} Ом, фаза: {phase_deg}°, R: {R_val}, X: {X_val}")

        # Маппинг кодов режимов
        mode_map = {
            0: "L",  # Индуктивность
            1: "C",  # Емкость
            2: "R",  # Сопротивление
            3: "Z",  # Импеданс
            4: "Y",  # Адмиттанс
            5: "Q",  # Добротность
            6: "D",  # Коэффициент диссипации
            7: "θ",  # Фазовый угол
        }

        # Маппинг скоростей
        speed_map = {
            0: "fast",
            1: "normal",
            2: "average",
        }

        # Вычисляем значение в зависимости от режима
        value, unit = None, "Ω"
        if mode_code == 0:  # Индуктивность L
            if freq > 0:
                value = z_mag / (2 * math.pi * freq)  # L = |Z| / (2πf)
                unit = "H"
            else:
                value = 0
                unit = "H"
        elif mode_code == 1:  # Емкость C
            if freq > 0 and z_mag > 0:
                value = 1 / (2 * math.pi * freq * z_mag)  # C = 1 / (2πf|Z|)
                unit = "F"
            else:
                value = 0
                unit = "F"
        elif mode_code in (2, 3):  # Сопротивление R или импеданс Z
            value = z_mag
            unit = "Ω"
        elif mode_code == 7:  # Фазовый угол θ
            value = phase_deg
            unit = "°"
        else:
            value = z_mag
            unit = "Ω"

        # Создаем запись измерения
        # ВАЖНО: Используем сохраненную частоту вместо прочитанной из ответа
        # так как может быть задержка в обновлении частоты на приборе
        measurement_freq = state.current_frequency if state.current_frequency is not None else freq
        
        with state._measurements_lock:
            measurement = {
                "id": state._last_measurement_id,
                "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                "mode_code": mode_code,
                "mode": mode_map.get(mode_code, "unknown"),
                "frequency": measurement_freq,  # Используем установленную частоту
                "z_mag": z_mag,  # Модуль импеданса в Омах
                "phase_rad": phase_rad,
                "phase_deg": phase_deg,
                "R": R_val,  # Активная составляющая
                "X": X_val,  # Реактивная составляющая
                "abs_X": abs(X_val),  # Модуль реактивной составляющей
                "bias_mv": bias_mv,
                "speed": speed_map.get(speed_code, "unknown"),
                "range": f"Range {range_code}" if range_code < 8 else "auto",
                "flags": f"0x{flags:02x}",
                "value": value,
                "unit": unit,
            }

            state._last_measurement_id += 1

            # Логируем для проверки
            logger.info(f"Измерение: f={measurement_freq:.0f} Гц, |Z|={z_mag:.2f} Ом, R={R_val:.2f} Ом, X={X_val:.2f} Ом")

        return measurement

    except struct.error as e:
        logger.error(f"Ошибка распаковки структуры: {e}")
        logger.error(f"Данные: {data.hex()}")
        return None
    except Exception as e:
        logger.error(f"Ошибка анализа измерения: {str(e)}")
        import traceback

        logger.error(traceback.format_exc())
        return None


def get_available_ports() -> List[str]:
    """Получить список доступных последовательных портов"""
    ports = [port.device for port in serial.tools.list_ports.comports()]
    logger.info(f"Список портов COM - {ports}")
    return ports


def perform_measurements():
    """Выполнить измерения согласно настройкам"""
    logger.info("Начало измерений...")

    # Сбрасываем событие остановки
    state._stop_event.clear()

    f_start = state.config.frequency_start
    f_end = state.config.frequency_end
    points = max(state.config.points, 1)

    with state._measurements_lock:
        state.measurements.clear()

    # Режим 1: одинаковые частоты - опрашиваем прибор N раз
    if f_start == f_end:
        logger.info(f"Режим 1: измерение на частоте {f_start} Гц, {points} точек")

        # Сохраняем установленную частоту
        state.current_frequency = f_start
        
        # Устанавливаем частоту (без умножения на 100!)
        result = send_command(CMD_SET_FREQ, struct.pack(">I", int(f_start)))  # Только частота в Гц
        if result is None:
            logger.error("Не удалось установить частоту")
            state.is_measuring = False
            return

        time.sleep(0.2)  # Даем время прибору установить частоту

        # Выполняем N измерений
        for i in range(points):
            if state._stop_event.is_set() or not state.is_measuring:
                break

            measurement = send_command(CMD_MEASURE)
            if measurement:
                with state._measurements_lock:
                    state.measurements.append(measurement)
            time.sleep(0.1)

    # Режим 2: разные частоты - строим список и идем по нему
    else:
        logger.info(f"Режим 2: sweep от {f_start} до {f_end} Гц")

        # Генерация списка частот
        freqs = []
        if points > 1:
            # Равномерное распределение по точкам
            step = (f_end - f_start) / (points - 1)
            logger.info(f"Равномерное распределение: шаг = {step:.2f} Гц, points = {points}")
            freqs = [f_start + i * step for i in range(points)]
        else:
            # По шагу (если points = 1, то используем frequency_step)
            step = max(state.config.frequency_step, 1.0)
            logger.info(f"Распределение по шагу: шаг = {step:.2f} Гц")
            f = f_start
            while f <= f_end:
                freqs.append(f)
                f += step
            # Добавляем конечную частоту, если она не попала в список
            if freqs and abs(freqs[-1] - f_end) > 0.01:
                freqs.append(f_end)

        logger.info(f"Сгенерировано {len(freqs)} частот для sweep")
        logger.debug(f"Частоты: {freqs[:5]}... {freqs[-5:]}")  # Первые и последние 5

        # Проход по всем частотам
        for idx, freq in enumerate(freqs):
            if state._stop_event.is_set() or not state.is_measuring:
                logger.info(f"Остановка измерений на точке {idx}/{len(freqs)}")
                break

            # Сохраняем установленную частоту
            state.current_frequency = freq
            
            # Устанавливаем частоту (без умножения на 100!)
            result = send_command(CMD_SET_FREQ, struct.pack(">I", int(freq)))  # Только частота в Гц
            if result is None:
                logger.error(f"Не удалось установить частоту {freq} Гц")
                continue

            time.sleep(0.2)  # Даем время прибору установить частоту

            # Выполняем измерение
            measurement = send_command(CMD_MEASURE)
            if measurement:
                with state._measurements_lock:
                    state.measurements.append(measurement)
            else:
                logger.warning(f"Не получено измерение на частоте {freq} Гц")
            
            time.sleep(0.1)

    state.is_measuring = False
    state.current_frequency = None
    logger.info(f"Измерения завершены. Получено {len(state.measurements)} точек")


@app.route("/")
def index():
    """Главная страница"""
    return render_template("index.html", ports=get_available_ports())


@app.route("/ports", methods=["GET"])
def get_ports():
    """API для получения доступных COM портов"""
    ports = get_available_ports()
    logger.info(f"Запрос портов: {ports}")
    return jsonify({"ports": ports})


@app.route("/connect", methods=["POST"])
def connect():
    """Обработка запроса на соединение"""
    port = request.form.get("port")
    baudrate_str = request.form.get("baudrate", "").strip()

    if not port:
        return jsonify({"success": False, "message": "Порт не выбран"})

    try:
        baudrate = int(baudrate_str) if baudrate_str.isdigit() else DEFAULT_BAUDRATE
    except ValueError:
        baudrate = DEFAULT_BAUDRATE

    try:
        success, message = connect_to_meter(port, baudrate)
        response = {"success": success, "message": message}

        if success:
            response["device_info"] = state.device_info

        return jsonify(response)
    except Exception as e:
        logger.error(f"Ошибка подключения: {e}")
        return jsonify({"success": False, "message": str(e)})


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

    else:  # POST
        try:
            # Получаем данные из формы или JSON
            if request.content_type and "application/json" in request.content_type:
                data = request.get_json()
            else:
                data = request.form.to_dict()

            logger.info(f"Получены данные конфигурации: {data}")

            # Обновляем конфигурацию
            if "frequency_start" in data:
                state.config.frequency_start = float(data.get("frequency_start", state.config.frequency_start))
            if "frequency_end" in data:
                state.config.frequency_end = float(data.get("frequency_end", state.config.frequency_end))
            if "frequency_step" in data:
                state.config.frequency_step = float(data.get("frequency_step", state.config.frequency_step))
            if "points" in data:
                points_val = data.get("points")
                if points_val is not None and str(points_val).strip() != "":
                    state.config.points = int(points_val)
                else:
                    state.config.points = 0
            if "mode" in data:
                state.config.mode = data.get("mode", state.config.mode)
            if "speed" in data:
                state.config.speed = data.get("speed", state.config.speed)
            if "range" in data:
                state.config.range = data.get("range", state.config.range)
            if "bias_voltage" in data:
                state.config.bias_voltage = float(data.get("bias_voltage", state.config.bias_voltage))

            # Если points > 1 и частоты разные, пересчитываем шаг
            if state.config.points > 1 and state.config.frequency_start != state.config.frequency_end:
                span = state.config.frequency_end - state.config.frequency_start
                state.config.frequency_step = span / (state.config.points - 1)

            # Применяем настройки к прибору, если подключен
            if state.connection and state.connection.is_open:
                # Устанавливаем начальную частоту (без умножения на 100!)
                freq_bytes = struct.pack(">I", int(state.config.frequency_start))
                send_command(CMD_SET_FREQ, freq_bytes)

                # Устанавливаем смещение
                bias_int16 = int(state.config.bias_voltage)
                bias_bytes = struct.pack("<h", bias_int16)
                send_command(CMD_SET_BIAS, bias_bytes)

            logger.info(
                f"Конфигурация обновлена: start={state.config.frequency_start}, "
                f"end={state.config.frequency_end}, step={state.config.frequency_step}, "
                f"points={state.config.points}"
            )

            return jsonify(
                {
                    "success": True,
                    "message": "Настройки применены",
                    "config": {
                        "frequency_start": state.config.frequency_start,
                        "frequency_end": state.config.frequency_end,
                        "frequency_step": state.config.frequency_step,
                        "points": state.config.points,
                    },
                }
            )

        except Exception as e:
            logger.error(f"Ошибка обновления конфигурации: {str(e)}")
            return jsonify({"success": False, "message": str(e)})


@app.route("/measurements", methods=["POST"])
def control_measurements():
    """Запуск или остановка измерений"""
    action = request.form.get("action")
    if action == "start":
        if not state.connection or not state.connection.is_open:
            return jsonify({"success": False, "message": "Нет подключения к прибору"})

        state.is_measuring = True
        # Сбрасываем событие остановки
        state._stop_event.clear()
        # Запускаем измерения в отдельном потоке
        thread = threading.Thread(target=perform_measurements, daemon=True)
        thread.start()
        return jsonify({"success": True, "message": "Измерение запущено"})

    elif action == "stop":
        state.is_measuring = False
        # Устанавливаем событие остановки
        state._stop_event.set()
        return jsonify({"success": True, "message": "Остановка измерения"})

    else:
        return jsonify({"success": False, "message": "Неверное действие"})


@app.route("/measurements", methods=["GET"])
def get_measurements():
    """Получить данные измерений"""
    # Если измерения не выполняются, но интервал продолжает опрашивать, возвращаем пустой результат
    if not state.is_measuring and not state.measurements:
        return jsonify({"measurements": [], "is_measuring": False, "last_id": state._last_measurement_id - 1})

    with state._measurements_lock:
        measurements_copy = state.measurements.copy()

    return jsonify(
        {
            "measurements": measurements_copy,
            "is_measuring": state.is_measuring,
            "last_id": state._last_measurement_id - 1,
        }
    )


@app.route("/export", methods=["GET"])
def export_data():
    """Экспорт данных в CSV"""
    with state._measurements_lock:
        if not state.measurements:
            return jsonify({"success": False, "message": "Нет данных для экспорта"})

        measurements_copy = state.measurements.copy()

    output = io.StringIO()
    fieldnames = [
        "timestamp",
        "frequency",
        "z_mag",
        "phase_deg",
        "R",
        "X",
        "abs_X",
        "mode",
        "value",
        "unit",
        "bias_mv",
        "speed",
        "range",
        "flags",
    ]

    try:
        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        
        # Фильтруем измерения, оставляя только те, которые содержат все необходимые поля
        valid_measurements = []
        for m in measurements_copy:
            if m is None:
                logger.warning("Пропущено None измерение")
                continue
            
            # Убеждаемся, что все необходимые поля есть
            try:
                row = {field: m.get(field, "") for field in fieldnames}
                valid_measurements.append(row)
            except Exception as e:
                logger.warning(f"Ошибка при подготовке строки для экспорта: {e}")
                continue
        
        if not valid_measurements:
            return jsonify({"success": False, "message": "Нет валидных данных для экспорта"})
        
        writer.writerows(valid_measurements)
        logger.info(f"Экспортировано {len(valid_measurements)} измерений в CSV")

        output.seek(0)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"lcr_measurements_{timestamp}.csv"
        return send_file(
            io.BytesIO(output.getvalue().encode("utf-8")),
            mimetype="text/csv",
            as_attachment=True,
            download_name=filename,
        )
    except Exception as e:
        logger.error(f"Ошибка при экспорте CSV: {e}", exc_info=True)
        return jsonify({"success": False, "message": f"Ошибка при экспорте: {str(e)}"})


if __name__ == "__main__":
    app.run(debug=True, host="0.0.0.0", port=5000)
