"""
main.py - Точка входа для PyInstaller
"""

import time
import webbrowser
import threading
import sys
import os

# Добавляем текущую директорию в путь для импорта
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import service


def open_browser():
    """Открывает браузер через 1 секунду"""
    time.sleep(1.5)
    try:
        webbrowser.open("http://localhost:5001")
        print("Браузер открыт: http://localhost:5001")
    except Exception as e:
        print(f"Не удалось открыть браузер: {e}")


if __name__ == "__main__":
    print("=" * 50)
    print("LCR Meter Service")
    print("Веб-интерфейс: http://localhost:5000")
    print("=" * 50)

    # Запускаем открытие браузера в отдельном потоке
    browser_thread = threading.Thread(target=open_browser, daemon=True)
    browser_thread.start()

    # Запускаем веб-сервер
    try:
        service.app.run(host="127.0.0.1", port=5001, debug=False, threaded=True, use_reloader=False)
    except KeyboardInterrupt:
        print("\nСервер остановлен пользователем")
    except Exception as e:
        print(f"Ошибка запуска сервера: {e}")
        sys.exit(1)
