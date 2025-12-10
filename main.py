import time
import webbrowser
import threading

import lcr_service


def open_browser():
    """Открывает браузер через 1 секунду"""
    time.sleep(1)
    webbrowser.open("http://localhost:8005")


if __name__ == "__main__":
    # open_browser()

    threading.Thread(target=lcr_service.measurement_worker, daemon=True).start()

    threading.Thread(target=open_browser, daemon=True).start()

    lcr_service.app.run(host="127.0.0.1", port=8005, debug=False, threaded=True)
