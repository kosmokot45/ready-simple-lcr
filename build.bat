@echo off
echo Установка зависимостей...
pip install -r requirements.txt

echo Сборка EXE файла...
uv run pyinstaller --onefile   --add-data "service.py;."   --add-data "templates;templates"   --hidden-import serial   --hidden-import serial.tools.list_ports  --hidden-import flask   --hidden-import flask.json   --hidden-import webbrowser   --hidden-import threading   --hidden-import csv   --hidden-import io   --hidden-import struct   --hidden-import math  --hidden-import logging   --hidden-import datetime   --hidden-import time   --hidden-import os   --hidden-import dataclasses   --hidden-import typing   main.py

echo.
echo Сборка завершена!
echo EXE файл находится в папке: dist\LCR_Meter\
pause