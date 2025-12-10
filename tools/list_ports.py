import serial.tools.list_ports


print([p.device for p in serial.tools.list_ports.comports()])
