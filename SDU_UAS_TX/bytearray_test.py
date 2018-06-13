import serial, struct
from time import sleep
ser = serial.Serial('/dev/ttyUSB0', 115200)


while True:
    sleep(1)
    pack1 = struct.pack('>6B', 0,0,0,0,0,0)
    ser.write(pack1)

    sleep(1)

    pack2 = struct.pack('>6B', 0, 0, 0, 0, 0, 0)
    ser.write(pack2)
