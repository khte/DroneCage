import serial, struct
from time import sleep
ser = serial.Serial('/dev/ttyUSB0', 115200)


# while True:
    # sleep(2)
    # pack1 = struct.pack('>6B',50,20,10,10,10,10)
    # ser.write(pack1)


pack2 = struct.pack('>6B',1,1 ,1,1,1,1)
ser.write(pack2)

    # sleep(2)