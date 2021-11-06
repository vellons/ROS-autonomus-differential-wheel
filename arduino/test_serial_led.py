import serial  # pip3 install pyserial
from time import sleep

ser = serial.Serial(
    port='/dev/ttyACM0',
#    port='/dev/serial0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1,
    write_timeout=1
)

def write(number):
    if number > 255:
        number = 255
    if number < 0:
        number = 0
    ser.write(bytes([number]))

def read(byte=1):
    return ser.readline()
#    return ser.read(byte)

if __name__ == "__main__":
    print("pyserial version: {}".format(serial.__version__))
    print("Waiting...")
    sleep(3)
    print("Reading buffer...")
    while ser.in_waiting:
        print(read())
    print("Ready.")
    while True:
        in_string = input(">> ")
        c = ord(list(in_string)[0])
        print(c)
        write(c)
        print(read())
