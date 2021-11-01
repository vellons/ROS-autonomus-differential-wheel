import serial  # pip3 install pyserial

ser = serial.Serial(
    port='/dev/serialArduino',
    baudrate=9600,
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

if __name__ == "__main__":
    ser.flush()
    while True:
        in_string = input(">> ")
        c = int(list(in_string)[0])
        print(c)
        write(c)
