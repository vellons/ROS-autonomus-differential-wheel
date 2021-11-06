from serial_bridge_checksum_half_duplex import SerialBridgeChecksumHalfDuplex
from time import sleep


if __name__ == '__main__':
    s = SerialBridgeChecksumHalfDuplex('/dev/ttyACM0', 115200)
#    s = SerialBridgeChecksumHalfDuplex('/dev/serial0', 115200)
    print("Waiting...")
    sleep(3)
    print("Reading buffer...")
    while s.in_waiting():
        print(s.read_raw())
    print("Ready.")
    sleep(3)
    print("Ping: {}".format(s.ping()))
    while 1:
        try:
            dir = input("Direzione (i/k/,) > ")
            if dir == "i":
                s.send_list([1, 250, 1, 250])
            elif dir == "k":
                s.send_list([0, 0, 0, 0])
            elif dir == ",":
                s.send_list([2, 250, 2, 250])
            elif dir == "kkk":
                for i in range(30):
                    s.send_list([0, 0, 0, 0])
                    sleep(0.2)
            elif dir == "kik":
                for i in range(10):
                    s.send_list([1, 190, 1, 190])
                    sleep(1)
                    s.send_list([0, 0, 0, 0])
                    sleep(0.5)
        except Exception as e:
            print("Error: {}".format(e))
            sleep(0.5)
            print("Ping: {}".format(s.ping()))


#    for i in range(25):
#        s.send_list([2, 255, 1, 255])
#        sleep(2)
#        s.send_list([0, 0, 0, 0])
#        sleep(0.5)
#        s.send_list([1, 130, 2, 130])
#        sleep(2)
#        s.send_list([0, 0, 0, 0])
#        sleep(0.5)

