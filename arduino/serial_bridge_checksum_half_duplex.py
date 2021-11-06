# Classe SerialBridgeV3.0 - Serial bridge con checksum
# Ad ogni carattere che viene inviato ne corrisponde uno identico di risposta.
# La comunicazione e' bloccante. Se non riceve risposta si blocca tutto.
# L'inizio della comunicazione comincia con 254, 253, 252.
# Il primo valore inviato X e' il numero di byte del payload della risposta.
# Poi seguono X valori.
# Esempio: list[0]=pin, list[1]=valore, list[X-2]=pin, list[X-1]=valore
# La comunicazione termina inviando 251.
# Il carattere 250 e' di ping

import serial


class SerialBridgeHalfDuplexException(Exception):
    pass


class SerialBridgeChecksumHalfDuplex:
    ser = None

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1,
            write_timeout=1
        )

    @staticmethod
    def _to_char_number(number):
        number = int(number)
        if number > 255:
            number = 255
        if number < 0:
            number = 0
        return number

    def _write_raw(self, number):
        self.ser.write(bytes([number]))

    def read_raw(self, length=1):
        return self.ser.read(length)

    def read_ord(self, length=1):
        return ord(self.ser.read(length))

    def write_and_ack(self, number):
        number = self._to_char_number(number)
        self._write_raw(number)
        x = self.read_ord(1)
        if x != number:
            # print("[SerialBridgeHalfDuplex]: ACK failed. Read: {}. Expected: {}".format(x, number))
            raise SerialBridgeHalfDuplexException("write_and_ack failed")

    def send_list(self, payload_list):
        self.write_and_ack(254)
        self.write_and_ack(253)
        self.write_and_ack(252)

        checksum = len(payload_list)
        self.write_and_ack(checksum)
        for n in payload_list:
            self.write_and_ack(n)
            checksum = (checksum + n) % 256

        # Check checksum
        self._write_raw(checksum)
        return_checksum = self.read_ord(1)
        print("Checksum: {} returned: {} Is valid: {}"
            .format(checksum, return_checksum, checksum == return_checksum))
        self.write_and_ack(251)
        return checksum == return_checksum

    def ping(self):
        try:
            self.write_and_ack(250)
            return True
        except:
            return False

    def in_waiting(self):
        return self.ser.in_waiting

    def flush(self):
        self.ser.flushInput()
        self.ser.flushOutput()

