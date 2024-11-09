# To receive data from the gpiobit driver
#
# Author: Pravin Raghul S
import RPi.GPIO as GPIO


class Protoc:

    def __init__(self, strbp, datap):
        self.strbp = strbp
        self.datap = datap

    def _wait_until_last_transaction(self):
        while GPIO.input(self.strbp):
            pass

    def setup(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.strbp, GPIO.IN)
        GPIO.setup(self.datap, GPIO.IN)

    def read_byte(self):
        byte = 0
        index = 0
        new_state = 0
        old_state = 0

        self._wait_until_last_transaction()

        while True:
            new_state = GPIO.input(self.strbp)
            if new_state != old_state:
                if new_state: # only if the clock is high
                    byte |= (GPIO.input(self.datap) << index)
                    index += 1
                if index == 8:
                    break
                old_state = new_state

        return byte
