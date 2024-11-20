# Test script to receive a byte
#
# Author: Pravin Raghul S
from protoc import Protoc

strbp = 26
datap = 20
proto = Protoc(strbp, datap)
proto.setup()

while True:
    byte = proto.read_byte()
    print("value : ", chr(byte))