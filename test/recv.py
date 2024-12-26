# Test script to receive a byte
#
# Author: Pravin Raghul S
import sys
from protoc import Protoc

strbp = 26
datap = 20
proto = Protoc(strbp, datap)
proto.setup()

# Check if sufficient arguments are provided
if len(sys.argv) != 2:
    print("Usage: python recv.py <lsb | msb>")
    sys.exit(1)

if sys.argv[1] not in ["lsb", "msb"]:
    print("not valid bit order")
    sys.exit(1)

while True:
    byte = proto.read_byte(sys.argv[1])
    print("value : ", chr(byte))