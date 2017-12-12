import sys

hexbytes = []
hexlines = []

with open(sys.argv[1], "rb") as f:
    byte = f.read(1)
    while len(byte) != 0:
        hexbytes.append(ord(byte))
        byte = f.read(1)

for i in range(0,len(hexbytes),16):
    hexlines.append( ','.join([ "0x%02X" %b  for b in hexbytes[i:i+16] ]))

print( ",\n".join(hexlines))