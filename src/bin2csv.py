import sys

hexbytes = []
hexlines = []

with open(sys.argv[1], "rb") as f:
    byte = f.read(1)
    while len(byte) != 0:
        hexbytes.append(ord(byte))
        byte = f.read(1)

# Need to pad out to 16384
if len(hexbytes) < 16384:
    hexbytes.extend( [0]*(16384-len(hexbytes)))

    
for i in range(0,16383,16):
    hexlines.append( ','.join([ "0x%02X" %b  for b in hexbytes[i:i+16] ]))

print( ",\n".join(hexlines))
print(",\n")
