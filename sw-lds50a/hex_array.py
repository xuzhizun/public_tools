import serial 
import time

ser=serial.Serial('/dev/ttyUSB0', 9600, timeout=5)
#cw=[0xff, 0x06, 0x00, 0x11, 0x00, 0x00, 0xd9, 0xcf]
#cw=[0x01, 0x06, 0x00, 0x11, 0x00, 0x00, 0xd9, 0xcf]
#cw1=[0xff, 0x06, 0x00, 0x11, 0x00, 0x02, 0x58, 0x0e]
#cw1=b"\x01\x06\x00\x11\x00\x00\xd9\xcf"
#cw1=b"\x01\x06\x00\x11\x00\x02\x58\x0e"
#ser.write(cw1)
#ser.write(serial.to_bytes(cw1))

#time.sleep(2)
#cw=[0xff, 0x03, 0x00, 0x15, 0x00, 0x02, 0xd5, 0xcf]
cw=b"\x01\x03\x00\x15\x00\x02\xd5\xcf"
ser.write(cw)
s=ser.read(9)
print(s)
print(s[0])
print(s[1])
print(s[2])
print(hex(s[3]))
print(hex(s[4]))
print(hex(s[5]))
print(hex(s[6]))
print(s[7])
print(s[8])

parse = str(hex(s[3])) +" " +str(hex(s[4])) +" "+str(hex(s[5])) + " "+str(hex(s[6]))
print(parse)
s_binary=''.join(bin(int(b,16))[2:].zfill(8) for b in parse.split())
print(int(s_binary))


def binaryToDecimal(binary):
    binary1 = binary
    decimal, i, n = 0, 0, 0
    while(binary != 0):
        dec = binary % 10
        decimal = decimal + dec * pow(2, i)
        binary = binary//10
        i += 1
    print(decimal)   


binaryToDecimal(int(s_binary))
