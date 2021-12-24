#JXSZ-1001-Turbidity Serial Communication Protocol

import serial 
import time

ser=serial.Serial('/dev/ttyUSB0', 9600, timeout=5)

#Acquire turbidity  
#cw=b"\x01\x03\x00\x02\x00\x01\x25\xca"

#Acquire turbidity and temperature
cw=b"\x01\x03\x00\x01\x00\x02\x95\xcb"

ser.write(cw)
#feedback for turbidity
#s=ser.read(7)

#feedback for temperature and turbidity
s=ser.read(9)
print(s)
print(s[0])
print(s[1])
print(s[2])
print(hex(s[3]))
print(hex(s[4]))
print(hex(s[5]))
print(hex(s[6]))

temp = s[3]*(16**2)+s[4]

turdidity = s[5]*(16**2)+s[6]

print("temperature is %f degree, turdidity is %f NTU" % (temp/10.0, turdidity/10.0))


#print(s[7])
#print(s[8])

#parse = str(hex(s[3])) +" " +str(hex(s[4])) +" "+str(hex(s[5])) + " "+str(hex(s[6]))
###print(parse)
#s_binary=''.join(bin(int(b,16))[2:].zfill(8) for b in parse.split())
#print(int(s_binary))


def binaryToDecimal(binary):
    binary1 = binary
    decimal, i, n = 0, 0, 0
    while(binary != 0):
        dec = binary % 10
        decimal = decimal + dec * pow(2, i)
        binary = binary//10
        i += 1
    print(decimal)   


#binaryToDecimal(int(s_binary))
