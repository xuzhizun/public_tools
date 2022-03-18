import serial
ser = serial.Serial("/dev/ttyUSB0", 115200);
while True:
  cc = str(ser.readline())
  print(cc)
