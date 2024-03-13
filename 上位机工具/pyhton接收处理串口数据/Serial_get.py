import serial


com = serial.Serial('COM13',115200)

while(1):
    bit = com.read(1)
    if com != r'\n':
        print(bit)
