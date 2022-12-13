import serial

ser = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout = 1)

if ser.isOpen() == False:
    ser.open()
    
while True:
    msg = input('Your message: ')
    if msg == '0':
        cw = [0xff, 0x00]
        ser.write(serial.to_bytes(cw))
    elif msg == '1':
        cw = [0xff, 0x01]
        ser.write(serial.to_bytes(cw))
    elif msg == '2':
        cw = [0xff, 0x02]
        ser.write(serial.to_bytes(cw))
        
ser.close()

