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
    elif msg == '3':
        cw = [0xff, 0x03]
        ser.write(serial.to_bytes(cw))
    elif msg == '4':
        cw = [0xff, 0x04]
        ser.write(serial.to_bytes(cw))
    elif msg == '5':
        cw = [0xff, 0x05]
        ser.write(serial.to_bytes(cw))
    elif msg == '6':
        cw = [0xff, 0x06]
        ser.write(serial.to_bytes(cw))
    elif msg == '7':
        cw = [0xff, 0x07]
        ser.write(serial.to_bytes(cw))
    elif msg == '8':
        cw = [0xff, 0x08]
        ser.write(serial.to_bytes(cw))
    elif msg == '9':
        cw = [0xff, 0x09]
        ser.write(serial.to_bytes(cw))
    elif msg == 'a':
        cw = [0xff, 0x0a]
        ser.write(serial.to_bytes(cw))
    elif msg == 'b':
        cw = [0xff, 0x0b]
        ser.write(serial.to_bytes(cw))
    elif msg == 'c':
        cw = [0xff, 0x0c]
        ser.write(serial.to_bytes(cw))
    elif msg == 'd':
        cw = [0xff, 0x0d]
        ser.write(serial.to_bytes(cw))


        
ser.close()

