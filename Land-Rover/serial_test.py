import serial

saber = serail.Serial()
saber.baudrate = 9200
saber.port = 'dev.tty01'
print(saber)
saber.open()
print(saber.is_open())

saber.write(b'hello')


saber.close()
print(saber.is_open())