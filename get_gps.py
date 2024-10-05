import serial

ser = serial.Serial('/dev/usb_gps'
                    , 4800, timeout=5)

while True:
    line = ser.readline().decode()
    splitline = line.split(',')


    if splitline[0] == '$GNGGA':
        latitude = line[18:29]
        longitude =line[30:42]
        print(line)
        print('latitude = ',latitude)
        print('longitude =',longitude)
        print('How to read : 3824.3139  means 38 deg 24.3139 mins')
        