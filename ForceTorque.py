# -*- coding: utf-8 -*-

from __future__ import unicode_literals, print_function
import serial
import sys
import time
import os
import csv
from serial.tools import list_ports
from datetime import datetime

def s16(value):
    return -(value & 0b1000000000000000) | (value & 0b0111111111111111)

if __name__ == "__main__":
    state = bytes(12)
    # change mode commands
    changeStateMeasurement = b"@A"+state+b":"
    changeStatefunction = b"@C"+state+b":"
    changeStateVarious = b"@D"+state+b":"

    # commands on measurement mode
    startMeasurement = b"@E"+state+b":"
    stopMeasurement = b"@F"+state+b":"
    azActivate = b"@G"+state+b":"
    azDeactivate = b"@H"+state+b":"

    # commands on function mode
    resistFunctionData = b"@S"+state+b":"
    outputFunctionData = b"@T"+state+b":"
    outputKyokusei = b"@U"+state+b":"
    resistKyokusei = b"@V"+state+b":"
    # commands on range mode
    outputLoadRange = b"@l"+state+b":"

    # characters for matching
    patternA = "@A"
    patternC = "@C"
    patternD = "@D"
    patternE = "@E"
    patternH = "@H"

    flag = 0

    # set serial port
    sensor = serial.Serial()
    sensor.baudrate = 921600
    sensor.timeout = 0.001


    #set COM port for obtaining 6-axis sensor data
    ports = list_ports.comports()
    devices = []

    for info in ports:
        devices.append(info.device)

    #found 0 device
    if len(devices) == 0:
        print("error: device not found")
        sys.exit(0)
    #found 1 device
    elif len(devices) == 1:
        sensor.port = devices[0]
        print ("sensor.port : "+devices[0])
        sensor.open()
        print("open " + sensor.port)
    #found more than 2 devices
    else:
        for i in range(len(devices)):
            print("input " + str(i) + ":\t open " + devices[i])
        print("input number of sensor port \n >> ", end="")
        num = int(input())
        sensor.port = devices[num]

        try:
            sensor.open()
            print("open " + sensor.port)
        except:
            print("cannot open" + sensor.port)
            sys.exit(0)
        

    # create folder or open folder

    try:
        basename = datetime.now().strftime("6-axisSensor/%y%m%d_%H%M%S")
        os.mkdir(basename)
        # measured data recode "sensore.txt"
        sensorBinary = open(os.path.join(basename, 'sensor.txt'), "wb")
    except:
        print("cannot open this file")


    # change various mode
    sensor.write(changeStateVarious)
    time.sleep(0.1)
    sensorLine = sensor.readline()
    print(sensorLine)

    # obtain load range
    sensor.write(outputLoadRange)
    time.sleep(0.1)
    sensorLine = sensor.readline()
    tfr = int.from_bytes(sensorLine[2:4],'big')
    cr = int.from_bytes(sensorLine[4:6],'big')
    print(str(tfr)+ ' ' + str(cr))

    # change to measurement mode
    sensor.write(changeStateMeasurement)
    sensorLine = sensor.readline()
    time.sleep(0.1)

    # stop to measurement mode
    sensor.write(stopMeasurement)
    time.sleep(0.1)
    sensorLine = sensor.readline()

    sensor.write(azActivate)
    time.sleep(0.1)
    sensorLine = sensor.readline()
    print(type(sensorLine))
    
    print("start\n")
    sensor.write(startMeasurement)

    time.sleep(2)

    count = 0
    eTime = 0


    FileName = datetime.now().strftime('FTSensorData/%y%m%d_%H%M%S')
    os.mkdir(FileName)
    f = open(os.path.join(FileName,'sensor.txt'), 'w')
    
    writer = csv.writer(f,lineterminator='\n')
    while True:
        # waiting commands
        robotline = b''
        sensorLine = b''
        byteTime = b''

        sensor.reset_input_buffer()
        sensorLine = sensor.read(32)

        byteTime = int(eTime).to_bytes(2, 'big')
        for i in range(15):
                if(sensorLine[i] == 64 and sensorLine[i+1] == 69 and
                    sensorLine[i+14] == 58):
                    sensorLine = sensorLine[i:i+15]
                    break

        fxInt = int.from_bytes(sensorLine[2:4],'big')
        fx = s16(fxInt)
        fx = (512.0 / 32768 * fx)
        fyInt = int.from_bytes(sensorLine[4:6],'big')
        fy = s16(fyInt)
        fy = (512.0 / 32768 * fy)
        fzInt = int.from_bytes(sensorLine[6:8],'big')
        fz = s16(fzInt)
        fz = (512.0 / 32768 * fz)

        mxInt = int.from_bytes(sensorLine[8:10],'big')
        mx = s16(mxInt)
        mx = (32.0 / 32768 * mx)
        myInt = int.from_bytes(sensorLine[10:12],'big')
        my = s16(myInt)
        my = (32.0 / 32768 * my)
        mzInt = int.from_bytes(sensorLine[12:14],'big')
        mz = s16(mzInt)
        mz = (32.0 / 32768 * mz)

        print("fx",fx)
        print("fy",fy)
        print("fz",fz)
        print("mx",mx)
        print("my",my)
        print("mz",mz)
        datalist = [fx,fy,fz,mx,my,mz]
        writer.writerow(datalist)

        count = count + 1
        # measurement continue until following count
        if(count > 4000):
            break
    sensor.close()
    f.close()
    print("Fin")
