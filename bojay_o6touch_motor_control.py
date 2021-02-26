# -*- coding: utf-8 -*-
# author tony_dong@zhbojay.com
# Version: o6touch_PLC_0.0.1
# initial version

# Version: o6touch_PLC_0.0.2
# update z axis cylinder control commands

__ver__ = 'o6touch_PLC_0.0.2'

import sys
import serial
import serial.tools.list_ports
import binascii
import struct
import time
import platform
import os

# auto get interpreter
Interpreter = platform.python_version()[0]
print(Interpreter)

class BojayPLCCommandClass:

    #Lock/unlock dut
    LockDUT = "%01#WCSR01101"
    UnlockDUT = "%01#WCSR01100"
    DUTLockOrUnlockSensor = '%01#RCSR007'

    # Cylinder in/out
    CylinderIN_ON = "%01#WCSR00771"
    CylinderIN_Sensor = '%01#RCSX0011'
    CylinderIN_OFF = '%01#WCSR00770'

    CylinderOUT_ON = "%01#WCSR00791"
    CylinderOUT_Sensor = '%01#RCSX0012'
    CylinderOUT_OFF = '%01#WCSR00790'

    #Enable/Disable USB
    EnableUSB = "%01#WCSR00801"
    DisableUSB = "%01#WCSR00800"
    SensorUSBLocked = '%01#RCSR0081'

    EnableUSB1 = '%01#WCSR01121'
    EnableUSB2 = '%01#WCSR01141'
    EnableUSB3 = '%01#WCSR01161'
    EnableUSB4 = '%01#WCSR01181'

    DisableUSB1 = "%01#WCSR01120"
    DisableUSB2 = "%01#WCSR01140"
    DisableUSB3 = "%01#WCSR01160"
    DisableUSB4 = "%01#WCSR01180"

    SensorUSB1Locked = '%01#RCSX0300'
    SensorUSB2Locked = '%01#RCSX0301'
    SensorUSB3Locked = '%01#RCSX0302'
    SensorUSB4Locked = '%01#RCSX0303'

    # z axis cylinder up/down
    ZaxisCylinderUp_front = '%01#WCSR10080'
    ZaxisCylinderUp_middle = '%01#WCSR10120'
    ZaxisCylinderUp_back = '%01#WCSR10100'

    ZaxisCylinderDown_front = '%01#WCSR10081'
    ZaxisCylinderDown_middle = '%01#WCSR10121'
    ZaxisCylinderDown_back = '%01#WCSR10101'

    # ZaxisCylinderFront_Sensor = '%01#RCSX0011'
    # ZaxisCylinderMiddle_Sensor = '%01#RCSX0011'
    # ZaxisCylinderBack_Sensor = '%01#RCSX0011'


    ReadPLCVersion = "%01#RDD0030000302"
    
    # Step move
    MoveStep_xAxis = "%01#WCSR00201"
    MoveStep_yAxis = "%01#WCSR00241"
    MoveStep_zAxis = "%01#WCSR00281"
    
    #Step set
    SetStep_xAxis = "%01#WDD0100001001"
    SetStep_yAxis = "%01#WDD0100801009"
    SetStep_zAxis = "%01#WDD0101601017"

    #Step get
    GetStep_xAxis = "%01#RDD0060000601"
    GetStep_yAxis = "%01#RDD0060200603"
    GetStep_zAxis = "%01#RDD0060400605"
    
    #Set speed
    SetSpeed_xAxis = "%01#WDD0020000201"
    SetSpeed_yAxis = "%01#WDD0021000211"
    SetSpeed_zAxis = "%01#WDD0022000221"
    SetSpeed_rAxis = "%01#WDD0023000231"
    
    #Get speed
    GetSpeed_xAxis = "%01#RDD0020000201"
    GetSpeed_yAxis = "%01#RDD0021000211"
    GetSpeed_zAxis = "%01#RDD0022000221"


    #Set x&y and z move distance
    SetDistane_xAxis = "%01#WDD0020200203"
    SetDistane_yAxis = "%01#WDD0021200213"
    SetDistane_zAxis = "%01#WDD0022200223"
    
    #Move x&y&z
    XYMove = "%01#WCSR002F1"
    XYZMove = "%01#WCSR002B1"
    XMove = "%01#WCSR002C1"
    YMove = "%01#WCSR002D1"
    ZMove = "%01#WCSR002E1"
    
    #Get x&y&z
    GetCoordiante_xAxis = "%01#RDD0014600147"
    GetCoordiante_yAxis = "%01#RDD0015000151"
    GetCoordiante_zAxis = "%01#RDD0015400155"
    
    #Single of moving axis
    SingleMoveFinish_xAxis = "%01#RCSR0800"
    SingleMoveFinish_yAxis = "%01#RCSR0801"
    SingleMoveFinish_zAxis = "%01#RCSR0802"
    SingleMoveFinish_xyAxis = "%01#RCSR0803"
    SingleMoveFinish_xyzAxis = "%01#RCSR0059"
    SingleHomeFinish_xAxis = "%01#RCSR0100"
    SingleHomeFinish_yAxis = "%01#RCSR0101"
    SingleHomeFinish_zAxis = "%01#RCSR0102"
    SingleHomeFinish_xyAxis = "%01#RCSR0103"
    SingleHomeFinish_xyzAxis = "%01#RCSR0104"
    
    # Get Limit
    GetMaxLimit_xAxis = "%01#RDD0062000621"
    GetMaxLimit_yAxis = "%01#RDD0062200623"
    GetMaxLimit_zAxis = "%01#RDD0062400625"
    GetMinLimit_xAxis = "%01#RDD0063000631"
    GetMinLimit_yAxis = "%01#RDD0063200633"
    GetMinLimit_zAxis = "%01#RDD0063400635"
    
    # Set Limit
    SetMaxLimit_xAxis = "%01#WDD0210002101"
    SetMinLimit_xAxis = "%01#WDD0210402105"
    SetMaxLimit_yAxis = "%01#WDD0210802109"
    SetMinLimit_yAxis = "%01#WDD0211202113"
    SetMaxLimit_zAxis = "%01#WDD0211602117"

    SetMinLimit_zAxis = "%01#WDD0212002121"
    
    #reset
    ResetCommand_ON = "%01#WCSR00841"
    ResetCommand_OFF = "%01#WCSR01040"
    
    #Sensor axis
    XAisHomeSensor = "%01#RCSX0010"
    XAxisLLSensor = "%01#RCSX0015"
    XAxisHLSensor = "%01#RCSX0014"

    YAisHomeSensor = "%01#RCSX0011"
    YAxisLLSensor = "%01#RCSX0015"
    YAxisHLSensor = "%01#RCSX0014"

    ZAisHomeSensor = "%01#RCSX0012"
    ZAxisLLSensor = "%01#RCSX0016"
    ZAxisHLSensor = "%01#RCSX0017"

    Check_DUT_Sensor = '%01#RCSX0011'


    FixtureStartButton = "%01#RCSR0536"
    CalibrationSensor = '%01#RCSX001F'

myBojayPLCCommandClass = BojayPLCCommandClass()

#*****************************************************************************#
class GOEControlClass:
    myPLCSerialPortName = None

    myPLCSerialPort = None
    strErrorMessage = ""

    Axis_x = 1
    Axis_y = 2
    Axis_z = 3
    Axis_xy = 4
    Axis_xyz = 5
    MaxLimit = 6
    MinLimit = 7
    XAxisHomeSensor = 8
    YAxisHomeSensor = 9
    ZAxisHomeSensor = 10
    Check_DUT_Sensor = 11

    Sensor_X_Max = 14
    Sensor_X_Min = 15
    Sensor_Y_Max = 17
    Sensor_Y_Min = 18
    Sensor_Z_Max = 20
    Sensor_Z_Min = 21

    LockDUT = 27
    UnlockDUT = 28
    EnableUSB = 29
    DisableUSB = 30
    USB1 = 37
    USB2 = 38
    USB3 = 39
    USB4 = 40
    USB_ALL = 41

    DUTSensorOn = 45
    DUTSensorOff = 46

    LightCurtainON = 33
    LightCurtainOFF = 34

    Cylinder_IN = 35
    Cylinder_OUT = 36

    ZAxisCylinder_front = 42
    ZAxisCylinder_middle = 43
    ZAxisCylinder_back = 44

    ZAxis_up = 45
    ZAxis_down = 46
    Sensor_Calibrate = 47
    Down =48
    Up = 49
    TouchA = 50
    TouchB = 51
    TouchC = 52

    DUT1 = 53
    DUT2 = 54
    DUT3 = 55
    DUT4 = 56

    Red_OFF = 57
    Red_ON = 58
    Yellow_OFF = 59
    Yellow_ON = 60
    Green_OFF = 61
    Green_ON = 62
    ReadDataFail = 'fail'
    # ********************************************************************#
    #This part is for GOE


    #Open the PLC port
    '''
    Index    :  1
    Function :  open serial port
    Param    :  no use
    Return   :  0:succuess -1: fail 
    Error    :  call strErrorMessage
    '''
    def OpenSerial(self,serialName=""):
        try:
            port_list = list(serial.tools.list_ports.comports())
            if len(port_list) < 0:
                self.strErrorMessage = "There is no serial port"
                return -1
            if len(serialName) < 1:
                for i in range(0,len(port_list),1):
                    print(port_list[i].device)
                    self.myPLCSerialPort = serial.Serial(port=port_list[i].device,
                                                 timeout=0.01,
                                                 baudrate=115200,
                                                 parity=serial.PARITY_ODD)
                    self.myPLCSerialPort.setRTS(False)
                    if self.myPLCSerialPort.is_open:
                        err = self.AutoChooseCom()
                        if err != 0:
                            self.myPLCSerialPort.close()
                        else:
                            self.myPLCSerialPortName = port_list[i].device
                            return 0
                self.strErrorMessage = "Did not find suitable serial port"
                return -1
            else:
                self.myPLCSerialPort = serial.Serial(port=serialName,
                                                     timeout=0.01,
                                                     baudrate=115200,
                                                     parity=serial.PARITY_ODD)
                return 0
        except Exception as e:
            self.strErrorMessage = "OpenSerial except fail " + "{0}".format(e)
            print(self.strErrorMessage)
            return -1


  
    '''
    Index    :  2
    Function :  close serial port
    Param    :  Nose
    Return   :  0:succuess
    Error    :  call strErrorMessage
    '''
    def CloseSerial(self):
        try:
            if self.myPLCSerialPort.is_open:
                self.myPLCSerialPort.close()
            return 0
        except Exception as e:
            self.strErrorMessage = "CloseSerial except fail " + "{0}".format(e)
            return -1
   


    '''
    Index    :  3
    Function :  move to specified coordinate
    Param    :  ofWhatAxis: specified axis, can be set Axis_x/Axis_y/Axis_z
                 Value    : sepcified coordinate
    Return   :  0:succuess -1:fail
    Error    :  call strErrorMessage
    '''
    def MoveToCoordinates(self,ofWhatAxis,Value,timeout=10):
        try:
            finalByte = self.__flipByte(Value)
            #start to move
            if ofWhatAxis == self.Axis_x:
                command = myBojayPLCCommandClass.SetDistane_xAxis + finalByte
                MoveCommand = myBojayPLCCommandClass.XMove
            elif ofWhatAxis == self.Axis_y:
                command = myBojayPLCCommandClass.SetDistane_yAxis + finalByte
                MoveCommand = myBojayPLCCommandClass.YMove
            elif ofWhatAxis == self.Axis_z:
                command = myBojayPLCCommandClass.SetDistane_zAxis + finalByte
                MoveCommand = myBojayPLCCommandClass.ZMove
            ret = self.__writeRead(command)
            if ret != 0:
                self.strErrorMessage = "MoveToCoordinates write distance fail"
                # print "MoveToCoordinates write distance fail"
                return -1

            #delay
            time.sleep(0.2)
            start = time.time()
            ret = self.__writeRead(MoveCommand)
            if ret != 0:
                self.strErrorMessage = "MoveToCoordinates move fail"
                # print "MoveToCoordinates move fail"
                return -1

            mytimeCount = 0
            while (self.GetmoveSignal(ofWhatAxis) == 1):
                time.sleep(0.005)
                mytimeCount = mytimeCount + 0.005
                if mytimeCount >= timeout:
                    break
            if mytimeCount >= timeout:
                self.strErrorMessage = "MoveToCoordinates time out"
                # print "MoveToCoordinates time out"
                return -1
            print("use time is  = {}".format(time.time() - start))
            time.sleep(0.2)#prevent continual movement
            current_value = self.GetCurrentCoordinate(ofWhatAxis)
            # print(current_value)
            # print(Value)
            if (float(Value) == current_value):
                return 0
            else:
                self.strErrorMessage = "MoveToCoordinates time out != current coordinate"
                return -1
        except Exception as e:
            self.strErrorMessage = "Axis %s MoveToCoordinates value %s except fail " % (ofWhatAxis,Value) + "{0}".format(e)
            return -1

    '''
    Index    :  4
    Function :  sync move x/y axis move
    Param    :  xValue/yValue    : sepcified coordinate
                timeout          : max time to finish the movement
    Return   :  0:succuess -1:fail
    Error    :  call strErrorMessage
    '''
    def SynchronousXY(self,xValue,yValue,timeout):
        try:
            command = myBojayPLCCommandClass.SetDistane_xAxis
            finalByte = self.__flipByte(xValue)
            command = command + finalByte
            ret = self.__writeRead(command)
            if(ret != 0):
                self.strErrorMessage =  "SynchronousXY write x axis distance fail"
                return -1
            command = myBojayPLCCommandClass.SetDistane_yAxis
            finalByte = self.__flipByte(yValue)
            command = command + finalByte
            ret = self.__writeRead(command)
            if(ret != 0):
                self.strErrorMessage = "SynchronousXY write y axis distance fail"
                return -1

            command = myBojayPLCCommandClass.XYMove
            ret = self.__writeRead(command)
            if(ret != 0):
                self.strErrorMessage = "SynchronousXY driver command fail"
                return -1
            else:
                mytimeCount = 0
                while (self.GetmoveSignal(self.Axis_x) == 1):
                    if (mytimeCount > timeout):
                        self.strErrorMessage = "SynchronousXY x axis time out"
                        return -1
                    time.sleep(0.1)
                    mytimeCount += 0.1

                mytimeCount = 0
                while (self.GetmoveSignal(self.Axis_y) == 1):
                    if (mytimeCount > timeout):
                        self.strErrorMessage = "SynchronousXY y axis timeout out"
                        return -1
                    time.sleep(0.1)
                    mytimeCount += 0.1
                return 0
        except Exception as ex:
            self.strErrorMessage = "SynchronousXY fail {}".format(ex)
            return -1


    '''
    Index    :  5
    Function :  set a specified speed 
    Param    :  ofWhatAxis: specified axis, can be set Axis_x/Axis_y/Axis_z
                 Value    : sepcified speed
    Return   :  0:succuess -1:fail
    Error    :  call strErrorMessage
    '''
    def SetSpeed(self, ofWhatAxis, Value):
        try:
            if ofWhatAxis == self.Axis_x:
                if Value >= 250:
                    Value = 250
                elif Value <= 2:
                    Value = 2
                command = myBojayPLCCommandClass.SetSpeed_xAxis
            elif ofWhatAxis == self.Axis_y:
                if Value >= 250:
                    Value = 250
                elif Value <= 2:
                    Value = 2
                command = myBojayPLCCommandClass.SetSpeed_yAxis
            elif ofWhatAxis == self.Axis_z:
                if Value >= 250:
                    Value = 250
                elif Value <= 2:
                    Value = 2
                command = myBojayPLCCommandClass.SetSpeed_zAxis

            finalByte = self.__flipByte(Value)
            command = command + finalByte
            ret = self.__writeRead(command)
            if (ret != 0):
                self.strErrorMessage = "SetSpeed:set speed fail"
                return -1
            return 0
        except Exception as e:
            self.strErrorMessage = "Axis %s SetSpeed except fail " % ofWhatAxis + "{0}".format(e)
            return -1



    '''
    Index    :  6
    Function :  get a specified speed 
    Param    :  ofWhatAxis: specified axis, can be set Axis_x/Axis_y/Axis_z
    Return   :  speed value:succuess -1:fail
    Error    :  call strErrorMessage
    '''
    def GetSpeed(self, ofWhatAxis):
        try:
            if ofWhatAxis == self.Axis_x:
                command = myBojayPLCCommandClass.GetSpeed_xAxis
            elif ofWhatAxis == self.Axis_y:
                command = myBojayPLCCommandClass.GetSpeed_yAxis
            elif ofWhatAxis == self.Axis_z:
                command = myBojayPLCCommandClass.GetSpeed_zAxis
            # write
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            command = command.upper()
            if Interpreter == '3':
                command = command.encode('utf-8')
            self.myPLCSerialPort.write(command)

            # read data
            readString = self.ReadData(0.1)
            if ("fail" in readString):
                self.strErrorMessage = "Axis %s GetAxisSpeed:read data timeout" % ofWhatAxis
                return -1
            value = self.__getValueOfByte(readString)
            iSpeed = int(value)
            return iSpeed
        except Exception as e:
            self.strErrorMessage = " Axis %s GetAxisSpeed except fail" % ofWhatAxis + "{0}".format(e)
            return -1


    '''
    Index    :  7
    Function :  get the current coordinate 
    Param    :  ofWhatAxis: specified axis, can be set Axis_x/Axis_y/Axis_z
    Return   :  coordinate value:succuess -9999:fail
    Error    :  call strErrorMessage
    '''
    def GetCurrentCoordinate(self,ofWhatAxis):
        try:
            if (ofWhatAxis == self.Axis_x):
                command = myBojayPLCCommandClass.GetCoordiante_xAxis
            elif (ofWhatAxis == self.Axis_y):
                command = myBojayPLCCommandClass.GetCoordiante_yAxis
            elif (ofWhatAxis == self.Axis_z):
                command = myBojayPLCCommandClass.GetCoordiante_zAxis

            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            command = command.upper()
            if Interpreter == '3':
                command = command.encode('utf-8')
            self.myPLCSerialPort.write(command)

            # read data
            readString = self.ReadData(0.1)
            if (self.ReadDataFail in readString):
                self.strErrorMessage = "GetCurrentCoordinate timeout"
                return -9999
            else:
                value = self.__getValueOfByte(readString)
                return round(value * 10,2)
        except Exception as e:
            self.strErrorMessage = "Axis %s GetCurrentCoordinate except fail " % ofWhatAxis + "{0}".format(e)
            print(self.strErrorMessage)
            return -9999



    '''
    Index    :  8
    Function :  All axis moves back to the origin 
    Param    :  ofWhatAxis: specified axis, can be set Axis_xy/Axis_xyz
    Return   :  0:succuess -1:fail
    Error    :  call strErrorMessage
    '''
    def SignalReset(self, timeout=50):
        try:
            command = myBojayPLCCommandClass.ResetCommand_ON
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            ret = self.__writeRead(command)
            if ret != 0:
                self.strErrorMessage = "SignalReset command on fail"
                return -1

            mytimeCount = 0
            while(self.GetHomeFinishState(self.Axis_xyz) == 1):
                if (mytimeCount > timeout):
                    self.strErrorMessage = "SignalReset Reset time out"
                    return -1
                time.sleep(0.5)
                mytimeCount = mytimeCount + 0.5
            if mytimeCount > timeout:
                self.strErrorMessage = "SignalReset Reset timeout"
                return -1

            command = myBojayPLCCommandClass.ResetCommand_OFF
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            ret = self.__writeRead(command)
            if (ret == -1):
                self.strErrorMessage = "SignalReset command off fail"
                return -1

            return 0
        except Exception as e:
            self.strErrorMessage = "SignalReset except fail " + "{0}".format(e)
            return -1



    '''
    Index    :  9
    Function :  set a specified limit 
    Param    :  ofWhatAxis: specified axis, can be set Axis_x/Axis_y/Axis_z
                ofWhatLimit: max/min limit, can be set MaxLimit/MinLimit
    Return   :  0:succuess -1:fail
    Error    :  call strErrorMessage
    '''
    def SetLimit(self,ofWhatAxis,ofWhatLimit,Limit):
        try:
            if ofWhatAxis == self.Axis_x:
                if ofWhatLimit == self.MaxLimit:
                    command = myBojayPLCCommandClass.SetMaxLimit_xAxis
                elif ofWhatLimit == self.MinLimit:
                    command = myBojayPLCCommandClass.SetMinLimit_xAxis
            elif ofWhatAxis == self.Axis_y:
                if ofWhatLimit == self.MaxLimit:
                    command = myBojayPLCCommandClass.SetMaxLimit_yAxis
                elif ofWhatLimit == self.MinLimit:
                    command = myBojayPLCCommandClass.SetMinLimit_yAxis
            elif ofWhatAxis == self.Axis_z:
                if ofWhatLimit == self.MaxLimit:
                    command = myBojayPLCCommandClass.SetMaxLimit_zAxis
                elif ofWhatLimit == self.MinLimit:
                    command = myBojayPLCCommandClass.SetMinLimit_zAxis
            finalByte = self.__flipByte(Limit)
            command = command + finalByte
            ret = self.__writeRead(command)
            if ret != 0:
                self.strErrorMessage = "SetLimit:set Limit fail"
                return -1
            return 0
        except Exception as e:
            self.strErrorMessage = "Axis %s %s SetLimit value %s except fail" % (ofWhatAxis,ofWhatLimit,Limit) + "{0}".format(e)
            return -1



    '''
    Index    :  10
    Function :  lock/unlock the dut
    Param    :  state: specified action, can be set LockDUT/UnlockDUT
    Return   :  0:succuess -1:fail
    Error    :  call strErrorMessage
    '''
    def DUTLockOrUnlock(self,state):
        try:
            if (self.myPLCSerialPort.isOpen() == False):
                self.strErrorMessage = "The serial port is not opened"
                return -1

            if state == self.LockDUT:
                command = myBojayPLCCommandClass.LockDUT
                exceptValue = 1
            elif state == self.UnlockDUT:
                command = myBojayPLCCommandClass.UnlockDUT
                exceptValue = 0
            sensor = myBojayPLCCommandClass.DUTLockOrUnlockSensor
            readString = self.__writeRead(command)
            if readString != 0:
                self.strErrorMessage = "DUTLockOrUnlock fail"
                return -1
            Timeout = 3
            mytimeout = 0
            while mytimeout < Timeout:
                ret = self.__readONorOFF(sensor)
                if ret == exceptValue:
                    return 0
                elif ret == -1:
                    self.strErrorMessage = "DUTLockOrUnlock read sensor error"
                    return -1
                else:
                    time.sleep(0.1)
                    mytimeout = mytimeout + 0.1
                    continue
            if mytimeout >= Timeout:
                self.strErrorMessage = "DUTLockOrUnlock timeout"
                return -1
            return 0
        except Exception as e:
            self.strErrorMessage = "DUTLockOrUnlock except fail"
            return -1


    '''
    Index    :  11
    Function :  insert/release the usb to dut
    Param    :  state: specified action, can be set EnableUSB/DisableUSB
                whichUSB: the usb module you choose, can be set USB1/USB2/USB3/USB4
    Return   :  0:succuess -1:fail
    Error    :  call strErrorMessage
    '''
    def USBEnableOrDisable(self,state, whichUSB = 41):
        try:
            if (self.myPLCSerialPort.isOpen() == False):
                self.strErrorMessage = "The serial port is not opened"
                return -1
            if state == self.EnableUSB:
                if whichUSB == self.USB1:
                    command = myBojayPLCCommandClass.EnableUSB1
                elif whichUSB == self.USB2:
                    command = myBojayPLCCommandClass.EnableUSB2
                elif whichUSB == self.USB3:
                    command = myBojayPLCCommandClass.EnableUSB3
                elif whichUSB == self.USB4:
                    command = myBojayPLCCommandClass.EnableUSB4
                else:
                    command = myBojayPLCCommandClass.EnableUSB
                exceptValue = 1
            elif state == self.DisableUSB:
                if whichUSB == self.USB1:
                    command = myBojayPLCCommandClass.DisableUSB1
                elif whichUSB == self.USB2:
                    command = myBojayPLCCommandClass.DisableUSB2
                elif whichUSB == self.USB3:
                    command = myBojayPLCCommandClass.DisableUSB3
                elif whichUSB == self.USB4:
                    command = myBojayPLCCommandClass.DisableUSB4
                else:
                    command = myBojayPLCCommandClass.DisableUSB
                exceptValue = 0
            # sensor command
            if (whichUSB == self.USB1):
                sensor = myBojayPLCCommandClass.SensorUSB1Locked
            elif (whichUSB == self.USB2):
                sensor = myBojayPLCCommandClass.SensorUSB2Locked
            elif (whichUSB == self.USB3):
                sensor = myBojayPLCCommandClass.SensorUSB3Locked
            elif (whichUSB == self.USB4):
                sensor = myBojayPLCCommandClass.SensorUSB4Locked
            elif (whichUSB == self.USB_ALL):
                sensor = myBojayPLCCommandClass.SensorUSBLocked
            readString = self.__writeRead(command)
            if readString != 0:
                self.strErrorMessage = "USBEnableOrDisable fail"
                return -1
            Timeout = 3
            mytimeout = 0
            while mytimeout < Timeout:
                ret = self.__readONorOFF(sensor)
                if ret == exceptValue:
                    break
                elif ret == -1:
                    self.strErrorMessage = "USBEnableOrDisable read sensor error"
                    return -1
                else:
                    time.sleep(0.1)
                    mytimeout = mytimeout + 0.1
                    continue
            if mytimeout >= Timeout:
                self.strErrorMessage = "USBEnableOrDisable timeout"
                return -1
            return 0
        except Exception as ex:
            self.strErrorMessage = "USBEnableOrDisable except fail {}".format(ex)
            return -1


    '''
    Index    :  12
    Function :  send hold board into or come back
    Param    :  state: specified action, can be set Cylinder_IN/Cylinder_OUT
    Return   :  0:succuess -1:fail
    Error    :  call strErrorMessage
    '''
    def Set_CylindeFunction(self,state):
        try:
            if (self.myPLCSerialPort.isOpen() == False):
                self.strErrorMessage = "The serial port is not opened"
                return -1
            if state == self.Cylinder_IN:
                command = myBojayPLCCommandClass.CylinderIN_ON
                commandoff = myBojayPLCCommandClass.CylinderIN_OFF
                sensor = myBojayPLCCommandClass.CylinderIN_Sensor
            elif state == self.Cylinder_OUT:
                command = myBojayPLCCommandClass.CylinderOUT_ON
                commandoff = myBojayPLCCommandClass.CylinderOUT_OFF
                sensor = myBojayPLCCommandClass.CylinderOUT_Sensor
            readString = self.__writeRead(command)
            if readString != 0:
                self.strErrorMessage = "Set_CylindeFunction on fail"
                return -1
            Timeout = 10
            mytimeout = 0
            while mytimeout < Timeout:
                ret = self.__readONorOFF(sensor)
                if ret == 1:
                    break
                elif ret == -1:
                    self.strErrorMessage = "Set_CylindeFunction read sensor error"
                    return -1
                else:
                    time.sleep(0.1)
                    mytimeout = mytimeout + 0.1
                    continue
            if mytimeout >= Timeout:
                self.strErrorMessage = "Set_CylindeFunction timeout"
                return -1

            readString = self.__writeRead(commandoff)
            if readString != 0:
                self.strErrorMessage = "Set_CylindeFunction off fail"
                return -1
            return 0
        except Exception as ex:
            self.strErrorMessage = "Set_CylindeFunction except fail {}".format(ex)
            return -1


    '''
    Index    :  13
    Function :  check start button of fixture press or not
    Param    :  None
    Return   :  0:succuess -1:fail
    Error    :  call strErrorMessage
    '''
    def CheckFixtureStartButton(self):
        try:
            ret = self.__readONorOFF(myBojayPLCCommandClass.FixtureStartButton)
            if ret == 1:
                return 0
            else:
                return -1
        except Exception as e:
            self.strErrorMessage = 'CheckFixtureStartButton except fail ' + str(e)
            print (self.strErrorMessage)



    '''
    Index    :  14
    Function :  read specified sensor state 
    Param    :  ofWhatSensor: specified sensor, can be set XAxisHomeSensor/Sensor_X_Max/Sensor_X_Min/YAxisHomeSensor/Sensor_Y_Max/Sensor_Y_Min
                                                            ZAxisHomeSensor/Sensor_Z_Max/Sensor_Z_Min/
    Return   :  0/1:succuess -1:fail
    Error    :  call strErrorMessage
    '''
    def GetSensorStatus(self, ofWhatSensor):
        try:
            if (self.myPLCSerialPort.isOpen() == False):
                self.strErrorMessage = "The serial port is not opened"
                return -1
            # X-axis Sensor
            if (ofWhatSensor == self.Sensor_X_Max):
                command = myBojayPLCCommandClass.XAxisHLSensor
            elif (ofWhatSensor == self.Sensor_X_Min):
                command = myBojayPLCCommandClass.XAxisLLSensor
            elif (ofWhatSensor == self.XAxisHomeSensor):
                command = myBojayPLCCommandClass.XAisHomeSensor
            # Y-axis Sensor
            if (ofWhatSensor == self.Sensor_Y_Max):
                command = myBojayPLCCommandClass.YAxisHLSensor
            elif (ofWhatSensor == self.Sensor_Y_Min):
                command = myBojayPLCCommandClass.YAxisLLSensor
            elif (ofWhatSensor == self.YAxisHomeSensor):
                command = myBojayPLCCommandClass.YAisHomeSensor
            # Z-axis Sensor
            elif (ofWhatSensor == self.Sensor_Z_Max):
                command = myBojayPLCCommandClass.ZAxisHLSensor
            elif (ofWhatSensor == self.Sensor_Z_Min):
                command = myBojayPLCCommandClass.ZAxisLLSensor
            elif (ofWhatSensor == self.ZAxisHomeSensor):
                command = myBojayPLCCommandClass.ZAisHomeSensor
            elif (ofWhatSensor == self.Check_DUT_Sensor):
                command = myBojayPLCCommandClass.Check_DUT_Sensor
            elif (ofWhatSensor == self.Sensor_Calibrate):
                command = myBojayPLCCommandClass.CalibrationSensor
            else:
                self.strErrorMessage = "GetSensorStatus Input parameter error"
                return -1
            ret = self.__readONorOFF(command)
            ret = int(ret)
            if (ret == -1):
                self.strErrorMessage = "GetSensorStatus Read command fail"
                return -1
            return ret
        except Exception as e:
            self.strErrorMessage = "%s GetSensorStatus except error " % ofWhatSensor + "{0}".format(e)
            return -1


    '''
    Index    :  15
    Function :  cylinder air pressure or lift
    Param    :  state: specified action, can be set ZAxis_down/ZAxis_up
                whichCylinder: the cylinder you choose, can be set ZAxisCylinder_front/ZAxisCylinder_middle/ZAxisCylinder_back
    Return   :  0:succuess -1:fail
    Error    :  call strErrorMessage
    '''
    def ZAxisCylinderUpOrDown(self,state, whichCylinder):
        try:
            if (self.myPLCSerialPort.isOpen() == False):
                self.strErrorMessage = "The serial port is not opened"
                return -1
            if state == self.ZAxis_down:
                if whichCylinder == self.ZAxisCylinder_front:
                    command = myBojayPLCCommandClass.ZaxisCylinderDown_front
                elif whichCylinder == self.ZAxisCylinder_middle:
                    command = myBojayPLCCommandClass.ZaxisCylinderDown_middle
                elif whichCylinder == self.ZAxisCylinder_back:
                    command = myBojayPLCCommandClass.ZaxisCylinderDown_back
            elif state == self.ZAxis_up:
                if whichCylinder == self.ZAxisCylinder_front:
                    command = myBojayPLCCommandClass.ZaxisCylinderUp_front
                elif whichCylinder == self.ZAxisCylinder_middle:
                    command = myBojayPLCCommandClass.ZaxisCylinderUp_middle
                elif whichCylinder == self.ZAxisCylinder_back:
                    command = myBojayPLCCommandClass.ZaxisCylinderUp_back
            readString = self.__writeRead(command)
            if readString != 0:
                self.strErrorMessage = "ZAxisCylinderUpOrDown fail"
                return -1
            time.sleep(0.2)
            return 0
        except Exception as ex:
            self.strErrorMessage = "ZAxisCylinderUpOrDown except fail {}".format(ex)
            return -1


    '''
    Index    :  16
    Function :  Return error message
    '''
    def GetErrorMessage(self):
        return self.strErrorMessage



    # *****************************************************************************#

    # *****************************************************************************#

    def LightCurtainOnOrOff(self,state):
        try:
            if state == self.LightCurtainON:
                command = "%01#WCSR01300"
            elif state == self.LightCurtainOFF:
                command = "%01#WCSR01301"
            ret = self.__writeRead(command)
            if ret != 0:
                self.strErrorMessage = "LightCurtainOnOrOff error"
                return -1
            return 0
        except Exception as e:
            self.strErrorMessage = "LightCurtainOnOrOff except fail " + "{0}".format(e)
            return -1
    # ********************************************************************#

    # ********************************************************************#
    def DUTSensorStatue(self,state):
        try:
            if (self.myPLCSerialPort.isOpen() == False):
                self.strErrorMessage = "The serial port is not opened"
                return -1
            if state == self.DUTSensorOff:
                command = "%01#WCSR30021"
            elif state == self.DUTSensorOn:
                command = "%01#WCSR30020"
            readString = self.__writeRead(command)
            if readString != 0:
                self.strErrorMessage = "DUTSensorStatus fail"
                return -1
            else:
                return 0
        except:
            self.strErrorMessage = "DUTSensorStatus except error"
            return -1
    # ********************************************************************#

    # ********************************************************************#
    def GetPLCInformation(self):
        try:
            command = "%01#RDD0230002301"
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            command = command.upper()
            if Interpreter == '3':
                command = command.encode('utf-8')
            ret = self.myPLCSerialPort.write(command)
            time.sleep(0.5)
            readString = self.myPLCSerialPort.read_all()
            if len(readString) > 0:
                if Interpreter == '3':
                    readString = readString.decode()
                value = self.__getValueOfByte(readString)
                return int(value * 1000)
            return -1
        except Exception as e:
            self.strErrorMessage = "GetPLCInformation except fail"
            return -1
    # ********************************************************************#

    # ********************************************************************#
    def AutoChooseCom(self):
        try:
            bcc = self.__bccValue("%01#RDD0015400155")
            command = "%01#RDD0015400155" + bcc + '\r'
            command = command.upper()
            if Interpreter == '3':
                command = command.encode('utf-8')
            ret = self.myPLCSerialPort.write(command)
            # print ret
            time.sleep(0.5)
            readString = self.myPLCSerialPort.read_all()
            if Interpreter == '3':
                readString = readString.decode()
            if len(readString) > 0:
                print(readString)
                return 0
            return -1
        except Exception as e:
            self.strErrorMessage = "AutoChooseCom except fail " + "{0}".format(e)
            return -1

    # ********************************************************************#

    # ********************************************************************#
    def SetStepValue(self, ofWhatAxis, Value):
        try:
            finalByte = self.__flipByte(Value)
            if ofWhatAxis == self.Axis_x:
                command = myBojayPLCCommandClass.SetStep_xAxis + finalByte
            elif ofWhatAxis == self.Axis_y:
                command = myBojayPLCCommandClass.SetStep_yAxis + finalByte
            elif ofWhatAxis == self.Axis_z:
                command = myBojayPLCCommandClass.SetStep_zAxis + finalByte
            ret = self.__writeRead(command)
            if ret != 0:
                self.strErrorMessage = "SetStepValue set fail"
                return -1
            return 0
        except Exception as e:
            self.strErrorMessage = "Axis %s SetStepValue %s except fail " % (ofWhatAxis,Value) + "{0}".format(e)
            return -1

    # ********************************************************************#

    # ********************************************************************#
    def MoveStep(self, ofWhatAxis, timeout=10):
        try:
            #start to move
            if (ofWhatAxis == self.Axis_x):
                command = myBojayPLCCommandClass.MoveStep_xAxis
            elif (ofWhatAxis == self.Axis_y):
                command = myBojayPLCCommandClass.MoveStep_yAxis
            elif (ofWhatAxis == self.Axis_z):
                command = myBojayPLCCommandClass.MoveStep_zAxis

            #delay
            time.sleep(0.2)

            ret = self.__writeRead(command)
            if(ret == -1):
                self.strErrorMessage = "MoveStep fail"
                return -1
            #wait finsh
            mytimeCount = 0
            while (self.GetmoveSignal(ofWhatAxis) == 1):
                time.sleep(0.1)
                mytimeCount = mytimeCount + 0.1
                if mytimeCount >= timeout:
                    self.strErrorMessage = "MoveStep time out"
                    return -1
            time.sleep(0.2)  # prevent continual movement
            return 0
        except Exception as e:
            self.strErrorMessage = "Axis %s MoveStep except fail " % ofWhatAxis + "{0}".format(e)
            return -1

    # ********************************************************************#

    # ********************************************************************#
    def GetLimit(self, ofWhatAxis, ofWhatLimit):
        try:
            if ofWhatAxis == self.Axis_x:
                if ofWhatLimit == self.MaxLimit:
                    command = myBojayPLCCommandClass.GetMaxLimit_xAxis
                elif ofWhatLimit == self.MinLimit:
                    command = myBojayPLCCommandClass.GetMinLimit_xAxis
            elif ofWhatAxis == self.Axis_y:
                if ofWhatLimit == self.MaxLimit:
                    command = myBojayPLCCommandClass.GetMaxLimit_yAxis
                elif ofWhatLimit == self.MinLimit:
                    command = myBojayPLCCommandClass.GetMinLimit_yAxis
            elif ofWhatAxis == self.Axis_z:
                if ofWhatLimit == self.MaxLimit:
                    command = myBojayPLCCommandClass.GetMaxLimit_zAxis
                elif ofWhatLimit == self.MinLimit:
                    command = myBojayPLCCommandClass.GetMinLimit_zAxis

            #write data
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            command = command.upper()
            if Interpreter == '3':
                command = command.encode('utf-8')
            self.myPLCSerialPort.write(command)

            #read data
            readString = self.ReadData(0.1)
            if(self.ReadDataFail in readString):
                self.strErrorMessage = "read data timeout"
                return -9999
            else:
                value = self.__getValueOfByte(readString)
                return (value * 10)
        except Exception as e:
            self.strErrorMessage = "Axis %s %s GetLimit except fail " % (ofWhatAxis,ofWhatLimit) + "{0}".format(e)
            return -9999

    # ********************************************************************#

    # ********************************************************************#
    def GetHomeFinishState(self, ofWhatAxis):
        try:
            if ofWhatAxis == self.Axis_x:
                command = myBojayPLCCommandClass.SingleHomeFinish_xAxis
            elif ofWhatAxis == self.Axis_y:
                command = myBojayPLCCommandClass.SingleHomeFinish_yAxis
            elif ofWhatAxis == self.Axis_z:
                command = myBojayPLCCommandClass.SingleHomeFinish_zAxis
            elif ofWhatAxis == self.Axis_xy:
                command = myBojayPLCCommandClass.SingleHomeFinish_xyAxis
            elif ofWhatAxis == self.Axis_xyz:
                command = myBojayPLCCommandClass.SingleHomeFinish_xyzAxis

            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            command = command.upper()
            if Interpreter == '3':
                command = command.encode('utf-8')
            self.myPLCSerialPort.write(command)
            readString = self.ReadData(0.01)
            # print(readString)
            if (self.ReadDataFail in readString):
                self.strErrorMessage = "GetHomeFinishState read time out"
                return -1
            readString = int(readString[6])
            if (readString == 1):
                return 0
            elif (readString == 0):
                return 1
        except Exception as e:
            self.strErrorMessage = "Axis %s GetHomeFinishState error " % ofWhatAxis + "{0}".format(e)
            return -1
    # ********************************************************************#

    # ********************************************************************#
    def GetmoveSignal(self, ofWhatAxis):
        try:
            if ofWhatAxis == self.Axis_x:
                command = myBojayPLCCommandClass.SingleMoveFinish_xAxis
            elif ofWhatAxis == self.Axis_y:
                command = myBojayPLCCommandClass.SingleMoveFinish_yAxis
            elif ofWhatAxis == self.Axis_z:
                command = myBojayPLCCommandClass.SingleMoveFinish_zAxis
            elif ofWhatAxis == self.Axis_xy:
                command = myBojayPLCCommandClass.SingleMoveFinish_xyAxis
            elif ofWhatAxis == self.Axis_xyz:
                command = myBojayPLCCommandClass.SingleMoveFinish_xyzAxis

            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            command = command.upper()
            if Interpreter == '3':
                command = command.encode('utf-8')
            self.myPLCSerialPort.write(command)
            readString = self.ReadData(0.01)
            if (self.ReadDataFail in readString):
                self.strErrorMessage = "GetmoveSignal read time out"
                return -1
            else:
                iread = int(readString[6])
                return iread
        except Exception as e:
            self.strErrorMessage = "Axis %s GetmoveSignal except fail " % ofWhatAxis + "{0}".format(e)
            return -1
    # ********************************************************************#

    # ********************************************************************#
    def __flipByte(self, code):
        code = float(code)
        code = int(code * 5000.0 / 5.0)
        X = binascii.hexlify(struct.pack('>i', code))

        byte1 = X[0:2]
        byte2 = X[2:4]
        byte3 = X[4:6]
        byte4 = X[6:8]
        finalbyte = byte4 + byte3 + byte2 + byte1
        finalbyte = finalbyte.upper()
        if Interpreter == '3':
            return finalbyte.decode()
        return finalbyte
    # **************************************************************#

    # **************************************************************#
    def __writeRead(self, command):
        try:
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            command = command.upper()
            if Interpreter == '3':
                command = command.encode('utf-8')
            print("command = {0}".format(command))
            self.myPLCSerialPort.write(command)
            readString = self.ReadData(0.1)
            if (readString[3] == '$'):
                return 0
            else:
                return -1
        except Exception as e:
            self.strErrorMessage = "__writeRead except fail " + "{0}".format(e)
            return -1
    # **************************************************************#

    # **************************************************************#
    def __bccValue(self, code):
        code1 = ord(code[0])
        code2 = ord(code[1])
        bcc = code1 ^ code2
        for i in range(code.__len__() - 2):
            codetem = ord(code[i + 2])
            bcc = bcc ^ codetem
        bcc = binascii.hexlify(struct.pack('>i', bcc))
        bcc = bcc[6:8]
        if Interpreter == '3':
            return bcc.decode()
        return bcc
    # **************************************************************#

    # **************************************************************#
    def ReadData(self, timeDelay):
        try:
            for i in range(0, 5, 1):
                time.sleep(timeDelay)
                readString = self.myPLCSerialPort.read_all()
                print(readString.decode())
                if (len(readString) > 1):
                    if Interpreter == '3':
                        return readString.decode()
                    return readString
            return "ReadData timeout fail"
        except Exception as e:
            self.strErrorMessage = "ReadData except fail " + "{0}".format(e)
            return self.strErrorMessage
    # **************************************************************#

    # **************************************************************#
    # __getValueOfByte function
    def __getValueOfByte(self, ByteString):
        finalbyte = ByteString[6:14]
        byte1 = finalbyte[0:2]
        byte2 = finalbyte[2:4]
        byte3 = finalbyte[4:6]
        byte4 = finalbyte[6:8]
        finalbyte = byte4 + byte3 + byte2 + byte1
        # finalbyte = int(finalbyte, 16)
        # finalbyte = struct.unpack('!i', finalbyte.decode('hex'))[0]
        finalbyte = struct.unpack('!i', binascii.unhexlify(finalbyte))[0]
        finalbyte = float(finalbyte)
        Value = finalbyte * 5.0 / 5000.0
        return Value
    # **************************************************************#

    # **************************************************************#
    # __readONorOFF function
    def __readONorOFF(self, command):
        bcc = self.__bccValue(command)
        command = command + bcc + '\r'
        command = command.upper()
        if Interpreter == '3':
            command = command.encode('utf-8')
        self.myPLCSerialPort.write(command)
        readString = self.ReadData(0.1)  # self.ser.readline()
        if (self.ReadDataFail in readString):
            return -1
        readState = readString[6]
        return int(readState)
    # **************************************************************#

    # ********************************************************************************************#
    # Set DUT1/DUT2/DUT3/DUT4 LED Color
    def SetLedLightColor(self, ofWhichLED, ofWhatColor):
        try:
            if (self.myPLCSerialPort.isOpen() == False):
                self.strErrorMessage = "The serial port is not open"
                return -1
            # Set DUT1 Color
            if (ofWhichLED == self.DUT1):
                # LEFT LED RED Color
                if (ofWhatColor == self.Red_OFF):
                    command = '%01#WCSR00850'
                elif (ofWhatColor == self.Red_ON):
                    command = '%01#WCSR00851'
                # LEFT LED Yellow Color
                elif (ofWhatColor == self.Yellow_OFF):
                    command = '%01#WCSR00860'
                elif (ofWhatColor == self.Yellow_ON):
                    command = '%01#WCSR00861'
                # LEFT LED Green Color
                elif (ofWhatColor == self.Green_OFF):
                    command = '%01#WCSR00870'
                elif (ofWhatColor == self.Green_ON):
                    command = '%01#WCSR00871'
                else:
                    self.strErrorMessage = "Input parameter error"
                    return -1  # Color Parameter Error
            # Set DUT2 Color
            elif (ofWhichLED == self.DUT2):
                # LEFT LED RED Color
                if (ofWhatColor == self.Red_OFF):
                    command = '%01#WCSR008A0'
                elif (ofWhatColor == self.Red_ON):
                    command = '%01#WCSR008A1'
                # LEFT LED Yellow Color
                elif (ofWhatColor == self.Yellow_OFF):
                    command = '%01#WCSR008B0'
                elif (ofWhatColor == self.Yellow_ON):
                    command = '%01#WCSR008B1'
                # LEFT LED Green Color
                elif (ofWhatColor == self.Green_OFF):
                    command = '%01#WCSR008C0'
                elif (ofWhatColor == self.Green_ON):
                    command = '%01#WCSR008C1'
                else:
                    self.strErrorMessage = "Input parameter error"
                    return -1  # Color Parameter Error

            elif (ofWhichLED == self.DUT3):
                # LEFT LED RED Color
                if (ofWhatColor == self.Red_OFF):
                    command = '%01#WCSR01200'
                elif (ofWhatColor == self.Red_ON):
                    command = '%01#WCSR01201'
                # LEFT LED Yellow Color
                elif (ofWhatColor == self.Yellow_OFF):
                    command = '%01#WCSR01210'
                elif (ofWhatColor == self.Yellow_ON):
                    command = '%01#WCSR01211'
                # LEFT LED Green Color
                elif (ofWhatColor == self.Green_OFF):
                    command = '%01#WCSR01220'
                elif (ofWhatColor == self.Green_ON):
                    command = '%01#WCSR01221'
                else:
                    self.strErrorMessage = "Input parameter error"
                    return -1  # Color Parameter Error
                # Set DUT2 Color
            elif (ofWhichLED == self.DUT4):
                # LEFT LED RED Color
                if (ofWhatColor == self.Red_OFF):
                    command = '%01#WCSR01230'
                elif (ofWhatColor == self.Red_ON):
                    command = '%01#WCSR01231'
                # LEFT LED Yellow Color
                elif (ofWhatColor == self.Yellow_OFF):
                    command = '%01#WCSR01240'
                elif (ofWhatColor == self.Yellow_ON):
                    command = '%01#WCSR01241'
                # LEFT LED Green Color
                elif (ofWhatColor == self.Green_OFF):
                    command = '%01#WCSR01250'
                elif (ofWhatColor == self.Green_ON):
                    command = '%01#WCSR01251'
                else:
                    self.strErrorMessage = "Input parameter error"
                    return -1  # Color Parameter Error

            ret = self.__writeRead(command)
            if (ret != 0):
                self.strErrorMessage = "SetLedLightColor write command fail"
                return -1
            return 0
        except Exception as e:
            self.strErrorMessage = "SetLedLightColor except %s" % e
            return -1  # Color Parameter Error

    # ********************************************************************************************#

    # **************************************************************#
    def GetPLCVersion(self):
        try:
            command = myBojayPLCCommandClass.ReadPLCVersion
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            command = command.upper()
            if Interpreter == '3':
                command = command.encode('utf-8')
            self.myPLCSerialPort.write(command)
            # read data
            readString = self.ReadData(0.1)
            if (self.ReadDataFail in readString):
                self.strErrorMessage = "GetPLCVersion fail"
                return -1
            version = ""
            for i in range(0, len(readString[6:18]), 2):
                str = readString[6:][i:i + 2]
                version = version + chr(int(str, 16))
            return version
        except Exception as e:
            self.strErrorMessage = "ReadPLCVersionandStation except fail"
            return -1

    def TouchDowmOrUp(self, index, state):
        if (self.myPLCSerialPort.isOpen() == False):
            self.strErrorMessage = "The serial port is not opened"
            return -1
        try:
            if (index == self.TouchA):
                if (state == self.Down):
                    command = "%01#WCSR10081"
                elif (state == self.Up):
                    command = "%01#WCSR10080"
            elif (index == self.TouchB):
                if (state == self.Down):
                    command = "%01#WCSR10121"
                elif (state == self.Up):
                    command = "%01#WCSR10120"
            elif (index == self.TouchC):
                if (state == self.Down):
                    command = "%01#WCSR10101"
                elif (state == self.Up):
                    command = "%01#WCSR10100"
            ret = self.__writeRead(command)
            if (ret != 0):
                self.strErrorMessage = "CylinderDowmOrUp Red command fail"
                return -1
            time.sleep(0.2)
            return 0
        except:
            self.strErrorMessage = "CylinderDowmOrUp error"
            return -1



    def SetPLCPoint(self, point1, point2, point3):
        try:
            command = '%01#WDD00100001030000'
            finalbyte1 = self.__setPointOfByte(point1)
            finalbyte2 = self.__setPointOfByte(point2)
            finalbyte3 = self.__setPointOfByte(point3)

            command = command + finalbyte1 + finalbyte2 + finalbyte3
            command = command.upper()
            ret = self.__writeRead(command)
            if (ret != 0):
                self.strErrorMessage = "SetPLCPoint Red command fail"
                return -1
            command = '%01#WCSR00101'
            ret = self.__writeRead(command)
            if (ret != 0):
                self.strErrorMessage = "SetPLCPoint Red command fail"
                return -1

            return 0
        except Exception as e:
            self.strErrorMessage = "SetPLCPoint error"
            return -1

    def GetPLCPoint(self):
        try:
            # Set X-axis max / min limit
            command = '%01#RDD0010100103'
            bcc = self.__bccValue(command)
            command = command + bcc + '\r'
            command = command.upper()


            if Interpreter == '3':
                command = command.encode('utf-8')
            self.myPLCSerialPort.write(command)

            # read data
            readString = self.ReadData(0.1)
            if ("fail" in readString):
                self.strErrorMessage = "read data timeout"
                return -9999,-9999,-9999
            else:
                point1 = self.__getPointOfByte(readString[6:10])
                point2 = self.__getPointOfByte(readString[10:14])
                point3 = self.__getPointOfByte(readString[14:18])
                return point1, point2, point3
        except Exception as e:
            self.strErrorMessage = "GetPLCPoint error"
            return -1

    def __setPointOfByte(self, ByteString):
        X = binascii.hexlify(struct.pack('>i', int(ByteString)))
        byte3 = X[4:6]
        byte4 = X[6:8]
        finalbyte = byte4 + byte3
        finalbyte = finalbyte.decode()
        return finalbyte

    def __getPointOfByte(self, ByteString):
        str1 = ByteString[2:4]
        str2 = ByteString[0:2]
        str3 = str1 + str2
        finalbyte = int(str3, 16)

        Value = finalbyte
        return Value
    # **************************************************************#

    def CalibrationPosition(self, offset):
        try:
            ### Declare Variables #####
            Xval_1 = 0.0
            Xval_2 = 0.0
            Xval_Final = 0.0
            Yval_1 = 0.0
            Yval_2 = 0.0
            Yval_Final = 0.0
            Zvsl_Finsl = 0.0
            StepSpeed = 15

            # 3:Clynder in
            err = self.Set_CylindeFunction(self.Cylinder_IN)
            if (err == -1):
                return -1

            # 4:Set speed
            err = self.SetSpeed(self.Axis_x, StepSpeed)
            if (err == -1):
                return -1
            err = self.SetSpeed(self.Axis_y, StepSpeed)
            if (err == -1):
                return -1
            err = self.SetSpeed(self.Axis_z, StepSpeed)
            if (err == -1):
                return -1

            # read each test calibration initial position
            # exeFolderPath = os.getcwd()
            # # exeFolderPath += "\\CalibrationInitial.txt"
            # ###################################################
            # os_name=os.name
            # if(os_name=='posix'):
            #     exeFolderPath += "/CalibrationInitial.txt"
            # elif(os_name=='nt'):
            #     exeFolderPath += "\\CalibrationInitial.txt"
            ###################################################
            # exeFolderPath = os.path.join(exeFolderPath+"\\CalibrationInitial",path)
            exeFolderPath = os.getcwd()
            filepath = exeFolderPath + "/CalibrationInitial.txt"
            calibraionFile = open(filepath)
            alllines = calibraionFile.readlines()
            for line in alllines:
                strLine = line.strip()
                if ("ZIncrementX=" in strLine):
                    zAxisCalX = float(strLine[strLine.find("=") + 1:])
                elif ("ZIncrementY=" in strLine):
                    zAxisCalY = float(strLine[strLine.find("=") + 1:])

                elif ("XAxisCalIncrementX=" in strLine):
                    XAxisCalIncrementX = float(strLine[strLine.find("=") + 1:])
                elif ("XAxisCalIncrementY=" in strLine):
                    XAxisCalIncrementY = float(strLine[strLine.find("=") + 1:])

                elif ("XAxisCalDecrementX=" in strLine):
                    XAxisCalDecrementX = float(strLine[strLine.find("=") + 1:])
                elif ("XAxisCalDecrementY=" in strLine):
                    XAxisCalDecrementY = float(strLine[strLine.find("=") + 1:])

                elif ("YAxisCalIncrementX=" in strLine):
                    YAxisCalIncrementX = float(strLine[strLine.find("=") + 1:])
                elif ("YAxisCalIncrementY=" in strLine):
                    YAxisCalIncrementY = float(strLine[strLine.find("=") + 1:])

                elif ("YAxisCalDecrementX=" in strLine):
                    YAxisCalDecrementX = float(strLine[strLine.find("=") + 1:])
                elif ("YAxisCalDecrementY=" in strLine):
                    YAxisCalDecrementY = float(strLine[strLine.find("=") + 1:])

            # 3:Z-axis calibration
            zSafeDistance = 0
            err = self.MoveToCoordinates(self.Axis_z, zSafeDistance, 10)
            if (err == -1):
                return -1
            else:
                # 1:Move to calibration position
                err = self.MoveToCoordinates(self.Axis_x, zAxisCalX, 10)
                if (err == -1):
                    return -1

                err = self.MoveToCoordinates(self.Axis_y, zAxisCalY, 10)
                if (err == -1):
                    return -1

                CalibrationBlockHeight = 0
                Zvsl_Finsl = self.Calibrate(self.Axis_z, "increment")
                Zvsl_Finsl = Zvsl_Finsl + CalibrationBlockHeight
                if (Zvsl_Finsl == -1):
                    print("Z axis calibration error")
                    return -1
                else:
                    print("Z axis calibration value=" + str(Zvsl_Finsl))

            # 4:X-axis calibration  X1
            # err = self.MoveToCoordinates(self.Axis_z, zSafeDistance, 10)
            # if (err == -1):
            #     return -1
            # err = self.MoveToCoordinates(self.Axis_x, XAxisCalIncrementX, 10)
            # if (err == -1):
            #     return -1
            # err = self.MoveToCoordinates(self.Axis_y, XAxisCalIncrementY, 10)
            # if (err == -1):
            #     return -1
            Zvalue_temp = float(Zvsl_Finsl)
            Zvalue_temp = Zvalue_temp + 5
            # err = self.MoveToCoordinates(self.Axis_z, Zvalue_temp, 10)
            # if (err == -1):
            #     return -1
            # else:
            #     Xval_1 = self.Calibrate(self.Axis_x, 'increment')
            #     if (Xval_1 == -1):
            #         print("Axis_x increment fail")
            #         return -1
            #     else:
            #         print("X axis calibration value=" + str(Xval_1))
            #
            # # 4:X-axis calibration  X2
            # err = self.MoveToCoordinates(self.Axis_z, zSafeDistance, 10)
            # if (err == -1):
            #     return -1
            # err = self.MoveToCoordinates(self.Axis_x, XAxisCalDecrementX, 10)
            # if (err == -1):
            #     return -1
            # err = self.MoveToCoordinates(self.Axis_y, XAxisCalDecrementY, 10)
            # if (err == -1):
            #     return -1
            # err = self.MoveToCoordinates(self.Axis_z, Zvalue_temp, 10)
            # if (err == -1):
            #     return -1
            # else:
            #     Xval_2 = self.Calibrate(self.Axis_x, 'decrement')
            #     if (Xval_2 == -1):
            #         print("Axis_x increment fail")
            #         return -1
            #     else:
            #         print("X axis calibration value=" + str(Xval_2))
            # Xval_Final = (Xval_1 + Xval_2) / 2
            # print("X calibrate result:" + str(Xval_Final))

            # 5:Y-axis calibration Y1
            err = self.MoveToCoordinates(self.Axis_z, zSafeDistance, 10)
            if (err == -1):
                return -1
            err = self.MoveToCoordinates(self.Axis_x, YAxisCalIncrementX, 10)
            if (err == -1):
                return -1
            err = self.MoveToCoordinates(self.Axis_y, YAxisCalIncrementY, 10)
            if (err == -1):
                return -1
            err = self.MoveToCoordinates(self.Axis_z, round(Zvalue_temp,2), 10)
            if (err == -1):
                return -1
            else:
                Yval_1 = self.Calibrate(self.Axis_y, 'increment')
                if (Yval_1 == -1):
                    print("Axis_y increment fail")
                    return -1
                else:
                    print("Y axis calibration value=" + str(Yval_1))

            # 5:Y-axis calibration Y2
            err = self.MoveToCoordinates(self.Axis_z, zSafeDistance, 10)
            if (err == -1):
                return -1
            err = self.MoveToCoordinates(self.Axis_x, YAxisCalDecrementX, 10)
            if (err == -1):
                return -1
            err = self.MoveToCoordinates(self.Axis_y, YAxisCalDecrementY, 10)
            if (err == -1):
                return -1
            err = self.MoveToCoordinates(self.Axis_z, round(Zvalue_temp,2), 10)
            if (err == -1):
                return -1
            else:
                Yval_2 = self.Calibrate(self.Axis_y, 'decrement')
                if (Yval_2 == -1):
                    print("Axis_y increment fail")
                    return -1
                else:
                    print("Y axis calibration value=" + str(Yval_2))
            Yval_Final = (Yval_1 + Yval_2) / 2
            print("Y calibrate result:" + str(Yval_Final))

            # Save calibrtion file
            Zvsl_Finsl = Zvsl_Finsl + offset
            exeFolderPath = os.getcwd()
            exeFolderPath += "/Calibration.txt"
            output = open(exeFolderPath, 'w')
            # output.write("X1=" + str(Xval_1) + "\n")
            # output.write("X2=" + str(Xval_2) + "\n")
            # output.write("X-Finial=" + str(round(Xval_Final,2)) + "\n")
            output.write("Y1=" + str(Yval_1) + "\n")
            output.write("Y2=" + str(Yval_2) + "\n")
            output.write("Y-Finial=" + str(round(Yval_Final,2)) + "\n")
            output.write("Z-Finial=" + str(Zvsl_Finsl) + "\n")

            err = self.MoveToCoordinates(self.Axis_z, zSafeDistance, 10)
            if (err == -1):
                return -1
            err = self.MoveToCoordinates(self.Axis_x, Xval_Final, 10)
            if (err == -1):
                return -1
            err = self.MoveToCoordinates(self.Axis_y, Yval_Final, 10)
            if (err == -1):
                return -1
            return 0
        except Exception as e:
            self.strErrorMessage = "CalibrationPosition error"
            return -1

        # ********************************************************************************************#

    def MoveToCalPosition(self,whichTouch):
        try:

            exeFolderPath = os.getcwd()
            if whichTouch == self.TouchA:
                filepath = exeFolderPath + "/Calibration_A.txt"
            elif whichTouch == self.TouchB:
                filepath = exeFolderPath + "/Calibration_B.txt"
            elif whichTouch == self.TouchC:
                filepath = exeFolderPath + "/Calibration_C.txt"
            calibraionFile = open(filepath)
            alllines = calibraionFile.readlines()
            for line in alllines:
                strLine = line.strip()
                if ("X-Finial=" in strLine):
                    XPosition = float(strLine[strLine.find("=") + 1:])
                elif ("Y-Finial=" in strLine):
                    YPosition = float(strLine[strLine.find("=") + 1:])
                elif ("Z-Finial=" in strLine):
                    ZPosition = float(strLine[strLine.find("=") + 1:])

            err = self.Set_CylindeFunction(self.Cylinder_IN)
            if err != 0:
                self.strErrorMessage = "Set_CylindeFunction error"
                return -1

                return -1

            ret = self.MoveToCoordinates(self.Axis_z, 0)
            if ret == -1:
                self.strErrorMessage = "Move XAxis CalPosition error"
                return -1


            ret = self.MoveToCoordinates(self.Axis_x, XPosition)
            if ret == -1:
                self.strErrorMessage = "Move XAxis CalPosition error"
                return -1

            ret = self.MoveToCoordinates(self.Axis_y, YPosition)
            if ret == -1:
                self.strErrorMessage = "Move XAxis CalPosition error"
                return -1


            ret = self.MoveToCoordinates(self.Axis_z, ZPosition)
            if ret == -1:
                self.strErrorMessage = "Move XAxis CalPosition error"
                return -1

            return 0


        except Exception as e:
            self.strErrorMessage = "MoveToCalPosition error"
            return -1

    # ********************************************************************************************#

    # ********************************************************************************#
    def Calibrate(self, ofWhichAxis, incrementOrDecrement):
        try:
            shouldCalibrate = 1
            LoopCounter = 0
            Coordinates = 0.0
            isSuccess = 0


            StepValue = 0.5
            if ("decrement" in incrementOrDecrement):
                StepValue = -0.5
            err = self.SetStepValue(self.Axis_x, StepValue)
            if (err == -1):
                return -1
            err = self.SetStepValue(self.Axis_y, StepValue)
            if (err == -1):
                return -1
            err = self.SetStepValue(self.Axis_z, StepValue)
            if (err == -1):
                return -1

            while (shouldCalibrate):
                err = self.GetSensorStatus(self.Sensor_Calibrate)
                if (err == -1):
                    return -1
                elif (err == 0):
                    if ("increment" in incrementOrDecrement):
                        ret = self.MoveStep(ofWhichAxis)
                        if (ret == -1):
                            return -1
                    elif ("decrement" in incrementOrDecrement):
                        ret = self.MoveStep(ofWhichAxis)
                        if (ret == -1):
                            return -1
                elif (err == 1):
                    # back
                    if ("increment" in incrementOrDecrement):
                        err = self.SetStepValue(ofWhichAxis, -3 * StepValue)
                        if (err == -1):
                            return -1
                        ret = self.MoveStep(ofWhichAxis)
                        if (ret == -1):
                            return -1
                    elif ("decrement" in incrementOrDecrement):
                        err = self.SetStepValue(ofWhichAxis, -3 * StepValue)
                        if (err == -1):
                            return -1
                        ret = self.MoveStep(ofWhichAxis)
                        if (ret == -1):
                            return -1

                    LoopCounter = LoopCounter + 1
                    if (LoopCounter == 1):
                        StepValue = 0.1
                        if ("decrement" in incrementOrDecrement):
                            StepValue = -0.1
                    elif (LoopCounter == 2):
                        StepValue = 0.02
                        if ("decrement" in incrementOrDecrement):
                            StepValue = -0.02
                    elif (LoopCounter == 3):
                        CalPosition = self.GetCurrentCoordinate(ofWhichAxis)
                        if (CalPosition == -9999):
                            return -1
                        else:
                            # back
                            if ("increment" in incrementOrDecrement):
                                err = self.SetStepValue(ofWhichAxis, -3 * StepValue)
                                if (err == -1):
                                    return -1
                                ret = self.MoveStep(ofWhichAxis)
                                if (ret == -1):
                                    return -1
                            elif ("decrement" in incrementOrDecrement):
                                err = self.SetStepValue(ofWhichAxis, -3 * StepValue)
                                if (err == -1):
                                    return -1
                                ret = self.MoveStep(ofWhichAxis)
                                if (ret == -1):
                                    return -1
                            return round(float(CalPosition), 2)
                    err = self.SetStepValue(self.Axis_x, StepValue)
                    if (err == -1):
                        return -1
                    err = self.SetStepValue(self.Axis_y, StepValue)
                    if (err == -1):
                        return -1
                    err = self.SetStepValue(self.Axis_z, StepValue)
                    if (err == -1):
                        return -1
        except:
            self.strErrorMessage = "Calibrate error"
            return -1
    # ********************************************************************************#
    # **************************************************************#
if __name__ == '__main__':
    goectl = GOEControlClass()
    # goectl.OpenSerial()