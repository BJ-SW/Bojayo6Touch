#-*-coding:utf-8-*-
try:
    from PyQt5 import QtCore,QtGui,QtWidgets
    from PyQt5.QtCore import QTimer
    from PyQt5.QtWidgets import QDialog,QMessageBox
    import os
except:
    from PySide2 import QtCore, QtGui, QtWidgets
    from PySide2.QtCore import QTimer
    from PySide2.QtWidgets import QDialog, QMessageBox
from bojay_o6touch_motor_control import *
from UI import *


myBojayClass = GOEControlClass()


class TonyFrame(QDialog):

    # **************************************************************#
    #initial function
    def __init__(self,parent=None):
        try:
            super(TonyFrame,self).__init__(parent)
            self.ui = Ui_Form()
            self.ui.setupUi(self)
            self.setFixedSize(self.width(), self.height())

            self.ui.pushButtonOpenPort.clicked.connect(lambda :self.OpenPort())
            self.ui.pushButtonClosePort.clicked.connect(lambda :self.ClosePort())
            self.ui.pushButtonSignalReSet.clicked.connect(lambda :self.ActionReset())
            self.ui.pushButtonSpeedSet.clicked.connect(lambda :self.PLCSpeed('set'))
            self.ui.pushButtonSpeedGet.clicked.connect(lambda :self.PLCSpeed('get'))

            self.ui.pushButtonLimitSet.clicked.connect(lambda :self.PLCLimit('set'))
            self.ui.pushButtonPassword.clicked.connect(lambda :self.PassWord())
            self.ui.pushButtonLimitGet.clicked.connect(lambda :self.PLCLimit('get'))

            self.ui.pushButtonMovex.clicked.connect(lambda :self.AbsoluteMove('x'))
            self.ui.pushButtonMovey.clicked.connect(lambda :self.AbsoluteMove('y'))
            self.ui.pushButtonMovez.clicked.connect(lambda :self.AbsoluteMove('z'))

            self.ui.pushButtonStepMoveX.clicked.connect(lambda :self.RelativeMove('x'))
            self.ui.pushButtonStepMoveY.clicked.connect(lambda :self.RelativeMove('y'))
            self.ui.pushButtonStepMoveZ.clicked.connect(lambda :self.RelativeMove('z'))

            self.ui.pushButtonGetx.clicked.connect(lambda :self.GetCurrent('x'))
            self.ui.pushButtonGety.clicked.connect(lambda :self.GetCurrent('y'))
            self.ui.pushButtonGetz.clicked.connect(lambda :self.GetCurrent('z'))

            self.ui.pushButtonEnableUSB.clicked.connect(lambda :self.ActionUSB('enable'))
            self.ui.pushButtonDisableUSB.clicked.connect(lambda :self.ActionUSB('disable'))

            self.ui.pushButtonLockDUT.clicked.connect(lambda :self.ActionDUT('lock'))
            self.ui.pushButtonUnlockDUT.clicked.connect(lambda :self.ActionDUT('unlock'))


            self.ui.ButtonCylinderIn.clicked.connect(lambda :self.ActionCylinder('in'))
            self.ui.ButtonCylinderOut.clicked.connect(lambda :self.ActionCylinder('out'))

            self.ui.ButtonCurtainON.clicked.connect(lambda :self.ActionCurtain('on'))
            self.ui.ButtonCurtainOFF.clicked.connect(lambda :self.ActionCurtain('off'))

            self.ui.ButtonBurninStart.clicked.connect(lambda :self.Burnin())
            self.ui.pushButtonGetPLCVer.clicked.connect(lambda :self.GetVer())
            self.ui.ButtonLEDControl.clicked.connect(lambda :self.LEDLight())

            self.ui.pushButtonLimitSet.setDisabled(True)

            self.ui.ButtonSetXCalUp.clicked.connect(lambda: self.SetPosition('up'))
            self.ui.ButtonSetXCalDown.clicked.connect(lambda: self.SetPosition('down'))
            self.ui.ButtonSetYCalLeft.clicked.connect(lambda: self.SetPosition('left'))
            self.ui.ButtonSetYCalRight.clicked.connect(lambda: self.SetPosition('right'))
            self.ui.ButtonSetZCal.clicked.connect(lambda: self.SetPosition('z'))

            self.ui.pushButtonPressureGet.clicked.connect(lambda :self.ActionGetPressure())
            self.ui.pushButtonPressureSet.clicked.connect(lambda: self.ActionSetPressure())

            self.ui.ButtonStartCal.clicked.connect(lambda: self.StartCal())
            self.ui.ButtonMoveToCalibrationPosition.clicked.connect(lambda: self.MoveToCalPosition())
            self.ui.ButtonCalZAxisHeight.clicked.connect(lambda: self.CalZAxisHeight())
            self.ui.ButtonPressureLoopTest.clicked.connect(lambda :self.PressureLooTest())
            self.ui.pushButtonCylinderDown.clicked.connect(lambda :self.ActionTouchDowmOrUp("Down"))
            self.ui.pushButtonCylinderUp.clicked.connect(lambda: self.ActionTouchDowmOrUp("Up"))
        except Exception as e:
            print ("__init__ except fail %s\n" % e)
            # self.ShowErroeMessage("__init__ except fail %s\n" % e)
            return
    # **************************************************************#

    # **************************************************************#
    def GetVer(self):
        err = myBojayClass.GetPLCVersion()
        self.ui.textEditPLCVer.setText(str(err))
    # **************************************************************#

    # **************************************************************#
    def LEDLight(self):
        try:
            if self.ui.comboBoxLED.currentText() == '1-red-on':
                err = myBojayClass.SetLedLightColor(myBojayClass.DUT1,myBojayClass.Red_ON)
            elif self.ui.comboBoxLED.currentText() == '1-red-off':
                err = myBojayClass.SetLedLightColor(myBojayClass.DUT1, myBojayClass.Red_OFF)
            elif self.ui.comboBoxLED.currentText() == '1-yellow-on':
                err = myBojayClass.SetLedLightColor(myBojayClass.DUT1, myBojayClass.Yellow_ON)
            elif self.ui.comboBoxLED.currentText() == '1-yellow-off':
                err = myBojayClass.SetLedLightColor(myBojayClass.DUT1, myBojayClass.Yellow_OFF)
            elif self.ui.comboBoxLED.currentText() == '1-green-on':
                err = myBojayClass.SetLedLightColor(myBojayClass.DUT1, myBojayClass.Green_ON)
            elif self.ui.comboBoxLED.currentText() == '1-green-off':
                err = myBojayClass.SetLedLightColor(myBojayClass.DUT1, myBojayClass.Green_OFF)

            elif self.ui.comboBoxLED.currentText() == '2-red-on':
                err = myBojayClass.SetLedLightColor(myBojayClass.DUT2,myBojayClass.Red_ON)
            elif self.ui.comboBoxLED.currentText() == '2-red-off':
                err = myBojayClass.SetLedLightColor(myBojayClass.DUT2, myBojayClass.Red_OFF)
            elif self.ui.comboBoxLED.currentText() == '2-yellow-on':
                err = myBojayClass.SetLedLightColor(myBojayClass.DUT2, myBojayClass.Yellow_ON)
            elif self.ui.comboBoxLED.currentText() == '2-yellow-off':
                err = myBojayClass.SetLedLightColor(myBojayClass.DUT2, myBojayClass.Yellow_OFF)
            elif self.ui.comboBoxLED.currentText() == '2-green-on':
                err = myBojayClass.SetLedLightColor(myBojayClass.DUT2, myBojayClass.Green_ON)
            elif self.ui.comboBoxLED.currentText() == '2-green-off':
                err = myBojayClass.SetLedLightColor(myBojayClass.DUT2, myBojayClass.Green_OFF)

            elif self.ui.comboBoxLED.currentText() == '3-red-on':
                err = myBojayClass.SetLedLightColor(myBojayClass.DUT3,myBojayClass.Red_ON)
            elif self.ui.comboBoxLED.currentText() == '3-red-off':
                err = myBojayClass.SetLedLightColor(myBojayClass.DUT3, myBojayClass.Red_OFF)
            elif self.ui.comboBoxLED.currentText() == '3-yellow-on':
                err = myBojayClass.SetLedLightColor(myBojayClass.DUT3, myBojayClass.Yellow_ON)
            elif self.ui.comboBoxLED.currentText() == '3-yellow-off':
                err = myBojayClass.SetLedLightColor(myBojayClass.DUT3, myBojayClass.Yellow_OFF)
            elif self.ui.comboBoxLED.currentText() == '3-green-on':
                err = myBojayClass.SetLedLightColor(myBojayClass.DUT3, myBojayClass.Green_ON)
            elif self.ui.comboBoxLED.currentText() == '3-green-off':
                err = myBojayClass.SetLedLightColor(myBojayClass.DUT3, myBojayClass.Green_OFF)

            elif self.ui.comboBoxLED.currentText() == '4-red-on':
                err = myBojayClass.SetLedLightColor(myBojayClass.DUT4,myBojayClass.Red_ON)
            elif self.ui.comboBoxLED.currentText() == '4-red-off':
                err = myBojayClass.SetLedLightColor(myBojayClass.DUT4, myBojayClass.Red_OFF)
            elif self.ui.comboBoxLED.currentText() == '4-yellow-on':
                err = myBojayClass.SetLedLightColor(myBojayClass.DUT4, myBojayClass.Yellow_ON)
            elif self.ui.comboBoxLED.currentText() == '4-yellow-off':
                err = myBojayClass.SetLedLightColor(myBojayClass.DUT4, myBojayClass.Yellow_OFF)
            elif self.ui.comboBoxLED.currentText() == '4-green-on':
                err = myBojayClass.SetLedLightColor(myBojayClass.DUT4, myBojayClass.Green_ON)
            elif self.ui.comboBoxLED.currentText() == '4-green-off':
                err = myBojayClass.SetLedLightColor(myBojayClass.DUT4, myBojayClass.Green_OFF)

            if err != 0:
                self.ShowErroeMessage(myBojayClass.strErrorMessage)
                return -1
            return 0
        except Exception as e:
            self.ShowErroeMessage("LEDLight except Fail %s" % e)
            return -1
    # **************************************************************#

    # **************************************************************#
    def DUTSensor(self,operation):
        try:
            if operation == 'off':
                if self.ui.radioButtonDUT1Sensor.isChecked() == True:
                    err = myBojayClass.DUTSensorOnorOFF(myBojayClass.SensorOFF,myBojayClass.DUT1)
                elif self.ui.radioButtonDUT2Sensor.isChecked() == True:
                    err = myBojayClass.DUTSensorOnorOFF(myBojayClass.SensorOFF, myBojayClass.DUT2)
            elif operation == 'on':
                if self.ui.radioButtonDUT1Sensor.isChecked() == True:
                    err = myBojayClass.DUTSensorOnorOFF(myBojayClass.SensorOn,myBojayClass.DUT1)
                elif self.ui.radioButtonDUT2Sensor.isChecked() == True:
                    err = myBojayClass.DUTSensorOnorOFF(myBojayClass.SensorOn, myBojayClass.DUT2)
            if err != 0:
                return -1
            return 0
        except Exception as e:
            self.ShowErroeMessage("DUTSensor except Fail")
            return -1


    def PassWord(self):
        try:
            if self.ui.textEditPassword.toPlainText() == 'Bojay':
                self.ui.pushButtonLimitSet.setDisabled(False)
                return 0
            else:
                self.ShowErroeMessage('Please input the correct password!')
                return -1
        except Exception as e:
            self.ShowErroeMessage("PassWord except Fail")
            return -1
    # **************************************************************#

    def SetPosition(self,direction):
        try:
            retx = myBojayClass.GetCurrentCoordinate(myBojayClass.Axis_x)
            if retx == -9999:
                self.ShowErroeMessage(myBojayClass.strErrorMessage)
                return -1


            rety = myBojayClass.GetCurrentCoordinate(myBojayClass.Axis_y)
            if rety == -9999:
                self.ShowErroeMessage(myBojayClass.strErrorMessage)
                return -1
            exeFolderPath = os.getcwd()
            filepath = exeFolderPath+"/CalibrationInitial.txt"
            output = open(filepath,'a')
            if "up" in direction:
                self.SetUpx = retx
                self.SetUpy = rety
                output.write("XAxisCalIncrementX=" + str(self.SetUpx) + "\n")
                output.write("XAxisCalIncrementY=" + str(self.SetUpy) + "\n")
            elif "left" in direction:
                self.SetLeftx = retx
                self.SetLefty = rety
                output.write("YAxisCalIncrementX=" + str(self.SetLeftx) + "\n")
                output.write("YAxisCalIncrementY=" + str(self.SetLefty) + "\n")
            elif "down" in direction:
                self.SetDownx = retx
                self.SetDowny = rety
                output.write("XAxisCalDecrementX=" + str(self.SetDownx) + "\n")
                output.write("XAxisCalDecrementY=" + str(self.SetDowny) + "\n")
            elif "right" in direction:
                self.SetRightx = retx
                self.SetRighty = rety
                output.write("YAxisCalDecrementX=" + str(self.SetRightx) + "\n")
                output.write("YAxisCalDecrementY=" + str(self.SetRighty) + "\n")
            elif "z" in direction:
                self.SetZx = retx
                self.SetZy = rety
                output.write("ZIncrementX=" + str(self.SetZx) + "\n")
                output.write("ZIncrementY=" + str(self.SetZy) + "\n")
            output.close()
            return 0
        except Exception as e:
            self.ShowErroeMessage('SetPosition except %s' % e)
            return -1
    # **************************************************************#

    def StartCal(self):
        try:
            offset = self.ui.textEditOffset.toPlainText()
            if offset != '':
                offset = float(offset)
            else:
                offset = 0
            err = myBojayClass.CalibrationPosition(offset)
            if err != 0:
                self.ShowErroeMessage(myBojayClass.strErrorMessage)
                return -1
            self.ShowErroeMessage('Calibration finish!')
        except Exception as e:
            self.ShowErroeMessage('StartCal except %s' % e)
            return -1

    # ********************************************************************************#

    def PressureLooTest(self):
        try:
            Testtime = self.ui.lineEditPressureLoopTest.text()
            if time == "":
                self.ShowErroeMessage("please input pressure loop test time")
                return -1
            err = myBojayClass.SignalReset()
            if err != 0:
                self.ShowErroeMessage(myBojayClass.strErrorMessage)
                return -1
            time.sleep(1)

            err = myBojayClass.Set_CylindeFunction(myBojayClass.Cylinder_IN)
            if err != 0:
                self.ShowErroeMessage(myBojayClass.strErrorMessage)
                return -1



            if self.ui.radioButtonCylinderFront.isChecked() == True:
                whichTouch  = myBojayClass.TouchA
                ret = myBojayClass.MoveToCalPosition(whichTouch)
                if ret != 0:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
            elif self.ui.radioButtonCylinderMiddle.isChecked() == True:
                whichTouch  = myBojayClass.TouchB
                ret = myBojayClass.MoveToCalPosition(whichTouch)
                if ret != 0:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1

            elif self.ui.radioButtonCylinderBack.isChecked() == True:
                whichTouch = myBojayClass.TouchC
                ret = myBojayClass.MoveToCalPosition(whichTouch)
                if ret != 0:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1

            for i in range(int(Testtime)):
                time.sleep(2)
                err = myBojayClass.TouchDowmOrUp(whichTouch, myBojayClass.Down)
                if err != 0:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
                time.sleep(20)
                err = myBojayClass.TouchDowmOrUp(whichTouch, myBojayClass.Up)
                if err != 0:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
            err = myBojayClass.SignalReset()
            if err != 0:
                self.ShowErroeMessage(myBojayClass.strErrorMessage)
                return -1

        except Exception as e:
            self.ShowErroeMessage('PressureLooTest except %s' % e)
            return -1


    def MoveToCalPosition(self):
        try:
            if self.ui.radioButtonCylinderFront.isChecked() == True:
                whichTouch = myBojayClass.TouchA

            elif self.ui.radioButtonCylinderMiddle.isChecked() == True:
                whichTouch = myBojayClass.TouchB


            elif self.ui.radioButtonCylinderBack.isChecked() == True:
                whichTouch = myBojayClass.TouchC


            err = myBojayClass.MoveToCalPosition(whichTouch)
            if err == -1:
                self.ShowErroeMessage(myBojayClass.strErrorMessage)
                return -1
            return 0
        except Exception as e:
            self.ShowErroeMessage('MoveToCalPosition except %s' % e)
            return -1


        # ********************************************************************************************#
    # **************************************************************#
    # show error message
    def ShowErroeMessage(self, message):
        try:
            myMessageBox = QMessageBox()
            myMessageBox.information(self, "Warning", message, myMessageBox.Ok)
            return 0
        except Exception as e:
            self.ShowErroeMessage("ShowErroeMessage except Fail")
            print("ShowErroeMessage except Fail " + "{0}".format(e))
            return -1
    # **************************************************************#

    # **************************************************************#
    def OpenPort(self):
        try:
            err = myBojayClass.OpenSerial()
            if err != 0:
                self.ShowErroeMessage(myBojayClass.strErrorMessage)
                return -1
            self.ShowErroeMessage('open port successful!')
            return 0
        except Exception as e:
            self.ShowErroeMessage('OpenPort except %s' % e)
            return -1
    # **************************************************************#

    # **************************************************************#
    def ClosePort(self):
        try:
            err = myBojayClass.CloseSerial()
            if err != 0:
                self.ShowErroeMessage(myBojayClass.strErrorMessage)
                return -1
            self.ShowErroeMessage('close port successful!')
            return 0
        except Exception as e:
            self.ShowErroeMessage('ClosePort except %s' % e)
            return -1
    # **************************************************************#

    # **************************************************************#
    def ActionReset(self):
        try:
            err = myBojayClass.SignalReset()
            if err != 0:
                self.ShowErroeMessage(myBojayClass.strErrorMessage)
                return -1
            return 0
        except Exception as e:
            self.ShowErroeMessage('ActionReset except %s' % e)
            return -1
    # **************************************************************#

    # **************************************************************#
    def PLCSpeed(self,operation):
        try:
            if operation == 'set':
                xValue = self.ui.textEditXSpeedSet.toPlainText()
                if xValue != '':
                    ret = myBojayClass.SetSpeed(myBojayClass.Axis_x,float(xValue))
                    if ret != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1
                yValue = self.ui.textEditYSpeedSet.toPlainText()
                if yValue != '':
                    ret = myBojayClass.SetSpeed(myBojayClass.Axis_y,float(yValue))
                    if ret != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1
                zValue = self.ui.textEditZSpeedSet.toPlainText()
                if zValue != '':
                    ret = myBojayClass.SetSpeed(myBojayClass.Axis_z,float(zValue))
                    if ret != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1
            elif operation == 'get':
                xValue = myBojayClass.GetSpeed(myBojayClass.Axis_x)
                if xValue != -1:
                    self.ui.textEditXSpeedSet.setText(str(xValue))
                yValue = myBojayClass.GetSpeed(myBojayClass.Axis_y)
                if yValue != -1:
                    self.ui.textEditYSpeedSet.setText(str(yValue))
                zValue = myBojayClass.GetSpeed(myBojayClass.Axis_z)
                if zValue != -1:
                    self.ui.textEditZSpeedSet.setText(str(zValue))
            return 0
        except Exception as e:
            self.ShowErroeMessage('PLCSpeed except %s' % e)
            return -1
    # **************************************************************#

    # **************************************************************#
    def PLCLimit(self,operation):
        try:
            if operation == 'set':
                # x axis
                xMin = self.ui.textEditXMinLimit.toPlainText()
                if xMin != '':
                    err = myBojayClass.SetLimit(myBojayClass.Axis_x,myBojayClass.MinLimit,float(xMin))
                    if err != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1
                xMax = self.ui.textEditXMaxLimit.toPlainText()
                if xMax != '':
                    err = myBojayClass.SetLimit(myBojayClass.Axis_x,myBojayClass.MaxLimit,float(xMax))
                    if err != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1

                # y axis
                yMin = self.ui.textEditYMinLimit.toPlainText()
                if yMin != '':
                    err = myBojayClass.SetLimit(myBojayClass.Axis_y,myBojayClass.MinLimit,float(yMin))
                    if err != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1
                yMax = self.ui.textEditYMaxLimit.toPlainText()
                if yMax != '':
                    err = myBojayClass.SetLimit(myBojayClass.Axis_y,myBojayClass.MaxLimit,float(yMax))
                    if err != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1


                # z axis
                zMin = self.ui.textEditZMinLimit.toPlainText()
                if zMin != '':
                    err = myBojayClass.SetLimit(myBojayClass.Axis_z, myBojayClass.MinLimit, float(zMin))
                    if err != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1
                zMax = self.ui.textEditZMaxLimit.toPlainText()
                if zMax != '':
                    err = myBojayClass.SetLimit(myBojayClass.Axis_z, myBojayClass.MaxLimit, float(zMax))
                    if err != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1
            elif operation == 'get':
                # x axis
                xMin = myBojayClass.GetLimit(myBojayClass.Axis_x,myBojayClass.MinLimit)
                if xMin == -9999:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
                self.ui.textEditXMinLimit.setText(str(xMin))
                xMax = myBojayClass.GetLimit(myBojayClass.Axis_x,myBojayClass.MaxLimit)
                if xMax == -9999:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
                self.ui.textEditXMaxLimit.setText(str(xMax))

                # y axis
                yMin = myBojayClass.GetLimit(myBojayClass.Axis_y,myBojayClass.MinLimit)
                if yMin == -9999:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
                self.ui.textEditYMinLimit.setText(str(yMin))
                yMax = myBojayClass.GetLimit(myBojayClass.Axis_y,myBojayClass.MaxLimit)
                if yMax == -9999:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
                self.ui.textEditYMaxLimit.setText(str(yMax))

                # z axis
                zMin = myBojayClass.GetLimit(myBojayClass.Axis_z,myBojayClass.MinLimit)
                if zMin == -9999:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
                self.ui.textEditZMinLimit.setText(str(zMin))
                zMax = myBojayClass.GetLimit(myBojayClass.Axis_z,myBojayClass.MaxLimit)
                if zMax == -9999:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
                self.ui.textEditZMaxLimit.setText(str(zMax))
            return 0
        except Exception as e:
            self.ShowErroeMessage('PLCLimit except %s' % e)
            return -1
    # **************************************************************#

    # **************************************************************#
    def AbsoluteMove(self,operation):
        try:
            if operation == 'x':
                value = self.ui.textEditxMove.toPlainText()
                if value != '':
                    err = myBojayClass.MoveToCoordinates(myBojayClass.Axis_x,float(value))
                    if err != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1
            elif operation == 'y':
                value = self.ui.textEdityMove.toPlainText()
                if value != '':
                    err = myBojayClass.MoveToCoordinates(myBojayClass.Axis_y,float(value))
                    if err != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1

            elif operation == 'z':
                value = self.ui.textEditzMove.toPlainText()
                if value != '':
                    err = myBojayClass.MoveToCoordinates(myBojayClass.Axis_z,float(value))
                    if err != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1
            return 0
        except Exception as e:
            self.ShowErroeMessage('AbsoluteMove except %s' % e)
            return -1
    # **************************************************************#

    # **************************************************************#
    def RelativeMove(self,operation):
        try:
            # x axis
            if operation == 'x':
                value = self.ui.textEditStepMoveX.toPlainText()
                if value != '':
                    err = myBojayClass.SetStepValue(myBojayClass.Axis_x,float(value))
                    if err != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1
                    err = myBojayClass.MoveStep(myBojayClass.Axis_x)
                    if err != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1

            # y axis
            elif operation == 'y':
                value = self.ui.textEditStepMoveY.toPlainText()
                if value != '':
                    err = myBojayClass.SetStepValue(myBojayClass.Axis_y,float(value))
                    if err != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1
                    err = myBojayClass.MoveStep(myBojayClass.Axis_y)
                    if err != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1

            # z axis
            elif operation == 'z':
                value = self.ui.textEditStepMoveZ.toPlainText()
                if value != '':
                    err = myBojayClass.SetStepValue(myBojayClass.Axis_z,float(value))
                    if err != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1
                    err = myBojayClass.MoveStep(myBojayClass.Axis_z)
                    if err != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1
            return 0
        except Exception as e:
            self.ShowErroeMessage('RelativeMove except %s' % e)
            return -1
    # **************************************************************#

    # **************************************************************#
    def GetCurrent(self,operation):
        try:
            # x axis
            if operation == 'x':
                err = myBojayClass.GetCurrentCoordinate(myBojayClass.Axis_x)
                if err == -9999:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
                self.ui.textEditxMove.setText(str(err))

            # y axis
            elif operation == 'y':
                err = myBojayClass.GetCurrentCoordinate(myBojayClass.Axis_y)
                if err == -9999:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
                self.ui.textEdityMove.setText(str(err))

            # z axis
            elif operation == 'z':
                err = myBojayClass.GetCurrentCoordinate(myBojayClass.Axis_z)
                if err == -9999:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
                self.ui.textEditzMove.setText(str(err))
            return 0
        except Exception as e:
            self.ShowErroeMessage('GetCurrent except %s' % e)
            return -1
    # **************************************************************#

    # **************************************************************#
    def ActionUSB(self,opertaion):
        try:
            if opertaion == 'enable':
                WhicUSB = 0
                if self.ui.radioButtonUSB1.isChecked() == True:
                    WhicUSB = myBojayClass.USB1
                elif self.ui.radioButtonUSB2.isChecked() == True:
                    WhicUSB = myBojayClass.USB2
                if self.ui.radioButtonUSB3.isChecked() == True:
                    WhicUSB = myBojayClass.USB3
                elif self.ui.radioButtonUSB4.isChecked() == True:
                    WhicUSB = myBojayClass.USB4
                elif self.ui.radioButtonUSBall.isChecked() == True:
                    WhicUSB = myBojayClass.USB_ALL
                err = myBojayClass.USBEnableOrDisable(myBojayClass.EnableUSB,WhicUSB)
                if err != 0:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
            elif opertaion == 'disable':
                WhicUSB = 0
                if self.ui.radioButtonUSB1.isChecked() == True:
                    WhicUSB = myBojayClass.USB1
                elif self.ui.radioButtonUSB2.isChecked() == True:
                    WhicUSB = myBojayClass.USB2
                if self.ui.radioButtonUSB3.isChecked() == True:
                    WhicUSB = myBojayClass.USB3
                elif self.ui.radioButtonUSB4.isChecked() == True:
                    WhicUSB = myBojayClass.USB4
                elif self.ui.radioButtonUSBall.isChecked() == True:
                    WhicUSB = myBojayClass.USB_ALL
                err = myBojayClass.USBEnableOrDisable(myBojayClass.DisableUSB,WhicUSB)
                if err != 0:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
            return 0
        except Exception as e:
            self.ShowErroeMessage('ActionUSB except %s' % e)
            return -1
    # **************************************************************#

    # **************************************************************#
    def ActionPower(self,operation):
        try:
            if operation == 'enable':
                WhichPower = 0
                if self.ui.radioButtonPower1.isChecked() == True:
                    WhichPower = myBojayClass.Power1
                elif self.ui.radioButtonPower2.isChecked() == True:
                    WhichPower = myBojayClass.Power2
                elif self.ui.radioButtonPowerall.isChecked() == True:
                    WhichPower = myBojayClass.Power_all
                err = myBojayClass.PowerEnableOrDisable(myBojayClass.PowerEnable,WhichPower)
                if err != 0:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
            elif operation == 'disable':
                WhichPower = 0
                if self.ui.radioButtonPower1.isChecked() == True:
                    WhichPower = myBojayClass.Power1
                elif self.ui.radioButtonPower2.isChecked() == True:
                    WhichPower = myBojayClass.Power2
                elif self.ui.radioButtonPowerall.isChecked() == True:
                    WhichPower = myBojayClass.Power_all
                err = myBojayClass.PowerEnableOrDisable(myBojayClass.PowerDisable,WhichPower)
                if err != 0:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
            return 0
        except Exception as e:
            self.ShowErroeMessage('ActionPower except %s' % e)
            return -1
    # **************************************************************#

    # **************************************************************#
    def ActionDUT(self,opertaion):
        try:
            if opertaion == 'lock':
                err = myBojayClass.DUTLockOrUnlock(myBojayClass.LockDUT)
                if err != 0:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
            elif opertaion == 'unlock':
                err = myBojayClass.DUTLockOrUnlock(myBojayClass.UnlockDUT)
                if err != 0:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
            return 0
        except Exception as e:
            self.ShowErroeMessage('ActionDUT except %s' % e)
            return -1
    # **************************************************************#

    def ActionTouchDowmOrUp(self,state):
        try:
            if state == "Up":
                if self.ui.radioButtonCylinderBack.isChecked() == True:
                    err = myBojayClass.TouchDowmOrUp(myBojayClass.TouchC,myBojayClass.Up)
                    if err != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1
                elif self.ui.radioButtonCylinderMiddle.isChecked() == True:
                    err = myBojayClass.TouchDowmOrUp(myBojayClass.TouchB, myBojayClass.Up)
                    if err != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1
                elif self.ui.radioButtonCylinderFront.isChecked() == True:
                    err = myBojayClass.TouchDowmOrUp(myBojayClass.TouchA, myBojayClass.Up)
                    if err != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1
                else:
                    self.ShowErroeMessage("please input touch module")
                    return -1
            elif state == "Down":
                if self.ui.radioButtonCylinderBack.isChecked() == True:
                    err = myBojayClass.TouchDowmOrUp(myBojayClass.TouchC,myBojayClass.Down)
                    if err != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1
                elif self.ui.radioButtonCylinderMiddle.isChecked() == True:
                    err = myBojayClass.TouchDowmOrUp(myBojayClass.TouchB, myBojayClass.Down)
                    if err != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1
                elif self.ui.radioButtonCylinderFront.isChecked() == True:
                    err = myBojayClass.TouchDowmOrUp(myBojayClass.TouchA, myBojayClass.Down)
                    if err != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1
                else:
                    self.ShowErroeMessage("please input touch module")
                    return -1
        except Exception as e:
            self.ShowErroeMessage('ActionDUT except %s' % e)
            return -1

    # **************************************************************#

    # **************************************************************#
    def StartCal(self):
        try:
            offset = self.ui.textEditOffset.toPlainText()
            if offset != '':
                offset = float(offset)
            else:
                offset = 0
            err = myBojayClass.CalibrationPosition(offset)
            if err != 0:
                self.ShowErroeMessage(myBojayClass.strErrorMessage)
                return -1
            self.ShowErroeMessage('Calibration finish!')
        except Exception as e:
            self.ShowErroeMessage('StartCal except %s' % e)
            return -1
    # **************************************************************#

    # **************************************************************#
    def ActionCylinder(self,operation):
        try:
            if operation == 'in':
                err = myBojayClass.Set_CylindeFunction(myBojayClass.Cylinder_IN)
                if err != 0:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
            elif operation == 'out':
                err = myBojayClass.Set_CylindeFunction(myBojayClass.Cylinder_OUT)
                if err != 0:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
            return 0
        except Exception as e:
            self.ShowErroeMessage('ActionCylinder except %s' % e)
            return -1
    # **************************************************************#
    def ActionGetPressure(self):
        try:
            point1,point2,point3 = myBojayClass.GetPLCPoint()
            if point1 == -9999 and point2 == -9999 and point3 == -9999 :
                self.ShowErroeMessage(myBojayClass.strErrorMessage)
                return -1
            self.ui.textEditPoint1.setText(str(point1))
            self.ui.textEditPoint2.setText(str(point2))
            self.ui.textEditPoint3.setText(str(point3))
        except Exception as e:
            self.ShowErroeMessage('ActionCylinder except %s' % e)
            return -1

    def ActionSetPressure(self):
        try:
            point1 = self.ui.textEditPoint1.toPlainText()
            point2 = self.ui.textEditPoint2.toPlainText()
            point3 = self.ui.textEditPoint3.toPlainText()
            err= myBojayClass.SetPLCPoint(point1,point2,point3)
            if err != 0:
                self.ShowErroeMessage(myBojayClass.strErrorMessage)
                return -1

        except Exception as e:
            self.ShowErroeMessage('ActionCylinder except %s' % e)
            return -1


    # **************************************************************#
    def ActionCurtain(self,operation):
        try:
            if operation == 'on':
                err = myBojayClass.LightCurtainOnOrOff(myBojayClass.LightCurtainON)
                if err != 0:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
            elif operation == 'off':
                err = myBojayClass.LightCurtainOnOrOff(myBojayClass.LightCurtainOFF)
                if err != 0:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
            return 0
        except Exception as e:
            self.ShowErroeMessage('ActionCurtain except %s' % e)
            return -1
    # **************************************************************#

    # **************************************************************#
    def RunPattern(self):
        try:
            # 1.lock dut
            err = myBojayClass.DUTLockOrUnlock(myBojayClass.DUTLock)
            if err != 0:
                self.ShowErroeMessage(myBojayClass.strErrorMessage)
                return -1

            err = myBojayClass.USBEableOrDisable(myBojayClass.USBEnable)
            if err != 0:
                self.ShowErroeMessage(myBojayClass.strErrorMessage)
                return -1

            err = myBojayClass.PowerEnableOrDisable(myBojayClass.PowerEnable)
            if err != 0:
                self.ShowErroeMessage(myBojayClass.strErrorMessage)
                return -1

            # 2.tray in
            err = myBojayClass.Set_CylindeFunction(myBojayClass.Cylinder_IN)
            if err != 0:
                self.ShowErroeMessage(myBojayClass.strErrorMessage)
                return -1

            # 3.run spiralmotion
            ContentList = []
            speedList = []
            motionContent = ""
            with open("spiralmotion.txt", "r") as motion:
                motionContent = motion.readlines()
                contentLength = len(motionContent)
            for index in range(0, contentLength):
                if (index < 3):
                    speed = motionContent[index].split("=")[1]
                    speedList.append(float(speed))
                else:
                    xyz = motionContent[index].split(',')
                    ContentList.append(xyz)

            # 4.set speed and move
            err = myBojayClass.SetSpeed(myBojayClass.Axis_x, speedList[0])
            err = myBojayClass.SetSpeed(myBojayClass.Axis_y, speedList[1])
            err = myBojayClass.SetSpeed(myBojayClass.Axis_z, speedList[2])

            for i in range(0,len(ContentList)):
                err = myBojayClass.SynchronousXY(float(ContentList[i][0]), float(ContentList[i][1]), 10)
                if err != 0:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
                if i == 1:
                    err = myBojayClass.MoveToCoordinates(myBojayClass.Axis_z, float(ContentList[i][2]))
                    if err != 0:
                        self.ShowErroeMessage(myBojayClass.strErrorMessage)
                        return -1

            ContentList = []
            speedList = []
            self.ShowErroeMessage('RunPattern finish!')
            return 0
        except Exception as e:
            self.ShowErroeMessage('RunPattern except %s' % e)
            return -1
    # **************************************************************#

    # **************************************************************#
    def Burnin(self):
        try:
            count = self.ui.textEditBurninTimes.toPlainText()
            if count == '':
                self.ShowErroeMessage('please input the burnin counts')
                return -1

            for i in range(int(count)):
                err = myBojayClass.SignalReset()
                if err != 0:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
                time.sleep(1)
                # 1.lock dut
                err = myBojayClass.DUTLockOrUnlock(myBojayClass.LockDUT)
                if err != 0:
                    self.ShowErroeMessage('burn in fail beause of locking dut err')
                    return -1

                # 2.enable usb
                err = myBojayClass.USBEnableOrDisable(myBojayClass.EnableUSB,myBojayClass.USB_ALL)
                if err != 0:
                    self.ShowErroeMessage('burn in fail beause of enable usb err')
                    return -1

                # 3.cylinder in
                err = myBojayClass.Set_CylindeFunction(myBojayClass.Cylinder_IN)
                if err != 0:
                    self.ShowErroeMessage('burn in fail beause of cylinder in err')
                    return -1

                err = myBojayClass.MoveToCoordinates(myBojayClass.Axis_x,155)
                if err != 0:
                    self.ShowErroeMessage('burn in fail beause of MoveToCoordinates err')
                    return -1

                err = myBojayClass.MoveToCoordinates(myBojayClass.Axis_x, -55)
                if err != 0:
                    self.ShowErroeMessage('burn in fail beause of MoveToCoordinates err')
                    return -1

                err = myBojayClass.MoveToCoordinates(myBojayClass.Axis_x, 0)
                if err != 0:
                    self.ShowErroeMessage('burn in fail beause of MoveToCoordinates err')
                    return -1

                err = myBojayClass.MoveToCoordinates(myBojayClass.Axis_y, 55)
                if err != 0:
                    self.ShowErroeMessage('burn in fail beause of MoveToCoordinates err')
                    return -1

                err = myBojayClass.MoveToCoordinates(myBojayClass.Axis_y, -55)
                if err != 0:
                    self.ShowErroeMessage('burn in fail beause of MoveToCoordinates err')
                    return -1

                err = myBojayClass.MoveToCoordinates(myBojayClass.Axis_y, 0)
                if err != 0:
                    self.ShowErroeMessage('burn in fail beause of MoveToCoordinates err')
                    return -1

                err = myBojayClass.MoveToCoordinates(myBojayClass.Axis_z, 30)
                if err != 0:
                    self.ShowErroeMessage('burn in fail beause of MoveToCoordinates err')
                    return -1

                err = myBojayClass.MoveToCoordinates(myBojayClass.Axis_z, 0)
                if err != 0:
                    self.ShowErroeMessage('burn in fail beause of MoveToCoordinates err')
                    return -1

                err = myBojayClass.MoveToCalPosition(myBojayClass.TouchA)
                if err != 0:
                    self.ShowErroeMessage('burn in fail beause of MoveToCalPosition err')
                    return -1

                err = myBojayClass.TouchDowmOrUp(myBojayClass.TouchA, myBojayClass.Down)
                if err != 0:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
                time.sleep(5)
                err = myBojayClass.TouchDowmOrUp(myBojayClass.TouchA, myBojayClass.Up)
                if err != 0:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1

                err = myBojayClass.MoveToCalPosition(myBojayClass.TouchB)
                if err != 0:
                    self.ShowErroeMessage('burn in fail beause of MoveToCalPosition err')
                    return -1

                err = myBojayClass.TouchDowmOrUp(myBojayClass.TouchB, myBojayClass.Down)
                if err != 0:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
                time.sleep(5)
                err = myBojayClass.TouchDowmOrUp(myBojayClass.TouchB, myBojayClass.Up)
                if err != 0:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1

                err = myBojayClass.MoveToCalPosition(myBojayClass.TouchC)
                if err != 0:
                    self.ShowErroeMessage('burn in fail beause of MoveToCalPosition err')
                    return -1

                err = myBojayClass.TouchDowmOrUp(myBojayClass.TouchC, myBojayClass.Down)
                if err != 0:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1
                time.sleep(5)
                err = myBojayClass.TouchDowmOrUp(myBojayClass.TouchC, myBojayClass.Up)
                if err != 0:
                    self.ShowErroeMessage(myBojayClass.strErrorMessage)
                    return -1

                err = myBojayClass.MoveToCoordinates(myBojayClass.Axis_z, 0)
                if err != 0:
                    self.ShowErroeMessage('burn in fail beause of MoveToCoordinates err')
                    return -1

                # # 6.cylinder out
                # err = myBojayClass.Set_CylindeFunction(myBojayClass.Cylinder_OUT)
                # if err != 0:
                #     self.ShowErroeMessage('burn in fail beause of cylinder out err')
                #     return -1
                #
                # # 7.disable usb
                # err = myBojayClass.USBEnableOrDisable(myBojayClass.DisableUSB,myBojayClass.USB_ALL)
                # if err != 0:
                #     self.ShowErroeMessage('burn in fail beause of disable usb err')
                #     return -1
                #
                # # 8.unlock dut
                # err = myBojayClass.DUTLockOrUnlock(myBojayClass.UnlockDUT)
                # if err != 0:
                #     self.ShowErroeMessage('burn in fail beause of unlocking dut err')
                #     return -1
                print(i+1)
            self.ShowErroeMessage('Burnin finish! Times %d' % int(count))
            return 0
        except Exception as e:
            self.ShowErroeMessage('RunPattern except %s' % e)
            return -1
    # **************************************************************#

    # **************************************************************#






app = QtWidgets.QApplication(sys.argv)
myTonyFrame = TonyFrame()
myTonyFrame.show()
myTonyFrame.exec_()