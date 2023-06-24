import sys
import math
import re
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from pynput.keyboard import Key, Listener, KeyCode
from multiprocessing import shared_memory

import socket
from AVIS_PYTHON_API import avisengine
import time

def saturation(a , min_ , max_):
    if a > max_:
        return max_
    elif a < min_:
        return min_
    return a 


class KeyMonitor(QtCore.QObject):
    avis_car_speed_sp = QtCore.pyqtSignal(int)
    avis_car_steering_sp = QtCore.pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.listener = Listener(on_release=self.on_release , on_press=self.on_pressed)

    def on_pressed(self,key):
        # print("key = " , str(key))
        if str(key) in "'a'" or str(key) in "'A'":
            self.avis_car_steering_sp.emit(-100)
        elif str(key) in "'d'" or str(key) in "'D'" : 
            self.avis_car_steering_sp.emit(100)
            
        if str(key) in "'w'" or str(key) in "'W'":
            self.avis_car_speed_sp.emit(100)
        elif str(key) in "'s'" or str(key) in "'S'" : 
            self.avis_car_speed_sp.emit(-100)
          
    def on_release(self, key):
        if str(key) in "'a'" or str(key) in "'A'":
            self.avis_car_steering_sp.emit(0)
        elif str(key) in "'d'" or str(key) in "'D'" : 
            self.avis_car_steering_sp.emit(0)
            
        if str(key) in "'w'" or str(key) in "'W'":
            self.avis_car_speed_sp.emit(0)
        elif str(key) in "'s'" or str(key) in "'S'" : 
            self.avis_car_speed_sp.emit(0)
            
            
    def stop_monitoring(self):
        self.listener.stop()

    def start_monitoring(self):
        self.listener.start()
        
class SerialMonitor(QtWidgets.QMainWindow):
    packet_recieved = QtCore.pyqtSignal(str)
    
    def __init__(self):
        super(SerialMonitor, self).__init__()
        self.port = QSerialPort()
        self.serialDataView = SerialDataView(self)
        self.serialSendView = SerialSendView(self)

        self.setCentralWidget( QtWidgets.QWidget(self) )
        layout = QtWidgets.QVBoxLayout( self.centralWidget() )
        layout.addWidget(self.serialDataView)
        layout.addWidget(self.serialSendView)
        layout.setContentsMargins(3, 3, 3, 3)
        self.setWindowTitle('Serial Monitor')

        ### Tool Bar ###
        self.toolBar = ToolBar(self)
        self.addToolBar(self.toolBar)

        ### Status Bar ###
        self.setStatusBar( QtWidgets.QStatusBar(self) )
        self.statusText = QtWidgets.QLabel(self)
        self.statusBar().addWidget( self.statusText )
        
        ### Signal Connect ###
        self.toolBar.portOpenButton.clicked.connect(self.portOpen)
        self.serialSendView.serialSendSignal.connect(self.sendFromPort)
        self.port.readyRead.connect(self.readFromPort)

    def portOpen(self, flag):
        if flag:
            self.port.setBaudRate( self.toolBar.baudRate() )
            self.port.setPortName( self.toolBar.portName() )
            self.port.setDataBits( self.toolBar.dataBit() )
            self.port.setParity( self.toolBar.parity() )
            self.port.setStopBits( self.toolBar.stopBit() )
            self.port.setFlowControl( self.toolBar.flowControl() )
            r = self.port.open(QtCore.QIODevice.ReadWrite)
            if not r:
                self.statusText.setText('Port open error')
                self.toolBar.portOpenButton.setChecked(False)
                self.toolBar.serialControlEnable(True)
            else:
                self.statusText.setText('Port opened')
                self.toolBar.serialControlEnable(False)
        else:
            self.port.close()
            self.statusText.setText('Port closed')
            self.toolBar.serialControlEnable(True)
        
    def readFromPort(self):
        # data = self.port.readAll()
        # if len(data) > 0:
        #     self.serialDataView.appendSerialText( QtCore.QTextStream(data).readAll(), QtGui.QColor(255, 0, 0) )
        
        while self.port.canReadLine():
            data = self.port.readLine()
            text = data.data().decode()
            # text = text.rstrip('\n\r')
            self.packet_recieved.emit(text.strip())
            if len(data) > 0:
                self.serialDataView.appendSerialText( QtCore.QTextStream(data).readAll(), QtGui.QColor(255, 0, 0) )
        
    def sendFromPort(self, text):
        self.port.write( text.encode() )
        self.serialDataView.appendSerialText( text, QtGui.QColor(0, 0, 255) )

class SerialDataView(QtWidgets.QWidget):
    def __init__(self, parent):
        super(SerialDataView, self).__init__(parent)
        self.serialData = QtWidgets.QTextEdit(self)
        self.serialData.setReadOnly(True)
        self.serialData.setFontFamily('Courier New')
        self.serialData.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

        self.serialDataHex = QtWidgets.QTextEdit(self)
        self.serialDataHex.setReadOnly(True)
        self.serialDataHex.setFontFamily('Courier New')
        self.serialDataHex.setFixedWidth(500)
        self.serialDataHex.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

        self.label = QtWidgets.QLabel('00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F')
        self.label.setFont( QtGui.QFont('Courier New') )
        self.label.setIndent(5)

        self.setLayout( QtWidgets.QGridLayout(self) )
        self.layout().addWidget(self.serialData,    0, 0, 2, 1)
        self.layout().addWidget(self.label,         0, 1, 1, 1)
        self.layout().addWidget(self.serialDataHex, 1, 1, 1, 1)
        self.layout().setContentsMargins(2, 2, 2, 2)
        
    def appendSerialText(self, appendText, color):
        self.serialData.moveCursor(QtGui.QTextCursor.End)
        self.serialData.setFontFamily('Courier New')
        self.serialData.setTextColor(color)
        self.serialDataHex.moveCursor(QtGui.QTextCursor.End)
        self.serialDataHex.setFontFamily('Courier New')
        self.serialDataHex.setTextColor(color)

        self.serialData.insertPlainText(appendText)
        
        lastData = self.serialDataHex.toPlainText().split('\n')[-1]
        lastLength = math.ceil( len(lastData) / 3 )
        
        appendLists = []
        splitedByTwoChar = re.split( '(..)', appendText.encode().hex() )[1::2]
        if lastLength > 0:
            t = splitedByTwoChar[ : 16-lastLength ] + ['\n']
            appendLists.append( ' '.join(t) )
            splitedByTwoChar = splitedByTwoChar[ 16-lastLength : ]

        appendLists += [ ' '.join(splitedByTwoChar[ i*16 : (i+1)*16 ] + ['\n']) for i in range( math.ceil(len(splitedByTwoChar)/16) ) ]
        if len(appendLists[-1]) < 47:
            appendLists[-1] = appendLists[-1][:-1]

        for insertText in appendLists:
            self.serialDataHex.insertPlainText(insertText)
        
        self.serialData.moveCursor(QtGui.QTextCursor.End)
        self.serialDataHex.moveCursor(QtGui.QTextCursor.End)

class SerialSendView(QtWidgets.QWidget):

    serialSendSignal = QtCore.pyqtSignal(str)

    def __init__(self, parent):
        super(SerialSendView, self).__init__(parent)

        self.sendData = QtWidgets.QTextEdit(self)
        self.sendData.setAcceptRichText(False)
        self.sendData.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)

        self.sendButton = QtWidgets.QPushButton('Send')
        self.sendButton.clicked.connect(self.sendButtonClicked)
        self.sendButton.setSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Preferred)
        
        self.setLayout( QtWidgets.QHBoxLayout(self) )
        self.layout().addWidget(self.sendData)
        self.layout().addWidget(self.sendButton)
        self.layout().setContentsMargins(3, 3, 3, 3)

    def sendButtonClicked(self):
        self.serialSendSignal.emit( self.sendData.toPlainText() )
        self.sendData.clear()

class ToolBar(QtWidgets.QToolBar):
    def __init__(self, parent):
        super(ToolBar, self).__init__(parent)
        
        self.connect_to_avisengin_btn = QtWidgets.QPushButton('Connect To AVIS')
        self.connect_to_avisengin_btn.setCheckable(True)
        self.connect_to_avisengin_btn.setMinimumHeight(32)
        
        self.record_btn = QtWidgets.QPushButton('Start Rec.')
        self.record_btn.setCheckable(True)
        self.record_btn.setMinimumHeight(32)
        
        self.portOpenButton = QtWidgets.QPushButton('Open')
        self.portOpenButton.setCheckable(True)
        self.portOpenButton.setMinimumHeight(32)

        self.portNames = QtWidgets.QComboBox(self)
        self.portNames.addItems([ port.portName() for port in QSerialPortInfo().availablePorts() ])
        self.portNames.setMinimumHeight(30)

        self.baudRates = QtWidgets.QComboBox(self)
        self.baudRates.addItems([
            '110', '300', '600', '1200', '2400', '4800', '9600', '14400', '19200', '28800', 
            '31250', '38400', '51200', '56000', '57600', '76800', '115200', '128000', '230400', '256000', '921600'
        ])
        self.baudRates.setCurrentText('115200')
        self.baudRates.setMinimumHeight(30)

        self.dataBits = QtWidgets.QComboBox(self)
        self.dataBits.addItems(['5 bit', '6 bit', '7 bit', '8 bit'])
        self.dataBits.setCurrentIndex(3)
        self.dataBits.setMinimumHeight(30)

        self._parity = QtWidgets.QComboBox(self)
        self._parity.addItems(['No Parity', 'Even Parity', 'Odd Parity', 'Space Parity', 'Mark Parity'])
        self._parity.setCurrentIndex(0)
        self._parity.setMinimumHeight(30)

        self.stopBits = QtWidgets.QComboBox(self)
        self.stopBits.addItems(['One Stop', 'One And Half Stop', 'Two Stop'])
        self.stopBits.setCurrentIndex(0)
        self.stopBits.setMinimumHeight(30)

        self._flowControl = QtWidgets.QComboBox(self)
        self._flowControl.addItems(['No Flow Control', 'Hardware Control', 'Software Control'])
        self._flowControl.setCurrentIndex(0)
        self._flowControl.setMinimumHeight(30)

        self.addWidget( self.portOpenButton )
        self.addWidget( self.portNames)
        self.addWidget( self.baudRates)
        self.addWidget( self.dataBits)
        self.addWidget( self._parity)
        self.addWidget( self.stopBits)
        self.addWidget( self._flowControl)
        self.addWidget(self.connect_to_avisengin_btn)
        self.addWidget(self.record_btn)

    def serialControlEnable(self, flag):
        self.portNames.setEnabled(flag)
        self.baudRates.setEnabled(flag)
        self.dataBits.setEnabled(flag)
        self._parity.setEnabled(flag)
        self.stopBits.setEnabled(flag)
        self._flowControl.setEnabled(flag)
        
    def baudRate(self):
        return int(self.baudRates.currentText())

    def portName(self):
        return self.portNames.currentText()

    def dataBit(self):
        return int(self.dataBits.currentIndex() + 5)

    def parity(self):
        return self._parity.currentIndex()

    def stopBit(self):
        return self.stopBits.currentIndex()

    def flowControl(self):
        return self._flowControl.currentIndex()

class AUT_AAFO(QtCore.QObject):
    def __init__(self , window , parent = None) -> None:
        super().__init__(parent)
    
        self.shared_mem = None 
        try:
            self.shared_mem = shared_memory.SharedMemory(name = "avis" , create = True , size = 10)
        except:
            self.shared_mem = shared_memory.SharedMemory(name = "avis" , create = False , size = 10)

        self.window = window
        self.avis_car_steering = 0
        self.avis_car_speed = 0
        
        self.avis_car_speed_sp = 0
        self.avis_car_steering_sp = 0 
        
        self.MAX_RECORD_TIME  = 120 # 120 sec
        self.saved_data = []
        self.start_record_time = time.time()
        
        self.is_record_started = False
        self.window.toolBar.record_btn.clicked.connect(self.record_btn_clicked)
        
        self.timer = QtCore.QTimer(self , interval = 50)
        self.timer.timeout.connect(self.timer_callback)
        self.timer.start()
    
    def record_btn_clicked(self):
        self.is_record_started = not self.is_record_started 
        if self.is_record_started:
            self.window.toolBar.record_btn.setText("Stop Rec.")
        else:
            self.window.toolBar.record_btn.setText("Start Rec.")
            
            
        if self.is_record_started:
            self.saved_data = []
            self.start_record_time = time.time() 
        else:
            name = QtWidgets.QFileDialog.getSaveFileName(self.window, 'Save File' , '.' , '*.csv')
            name = name[0]
            if name == "":
                return
            
            file = open(name,'w')
            header = ["time" , "theta_ankle" , "theta_ankle_sp" , "force_loadcell" , "force_sp"]
            
            file.write(" , ".join(header))
            file.write("\n")
            
            for r in self.saved_data:
                t = r[0]
                data = r[1]
                data_str = [str(f) for f in data]
                file.write(f"{t} , ")
                file.write(" , ".join(data_str))
                file.write("\n")
            
            file.close()
            self.saved_data = []
            
        
    def timer_callback(self):
        # control signal ...
        alpha = 0.85
        self.avis_car_steering = alpha*self.avis_car_steering + (1-alpha)*self.avis_car_steering_sp 
        self.avis_car_speed = alpha*self.avis_car_speed + (1-alpha)*self.avis_car_speed_sp  
        # print("steering : " , self.avis_car_steering)
        
        ## steering 
        self.shared_mem.buf[1]  = int(saturation(self.avis_car_steering , -100 , 100) + 100)
        ## speed
        self.shared_mem.buf[0] = int(self.avis_car_speed)+101 
        
        if self.is_record_started:
            t = time.time() - self.start_record_time
            if t > self.MAX_RECORD_TIME:
                self.record_btn_clicked()
    
    def packet_reader(self, packet_str):
        vec_str = packet_str.split(',')
        # print(vec_str)
        vec_float = []
        try:
            vec_float = [float(v.strip()) for v in vec_str]
        except:
            print("wrong packet recieved Errrrrrrrrrrrrrrr")
            return 
        
        if len(vec_float) != 4:
            return 
        theta_ankle = vec_float[0]
        sp_theta_ankle = vec_float[1]
        force_loadcell = vec_float[2]
        sp_force = vec_float[3] 
        
        speed = 0
        if theta_ankle >= 0 :
            speed = int(saturation(100*theta_ankle/0.5 , 0 , 100))
        else:
            speed = int(saturation(100*theta_ankle/0.45 , -100 , 0))
        
        if abs(speed) > 4:
            self.set_speed_sp(speed)
            
            
        ## save to buff 
        if self.is_record_started:
            t = time.time() - self.start_record_time
            a = [t , [theta_ankle , sp_theta_ankle , force_loadcell , sp_force]]
            self.saved_data.append(a)
            
            if t > self.MAX_RECORD_TIME:
                self.record_btn_clicked()
        
              
    def set_steering_sp(self , sp):
        self.avis_car_steering_sp = sp 
        
    def set_speed_sp(self , sp):
        self.avis_car_speed_sp = sp 
         

class AVIS_ENGIN_CONNCTION(QtCore.QObject):
    def __init__(self , parent = None) -> None:
        super().__init__(parent)
    
        self.shared_mem = None 
        try:
            self.shared_mem = shared_memory.SharedMemory(name = "avis" , create = True , size = 10)
        except:
            self.shared_mem = shared_memory.SharedMemory(name = "avis" , create = False , size = 10)
            
        self.SIMULATOR_IP = '127.0.0.1'
        self.SIMULATOR_PORT = 25001
        self.car =  avisengine.Car() 
        self.car.connect(self.SIMULATOR_IP, self.SIMULATOR_PORT)
        time.sleep(1)
        self.timer = QtCore.QTimer(self , interval = 50)
        self.timer.timeout.connect(self.timer_callback)
        self.timer.start()
    
    def reconnect(self):
        print("Reconnect to AVIS Engin ......")
        try:
            self.car.stop()
            time.sleep(1)
            # del self.car
        except:
            pass
        
        # self.car =  avisengine.Car() 
        try:
            self.car.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.car.connect(self.SIMULATOR_IP, self.SIMULATOR_PORT)
            time.sleep(3)
        except:
            print("NOT CONNECTED TO SERVER ..........")
        
    def timer_callback(self):
        try:
            speed = self.shared_mem.buf[0] - 100
            self.car.setSpeed(speed)     ##  -100 < speed < 100

            steering = self.shared_mem.buf[1] - 100
            # Set the Steering of the car -10 degree from center, results the car to steer to the left
            self.car.setSteering(steering)  ##  -100 < steering < 100
        except:
            pass 
        
    def __del__(self):
        self.car.stop()
        
if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = SerialMonitor()
    
    
    aafo = AUT_AAFO(window)
    avis_engin = AVIS_ENGIN_CONNCTION()
    
    monitor = KeyMonitor()
    monitor.avis_car_steering_sp.connect(aafo.set_steering_sp)
    monitor.avis_car_speed_sp.connect(aafo.set_speed_sp)
    monitor.start_monitoring()
    
    window.packet_recieved.connect(aafo.packet_reader)
    window.toolBar.connect_to_avisengin_btn.clicked.connect(avis_engin.reconnect)
    window.show()
    app.exec()