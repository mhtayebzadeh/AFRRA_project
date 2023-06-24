import serial
import threading
import matplotlib.pyplot as plt

from PyQt5 import QtGui, QtWidgets, QtCore
from PyQt5.QtCore import QThread
import sys  # We need sys so that we can pass argv to QApplication
import time
import numpy as np
from PyQt5.QtCore import Qt , QTimer
from PyQt5.QtWidgets import QWidget, QApplication

## shared memory
from multiprocessing import shared_memory
shared_mem = None 
try:
    shared_mem = shared_memory.SharedMemory(name = "avis" , create = True , size = 10)
except:
    shared_mem = shared_memory.SharedMemory(name = "avis" , create = False , size = 10)
    
    
arr = []

class SerialReaderThread(threading.Thread):
    def __init__(self, port, baudrate):
        
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = False

    def run(self):
        with serial.Serial(self.port, self.baudrate, timeout=0.02) as ser:
            test = 0
            err = 0
            self.running = True
            while self.running:
                if ser.in_waiting:
                    # data = ser.readline().decode().strip()
                    data = ser.read_until(b'\n\r').decode().strip()
                    if (data == "" or data == "0" ):
                        continue
                    
                    vec_str = data.split(',')
                    # print("----------------------------------------------")
                    # print("input string:", data)
                    # print("vec_str :", vec_str)
                    try:
                        vec_float = [float(v.strip()) for v in vec_str]
                        # print("vec float:", vec_float)
                        packet_reader(vec_float)
                        # n = int(vec_float[0])
                        # test +=1
                        # arr.append(vec_float[0])
                        # if not (n == test):
                        #     print("Wrong Package ERRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR")
                            
                        #     print("Test , N :", test , n)
                            
                        #     # test = n
                        #     # err += 1
                        #     # if err > 5:
                        #     #     self.stop()
                            
                        # else:
                        #     pass
                            
                    except:
                        print("Except ++++++++++++++++++++++++++++++++++++++++++++++")
                        pass
                    

    def stop(self):
        self.running = False
        del self
        # plot_array(arr)



class Example(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.steering = 0
        self.steering_sp = 0 
        self.timer = QTimer(self , interval = 100)
        self.timer.timeout.connect(self.timer_callback)
        self.timer.start()

    def __del__(self):
        pass

    def timer_callback(self):
        # control signal ...
        alpha = 0.85
        self.steering = alpha*self.steering + (1-alpha)*self.steering_sp 
        print("steering : " , self.steering)
        shared_mem.buf[1]  = int(saturation(self.steering , -100 , 100) + 100)
        
    def initUI(self):
        self.setGeometry(300, 300, 250, 150)
        self.setWindowTitle('Event handler')
        self.show()

    def keyPressEvent(self, e):
        if e.key() == Qt.Key_A:
            self.steering_sp = -100
        elif e.key() == Qt.Key_D:
            self.steering_sp = 100

    def keyReleaseEvent(self, e):
        if e.key() == Qt.Key_A:
            self.steering_sp = 0
        elif e.key() == Qt.Key_D:
            self.steering_sp = 0
     


def plot_array(array):
    # Create a figure and axis
    fig, ax = plt.subplots()

    # Plot the array values
    ax.plot(array)

    # Set labels and title
    ax.set_xlabel('Index')
    ax.set_ylabel('Value')
    ax.set_title('Array Plot')

    # Show the plot
    plt.show()
    
def saturation(a , min_ , max_):
    if a > max_:
        return max_
    elif a < min_:
        return min_
    return a 

def packet_reader(vec):
    
    if len(vec) != 4:
        return 
    theta_ankle = vec[0]
    sp_theta_ankle = vec[1]
    force_loadcell = vec[2]
    sp_force = vec[3] 
    
    avis_car_speed = 0
    if theta_ankle >= 0 :
        avis_car_speed = int(saturation(100*theta_ankle/0.5 , 0 , 100))
    else:
        avis_car_speed = int(saturation(100*theta_ankle/0.2 , -100 , 0))
        
    shared_mem.buf[0] = avis_car_speed+101
    
# Create an instance of the SerialReaderThread class
serial_thread = SerialReaderThread("COM9", 115200)
serial_thread.setDaemon(False)
# Start the thread
serial_thread.start()

app = QApplication(sys.argv)
ex = Example()
sys.exit(app.exec_())
    
# while True:
#     print("test")
        
# Do other tasks here...



# Stop the thread when you're done
serial_thread.stop()
sys.exit(0)