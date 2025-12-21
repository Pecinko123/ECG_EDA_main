import datetime
import sys
import socket
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import numpy as np
from collections import deque
from c_struct_manager import SocketDataStruct

# NOT WORKING NOW

port = 8000
print('size of struct: ' + str(SocketDataStruct.size)) 

def recvall(sock, n):
    """ Helper function to receive n bytes from the socket. """
    data = b''
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data += packet
    return data

class RealTimePlotter(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.data = deque(maxlen=3000)  # To store last 2 seconds of data (1000Hz * 2s)
        self.data_file = open(" .txt", "a")
        
        local_hostname = socket.gethostname()
        ip_addresses = socket.gethostbyname_ex(local_hostname)[2]
        filtered_ips = [ip for ip in ip_addresses if not ip.startswith("127.")]

        print('server ip address: '+ filtered_ips[0] )
        server_address = filtered_ips[0]
        # Set up socket connection (example setup, replace with your actual socket details)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((server_address, port))
        self.sock.listen(10)  

     
        print('listening for connection')  
        self.client_socket, addr = self.sock.accept()
        print('accepted connection')

        # # Timer to read from socket and update plot
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updatePlot)
        self.timer.start(100)  # Update every 1 ms

    def initUI(self):
        self.setWindowTitle('Real-Time Data Plotting')
        self.graphWidget = pg.PlotWidget()
        # self.graphWidget.setYRange(-1000, 1000)
        self.setCentralWidget(self.graphWidget)

        self.plot = self.graphWidget.plot()
        self.show()

    def readSocketData(self):
        try:
            newData = self.client_socket.recv(SocketDataStruct.size)
            if newData: 
                # print('newData received')
                received = SocketDataStruct(newData)
                # print('recived:' +str(recived.emg_data_arr))

                # dataStruct = struct.unpack(newData, STRUCT_FORMAT)

                # actual_time = datetime(1970, 1, 1) + datetime.timedelta(milliseconds=dataStruct[2])
    

                # print(exampl)
                # print(actual_time)
                emg_data = received.emg_data_arr
                compass_data = received.compass_data_arr
                print('counter: '+str(received.counter))
                # stringData = newData.decode('utf-8')
                self.data_file.write(str(emg_data) + '\n')
                # timeAndData  = stringData.split('|')
                # if(len(timeAndData) == 2):
                #     print(timeAndData[1])
                
                # values = [float(val) for val in timeAndData[0].split(',') if val]
                self.data.extend(emg_data)

        except BlockingIOError:
            print('BlockingIOError')
            pass  # No data available yet

    def updatePlot(self):
        self.readSocketData()
        if len(self.data) > 0:
            self.plot.setData(np.array(self.data))

def main():
    app = QtWidgets.QApplication(sys.argv)
    mainWindow = RealTimePlotter()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
