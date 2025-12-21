from random import randint
import re
import socket
import threading
import numpy as np
import datetime
from c_struct_manager import SocketDataStructure
import csv
import os

path = './data'
if not os.path.exists(path):
    os.mkdir(path)
# FULLBATTERYLIFE = "2:20"
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

port = 8888
# bind the socket to a specific address and port
connectionsCount = 0
running = True
local_hostname = socket.gethostname()
ip_addresses = socket.gethostbyname_ex(local_hostname)[2]
filtered_ips = [ip for ip in ip_addresses if not ip.startswith("127.")]
print('machine addresses: '+ str(filtered_ips))
# machine can have multiple ip addresses, we will use the first one, 
# CHANGE THIS IF NEEDED TO SELECT DIFFERENT IP ADDRESS
server_address = filtered_ips[0]
print('server ip addr: '+ server_address)
server_socket.bind((server_address, port))
server_socket.listen(5)




def recvall(sock, n):
    """ Helper function to receive n bytes from the socket. """
    data = b''
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data += packet
    return data


def on_new_client(client_socket, addr):
    file = None
    try:
        while True:
            newData = recvall(client_socket, SocketDataStructure.size)
            print(f"Received data length: {len(newData) if newData else 0} (Expected: {SocketDataStructure.size})") #extra print
            if not newData:
                break
            revivedStruct = SocketDataStructure(newData)
            if revivedStruct.isInitialData > 0:
                receivedInitialData = revivedStruct.unionData.initialData
                participant = receivedInitialData.participant.decode('utf-8')
                cleanPart = re.sub('[^A-Za-z0-9]+', '', participant)
                position = receivedInitialData.position.decode('utf-8')
                clearPos = re.sub('[^A-Za-z0-9]+', '', position)
                device_name = "device" + str(receivedInitialData.device_id)
                filename = f"data/{device_name}_{cleanPart}_{clearPos}.txt"
                if file:
                    file.close()
                file = open(filename, "a")
            else:
                receivedData = revivedStruct.unionData.data #TRIED CHANGING TO MAINDATA AS IN C FILE
                if file is None or file.closed:
                    name = f"data/defaultFile_{addr}.txt"
                    file = open(name, "a")
                print("File write section reached, writing ECG/EDA/IMU data.") #extra print
                file.write("ECG: " + str(receivedData.ecg_data_arr) + "\n")
                #file.write("EDA: " + str(receivedData.eda_data_arr) + "\n")
                file.write("ACC_X: " + str(receivedData.imu_acc_x) + "\n")
                file.write("ACC_Y: " + str(receivedData.imu_acc_y) + "\n")
                file.write("ACC_Z: " + str(receivedData.imu_acc_z) + "\n")
                file.write("GYRO_X: " + str(receivedData.imu_gyro_x) + "\n")
                file.write("GYRO_Y: " + str(receivedData.imu_gyro_y) + "\n")
                file.write("GYRO_Z: " + str(receivedData.imu_gyro_z) + "\n")
                #file.write("TIME: " + str(receivedData.time) + "\n")
                file.flush()  # ✅ Force write to disk immediately
        print(f"Connection with {addr} lost.")
    finally:
        if file and not file.closed:
            file.close()
        client_socket.close()
        print(f"Connection with {addr} closed.")


while running:
    # Establish a connection
    client_socket, addr = server_socket.accept()
    print(f"Got a connection from {addr}")
    connectionsCount += 1
    threading.Thread(target=on_new_client,
        args=(client_socket,addr),
    ).start()

    # Continuously read data from the client
server_socket.close()
