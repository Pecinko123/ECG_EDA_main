from datetime import datetime
import re
import socket
import threading
from c_struct_manager import SocketDataStructure
import numpy as np
import time
import os

from influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS
from influxdb_client.rest import ApiException
from influxdb_client import Point, WritePrecision




server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Allow reusing the address. This is useful for quickly restarting the server
# and prevents the "Address already in use" error.
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

port = 8888
# bind the socket to a specific address and port
connectionsCount = 0
running = True
local_hostname = socket.gethostname()
ip_addresses = socket.gethostbyname_ex(local_hostname)[2]
filtered_ips = [ip for ip in ip_addresses if not ip.startswith("127.")]
print(filtered_ips)
print("Server running on: " + filtered_ips[0])
server_address = filtered_ips[0]
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
    device_name = ""
    clearPos = ""

    try:
        time_correction = -1

        while True:
            try:
                newData = recvall(client_socket, SocketDataStructure.size)

                if not newData:
                    break  # No more data from client

                revivedStruct = SocketDataStructure(newData)
                if revivedStruct.isInitialData > 0:
                    receivedInitialData = revivedStruct.unionData.initialData
                    participant = receivedInitialData.participant.decode('utf-8')
                    cleanPart = re.sub('[^A-Za-z0-9]+', '', participant)

                    position = receivedInitialData.position.decode('utf-8')
                    clearPos = re.sub('[^A-Za-z0-9]+', '', position)

                    print("device connected with battery level: " + str(revivedStruct.batteryLevel))
                    device_name = "device" + str(receivedInitialData.device_id)
                    print(f"{device_name} connected - Participant: {cleanPart}, Position: {clearPos}")
                else:
                    receivedData = revivedStruct.unionData.data

                    # Create a new InfluxDB points for emg data
                    imu_data = []
                    ecg_data = []
                    bioz_data = []
                    points = []
                    measurement = "data"


                    if time_correction < 0:
                        if len(receivedData.time) > 0 and receivedData.time[0] < 1577836800000: # Posix time 2020-01-01 0:00:00 in milliseconds
                            time_correction = time.time() * 1000 - receivedData.time[0]
                        else:
                            time_correction = 0

                    corrected_time = np.array(receivedData.time) + time_correction

                    # Create a new InfluxDB points for imu data
                    for i in range(len(receivedData.imu_acc_x)):
                        imu_data.append(
                            Point(measurement)
                            .tag("device", device_name)
                            .tag("position", clearPos)
                            #HERE ADD compass values from IST8310 magnetometer and compass
                            #
                            #
                            .field(f"{device_name}_acc_x", receivedData.imu_acc_x[i])
                            .field(f"{device_name}_acc_y", receivedData.imu_acc_y[i])
                            .field(f"{device_name}_acc_z", receivedData.imu_acc_z[i])
                            .field(f"{device_name}_gyro_x", receivedData.imu_gyro_x[i])
                            .field(f"{device_name}_gyro_y", receivedData.imu_gyro_y[i])
                            .field(f"{device_name}_gyro_z", receivedData.imu_gyro_z[i])
                            .time(int(corrected_time[i]), write_precision=WritePrecision.MS)
                        )

                    # Create time for emg
                    interpolated_time = np.interp(
                        np.linspace(0, len(receivedData.time) - 1, 100),
                        np.arange(len(receivedData.time)),
                        corrected_time
                    ).astype(int)

                    # Create a new InfluxDB points for ecg_eda
                    # --- ECG data (MAX30001) ---
                    for i in range(len(receivedData.ecg_data_arr)):
                        ecg_data.append(
                            Point(measurement)
                            .tag("device", device_name)
                            .tag("position", clearPos)
                            .field(f"{device_name}_ecg", receivedData.ecg_data_arr[i])
                            .time(int(interpolated_time[i]), write_precision=WritePrecision.MS)
                        )

                    # --- EDA (BIOZ) data (MAX30001) ---
                    bioz_time_axis = interpolated_time[::4] #BIOZ_RATIO
                    
                    
                    # --- replace 0 values in  bioz_data_arr ---
                    last_idx = 24  # Since size is 10 (CHUNK_SEND_SIZE / BIOZ_RATIO)

                    # 1. Check First Value (Index 0)
                    # Rule: Replace with the value next to it
                    if receivedData.bioz_data_arr[0] == 0:
                        receivedData.bioz_data_arr[0] = receivedData.bioz_data_arr[1]

                    # 2. Check Middle Values (Indices 1 to 8)
                    # Rule: Interpolate between two neighboring values (prev + next) / 2
                    for i in range(1, last_idx): # range(1, 9) covers indices 1 through 8
                        if receivedData.bioz_data_arr[i] == 0:
                            # Use // for integer division to stick to int32 format
                            receivedData.bioz_data_arr[i] = (receivedData.bioz_data_arr[i-1] + receivedData.bioz_data_arr[i+1]) // 2

                    # 3. Check Last Value (Index 9)
                    # Rule: Replace with the value previous to it
                    if receivedData.bioz_data_arr[last_idx] == 0:
                        receivedData.bioz_data_arr[last_idx] = receivedData.bioz_data_arr[last_idx-1]
                    
                    
                    
                    for i in range(len(bioz_time_axis)):
                        bioz_data.append(
                            Point(measurement)
                            .tag("device", device_name)
                            .tag("position", clearPos)
                            .field(f"{device_name}_eda", receivedData.bioz_data_arr[i])
                            .time(int(bioz_time_axis[i]), write_precision=WritePrecision.MS)
                        )

                    points.extend(imu_data)
                    points.extend(ecg_data)
                    points.extend(bioz_data)


                    # Write data to InfluxDB
                    URL = "http://localhost:8086"
                    #TOKEN = "nX2t9OkKI_l6WZ38-xRS4U1lOsf9aLO2kyh0XIt-orG3iAG--KxQq8Yp7nZAseEBZnP58VD4j9WTLrHjOUYzUw==" #TONNER
                    TOKEN = "Y-QCbsQ92iF1bz71XhU_cAuCOYR76cNPvmstlGcmrbRZOU4-gu4LdIZUw03O5iNh_GFHohVo_Qbj50yO6MNgoA=="  #token from ifluxDB localhost
                    ORG = "ESP32_ECG_EDA"
                    BUCKET = "ECG_EDA_V1"
                    with InfluxDBClient(url=URL, token=TOKEN, org=ORG, enable_gzip=True) as client:
                        try:
                            client.write_api(write_options=SYNCHRONOUS).write(bucket=BUCKET, org=ORG, record=points)
                        except ApiException as e:
                            print(e)
                            continue
                        except Exception as e:
                            print(e)
                            continue
            except Exception as e:
                print(e)
                continue

            # Send a response to the client
        # client_socket.send("Acknowledged".encode('utf-8'))
    except ConnectionResetError:
        print(f"Connection with {addr} lost.")
    except Exception as e:
        print(e)

    finally:
        # Close the connection
        client_socket.close()
        print(f"Connection with {addr} closed.")


try:
    while running:
        # Establish a connection (this is a blocking call)
        client_socket, addr = server_socket.accept()
        print(f"Got a connection from {addr}")
        connectionsCount += 1
        # Start a new thread to handle the client
        threading.Thread(target=on_new_client,
                         args=(client_socket, addr),
                         daemon=True,
                         ).start()
except KeyboardInterrupt:
    print("\nServer shutting down due to KeyboardInterrupt.")
finally:
    server_socket.close()
    print("Server socket closed.")
