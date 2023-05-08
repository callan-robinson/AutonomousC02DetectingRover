# import socket 
# import csv

# with open('/home/bsb/Desktop/xydata/x_y_co2_data.csv', 'r') as csv_file:
#     csv_data=csv_file.read()

# with open('~/home/bsb/catkin_ws/src/maps/mymaps.png', 'rb') as png_file:
#     png_data=png_file.read()


# port_number=5000

# client_socket=socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# server_address=('192.168.50.111', port_number)
# client_socket.accept(server_address)

# client_socket.sendall(csv_data.encode())

# client_socket.sendall(png_data)

# client_socket.close()

import socket
import time
from threading import Timer

SERVER="192.168.50.111"
PORT = 5000

s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('', 5000))
s.listen(5)
print('Server is now listening')

def background_controller():
    message='robot!'
    print(message)
    clientsocket.send(bytes(message, "utf-8"))
    Timer(5, background_controller).start()



clientsocket, address =s.accept()
print(f"Connection from {address}")
background_controller()
# client=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# client.connect((SERVER, PORT))
# client.send("hellow world!".encode())
# time.sleep(4)
# client.shutdown(1)
# client.close