import socket
import csv
import os

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("192.168.50.94", 5000))

script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)

csv_path = '/Users/callanrobinson/VSCode Projects/fire_truck_robot/HeatMapping/co2_concentrations.csv'

if os.path.exists(csv_path):
    os.remove(csv_path)

while True:
    new_data_point = s.recv(4096).decode("utf-8")
    
    if new_data_point != '':
        with open(csv_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            values = new_data_point.split(',')
            writer.writerow(values)