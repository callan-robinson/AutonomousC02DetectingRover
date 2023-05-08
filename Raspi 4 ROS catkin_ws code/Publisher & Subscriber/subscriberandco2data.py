#!/usr/bin/env python3

import time
import rospy
from geometry_msgs.msg import PoseStamped, Point

import board
import adafruit_scd30
import csv
import os

import socket

from threading import Timer

SERVER="192.168.50.111"
PORT = 9797

s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('', 5000))
s.listen(5)
print('Server is now listening')


scd=adafruit_scd30.SCD30(board.I2C())

# if os.path.exists('/home/bsb/Desktop/xydata/x_y_co2_data.csv'):
#    os.remove('/home/bsb/Desktop/xydata/x_y_co2_data.csv')


global ps 
x=1
y=1
ps=PoseStamped()

clientsocket, address = s.accept()
print(f"Connection from {address}")

def callback(data):
    slam_pose = rospy.wait_for_message('slam_out_pose', PoseStamped, timeout=1)
    x=slam_pose.pose.position.x
    y=slam_pose.pose.position.y
    co2_measure=scd.CO2
    csv_line = str(x)+','+str(y)+','+str(co2_measure)
    background_controller(csv_line)

def background_controller(message):
    print(message)
    clientsocket.send(bytes(message, "utf-8"))
    #Timer(1, background_controller).start()

'''
    with open('/home/bsb/Desktop/xydata/x_y_co2_data.csv', 'a', newline="") as csvfile:
    
        write=csv.writer(csvfile, delimiter=",")
        write.writerow([x, y, co2_measure])
        time.sleep(1)
'''

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("my_pose", Point, callback)
    rospy.spin()



if __name__ == '__main__':
    listener()




