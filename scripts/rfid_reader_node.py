#!/usr/bin/env python
import serial
import rospy
from std_msgs.msg import String
import argparse
import sys


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-b', '--baud', help="The baud rate of the serial device. Default: 9600.", required=False)
    parser.add_argument('serialID', help="The serial device to read from. Example: ttyUSB0")
    args = parser.parse_args()

    serial_device = args.serialID
    baud_rate = 9600
    if args.baud:
        baud_rate = args.baud

    rospy.init_node("rfid_reader")
    pub = rospy.Publisher("rfid_data", String, queue_size=10)

    ser = serial.Serial('/dev/' + serial_device, baud_rate)
    rospy.loginfo("Node `rfid_reader` started.")
    while True:
        try:
            data = ser.readline()
            pub.publish(data)
        except KeyboardInterrupt, e:
            sys.exit(0)

