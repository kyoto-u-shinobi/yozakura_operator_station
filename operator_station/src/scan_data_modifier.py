#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(scan):
    scan.range_min = 0.25
    pub.publish(scan)
    print("I modified scan and pub!!")

    

rospy.init_node('scan_data_modifyer')
sub = rospy.Subscriber('scan', LaserScan, callback)
pub = rospy.Publisher('scan_modified', LaserScan, queue_size=100)
rospy.spin()
    

