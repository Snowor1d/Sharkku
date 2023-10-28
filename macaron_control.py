#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from morai_msgs.msg import CtrlCmd
from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage


import math
def quaternion_to_yaw(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return yaw_z

def gps_to_enu(gps_coord, center_coord):
    lat = gps_coord[0]
    lon = gps_coord[1]
    hae = gps_coord[2]

    ref_lat_rad = center_coord[0]
    ref_lon_rad = center_coord[1]
    ref_hae = center_coord[2]    
        # Convert degrees to radians
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    ref_lat_rad = math.radians(center_coord[0])
    ref_lon_rad = math.radians(center_coord[1])
        
        # Earth radius in meters
    R = 6378137
        
        # Calculate Cartesian coordinates
    x = (R + hae) * math.cos(lat_rad) * math.cos(lon_rad) - (R + ref_hae) * math.cos(ref_lat_rad) * math.cos(ref_lon_rad)
    y = (R + hae) * math.cos(lat_rad) * math.sin(lon_rad) - (R + ref_hae) * math.cos(ref_lat_rad) * math.sin(ref_lon_rad)
    z = (R + hae) * math.sin(lat_rad) - (R + ref_hae) * math.sin(ref_lat_rad)

    return [x,y,z]

class s_drive():
    def __init__(self):
        rospy.init_node('s_drive', anonymous=True)
        imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size = 1)
        gps_sub = rospy.Subscriber('/gps', GPSMessage, self.gps_callback, queue_size = 1)
        cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size = 1)
        rate = rospy.Rate(30)
        cmd = CtrlCmd()
        cmd.longlCmdType = 2
        cmd.velocity = 10
        steering_cmd = [-0.2, 0.2]
        cmd_cnts = 50

        self.yaw = 0
        self.enu_q = [0, 0, 0]
        self.gps_center = [37.573617935844275, 126.8990012359786, 38.162605841221286]
        

        while not rospy.is_shutdown():
            for i in range(2):
                cmd.steering = steering_cmd[i]
                rospy.loginfo(cmd)
                print("enu coordinate : ", self.enu_q, " yaw : ", self.yaw)
                for _ in range(cmd_cnts):
                    cmd_pub.publish(cmd)
                    rate.sleep()


    def imu_callback(self, msg):
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        self.yaw = quaternion_to_yaw(x,y,z,w)

    def gps_callback(self, msg):
        self.enu_q = gps_to_enu([msg.latitude, msg.longitude, msg.altitude], self.gps_center)
        pass
if __name__ == "__main__":
    try:
        s_d = s_drive()
    except rospy.ROSInterruptException:
        pass