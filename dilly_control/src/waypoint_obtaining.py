#!/home/icas/anaconda3/envs/morai/bin/python3

import sys, os
import pandas as pd
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import tty
import termios
import rospy
from pyproj import Proj, transform
from morai_msgs.msg import SkidSteer6wUGVCtrlCmd
from Lidarprocess import LIDARConnector
from GPSprocess import GPSConnector
from IMUprocess import IMUConnector
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32
import sensor_msgs.point_cloud2 as pc2
from imu_module import euler_from_quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math  # 거리 계산을 위한 math 모듈 추가
import time
import copy
import numpy as np


csv_file_path = '/home/wonyeol/catkin_ws/src/dilly_control/src/waypoints/waypoints_gunamnam.csv'
proj_UTMK = Proj(init='epsg:5178')
proj_WGS84 = Proj(init='epsg:4326')
rospy.init_node('Robot_control_node', anonymous=True)

try:

    gps_x_timeline = []
    gps_y_timeline = []
    timeline = []

    gps_manager = GPSConnector('ROS')
    gps_manager.connect('', 0, '/gps')

    while True:
        
        time_ = time.strftime('%x %X')
        pos_x, pos_y = gps_manager.getPose()

        pos_x, pos_y = transform(proj_WGS84, proj_UTMK, pos_x, pos_y)
        gps = [time_, pos_x, pos_y]
        
        print(gps)
        gps_x_timeline.append(pos_x)
        gps_y_timeline.append(pos_y)
        timeline.append(time_)
        df = pd.DataFrame({'Time':timeline, 'X':gps_x_timeline, 'Y':gps_y_timeline})
        df.to_csv(csv_file_path)

except Exception as e:

    print(f"Exception: {e}")

