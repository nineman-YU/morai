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

csv_file_path = '/home/icas/woowa/src/dilly_control/src/waypoints/original/kbj/mission_1and4/4box_to_d_kbj_inv.csv'
waypoint_inv = pd.read_csv(csv_file_path)

timeline = waypoint_inv['Time'].to_numpy()
x_inv_list = waypoint_inv['X'].to_numpy()
y_inv_list = waypoint_inv['Y'].to_numpy()

x_list = np.flip(x_inv_list)
y_list = np.flip(y_inv_list)

df = pd.DataFrame({'Time':timeline, 'X':x_list, 'Y':y_list})
df.to_csv('/home/icas/woowa/src/dilly_control/src/waypoints/original/kbj/mission_1and4/4box_to_d_kbj.csv')




