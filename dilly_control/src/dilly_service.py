#!/home/wonyeol/anaconda3/envs/morai/bin/python3

import sys, os
import pandas as pd
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
# import keyboard
import rospy
from pyproj import Proj, transform
from morai_msgs.msg import SkidSteer6wUGVCtrlCmd
from Lidarprocess import LIDARConnector
from GPSprocess import GPSConnector
from IMUprocess import IMUConnector
from Lidarprocess import LIDARConnector
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from sensor_msgs.msg import PointCloud2 
from sensor_msgs.point_cloud2 import create_cloud_xyz32
import sensor_msgs.point_cloud2 as pc2
from imu_module import euler_from_quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math  # 거리 계산을 위한 math 모듈 추가
import time
from morai_msgs.msg import WoowaDillyStatus
#Service
from morai_msgs.srv import WoowaDillyEventCmdSrv
from morai_msgs.msg import DillyCmd
from morai_msgs.msg import DillyCmdResponse
from morai_msgs.msg import WoowaDillyStatus
import copy
import glob
import numpy as np
from scipy.linalg import inv
from math import pi

from scipy.integrate import cumtrapz


proj_UTMK = Proj(init='epsg:5178')
proj_WGS84 = Proj(init='epsg:4326')

person_list = []
# 초기 GPS 위치 (위도, 경도)
init_pos_x, init_pos_y = transform(proj_WGS84, proj_UTMK, 127.1265, 37.4193)
initial_gps_location = np.array([init_pos_x, init_pos_y])

# 초기 IMU 가속도 및 각속도
initial_acceleration = np.array([0.0, 0.0, 9.8])  # m/s^2
initial_angular_velocity = np.array([0.0, 0.0, 0.0])  # rad/s

# IMU 및 GPS 데이터의 시간 간격
dt = 0.1  # 초

def emergency_turn(flag):
    if flag == 'left':
        vel_data.Target_linear_velocity = 0
        vel_data.Target_angular_velocity = -0.83
        wheel_vel_cmd_pub.publish(vel_data)
        time.sleep(1.29)
        vel_data.Target_linear_velocity = 2
        vel_data.Target_angular_velocity = 0
        wheel_vel_cmd_pub.publish(vel_data)
        time.sleep(0.5)
        vel_data.Target_linear_velocity = 0
        vel_data.Target_angular_velocity = 0.83
        wheel_vel_cmd_pub.publish(vel_data)
        time.sleep(1.29)
        vel_data.Target_linear_velocity = 2
        vel_data.Target_angular_velocity = 0
        wheel_vel_cmd_pub.publish(vel_data)
        time.sleep(0.5)
    elif flag == 'right':
        vel_data.Target_linear_velocity = 0
        vel_data.Target_angular_velocity = 0.83
        wheel_vel_cmd_pub.publish(vel_data)
        time.sleep(1.29)
        vel_data.Target_linear_velocity = 2
        vel_data.Target_angular_velocity = 0
        wheel_vel_cmd_pub.publish(vel_data)
        time.sleep(0.5)
        vel_data.Target_linear_velocity = 0
        vel_data.Target_angular_velocity = -0.83
        wheel_vel_cmd_pub.publish(vel_data)
        time.sleep(1.29)
        vel_data.Target_linear_velocity = 2
        vel_data.Target_angular_velocity = 0
        wheel_vel_cmd_pub.publish(vel_data)
        time.sleep(0.5)
    else:
        vel_data.Target_linear_velocity = 0
        vel_data.Target_angular_velocity = 0
        time.sleep(0.5)

def person_callback(data):
    global person_list
    person_list = data.data

try:
    #lidar_manager = LIDARConnector('UDP')
    gps_manager = GPSConnector('ROS')
    imu_manager = IMUConnector('ROS')
    #lidar_manager.connect('127.0.0.1', 2368, '')
    gps_manager.connect('', 0, '/gps')
    imu_manager.connect('1', 1, '/imu')
    # while not lidar_manager.recvChk:
    #     pass
except:
    pass

gps_noise_info_y_1 = 0
gps_noise_info_y_1 = 0



rospy.init_node('Robot_control_node', anonymous=True)

wheel_vel_cmd_pub = rospy.Publisher('/6wheel_skid_ctrl_cmd', SkidSteer6wUGVCtrlCmd, queue_size=1)
vel_data = SkidSteer6wUGVCtrlCmd()
person_sub = rospy.Subscriber('/person_detect', Int32MultiArray, callback=person_callback)

# lidar_pub = rospy.Publisher('/velodyne_distance', Float32MultiArray, queue_size=10)  # 수정: Float32 메시지 사용
request_srv = DillyCmd()
response_srv = DillyCmdResponse()
# def get_lidar_diff():
#     for i in range(10):
#         a.append(get_Lidar_Val())
#     diff = np.diff(a)
#     diff.all() == 0
#     finish_flag = True
#     return finish_flag

# 초기 상태 및 공분산 행렬 설정
initial_state = np.array([0, 0])  # 초기 상태 [x, y]
initial_covariance = np.diag([1, 1])  # 초기 공분산 행렬

# 프로세스 노이즈 및 측정 노이즈 공분산 행렬 설정
process_noise = np.diag([0.1, 0.1])  # 프로세스 노이즈 공분산
measurement_noise = np.diag([1, 1])  # 측정 노이즈 공분산

# 상태 전이 행렬과 관측 행렬 설정 (이 값들은 GPS 데이터에 따라 조절해야 함)
A = np.array([[1, 1], [0, 1]])  # 상태 전이 행렬
H = np.array([[1, 0], [0, 1]])  # 관측 행렬

# 칼만 필터 초기화


state = initial_state
covariance = initial_covariance


def get_angvel_and_goaldist(current_pos_x, current_pos_y, goal_x, goal_y):
    # position_x, position_y = self.gps_manager.getPose()
    # self.position_x, self.position_y = transform(self.proj_WGS84, self.proj_UTMK, position_x, position_y)
    imu_data = imu_manager.getIMU()
    orientation_list = [imu_data.orientation_x, imu_data.orientation_y, imu_data.orientation_z, imu_data.orientation_w]
    _, _, yaw = euler_from_quaternion(orientation_list)
    goal_angle = math.atan2(goal_y - current_pos_y, goal_x - current_pos_x)
    heading = goal_angle - yaw
    if heading > pi:
        heading -= 2 * pi
    elif heading < -pi:
        heading += 2 * pi
    # self.goal_angle_list.append(goal_angle)
    # self.yaw_list.append(yaw)
    heading_error = round(heading, 3)
    goal_to_current_pos_distance = round(math.hypot(goal_x - current_pos_x, goal_y - current_pos_y), 2)
    
    return heading_error, goal_to_current_pos_distance


def mission_to_distance(current_pos, target_pos):
    dily_to_target_distance = np.sqrt(np.square(current_pos - target_pos).sum())
    return dily_to_target_distance


def Item_check(data):
    global Item_list
    #rospy.loginfo('Get Item : {}'.format(data.deliveryItem))
    Item_list = data.deliveryItem
    return Item_list

# 적재함 확인 Network
Status_Subscriber = rospy.Subscriber('/WoowaDillyStatus', WoowaDillyStatus, Item_check)

rospy.wait_for_service("/WoowaDillyEventCmd")
service_client = rospy.ServiceProxy("/WoowaDillyEventCmd", WoowaDillyEventCmdSrv)


def update_gps_with_imu(gps_location, imu_acceleration, imu_angular_velocity, dt):
    # IMU 가속도 데이터를 적분하여 속도를 계산
    velocity_change = cumtrapz(imu_acceleration, dx=dt, initial=0.0)
    
    # 속도를 적분하여 위치를 계산
    position_change = cumtrapz(velocity_change[1:], dx=dt, initial=0.0)

    # GPS 위치를 IMU 데이터로 업데이트
    updated_gps_location = gps_location + position_change
    
    return updated_gps_location


def sensor_fusion_with_imu(pos_x, pos_y, imu_data):
    # Extract IMU data
    imu_acceleration = np.array([imu_data.linear_acceleration_x, imu_data.linear_acceleration_y, imu_data.linear_acceleration_z])
    imu_angular_velocity = np.array([imu_data.angular_velocity_x, imu_data.angular_velocity_y, imu_data.angular_velocity_z])

    # Perform sensor fusion to improve the GPS data
    fused_pos_x, fused_pos_y = update_gps_with_imu(np.array([pos_x, pos_y]), imu_acceleration, imu_angular_velocity, dt)

    return fused_pos_x, fused_pos_y


def tracking_waypoint(file_name, idx):
  
    global gps_noise_info_x_2
    global gps_noise_info_y_2
    
    gps_noise_info_x = 0
    gps_noise_info_y = 0
    
    gt_x_2 = 967145.48993318
    gt_y_2 = 1935340.43417661

    if idx == 0:
        for i in range(2000):
            print(i)
            pos_x, pos_y = gps_manager.getPose()
            pos_x, pos_y = transform(proj_WGS84, proj_UTMK, pos_x, pos_y)
            # print("px : ", pos_x)
            # print("py : ", pos_y)
            gps_noise_info_x += pos_x
            gps_noise_info_y += pos_y

        gps_noise_info_x /= 2000
        gps_noise_info_y /= 2000

        gps_noise_info_x_2 = gps_noise_info_x - gt_x_2
        gps_noise_info_y_2 = gps_noise_info_y - gt_y_2

    waypoint_x = []
    waypoint_y = []
    
    cnt_waypoint = 0
    waypoint_interval = 6
    Kp_angular = 1
    dilly_to_mission_distance = None  
    arrived_mission_pos = False
    response_flag = False

    waypoint_pos = pd.read_csv(file_name)
    
    for i in range(0, waypoint_pos['X'].shape[0]):
        if i == 0:
            waypoint_x.append(waypoint_pos['X'][i])
            waypoint_y.append(waypoint_pos['Y'][i])
        elif i == waypoint_pos['X'].shape[0]-1:
            waypoint_x.append(waypoint_pos['X'][i])
            waypoint_y.append(waypoint_pos['Y'][i])
        else:
            if math.sqrt(math.pow(waypoint_pos['X'][i] - waypoint_x[-1], 2) + math.pow(waypoint_pos['Y'][i] - waypoint_y[-1], 2)) >= waypoint_interval:
                waypoint_x.append(waypoint_pos['X'][i])
                waypoint_y.append(waypoint_pos['Y'][i])    
        
    waypoint_pos = pd.DataFrame({'X':waypoint_x, 'Y':waypoint_y})
    
    vel_data.cmd_type = 3
    vel_data.Forward_input = True
    vel_data.Backward_input = True
    vel_data.Left_Turn_input = True
    vel_data.Right_Turn_input = True

    black_out_x_min = 967115.8455480705
    black_out_x_max = 967180.2231423993
    black_out_y_min = 1935313.7761079022
    black_out_y_max = 1935370.2139747061

    while True:
        #print("noise info : {}, {}".format(gps_noise_info_x_1, gps_noise_info_y_1) )
        pos_x, pos_y = gps_manager.getPose()
        pos_x, pos_y = transform(proj_WGS84, proj_UTMK, pos_x, pos_y)

        goal_x = waypoint_pos['X'][cnt_waypoint]
        goal_y = waypoint_pos['Y'][cnt_waypoint]

        if black_out_x_min < pos_x - gps_noise_info_x_2 < black_out_x_max and black_out_y_min < pos_y - gps_noise_info_y_2 < black_out_y_max:
            pos_x = pos_x - gps_noise_info_x_2
            pos_y = pos_y - gps_noise_info_y_2
            goal_x = waypoint_pos['X'][cnt_waypoint] - gps_noise_info_x_2
            goal_y = waypoint_pos['Y'][cnt_waypoint] - gps_noise_info_y_2

        pos_x, pos_y = sensor_fusion_with_imu(pos_x, pos_y, imu_manager.getIMU())
        current_pos = np.array([pos_x,pos_y])
        # current_pos = kalman_filter(current_pos)
        print("currnet_pos : ",current_pos)
        
        angular_velocity, goal_to_current_pos_distance = get_angvel_and_goaldist(pos_x, pos_y, goal_x, goal_y)
        angular_velocity *= Kp_angular

        print("file idx : {}, file name : {}".format(idx, file_name[-5:]))
        print("wayp idx : {}, wayp intv : {}".format(cnt_waypoint, waypoint_interval))
        print("distance to goal waypoint : ", goal_to_current_pos_distance)
        print("distance to mission pos : ", dilly_to_mission_distance)

        #### General
        if black_out_x_min < pos_x - gps_noise_info_x_2 < black_out_x_max and black_out_y_min < pos_y - gps_noise_info_y_2 < black_out_y_max:
            if abs(angular_velocity) > 0.3 and abs(angular_velocity) <= 0.6:
                linear_velocity = 0.8
            elif abs(angular_velocity) > 0.6:
                linear_velocity = 0.4
            else:
                linear_velocity = 1.5
        else:
            if abs(angular_velocity) > 0.3 and abs(angular_velocity) <= 0.6:
                linear_velocity = 1.0
            elif abs(angular_velocity) > 0.6:
                linear_velocity = 0.5
            else:
                linear_velocity = 2
        #### Near Goal
        if idx == 0 or idx == 2 or idx == 4 or idx == 5 or idx == 7 or idx == 8 or idx == 10 or idx == 11 or idx == 13 or idx == 14:
            mission_x = waypoint_pos['X'][len(waypoint_pos['X'])-1]
            mission_y = waypoint_pos['Y'][len(waypoint_pos['X'])-1]

            if black_out_x_min < pos_x - gps_noise_info_x_2 < black_out_x_max and black_out_y_min < pos_y - gps_noise_info_y_2 < black_out_y_max:
                mission_x = waypoint_pos['X'][len(waypoint_pos['X'])-1] - gps_noise_info_x_2
                mission_y = waypoint_pos['Y'][len(waypoint_pos['X'])-1] - gps_noise_info_y_2

            mission_pos = np.array([mission_x,mission_y])

            dilly_to_mission_distance = mission_to_distance(current_pos, mission_pos)

            if dilly_to_mission_distance < 0.5:
                linear_velocity = 0.0
                angular_velocity = 0.0

                arrived_mission_pos = True

        if cnt_waypoint != len(waypoint_pos['X']-1):
            if len(person_list) > 0:
                if len(person_list) > 5:
                    linear_velocity = 0
                    angular_velocity = 0
                else:
                    if person_list[2] > 10000:
                        if 0 < person_list[0] < 250:
                            emergency_turn('right')
                        elif 390 < person_list[0] < 640:
                            emergency_turn('left')
                        else:
                            pass

        vel_data.Target_linear_velocity = linear_velocity
        vel_data.Target_angular_velocity = -angular_velocity

        wheel_vel_cmd_pub.publish(vel_data)

        if arrived_mission_pos:
            for _ in range(3):
                wheel_vel_cmd_pub.publish(vel_data)
                time.sleep(0.5)

        if idx == 0 and arrived_mission_pos:
            request_srv.isPickup = True
            request_srv.deliveryItemIndex = 5
            arrived_mission_pos = False
            service_client(request_srv)
            # time.sleep(1)
            if len(Item_list) == 1:
                response_flag = True
        
        elif idx == 2 and arrived_mission_pos:
            request_srv.isPickup = False
            request_srv.deliveryItemIndex = 5
            arrived_mission_pos = False
            service_client(request_srv)
            # time.sleep(1)
            if len(Item_list) == 0:
                response_flag = True

        elif idx == 4 and arrived_mission_pos:
            request_srv.isPickup = True
            request_srv.deliveryItemIndex = 1
            arrived_mission_pos = False
            service_client(request_srv)
            #time.sleep(1)
            if len(Item_list) == 1:
                response_flag = True

        elif idx == 5 and arrived_mission_pos:
            request_srv.isPickup = True
            request_srv.deliveryItemIndex = 4
            arrived_mission_pos = False
            service_client(request_srv)
            #time.sleep(1)
            if len(Item_list) == 2:
                response_flag = True

        elif idx == 7 and arrived_mission_pos:
            request_srv.isPickup = False
            request_srv.deliveryItemIndex = 4
            arrived_mission_pos = False
            service_client(request_srv)
            #time.sleep(1)
            if len(Item_list) == 1:
                response_flag = True            

        elif idx == 8 and arrived_mission_pos:
            request_srv.isPickup = False
            request_srv.deliveryItemIndex = 1
            arrived_mission_pos = False
            service_client(request_srv)
            #time.sleep(1)
            if len(Item_list) == 0:            
                response_flag = True

        elif idx == 10 and arrived_mission_pos:
            request_srv.isPickup = True
            request_srv.deliveryItemIndex = 2
            arrived_mission_pos = False
            service_client(request_srv)
            #time.sleep(1)
            if len(Item_list) == 1:
                response_flag = True  

        elif idx == 11 and arrived_mission_pos:
            request_srv.isPickup = True
            request_srv.deliveryItemIndex = 3
            arrived_mission_pos = False
            service_client(request_srv)
            #time.sleep(1)
            if len(Item_list) == 2:
                response_flag = True  

        elif idx == 13 and arrived_mission_pos:
            request_srv.isPickup = False
            request_srv.deliveryItemIndex = 2
            arrived_mission_pos = False
            service_client(request_srv)
            #time.sleep(1)
            if len(Item_list) == 1:
                response_flag = True  

        elif idx == 14 and arrived_mission_pos:
            request_srv.isPickup = False
            request_srv.deliveryItemIndex = 3
            arrived_mission_pos = False
            service_client(request_srv)
            #time.sleep(1)
            if len(Item_list) == 0:
                response_flag = True  

        #if linear_velocity == 0.0 and angular_velocity == 0.0:
            #time.sleep(1)

        if goal_to_current_pos_distance <= 0.3:
            cnt_waypoint += 1        
        print("response flag : ", response_flag)
        print("response result : ", response_srv.result)
        if response_flag or (cnt_waypoint-1) == (len(waypoint_pos['X'])-1) or np.isnan(goal_to_current_pos_distance):
            cnt_waypoint = 0
            break
        #print("Item_list :",len(Item_list))
def back_moving():

    vel_data.cmd_type = 3
    vel_data.Forward_input = True
    vel_data.Backward_input = True
    vel_data.Left_Turn_input = True
    vel_data.Right_Turn_input = True

    first_time = rospy.Time.now()
    while rospy.Time.now().to_sec() - first_time.to_sec() < 0.25:
        vel_data.Target_linear_velocity = -2
        vel_data.Target_angular_velocity = 0.0
        wheel_vel_cmd_pub.publish(vel_data)

    second_time = rospy.Time.now()
    while rospy.Time.now().to_sec() - second_time.to_sec() < 0.25:
        vel_data.Target_linear_velocity = 0.0
        vel_data.Target_angular_velocity = 0.0
        wheel_vel_cmd_pub.publish(vel_data)

    third_time = rospy.Time.now()
    while rospy.Time.now().to_sec() - third_time.to_sec() < 3:
        vel_data.Target_linear_velocity = 0.0
        vel_data.Target_angular_velocity = 0.83
        wheel_vel_cmd_pub.publish(vel_data)


def front_moving():
    vel_data.cmd_type = 3
    vel_data.Forward_input = True
    vel_data.Backward_input = True
    vel_data.Left_Turn_input = True
    vel_data.Right_Turn_input = True
    pos_x, pos_y = gps_manager.getPose()
    pos_x, pos_y = transform(proj_WGS84, proj_UTMK, pos_x, pos_y)

    first_time = rospy.Time.now()
    while rospy.Time.now().to_sec() - first_time.to_sec() < 1:
        vel_data.Target_linear_velocity = 2
        vel_data.Target_angular_velocity = 0.0
        wheel_vel_cmd_pub.publish(vel_data)

    second_time = rospy.Time.now()
    while rospy.Time.now().to_sec() - second_time.to_sec() < 1:
        vel_data.Target_linear_velocity = 0.0
        vel_data.Target_angular_velocity = 0.0
        wheel_vel_cmd_pub.publish(vel_data)

def smooth_moving():
    vel_data.cmd_type = 3
    vel_data.Forward_input = True
    vel_data.Backward_input = True
    vel_data.Left_Turn_input = True
    vel_data.Right_Turn_input = True
    pos_x, pos_y = gps_manager.getPose()
    pos_x, pos_y = transform(proj_WGS84, proj_UTMK, pos_x, pos_y)

    first_time = rospy.Time.now()
    while rospy.Time.now().to_sec() - first_time.to_sec() < 2:
        vel_data.Target_linear_velocity = 0
        vel_data.Target_angular_velocity = -0.83
        wheel_vel_cmd_pub.publish(vel_data)
    
    second_time = rospy.Time.now()
    while rospy.Time.now().to_sec() - second_time.to_sec() < 1.5:
        vel_data.Target_linear_velocity = 2.0
        vel_data.Target_angular_velocity = 0.0
        wheel_vel_cmd_pub.publish(vel_data)

waypoint_file_name = sorted(glob.glob('/home/wonyeol/catkin_ws/src/dilly_control/src/waypoints/waypoint_with_noise/*.csv'))
#print(waypoint_file_name)

back_moving_list = [0, 4, 5, 10, 11]
forward_moving_list = [2, 7, 8, 13]
for idx, file_name in enumerate(waypoint_file_name):

    tracking_waypoint(file_name, idx)
    if idx in back_moving_list:
        back_moving()
    elif idx in forward_moving_list:
        front_moving()
        smooth_moving()






























