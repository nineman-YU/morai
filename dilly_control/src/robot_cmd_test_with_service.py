#!/home/icas/anaconda3/envs/morai/bin/python3

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
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32
import sensor_msgs.point_cloud2 as pc2
from imu_module import euler_from_quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math  # 거리 계산을 위한 math 모듈 추가
import time
from morai_env import Env
from morai_msgs.msg import WoowaDillyStatus
#Service
from morai_msgs.srv import WoowaDillyEventCmdSrv
from morai_msgs.msg import DillyCmd
from morai_msgs.msg import DillyCmdResponse
from morai_msgs.msg import WoowaDillyStatus
import glob
import numpy as np
# def on_press(key):
#     print('Key %s pressed' % key)

# def on_release(key):
#     print('Key %s released' %key)
#     if key == keyboard.Key.esc: #esc 키가 입력되면 종료
#         return False

proj_UTMK = Proj(init='epsg:5178')
proj_WGS84 = Proj(init='epsg:4326')
rospy.init_node('Robot_control_node', anonymous=True)
wheel_vel_cmd_pub = rospy.Publisher('/6wheel_skid_ctrl_cmd', SkidSteer6wUGVCtrlCmd, queue_size=1)
vel_data = SkidSteer6wUGVCtrlCmd()

mission_get_list = ['get_5', 'get_1and4', 'get_2and3']
mission_send_list = ['send_5', 'send_4and1', 'send_2and3']
morai_env = Env(get_list=mission_get_list, send_list=mission_send_list)
# lidar_pub = rospy.Publisher('/velodyne_distance', Float32MultiArray, queue_size=10)  # 수정: Float32 메시지 사용

def Item_check(data):
    global Item_list
    rospy.loginfo('Get Item : {}'.format(data.deliveryItem))
    Item_list = data.deliveryItem
    return Item_list

rospy.wait_for_service("/WoowaDillyEventCmd")
service_client = rospy.ServiceProxy("/WoowaDillyEventCmd", WoowaDillyEventCmdSrv)

request_srv = DillyCmd()
response_srv = DillyCmdResponse()
# 적재함 확인 Network
Status_Subscriber = rospy.Subscriber('/WoowaDillyStatus', WoowaDillyStatus, Item_check) 

get_5_pos = np.array([967167.578409669, 1935338.60607898])
send_5_pos = np.array([967510.5185, 1935269.869])

try:
    Kp_angular = 1.0
    gps_x_timeline = []
    gps_y_timeline = []
    timeline = []
    target_arrived_flag = False
    # lidar_manager = LIDARConnector('UDP')
    gps_manager = GPSConnector('ROS')
    imu_manager = IMUConnector('ROS')

    # lidar_manager.connect('127.0.0.1', 2368, '')
    gps_manager.connect('', 0, '/gps')
    imu_manager.connect('1', 1, '/imu')
    coexto5_file = sorted(glob.glob('/home/icas/woowa/src/dilly_control/src/waypoints/sampled/interval/test/*.csv'))
    
    #coexto5 = pd.read_csv('/home/icas/woowa/src/dilly_control/src/waypoints/sampled/interval/test/Startto5box.csv')
    cnt_waypoint = 0
    print('Connect success')

    # 대기: LIDAR 데이터가 수신되고 처리될 때까지 기다립니다.
    # while not lidar_manager.recvChk:
    #     pass
    x = 0
    for idx, file_name in enumerate(coexto5_file):
    #for i in range(1):
        print("DDDDDDDDDDDDDDDDDDDDD")
        # if lidar_manager.connChk:
        #     lidar_data = lidar_manager.getLidar()
        #     # print("Lidar ", lidar_data[0].shape, lidar_data[1].shape, lidar_data[2].shape)
        # else:
        #     print("Not connected")

        # lidar_x = list(lidar_data[0])
        # lidar_y = list(lidar_data[1])
        # lidar_z = list(lidar_data[2])

        # lidar_points = []
        # lidar_distances = []
        # distances = []
        # num_points_to_keep = 10  # 최종적으로 남길 포인트 수
        # angle_interval = 18  # 18도마다 하나의 포인트 선택
        # current_angle = 0  # 현재 각도

        # # 정확히 30cm 높이에 있는 포인트만 선택
        # target_height = 0.3

        # for i in range(len(lidar_x)):
        #         current_angle += angle_interval
        #     if abs(lidar_z[i] - target_height) < 0.01:  # 30cm 높이 근처에 있는 포인트 선택
        #         x, y, z = lidar_x[i], lidar_y[i], lidar_z[i]
        #         distance = math.sqrt(x**2 + y**2 + z**2)  # 3D 좌표에서 Euclidean 거리 계산
        #         lidar_points.append([x, y, z]) 
        #         lidar_distances.append(distance) # 거리 정보도 함께 저장
        #         if current_angle >= 180:  # 180도까지 선택한 경우 종료
        #             break

        # # float_array_msg = Float32MultiArray()  # Float32MultiArray 메시지 생성
        # # float_array_msg.data = lidar_distances  # 거리 값 리스트를 메시지에 할당
        # # lidar_pub.publish(float_array_msg)  # 거리 값을 리스트로 publish
        cnt_waypoint = 0
        response_flag = False
        coexto5 = pd.read_csv(file_name)
        print(file_name)
        print(coexto5)
        while True:
            print("idx : ", idx)
            print("rsp : ", response_flag)
            # for i in range(len(coexto5['X'])):
            # time_ = time.strftime('%X', time.localtime(time.time()))
        
            # linear_velocity = 0.0
            # angular_velocity = 0.0             

            #print([time_, pos_x, pos_y])
            pos_x, pos_y = gps_manager.getPose()
            pos_x, pos_y = transform(proj_WGS84, proj_UTMK, pos_x, pos_y)

            current_pos = np.array([pos_x,pos_y])
            print("pos : ",current_pos)
            goal_x = coexto5['X'][cnt_waypoint]
            goal_y = coexto5['Y'][cnt_waypoint]
            
            #print("current_pos : {}".format(current_pos))
            #print("goal_x :{},goal_y:{}".format(goal_x,goal_y))
            angular_velocity, goal_to_current_pos_distance = morai_env.waypoint_driving_test(pos_x, pos_y, goal_x, goal_y)
            #angular_velocity = morai_env.normalize(angular_velocity, -math.pi, math.pi, -0.83, 0.83)        
            #print("heading : ", angular_velocity)
            linear_velocity = 2
            angular_velocity *= Kp_angular
            
            target_pos_x = coexto5['X'][len(coexto5['X'])-1]
            target_pos_y = coexto5['Y'][len(coexto5['X'])-1]
            print(target_pos_x)
            
            target_pos = np.array([target_pos_x,target_pos_y])
            dily_to_target_distance = morai_env.target_to_distance(current_pos, target_pos)

            # =========== Set ROS msg value =========== #
            vel_data.cmd_type = 3
            vel_data.Forward_input = True
            vel_data.Backward_input = True
            vel_data.Left_Turn_input = True
            vel_data.Right_Turn_input = True
            print("distance : ", dily_to_target_distance)
            if dily_to_target_distance < 1.2:
                print("Service---------------")
                if (idx == 0 or idx == 2):
                    vel_data.Target_linear_velocity = 0.0
                    vel_data.Target_angular_velocity = 0.0
                elif idx == 1 or idx == 3:
                    vel_data.Target_linear_velocity = 2.0
                    vel_data.Target_angular_velocity = -angular_velocity

                if idx == 0:
                    request_srv.isPickup = True
                    request_srv.deliveryItemIndex = 5
                    service_client(request_srv)
                elif idx == 2:
                    request_srv.isPickup = False
                    request_srv.deliveryItemIndex = 5
                    service_client(request_srv)
                # =========== Set ROS msg value =========== #
                # 1번위치에 도달하면 물품 수령!
                # if pos == get_1_pos:
                #     request_srv.isPickup = True
                #     request_srv.deliveryItemIndex = 1
                # 2번위치에 도달하면 물품 수령!
                # elif pos == get_2_pos:
                #     request_srv.isPickup = True
                #     request_srv.deliveryItemIndex = 2
                # # 3번위치에 도달하면 물품 수령!
                # elif pos == get_3_pos:
                #     request_srv.isPickup = True
                #     request_srv.deliveryItemIndex = 3
                # # 4번위치에 도달하면 물품 수령!
                # elif pos == get_4_pos:
                #     request_srv.isPickup = True
                #     request_srv.deliveryItemIndex = 4
                # 5번위치에 도달하면 물품 수령!
                # if current_pos == get_5_pos:
                #     request_srv.isPickup = True
                #     request_srv.deliveryItemIndex = 5
                
                # # 1번위치에 도달하면 물품 전달!
                # elif pos == send_1_pos:
                #     request_srv.isPickup = False
                #     request_srv.deliveryItemIndex = 1
                # # 2번위치에 도달하면 물품 전달!
                # elif pos == send_2_pos:
                #     request_srv.isPickup = False
                #     request_srv.deliveryItemIndex = 2
                # # 3번위치에 도달하면 물품 전달!
                # elif pos == send_3_pos:
                #     request_srv.isPickup = False
                #     request_srv.deliveryItemIndex = 3       
                # # 4번위치에 도달하면 물품 전달!
                # elif pos == send_4_pos:
                #     request_srv.isPickup = False
                #     request_srv.deliveryItemIndex = 4
                # # 5번위치에 도달하면 물품 전달!
                # elif pos == send_5_pos:
                # request_srv.isPickup = False
                # request_srv.deliveryItemIndex = 5
                
                print(response_srv.result)
                first_time = rospy.Time.now()
                while rospy.Time.now().to_sec() - first_time.to_sec() < 1.0:
                    wheel_vel_cmd_pub.publish(vel_data)
                if (idx == 0 or idx == 2) and response_srv.result:
                    response_flag = True
                # if (idx == 0 or idx == 2):
                #     response_flag = True
                
                elif idx == 1 or idx == 3:
                    response_flag = True
                    
            else:
                vel_data.Target_linear_velocity = linear_velocity
                vel_data.Target_angular_velocity = -angular_velocity
            
                wheel_vel_cmd_pub.publish(vel_data)

            if goal_to_current_pos_distance <= 0.2:
                cnt_waypoint += 1
                
            if response_flag:
                break
        
        first_time = rospy.Time.now()
        
        if idx == 0 or idx == 2:
            vel_data.Target_linear_velocity = -2
            vel_data.Target_angular_velocity = 0.83
            while rospy.Time.now().to_sec() - first_time.to_sec() < 4.0:
                wheel_vel_cmd_pub.publish(vel_data)
            

            
            # ========================================= #

            # gps_x_timeline.append(pos_x)
            # gps_y_timeline.append(pos_y)
            # timeline.append(time_)
            #time.sleep(1/3)
            # imu_data = imu_manager.getIMU()
            # orientation_list = [imu_data.orientation_x, imu_data.orientation_y, imu_data.orientation_z, imu_data.orientation_w]
            # _, _, yaw = euler_from_quaternion(orientation_list)
            # print("Yaw: ", yaw)
            #x += 1
        #print("Disconnect lidar")
        # lidar_manager.disconnect()
        #  del lidar_manager
    

# except NetworkError as e:
#     print(f"NetworkError: {e}")

except Exception as e:
    print(f"Exception: {e}")

# pub_robot_cmd = rospy.Publisher('/6wheel_skid_ctrl_cmd', SkidSteer6wUGVCtrlCmd, queue_size=10)

# count = 0

# while not rospy.is_shutdown():

#     if count < 1000000:
#         linear_vel = 2
#         angular_vel = 0
#     else:
#         linear_vel = 0

#     vel_cmd = SkidSteer6wUGVCtrlCmd()

#     vel_cmd.cmd_type = 3

#     vel_cmd.Target_linear_velocity = linear_vel
#     vel_cmd.Target_angular_velocity = angular_vel

#     vel_cmd.Forward_input = True
#     vel_cmd.Backward_input = True
#     vel_cmd.Left_Turn_input = True
#     vel_cmd.Right_Turn_input = True

#     pub_robot_cmd.publish(vel_cmd)
#     count += 1