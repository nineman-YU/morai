#!/home/icas/anaconda3/envs/morai/bin/python3


import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import rospy
from morai_msgs.msg import SkidSteer6wUGVCtrlCmd, CollisionData
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
import numpy as np
from math import pi
import pandas as pd
from pyproj import Proj, transform


class Env():
    def __init__(self, action_dim=2):
        # rospy.init_node('Robot_control_node', anonymous=True)

        self.lidar_pub = rospy.Publisher('/velodyne_distance', Float32MultiArray, queue_size=10)  # 수정: Float32 메시지 사용
        self.robot_cmd_pub = rospy.Publisher('/6wheel_skid_ctrl_cmd', SkidSteer6wUGVCtrlCmd, queue_size=10)
        self.collision_sub = rospy.Subscriber('/CollisionData', CollisionData, callback=self.collision_detect)

        # self.num_points_to_keep = 10  # 최종적으로 남길 포인트 수
        self.angle_interval = 18  # 18도마다 하나의 포인트 선택
        self.current_angle = -90  # 현재 각도
        self.target_height = 0.1      # 정확히 30cm 높이에 있는 포인트만 선택
        self.action_dim = action_dim
        self.heading = 0

        self.goal_list = pd.read_csv('/home/icas/woowa/src/rl/src/waypoints/coexto5.csv')
        self.goal_x = self.goal_list['x'].values.tolist()
        self.goal_y = self.goal_list['y'].values.tolist()

        self.proj_UTMK = Proj(init='epsg:5178')
        self.proj_WGS84 = Proj(init='epsg:4326')
        self.waypoint_count = 0

        self.position_x_list = []
        self.position_y_list = []
        self.waypount_count_list = []
        self.goal_x_list = []
        self.goal_y_list = []
        self.heading_list = []
        self.current_distance_list = []
        self.yaw_list = []
        self.goal_angle_list = []
        self.obstacle_distance_list = []
        self.obstacle_angle_list = []

        self.position_x = 0
        self.position_y = 0

        try:
            self.lidar_manager = LIDARConnector('UDP')
            self.gps_manager = GPSConnector('ROS')
            self.imu_manager = IMUConnector('ROS')

            self.lidar_manager.connect('127.0.0.1', 2368, '')
            self.gps_manager.connect('', 0, '/gps')
            self.imu_manager.connect('1', 1, '/imu')

            # 대기: LIDAR 데이터가 수신되고 처리될 때까지 기다립니다.
            while not self.lidar_manager.recvChk:
                pass
            print('Connect success')
            
        except Exception as e:
            print(f"Exception: {e}")

    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x[self.waypoint_count] - self.position.x, self.goal_y[self.waypoint_count] - self.position.y), 2)
        self.past_distance = goal_distance

        return goal_distance

    def getScanValue(self):
        if self.lidar_manager.connChk:
            lidar_data = self.lidar_manager.getLidar()
        else:
            print("Not connected")

        x = list(lidar_data[0])
        y = list(lidar_data[1])
        z = list(lidar_data[2])
                   
        distances = []
        distance1 = []
        distance2 = []
        distance3 = []
        distance4 = []
        distance5 = []
        distance6 = []
        distance7 = []
        distance8 = []
        distance9 = []
        distance10 = []

        for i in range(0, len(x)):
            angle = math.degrees(math.atan2(y[i], x[i]))
            if 0 < angle < 180 and abs(z[i] - 0.0) < 0.03:
                if 0 <= angle < 18:
                    distance1.append(math.sqrt(x[i]**2 + y[i]**2 + z[i]**2))
                elif 18 <= angle < 36:
                    distance2.append(math.sqrt(x[i]**2 + y[i]**2 + z[i]**2))
                elif 36 <= angle < 54:
                    distance3.append(math.sqrt(x[i]**2 + y[i]**2 + z[i]**2))
                elif 54 <= angle < 72:
                    distance4.append(math.sqrt(x[i]**2 + y[i]**2 + z[i]**2))
                elif 72 <= angle < 90:
                    distance5.append(math.sqrt(x[i]**2 + y[i]**2 + z[i]**2))
                elif 90 <= angle < 108:
                    distance6.append(math.sqrt(x[i]**2 + y[i]**2 + z[i]**2))
                elif 108 <= angle < 126:
                    distance7.append(math.sqrt(x[i]**2 + y[i]**2 + z[i]**2))
                elif 126 <= angle < 144:
                    distance8.append(math.sqrt(x[i]**2 + y[i]**2 + z[i]**2))
                elif 144 <= angle < 162:
                    distance9.append(math.sqrt(x[i]**2 + y[i]**2 + z[i]**2))
                elif 162 <= angle < 180:
                    distance10.append(math.sqrt(x[i]**2 + y[i]**2 + z[i]**2))
        
        distances.append(np.mean(distance10))
        distances.append(np.mean(distance9))
        distances.append(np.mean(distance8))
        distances.append(np.mean(distance7))
        distances.append(np.mean(distance6))
        distances.append(np.mean(distance5))
        distances.append(np.mean(distance4))
        distances.append(np.mean(distance3))
        distances.append(np.mean(distance2))
        distances.append(np.mean(distance1))


        for j in range(len(distances)):
            if distances[j] == float('Inf') or distances[j] == float('inf') or distances[j] > 3.5: 
                distances[j] = 3.5
            elif np.isnan(distances[j]) or distances[j] == float('nan'):
                distances[j] = 0
            # if abs(lidar_z[i] - self.target_height) < 0.03:  # 30cm 높이 근처에 있는 포인트 선택
            #     x, y, z = lidar_x[i], lidar_y[i], lidar_z[i]
            #     distance = math.sqrt(x**2 + y**2 + z**2)  # 3D 좌표에서 Euclidean 거리 계산
            #     if distance > 3.5:
            #         lidar_distances.append(3.5)
            #     else:
            #         lidar_distances.append(distance) # 거리 정보도 함께 저장
            #     self.current_angle += self.angle_interval
            #     if self.current_angle >= 90:  # 180도까지 선택한 경우 종료
            #         break

        return distances

    def getOdometry(self):
        position_x, position_y = self.gps_manager.getPose()
        self.position_x, self.position_y = transform(self.proj_WGS84, self.proj_UTMK, position_x, position_y)

        imu_data = self.imu_manager.getIMU()
        orientation_list = [imu_data.orientation_x, imu_data.orientation_y, imu_data.orientation_z, imu_data.orientation_w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        goal_angle = math.atan2(self.goal_y[self.waypoint_count] - self.position_y, self.goal_x[self.waypoint_count] - self.position_x)

        # print(self.goal_y[self.waypoint_count] - self.position_y, self.goal_x[self.waypoint_count] - self.position_x)
        # print(goal_angle)
        heading = goal_angle - yaw

        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.goal_angle_list.append(goal_angle)
        self.yaw_list.append(yaw)
        # print("goal angle: ", goal_angle)
        # print("yaw: ", yaw)
        # print("Heading before round: ", heading)q
        self.heading = round(heading, 3)
        # print("Heading after round: ", self.heading)


    def getState(self, past_action):
        self.getOdometry()
        heading = self.heading
        scan_range = self.getScanValue()
        obstacle_min_range = round(min(scan_range), 2)
        obstacle_angle = np.argmin(scan_range)
        current_distance = round(math.hypot(self.goal_x[self.waypoint_count] - self.position_x, self.goal_y[self.waypoint_count] - self.position_y), 2)
        
        self.position_x_list.append(self.position_x)
        self.position_y_list.append(self.position_y)
        self.goal_x_list.append(self.goal_x[self.waypoint_count])
        self.goal_y_list.append(self.goal_y[self.waypoint_count])
        self.waypount_count_list.append(self.waypoint_count)
        self.current_distance_list.append(current_distance)
        self.heading_list.append(heading)
        self.obstacle_angle_list.append(obstacle_angle)
        self.obstacle_distance_list.append(obstacle_min_range)

        my_dict = {'x' : self.position_x_list, 'y' : self.position_y_list, 'goal x': self.goal_x_list, 'goal y': self.goal_y_list, 'waypoint': self.waypount_count_list, 'current distance': self.current_distance_list, 'obstacle distance': self.obstacle_distance_list, 'obstacle angle': self.obstacle_angle_list, 'heading':self.heading_list, 'yaw': self.yaw_list, 'goal angle': self.goal_angle_list}
        df = pd.DataFrame(my_dict)
        df.to_csv('/home/icas/woowa/src/rl/src/waypoints/driving_data.csv')

        # print("waypoint: ", self.waypoint_count + 1)
        # print("goal: ", self.goal_x[self.waypoint_count], self.goal_y[self.waypoint_count])
        # print("position: ", self.position_x, self.position_y)
        # print("scan range: ", scan_range)
        # print("heading: ", heading)
        # print("obstacle min range: ", obstacle_min_range)
        # print("obstacle angle: ", obstacle_angle)
        # print("current distance: ", current_distance)

        if current_distance < 0.2:
            self.waypoint_count += 1

        return scan_range + [past_action[0], past_action[1]] + [heading, current_distance, obstacle_min_range, obstacle_angle]

    def step(self, action, past_action):
        
        # linear_vel = self.normalize(action[0], 0.0, 0.22, 0.0, 2)
        # ang_vel = self.normalize(action[1], -2., 2., -0.83, 0.83)
        linear_vel = action[0]
        ang_vel = action[1]
        vel_cmd = SkidSteer6wUGVCtrlCmd()

        print("Action: ", linear_vel, ang_vel)
        vel_cmd.cmd_type = 3
        vel_cmd.Target_linear_velocity = linear_vel
        vel_cmd.Target_angular_velocity = ang_vel
        vel_cmd.Forward_input = True
        vel_cmd.Backward_input = True
        vel_cmd.Left_Turn_input = True
        vel_cmd.Right_Turn_input = True

        self.robot_cmd_pub.publish(vel_cmd)

        state = self.getState(past_action)

        return np.asarray(state)

    def reset(self):
        return self.getState([0] * self.action_dim)

    def normalize(self, value, min_value, max_value, new_min, new_max):
        # 현재 범위에서의 비율 계산
        normalized_value = (value - min_value) / (max_value - min_value)
        
        # 새 범위에서의 값 계산
        new_value = (normalized_value * (new_max - new_min)) + new_min
        
        return new_value

    def collision_detect(self, collision_data):
        if len(collision_data.collision_object):
            while True:
                vel_cmd = SkidSteer6wUGVCtrlCmd()
                vel_cmd.cmd_type = 3
                vel_cmd.Target_linear_velocity = 0
                vel_cmd.Target_angular_velocity = 0
                vel_cmd.Forward_input = True
                vel_cmd.Backward_input = True
                vel_cmd.Left_Turn_input = True
                vel_cmd.Right_Turn_input = True

                self.robot_cmd_pub.publish(vel_cmd)