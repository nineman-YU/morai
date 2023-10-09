#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class SLAMController:
    def __init__(self):
        self.num_points_to_keep = 10  # 최종적으로 남길 포인트 수
        self.angle_interval = 18  # 18도마다 하나의 포인트 선택
        self.current_angle = 0  # 현재 각도
        self.target_height = 0.3      # 정확히 30cm 높이에 있는 포인트만 선택

        rospy.init_node('slam_controller', anonymous=True)
        self.robot_cmd_pub = rospy.Publisher('/6wheel_skid_ctrl_cmd', SkidSteer6wUGVCtrlCmd, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.mapping_complete = False

        try:
            self.lidar_manager = LIDARConnector('UDP')
            self.imu_manager = IMUConnector('ROS')

            self.lidar_manager.connect('127.0.0.1', 2368, '')
            self.imu_manager.connect('1', 1, '/imu')

            while not self.lidar_manager.recvChk:
                pass
            print('Connect success')
            
        except Exception as e:
            print(f"Exception: {e}")


    def laser_callback(self, data):
        lidar_distances = []
        if self.lidar_manager.connChk:
            lidar_data = self.lidar_manager.getLidar()
        else:
            print("Not connected")

        lidar_x = list(lidar_data[0])
        lidar_y = list(lidar_data[1])
        lidar_z = list(lidar_data[2])

        for i in range(len(lidar_x)):
            if abs(lidar_z[i] - target_height) < 0.01:  # 30cm 높이 근처에 있는 포인트 선택
                x, y, z = lidar_x[i], lidar_y[i], lidar_z[i]
                distance = math.sqrt(x**2 + y**2 + z**2)  # 3D 좌표에서 Euclidean 거리 계산
                self.lidar_distances.append(distance) # 거리 정보도 함께 저장
                current_angle += angle_interval
                if current_angle >= 180:  # 180도까지 선택한 경우 종료
                    break

        return lidar_distances

        # 여기에서 라이다 데이터 처리 및 매핑 진행
        if not self.mapping_complete:
            # GMapping 등의 SLAM 알고리즘을 사용하여 매핑 작업 진행
            # 매핑이 완료되면 self.mapping_complete를 True로 설정
            # 매핑이 완료되면 네비게이션을 시작하기 위한 목표 위치 설정 및 주행 명령 전송

    def odom_callback(self, data):
        position_x, position_y = self.gps_manager.getPose()
        imu_data = self.imu_manager.getIMU()
        orientation_list = [imu_data.orientation_x, imu_data.orientation_y, imu_data.orientation_z, imu_data.orientation_w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        goal_angle = math.atan2(goal_y - position_y, goal_x - position_x)

        heading = goal_angle - yaw

        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 3)
        if self.mapping_complete:
            # 매핑이 완료된 후 SLAM 주행 진행
            # 예를 들어, 목표 위치로 로봇을 이동시키는 제어 로직 추가


class NavigationController:
    def __init__(self):
        rospy.init_node('move_to_goal_node', anonymous=True)
        self.robot_cmd_pub = rospy.Publisher('/6wheel_skid_ctrl_cmd', SkidSteer6wUGVCtrlCmd, queue_size=10)

    def move_to_goal(self, x, y):
        # MoveBaseAction 서버에 연결
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        # 이동 목표 설정
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # 목표 위치 설정
        goal.target_pose.pose = Pose(Point(x, y, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))

        # 로봇 이동 명령 전송
        client.send_goal(goal)

        # 목표 지점에 도달할 때까지 대기
        client.wait_for_result()

if __name__ == '__main__':
    try:
        slam_controller = SLAMController()
        navigation_controller = NavigationController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
