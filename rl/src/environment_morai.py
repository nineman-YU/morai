#!/home/icas/anaconda3/envs/morai/bin/python3
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler

world = False
if world:
    from respawnGoal_custom_worlds import Respawn
else:
    from respawnGoal import Respawn
import copy

target_not_movable = False

# (22.07.22 kwon) 성공/충돌 시 보상 변수로 설정
# goal reward / collision reward
goal_reward = 500
collision_reward = -550


class Env():
    def __init__(self, action_dim=2):
        self.goal_x = 0
        self.goal_y = 0
        self.heading = 0
        self.initGoal = True
        self.get_goalbox = False
        self.past_obstacle_distance = 0.
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.respawn_goal = Respawn()
        self.past_distance = 0.
        self.stopped = 0
        self.action_dim = action_dim
        
        self.past_action = [0, 0]
        
        # Keys CTRL + c will stop script
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        # you can stop turtlebot by publishing an empty Twist
        # message
        rospy.loginfo("Stopping TurtleBot")
        self.pub_cmd_vel.publish(Twist())
        rospy.sleep(1)

    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
        self.past_distance = goal_distance

        return goal_distance

    def getOdometry(self, odom):
        self.past_position = copy.deepcopy(self.position)
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)

        # print 'yaw', yaw
        # print 'gA', goal_angle

        heading = goal_angle - yaw
        # print 'heading', heading
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 3)
        # print("yaw : ", yaw)
        # print("goal_angle : ", goal_angle)

    def getState(self, scan, past_action):
        scan_range = []
        heading = self.heading
        min_range = 0.2
        done = False

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf') or scan.ranges[i] == float('inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]) or scan.ranges[i] == float('nan'):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])

        # print("scan length : ", len(scan_range))
        
        # (22.08.11 Park) 장애물과의 최소 거리 및 방향(각도를 이용한 인덱스 찾기) 
        obstacle_min_range = round(min(scan_range), 2)
        obstacle_angle = np.argmin(scan_range)
        if min_range > min(scan_range) > 0:
            done = True

        # append two past action.
        for pa in past_action:
            scan_range.append(pa)

        cord = [self.position.x, self.position.y, self.goal_x, self.goal_y]
        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
        # current_distance = self.getGoalDistace()
        if current_distance < 0.15:
            self.get_goalbox = True

        b = [heading, current_distance]
        a = scan_range + b
        # print("scan range : ", len(scan.ranges))
        # print("scan : ", len(scan_range))
        # print("heading : ", heading)
        # print("current_distance : ", current_distance)
        # print("list : ", a[10:])
        # print("sub : ", scan_range[8:])
        
        # (22.08.11  Park) 장애물과의 최소 거리 및 방향을 상태에 추가
        return scan_range + [heading, current_distance, obstacle_min_range, obstacle_angle], done, self.get_goalbox, cord

    def setReward(self, state, done):
        current_distance = state[-3]
        LDS_values = state[0:10]
        sorted_LDS_values = sorted(LDS_values)
        min_LDS = sorted_LDS_values[0]
        max_LDS = sorted_LDS_values[-1]
        
        # if self.past_obstacle_distance > 0:
        #    if self.past_obstacle_distance - min_LDS > 0:
        #        obstacle_penalty = -550 * np.exp(-70*(min_LDS-0.2))
        #    else:
        #        obstacle_penalty = 550 * np.exp(-70*(min_LDS-0.2))
        # else:
        #    obstacle_penalty = 0
            
        # heading = state[-2]
        # print('cur:', current_distance, self.past_distance)

        distance_rate = (self.past_distance - current_distance)
        if distance_rate > 0:
            reward = 200. * distance_rate
            # reward = 
        if distance_rate <= 0:
            reward = -8
            # reward = 0.

        reward += obstacle_penalty

        # angle_reward = math.pi - abs(heading)
        # print('d', 500*distance_rate)
        # reward = 500.*distance_rate #+ 3.*angle_reward
        self.past_distance = current_distance
        self.past_obstacle_distance = min_LDS                                             

        a, b, c, d = float('{0:.3f}'.format(self.position.x)), float('{0:.3f}'.format(self.past_position.x)), float('{0:.3f}'.format(self.position.y)), float('{0:.3f}'.format(self.past_position.y))
        if a == b and c == d:
            # rospy.loginfo('\n<<<<<Stopped>>>>>\n')
            # print('\n' + str(a) + ' ' + str(b) + ' ' + str(c) + ' ' + str(d) + '\n')
            self.stopped += 1
            if self.stopped == 20:
                rospy.loginfo('Robot is in the same 10 times in a row')
                self.stopped = 0
                done = True
        else:
            # rospy.loginfo('\n>>>>> not stopped>>>>>\n')
            self.stopped = 0

        new_reward = -1  # 기본 행동 1회 할 때마다 -1

        if done:
            rospy.loginfo("Collision!!")
            reward = collision_reward
            new_reward = collision_reward
            # reward = -10.
            self.pub_cmd_vel.publish(Twist())

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward = goal_reward
            new_reward = goal_reward
            # reward = 100.
            self.pub_cmd_vel.publish(Twist())
            if world:
                self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True, running=True)
                if target_not_movable:
                    self.reset()
            else:
                self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
            self.goal_distance = self.getGoalDistace()
            self.get_goalbox = False

        return reward, new_reward, done

    def step(self, action, past_action):
        linear_vel = action[0]
        ang_vel = action[1]

        vel_cmd = Twist()
        vel_cmd.linear.x = linear_vel
        vel_cmd.angular.z = ang_vel
        self.pub_cmd_vel.publish(vel_cmd)
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        state, done, goalbox, cord = self.getState(data, past_action)
        reward, new_reward, done = self.setReward(state, done)

        # (22.07.21. park) original reward, new reward 모두 Return 하도록 코드 수정
        return np.asarray(state), reward, new_reward, done, goalbox, cord

    def reset(self):
        # print('aqui2_____________---')
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False
        else:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)

        self.goal_distance = self.getGoalDistace()
        state, _, _, cord = self.getState(data, [0] * self.action_dim)

        return np.asarray(state), cord

