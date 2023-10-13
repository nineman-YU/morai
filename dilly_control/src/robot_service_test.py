#!/home/icas/anaconda3/envs/morai/bin/python3

import sys, os
import pandas as pd
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import rospy
from pyproj import Proj, transform
from std_msgs.msg import  Int32
from morai_env import Env
from morai_msgs.msg import WoowaDillyStatus
#Service
from morai_msgs.srv import WoowaDillyEventCmdSrv
from morai_msgs.msg import DillyCmd
from morai_msgs.msg import DillyCmdResponse
from morai_msgs.msg import WoowaDillyStatus
from morai_env import Env
import numpy as np

rospy.init_node("Item_Check_node", anonymous=True)
#rospy.loginfo("==============ITEM CHECK==============")

def Item_check(data):
    global Item_list
    rospy.loginfo('Get Item : {}'.format(data.deliveryItem))
    Item_list = data.deliveryItem
    return Item_list
print("-dddddddddddddddddd")
rospy.wait_for_service("/WoowaDillyEventCmd")
print("aaaaaaaaaaaaaaaa")
service_client = rospy.ServiceProxy("/WoowaDillyEventCmd", WoowaDillyEventCmdSrv)
#rospy.loginfo("==============ITEM CHECK2==============")

request_srv = DillyCmd()
response_srv = DillyCmdResponse()
# 적재함 확인 Network
Status_Subscriber = rospy.Subscriber('/WoowaDillyStatus',WoowaDillyStatus, Item_check) 
#rospy.loginfo("==============ITEM CHECK3==============")

morai_env = Env()
idx = 0
mission_pos = np.array([967058.7615,1935385.077])
arrived_mission_pos = False
while True:
    try:
        pos = morai_env.get_current_pos()

        dilly_to_mission_distance = morai_env.mission_to_distance(pos, mission_pos)
        print("dd : ", dilly_to_mission_distance)
        if dilly_to_mission_distance < 0.8:
            linear_velocity = 0.0
            angular_velocity = 0.0

            arrived_mission_pos = True
        arrived_mission_pos = True
        idx = 5
        # =========== Set ROS msg value =========== #
        # 1번위치에 도달하면 물품 수령!
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

        service_client(request_srv)
    except Exception as e:
        print(f"Exception: {e}")
