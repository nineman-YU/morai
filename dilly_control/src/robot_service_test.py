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


rospy.init_node("Item_Check_node", anonymous=True)
#rospy.loginfo("==============ITEM CHECK==============")

def Item_check(data):
    global Item_list
    rospy.loginfo('Get Item : {}'.format(data.deliveryItem))
    Item_list = data.deliveryItem
    return Item_list

rospy.wait_for_service("/WoowaDillyEventCmd")
service_client = rospy.ServiceProxy("/WoowaDillyEventCmd", WoowaDillyEventCmdSrv)
#rospy.loginfo("==============ITEM CHECK2==============")

request_srv = DillyCmd()
response_srv = DillyCmdResponse()
# 적재함 확인 Network
Status_Subscriber = rospy.Subscriber('/WoowaDillyStatus',WoowaDillyStatus, Item_check) 
#rospy.loginfo("==============ITEM CHECK3==============")

morai_env = Env()

while True:
    try:
        pos = morai_env.get_current_pos()
        
        # =========== Set ROS msg value =========== #
        # 1번위치에 도달하면 물품 수령!
        if pos == get_1:
            request_srv.isPickup = True
            request_srv.deliveryItemIndex = 1
        # 2번위치에 도달하면 물품 수령!
        elif pos == get_2:
            request_srv.isPickup = True
            request_srv.deliveryItemIndex = 2
        # 3번위치에 도달하면 물품 수령!
        elif pos == get_3:
            request_srv.isPickup = True
            request_srv.deliveryItemIndex = 3
        # 4번위치에 도달하면 물품 수령!
        elif pos == get_4:
            request_srv.isPickup = True
            request_srv.deliveryItemIndex = 4
        # 5번위치에 도달하면 물품 수령!
        elif pos == get_5:
            request_srv.isPickup = True
            request_srv.deliveryItemIndex = 5
        
        # 1번위치에 도달하면 물품 전달!
        elif pos == send_1:
            request_srv.isPickup = False
            request_srv.deliveryItemIndex = 1
        # 2번위치에 도달하면 물품 전달!
        elif pos == send_2:
            request_srv.isPickup = False
            request_srv.deliveryItemIndex = 2
        # 3번위치에 도달하면 물품 전달!
        elif pos == send_3:
            request_srv.isPickup = False
            request_srv.deliveryItemIndex = 3       
        # 4번위치에 도달하면 물품 전달!
        elif pos == send_4:
            request_srv.isPickup = False
            request_srv.deliveryItemIndex = 4
        # 5번위치에 도달하면 물품 전달!
        elif pos == send_5:
            request_srv.isPickup = False
            request_srv.deliveryItemIndex = 5

        service_client(request_srv)
    except Exception as e:
        print(f"Exception: {e}")
