#!/home/icas/anaconda3/envs/morai/bin/python3

import pandas as pd
import glob
import math
import numpy as np
import os

waypoint_distance_interval = float(input("waypoint distance interval : "))

waypoint_save_path = "/home/icas/woowa/src/dilly_control/src/waypoints/sampled/interval/{}m".format(waypoint_distance_interval)

if not os.path.exists(waypoint_save_path):
    os.makedirs(waypoint_save_path)

# waypoint_file_names = glob.glob('/home/icas/woowa/src/dilly_control/src/waypoints/sampled/interval/*.csv')
waypoint_file_names = glob.glob('/home/icas/woowa/src/dilly_control/src/waypoints/sampled/interval/test/g.csv')

for idx, file_name in enumerate(waypoint_file_names):
    waypoint_x = []
    waypoint_y = []
    waypoint_file = pd.read_csv(file_name)
    
    for i in range(0, waypoint_file['X'].shape[0]):
        if i == 0:
            waypoint_x.append(waypoint_file['X'][i])
            waypoint_y.append(waypoint_file['Y'][i])
        elif i == waypoint_file['X'].shape[0]-1:
            waypoint_x.append(waypoint_file['X'][i])
            waypoint_y.append(waypoint_file['Y'][i])
        else:
            if math.sqrt(math.pow(waypoint_file['X'][i] - waypoint_x[-1], 2) + math.pow(waypoint_file['Y'][i] - waypoint_y[-1], 2)) >= waypoint_distance_interval:
                waypoint_x.append(waypoint_file['X'][i])
                waypoint_y.append(waypoint_file['Y'][i])
            
    df = pd.DataFrame({'X':waypoint_x, 'Y':waypoint_y})
    # df.to_csv('/home/icas/woowa/src/dilly_control/src/waypoints/sampled/interval/{}m_{}'.format(waypoint_distance_interval, file_name[-5:]))
    df.to_csv('/home/icas/woowa/src/dilly_control/src/waypoints/sampled/interval/tt_{}'.format(file_name[-5:]))
