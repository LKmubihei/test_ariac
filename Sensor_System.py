#!/usr/bin/env python
#-*-coding:utf-8-*-

#agvn_ksn_tray_parts的列表需要在机器人控制部分更新
#logical_camera_conveyor_parts的列表需要在机器人控制部分更新
#agv_new_part_flag需要在机器人控制部分更新


import rospy
import tf2_ros
from tf.transformations import *
import time
from geometry_msgs.msg import TransformStamped, Pose
from nist_gear.msg import Order, Model, LogicalCameraImage, VacuumGripperState,Proximity
from std_msgs.msg import String

from std_srvs.srv import Trigger

import sys
import copy
import yaml
import re
from math import pi, sqrt
from public_data import *
import threading
import thread 

conveyor_vel = 0.2  # conveyor velocity
conveyor_height = 0.875 
bin_position = {
    'bin1': [-1.898993, 3.379920, 0],
    'bin2': [-1.898993, 2.565006, 0],
    'bin3': [-2.651690, 2.565006, 0],
    'bin4': [-2.651690, 3.379920, 0],
    'bin5': [-1.898993, -3.379920, 0],
    'bin6': [-1.898993, -2.565006, 0],
    'bin7': [-2.651690, -2.565006, 0],
    'bin8': [-2.651690, -3.379920, 0], 
}

## Bin_1 boundary
bin_1_x_max = bin_position["bin1"][0] + 0.3
bin_1_x_min = bin_position["bin1"][0] - 0.3
bin_1_y_max = bin_position["bin1"][1] + 0.3
bin_1_y_min = bin_position["bin1"][1] - 0.3
## Bin_4 boundary
bin_4_x_max = bin_position["bin4"][0] + 0.3
bin_4_x_min = bin_position["bin4"][0] - 0.3
bin_4_y_max = bin_position["bin4"][1] + 0.3
bin_4_y_min = bin_position["bin4"][1] - 0.3
                ## Bin_1 boundary
bin_2_x_max = bin_position["bin2"][0] + 0.3
bin_2_x_min = bin_position["bin2"][0] - 0.3
bin_2_y_max = bin_position["bin2"][1] + 0.3
bin_2_y_min = bin_position["bin2"][1] - 0.3
## Bin_4 boundary
bin_3_x_max = bin_position["bin3"][0] + 0.3
bin_3_x_min = bin_position["bin3"][0] - 0.3
bin_3_y_max = bin_position["bin3"][1] + 0.3
bin_3_y_min = bin_position["bin3"][1] - 0.3
                ## Bin_1 boundary
bin_6_x_max = bin_position["bin6"][0] + 0.3
bin_6_x_min = bin_position["bin6"][0] - 0.3
bin_6_y_max = bin_position["bin6"][1] + 0.3
bin_6_y_min = bin_position["bin6"][1] - 0.3
## Bin_4 boundary
bin_7_x_max = bin_position["bin7"][0] + 0.3
bin_7_x_min = bin_position["bin7"][0] - 0.3
bin_7_y_max = bin_position["bin7"][1] + 0.3
bin_7_y_min = bin_position["bin7"][1] - 0.3
                ## Bin_1 boundary
bin_5_x_max = bin_position["bin5"][0] + 0.3
bin_5_x_min = bin_position["bin5"][0] - 0.3
bin_5_y_max = bin_position["bin5"][1] + 0.3
bin_5_y_min = bin_position["bin5"][1] - 0.3
## Bin_4 boundary
bin_8_x_max = bin_position["bin8"][0] + 0.3
bin_8_x_min = bin_position["bin8"][0] - 0.3
bin_8_y_max = bin_position["bin8"][1] + 0.3
bin_8_y_min = bin_position["bin8"][1] - 0.3



agv_boundary = {
    'agv1_ks1_tray': [-2.45, -1.77, 4.44,  4.94],
    'agv2_ks2_tray': [-2.45, -1.77, 1.12,  1.60],
    'agv3_ks3_tray': [-2.45, -1.77, -1.58, -1.10],
    'agv4_ks4_tray': [-2.45, -1.77, -4.91, -4.43],

    'agv1_as1_tray': [-5.785, -5.105, 4.44,  4.94],
    'agv2_as1_tray': [-5.785, -5.105, 1.12,  1.60],
    'agv3_as3_tray': [-5.785, -5.105, -1.58, -1.10],
    'agv4_as3_tray': [-5.785, -5.105, -4.91, -4.43],

    'agv1_as2_tray': [-10.775, -10.095, 4.44,  4.94],
    'agv2_as2_tray': [-10.775, -10.095, 1.12,  1.60],
    'agv3_as4_tray': [-10.775, -10.095, -1.58, -1.10],
    'agv4_as4_tray': [-10.775, -10.095, -4.91, -4.43],
}




part_on_conveyor_z = {
    'assembly_regulator_red': 0.907002,
    'assembly_regulator_blue': 0.907002,
    'assembly_regulator_green': 0.907002,  

    'assembly_sensor_red': 0.907000,
    'assembly_sensor_blue': 0.907000,
    'assembly_sensor_green': 0.907000,  

    'assembly_pump_red': 0.93000,
    'assembly_pump_blue':0.9300,
    'assembly_pump_green':0.9300,

    'assembly_battery_red':0.901642,
    'assembly_battery_blue':0.901642,
    'assembly_battery_green':0.901642,

}




assembly_sensor_heigh = {
    
    'assembly_sensor_red': [0.032, 0.041],
    'assembly_sensor_blue': [0.032, 0.041],
    'assembly_sensor_green': [0.032, 0.041]

}
part_heigh= {
    'assembly_sensor_red': [0.032, 0.064],
    'assembly_sensor_blue': [0.032, 0.064],
    'assembly_sensor_green': [0.032, 0.064],
    'assembly_regulator_red': [0.032,0.064],
    'assembly_regulator_blue': [0.032,0.064],
    'assembly_regulator_green': [0.032,0.064],
    'assembly_pump_red': [0.056, 0.112],
    'assembly_pump_blue':[0.056, 0.112],
    'assembly_pump_green':[0.056, 0.112],
    'assembly_battery_red':[0.026, 0.053],
    'assembly_battery_blue':[0.026, 0.053],
    'assembly_battery_green':[0.026, 0.053]
}

part_heigh_R_90 = {
    'assembly_sensor_red': [0.041, 0.1032],
    'assembly_sensor_blue': [0.041, 0.1032],
    'assembly_sensor_green': [0.041, 0.1032]
}

# part_heigh= {
#     'assembly_sensor_red': [0.032, 0.064],
#     'assembly_sensor_blue': [0.032, 0.064],
#     'assembly_sensor_green': [0.032, 0.064],
#     'assembly_regulator_red': [0.032,0.064],
#     'assembly_regulator_blue': [0.032,0.064],
#     'assembly_regulator_green': [0.032,0.064],
#     'assembly_pump_red': [0.056, 0.112],
#     'assembly_pump_blue':[0.056, 0.112],
#     'assembly_pump_green':[0.056, 0.112],
#     'assembly_battery_red':[0.026, 0.053],
#     'assembly_battery_blue':[0.026, 0.053],
#     'assembly_battery_green':[0.026, 0.053]
# }
bin_heigh = 0.725
delta_error = 0.05
tray_on_agv_height = 0.78

###############ZT
logic_camera_error = +0.05
logic_camera_height_error = 0.035

bins_size = 0.6

bins_ks_boundary = {
        'bin1_x': [bin_position["bin1"][0] - bins_size/2 - logic_camera_error,\
                   bin_position["bin1"][0] + bins_size/2 + logic_camera_error],
        'bin1_y': [bin_position["bin1"][1] - bins_size/2 - logic_camera_error,\
                   bin_position["bin1"][1] + bins_size/2 + logic_camera_error],
        'bin2_x': [bin_position["bin2"][0] - bins_size/2 - logic_camera_error,\
                   bin_position["bin2"][0] + bins_size/2 + logic_camera_error],
        'bin2_y': [bin_position["bin2"][1] - bins_size/2 - logic_camera_error,\
                   bin_position["bin2"][1] + bins_size/2 + logic_camera_error],
        'bin3_x': [bin_position["bin3"][0] - bins_size/2 - logic_camera_error,\
                   bin_position["bin3"][0] + bins_size/2 + logic_camera_error],
        'bin3_y': [bin_position["bin3"][1] - bins_size/2 - logic_camera_error,\
                   bin_position["bin3"][1] + bins_size/2 + logic_camera_error],
        'bin4_x': [bin_position["bin4"][0] - bins_size/2 - logic_camera_error,\
                   bin_position["bin4"][0] + bins_size/2 + logic_camera_error],
        'bin4_y': [bin_position["bin4"][1] - bins_size/2 - logic_camera_error,\
                   bin_position["bin4"][1] + bins_size/2 + logic_camera_error],
        'bin5_x': [bin_position["bin5"][0] - bins_size/2 - logic_camera_error,\
                   bin_position["bin5"][0] + bins_size/2 + logic_camera_error],
        'bin5_y': [bin_position["bin5"][1] - bins_size/2 - logic_camera_error,\
                   bin_position["bin5"][1] + bins_size/2 + logic_camera_error],
        'bin6_x': [bin_position["bin6"][0] - bins_size/2 - logic_camera_error,\
                   bin_position["bin6"][0] + bins_size/2 + logic_camera_error],
        'bin6_y': [bin_position["bin6"][1] - bins_size/2 - logic_camera_error,\
                   bin_position["bin6"][1] + bins_size/2 + logic_camera_error],
        'bin7_x': [bin_position["bin7"][0] - bins_size/2 - logic_camera_error,\
                   bin_position["bin7"][0] + bins_size/2 + logic_camera_error],
        'bin7_y': [bin_position["bin7"][1] - bins_size/2 - logic_camera_error,\
                   bin_position["bin7"][1] + bins_size/2 + logic_camera_error],
        'bin8_x': [bin_position["bin8"][0] - bins_size/2 - logic_camera_error,\
                   bin_position["bin8"][0] + bins_size/2 + logic_camera_error],
        'bin8_y': [bin_position["bin8"][1] - bins_size/2 - logic_camera_error,\
                   bin_position["bin8"][1] + bins_size/2 + logic_camera_error],
               }

# bins_product_height = {
#         'regulator': 0.757002 + logic_camera_height_error,
#         'pump':  0.781468 + logic_camera_height_error,
#         'battery': 0.751651 + logic_camera_height_error,
#         'sensor': 0.756965 + logic_camera_height_error,
#         'sensor_90': 0.765998 + logic_camera_height_error,
#         }

bins_product_height = {
        'regulator': 0.757002 + logic_camera_height_error,
        'pump':  0.779616 + logic_camera_height_error,
        'battery': 0.751490 + logic_camera_height_error,
        'sensor': 0.757001 + logic_camera_height_error,
        }

agv_tray_size = {
    'length_y': 0.5,
    'width_x': 0.7,
    }

# agv_product_height = {
#         'regulator': 0.786783 + logic_camera_height_error,
#         'pump':  0.811427 + logic_camera_height_error,
#         'battery': 0.781652 + logic_camera_height_error,
#         'sensor': 0.786998 + logic_camera_height_error,
#         'sensor_90': 0.795893 + logic_camera_height_error,
#         }

agv_product_height = {
        'regulator': 0.786991 + logic_camera_height_error,
        'pump':  0.809640 + logic_camera_height_error,
        'battery': 0.781324 + logic_camera_height_error,
        'sensor': 0.787000 + logic_camera_height_error,
        'sensor_90': 0.795893 + logic_camera_height_error,
        }


agv_product_height_with_tray = {
        'regulator': 0.796282 + logic_camera_height_error,
        'pump':  0.819327 + logic_camera_height_error,
        'battery': 0.790797 + logic_camera_height_error,
        'sensor': 0.796252 + logic_camera_height_error,
        }


agv_ks_position = {
    'agv1': [-2.115644, 4.675404, 0],
    'agv2': [-2.115644, 1.367643, 0],
    'agv3': [-2.115644, -1.333917, 0],
    'agv4': [-2.115644, -4.696062, 0],
    }

agv_ks_boundary = {
        'agv1_x': [agv_ks_position["agv1"][0] - agv_tray_size["width_x"]/2 - logic_camera_error,\
                   agv_ks_position["agv1"][0] + agv_tray_size["width_x"]/2 + logic_camera_error],
        'agv1_y': [agv_ks_position["agv1"][1] - agv_tray_size["length_y"]/2 - logic_camera_error,\
                   agv_ks_position["agv1"][1] + agv_tray_size["length_y"]/2 + logic_camera_error],
        'agv2_x': [agv_ks_position["agv2"][0] - agv_tray_size["width_x"]/2 - logic_camera_error,\
                   agv_ks_position["agv2"][0] + agv_tray_size["width_x"]/2 + logic_camera_error],
        'agv2_y': [agv_ks_position["agv2"][1] - agv_tray_size["length_y"]/2 - logic_camera_error,\
                   agv_ks_position["agv2"][1] + agv_tray_size["length_y"]/2 + logic_camera_error],
        'agv3_x': [agv_ks_position["agv3"][0] - agv_tray_size["width_x"]/2 - logic_camera_error,\
                   agv_ks_position["agv3"][0] + agv_tray_size["width_x"]/2 + logic_camera_error],
        'agv3_y': [agv_ks_position["agv3"][1] - agv_tray_size["length_y"]/2 - logic_camera_error,\
                   agv_ks_position["agv3"][1] + agv_tray_size["length_y"]/2 + logic_camera_error],
        'agv4_x': [agv_ks_position["agv4"][0] - agv_tray_size["width_x"]/2 - logic_camera_error,\
                   agv_ks_position["agv4"][0] + agv_tray_size["width_x"]/2 + logic_camera_error],
        'agv4_y': [agv_ks_position["agv4"][1] - agv_tray_size["length_y"]/2 - logic_camera_error,\
                   agv_ks_position["agv4"][1] + agv_tray_size["length_y"]/2 + logic_camera_error],
        }

####agv_as_boundary范围不正确####
agv_as_boundary = {
        'agv1_as1_x': [agv1_kitting_location['agv1_as1_tray'][0] - agv_tray_size["width_x"]/2 - logic_camera_error,\
                       agv1_kitting_location['agv1_as1_tray'][0] + agv_tray_size["width_x"]/2 + logic_camera_error],
        'agv1_as1_y': [4.44, 4.94],
        'agv1_as2_x': [agv1_kitting_location['agv1_as2_tray'][0] - agv_tray_size["width_x"]/2 - logic_camera_error,\
                       agv1_kitting_location['agv1_as2_tray'][0] + agv_tray_size["width_x"]/2 + logic_camera_error],
        'agv1_as2_y': [4.44, 4.94],
        'agv2_as1_x': [agv1_kitting_location['agv1_as1_tray'][0] - agv_tray_size["width_x"]/2 - logic_camera_error,\
                       agv1_kitting_location['agv1_as1_tray'][0] + agv_tray_size["width_x"]/2 + logic_camera_error],
        'agv2_as1_y': [1.12, 1.60],
        'agv2_as2_x': [agv1_kitting_location['agv1_as2_tray'][0] - agv_tray_size["width_x"]/2 - logic_camera_error,\
                       agv1_kitting_location['agv1_as2_tray'][0] + agv_tray_size["width_x"]/2 + logic_camera_error],
        'agv2_as2_y': [1.12, 1.60],
        'agv3_as3_x': [agv1_kitting_location['agv1_as1_tray'][0] - agv_tray_size["width_x"]/2 - logic_camera_error,\
                       agv1_kitting_location['agv1_as1_tray'][0] + agv_tray_size["width_x"]/2 + logic_camera_error],
        'agv3_as3_y': [-1.58, -1.10],
        'agv3_as4_x': [agv1_kitting_location['agv1_as2_tray'][0] - agv_tray_size["width_x"]/2 - logic_camera_error,\
                       agv1_kitting_location['agv1_as2_tray'][0] + agv_tray_size["width_x"]/2 + logic_camera_error],
        'agv3_as4_y': [-1.58, -1.10],
        'agv4_as3_x': [agv1_kitting_location['agv1_as1_tray'][0] - agv_tray_size["width_x"]/2 - logic_camera_error,\
                       agv1_kitting_location['agv1_as1_tray'][0] + agv_tray_size["width_x"]/2 + logic_camera_error],
        'agv4_as3_y': [-4.91, -4.43],
        'agv4_as4_x': [agv1_kitting_location['agv1_as2_tray'][0] - agv_tray_size["width_x"]/2 - logic_camera_error,\
                       agv1_kitting_location['agv1_as2_tray'][0] + agv_tray_size["width_x"]/2 + logic_camera_error],
        'agv4_as4_y': [-4.91, -4.43],        
        }

convey_position = [-0.573076, 0.00]

convey_size = {
    'length_y': 9 + 2,
    'width_x': 0.65,
    }

convey_boundary ={
        'convey_x': [convey_position[0] - convey_size["width_x"]/2 - logic_camera_error,\
                   convey_position[0] + convey_size["width_x"]/2 + logic_camera_error],
        'convey_y': [convey_position[1] - convey_size["length_y"]/2 - logic_camera_error,\
                   convey_position[1] + convey_size["length_y"]/2 + logic_camera_error],        
        }

convey_product_height = {
        'regulator': 0.907005 + logic_camera_height_error,
        'pump':  0.931645 + logic_camera_height_error,
        'battery': 0.901652 + logic_camera_height_error,
        'sensor': 0.907003 + logic_camera_height_error,
        'sensor_90': 0.916010 + logic_camera_height_error,
        }


class sPart:
    def __init__(self, name, location, pose):
        self.type = name
        self.location = location 
        self.pose = pose
        self.time_stamp = False
        self.u_id = -1
        self.final_check = False
    def set_time_stamp(self, time_stamp):
        self.time_stamp = time_stamp


def lower_bound_search(list_name, search_value ):
    result = -1 # if the numeber is not found
    start = 0
    end = len(list_name)-1
    while start <= end:
        middle = (start+end)//2
        if list_name[middle] ==search_value:
            result = middle
            end = middle -1
        elif list_name[middle] > search_value:
            result = middle
            end = middle -1
        else:
            start = middle+1        
    return result
        

def Is_In_Effective_Range(model_type, position, x_range, y_range, product_height):
#    if abs(abs(part_roll) % pi - pi / 2)< pi/8:
#        sensor_height = product_height['sensor_90']
#    else:
#        sensor_height = product_height['sensor']
        
    if position.x > x_range[0] and position.x < x_range[1] \
    and position.y > y_range[0] and position.y < y_range[1]:        
        if ("regulator" in model_type  and position.z < product_height['regulator']) \
        or ("pump" in model_type  and position.z < product_height['pump']) \
        or ("battery" in model_type  and position.z < product_height['battery']) \
        or ("sensor" in model_type and position.z < product_height['sensor']) : 
            return True       
    return False

#零件前后位姿误差为多少接受为同一零件，需要测试
normal_part_move_skew = {
        "x": 0.05,
        "y": 0.05,
        "z": 1,
        }

conveyor_part_move_skew = {
        "x": 0.05,
        "y": 0.05 + conveyor_vel * 0.1,
        "z": 1,
        }




def Part_Compare(part_1, part_in_list, part_move_skew=normal_part_move_skew):
    if part_1.type == part_in_list.type \
    and abs(part_1.pose.position.x - part_in_list.pose.position.x) < part_move_skew["x"] \
    and abs(part_1.pose.position.y - part_in_list.pose.position.y) < part_move_skew["y"] \
    and abs(part_1.pose.position.z - part_in_list.pose.position.z) < part_move_skew["z"] :
        return True
        pass
    return False

#def Part_Compare_Convey(part_1, part_in_list): 
#    if part_1.type == part_in_list.type and part_1.location == 'conveyor':
#        if part_1.pose.position.y > camera_1_r_boundary \
#        and part_in_list.pose.position.y < camera_0_l_boundary:
#            current_time = rospy.get_time()
#            if abs(current_time - part_in_list.time_stamp) < convey_camera_False_time:
#                return True
#                pass
#    return False  







class Sensor_System:
    def __init__(self, start_time):
        #logical_camera_0_parts
        self.has_blocked = False
        self.has_blocked_for_check = False
        self.bin1_parts = []
        self.bin4_parts = []
        self.agv1_ks1_tray_parts = []

        self.bin2_parts = []
        self.bin3_parts = []
        self.agv2_ks2_tray_parts = []
        
        #logical_camera_1_parts
        self.bin6_parts = []
        self.bin7_parts = []
        self.agv3_ks3_tray_parts = []
        
        self.bin5_parts = []
        self.bin8_parts = []
        self.agv4_ks4_tray_parts = []
        self.threadLock = threading.Lock()

        self.kit_tray_station_trays = []
        #wan
        self.new_part_flag_dict = {
            'agv1_ks1_tray':False,
            'agv2_ks2_tray':False,
            'agv3_ks3_tray':False,
            'agv4_ks4_tray':False,

            'agv1_as1_tray':False,
            'agv2_as1_tray':False,
            'agv3_as3_tray':False,
            'agv4_as3_tray':False,

            'agv1_as2_tray':False,
            'agv2_as2_tray':False,
            'agv3_as4_tray':False,
            'agv4_as4_tray':False,

            'bin1':False,
            'bin2':False,
            'bin3':False,
            'bin4':False,
            'bin5':False,
            'bin6':False,
            'bin7':False,
            'bin8':False,
            'conveyor':False,
        }
           
     
        self.new_part_dict = {
            'agv1_ks1_tray':None,
            'agv2_ks2_tray':None,
            'agv3_ks3_tray':None,
            'agv4_ks4_tray':None,

            'agv1_as1_tray':None,
            'agv2_as1_tray':None,
            'agv3_as3_tray':None,
            'agv4_as3_tray':None,

            'agv1_as2_tray':None,
            'agv2_as2_tray':None,
            'agv3_as4_tray':None,
            'agv4_as4_tray':None,
            'bin1':None,
            'bin2':None,
            'bin3':None,
            'bin4':None,
            'bin5':None,
            'bin6':None,
            'bin7':None,
            'bin8':None,
            'conveyor':None,
        }


        
        self.logical_camera_as_11_parts = []
        self.logical_camera_as_12_parts = []
        self.logical_camera_as_21_parts = []
        self.logical_camera_as_22_parts = []
        self.logical_camera_as_33_parts = []
        self.logical_camera_as_34_parts = []
        self.logical_camera_as_43_parts = []
        self.logical_camera_as_44_parts = []

        self.logical_camera_conveyor_parts = []
        self.logical_camera_update_rate = 0.1
        self.heart_beat = start_time
        self.time_stamp_buffer=[]
        self.u_id_count = 0
        self.AGV_location={
            'agv1': 'agv1_ks1_tray',
            'agv2': 'agv2_ks2_tray',
            'agv3': 'agv3_ks3_tray',
            'agv4': 'agv4_ks4_tray',
        }
        
        self.AGV_state={
            'agv1': 'init',
            'agv2': 'init',
            'agv3': 'init',
            'agv4': 'init',
        }
        
        
        self.assembly_parts = []
        self.conveyor_parts = []
        self.parts_type_dict={}
        self.parts_location_dict={}
        self.parts_on_conveyor_dict={}


        self.update_flag   = False
        self.camera_0_flag = False
        self.camera_1_flag = False
        self.camera_2_flag = False
        self.camera_3_flag = False
        self.camera_4_flag = False

        self.camera_as_11_flag = False
        self.camera_as_12_flag = False
        self.camera_as_21_flag = False
        self.camera_as_22_flag = False
        self.camera_as_33_flag = False
        self.camera_as_34_flag = False
        self.camera_as_43_flag = False
        self.camera_as_44_flag = False

        self.AGV1_location_flag = False
        self.AGV2_location_flag = False
        self.AGV3_location_flag = False
        self.AGV4_location_flag = False

        self.AGV1_state_flag = False
        self.AGV2_state_flag = False
        self.AGV3_state_flag = False
        self.AGV4_state_flag = False

        self.as2_human_flag = False
        self.as4_human_flag = False

        self.AGV1_movable_tray = None
        self.AGV2_movable_tray = None
        self.AGV3_movable_tray = None
        self.AGV4_movable_tray = None

        
        self.quality_control_sensor_1_read_flag = False
        self.quality_control_sensor_2_read_flag = False
        self.quality_control_sensor_3_read_flag = False
        self.quality_control_sensor_4_read_flag = False
        self.quality_control_sensor_1_parts = []
        self.quality_control_sensor_2_parts = []
        self.quality_control_sensor_3_parts = []
        self.quality_control_sensor_4_parts = []

        self.quality_control_sensor_1_flag = False
        self.quality_control_sensor_2_flag = False
        self.quality_control_sensor_3_flag = False
        self.quality_control_sensor_4_flag = False
        self.quality_control_sensor_dict = {
            'agv1_ks1_tray': False,
            'agv2_ks2_tray': False,
            'agv3_ks3_tray': False,
            'agv4_ks4_tray': False,
        }


        self.logical_camera_0_sub_ = rospy.Subscriber("/ariac/logical_camera_0", LogicalCameraImage,self.logical_camera_0_callback)
        self.logical_camera_1_sub_ = rospy.Subscriber("/ariac/logical_camera_1", LogicalCameraImage,self.logical_camera_1_callback)
        self.logical_camera_2_sub_ = rospy.Subscriber("/ariac/logical_camera_2", LogicalCameraImage,self.logical_camera_2_callback)
        self.logical_camera_3_sub_ = rospy.Subscriber("/ariac/logical_camera_3", LogicalCameraImage,self.logical_camera_3_callback)

        self.logical_camera_4_sub_ = rospy.Subscriber("/ariac/logical_camera_4", LogicalCameraImage,self.logical_camera_4_callback)

        self.logical_camera_as_11_sub_ = rospy.Subscriber("/ariac/logical_camera_station1_1", LogicalCameraImage,self.logical_camera_as_11_callback)
        self.logical_camera_as_12_sub_ = rospy.Subscriber("/ariac/logical_camera_station2_1", LogicalCameraImage,self.logical_camera_as_12_callback)

        self.logical_camera_as_21_sub_ = rospy.Subscriber("/ariac/logical_camera_station1_2", LogicalCameraImage,self.logical_camera_as_21_callback)
        self.logical_camera_as_22_sub_ = rospy.Subscriber("/ariac/logical_camera_station2_2", LogicalCameraImage,self.logical_camera_as_22_callback)

        self.logical_camera_as_33_sub_ = rospy.Subscriber("/ariac/logical_camera_station3_1", LogicalCameraImage,self.logical_camera_as_33_callback)
        self.logical_camera_as_34_sub_ = rospy.Subscriber("/ariac/logical_camera_station4_1", LogicalCameraImage,self.logical_camera_as_34_callback)

        self.logical_camera_as_43_sub_ = rospy.Subscriber("/ariac/logical_camera_station3_2", LogicalCameraImage,self.logical_camera_as_43_callback)
        self.logical_camera_as_44_sub_ = rospy.Subscriber("/ariac/logical_camera_station4_2", LogicalCameraImage,self.logical_camera_as_44_callback)

        self.logical_camera_conveyor_sub_ = rospy.Subscriber("/ariac/logical_camera_conveyor", LogicalCameraImage,self.logical_camera_conveyor_callback)

        #self.logical_camera_quaternion = quaternion_from_euler(1.570797, 1.570797, 1.570797)

        self.AGV1_location_sub_ = rospy.Subscriber("/ariac/agv1/station", String, self.AGV1_location_callback)
        self.AGV2_location_sub_ = rospy.Subscriber("/ariac/agv2/station", String, self.AGV2_location_callback)
        self.AGV3_location_sub_ = rospy.Subscriber("/ariac/agv3/station", String, self.AGV3_location_callback)
        self.AGV4_location_sub_ = rospy.Subscriber("/ariac/agv4/station", String, self.AGV4_location_callback)

        self.AGV1_state_sub_ = rospy.Subscriber("/ariac/agv1/state", String, self.AGV1_state_callback)
        self.AGV2_state_sub_ = rospy.Subscriber("/ariac/agv2/state", String, self.AGV2_state_callback)
        self.AGV3_state_sub_ = rospy.Subscriber("/ariac/agv3/state", String, self.AGV3_state_callback)
        self.AGV4_state_sub_ = rospy.Subscriber("/ariac/agv4/state", String, self.AGV4_state_callback)

        self.breakbeam0_sub_ = rospy.Subscriber("/ariac/breakbeam_0", Proximity, self.breakbeam0_callback)
        self.breakbeam1_sub_ = rospy.Subscriber("/ariac/breakbeam_1", Proximity, self.breakbeam1_callback)
        
        self.quality_control_sensor_1_sub_=rospy.Subscriber("/ariac/quality_control_sensor_1",LogicalCameraImage, self.quality_control_sensor_1_callback)
        self.quality_control_sensor_2_sub_=rospy.Subscriber("/ariac/quality_control_sensor_2",LogicalCameraImage, self.quality_control_sensor_2_callback)
        self.quality_control_sensor_3_sub_=rospy.Subscriber("/ariac/quality_control_sensor_3",LogicalCameraImage, self.quality_control_sensor_3_callback)
        self.quality_control_sensor_4_sub_=rospy.Subscriber("/ariac/quality_control_sensor_4",LogicalCameraImage, self.quality_control_sensor_4_callback)

    def part_position_z_limit(self, part):
        if 'bin' in part.location:
            temp_product_height = bins_product_height
        elif 'agv' in part.location:
            temp_product_height = agv_product_height
        elif 'conveyor' in part.location:
            temp_product_height = convey_product_height
        	    
        if 'regulator' in part.type:
            temp_product_height_n = temp_product_height['regulator'] - logic_camera_height_error
        elif 'pump' in part.type:
            temp_product_height_n = temp_product_height['pump'] - logic_camera_height_error
        elif 'battery' in part.type:
            temp_product_height_n = temp_product_height['battery'] - logic_camera_height_error
        elif 'sensor' in part.type:
            temp_product_height_n = temp_product_height['sensor'] - logic_camera_height_error  

        if 'tray' in part.type:
            temp_product_height_n = 1.016765
        
        if abs(part.pose.position.z - temp_product_height_n) < 0.01:
            return temp_product_height_n
        else:
            return part.pose.position.z

    def parts_lsit_update(self, part, parts_list, part_move_skew = normal_part_move_skew):
        current_time = rospy.get_time()
        part.set_time_stamp(current_time)
        part.pose.position.z = self.part_position_z_limit(part)
        in_list_flag = 0
        if parts_list:
            parts_list_len = len(parts_list)                     
            for parts_list_i in range(parts_list_len):
                if Part_Compare(part, parts_list[parts_list_i], part_move_skew):
                    if part.location == 'conveyor':
                        if abs(current_time - parts_list[parts_list_i].time_stamp) < 0.3:
                            part.final_check = parts_list[parts_list_i].final_check
                            part.u_id = parts_list[parts_list_i].u_id                           
                            parts_list[parts_list_i] = part
                            in_list_flag = 1
                            break
                    else:
                        part.u_id = parts_list[parts_list_i].u_id
                        part.final_check = parts_list[parts_list_i].final_check
                        last_msg_weight = 0.15
                        
                        part.pose.position.x = (1-last_msg_weight) * part.pose.position.x + last_msg_weight * parts_list[parts_list_i].pose.position.x
                        part.pose.position.y = (1-last_msg_weight) * part.pose.position.y + last_msg_weight * parts_list[parts_list_i].pose.position.y
                        part.pose.position.z = (1-last_msg_weight) * part.pose.position.z + last_msg_weight * parts_list[parts_list_i].pose.position.z
                        
                        part.pose.orientation.x = (1-last_msg_weight) * part.pose.orientation.x + last_msg_weight * parts_list[parts_list_i].pose.orientation.x
                        part.pose.orientation.y = (1-last_msg_weight) * part.pose.orientation.y + last_msg_weight * parts_list[parts_list_i].pose.orientation.y
                        part.pose.orientation.z = (1-last_msg_weight) * part.pose.orientation.z + last_msg_weight * parts_list[parts_list_i].pose.orientation.z
                        part.pose.orientation.w = (1-last_msg_weight) * part.pose.orientation.w + last_msg_weight * parts_list[parts_list_i].pose.orientation.w
                        parts_list[parts_list_i] = part
                        in_list_flag = 1
                        break 

        if not in_list_flag:
            if part.location == 'conveyor':
                if part.pose.position.y >=4.0:
                    self.u_id_count = self.u_id_count +1
                    part.u_id = self.u_id_count
                    parts_list.append(part)
                    self.new_part_dict[part.location] = part
                    self.new_part_flag_dict[part.location] = True 
            else:
                self.u_id_count = self.u_id_count +1
                part.u_id = self.u_id_count
                parts_list.append(part)
                self.new_part_dict[part.location] = part
                self.new_part_flag_dict[part.location] = True 

    def logical_camera_0_callback(self,msg):
        self.heart_beat = rospy.get_time()
        self.camera_0_flag = True
        if self.camera_0_flag:
            for model in msg.models:
                part_type = model.type
                # frame_to_word
                part_pose = Pose()
                part_pose.position.x = msg.pose.position.x+model.pose.position.z
                part_pose.position.y = msg.pose.position.y+model.pose.position.y
                part_pose.position.z = msg.pose.position.z-model.pose.position.x

                p = quaternion_multiply( \
                    [msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]

                if ('movable_tray' in part_type) and (self.AGV_location['agv1'] == 'agv1_ks1_tray') and (part_pose.position.z < tray_on_agv_height):
                    part = sPart(part_type,"kit_tray_station",part_pose)
                    self.AGV1_movable_tray = part
                else:
                    # to determin the location
                    if  Is_In_Effective_Range(model.type, part_pose.position, bins_ks_boundary["bin1_x"] ,bins_ks_boundary["bin1_y"], bins_product_height):
                        part = sPart(part_type,"bin1",part_pose)
                        self.parts_lsit_update(part, self.bin1_parts)
                        continue
                    if  Is_In_Effective_Range(model.type, part_pose.position, bins_ks_boundary["bin4_x"] ,bins_ks_boundary["bin4_y"], bins_product_height):
                        part = sPart(part_type,"bin4",part_pose)
                        self.parts_lsit_update(part, self.bin4_parts)
                        continue
                    if  Is_In_Effective_Range(model.type, part_pose.position, agv_ks_boundary["agv1_x"] ,agv_ks_boundary["agv1_y"], agv_product_height) \
                    and self.AGV_state['agv1'] == 'READY_TO_DELIVER':
                        part = sPart(part_type,"agv1_ks1_tray",part_pose)
                        self.parts_lsit_update(part, self.agv1_ks1_tray_parts)
                        continue
            self.camera_0_flag =False
#            print("bin1_parts:")
#            print("total number:%d", len(self.bin1_parts))
        else:
            pass

    def logical_camera_1_callback(self,msg):
        self.heart_beat = rospy.get_time()
        self.camera_1_flag = True
        if self.camera_1_flag:
            for model in msg.models:
                part_type = model.type
                # frame_to_word
                part_pose = Pose()
                part_pose.position.x = msg.pose.position.x+model.pose.position.z
                part_pose.position.y = msg.pose.position.y+model.pose.position.y
                part_pose.position.z = msg.pose.position.z-model.pose.position.x

                p = quaternion_multiply( \
                    [msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]
                if ('movable_tray' in part_type) and (self.AGV_location['agv2'] == 'agv2_ks2_tray') and (part_pose.position.z < tray_on_agv_height):
                    part = sPart(part_type,"kit_tray_station",part_pose)
                    self.AGV2_movable_tray = part
                else:
                    # to determin the location
                    if  Is_In_Effective_Range(model.type, part_pose.position, bins_ks_boundary["bin2_x"] ,bins_ks_boundary["bin2_y"], bins_product_height):
                        part = sPart(part_type,"bin2",part_pose)
                        self.parts_lsit_update(part, self.bin2_parts)
                        continue
                    if  Is_In_Effective_Range(model.type, part_pose.position, bins_ks_boundary["bin3_x"] ,bins_ks_boundary["bin3_y"], bins_product_height):
                        part = sPart(part_type,"bin3",part_pose)
                        self.parts_lsit_update(part, self.bin3_parts)
                        continue
                    if  Is_In_Effective_Range(model.type, part_pose.position, agv_ks_boundary["agv2_x"] ,agv_ks_boundary["agv2_y"], agv_product_height) \
                    and self.AGV_state['agv2'] == 'READY_TO_DELIVER':
                        part = sPart(part_type,"agv2_ks2_tray",part_pose)
                        self.parts_lsit_update(part, self.agv2_ks2_tray_parts)
                        continue
            self.camera_1_flag =False
        else:
            pass

    def logical_camera_2_callback(self,msg):
        self.heart_beat = rospy.get_time()
        self.camera_2_flag = True
        if self.camera_2_flag:
            for model in msg.models:
                part_type = model.type
                # frame_to_word
                part_pose = Pose()
                part_pose.position.x = msg.pose.position.x+model.pose.position.z
                part_pose.position.y = msg.pose.position.y+model.pose.position.y
                part_pose.position.z = msg.pose.position.z-model.pose.position.x

                p = quaternion_multiply( \
                    [msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]

                if ('movable_tray' in part_type) and (self.AGV_location['agv3'] == 'agv3_ks3_tray') and (part_pose.position.z < tray_on_agv_height):
                    part = sPart(part_type,"kit_tray_station",part_pose)
                    self.AGV3_movable_tray = part
                else:                
                    # to determin the location
                    if  Is_In_Effective_Range(model.type, part_pose.position, bins_ks_boundary["bin6_x"] ,bins_ks_boundary["bin6_y"], bins_product_height):
                        part = sPart(part_type,"bin6",part_pose)
                        self.parts_lsit_update(part, self.bin6_parts)
                        continue
                    if  Is_In_Effective_Range(model.type, part_pose.position, bins_ks_boundary["bin7_x"] ,bins_ks_boundary["bin7_y"], bins_product_height):
                        part = sPart(part_type,"bin7",part_pose)
                        self.parts_lsit_update(part, self.bin7_parts)
                        continue
                    if  Is_In_Effective_Range(model.type, part_pose.position, agv_ks_boundary["agv3_x"] ,agv_ks_boundary["agv3_y"], agv_product_height) \
                    and self.AGV_state['agv3'] == 'READY_TO_DELIVER':
                        part = sPart(part_type,"agv3_ks3_tray",part_pose)
                        self.parts_lsit_update(part, self.agv3_ks3_tray_parts)
                        continue 
            self.camera_2_flag =False
        else:
            pass

    def logical_camera_3_callback(self,msg):
        self.heart_beat = rospy.get_time()
        self.camera_3_flag = True
        if self.camera_3_flag:
            for model in msg.models:
                part_type = model.type
                # frame_to_word
                part_pose = Pose()
                part_pose.position.x = msg.pose.position.x+model.pose.position.z
                part_pose.position.y = msg.pose.position.y+model.pose.position.y
                part_pose.position.z = msg.pose.position.z-model.pose.position.x

                p = quaternion_multiply( \
                    [msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]
                if ('movable_tray' in part_type) and (self.AGV_location['agv4'] == 'agv4_ks4_tray') and (part_pose.position.z < tray_on_agv_height):
                    part = sPart(part_type,"kit_tray_station",part_pose)
                    self.AGV4_movable_tray = part
                else:
                    # to determin the location
                    if  Is_In_Effective_Range(model.type, part_pose.position, bins_ks_boundary["bin5_x"] ,bins_ks_boundary["bin5_y"], bins_product_height):
                        part = sPart(part_type,"bin5",part_pose)
                        self.parts_lsit_update(part, self.bin5_parts)
                        continue
                    if  Is_In_Effective_Range(model.type, part_pose.position, bins_ks_boundary["bin8_x"] ,bins_ks_boundary["bin8_y"], bins_product_height):
                        part = sPart(part_type,"bin8",part_pose)
                        self.parts_lsit_update(part, self.bin8_parts)
                        continue
                    if  Is_In_Effective_Range(model.type, part_pose.position, agv_ks_boundary["agv4_x"] ,agv_ks_boundary["agv4_y"], agv_product_height) \
                    and self.AGV_state['agv4'] == 'READY_TO_DELIVER':
                        part = sPart(part_type,"agv4_ks4_tray",part_pose)
                        self.parts_lsit_update(part, self.agv4_ks4_tray_parts)
                        continue 
            self.camera_3_flag =False
        else:
            pass

    def logical_camera_4_callback(self,msg):
        self.heart_beat = rospy.get_time()
        self.camera_4_flag = True
        if self.camera_4_flag:
            for model in msg.models:
                part_type = model.type
                if 'tray' in part_type:
                    # frame_to_word
                    part_pose = Pose()
                    part_pose.position.x = msg.pose.position.x + model.pose.position.z
                    part_pose.position.y = msg.pose.position.y + model.pose.position.y
                    part_pose.position.z = msg.pose.position.z - model.pose.position.x

                    p = quaternion_multiply( \
                        [msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w],  \
                        [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                    part_pose.orientation.x = p[0]
                    part_pose.orientation.y = p[1]
                    part_pose.orientation.z = p[2]
                    part_pose.orientation.w = p[3]

                    if 'movable_tray' in part_type:
                        part = sPart(part_type,"kit_tray_station",part_pose)
                        self.parts_lsit_update(part, self.kit_tray_station_trays)

            self.camera_4_flag =False
        else:
            pass

        # print("kit_tray_station_trays:")
        # print("total number:%d", len(self.kit_tray_station_trays))
        # for trat_n in self.kit_tray_station_trays:
        #     print(trat_n.type)
        #     print(trat_n.pose)

    def logical_camera_as_11_callback(self,msg):
        self.heart_beat = rospy.get_time()
        self.camera_as_11_flag = True
        if self.camera_as_11_flag:
            for model in msg.models:
                part_type = model.type
                # frame_to_word
                part_pose = Pose()
                part_pose.position.x = msg.pose.position.x+model.pose.position.z
                part_pose.position.y = msg.pose.position.y+model.pose.position.y
                part_pose.position.z = msg.pose.position.z-model.pose.position.x

                p = quaternion_multiply( \
                    [msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]

                if ('movable_tray' in part_type) and (self.AGV_location['agv1'] == 'agv1_as1_tray') and (part_pose.position.z < tray_on_agv_height):
                    part = sPart(part_type,"kit_tray_station",part_pose)
                    self.AGV1_movable_tray = part
                else:                
                    # to determin the location
                
                    if  Is_In_Effective_Range(model.type, part_pose.position, agv_as_boundary["agv1_as1_x"], agv_as_boundary['agv1_as1_y'], agv_product_height_with_tray) \
                    and self.AGV_state['agv1'] == 'READY_TO_DELIVER':
                        part = sPart(part_type,"agv1_as1_tray",part_pose)
                        self.parts_lsit_update(part, self.logical_camera_as_11_parts)
                        continue
            self.camera_as_11_flag = False
        else:
            pass  
        
    def logical_camera_as_12_callback(self,msg):
        self.heart_beat = rospy.get_time()
        self.camera_as_12_flag = True
        if self.camera_as_12_flag:
            for model in msg.models:
                part_type = model.type
                # frame_to_word
                part_pose = Pose()
                part_pose.position.x = msg.pose.position.x+model.pose.position.z
                part_pose.position.y = msg.pose.position.y+model.pose.position.y
                part_pose.position.z = msg.pose.position.z-model.pose.position.x

                p = quaternion_multiply( \
                    [msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]

                if ('movable_tray' in part_type) and (self.AGV_location['agv1'] == 'agv1_as2_tray') and (part_pose.position.z < tray_on_agv_height):
                    part = sPart(part_type,"kit_tray_station",part_pose)
                    self.AGV1_movable_tray = part
                else:                               
                    # to determin the location
                    if  Is_In_Effective_Range(model.type, part_pose.position, agv_as_boundary["agv1_as2_x"], agv_as_boundary["agv1_as2_y"], agv_product_height) \
                    and self.AGV_state['agv1'] == 'READY_TO_DELIVER':
                        part = sPart(part_type,"agv1_as2_tray",part_pose)
                        self.parts_lsit_update(part, self.logical_camera_as_12_parts)
                        continue
            self.camera_as_12_flag =False
        else:
            pass          
        
    def logical_camera_as_21_callback(self,msg):
        self.heart_beat = rospy.get_time()
        self.camera_as_21_flag = True
        if self.camera_as_21_flag:
            for model in msg.models:
                part_type = model.type
                # frame_to_word
                part_pose = Pose()
                part_pose.position.x = msg.pose.position.x+model.pose.position.z
                part_pose.position.y = msg.pose.position.y+model.pose.position.y
                part_pose.position.z = msg.pose.position.z-model.pose.position.x

                p = quaternion_multiply( \
                    [msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]

                if ('movable_tray' in part_type) and (self.AGV_location['agv2'] == 'agv2_as1_tray') and (part_pose.position.z < tray_on_agv_height):
                    part = sPart(part_type,"kit_tray_station",part_pose)
                    self.AGV2_movable_tray = part
                else:                               
                    # to determin the location
                    if  Is_In_Effective_Range(model.type, part_pose.position, agv_as_boundary["agv2_as1_x"], agv_as_boundary["agv2_as1_y"], agv_product_height) \
                    and self.AGV_state['agv2'] == 'READY_TO_DELIVER':
                        part = sPart(part_type,"agv2_as1_tray",part_pose)
                        self.parts_lsit_update(part, self.logical_camera_as_21_parts)
                        continue
            self.camera_as_21_flag =False
        else:
            pass  
        
    def logical_camera_as_22_callback(self,msg):
        self.heart_beat = rospy.get_time()
        self.camera_as_22_flag = True
        if self.camera_as_22_flag:
            for model in msg.models:
                part_type = model.type
                # frame_to_word
                part_pose = Pose()
                part_pose.position.x = msg.pose.position.x+model.pose.position.z
                part_pose.position.y = msg.pose.position.y+model.pose.position.y
                part_pose.position.z = msg.pose.position.z-model.pose.position.x

                p = quaternion_multiply( \
                    [msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]

                if ('movable_tray' in part_type) and (self.AGV_location['agv2'] == 'agv2_as2_tray') and (part_pose.position.z < tray_on_agv_height):
                    part = sPart(part_type,"kit_tray_station",part_pose)
                    self.AGV2_movable_tray = part
                else:                               
                    # to determin the location
                    if  Is_In_Effective_Range(model.type, part_pose.position, agv_as_boundary["agv2_as2_x"], agv_as_boundary["agv2_as2_y"], agv_product_height) \
                    and self.AGV_state['agv2'] == 'READY_TO_DELIVER':
                        part = sPart(part_type,"agv2_as2_tray",part_pose)
                        self.parts_lsit_update(part, self.logical_camera_as_22_parts) 
                        continue
            self.camera_as_22_flag =False
        else:
            pass           


    def logical_camera_as_33_callback(self,msg):
        self.heart_beat = rospy.get_time()
        self.camera_as_33_flag = True
        if self.camera_as_33_flag:
            for model in msg.models:
                part_type = model.type
                # frame_to_word
                part_pose = Pose()
                part_pose.position.x = msg.pose.position.x+model.pose.position.z
                part_pose.position.y = msg.pose.position.y+model.pose.position.y
                part_pose.position.z = msg.pose.position.z-model.pose.position.x

                p = quaternion_multiply( \
                    [msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]

                if ('movable_tray' in part_type) and (self.AGV_location['agv3'] == 'agv3_as3_tray') and (part_pose.position.z < tray_on_agv_height):
                    part = sPart(part_type,"kit_tray_station",part_pose)
                    self.AGV3_movable_tray = part
                else:               
                    # to determin the location
                    if  Is_In_Effective_Range(model.type, part_pose.position, agv_as_boundary["agv3_as3_x"], agv_as_boundary["agv3_as3_y"], agv_product_height) \
                    and self.AGV_state['agv3'] == 'READY_TO_DELIVER':
                        part = sPart(part_type,"agv3_as3_tray",part_pose)
                        self.parts_lsit_update(part, self.logical_camera_as_33_parts)
                        continue
            self.camera_as_33_flag =False
        else:
            pass  

    def logical_camera_as_34_callback(self,msg):
        self.heart_beat = rospy.get_time()
        self.camera_as_34_flag = True
        if self.camera_as_34_flag:
            for model in msg.models:
                part_type = model.type
                # frame_to_word
                part_pose = Pose()
                part_pose.position.x = msg.pose.position.x+model.pose.position.z
                part_pose.position.y = msg.pose.position.y+model.pose.position.y
                part_pose.position.z = msg.pose.position.z-model.pose.position.x

                p = quaternion_multiply( \
                    [msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]

                if ('movable_tray' in part_type) and (self.AGV_location['agv3'] == 'agv3_as4_tray') and (part_pose.position.z < tray_on_agv_height):
                    part = sPart(part_type,"kit_tray_station",part_pose)
                    self.AGV3_movable_tray = part
                else:                               
                    # to determin the location
                    if  Is_In_Effective_Range(model.type, part_pose.position, agv_as_boundary["agv3_as4_x"], agv_as_boundary["agv3_as4_y"], agv_product_height) \
                    and self.AGV_state['agv3'] == 'READY_TO_DELIVER':
                        part = sPart(part_type,"agv3_as4_tray",part_pose)
                        self.parts_lsit_update(part, self.logical_camera_as_34_parts) 
                        continue
            self.camera_as_34_flag =False
        else:
            pass  

    def logical_camera_as_43_callback(self,msg):
        self.heart_beat = rospy.get_time()
        self.camera_as_43_flag = True
        if self.camera_as_43_flag:
            for model in msg.models:
                part_type = model.type
                # frame_to_word
                part_pose = Pose()
                part_pose.position.x = msg.pose.position.x+model.pose.position.z
                part_pose.position.y = msg.pose.position.y+model.pose.position.y
                part_pose.position.z = msg.pose.position.z-model.pose.position.x

                p = quaternion_multiply( \
                    [msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]

                if ('movable_tray' in part_type) and (self.AGV_location['agv4'] == 'agv4_as3_tray') and (part_pose.position.z < tray_on_agv_height):
                    part = sPart(part_type,"kit_tray_station",part_pose)
                    self.AGV4_movable_tray = part
                else:                               
                    # to determin the location
                    if  Is_In_Effective_Range(model.type, part_pose.position, agv_as_boundary["agv4_as3_x"], agv_as_boundary["agv4_as3_y"], agv_product_height) \
                    and self.AGV_state['agv4'] == 'READY_TO_DELIVER':
                        part = sPart(part_type,"agv4_as3_tray",part_pose) 
                        self.parts_lsit_update(part, self.logical_camera_as_43_parts)
                        continue
            self.camera_as_43_flag =False
        else:
            pass
        
    def logical_camera_as_44_callback(self,msg):
        self.heart_beat = rospy.get_time()
        self.camera_as_44_flag = True
        if self.camera_as_44_flag:
            for model in msg.models:
                part_type = model.type
                # frame_to_word
                part_pose = Pose()
                part_pose.position.x = msg.pose.position.x+model.pose.position.z
                part_pose.position.y = msg.pose.position.y+model.pose.position.y
                part_pose.position.z = msg.pose.position.z-model.pose.position.x

                p = quaternion_multiply( \
                    [msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]

                if ('movable_tray' in part_type) and (self.AGV_location['agv4'] == 'agv4_as4_tray') and (part_pose.position.z < tray_on_agv_height):
                    part = sPart(part_type,"kit_tray_station",part_pose)
                    self.AGV4_movable_tray = part
                else:                  
                    # to determin the location
                    if  Is_In_Effective_Range(model.type, part_pose.position, agv_as_boundary["agv4_as4_x"], agv_as_boundary["agv4_as4_y"], agv_product_height) \
                    and self.AGV_state['agv4'] == 'READY_TO_DELIVER':
                        part = sPart(part_type,"agv4_as4_tray",part_pose)
                        self.parts_lsit_update(part, self.logical_camera_as_44_parts)
                        continue
            self.camera_as_44_flag =False
        else:
            pass        
        
    def logical_camera_conveyor_callback(self,msg):
        self.heart_beat = rospy.get_time()
        for model in msg.models:
            part_type = model.type
            # frame_to_word
            part_pose = Pose()
            part_pose.position.x = msg.pose.position.x+model.pose.position.z
            part_pose.position.y = msg.pose.position.y+model.pose.position.y

            if ('movable_tray' in part_type):
                pass
            else:
                #part_pose.position.z = msg.pose.position.z-model.pose.position.x

                part_pose.position.z = part_on_conveyor_z[part_type]

                p = quaternion_multiply( \
                    [msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w],  \
                    [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                part_pose.orientation.x = p[0]
                part_pose.orientation.y = p[1]
                part_pose.orientation.z = p[2]
                part_pose.orientation.w = p[3]
                # to determin the location
                if  Is_In_Effective_Range(model.type, part_pose.position, convey_boundary["convey_x"] ,convey_boundary["convey_y"], convey_product_height):
                    part = sPart(part_type,"conveyor",part_pose)
                    self.parts_lsit_update(part, self.logical_camera_conveyor_parts, conveyor_part_move_skew)
#        print("logical_camera_conveyor_parts:")
#        print("total number:%d", len(self.logical_camera_conveyor_parts))

#        print("bin1_parts:")
#        print("total number:%d", len(self.bin1_parts))
#        print("bin2_parts:")
#        print("total number:%d", len(self.bin2_parts))  
#        print("bin3_parts:")
#        print("total number:%d", len(self.bin3_parts))  
#        print("bin4_parts:")
#        print("total number:%d", len(self.bin4_parts))
        # print("agv1_parts:")
        # print("total number:%d", len(self.agv3_ks3_tray_parts)) 
#        print("agv2_parts:")
#        print("total number:%d", len(self.agv2_ks2_tray_parts))
                            

#            print("agv1_as1_parts:")
#            print("total number:%d", len(self.logical_camera_as_11_parts)) 
#            print("agv2_as1_parts:")
#            print("total number:%d", len(self.logical_camera_as_21_parts))

    def breakbeam0_callback(self, msg):
        self.as2_human_flag = msg.object_detected

    def breakbeam1_callback(self, msg):
        self.as4_human_flag = msg.object_detected
                                                                      
    def quality_control_sensor_1_callback(self,msg):
        self.heart_beat = rospy.get_time()
        del self.quality_control_sensor_1_parts[:]
        if len(msg.models)>=1:
            self.quality_control_sensor_1_read_flag = True
            self.quality_control_sensor_1_flag = False
            if self.quality_control_sensor_1_read_flag:
                for model in msg.models:
                    # frame_to_word
                    part_pose = Pose()
                    part_pose.position.x = msg.pose.position.x-model.pose.position.z
                    part_pose.position.y = msg.pose.position.y-model.pose.position.y        
                    part_pose.position.z = msg.pose.position.z-model.pose.position.x
                    
                    p = quaternion_multiply( \
                        [msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w],  \
                        [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                    part_pose.orientation.x = p[0]
                    part_pose.orientation.y = p[1]
                    part_pose.orientation.z = p[2]
                    part_pose.orientation.w = p[3]
                    
                    part = sPart(None, None, part_pose)
                    self.quality_control_sensor_1_parts.append(part)

        else:
            self.quality_control_sensor_1_flag = True
        self.quality_control_sensor_dict['agv1_ks1_tray'] = self.quality_control_sensor_1_flag
        
    def quality_control_sensor_2_callback(self,msg):
        self.heart_beat = rospy.get_time()
        del self.quality_control_sensor_2_parts[:]
        if len(msg.models)>=1:
            self.quality_control_sensor_2_flag = False
            self.quality_control_sensor_2_read_flag = True
            if self.quality_control_sensor_2_read_flag:
                for model in msg.models:
                    # frame_to_word
                    part_pose = Pose()
                    part_pose.position.x = msg.pose.position.x-model.pose.position.z
                    part_pose.position.y = msg.pose.position.y-model.pose.position.y        
                    part_pose.position.z = msg.pose.position.z-model.pose.position.x
                    
                    p = quaternion_multiply( \
                        [msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w],  \
                        [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                    part_pose.orientation.x = p[0]
                    part_pose.orientation.y = p[1]
                    part_pose.orientation.z = p[2]
                    part_pose.orientation.w = p[3]
                    
                    part = sPart(None, None, part_pose)
                    self.quality_control_sensor_2_parts.append(part)
                    
        else:
            self.quality_control_sensor_2_flag = True
        self.quality_control_sensor_dict['agv2_ks2_tray'] = self.quality_control_sensor_2_flag       

    def quality_control_sensor_3_callback(self,msg):
        self.heart_beat = rospy.get_time()
        del self.quality_control_sensor_3_parts[:]
        if len(msg.models)>=1:
            self.quality_control_sensor_3_flag = False
            self.quality_control_sensor_3_read_flag = True
            if self.quality_control_sensor_3_read_flag:
                for model in msg.models:
                    # frame_to_word
                    part_pose = Pose()
                    part_pose.position.x = msg.pose.position.x-model.pose.position.z
                    part_pose.position.y = msg.pose.position.y-model.pose.position.y        
                    part_pose.position.z = msg.pose.position.z-model.pose.position.x
                    
                    p = quaternion_multiply( \
                        [msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w],  \
                        [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                    part_pose.orientation.x = p[0]
                    part_pose.orientation.y = p[1]
                    part_pose.orientation.z = p[2]
                    part_pose.orientation.w = p[3]
                    
                    part = sPart(None, None, part_pose)
                    self.quality_control_sensor_3_parts.append(part)
                    
        else:
            self.quality_control_sensor_3_flag = True
        self.quality_control_sensor_dict['agv3_ks3_tray'] = self.quality_control_sensor_3_flag  
        
    def quality_control_sensor_4_callback(self,msg):
        self.heart_beat = rospy.get_time()
        del self.quality_control_sensor_4_parts[:]
        if len(msg.models)>=1:
            self.quality_control_sensor_4_flag = False
            self.quality_control_sensor_4_read_flag = True
            if self.quality_control_sensor_4_read_flag:
                for model in msg.models:
                    # frame_to_word
                    part_pose = Pose()
                    part_pose.position.x = msg.pose.position.x-model.pose.position.z
                    part_pose.position.y = msg.pose.position.y-model.pose.position.y        
                    part_pose.position.z = msg.pose.position.z-model.pose.position.x
                    
                    p = quaternion_multiply( \
                        [msg.pose.orientation.x, msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w],  \
                        [model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w])
                    part_pose.orientation.x = p[0]
                    part_pose.orientation.y = p[1]
                    part_pose.orientation.z = p[2]
                    part_pose.orientation.w = p[3]
                    
                    part = sPart(None, None, part_pose)
                    self.quality_control_sensor_4_parts.append(part)
                    
        else:
            self.quality_control_sensor_4_flag = True
        self.quality_control_sensor_dict['agv4_ks4_tray'] = self.quality_control_sensor_4_flag    


    def parts_list_old_update(self, parts_list, old_time = 1):
        

        while self.has_blocked and self.is_alive():
            rospy.sleep(1)
            self.has_blocked = False

        if self.is_alive():
            current_time = rospy.get_time()
            part_old_flag = True
            while part_old_flag:
                list_len = len(parts_list)
                part_old_flag = False
                self.threadLock.acquire()
                for list_count in range(0, list_len):
                    if list_count < len(parts_list) and parts_list[list_count].time_stamp:
                        if list_count < len(parts_list) and abs(current_time - parts_list[list_count].time_stamp) > old_time:
                            del parts_list[list_count]
                            part_old_flag = True
                            break
                self.threadLock.release()
    

    def all_parts_list_old_update(self):
        self.parts_list_old_update(self.bin1_parts)
        self.parts_list_old_update(self.bin2_parts)
        self.parts_list_old_update(self.bin3_parts)
        self.parts_list_old_update(self.bin4_parts)
        self.parts_list_old_update(self.bin5_parts)
        self.parts_list_old_update(self.bin6_parts)
        self.parts_list_old_update(self.bin7_parts)
        self.parts_list_old_update(self.bin8_parts)
        self.parts_list_old_update(self.agv1_ks1_tray_parts)
        self.parts_list_old_update(self.agv2_ks2_tray_parts)
        self.parts_list_old_update(self.agv3_ks3_tray_parts)
        self.parts_list_old_update(self.agv4_ks4_tray_parts)
        self.parts_list_old_update(self.kit_tray_station_trays)  
        
        # self.parts_list_old_update(self.logical_camera_conveyor_parts)

    # sort by type   not include parts on conveyor     
    def part_type_sort(self):
        self.all_parts_list_old_update()
        # clear 
        self.parts_type_dict.clear()
        # merge all_part
        all_parts = self.bin1_parts + self.bin4_parts + self.bin2_parts + self.bin3_parts +\
                    self.bin6_parts + self.bin7_parts + self.bin5_parts + self.bin8_parts + \
                    self.logical_camera_as_11_parts + self.logical_camera_as_12_parts + \
                    self.logical_camera_as_21_parts + self.logical_camera_as_22_parts + \
                    self.logical_camera_as_33_parts + self.logical_camera_as_34_parts + \
                    self.logical_camera_as_43_parts + self.logical_camera_as_44_parts + \
                    self.agv1_ks1_tray_parts + self.agv2_ks2_tray_parts + self.agv3_ks3_tray_parts + self.agv4_ks4_tray_parts

        if len(all_parts)>=1:
            # build dict
            for part in all_parts:
                self.parts_type_dict.setdefault(part.type,[]).append(part)
        else:
            pass
        
    def part_location_sort(self):
        self.all_parts_list_old_update()
        # clear 
        self.parts_location_dict.clear() 
        # merge all_part
        all_parts = self.bin1_parts + self.bin4_parts + self.bin2_parts + self.bin3_parts +\
                    self.bin6_parts + self.bin7_parts + self.bin5_parts + self.bin8_parts + \
                    self.logical_camera_as_11_parts + self.logical_camera_as_12_parts + \
                    self.logical_camera_as_21_parts + self.logical_camera_as_22_parts + \
                    self.logical_camera_as_33_parts + self.logical_camera_as_34_parts + \
                    self.logical_camera_as_43_parts + self.logical_camera_as_44_parts + \
                    self.agv1_ks1_tray_parts + self.agv2_ks2_tray_parts + self.agv3_ks3_tray_parts + self.agv4_ks4_tray_parts
                    
        if len(all_parts)>=1:
            # build dict
            for part in all_parts:
                self.parts_location_dict.setdefault(part.location,[]).append(part)

            #print test    
            # for key,value in self.parts_location_dict.items():
            #     print(key+": ")
            #     for p in value:
            #         print(p.location)
        else:
            pass

    def update(self):

        self.camera_as_1_flag = True
        self.camera_as_2_flag = True
        self.camera_as_3_flag = True
        self.camera_as_4_flag = True

    def search_part_type(self, part_type):
        self.part_type_sort()
        if part_type in self.parts_type_dict.keys():
            return self.parts_type_dict[part_type]
        else:
            return False

    def search_tray_by_type(self, tray_type):
        trays_list = self.kit_tray_station_trays
        tray_find = None
        for tray_n in trays_list:
            if tray_n.type  == tray_type:
                if not tray_find:
                    tray_find = tray_n
                else:
                    if tray_n.pose.position.y < tray_find.pose.position.y:
                        tray_find = tray_n

        return tray_find


    
    def search_part_location(self,part_location):

        self.part_location_sort()
        if part_location in self.parts_location_dict.keys():
            return self.parts_location_dict[part_location]
        else:
            return False
    def search_part_location_type(self,part_location,part_type): 
        """
        robot_system 使用
        """  
        self.part_location_sort()
        part_type_list = []
        if part_location in self.parts_location_dict.keys():
            part_list = self.parts_location_dict[part_location]
            if part_list:
                for part in part_list:
                    if part.type == part_type:
                        part_type_list.append(part)
                return part_type_list
            else:
              return False  
        else:
            return False       
 

    def search_part_on_conveyor(self,part_type):
        # clear 
        self.parts_on_conveyor_dict.clear()
        if len(self.logical_camera_conveyor_parts)>=1:
            # print "parts on conveyor:",len(self.logical_camera_conveyor_parts)
            # if time out then delete
            current_time = rospy.get_time()
            flag = False
            for part in self.logical_camera_conveyor_parts:
                if (current_time - part.time_stamp)*conveyor_vel > 6.5:
                    self.del_part_from_parts_list(part)

            effective_parts =[]
            for part in self.logical_camera_conveyor_parts:
                if (current_time - part.time_stamp)*conveyor_vel < 6.5:
                    effective_parts.append(part)

            for part in effective_parts:
                self.parts_on_conveyor_dict.setdefault(part.type,[]).append(part)
                    
            if part_type in self.parts_on_conveyor_dict.keys():
                # print "parts_need number:",len(self.parts_on_conveyor_dict[part_type])
                return self.parts_on_conveyor_dict[part_type]
            else:
                return False
        else:
            return False
      
    def AGV1_location_callback(self, msg):
        # self.heart_beat = rospy.get_time()
        self.AGV1_location_flag = True
        if self.AGV1_location_flag:
            self.AGV_location['agv1'] = 'agv1_'+msg.data+'_tray'
            self.AGV1_location_flag = False 
            
    def AGV2_location_callback(self, msg):
        # self.heart_beat = rospy.get_time()
        self.AGV2_location_flag = True
        if self.AGV2_location_flag:
            self.AGV_location['agv2'] = 'agv2_'+msg.data+'_tray'
            self.AGV2_location_flag = False 
            
    def AGV3_location_callback(self, msg):
        # self.heart_beat = rospy.get_time()
        self.AGV3_location_flag = True
        if self.AGV3_location_flag:
            self.AGV_location['agv3'] = 'agv3_'+msg.data+'_tray'
            self.AGV3_location_flag = False 

    def AGV4_location_callback(self, msg):
        # self.heart_beat = rospy.get_time()
        self.AGV4_location_flag = True
        if self.AGV4_location_flag:
            self.AGV_location['agv4'] = 'agv4_'+msg.data+'_tray'
            self.AGV4_location_flag = False  

    def AGV1_state_callback(self, msg):
        # self.heart_beat = rospy.get_time()
        self.AGV1_state_flag = True
        if self.AGV1_state_flag:
            self.AGV_state['agv1'] = msg.data
            self.AGV1_location_flag = False 

    def AGV2_state_callback(self, msg):
        #self.heart_beat = rospy.get_time()
        self.AGV2_state_flag = True
        if self.AGV2_state_flag:
            self.AGV_state['agv2'] = msg.data
            self.AGV2_location_flag = False 
            
    def AGV3_state_callback(self, msg):
        #self.heart_beat = rospy.get_time()
        self.AGV3_state_flag = True
        if self.AGV3_state_flag:
            self.AGV_state['agv3'] = msg.data
            self.AGV3_location_flag = False             
            
    def AGV4_state_callback(self, msg):
        #self.heart_beat = rospy.get_time()
        self.AGV4_state_flag = True
        if self.AGV4_state_flag:
            self.AGV_state['agv4'] = msg.data
            self.AGV4_location_flag = False             
            
    def search_del_part_use_id(self,parts_list, part):
        list_len = len(parts_list)
        for list_count in range(list_len):
            if parts_list[list_count].u_id == part.u_id:
                del parts_list[list_count]
                break
        
    def del_part_from_parts_list(self, del_part):
        if del_part.location == 'bin1':
            self.search_del_part_use_id(self.bin1_parts, del_part)
        elif del_part.location == 'bin2':
            self.search_del_part_use_id(self.bin2_parts, del_part)
        elif del_part.location == 'bin3':
            self.search_del_part_use_id(self.bin3_parts, del_part)
        elif del_part.location == 'bin4':
            self.search_del_part_use_id(self.bin4_parts, del_part)            
        elif del_part.location == 'bin5':
            self.search_del_part_use_id(self.bin5_parts, del_part)            
        elif del_part.location == 'bin6':
            self.search_del_part_use_id(self.bin6_parts, del_part)            
        elif del_part.location == 'bin7':
            self.search_del_part_use_id(self.bin7_parts, del_part)            
        elif del_part.location == 'bin8':
            self.search_del_part_use_id(self.bin8_parts, del_part)
        elif del_part.location == 'agv1_ks1_tray':
            self.search_del_part_use_id(self.agv1_ks1_tray_parts, del_part)
        elif del_part.location == 'agv2_ks2_tray':
            self.search_del_part_use_id(self.agv2_ks2_tray_parts, del_part)                  
        elif del_part.location == 'agv3_ks3_tray':
            self.search_del_part_use_id(self.agv3_ks3_tray_parts, del_part)            
        elif del_part.location == 'agv4_ks4_tray':
            self.search_del_part_use_id(self.agv4_ks4_tray_parts, del_part) 

        elif del_part.location == 'agv1_as1_tray':
            self.search_del_part_use_id(self.logical_camera_as_11_parts, del_part)
        elif del_part.location == 'agv1_as2_tray':
            self.search_del_part_use_id(self.logical_camera_as_12_parts, del_part)
        elif del_part.location == 'agv2_as1_tray':
            self.search_del_part_use_id(self.logical_camera_as_21_parts, del_part)
        elif del_part.location == 'agv2_as2_tray':
            self.search_del_part_use_id(self.logical_camera_as_22_parts, del_part)            
        elif del_part.location == 'agv3_as3_tray':
            self.search_del_part_use_id(self.logical_camera_as_33_parts, del_part)
        elif del_part.location == 'agv3_as4_tray':
            self.search_del_part_use_id(self.logical_camera_as_34_parts, del_part)
        elif del_part.location == 'agv4_as3_tray':
            self.search_del_part_use_id(self.logical_camera_as_43_parts, del_part)
        elif del_part.location == 'agv4_as4_tray':
            self.search_del_part_use_id(self.logical_camera_as_44_parts, del_part)
        elif del_part.location == 'conveyor':
            self.search_del_part_use_id(self.logical_camera_conveyor_parts, del_part)  
        elif del_part.location == 'kit_tray_station':
            self.search_del_part_use_id(self.kit_tray_station_trays, del_part)
            
    def search_part_use_part(self, part):
        self.all_parts_list_old_update()

        parts_list = self.bin1_parts + self.bin4_parts + self.agv1_ks1_tray_parts + \
            self.bin2_parts + self.bin3_parts + self.agv2_ks2_tray_parts + \
            self.bin6_parts + self.bin7_parts + self.agv3_ks3_tray_parts + \
            self.bin5_parts + self.bin8_parts + self.agv4_ks4_tray_parts + \
            self.logical_camera_as_11_parts + self.logical_camera_as_12_parts + \
            self.logical_camera_as_21_parts + self.logical_camera_as_22_parts + \
            self.logical_camera_as_33_parts + self.logical_camera_as_34_parts + \
            self.logical_camera_as_43_parts + self.logical_camera_as_44_parts 
        list_len = len(parts_list)
        for list_count in range(list_len):
            if parts_list[list_count].u_id == part.u_id:
                return parts_list[list_count]
               

    def search_agv(self, agv_name):
        # 如果agv在运动，等待
        for agv_state in self.AGV_state:
            while self.AGV_state[agv_state] != "READY_TO_DELIVER" and not rospy.is_shutdown():
                rospy.sleep(0.5)

        return self.AGV_location[agv_name]
    def search_all_agv(self):
        # 如果agv在运动，等待
        for agv_state in self.AGV_state:
            while self.AGV_state[agv_state] != "READY_TO_DELIVER" and not rospy.is_shutdown():
                rospy.sleep(0.5)
        return self.AGV_location

    def search_faulty_part_on(self, agv_ks_name):
        return self.quality_control_sensor_dict[agv_ks_name]   

    def is_alive(self):
        current_time = rospy.get_time()
        if current_time - self.heart_beat > 0.5:
            self.has_blocked = True
            self.has_blocked_for_check = True
            return False
        else:
            return True

    def read_agv_list_by_agv_id(self, agv_id):
        self.all_parts_list_old_update()
        if agv_id == 'agv1'or agv_id=="agv1_ks1_tray":
            agv_parts_list_n = self.agv1_ks1_tray_parts
        elif agv_id == 'agv2'or agv_id=="agv2_ks2_tray":
            agv_parts_list_n = self.agv2_ks2_tray_parts
        elif agv_id == 'agv3'or agv_id=="agv3_ks3_tray":
            agv_parts_list_n = self.agv3_ks3_tray_parts
        elif agv_id == 'agv4'or agv_id=="agv4_ks4_tray":  
            agv_parts_list_n = self.agv4_ks4_tray_parts   
        return agv_parts_list_n

    def read_faulty_list_by_agv_id(self, agv_id):
        self.all_parts_list_old_update()
        if agv_id == 'agv1'or agv_id=="agv1_ks1_tray":
            agv_parts_list_n = self.quality_control_sensor_1_parts
        elif agv_id == 'agv2'or agv_id=="agv2_ks2_tray":
            agv_parts_list_n = self.quality_control_sensor_2_parts
        elif agv_id == 'agv3'or agv_id=="agv3_ks3_tray":
            agv_parts_list_n = self.quality_control_sensor_3_parts
        elif agv_id == 'agv4'or agv_id=="agv4_ks4_tray":  
            agv_parts_list_n = self.quality_control_sensor_4_parts   
        return agv_parts_list_n

    def change_part_check_part_use_id(self, parts_list, part):
        list_len = len(parts_list)
        for list_count in range(list_len):
            if parts_list[list_count].u_id == part.u_id:
                parts_list[list_count].final_check = True
                break

    def Faulty_Part_Search(self, part_1_position, part_list, part_distance = 0.03):
        for part in part_list:
            if abs(part_1_position.x - part.pose.position.x) < part_distance \
            and abs(part_1_position.y - part.pose.position.y) < part_distance \
            and abs(part_1_position.z - part.pose.position.z) < 0.1:
                return part
        return False

    def Withdraw_Part_Search(self, part_n, part_list, part_distance = 0.05):
        for part in part_list:
            # print ("part_n.type == part.type",part_n.type == part.type)
            # print("abs(part_n.pose.position.x - part.pose.position.x)",abs(part_n.pose.position.x - part.pose.position.x))
            # print("abs(part_n.pose.position.y - part.pose.position.y)",abs(part_n.pose.position.y - part.pose.position.y))
            # print("abs(part_n.pose.position.z - part.pose.position.z)",abs(part_n.pose.position.z - part.pose.position.z))
            if part_n.type == part.type \
            and abs(part_n.pose.position.x - part.pose.position.x) < part_distance \
            and abs(part_n.pose.position.y - part.pose.position.y) < part_distance \
            and abs(part_n.pose.position.z - part.pose.position.z) < 0.1:
                return part
        return False

    def Part_Final_Check(self, part_1, part_list):
        for part_num in range(0,len(part_list)):
            rpy = euler_from_quaternion([part_list[part_num].pose.orientation.x,part_list[part_num].pose.orientation.y,\
                part_list[part_num].pose.orientation.z,part_list[part_num].pose.orientation.w])

            rpy_1 = euler_from_quaternion([part_1.pose.orientation.x,part_1.pose.orientation.y,\
                part_1.pose.orientation.z,part_1.pose.orientation.w])



            if part_1.type == part_list[part_num].type and "pump" in part_1.type and abs(abs(rpy_1[0])-pi)< 0.1:#flip零件  part_1是从订单中获取的

                if abs(rpy[0])<0.1 or not(abs(rpy[0])<=0.05 or abs(abs(rpy[0])-pi)<=0.1):
                    part_list[part_num].final_check = False
                    return False
                elif xyz_check(part_1, part_list[part_num]) and rpy_check(rpy,rpy_1):

                    part_list[part_num].final_check = True
                    return True


            if part_1.type == part_list[part_num].type and xyz_check(part_1, part_list[part_num]) and rpy_check(rpy,rpy_1):
                part_list[part_num].final_check = True
                return True
        return False

    def Reset_Part_Final_Check(self, part_list):
        for part_num in range(0,len(part_list)):
            part_list[part_num].final_check = False
        return True

if __name__ == '__main__':
     
    rospy.init_node("ariac_example_node")
    start_time_second = rospy.get_time()
    
    
    sensor_system = Sensor_System(start_time_second)
#    while not rospy.is_shutdown():
#        sensor_system.search_part_on_conveyor("assembly_regulator_red")
#        rospy.sleep(1)
#    while 1:
#        sensor_system.all_parts_list_old_update()
#     time.sleep(1)

#     agv_location = sensor_system.search_agv("agv4")
#     #agv = sensor_system.search_all_agv()
#     if sensor_system.is_alive():
#         print(str(agv_location))
     

#     # time.sleep(40)
#     # agv_location = sensor_system.search_all_agv()
#     # print(str(agv_location))

#     # print("Time: 40s")
#     # parts = logical_camera_system.search_part_on_conveyor("assembly_battery_red")
#     # if not parts:
#     #     print("not found!")
#     # else:    
#     #     print("find where assembly_battery_red")
#     #     for p in parts:
#     #         print(p.type)
#     #         print("delete this part!")
#     #         logical_camera_system.remove_part_on_vonveyor(p.time_stamp)

#     # parts = logical_camera_system.search_part_on_conveyor("assembly_battery_red")
#     # if not parts:
#     #     print("not found!")
#     # else:    
#     #     print("find where assembly_battery_red")
#     #     for p in parts:
#     #         print(p.type)
#     #         print(p.location)
#     #         print(p.pose)

#     #print("second call update!")

#     #quality sensor test
#     while not rospy.is_shutdown():
#         time.sleep(5)
#         print("search the faulty parts!")
#         r1 = sensor_system.search_faulty_part_on('agv1_ks1')
#         r2 = sensor_system.search_faulty_part_on('agv2_ks2')
#         r3 = sensor_system.search_faulty_part_on('agv3_ks3')
#         r4 = sensor_system.search_faulty_part_on('agv4_ks4')
#         print("the result:",r1, r2, r3, r4)
        
        
    

    rospy.spin()










