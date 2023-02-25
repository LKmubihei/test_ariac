#!/usr/bin/env python
#coding:utf-8
## version v0.1
import rospy
import tf2_ros
from tf.transformations import *
import time
from geometry_msgs.msg import TransformStamped, Pose
from nist_gear.msg import Order, Model, LogicalCameraImage, VacuumGripperState,Proximity

from std_srvs.srv import Trigger
from nist_gear.srv import SubmitKittingShipment,AssemblyStationSubmitShipment
from nist_gear.srv import AgvGoFromASToAS,AgvGoFromASToKS,AgvGoFromKSToAS
from Sensor_System import *
#from Order_Processor_1 import Part, Order_Processing 
from Order_System import Part, Order_Processing
import threading
import thread 

import sys
import copy
import yaml
from math import pi, sqrt, atan2
from public_data import *

insufficient_wait_time = 50

kitting_robot_map_dic = {
    'agv1_ks1_tray':[-2.265685, 4.675404, 0],
    'bin1':[-1.898993, 3.379920, 0],
    'bin2':[-1.898993, 2.565006, 0],
    'agv2_ks2_tray':[-2.265685, 1.367643, 0],
    'can': [-2.188252, -0.014119, 0],
    'agv3_ks3_tray':[-2.265685, -1.333917, 0],
    'bin6':[-1.898993, -2.565006, 0],
    'bin5':[-1.898993, -3.379920, 0],
    'agv4_ks4_tray':[-2.265685, -4.696062, 0],
}

gantry_robot_map_dic = {
    'bin1': [-1.898993, 3.379920, 0],
    'bin2': [-1.898993, 2.565006, 0],
    'bin3': [-2.651690, 2.565006, 0],
    'bin4': [-2.651690, 3.379920, 0],
    'bin5': [-1.898993, -3.379920, 0],
    'bin6': [-1.898993, -2.565006, 0],
    'bin7': [-2.651690, -2.565006, 0],
    'bin8': [-2.651690, -3.379920, 0],

    'agv1_ks1_tray': [-2.265685,4.675404,0],
    'agv2_ks2_tray': [-2.265685,1.367643, 0],
    'agv3_ks3_tray': [-2.265685,-1.333917, 0],
    'agv4_ks4_tray': [-2.265685, -4.696062, 0],

    'agv1_as1_tray': [-5.60,    4.675404,0],
    'agv2_as1_tray': [-5.60,    1.367643, 0],
    'agv3_as3_tray': [-5.60,    -1.333917, 0],
    'agv4_as3_tray': [-5.60,    -4.696062, 0],

    'agv1_as2_tray': [-10.590274,    4.675404,0],
    'agv2_as2_tray': [-10.590274,    1.367643, 0],
    'agv3_as4_tray': [-10.590274,    -1.333917, 0],
    'agv4_as4_tray': [-10.590274,    -4.696062, 0],
}







class Command:
    def __init__(self):
        self.command_id = rospy.get_time()
        self.robot_name = None
        self.type = None
        self.pick_part = Part()
        self.pick_part_on = None
        self.target_position = None
        self.is_done = True
        self.exe_time = 50
        self.start_time  = None

    def __eq__(self,other):
        return self.command_id==other.command_id and self.robot_name==other.robot_name and self.type ==other.type and self.pick_part == other.pick_part and self.pick_part_on == other.pick_part_on \
            and self.target_position == other.target_position and self.is_done == other.is_done

def kitting_submit_shipment(agv_id, station_id, kitting_shipment_type):
    rospy.wait_for_service('/ariac/'+ agv_id +'/submit_kitting_shipment')
    result = rospy.ServiceProxy('/ariac/'+ agv_id +'/submit_kitting_shipment',SubmitKittingShipment)(station_id,kitting_shipment_type)
    return result

def assembly_submit_shipment(station_id, kitting_shipment_type):
    rospy.wait_for_service('/ariac/'+ station_id +'/submit_assembly_shipment')
    rospy.ServiceProxy('/ariac/'+ station_id +'/submit_assembly_shipment', AssemblyStationSubmitShipment)(kitting_shipment_type)    


def deliver_agv_to_station(agv_id, station_id):
    rospy.wait_for_service('/ariac/'+ agv_id +'/to_'+station_id)
    rospy.ServiceProxy('/ariac/'+ agv_id +'/to_'+station_id, Trigger)()

def move_agv(agv_id, pos_1, pos_2):
    #agv_id move from pos_1 to pos_2
    if ('ks' in pos_1) and ('as' in pos_2):
        srv_type = AgvGoFromASToKS
    elif ('as' in pos_1) and ('ks' in pos_2):
        srv_type = AgvGoFromASToKS
    elif ('as' in pos_1) and ('as' in pos_2):
        srv_type = AgvGoFromASToAS
    else:
        return False

    rospy.wait_for_service('/ariac/' + agv_id + '/' + pos_1 + '_to_' + pos_2)
    rospy.ServiceProxy('/ariac/' + agv_id + '/' + pos_1 + '_to_' + pos_2, srv_type)()  
    return True




class Task_Dispatcher:
    def __init__(self,order_system,sensor_system,robot_system):
        self.order_list = []
        self.robot_info = robot_system.robot_info
        self.agv_location = None
        self.agv_state = []
        self.parts_on_warehouse_type = {}
        self.parts_on_warehouse_location = {}
        self.parts_on_conveyor = {}
        self.cmd_list = [Command(), Command()]
        self.empty_comd_list = []
        self.parts_have_placed = []
        self.parts_location_dict = {}
        self.lacked_parts_dic = {}
        self.history_cmd = []
        self.first_flag =True
        self.do_kitting_first_time = True
        self.timer_ = rospy.Timer(rospy.Duration(450), self.timer_callback, oneshot=True)
        self.kitting_bin_part_list_gantry = []
        self.kitting_bin_part_list_kitting = []
        self.kitting_no_bin_part_list = []
        self.assembly_part_list_gantry = []
        self.withdraw_part_list = []
        self.adjust_part_list = []
        self.shipment_complete_not_check = []
        self.can_not_find_part_type = []

        self.order_system = order_system
        self.sensor_system = sensor_system
        self.robot_system = robot_system
    
   
    def read_order_list(self):
        #@6 self.order_system.output_orders()
        self.order_list = self.order_system.return_order_list()        
            
        
    def timer_callback(self, event):
        self.order_list = self.order_system.return_order_list()
        for order in self.order_list:
            if order.kitting_shipments:
                for kitting_shipment in order.kitting_shipments.kitting_shipment:
                    if not kitting_shipment.is_submit:           
                                         
                        sub_shipment_type = kitting_shipment.shipment_type
                        if "update" in sub_shipment_type:
                            sub_shipment_type = sub_shipment_type.replace("update","")
                        kitting_submit_shipment(kitting_shipment.agv_id, kitting_shipment.station_id, sub_shipment_type)

            if order.assembly_shipments:
                for assembly_shipment in order.assembly_shipments.assembly_shipment:
                    if not assembly_shipment.is_submit:
                        sub_shipment_type = assembly_shipment.shipment_type
                        if "update" in sub_shipment_type:
                            sub_shipment_type = sub_shipment_type.replace("update","")
                        assembly_submit_shipment(assembly_shipment.station_id,sub_shipment_type)


    def read_agv_location(self):
        self.agv_location = self.sensor_system.search_all_agv()

    def command_list_init(self):
        cmd_1 = Command()
        cmd_2 = Command()
        self.cmd_list.append(cmd_1)
        self.cmd_list.append(cmd_2)
        self.empty_comd_list.append(cmd_1)
        self.empty_comd_list.append(cmd_2)
        self.history_cmd.append(cmd_1)
        self.history_cmd.append(cmd_2)


    # @ used above the loop
    def get_all_part_on_warehouse(self):

        # self.sensor_system.update()
        # time.sleep(1)
        part_type = 'assembly_regulator_red'
        part_list = self.sensor_system.search_part_type(part_type)
        self.parts_on_warehouse_type[part_type] = part_list
        part_type = 'assembly_regulator_blue'
        part_list = self.sensor_system.search_part_type(part_type)
        self.parts_on_warehouse_type[part_type] = part_list     
        part_type = 'assembly_regulator_green'
        part_list = self.sensor_system.search_part_type(part_type)
        self.parts_on_warehouse_type[part_type] = part_list   

        part_type = 'assembly_pump_red'
        part_list = self.sensor_system.search_part_type(part_type)
        self.parts_on_warehouse_type[part_type] = part_list   
        part_type = 'assembly_pump_blue'
        part_list = self.sensor_system.search_part_type(part_type)
        self.parts_on_warehouse_type[part_type] = part_list   
        part_type = 'assembly_pump_green'
        part_list = self.sensor_system.search_part_type(part_type)
        self.parts_on_warehouse_type[part_type] = part_list   

        part_type = 'assembly_battery_red'
        part_list = self.sensor_system.search_part_type(part_type)
        self.parts_on_warehouse_type[part_type] = part_list   
        part_type = 'assembly_battery_blue'
        part_list = self.sensor_system.search_part_type(part_type)
        self.parts_on_warehouse_type[part_type] = part_list   
        part_type = 'assembly_battery_green'
        part_list = self.sensor_system.search_part_type(part_type)
        self.parts_on_warehouse_type[part_type] = part_list   

        part_type = 'assembly_sensor_red'
        part_list = self.sensor_system.search_part_type(part_type)
        self.parts_on_warehouse_type[part_type] = part_list   
        part_type = 'assembly_sensor_blue'
        part_list = self.sensor_system.search_part_type(part_type)
        self.parts_on_warehouse_type[part_type] = part_list   
        part_type = 'assembly_sensor_green'
        part_list = self.sensor_system.search_part_type(part_type)
        self.parts_on_warehouse_type[part_type] = part_list  

        part_location = 'bin1'
        part_list = self.sensor_system.search_part_location(part_location)
        self.parts_on_warehouse_location[part_location] = part_list
        part_location = 'bin2'
        part_list = self.sensor_system.search_part_location(part_location)
        self.parts_on_warehouse_location[part_location] = part_list     
        part_location = 'bin3'
        part_list = self.sensor_system.search_part_location(part_location)
        self.parts_on_warehouse_location[part_location] = part_list   

        part_location = 'bin4'
        part_list = self.sensor_system.search_part_location(part_location)
        self.parts_on_warehouse_location[part_location] = part_list   
        part_location = 'bin5'
        part_list = self.sensor_system.search_part_location(part_location)
        self.parts_on_warehouse_location[part_location] = part_list   
        part_location = 'bin5'
        part_list = self.sensor_system.search_part_location(part_location)
        self.parts_on_warehouse_location[part_location] = part_list   

        part_location = 'bin6'
        part_list = self.sensor_system.search_part_location(part_location)
        self.parts_on_warehouse_location[part_location] = part_list   
        part_location = 'bin7'
        part_list = self.sensor_system.search_part_location(part_location)
        self.parts_on_warehouse_location[part_location] = part_list   
        part_location = 'bin8'
        part_list = self.sensor_system.search_part_location(part_location)
        self.parts_on_warehouse_location[part_location] = part_list   

        part_location = 'agv1_ks1_tray'
        part_list = self.sensor_system.search_part_location(part_location)
        self.parts_on_warehouse_location[part_location] = part_list   
        part_location = 'agv1_as1_tray'
        part_list = self.sensor_system.search_part_location(part_location)
        self.parts_on_warehouse_location[part_location] = part_list   
        part_location = 'agv1_as2_tray'
        part_list = self.sensor_system.search_part_location(part_location)
        self.parts_on_warehouse_location[part_location] = part_list  

        part_location = 'agv1_ks1_tray'
        part_list = self.sensor_system.search_part_location(part_location)
        self.parts_on_warehouse_location[part_location] = part_list   
        part_location = 'agv1_as1_tray'
        part_list = self.sensor_system.search_part_location(part_location)
        self.parts_on_warehouse_location[part_location] = part_list   
        part_location = 'agv1_as2_tray'
        part_list = self.sensor_system.search_part_location(part_location)
        self.parts_on_warehouse_location[part_location] = part_list  



        part_location = 'agv2_ks2_tray'
        part_list = self.sensor_system.search_part_location(part_location)
        self.parts_on_warehouse_location[part_location] = part_list   
        part_location = 'agv2_as1_tray'
        part_list = self.sensor_system.search_part_location(part_location)
        self.parts_on_warehouse_location[part_location] = part_list   
        part_location = 'agv2_as2_tray'
        part_list = self.sensor_system.search_part_location(part_location)
        self.parts_on_warehouse_location[part_location] = part_list  

        part_location = 'agv3_ks3_tray'
        part_list = self.sensor_system.search_part_location(part_location)
        self.parts_on_warehouse_location[part_location] = part_list   
        part_location = 'agv3_as3_tray'
        part_list = self.sensor_system.search_part_location(part_location)
        self.parts_on_warehouse_location[part_location] = part_list   
        part_location = 'agv3_as4_tray'
        part_list = self.sensor_system.search_part_location(part_location)
        self.parts_on_warehouse_location[part_location] = part_list  

        part_location = 'agv4_ks4_tray'
        part_list = self.sensor_system.search_part_location(part_location)
        self.parts_on_warehouse_location[part_location] = part_list   
        part_location = 'agv4_as3_tray'
        part_list = self.sensor_system.search_part_location(part_location)
        self.parts_on_warehouse_location[part_location] = part_list   
        part_location = 'agv4_as4_tray'
        part_list = self.sensor_system.search_part_location(part_location)
        self.parts_on_warehouse_location[part_location] = part_list  

    def kitting_robot_opt_short_path(self, robot_position,target_region_list, agv_present_location):
        """
        计算Kitting机器人到零件的最短路径
        """
        min_path = 200  # 初始化最短路径为200
        location_name = None  # 初始化零件位置为None
        for part_location in target_region_list:
            dis1 = abs(robot_position.y -kitting_robot_map_dic[part_location][1])  # 计算机器人和零件之间的纵坐标差值
            dis2 = abs(kitting_robot_map_dic[part_location][1] - kitting_robot_map_dic[agv_present_location][1])  # 计算AGV和零件之间的纵坐标差值
            if min_path > dis1 + dis2:  # 如果路径更短
                min_path = dis1 + dis2  # 更新最短路径
                location_name = part_location  # 更新零件位置
        return location_name  # 返回最短路径所在的零件位置
            

    def gantry_robot_opt_short_path(self, robot_position,target_region_list, agv_present_location):
        min_path = 200000000
        location_name = None 
        for part_location in target_region_list:
            dis1 = abs(robot_position.x -gantry_robot_map_dic[part_location][0])     # 计算机器人和零件之间的横坐标差值
            dis2 = abs(robot_position.y -gantry_robot_map_dic[part_location][1])
            dis3 = abs(gantry_robot_map_dic[part_location][0] - gantry_robot_map_dic[agv_present_location][0])
            dis4 = abs(gantry_robot_map_dic[part_location][1] - gantry_robot_map_dic[agv_present_location][1])
            path = dis1+dis2+dis3+dis4
            if min_path >= path:
                min_path = path
                location_name = part_location
        return location_name


    def part_location_sort(self, parts_type_list):
        # clear 
        part_type_location = {}

        if parts_type_list and len(parts_type_list)>=1:
            # build dict
            for part in parts_type_list:
                part_type_location.setdefault(part.location,[]).append(part)
        else:
            pass

        return part_type_location
         

    #@ do not consider place on bins
    def kitting_operation(self, target_part, target_agv_id):
        
        # #@8 if target_agv_id have exist some part
        # # where to place
        agv_present_location = self.agv_location[target_agv_id] # ks or as_x
        target_part = target_pose_to_world(target_part,target_agv_id, agv_present_location)
        #@ decide where to place on
        place_part_on = agv_present_location
        #先判断是否已经有了
        part_on_expect_agv = self.sensor_system.search_part_location_type(agv_present_location,target_part.type)
        target_part_rpy = euler_from_quaternion([target_part.pose.orientation.x,target_part.pose.orientation.y,\
            target_part.pose.orientation.z,target_part.pose.orientation.w])
        if part_on_expect_agv:           #如果存在，则比较部件的位置（包括位置和姿态）是否匹配，如果匹配，则更新部件信息并返回True
            for part in part_on_expect_agv:
                part_rpy = euler_from_quaternion([part.pose.orientation.x,part.pose.orientation.y,\
                   part.pose.orientation.z,part.pose.orientation.w])
                if  xyz_check(part, target_part) and rpy_check(part_rpy,target_part_rpy):
                    self.order_system.part_update(target_part.i_d)
                    return True

        #@ decide where to pick on 
        pick_part_on = None
        part_on_kitting_robot_region = False 
         
        #@5 get robot_info from robot_system
        #self.robot_system.robot_info()
        #@@ 
        #self.robot_info = self.robot_system.robot_info()

        kitting_robot_region =[]
        gantry_robot_region = []
        part_on_conveyor_flag = False
        #@3 search part on_conveyor 
        self.parts_on_conveyor = self.sensor_system.search_part_on_conveyor(target_part.type)
        # print "self.parts_on_conveyor",target_part.type, self.parts_on_conveyor
        if self.parts_on_conveyor: # @7 if kitting to there takes too lone time, it will be failed
            if target_part.is_flip: #优先分配给gantry
                if self.robot_info['gantry_robot'].is_enabled and \
                    self.robot_info['gantry_robot'].is_idle:
                    #@ gantry_robot_opt_short_path()
                    # kitting_robot_region
                    robot_position = self.robot_info["gantry_robot"].position
                    target_part_con = self.parts_on_conveyor[-1]
                    #判断机器人能不能到达
                    current_time = rospy.get_time()
                    target_part_time = target_part_con.time_stamp
                    target_part_y = target_part_con.pose.position.y

                    c_y = -(current_time - target_part_time)*conveyor_vel+target_part_y
                    
                    if c_y <= robot_position.y:
                        t_best = (robot_position.y- c_y)/(gantry_velocity - conveyor_vel)  
                    else:
                        t_best = (c_y - robot_position.y)/(gantry_velocity + conveyor_vel) 
                    t_best = t_best+abs(robot_position.x-0.3)/gantry_velocity
                    # 最佳抓取位置    
                    f_y = c_y - t_best*conveyor_vel
                    if f_y > conveyor_end:  #gantry_robot可以到达
                        pick_part_on = "conveyor"
                        part_on_conveyor_flag = True
                        ##################
                        self.cmd_list[1].command_id = rospy.get_time()
                        self.cmd_list[1].robot_name = "gantry_robot"
                        self.cmd_list[1].type = "kitting"
                        self.cmd_list[1].pick_part = target_part
                        self.cmd_list[1].target_position = place_part_on
                        self.cmd_list[1].pick_part_on = pick_part_on
                        self.cmd_list[1].is_done = False
                        self.history_cmd[1] = self.cmd_list[1]
                        return self.cmd_list
                    else:
                        part_on_conveyor_flag= False    

                elif self.robot_info['kitting_robot'].is_enabled and \
                    self.robot_info['kitting_robot'].is_idle and 'ks' in agv_present_location:
                    #判断机器人能不能到达
                    robot_position = self.robot_info["kitting_robot"].position
                    target_part_con = self.parts_on_conveyor[-1]
                    #计算最佳抓取位置   
                    current_time = rospy.get_time()
                    target_part_time = target_part_con.time_stamp
                    target_part_y = target_part_con.pose.position.y

                    c_y = -(current_time - target_part_time)*conveyor_vel+target_part_y
                    
                    if c_y <= robot_position.y:
                        t_best = (robot_position.y- c_y)/(kitting_velocity - conveyor_vel)  
                    else:
                        t_best = (c_y - robot_position.y)/(kitting_velocity + conveyor_vel) 
                    f_y = c_y - t_best*conveyor_vel - 0.17
                    if f_y > conveyor_end:  #说明kitting可以到达
                        pick_part_on = "conveyor"
                        part_on_conveyor_flag = True
                        ##################
                        self.cmd_list[0].command_id = rospy.get_time()
                        self.cmd_list[0].robot_name = "kitting_robot"
                        self.cmd_list[0].type = "kitting"
                        self.cmd_list[0].pick_part= target_part
                        self.cmd_list[0].target_position = place_part_on
                        self.cmd_list[0].pick_part_on = pick_part_on
                        self.cmd_list[0].is_done = False
                        # update 
                        self.history_cmd[0] = self.cmd_list[0]
                        return self.cmd_list
                    else:
                        part_on_conveyor_flag= False
                else:
                    pass   
            else:
                if self.robot_info['kitting_robot'].is_enabled and \
                    self.robot_info['kitting_robot'].is_idle and 'ks' in agv_present_location:
                    #判断机器人能不能到达
                    robot_position = self.robot_info["kitting_robot"].position
                    target_part_con = self.parts_on_conveyor[-1]
                    #计算最佳抓取位置   
                    current_time = rospy.get_time()
                    target_part_time = target_part_con.time_stamp
                    target_part_y = target_part_con.pose.position.y

                    c_y = -(current_time - target_part_time)*conveyor_vel+target_part_y
                    
                    if c_y <= robot_position.y:
                        t_best = (robot_position.y- c_y)/(kitting_velocity - conveyor_vel)  
                    else:
                        t_best = (c_y - robot_position.y)/(kitting_velocity + conveyor_vel) 
                    f_y = c_y - t_best*conveyor_vel - 0.17
                    if f_y > conveyor_end:  #说明kitting可以到达
                        pick_part_on = "conveyor"
                        part_on_conveyor_flag = True
                        ##################
                        self.cmd_list[0].command_id = rospy.get_time()
                        self.cmd_list[0].robot_name = "kitting_robot"
                        self.cmd_list[0].type = "kitting"
                        self.cmd_list[0].pick_part= target_part
                        self.cmd_list[0].target_position = place_part_on
                        self.cmd_list[0].pick_part_on = pick_part_on
                        self.cmd_list[0].is_done = False
                        # update 
                        self.history_cmd[0] = self.cmd_list[0]
                        return self.cmd_list

                    else:
                        part_on_conveyor_flag= False
                elif self.robot_info['gantry_robot'].is_enabled and \
                    self.robot_info['gantry_robot'].is_idle:
                    #@ gantry_robot_opt_short_path()
                    # kitting_robot_region
                    robot_position = self.robot_info["gantry_robot"].position
                    target_part_con = self.parts_on_conveyor[-1]
                    #判断机器人能不能到达
                    current_time = rospy.get_time()
                    target_part_time = target_part_con.time_stamp
                    target_part_y = target_part_con.pose.position.y

                    c_y = -(current_time - target_part_time)*conveyor_vel+target_part_y
                    
                    if c_y <= robot_position.y:
                        t_best = (robot_position.y- c_y)/(gantry_velocity - conveyor_vel)  
                    else:
                        t_best = (c_y - robot_position.y)/(gantry_velocity + conveyor_vel) 
                    t_best = t_best+abs(robot_position.x-0.3)/gantry_velocity
                    #最佳抓取位置    
                    f_y = c_y - t_best*conveyor_vel
                    if f_y > conveyor_end:  #gantry_robot可以到达
                        pick_part_on = "conveyor"
                        part_on_conveyor_flag = True
                        ##################
                        self.cmd_list[1].command_id = rospy.get_time()
                        self.cmd_list[1].robot_name = "gantry_robot"
                        self.cmd_list[1].type = "kitting"
                        self.cmd_list[1].pick_part = target_part
                        self.cmd_list[1].target_position = place_part_on
                        self.cmd_list[1].pick_part_on = pick_part_on
                        self.cmd_list[1].is_done = False
                        self.history_cmd[1]=self.cmd_list[1]
                        return self.cmd_list
                    else:
                        part_on_conveyor_flag= False  
                else:
                    pass                 
        if part_on_conveyor_flag == False:
            self.get_all_part_on_warehouse()
            # print "self.parts_on_warehouse_type[target_part.type]",self.parts_on_warehouse_type[target_part.type
            if self.parts_on_warehouse_type[target_part.type]:
                need_part = self.parts_on_warehouse_type[target_part.type]
                part_type_location_dic = self.part_location_sort(need_part)# bin3 :[p1, p2]
                # print "part_type_location_dic:"
                #先做一个分类，先寻找bin上有没有，在看agv上有木有
                part_exist_on_bin = []
                part_exist_on_agv = []

                for key in part_type_location_dic:
                    if "bin" in key:
                        part_exist_on_bin.append(key)
                    if "agv" in key:
                        part_exist_on_agv.append(key)

                for key in part_exist_on_bin:
                    if key in kitting_robot_map_dic:
                        kitting_robot_region.append(key)
                    else:
                        gantry_robot_region.append(key)

                if len(kitting_robot_region) ==0:
                    for key in part_exist_on_agv:
                        if key in kitting_robot_map_dic:
                            kitting_robot_region.append(key)
                if len(kitting_robot_region)==0 and len(gantry_robot_region)==0:
                    for key in part_exist_on_agv:
                        if key in kitting_robot_map_dic:
                            kitting_robot_region.append(key)
                        else:
                            gantry_robot_region.append(key)
                            
                if len(kitting_robot_region)==0 and not self.robot_info['gantry_robot'].is_enabled:
                    if not (target_part.i_d in self.lacked_parts_dic):
                        target_part.time_lack = copy.deepcopy(rospy.get_time())
                        self.lacked_parts_dic[target_part.i_d] = [target_part,target_part.target_agv_id]
                        print("kitting insufficient part")
                        return self.cmd_list
                    

                if len(kitting_robot_region) >=1:
                    part_on_kitting_robot_region = True
                else:
                    part_on_kitting_robot_region = False
        if not self.parts_on_conveyor and not self.parts_on_warehouse_type[target_part.type]:
            # current warehouse do not have parts
            #@ save this part, return null cmd
            if not (target_part.i_d in self.lacked_parts_dic):
                target_part.time_lack = copy.deepcopy(rospy.get_time())
                self.lacked_parts_dic[target_part.i_d] = [target_part,target_part.target_agv_id]
                return self.cmd_list

        #@4 decide which robot to do_kitting


        # print "self.robot_info['gantry_robot'].is_idle",self.robot_info['gantry_robot'].is_idle
        # print "self.robot_info['kitting_robot'].is_idle",self.robot_info['kitting_robot'].is_idle

        if target_part.is_flip:
            
            if self.robot_info['gantry_robot'].is_enabled and \
                self.robot_info['gantry_robot'].is_idle and not part_on_conveyor_flag:
                
                if len(gantry_robot_region)>0:
                    pick_part_on = self.gantry_robot_opt_short_path(self.robot_info['gantry_robot'].position,gantry_robot_region, agv_present_location)
                else:
                    if part_on_kitting_robot_region:
                        pick_part_on = self.gantry_robot_opt_short_path(self.robot_info['gantry_robot'].position,kitting_robot_region, agv_present_location)
                    else:
                        return False
                    

                #pick_part_on = self.gantry_robot_opt_short_path(part_type_location_dic, agv_present_location)
                # print "gantry_robot_pick_part_on",pick_part_on

                self.cmd_list[1].command_id = rospy.get_time()
                self.cmd_list[1].robot_name = "gantry_robot"
                self.cmd_list[1].type = "kitting"
                self.cmd_list[1].pick_part = target_part
                self.cmd_list[1].target_position = place_part_on
                self.cmd_list[1].pick_part_on = pick_part_on
                self.cmd_list[1].is_done = False
                self.history_cmd[1] = self.cmd_list[1]
                return self.cmd_list

                        
            elif self.robot_info['kitting_robot'].is_enabled and \
                self.robot_info['kitting_robot'].is_idle and \
                'ks' in agv_present_location and part_on_kitting_robot_region and not part_on_conveyor_flag:
                pick_part_on = \
                self.kitting_robot_opt_short_path(self.robot_info['kitting_robot'].position,kitting_robot_region, agv_present_location)
                self.first_flag = False
                self.cmd_list[0].command_id = rospy.get_time()
                self.cmd_list[0].robot_name = "kitting_robot"
                self.cmd_list[0].type = "kitting"
                self.cmd_list[0].pick_part= target_part
                self.cmd_list[0].target_position = place_part_on
                self.cmd_list[0].pick_part_on = pick_part_on
                self.cmd_list[0].is_done = False
                # update 
                self.history_cmd[0] = self.cmd_list[0]
                return self.cmd_list

            else:
                pass

        else:
            if self.robot_info['kitting_robot'].is_enabled and \
                self.robot_info['kitting_robot'].is_idle and \
                'ks' in agv_present_location and part_on_kitting_robot_region and not part_on_conveyor_flag:
                pick_part_on = \
                self.kitting_robot_opt_short_path(self.robot_info['kitting_robot'].position,kitting_robot_region, agv_present_location)
                self.first_flag = False
                self.cmd_list[0].command_id = rospy.get_time()
                self.cmd_list[0].robot_name = "kitting_robot"
                self.cmd_list[0].type = "kitting"
                self.cmd_list[0].pick_part= target_part
                self.cmd_list[0].target_position = place_part_on
                self.cmd_list[0].pick_part_on = pick_part_on
                self.cmd_list[0].is_done = False
                # update 
                self.history_cmd[0] = self.cmd_list[0]
    
            elif self.robot_info['gantry_robot'].is_enabled and \
                self.robot_info['gantry_robot'].is_idle and not part_on_conveyor_flag:
                #@ gantry_robot_opt_short_path()
                # kitting_robot_region

                ####可以优化，gantry优先抓gantry——region的零件
                if len(gantry_robot_region)>0:
                    pick_part_on = self.gantry_robot_opt_short_path(self.robot_info['gantry_robot'].position,gantry_robot_region, agv_present_location)
                else:
                    if part_on_kitting_robot_region:
                        pick_part_on = self.gantry_robot_opt_short_path(self.robot_info['gantry_robot'].position,kitting_robot_region, agv_present_location)
                    else:
                        return False
                    
                #pick_part_on = self.gantry_robot_opt_short_path(part_type_location_dic, agv_present_location)
                # print "gantry_robot_pick_part_on",pick_part_on

                self.cmd_list[1].command_id = rospy.get_time()
                self.cmd_list[1].robot_name = "gantry_robot"
                self.cmd_list[1].type = "kitting"
                self.cmd_list[1].pick_part = target_part
                self.cmd_list[1].target_position = place_part_on
                self.cmd_list[1].pick_part_on = pick_part_on
                self.cmd_list[1].is_done = False
                self.history_cmd[1] = self.cmd_list[1]
            else:
                pass


    def withdraw_operation(self, target_part, target_agv_id):
        # where to pick_up
        self.read_agv_location()
        agv_present_location = self.agv_location[target_agv_id]
        pick_part_on = agv_present_location
        # target_on_world_frame #有可能不准确,两种方案，如果传感器有效，以传感器为准。
    
        self.get_all_part_on_warehouse()
        target_part_list = self.sensor_system.search_part_location_type(agv_present_location,target_part.type)
        if target_part_list:
            tmp_part = self.sensor_system.Withdraw_Part_Search(target_part, target_part_list, part_distance = 0.05)
            target_part.pose = tmp_part.pose
        else:
            target_part = target_pose_to_world(target_part,target_agv_id, agv_present_location)

        # where to put， is exits empty bins
        place_part_on = None
      
        available_bins =[]
        available_bins_for_kitting =[]
        for bin in bins:
            if not self.parts_on_warehouse_location[bin] or len(self.parts_on_warehouse_location[bin])<4:
                available_bins.append(bin)
                if bin in kitting_bins:
                    available_bins_for_kitting.append(bin)
    
        # who do this?
        part_on_kitting_robot_region = False
        if 'ks' in agv_present_location and len(available_bins_for_kitting)>=1:
            part_on_kitting_robot_region = True

        if len(available_bins)==0:
            place_part_on = "can" 
            part_on_kitting_robot_region = True

        if self.robot_info['kitting_robot'].is_enabled and self.robot_info['kitting_robot'].is_idle and \
            part_on_kitting_robot_region:
            # 求一个距离最短的位置
            if place_part_on == "can":
                pass
            else:
                place_part_on = self.kitting_robot_opt_short_path(self.robot_info['kitting_robot'].position,available_bins_for_kitting, agv_present_location)
            print("Task: Kitting to do withdraw:",target_part.type)
            self.cmd_list[0].command_id = rospy.get_time()
            self.cmd_list[0].robot_name = "kitting_robot"
            self.cmd_list[0].type = "withdraw"
            self.cmd_list[0].pick_part= target_part
            self.cmd_list[0].target_position = place_part_on
            self.cmd_list[0].pick_part_on = pick_part_on
            self.cmd_list[0].is_done = False
            # update 
            self.history_cmd[0] = self.cmd_list[0]

        elif self.robot_info['gantry_robot'].is_enabled and  self.robot_info['gantry_robot'].is_idle:

            if place_part_on == "can":
                pass
            else:
                place_part_on = self.gantry_robot_opt_short_path(self.robot_info['gantry_robot'].position,available_bins, agv_present_location)


            print("Task: gantry to do withdraw:",target_part.type)
            #@ gantry_robot_opt_short_path()
            self.cmd_list[1].command_id = rospy.get_time()
            self.cmd_list[1].robot_name = "gantry_robot"
            self.cmd_list[1].type = "withdraw"
            self.cmd_list[1].pick_part = target_part
            self.cmd_list[1].target_position = place_part_on
            self.cmd_list[1].pick_part_on = pick_part_on
            self.cmd_list[1].is_done = False
            self.history_cmd[1] = self.cmd_list[1]
        else:
            pass

    def assembly_operation(self, target_part, station_id):
        # where to put on
        place_part_on = station_id
        # where to pick
        expect_agv_on_as_location = AS_AGV_location[station_id]

        agv_on_as = AS_AGV[station_id]

        real_agv_position = [self.agv_location[agv_on_as[0]], self.agv_location[agv_on_as[1]]]
        pick_part_on = None
        part_on_expect_agv_on_as =[]
        # if two agv on as, search first
        if expect_agv_on_as_location[0] == real_agv_position[0] and expect_agv_on_as_location[1] == real_agv_position[1]:
            
            # have world frame
            if self.parts_on_warehouse_location[real_agv_position[0]] and self.parts_on_warehouse_location[real_agv_position[1]]:
                part_on_expect_agv_on_as = self.parts_on_warehouse_location[real_agv_position[0]]+ \
                    self.parts_on_warehouse_location[real_agv_position[1]]

            elif self.parts_on_warehouse_location[real_agv_position[0]]:
                part_on_expect_agv_on_as = self.parts_on_warehouse_location[real_agv_position[0]]
            elif self.parts_on_warehouse_location[real_agv_position[1]]:
                part_on_expect_agv_on_as = self.parts_on_warehouse_location[real_agv_position[1]]
            else:
                pass

        elif expect_agv_on_as_location[0] == real_agv_position[0] and not expect_agv_on_as_location[1] == real_agv_position[1]:
            if self.parts_on_warehouse_location[real_agv_position[0]]:
                part_on_expect_agv_on_as = self.parts_on_warehouse_location[real_agv_position[0]]
        elif not expect_agv_on_as_location[0] == real_agv_position[0] and  expect_agv_on_as_location[1] == real_agv_position[1]:
            if self.parts_on_warehouse_location[real_agv_position[1]]:
                part_on_expect_agv_on_as = self.parts_on_warehouse_location[real_agv_position[1]]
        else:
            pass

        for part in part_on_expect_agv_on_as:
            if part.type == target_part.type:
                pick_part_on = part.location
                break
        # if part not on agvs beside as
        if pick_part_on == None:
            # conveyor
            #print("is on conveyor?")
            self.parts_on_conveyor = self.sensor_system.search_part_on_conveyor(target_part.type)
            if self.parts_on_conveyor:
                robot_position = self.robot_info["gantry_robot"].position
                ##########修改了目标属性#######需要重新改过来
                pick_part = self.parts_on_conveyor[-1]

                #判断机器人能不能到达
                current_time = rospy.get_time()
                target_part_time = pick_part.time_stamp
                target_part_y = pick_part.pose.position.y

                c_y = -(current_time - target_part_time)*conveyor_vel+target_part_y
                if c_y <= robot_position.y:
                    t_best = (robot_position.y- c_y)/(gantry_velocity - conveyor_vel)  
                else:
                    t_best = (c_y - robot_position.y)/(gantry_velocity + conveyor_vel) 
                t_best = t_best+abs(robot_position.x-0.3)/gantry_velocity
                #最佳抓取位置    
                f_y = c_y - t_best*conveyor_vel
                if f_y > conveyor_end:  #gantry_robot可以到达
                    pick_part_on = "conveyor"
    
                
        if pick_part_on == None:
            #print("is on other position?")
            if self.parts_on_warehouse_type[target_part.type]:
                #print("on other position")
                need_part = self.parts_on_warehouse_type[target_part.type]
                part_type_location_dic = self.part_location_sort(need_part)# bin3 :[p1, p2]
                #@ gantry_robot_opt_short_path()
                pick_part_on = self.gantry_robot_opt_short_path(self.robot_info['gantry_robot'].position,part_type_location_dic, expect_agv_on_as_location[0])

            else:
                #print("not found")
                if not (target_part.i_d in self.lacked_parts_dic):
                    target_part.time_lack = copy.deepcopy(rospy.get_time())
                    self.lacked_parts_dic[target_part.i_d] = [target_part,station_id]
                    #print (target_part.i_d + '_is lacked')
                    return self.cmd_list

        if pick_part_on == None:
            if not (target_part.i_d in self.lacked_parts_dic):
                target_part.time_lack = copy.deepcopy(rospy.get_time())
                self.lacked_parts_dic[target_part.i_d] = [target_part,station_id]
                return self.cmd_list
            return self.cmd_list
           
        if self.robot_info['gantry_robot'].is_enabled and self.robot_info['gantry_robot'].is_idle:
            self.cmd_list[1].command_id = rospy.get_time()
            self.cmd_list[1].robot_name = "gantry_robot"
            self.cmd_list[1].type = "assembly"
            self.cmd_list[1].pick_part = target_part
            self.cmd_list[1].target_position = place_part_on
            self.cmd_list[1].pick_part_on = pick_part_on
            self.cmd_list[1].is_done = False
            self.history_cmd[1] = self.cmd_list[1]


    def cmd_list_feedback(self, cmd_list):
        self.history_cmd[0] = cmd_list[0]
        self.history_cmd[1] = cmd_list[1]
  
    def cmd_realocation(self, cmd):
        print("cmd_realocation<---task_dispator have know!")
        for command in range (0,len(self.cmd_list)):

            if cmd.pick_part.i_d == self.cmd_list[command].pick_part.i_d:
                self.cmd_list[command] = Command()
                
        for command in range (0,len(self.history_cmd)):
            if cmd.pick_part.i_d == self.history_cmd[command].pick_part.i_d:
                self.history_cmd[command] = Command()               


    def part_in_cmd_list(self, part):
        # if self.cmd_list[0].pick_part.i_d == part.i_d:
        #     return True
        # if self.cmd_list[1].pick_part.i_d == part.i_d:
        #     return True            


        if (self.history_cmd[0].pick_part.i_d == "") and (self.history_cmd[1].pick_part.i_d == ""):
            
            return False
        elif self.history_cmd[0].pick_part.i_d == "":
            if "update" in part.i_d: 
                if not "update" in self.history_cmd[1].pick_part.i_d:#有更新前的历史任务未完成
                    return True
                else: #正在执行更新后的订单
                    if part.i_d == self.history_cmd[1].pick_part.i_d:
                        return True
                    else:
                        return False
            else:
                if part.i_d == self.history_cmd[1].pick_part.i_d:
                    return True
                else:
                    return False
                    
        elif self.history_cmd[1].pick_part.i_d == "":
            if "update" in part.i_d: 
                if not "update" in self.history_cmd[0].pick_part.i_d:#有更新前的历史任务未完成
                    return True
                else: #正在执行更新后的订单
                    if part.i_d == self.history_cmd[0].pick_part.i_d:
                        return True
                    else:
                        return False
            else:
                if part.i_d == self.history_cmd[0].pick_part.i_d:
                    return True
                else:
                    return False

        for cmd in self.history_cmd:
            if ("update" in part.i_d) and (part.i_d[14:-3] == cmd.pick_part.i_d[8:-3]): 
                return True

            else:
                if part.i_d == cmd.pick_part.i_d:
                    return True
                else:
                    return False                


    def kitting_task(self, kitting_shipment):
        #先排序，把最位置最靠下的排在第一个，第二个是排在最上面的，其余的不用管
        # parts_number = len(kitting_shipment.products)
        # if parts_number>=2:
        #     min_y = 10
        #     min_part = None
        #     max_y = -10
        #     max_part = None
        #     for part in kitting_shipment.products:
        #         if part.pose.position.y <= min_y:
        #             min_y = part.pose.position.y
        #             min_part = part
        #         if part.pose.position.y >= max_y:
        #             max_y = part.pose.position.y
        #             max_part = part  
        #     tmp_products = []
        #     tmp_products.append(max_part)
        #     tmp_products.append(min_part)
        #     for iter_part in range(0,parts_number):
        #         if kitting_shipment.products[iter_part].i_d == min_part.i_d \
        #             or  kitting_shipment.products[iter_part].i_d == max_part.i_d:
        #             pass
        #         else:
        #             tmp_products.append(kitting_shipment.products[iter_part])              

        #     kitting_shipment.products = tmp_products


        for part in kitting_shipment.products:
            # print "part.i_d in self.lacked_parts_dic" , part.i_d,part.i_d in self.lacked_parts_dic
            if part.is_done or (self.part_in_cmd_list(part)) or (part.i_d in self.lacked_parts_dic):
                pass  
            else:
                # print "ready to kitting"
                self.kitting_operation(part, kitting_shipment.agv_id)
                break


    def withdraw_task(self, withdraw_shipment):
        for part in withdraw_shipment.products:
            # print "withdraw part", part.type, part.i_d, part.is_done
            # print "self.part_in_cmd_list(part)",self.part_in_cmd_list(part)
            if part.is_done or (self.part_in_cmd_list(part)):
                pass
            else:
                self.withdraw_operation(part, withdraw_shipment.agv_id)
                print("withdraw_operation: receave withdraw!")
                break

    def assembly_task(self, assembly_shipment):
        for part in assembly_shipment.products:
            if part.is_done or (self.part_in_cmd_list(part)) or (part.i_d in self.lacked_parts_dic):
                pass
            else:
                if "regulator" in part.type or "sensor" in part.type:
                    self.order_system.part_update(part.i_d)
                else:
                    self.assembly_operation(part,assembly_shipment.station_id)
                    break

    def withdraw_shipments(self,withdraw_shipments):
        for withdraw_shipment in withdraw_shipments:
            if withdraw_shipment.is_done:
                pass
            else:
                self.withdraw_task(withdraw_shipment)
                break

    def kitting_shipments(self,kitting_shipments):
        # print "kitting shipments info"
        for kitting_shipment in kitting_shipments:
            # print "kitting_shipment.is_done",kitting_shipment.is_done
            # print "product in kittinf_shipment:", len(kitting_shipment.products)
            if kitting_shipment.is_done:
                if not kitting_shipment.is_submit:
                    self.read_agv_location()
                    agv_present_location = self.agv_location[kitting_shipment.agv_id] # ks or as_x                    

                    ##################################zt#########################################
                    if not kitting_shipment.is_submit:
                    #    if kitting_shipment.agv_id == 'agv1':
                    #        agv_parts_list_n = self.sensor_system.agv1_ks1_tray_parts
                    #        quality_control_sensor_n = self.sensor_system.quality_control_sensor_1_parts
                    #    elif kitting_shipment.agv_id == 'agv2':
                    #        agv_parts_list_n = self.sensor_system.agv2_ks2_tray_parts
                    #        quality_control_sensor_n = self.sensor_system.quality_control_sensor_2_parts
                    #    elif kitting_shipment.agv_id == 'agv3':
                    #        agv_parts_list_n = self.sensor_system.agv3_ks3_tray_parts
                    #        quality_control_sensor_n = self.sensor_system.quality_control_sensor_3_parts
                    #    elif kitting_shipment.agv_id == 'agv4':  
                    #        agv_parts_list_n = self.sensor_system.agv4_ks4_tray_parts
                    #        quality_control_sensor_n = self.sensor_system.quality_control_sensor_4_parts
                        rospy.sleep(1.0)
                        if self.robot_info['kitting_robot'].is_enabled and self.robot_info['kitting_robot'].is_idle:
                            # self.robot_system.kitting_robot_do_part_check(kitting_shipment,agv_parts_list_n,quality_control_sensor_n,)
                            try:
                                thread.start_new_thread(self.robot_system.kitting_robot_do_part_check,(kitting_shipment,) )
                                kitting_shipment.is_submit = True
                            except:
                                print("Error: unable to start thread")
                        elif self.robot_info['gantry_robot'].is_enabled and self.robot_info['gantry_robot'].is_idle:
                            # self.robot_system.gantry_robot_do_part_check(kitting_shipment,agv_parts_list_n,quality_control_sensor_n,)
                            try:
                                thread.start_new_thread(self.robot_system.gantry_robot_do_part_check,(kitting_shipment,) )
                                kitting_shipment.is_submit = True
                            except:
                                print("Error: unable to start thread")


                
                        
            elif not kitting_shipment.is_done:
                self.kitting_task(kitting_shipment) 
                break 
            else:
                pass                        

    def assembly_shipments(self,assembly_shipments):
        # print "assembly shipments info"
        for assembly_shipment in assembly_shipments:
            # print "assembly_shipment.is_done",assembly_shipment.is_done
            if assembly_shipment.is_done:
                if not assembly_shipment.is_submit:
                    sub_shipment_type = assembly_shipment.shipment_type
                    if "update" in sub_shipment_type:
                        sub_shipment_type = sub_shipment_type.replace("update","")
                    assembly_submit_shipment(assembly_shipment.station_id,sub_shipment_type)
                    print("submit")
                    assembly_shipment.is_submit = True
                    self.order_system.shipment_is_submit(assembly_shipment.shipment_type)
                #@3 tell order_processor assembly_shipment.is_done = true
                # self.order_system.shipment_update(assembly_shipment.shipment_type)
                
            elif not assembly_shipment.is_done:
                self.assembly_task(assembly_shipment)
                break
            else:
                pass

########################################################################################################
#######################################################################################################


    def cmd_feedback(self, cmd):
        
        if cmd.is_done:                  
            # if cmd.robot_name == "gantry_robot":
            #     self.robot_info["gantry_robot"].is_idle = True

            
            if not (cmd.type == "pick_tray"):
                print('cmd.robot_name', cmd.robot_name)
                print("DONE:cmd.pick_part.i_d",cmd.pick_part.i_d)
                self.order_system.part_update(cmd.pick_part.i_d)    

            for i in range(0,len(self.history_cmd)):
                if (cmd.type == "pick_tray"):
                    self.history_cmd[1] = Command()
                else:
                    if cmd.pick_part.i_d == self.history_cmd[i].pick_part.i_d:
                        self.history_cmd[i] = Command()

            # if self.sensor_system.is_alive:
        self.get_all_part_on_warehouse()

    def gantry_robot_assembly_cmd_generate(self, target_part, station_id):
        # where to put on
        place_part_on = station_id
        # where to pick
        expect_agv_on_as_location = AS_AGV_location[station_id]

        agv_on_as = AS_AGV[station_id]

        real_agv_position = [self.agv_location[agv_on_as[0]], self.agv_location[agv_on_as[1]]]
        pick_part_on = None
        part_on_expect_agv_on_as =[]
        
        self.get_all_part_on_warehouse()

        # if two agv on as, search first
        if (expect_agv_on_as_location[0] == real_agv_position[0]) and (expect_agv_on_as_location[1] == real_agv_position[1]):
            #print('agv on target location')
            
            # have world frame
            if self.parts_on_warehouse_location[real_agv_position[0]] and self.parts_on_warehouse_location[real_agv_position[1]]:
                part_on_expect_agv_on_as = self.parts_on_warehouse_location[real_agv_position[0]]+ \
                    self.parts_on_warehouse_location[real_agv_position[1]]

            elif self.parts_on_warehouse_location[real_agv_position[0]]:
                part_on_expect_agv_on_as = self.parts_on_warehouse_location[real_agv_position[0]]
            elif self.parts_on_warehouse_location[real_agv_position[1]]:
                part_on_expect_agv_on_as = self.parts_on_warehouse_location[real_agv_position[1]]
            else:
                pass

        elif (expect_agv_on_as_location[0] == real_agv_position[0]) and (not (expect_agv_on_as_location[1] == real_agv_position[1])):
            # print('1')
            # print(self.parts_on_warehouse_location[real_agv_position[0]])
            if self.parts_on_warehouse_location[real_agv_position[0]]:
                part_on_expect_agv_on_as = self.parts_on_warehouse_location[real_agv_position[0]]

        elif (not (expect_agv_on_as_location[0] == real_agv_position[0])) and  (expect_agv_on_as_location[1] == real_agv_position[1]):
            if self.parts_on_warehouse_location[real_agv_position[1]]:
                part_on_expect_agv_on_as = self.parts_on_warehouse_location[real_agv_position[1]]
        else:
            pass

        for part in part_on_expect_agv_on_as:
            if part.type == target_part.type:
                pick_part_on = part.location
                break
        # if part not on agvs beside as
        if pick_part_on == None:
            # conveyor
            #print("is on conveyor?")
            self.parts_on_conveyor = self.sensor_system.search_part_on_conveyor(target_part.type)
            if self.parts_on_conveyor:
                robot_position = self.robot_info["gantry_robot"].position
                ##########修改了目标属性#######需要重新改过来
                pick_part = self.parts_on_conveyor[-1]

                #判断机器人能不能到达
                current_time = rospy.get_time()
                target_part_time = pick_part.time_stamp
                target_part_y = pick_part.pose.position.y

                c_y = -(current_time - target_part_time)*conveyor_vel+target_part_y
                if c_y <= robot_position.y:
                    t_best = (robot_position.y- c_y)/(gantry_velocity - conveyor_vel)  
                else:
                    t_best = (c_y - robot_position.y)/(gantry_velocity + conveyor_vel) 
                t_best = t_best+abs(robot_position.x-0.3)/gantry_velocity
                #最佳抓取位置    
                f_y = c_y - t_best*conveyor_vel
                if f_y > conveyor_end:  #gantry_robot可以到达
                    pick_part_on = "conveyor"
    
                
        if pick_part_on == None:
            #print("is on other position?")
            if self.parts_on_warehouse_type[target_part.type]:
                #print("on other position")
                need_part = self.parts_on_warehouse_type[target_part.type]
                part_type_location_dic = self.part_location_sort(need_part)# bin3 :[p1, p2]
                #@ gantry_robot_opt_short_path()
                pick_part_on = self.gantry_robot_opt_short_path(self.robot_info['gantry_robot'].position,part_type_location_dic, expect_agv_on_as_location[0])

            else:
                #print("not found")
                if not (target_part.i_d in self.lacked_parts_dic.keys()):
                    target_part.time_lack = copy.deepcopy(rospy.get_time())
                    self.lacked_parts_dic[target_part.i_d] = [target_part,station_id]
                    print (target_part.i_d + '_is lacked！！！')
                    return self.cmd_list

        if pick_part_on == None:
            if not (target_part.i_d in self.lacked_parts_dic):
                target_part.time_lack = copy.deepcopy(rospy.get_time())
                self.lacked_parts_dic[target_part.i_d] = [target_part,target_part.target_agv_id]
                return self.cmd_list
            return self.cmd_list
           
        if self.robot_info['gantry_robot'].is_enabled and self.robot_info['gantry_robot'].is_idle:
            self.cmd_list[1].command_id = rospy.get_time()
            self.cmd_list[1].robot_name = "gantry_robot"
            self.cmd_list[1].type = "assembly"
            self.cmd_list[1].pick_part = target_part
            self.cmd_list[1].target_position = place_part_on
            self.cmd_list[1].pick_part_on = pick_part_on
            self.cmd_list[1].is_done = False
            self.history_cmd[1] = self.cmd_list[1]
            return self.cmd_list
        else:
            return self.empty_comd_list

    def withdraw_cmd_generate(self,target_part, target_agv_id):

        # where to pick_up
        self.read_agv_location()
        agv_present_location = self.agv_location[target_agv_id]
        pick_part_on = agv_present_location

        self.get_all_part_on_warehouse()
        target_part_list = self.sensor_system.search_part_location_type(agv_present_location,target_part.type)
        target_part = target_pose_to_world(target_part,target_agv_id, agv_present_location)
        if target_part_list:
            tmp_part = self.sensor_system.Withdraw_Part_Search(target_part, target_part_list, part_distance = 0.05)        
            if tmp_part:
                target_part.pose = tmp_part.pose
                target_part.u_id = tmp_part.u_id




        place_part_on = None
        available_bins =[]
        available_bins_for_kitting =[]
        for bin in bins:
            if not self.parts_on_warehouse_location[bin] or len(self.parts_on_warehouse_location[bin])<4:
                available_bins.append(bin)
                if bin in kitting_bins:
                    available_bins_for_kitting.append(bin)
    
        # who do this?
        part_on_kitting_robot_region = False
        if 'ks' in agv_present_location and len(available_bins_for_kitting)>=1:
            part_on_kitting_robot_region = True

        if len(available_bins)==0:
            place_part_on = "can" 
            part_on_kitting_robot_region = True

        if self.robot_info['kitting_robot'].is_enabled and self.robot_info['kitting_robot'].is_idle and \
            part_on_kitting_robot_region:
            # 求一个距离最短的位置
            if place_part_on == "can":
                pass
            else:
                place_part_on = self.kitting_robot_opt_short_path(self.robot_info['kitting_robot'].position,available_bins_for_kitting, agv_present_location)
            print("Task: Kitting to do withdraw:",target_part.i_d)
            self.cmd_list[0].command_id = rospy.get_time()
            self.cmd_list[0].robot_name = "kitting_robot"
            self.cmd_list[0].type = "withdraw"
            self.cmd_list[0].pick_part= target_part
            self.cmd_list[0].target_position = place_part_on
            self.cmd_list[0].pick_part_on = pick_part_on
            self.cmd_list[0].is_done = False
            # update 
            self.history_cmd[0] = self.cmd_list[0]
            return self.cmd_list

        elif self.robot_info['gantry_robot'].is_enabled and  self.robot_info['gantry_robot'].is_idle:

            if place_part_on == "can":
                pass
            else:
                place_part_on = self.gantry_robot_opt_short_path(self.robot_info['gantry_robot'].position,available_bins, agv_present_location)


            print("Task: gantry to do withdraw:",target_part.i_d)
            #@ gantry_robot_opt_short_path()
            self.cmd_list[1].command_id = rospy.get_time()
            self.cmd_list[1].robot_name = "gantry_robot"
            self.cmd_list[1].type = "withdraw"
            self.cmd_list[1].pick_part = target_part
            self.cmd_list[1].target_position = place_part_on
            self.cmd_list[1].pick_part_on = pick_part_on
            self.cmd_list[1].is_done = False
            self.history_cmd[1] = self.cmd_list[1]
            return self.cmd_list
        else:
            pass
        return self.empty_comd_list

    def adjust_cmd_generate(self,target_part,target_agv_id):
          
        # where to pick_up
        self.read_agv_location()
        agv_present_location = self.agv_location[target_agv_id]
        pick_part_on = agv_present_location
        
        
        self.get_all_part_on_warehouse()
        target_part_list = self.sensor_system.search_part_location_type(agv_present_location,target_part.type)
        target_part = target_pose_to_world(target_part,target_agv_id, agv_present_location)
        if target_part_list:
            tmp_part = self.sensor_system.Withdraw_Part_Search(target_part, target_part_list, part_distance = 0.05)
            if tmp_part:
                target_part.pose = tmp_part.pose
                target_part.u_id = tmp_part.u_id

        #定义调整后的part
        adjust_part = copy.deepcopy(target_part)   
        adjust_part.pose = target_part.adjust_pose
        adjust_part = target_pose_to_world(adjust_part,target_agv_id, agv_present_location)
        target_part.adjust_pose = adjust_part.pose 

        
        
        place_part_on = pick_part_on

        if self.robot_info['kitting_robot'].is_enabled and self.robot_info['kitting_robot'].is_idle and \
            "ks" in place_part_on:
            self.cmd_list[0].command_id = rospy.get_time()
            self.cmd_list[0].robot_name = "kitting_robot"
            self.cmd_list[0].type = "adjust"
            self.cmd_list[0].pick_part= target_part
            self.cmd_list[0].target_position = place_part_on
            self.cmd_list[0].pick_part_on = pick_part_on
            self.cmd_list[0].is_done = False
            # update 
            self.history_cmd[0] = self.cmd_list[0]
            return self.cmd_list

        elif self.robot_info['gantry_robot'].is_enabled and  self.robot_info['gantry_robot'].is_idle:

            #@ gantry_robot_opt_short_path()
            self.cmd_list[1].command_id = rospy.get_time()
            self.cmd_list[1].robot_name = "gantry_robot"
            self.cmd_list[1].type = "adjust"
            self.cmd_list[1].pick_part = target_part
            self.cmd_list[1].target_position = place_part_on
            self.cmd_list[1].pick_part_on = pick_part_on
            self.cmd_list[1].is_done = False
            self.history_cmd[1] = self.cmd_list[1]
            return self.cmd_list
        else:
            pass
        return self.empty_comd_list


    def gantry_robot_cmd_generate(self, tar_part, task_flag, order_working_agv_id):
        #机器人cmd命令生成
        agv_present_location = self.agv_location[tar_part.target_agv_id] # ks or as_x
        target_part = target_pose_to_world(tar_part,tar_part.target_agv_id, agv_present_location)
        #@ decide where to place on
        place_part_on = agv_present_location
        #先判断是否已经有了
        part_on_expect_agv = self.sensor_system.search_part_location_type(agv_present_location,target_part.type)
        target_part_rpy = euler_from_quaternion([target_part.pose.orientation.x,target_part.pose.orientation.y,\
            target_part.pose.orientation.z,target_part.pose.orientation.w])
        if part_on_expect_agv:
            for part in part_on_expect_agv:
                part_rpy = euler_from_quaternion([part.pose.orientation.x,part.pose.orientation.y,\
                part.pose.orientation.z,part.pose.orientation.w])
                # print ("part_on_agv_rpy",part_rpy)
                # print ("target_part_rpy",target_part_rpy)
                if  xyz_check(part, target_part) and rpy_check(part_rpy,target_part_rpy):
                    print("part have been put on ")
                    self.order_system.part_update(target_part.i_d)
                    self.cmd_list[1] = self.empty_comd_list[1]
                    return self.cmd_list

        
        #优先获取传送带上有对应的零件
        part_on_conveyor_flag = False
        self.parts_on_conveyor = self.sensor_system.search_part_on_conveyor(target_part.type)
        if self.parts_on_conveyor:
            robot_position = self.robot_info["gantry_robot"].position
            target_part_con = self.parts_on_conveyor[-1]
        
            #判断机器人能不能到达
            current_time = rospy.get_time()
            target_part_time = target_part_con.time_stamp
            target_part_y = target_part_con.pose.position.y
            
            c_y = -(current_time - target_part_time)*conveyor_vel+target_part_y
            
            if c_y <= robot_position.y:
                t_best = (robot_position.y- c_y)/(gantry_velocity - conveyor_vel)  
            else:
                t_best = (c_y - robot_position.y)/(gantry_velocity + conveyor_vel) 
            t_best = t_best+abs(robot_position.x-0.3)/gantry_velocity
            #最佳抓取位置    
            f_y = c_y - t_best*conveyor_vel
            if f_y > conveyor_end:  #gantry_robot可以到达
                pick_part_on = "conveyor"
                part_on_conveyor_flag = True
                ##################
                self.cmd_list[1].command_id = rospy.get_time()
                self.cmd_list[1].robot_name = "gantry_robot"
                self.cmd_list[1].type = "kitting"
                self.cmd_list[1].pick_part = target_part
                self.cmd_list[1].target_position = place_part_on
                self.cmd_list[1].pick_part_on = pick_part_on
                self.cmd_list[1].is_done = False
                self.history_cmd[1] = self.cmd_list[1]
                return self.cmd_list
        else:
            part_on_conveyor_flag = False
            
        if part_on_conveyor_flag == False:
            #更新零件列表
            self.get_all_part_on_warehouse()

            #仅获取在bin上的零件，agv上的零件待处理
            part_exist_on_bin = []
            part_exist_on_agv = []
            if self.parts_on_warehouse_type[target_part.type]:
                need_part = self.parts_on_warehouse_type[target_part.type]
                part_type_location_dic = self.part_location_sort(need_part)
                
                for key in part_type_location_dic:
                    if "bin" in key:
                        part_exist_on_bin.append(key)
                    if "agv" in key and key not in order_working_agv_id and "ks" in key:
                        part_exist_on_agv.append(key)                        
            
            if not part_exist_on_bin:
                if task_flag == 2 and part_exist_on_agv:
                    #for temp_num in range(0, len(part_exist_on_agv)):
                    pick_part_on = self.gantry_robot_opt_short_path(self.robot_info['gantry_robot'].position,part_exist_on_agv, agv_present_location)
                    place_part_on = agv_present_location
                    self.cmd_list[1].command_id = rospy.get_time()
                    self.cmd_list[1].robot_name = "gantry_robot"
                    self.cmd_list[1].type = "kitting"
                    self.cmd_list[1].pick_part = target_part
                    self.cmd_list[1].target_position = place_part_on
                    self.cmd_list[1].pick_part_on = pick_part_on
                    self.cmd_list[1].is_done = False
                    self.history_cmd[1] = self.cmd_list[1]
                    return self.cmd_list                            

                #零件不足
                if not (target_part.i_d in self.lacked_parts_dic):
                    target_part.time_lack = copy.deepcopy(rospy.get_time())
                    self.lacked_parts_dic[target_part.i_d] = [target_part,target_part.target_agv_id]
                    return self.cmd_list
            else:
               
                pick_part_on = self.gantry_robot_opt_short_path(self.robot_info['gantry_robot'].position,part_exist_on_bin, agv_present_location)
              
                place_part_on = agv_present_location
                self.cmd_list[1].command_id = rospy.get_time()
                self.cmd_list[1].robot_name = "gantry_robot"
                self.cmd_list[1].type = "kitting"
                self.cmd_list[1].pick_part = target_part
                self.cmd_list[1].target_position = place_part_on
                self.cmd_list[1].pick_part_on = pick_part_on
                self.cmd_list[1].is_done = False
                self.history_cmd[1] = self.cmd_list[1]
                return self.cmd_list

        self.cmd_list[1] = self.empty_comd_list[1]
        return self.cmd_list
      

    def kitting_robot_cmd_generate(self, tar_part, task_flag, order_working_agv_id):
        #机器人cmd命令生成
        agv_present_location = self.agv_location[tar_part.target_agv_id] # ks or as_x
        target_part = target_pose_to_world(tar_part,tar_part.target_agv_id, agv_present_location)
        #@ decide where to place on
        place_part_on = agv_present_location
        #先判断是否已经有了
        part_on_expect_agv = self.sensor_system.search_part_location_type(agv_present_location,target_part.type)
        target_part_rpy = euler_from_quaternion([target_part.pose.orientation.x,target_part.pose.orientation.y,\
            target_part.pose.orientation.z,target_part.pose.orientation.w])
        if part_on_expect_agv:
            for part in part_on_expect_agv:
                part_rpy = euler_from_quaternion([part.pose.orientation.x,part.pose.orientation.y,\
                part.pose.orientation.z,part.pose.orientation.w])

                if xyz_check(part, target_part) and rpy_check(part_rpy,target_part_rpy):
                    self.order_system.part_update(target_part.i_d)
                    self.cmd_list[0] = self.empty_comd_list[0]
                    return self.cmd_list


        #优先获取传送带上有对应的零件
        part_on_conveyor_flag = False
        self.parts_on_conveyor = self.sensor_system.search_part_on_conveyor(target_part.type)
        if self.parts_on_conveyor:
            robot_position = self.robot_info["kitting_robot"].position
            target_part_con = self.parts_on_conveyor[-1]
        
            #判断机器人能不能到达
            current_time = rospy.get_time()
            target_part_time = target_part_con.time_stamp
            target_part_y = target_part_con.pose.position.y

            c_y = -(current_time - target_part_time)*conveyor_vel+target_part_y
            
            if c_y <= robot_position.y:
                t_best = (robot_position.y- c_y)/(kitting_velocity - conveyor_vel)  
            else:
                t_best = (c_y - robot_position.y)/(kitting_velocity + conveyor_vel) 
            f_y = c_y - t_best*conveyor_vel - 0.17
            if f_y > conveyor_end:  #说明kitting可以到达
                pick_part_on = "conveyor"
                part_on_conveyor_flag = True
                ##################
                self.cmd_list[0].command_id = rospy.get_time()
                self.cmd_list[0].robot_name = "kitting_robot"
                self.cmd_list[0].type = "kitting"
                self.cmd_list[0].pick_part= target_part
                self.cmd_list[0].target_position = place_part_on
                self.cmd_list[0].pick_part_on = pick_part_on
                self.cmd_list[0].is_done = False
                # update 
                self.history_cmd[0] = self.cmd_list[0]
                return self.cmd_list
        else:
            part_on_conveyor_flag = False
            
        if part_on_conveyor_flag == False and task_flag != 3:
            #更新零件列表
            self.get_all_part_on_warehouse()

            #仅获取在bin上的零件，agv上的零件待处理
            part_exist_on_bin = []
            part_exist_on_agv = []
            if self.parts_on_warehouse_type[target_part.type]:
                need_part = self.parts_on_warehouse_type[target_part.type]
                part_type_location_dic = self.part_location_sort(need_part)                

                for key in part_type_location_dic:
                    if ("bin1" in key or\
                        "bin2" in key or\
                        "bin5" in key or\
                        "bin6" in key):
                        part_exist_on_bin.append(key)
                    if "agv" in key and "ks" in key and key not in order_working_agv_id:
                        part_exist_on_agv.append(key) 

            if not part_exist_on_bin:
                if task_flag == 2 and part_exist_on_agv:
                    pick_part_on = self.kitting_robot_opt_short_path(self.robot_info['kitting_robot'].position,part_exist_on_agv, agv_present_location)
                    place_part_on = agv_present_location
                    self.cmd_list[0].command_id = rospy.get_time()
                    self.cmd_list[0].robot_name = "kitting_robot"
                    self.cmd_list[0].type = "kitting"
                    self.cmd_list[0].pick_part = target_part
                    self.cmd_list[0].target_position = place_part_on
                    self.cmd_list[0].pick_part_on = pick_part_on
                    self.cmd_list[0].is_done = False
                    self.history_cmd[0] = self.cmd_list[0]
                    return self.cmd_list                            

                if not (target_part.i_d in self.lacked_parts_dic):
                    target_part.time_lack = copy.deepcopy(rospy.get_time())
                    self.lacked_parts_dic[target_part.i_d] = [target_part,target_part.target_agv_id]
                    return self.cmd_list
            else:        
                #零件flip未解决
                pick_part_on = self.kitting_robot_opt_short_path(self.robot_info['kitting_robot'].position,part_exist_on_bin, agv_present_location)
                place_part_on = agv_present_location
                self.cmd_list[0].command_id = rospy.get_time()
                self.cmd_list[0].robot_name = "kitting_robot"
                self.cmd_list[0].type = "kitting"
                self.cmd_list[0].pick_part= target_part
                self.cmd_list[0].target_position = place_part_on
                self.cmd_list[0].pick_part_on = pick_part_on
                self.cmd_list[0].is_done = False
                # update 
                self.history_cmd[0] = self.cmd_list[0]
                # print('self.cmd_list[0].pick_partaaaaaa',self.cmd_list[0].pick_part.i_d)
                return self.cmd_list
        self.cmd_list[0] = self.empty_comd_list[0]
        return self.cmd_list

    def order_working_agv_back(self, order_list_n):
        order_working_agv_id = []
        for kitting_shipment in order_list_n.kitting_shipments.kitting_shipment:
            order_working_agv_id.append(self.agv_location[kitting_shipment.agv_id]) 
        return order_working_agv_id

    def order_list_handle(self, order_list_n):

        all_parts_shipments = []
        for kitting_shipment in order_list_n.kitting_shipments.kitting_shipment:
            kitting_shipment_flag = True

            if kitting_shipment.is_submit == True:
                continue
            elif kitting_shipment.is_submit == False:

                for part in kitting_shipment.products:
                    all_parts_shipments.append(part)
                    if not part.is_done:
                        if part.type in self.can_not_find_part_type:
                            self.order_system.part_update(part.i_d)
                            continue
                        kitting_shipment_flag = False
                        part.target_agv_id = kitting_shipment.agv_id
                        need_part = self.parts_on_warehouse_type[part.type]
                        part_type_location_dic = self.part_location_sort(need_part)
                        if ("bin8" in part_type_location_dic or\
                            "bin7" in part_type_location_dic or\
                            "bin4" in part_type_location_dic or\
                            "bin3" in part_type_location_dic) and\
                            ("bin6" not in part_type_location_dic and\
                            "bin5" not in part_type_location_dic and\
                            "bin2" not in part_type_location_dic and\
                            "bin1" not in part_type_location_dic):
                            #只能用gantry机器人去做的零件
                            self.kitting_bin_part_list_gantry.append(part)
                        elif ("bin6" in part_type_location_dic or\
                            "bin5" in part_type_location_dic or\
                            "bin2" in part_type_location_dic or\
                            "bin1" in part_type_location_dic):
                            #kitting和gantry机器人都可以去做的零件
                            self.kitting_bin_part_list_kitting.append(part)
                        elif 'bin' not in part_type_location_dic:
                            #不在bin上的零件
                            self.kitting_no_bin_part_list.append(part)
           
            if kitting_shipment_flag == True and kitting_shipment.is_submit == False and kitting_shipment.check_flag == 0:
                self.shipment_complete_not_check.append(kitting_shipment)
                self.order_system.change_kitting_shipment_part_flag_id(kitting_shipment, part_check_flag=1)

            if kitting_shipment_flag == False and kitting_shipment.check_flag == 2:
                self.order_system.change_kitting_shipment_part_flag_id(kitting_shipment, part_check_flag=0)

            if (kitting_shipment_flag == True and kitting_shipment.check_flag == 2 and kitting_shipment.is_submit == False and self.sensor_system.is_alive()):
                #提交订单
                agv_parts_list_n = self.sensor_system.read_agv_list_by_agv_id(kitting_shipment.agv_id)
                print("agv_parts_list_n")
                print(len(agv_parts_list_n))
                agv_present_location = self.agv_location[kitting_shipment.agv_id]
                if self.order_system.reset_kitting_shipment_part_done_id(kitting_shipment, agv_parts_list_n, agv_present_location, self.can_not_find_part_type):
                    print("submit")
                    respond = self.robot_system.submit_alert(agv_present_location)
    
                    if respond:
                        sub_shipment_type = kitting_shipment.shipment_type
                        if "update" in sub_shipment_type:
                            sub_shipment_type = sub_shipment_type.replace("update","")
                        print ("kitting_shipment.agv_id",kitting_shipment.agv_id)
                        print ("kitting_shipment.station_id",kitting_shipment.station_id)
                        print ("kitting_shipment.shipment_type",sub_shipment_type)
                        kitting_submit_shipment(kitting_shipment.agv_id, kitting_shipment.station_id, sub_shipment_type)
                        print ("submit--------------------------")
                        self.order_system.change_kitting_shipment_submit_flag_id(kitting_shipment, submit_flag=True)
                        rospy.sleep(2)
                        #删除操作
                        for part in kitting_shipment.products:
                            part_on_agv = self.sensor_system.search_part_location_type(agv_ks_location[kitting_shipment.agv_id], part.type)
                            if part_on_agv:
                                for p in part_on_agv:
                                    self.sensor_system.del_part_from_parts_list(p)
                else:
                    self.order_system.change_kitting_shipment_part_flag_id(kitting_shipment, part_check_flag=0)


        #print "***********************"
        #for part in all_parts_shipments:
        #    print part.i_d

        #print "***********************"
        
        for assembly_shipment in order_list_n.assembly_shipments.assembly_shipment:
            assembly_shipment_flag = True
            if assembly_shipment.is_done == True and assembly_shipment.is_submit == True:
                continue
            elif assembly_shipment.is_done == False:
                for part in assembly_shipment.products:
                    part.target_agv_id = assembly_shipment.station_id
                    if not part.is_done:
                        assembly_shipment_flag = False
                        self.assembly_part_list_gantry.append(part)
            elif assembly_shipment_flag == True  and assembly_shipment.is_submit == False:
                #提交订单
                sub_shipment_type = assembly_shipment.shipment_type
                if "update" in sub_shipment_type:
                    sub_shipment_type = sub_shipment_type.replace("update","")
                assembly_submit_shipment(assembly_shipment.station_id,sub_shipment_type)
                print("submit")
                assembly_shipment.is_submit = True
                self.order_system.shipment_is_submit(assembly_shipment.shipment_type) 
                assembly_shipment.is_done = True
                pass                        
       
        for withdraw_shipment in order_list_n.withdraw_shipments.withdraw_shipment:
            withdraw_shipment_flag = True
            if withdraw_shipment.is_done == True:
                continue
            elif withdraw_shipment.is_done == False:
                for part in withdraw_shipment.products:
                    part.target_agv_id = withdraw_shipment.agv_id
                    if not part.is_done:
                        withdraw_shipment_flag = False
                        if part.is_adjust:
                            if "pump" in part.type:
                                o_part = copy.deepcopy(part)
                                t_part = copy.deepcopy(part)
                                t_part.pose = part.adjust_pose

                                po = euler_from_quaternion([o_part.pose.orientation.x,o_part.pose.orientation.y,o_part.pose.orientation.z,o_part.pose.orientation.w])
                                pt = euler_from_quaternion([t_part.pose.orientation.x,t_part.pose.orientation.y,t_part.pose.orientation.z,t_part.pose.orientation.w])

                                if abs(abs(po[0]) - abs(pt[0]))> 3: 
                                    self.withdraw_part_list.append(part)
                                else:
                                    self.adjust_part_list.append(part)
                            else:
                                self.adjust_part_list.append(part)
                        else:
                            self.withdraw_part_list.append(part) 

    def all_robot_cmd_generate(self, order_list_n, order_working_agv_id):
        task_allocation_flag = False
        self.order_list = self.order_system.return_order_list()
        if len(self.order_list) >= 2 and order_list_n == 1:  
            self.order_list_handle(self.order_list[order_list_n]) 
        elif len(self.order_list) >= 1 and order_list_n == 0:  
            self.order_list_handle(self.order_list[order_list_n]) 
        #任务分配  

        if self.withdraw_part_list:
            ####任务分配
            for part in self.withdraw_part_list:
                if part.is_done or (self.part_in_cmd_list(part)):
                    continue  
                else:
                    cmd_list = self.withdraw_cmd_generate(part, part.target_agv_id)
                    task_allocation_flag = True
                    return cmd_list, task_allocation_flag
                    break
        
        if self.adjust_part_list:
            ####任务分配
            for part in self.adjust_part_list:
                if part.is_done or (self.part_in_cmd_list(part)):
                    continue  
                else:
                    cmd_list = self.adjust_cmd_generate(part, part.target_agv_id)
                    task_allocation_flag = True
                    return cmd_list, task_allocation_flag
                    break
           
                           
        if self.robot_info['kitting_robot'].is_enabled and self.robot_info['kitting_robot'].is_idle:
            #先判断是否有特殊任务
            #判断是否有订单待检测
            task_flag = 0               
            if self.sensor_system.is_alive():
                if self.shipment_complete_not_check:
                    try:
                        shipment_complete_temp = copy.deepcopy(self.shipment_complete_not_check[0])
                        thread.start_new_thread(self.robot_system.kitting_robot_do_part_check,(shipment_complete_temp,) )
                        del self.shipment_complete_not_check[0]
                        self.cmd_list[0] = self.empty_comd_list[0]
                        task_allocation_flag = True
                        return self.cmd_list, task_allocation_flag
                    except:
                        print("Error: unable to start thread")


            if self.kitting_no_bin_part_list and task_flag == 0:
                # print("self.kitting_no_bin_part_list")
                # print(len(self.kitting_no_bin_part_list))
                for part in self.kitting_no_bin_part_list:
                    # print "kitting *****************************************"
                    # print "part.i_d:",part.i_d
                    # print "part.is_done:",part.is_done
                    # print "self.part_in_cmd_list(part)",self.part_in_cmd_list(part)
                    # print "part.i_d in self.lacked_parts_dic",part.i_d in self.lacked_parts_dic
                    # print "kitting *****************************************"
                   
                    if part.is_done or (self.part_in_cmd_list(part)) or (part.i_d in self.lacked_parts_dic):
                        continue  
                    else:
                        target_part = part
                        task_flag = 2
                        break

            if self.kitting_bin_part_list_kitting and task_flag == 0:
                for part in self.kitting_bin_part_list_kitting:
                    if part.is_done or (self.part_in_cmd_list(part)) or (part.i_d in self.lacked_parts_dic):
                        continue  
                    else:
                        target_part = part
                        task_flag = 1
                        break

            if self.kitting_bin_part_list_gantry and task_flag == 0:
                for part in self.kitting_bin_part_list_gantry:
                    if part.is_done or (self.part_in_cmd_list(part)) or (part.i_d in self.lacked_parts_dic):
                        continue  
                    else:
                        target_part = part
                        task_flag = 3
                        break      
                    
            if task_flag != 0:           
                cmd_list = self.kitting_robot_cmd_generate(target_part, task_flag, order_working_agv_id)
                # print('kitting',task_flag)
                # print ('target_part.i_d',target_part.i_d)
                # print('cmd_list[0].pick_part.i_d',self.history_cmd[0].pick_part.i_d)
                # print('cmd_list[1].pick_part.i_d',self.history_cmd[1].pick_part.i_d)                     
                task_allocation_flag = True
                if cmd_list[0] != self.empty_comd_list[0]:
                    return cmd_list, task_allocation_flag
        
        if self.robot_info['gantry_robot'].is_enabled and self.robot_info['gantry_robot'].is_idle:
            #先判断是否有特殊任务
            #判断是否有订单待检测
            #目标零件信息获取
            task_flag = 0

            if self.assembly_part_list_gantry and task_flag == 0:
                ####任务分配
                for part in self.assembly_part_list_gantry:
                    # print "gantry ---------------------------------------"
                    # print "part.i_d:",part.i_d
                    # print "part.is_done:",part.is_done
                    # print "self.part_in_cmd_list(part)", self.part_in_cmd_list(part)
                    # print "part.i_d in self.lacked_parts_dic",part.i_d in self.lacked_parts_dic
                    # print "gantry ---------------------------------------"
                    if part.is_done or (self.part_in_cmd_list(part)) or (part.i_d in self.lacked_parts_dic) or \
                    (('as2' in part.target_agv_id) and self.sensor_system.as2_human_flag) or (('as4' in part.target_agv_id) and self.sensor_system.as4_human_flag):
                        continue  
                    else:
                        cmd_list = self.gantry_robot_assembly_cmd_generate(part, part.target_agv_id)
                        task_flag = 1
                        task_allocation_flag = True
                        return cmd_list, task_allocation_flag
                        break         


            if self.sensor_system.is_alive():
                if self.shipment_complete_not_check:
                    try:
                        shipment_complete_temp = copy.deepcopy(self.shipment_complete_not_check[0])
                        thread.start_new_thread(self.robot_system.gantry_robot_do_part_check,(shipment_complete_temp,) )
                        del self.shipment_complete_not_check[0]
                        self.cmd_list[1] = self.empty_comd_list[1]
                        task_allocation_flag = True
                        return self.cmd_list, task_allocation_flag
                    except:
                        print("Error: unable to start thread")

          
               #return  return cmd_list



            if self.kitting_bin_part_list_gantry and task_flag == 0:
                for part in self.kitting_bin_part_list_gantry:
                    if part.is_done or (self.part_in_cmd_list(part)) or (part.i_d in self.lacked_parts_dic):
                        continue  
                    else:
                        target_part = part
                        task_flag = 1
                        break                


            if self.kitting_bin_part_list_kitting and task_flag == 0:
                for part in self.kitting_bin_part_list_kitting:
                    if part.is_done or (self.part_in_cmd_list(part)) or (part.i_d in self.lacked_parts_dic):
                        continue  
                    else:
                        target_part = part
                        task_flag = 1
                        break
            if self.kitting_no_bin_part_list and task_flag == 0:
                for part in self.kitting_no_bin_part_list:
                    if part.is_done or (self.part_in_cmd_list(part)) or (part.i_d in self.lacked_parts_dic):
                        continue  
                    else:
                        target_part = part
                        task_flag = 2
                        break
            if task_flag != 0:                 
                cmd_list = self.gantry_robot_cmd_generate(target_part, task_flag, order_working_agv_id)
                # print('gantry')
                # print ('target_part.i_d',target_part.i_d)
                # print('cmd_list[0].pick_part.i_d',self.history_cmd[0].pick_part.i_d)
                # print('cmd_list[1].pick_part.i_d',self.history_cmd[1].pick_part.i_d)  
                task_allocation_flag = True
                if cmd_list[1] != self.empty_comd_list[1]:                
                    return cmd_list, task_allocation_flag
            
        return self.cmd_list, task_allocation_flag

    def order_needed_agv_tray(self, order_list_n):
        order_needed_agv_tray = {}
        for kitting_shipment in order_list_n.kitting_shipments.kitting_shipment:
            #默认agv只在ks处需要放置托盘
            target_position = None
            agv_tray_now = None

            if 'agv1' in kitting_shipment.agv_id:
                target_position = 'agv1_ks1_tray'
                agv_tray_now = self.sensor_system.AGV1_movable_tray
            elif 'agv2' in kitting_shipment.agv_id:
                target_position = 'agv2_ks2_tray'
                agv_tray_now = self.sensor_system.AGV2_movable_tray
            elif 'agv3' in kitting_shipment.agv_id:
                target_position = 'agv3_ks3_tray'
                agv_tray_now = self.sensor_system.AGV3_movable_tray
            elif 'agv4' in kitting_shipment.agv_id:
                target_position = 'agv4_ks4_tray'
                agv_tray_now = self.sensor_system.AGV4_movable_tray
            else:
                return order_needed_agv_tray
            
            #默认agv上不会有错误的托盘类型
            if (agv_tray_now == None) or (agv_tray_now.type != kitting_shipment.movable_tray.type):
                order_needed_agv_tray[target_position] = kitting_shipment.movable_tray

        return order_needed_agv_tray

    def gantry_tray_cmd_generate(self, order_needed_agv_tray):

        for place_tray_pos in order_needed_agv_tray:
            target_tray = order_needed_agv_tray[place_tray_pos]
            place_tray_on = place_tray_pos
            break
            

        if self.robot_info['gantry_robot'].is_enabled and self.robot_info['gantry_robot'].is_idle:
            self.cmd_list[1].command_id = rospy.get_time()
            self.cmd_list[1].robot_name = "gantry_robot"
            self.cmd_list[1].type = "pick_tray"
            self.cmd_list[1].pick_part = target_tray
            self.cmd_list[1].target_position = place_tray_on
            self.cmd_list[1].pick_part_on = 'tray_table'
            self.cmd_list[1].is_done = False
            self.history_cmd[1] = self.cmd_list[1]
            return self.cmd_list, True
        else:
            pass

        return self.empty_comd_list, False

    def agv_location_check(self, order_list_n):
        for kitting_shipment in order_list_n.kitting_shipments.kitting_shipment:
            if kitting_shipment.is_submit == True:
                continue
            elif kitting_shipment.is_submit == False:
                if ('ks' not in self.sensor_system.AGV_location[kitting_shipment.agv_id]) and \
                    (self.sensor_system.AGV_state[kitting_shipment.agv_id] == 'READY_TO_DELIVER'):
                    print('move agv')
                    if self.robot_system.submit_alert(self.sensor_system.AGV_location[kitting_shipment.agv_id]):
                        move_agv(kitting_shipment.agv_id, self.sensor_system.AGV_location[kitting_shipment.agv_id][5:8], 'ks')
                        rospy.sleep(0.2)
                        while (self.sensor_system.AGV_state[kitting_shipment.agv_id] != 'READY_TO_DELIVER'):
                            rospy.sleep(0.2)        


    def test_run(self):
        
        self.sensor_system.search_all_agv()
        self.get_all_part_on_warehouse()

        for key in list(self.lacked_parts_dic.keys()):
            target_part = self.lacked_parts_dic[key][0]
            self.parts_on_conveyor = self.sensor_system.search_part_on_conveyor(target_part.type)

            if self.parts_on_conveyor: # if exist, del this part in lacked_parts_list
                # print("found on conveyor")
                del self.lacked_parts_dic[key]
            else:
                current_time = rospy.get_time()
                delta = current_time - target_part.time_lack
                if delta > insufficient_wait_time:
                    if target_part.type not in self.can_not_find_part_type:
                        self.can_not_find_part_type.append(target_part.type)
                    del self.lacked_parts_dic[key]
                    print ("update lacked part!!!!!!!")
                    print ("target_part.i_d",target_part.i_d)
                    self.order_system.part_update(target_part.i_d)
                else:
                    pass

        if (not self.robot_info["kitting_robot"].is_idle) and (not self.robot_info["gantry_robot"].is_idle):
            return self.empty_comd_list     
        
        self.order_list = self.order_system.return_order_list()
        handle_cmd_list = self.cmd_list
        #先把两个订单进行重新处理
        if self.order_list == None or len(self.order_list)==0:
            return self.empty_comd_list
        self.kitting_bin_part_list_gantry = []
        self.kitting_bin_part_list_kitting = []
        self.kitting_no_bin_part_list = []
        self.assembly_part_list_gantry = []
        self.withdraw_part_list = []
        self.adjust_part_list = []
        order_working_agv_id = []
        if len(self.order_list) >= 2 and self.order_list[1].is_done == False:
            self.agv_location_check(self.order_list[1])
            order_working_agv_id = self.order_working_agv_back(self.order_list[1]) 
            order_needed_agv_tray = self.order_needed_agv_tray(self.order_list[1])
            if order_needed_agv_tray != {}:
                handle_cmd_list, task_allocation_flag = self.gantry_tray_cmd_generate(order_needed_agv_tray)
                if task_allocation_flag:
                    return handle_cmd_list  

            handle_cmd_list, task_allocation_flag = self.all_robot_cmd_generate(1, order_working_agv_id)
            if task_allocation_flag:
                return handle_cmd_list


        #订单1已经完成，仍有机器人空闲 
        if (self.robot_info['gantry_robot'].is_enabled and self.robot_info['gantry_robot'].is_idle) or\
        (self.robot_info['kitting_robot'].is_enabled and self.robot_info['kitting_robot'].is_idle):
            
            if len(self.order_list) >= 1 and self.order_list[0].is_done == False:
                self.agv_location_check(self.order_list[0])
                order_working_agv_id = self.order_working_agv_back(self.order_list[0]) 

                order_needed_agv_tray = self.order_needed_agv_tray(self.order_list[0])


                if order_needed_agv_tray != {}:

                    handle_cmd_list, task_allocation_flag = self.gantry_tray_cmd_generate(order_needed_agv_tray)
                    if task_allocation_flag:
                        return handle_cmd_list   
                                                                
                handle_cmd_list, task_allocation_flag = self.all_robot_cmd_generate(0, order_working_agv_id)
                if task_allocation_flag:
                    return handle_cmd_list

        return handle_cmd_list




