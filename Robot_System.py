#!/usr/bin/env python
#coding:utf-8
## version v0.1

import copy

import tf2_ros
from tf.transformations import *
import rospy
import time
import math

from ariac_path_plan import *

from nist_gear.msg import Order, RobotHealth
from nist_gear.msg import VacuumGripperState
from nist_gear.srv import AGVControl
from nist_gear.srv import ConveyorBeltControl
from nist_gear.srv import DroneControl
from nist_gear.srv import SubmitShipment
from nist_gear.srv import VacuumGripperControl,LockUnlockMovableTrayOnAgv
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped,Point,Pose
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import threading
import thread 


from Sensor_System import *
from Task_Dispatcher import *
from kitting_arm import *
from gantry_robot import *
from public_data import *
from Order_System import Part, Order_Processing


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

bin_boundary ={
    'bin1': [bin_1_x_min,bin_1_x_max,bin_1_y_min,bin_1_y_max],
    'bin2': [bin_2_x_min,bin_2_x_max,bin_2_y_min,bin_2_y_max],
    'bin3': [bin_3_x_min,bin_3_x_max,bin_3_y_min,bin_3_y_max],
    'bin4': [bin_4_x_min,bin_4_x_max,bin_4_y_min,bin_4_y_max],
    'bin5': [bin_5_x_min,bin_5_x_max,bin_5_y_min,bin_5_y_max],
    'bin6': [bin_6_x_min,bin_6_x_max,bin_6_y_min,bin_6_y_max],
    'bin7': [bin_7_x_min,bin_7_x_max,bin_7_y_min,bin_7_y_max],
    'bin8': [bin_8_x_min,bin_8_x_max,bin_8_y_min,bin_8_y_max],
}

###################
    #1  #   #2
        #
###################
    #3  #   #4
        #
###################

def lock_agv_tray(agv_tray_id):  #agv_tray_id:kit_tray_1
    rospy.wait_for_service('/ariac/'+ agv_tray_id +'/lock')
    result = rospy.ServiceProxy('/ariac/'+ agv_tray_id +'/lock',Trigger)()
    return result

def unlock_agv_tray(agv_tray_id):
    rospy.wait_for_service('/ariac/'+ agv_tray_id +'/unlock')
    result = rospy.ServiceProxy('/ariac/'+ agv_tray_id +'/unlock',Trigger)()
    return result

class Robot_System:
    def __init__(self, sensor_system, order_system):
        
        self.last_cmd_list = [Command(),Command()]
        self.effect_cmd_1 = Command()
        self.effect_cmd_2 = Command()
        self.kitting_robot = Kitting_Robot(sensor_system)
        self.gantry_robot = Gantry_Robot(sensor_system)
        self.robot_info= {"kitting_robot":self.kitting_robot.robot_info, "gantry_robot":self.gantry_robot.robot_info}
        self.sensor_system = sensor_system
        self.order_system = order_system
        self.robot_health_sub = rospy.Subscriber("/ariac/robot_health",RobotHealth, self.robot_health_callback)
        self.threadLock = threading.Lock()
        self.agv_state = []
        self.parts_on_warehouse_type = {}
        self.parts_on_warehouse_location = {}
        self.parts_on_conveyor = {}
        self.agv_locations = None
        self.gantry_robot_move_to = [None, 0]
        self.kitting_robot_move_to = [None, 0]
        self.hold_commands =[]

        self.tray_lockflag = {
            'agv1_ks1_tray' : False,
            'agv2_ks2_tray' : False,
            'agv3_ks3_tray' : False,
            'agv4_ks4_tray' : False
        }    



    def read_agv_location(self):
        self.agv_locations = self.sensor_system.search_all_agv()


    def get_all_part_on_warehouse(self):

        self.sensor_system.update()
        time.sleep(1)
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
        min_path = 200
        location_name = None 
        for part_location in target_region_list:
            dis1 = abs(robot_position.y -kitting_robot_map_dic[part_location][1])
            dis2 = abs(kitting_robot_map_dic[part_location][1] - kitting_robot_map_dic[agv_present_location][1])
            if min_path > dis1+dis2:
                min_path = dis1+dis2
                location_name = part_location
        return location_name
    def gantry_robot_opt_short_path(self, robot_position,target_region_list, agv_present_location):
        min_path = 200000000
        location_name = None 
        for part_location in target_region_list:
            dis1 = abs(robot_position.x -gantry_robot_map_dic[part_location][0])
            dis2 = abs(robot_position.y -gantry_robot_map_dic[part_location][1])
            dis3 = abs(gantry_robot_map_dic[part_location][0] - gantry_robot_map_dic[agv_present_location][0])
            dis4 = abs(gantry_robot_map_dic[part_location][1] - gantry_robot_map_dic[agv_present_location][1])
            path = dis1+dis2+dis3+dis4
            if min_path >= path:
                min_path = path
                location_name = part_location
        return location_name
    
    def robot_health_callback(self,msg):
        # self.kitting_robot.robot_info.is_enabled = msg.kitting_robot_health
        # self.gantry_robot.robot_info.is_enabled = msg.assembly_robot_health
        # print('self.kitting_robot.robot_info.is_enabled',self.kitting_robot.robot_info.is_enabled)
        # print('self.gantry_robot.robot_info.is_enabled',self.gantry_robot.robot_info.is_enabled)
        if msg.kitting_robot_health == "active":
            self.kitting_robot.robot_info.is_enabled = True
        else:
            self.kitting_robot.robot_info.is_enabled = False
        if msg.assembly_robot_health == "active":
            self.gantry_robot.robot_info.is_enabled = True
        else:
            self.gantry_robot.robot_info.is_enabled = False
        print('self.kitting_robot.robot_info.is_enabled',self.kitting_robot.robot_info.is_enabled)
        print('self.gantry_robot.robot_info.is_enabled',self.gantry_robot.robot_info.is_enabled)
        
    def robot_system_init(self):
        self.sensor_system.update()

        self.get_all_part_on_warehouse()
        self.read_agv_location()
        
        if self.kitting_robot.kitting_robot_init(Wait_flag=False) and self.gantry_robot.gantry_robot_init():
            
            return True



    ###################
        #1  #   #2
            #
    ###################
        #3  #   #4
            #
    ###################      
    def withdraw_place_position(self,bin_name, part_type):
        part_on_bin = self.sensor_system.search_part_location(bin_name)
        region_1 = True
        region_2 = True
        region_3 = True
        region_4 = True
        pose = Pose()
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        pose.position.x = bin_position[bin_name][0]
        pose.position.y = bin_position[bin_name][1]
        pose.position.z = bin_heigh + kitting_pick_part_heights_on_bin_agv[part_type]

        if part_on_bin:
            for part in part_on_bin:
                if part.pose.position.x < 0.5*(bin_boundary[bin_name][0]+bin_boundary[bin_name][1]):
                    if part.pose.position.y < 0.5*(bin_boundary[bin_name][2]+bin_boundary[bin_name][3]):
                        region_1 = False
                    else:
                        region_2 = False
                else:
                    if part.pose.position.y < 0.5*(bin_boundary[bin_name][2]+bin_boundary[bin_name][3]):
                        region_3 = False
                    else:
                        region_4 = False

        if region_2:
            pose.position.x = bin_position[bin_name][0]-0.5*0.125
            pose.position.y = bin_position[bin_name][1]+0.5*0.2
            pose.position.z = bin_heigh + kitting_pick_part_heights_on_bin_agv[part_type]
            
            return pose

        if region_3:
            pose.position.x = bin_position[bin_name][0]+0.5*0.125
            pose.position.y = bin_position[bin_name][1]-0.5*0.2
            pose.position.z = bin_heigh + kitting_pick_part_heights_on_bin_agv[part_type]
            
            return pose
        if region_4:
            pose.position.x = bin_position[bin_name][0]+0.5*0.125
            pose.position.y = bin_position[bin_name][1]+0.5*0.2
            pose.position.z = bin_heigh + kitting_pick_part_heights_on_bin_agv[part_type]
            
            return pose
        if region_1:
            pose.position.x = bin_position[bin_name][0]-0.5*0.125
            pose.position.y = bin_position[bin_name][1]-0.5*0.2
            pose.position.z = bin_heigh + kitting_pick_part_heights_on_bin_agv[part_type]
            
            return pose
            
    def find_best_check_location(self, robot_location):
        self.read_agv_location()
        agv_on_ks_list = []
        min_len = 1000000
        location = None
        for key in self.agv_locations:
            if "ks" in self.agv_locations[key]:
                agv_on_ks_list.append(self.agv_locations[key])
      
        for agv in agv_on_ks_list:
            dis_x =abs(gantry_robot_park_location[agv][0]- gantry_robot_park_location[robot_location][0])
            dis_y =abs(gantry_robot_park_location[agv][1]- gantry_robot_park_location[robot_location][1])

            if min_len>dis_x+dis_y:
                min_len = dis_x+dis_y
                location = agv

        return location

    # 把坏的物品丢到垃圾箱后，调用此函数
    def part_faulty_event(self, cmd,task_dispatcher):
        print("waitiing for new order")
        task_dispatcher.part_faulty_event(cmd)

    # 命令重新分配
    def cmd_realocation(self, cmd,task_dispatcher):
        # print("waitting for new order")
        task_dispatcher.cmd_realocation(cmd)

    # 订单完成反馈
    def cmd_feed_back(self, cmd, task_dispatcher):
        task_dispatcher.cmd_feedback(cmd)

    def submit_alert(self, agv_present_location):
        agv2_right_zone = ["bin1","bin2","bin3","bin4","agv1_ks1_tray"]
        agv2_left_zone = ["bin5","bin6","bin7","bin8","agv3_ks3_tray","agv4_ks4_tray","can"]

        agv3_right_zone = ["bin1","bin2","bin3","bin4","agv1_ks1_tray","agv2_ks2_tray","can"]
        agv3_left_zone = ["bin5","bin6","bin7","bin8","agv4_ks4_tray"]  
        #print("receive alert!!!!!")
        if self.gantry_robot.robot_info.location == agv_present_location:
            # print("self.gantry_robot.robot_info.is_idle",self.gantry_robot.robot_info.is_idle)
            if self.gantry_robot.robot_info.is_idle:
                near_park = gantry_location_near_ks(agv_present_location)
                if self.gantry_robot.robot_info.is_enabled:
                    self.gantry_robot.move_to(near_park)
                else:
                    return True
            else: #路过
                # work_state = ["picking", "placing", "flipping"]
                # while self.gantry_robot.robot_info.is_enabled and not rospy.is_shutdown() :
                rospy.sleep(2)
                print("waiting...")
                return True
    
        if not self.gantry_robot.robot_info.location == agv_present_location:
            if "agv2" in agv_present_location: 
                if self.gantry_robot.robot_info.work_state =="moving":
                    if self.gantry_robot.robot_info.next_park_location == agv_present_location:
                        return True 

                    if self.gantry_robot.robot_info.next_park_location in agv2_right_zone:
                        if self.gantry_robot.robot_info.position.y > 1.367643:
                            return True
                        elif self.gantry_robot.robot_info.position.y < 0:
                            return True
                        else:
                            rospy.sleep(0.5)#3
                            return True
                    if self.gantry_robot.robot_info.next_park_location in agv2_left_zone:
                        if self.gantry_robot.robot_info.location in agv2_right_zone:
                            if self.gantry_robot.robot_info.position.y > 2.565006:
                                return True
                            else:
                                rospy.sleep(0.5)
                                return True
                        return True 
                    return True 
                else:
                    return True   

            if "agv3" in agv_present_location:   

                if self.gantry_robot.robot_info.work_state =="moving":
                    if self.gantry_robot.robot_info.next_park_location == agv_present_location:
                        return True 

                    if self.gantry_robot.robot_info.next_park_location in agv3_left_zone:
                        if self.gantry_robot.robot_info.position.y <-1.333917:
                            return True
                        elif self.gantry_robot.robot_info.position.y > 0:
                            return True
                        else:
                            rospy.sleep(0.5)
                            return True

                    if self.gantry_robot.robot_info.next_park_location in agv3_right_zone:
                        if self.gantry_robot.robot_info.location in agv3_left_zone:
                            if self.gantry_robot.robot_info.position.y < -2.565006:
                                return True
                            else:
                                rospy.sleep(0.5)
                                return True
                        return True 
                    return True 
                else:
                    return True 

        return True

    def run(self, cmd_list, task_dispatcher):
        # print(cmd_list[0].__dict__)
        
        # print('------------------')
        # print(cmd_list[1].__dict__)
        # print('------------------')
        #print "robot system is running"
        #判断指令是否有效
        if  (not cmd_list[0]) or (cmd_list[0].is_done):
            pass
        else:
            if cmd_list[0] == self.last_cmd_list[0]:
                pass
            else:
                self.effect_cmd_1 = cmd_list[0]
                self.last_cmd_list[0] = self.effect_cmd_1
       
        if (not cmd_list[1]) or (cmd_list[1].is_done):
            pass
        else:
            if cmd_list[1] == self.last_cmd_list[1]:
                pass
            else:
                self.effect_cmd_2 = cmd_list[1]
                self.last_cmd_list[1] = self.effect_cmd_2

        #######################机器由故障到正常##################     
        if self.kitting_robot.robot_info.has_been_disabled and self.kitting_robot.robot_info.is_enabled:
            self.kitting_robot.robot_info.has_been_disabled = False
            self.kitting_robot.kitting_arm_robot_init()

            if self.kitting_robot.gripper.is_object_attached():
            #    self.kitting_robot.move_to("can")
            #    self.kitting_robot.place_part_on_can()
                self.kitting_place_part_on_near_bin()
            self.kitting_robot.robot_info.is_idle = True
            self.kitting_robot.kitting_robot_idle_time = rospy.get_time()
            self.kitting_robot.robot_info.is_alive = True
            # self.kitting_robot.robot_info.work_state= "standby"

        if self.gantry_robot.robot_info.has_been_disabled and self.gantry_robot.robot_info.is_enabled:
            self.gantry_robot.robot_info.has_been_disabled = False

            self.gantry_robot.gantry_arm_init()
            if self.gantry_robot.gripper.is_object_attached():
               self.gantry_robot.move_to("can")
               self.gantry_robot.place_part_on_can()
                # self.gantry_place_part_on_near_bin()
            self.gantry_robot.robot_info.is_idle = True
            self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
            self.gantry_robot.robot_info.is_alive = True
        #######################机器由正常到故障##################   
        if self.effect_cmd_1.robot_name == "kitting_robot" and not self.kitting_robot.robot_info.is_enabled:
            print("Kitting _robot is disabled!!!")
            self.kitting_robot.robot_info.is_idle = False
            self.kitting_robot.robot_info.has_been_disabled = True
            self.cmd_realocation(self.effect_cmd_1,task_dispatcher) # 说明任务需要重新分配
            self.effect_cmd_1 = Command()
            self.kitting_robot.robot_info.is_idle = False
            self.kitting_robot.robot_info.has_been_disabled = True
            return False

        elif self.effect_cmd_2.robot_name == "kitting_robot" and not self.kitting_robot.robot_info.is_enabled:
            print("Kitting _robot is disabled!!!")
            self.kitting_robot.robot_info.is_idle = False
            self.kitting_robot.robot_info.has_been_disabled = True
            self.cmd_realocation(self.effect_cmd_2,task_dispatcher) # 说明任务需要重新分配
            print("Kitting _robot is disabled!!!")
            self.effect_cmd_2 = Command()
            self.kitting_robot.robot_info.is_idle =False
            self.kitting_robot.robot_info.has_been_disabled = True
            return False

        if self.effect_cmd_1.robot_name == "gantry_robot" and not self.gantry_robot.robot_info.is_enabled:
            print("gantry _robot is disabled!!!")
            self.gantry_robot.robot_info.is_idle = False
            self.gantry_robot.robot_info.has_been_disabled = True
            self.cmd_realocation(self.effect_cmd_1,task_dispatcher) # 说明任务需要重新分配
            self.effect_cmd_1 = Command()
            self.gantry_robot.robot_info.is_idle = False
            self.gantry_robot.robot_info.has_been_disabled = True
            return False
        elif self.effect_cmd_2.robot_name == "gantry_robot" and not self.gantry_robot.robot_info.is_enabled:
            print("gantry _robot is disabled!!!")

        
            self.gantry_robot.robot_info.is_idle = False
            self.gantry_robot.robot_info.has_been_disabled = True       
            self.cmd_realocation(self.effect_cmd_2,task_dispatcher) # 说明任务需要重新分配
            self.effect_cmd_2 = Command()
            self.gantry_robot.robot_info.is_idle = False
            self.gantry_robot.robot_info.has_been_disabled = True
            return False


        if not self.effect_cmd_1.is_done:
            #判断CMD1指令类型
            if self.effect_cmd_1.type == "kitting":
                if self.effect_cmd_1.robot_name == "kitting_robot" and self.kitting_robot.robot_info.is_idle:
                    #判断机器人是否禁用
                    #self.kitting_robot.robot_info.is_enabled = False
                    if not self.kitting_robot.robot_info.is_enabled:
                        self.cmd_realocation(self.effect_cmd_1,task_dispatcher) # 说明任务需要重新分配
                    else:
                        if self.kitting_robot.robot_info.is_idle:
                            self.kitting_robot.robot_info.is_idle = False
                            try:
                                thread.start_new_thread(self.kitting_robot_do_kitting,(self.effect_cmd_1,task_dispatcher,) )
                                print('lkalal')
                            except:
                                print ("Error: unable to start thread") 
                                self.kitting_robot.robot_info.is_idle = True                       
                        # self.kitting_robot_do_kitting(self.effect_cmd_1,task_dispatcher)
                        self.last_cmd_list[0].is_done = True
                        
                elif self.effect_cmd_1.robot_name == "gantry_robot" and self.gantry_robot.robot_info.is_idle:
                    #判断机器人是否禁用
                    if not self.gantry_robot.robot_info.is_enabled:
                        #修改当前有效命令
                        self.cmd_realocation(self.effect_cmd_1,task_dispatcher) # 说明任务需要重新分配
                    else:
                        self.gantry_robot.robot_info.is_idle =False
                        try:
                            thread.start_new_thread(self.gantry_robot_do_kitting,(self.effect_cmd_1,task_dispatcher,) )
                        except:
                            print ("Error: unable to start thread" ) 
                            self.gantry_robot.robot_info.is_idle =True
                        
                        self.last_cmd_list[0].is_done = True
            #判断指令类型
            elif self.effect_cmd_1.type =="assembly":
                if self.effect_cmd_1.robot_name == "gantry_robot" and self.gantry_robot.robot_info.is_idle:
                    self.gantry_robot.robot_info.is_idle =False
                    try:
                        thread.start_new_thread(self.gantry_robot_do_assembly,(self.effect_cmd_1,task_dispatcher,) )
                    except:
                        print ("Error: unable to start thread" )
                        self.gantry_robot.robot_info.is_idle =True
                    self.last_cmd_list[0].is_done = True

            #判断指令类型
            elif self.effect_cmd_1.type =="withdraw":
                print ("cm1 receive withdraw task!!!!!!!!!!")
                if self.effect_cmd_1.robot_name == 'kitting_robot' and self.kitting_robot.robot_info.is_idle:

                    #判断机器人是否禁用
                    if not self.kitting_robot.robot_info.is_enabled:
                        #修改当前有效命令
                        self.cmd_realocation(self.effect_cmd_1,task_dispatcher) # 说明任务需要重新分配
                    else:
                        self.kitting_robot.robot_info.is_idle =False
                        print ("kitting robot do withdraw")
                        try:
                            thread.start_new_thread(self.kitting_robot_do_withdraw,(self.effect_cmd_1,task_dispatcher,) )
                        except:
                            print ("Error: unable to start thread" )
                            self.kitting_robot.robot_info.is_idle =True
                        self.last_cmd_list[0].is_done = True

                elif self.effect_cmd_1.robot_name == "gantry_robot" and self.gantry_robot.robot_info.is_idle:
                    #判断机器人是否禁用
                    if not self.gantry_robot.robot_info.is_enabled:
                        #修改当前有效命令
                        self.cmd_realocation(self.effect_cmd_1,task_dispatcher) # 说明任务需要重新分配
                    else:
                        self.gantry_robot.robot_info.is_idle =False
                        try:
                            thread.start_new_thread(self.gantry_robot_do_withdraw,(self.effect_cmd_1,task_dispatcher,) )
                        except:
                            print ("Error: unable to start thread")
                            self.gantry_robot.robot_info.is_idle =True

                        self.last_cmd_list[0].is_done = True             
            elif self.effect_cmd_1.type =="adjust":
                print ("cm1 receive adjust task!!!!!!!!!!")
                if self.effect_cmd_1.robot_name == 'kitting_robot' and self.kitting_robot.robot_info.is_idle:

                    #判断机器人是否禁用
                    if not self.kitting_robot.robot_info.is_enabled:
                        #修改当前有效命令
                        self.cmd_realocation(self.effect_cmd_1,task_dispatcher) # 说明任务需要重新分配
                    else:
                        self.kitting_robot.robot_info.is_idle =False
                        print ("kitting robot do withdraw")
                        try:
                            thread.start_new_thread(self.kitting_robot_do_adjust,(self.effect_cmd_1,task_dispatcher,) )
                        except:
                            print ("Error: unable to start thread" )
                            self.kitting_robot.robot_info.is_idle =True
                        self.last_cmd_list[0].is_done = True

                elif self.effect_cmd_1.robot_name == "gantry_robot" and self.gantry_robot.robot_info.is_idle:
                    #判断机器人是否禁用
                    if not self.gantry_robot.robot_info.is_enabled:
                        #修改当前有效命令
                        self.cmd_realocation(self.effect_cmd_1,task_dispatcher) # 说明任务需要重新分配
                    else:
                        self.gantry_robot.robot_info.is_idle =False
                        try:
                            thread.start_new_thread(self.gantry_robot_do_adjust,(self.effect_cmd_1,task_dispatcher,) )
                        except:
                            print ("Error: unable to start thread")
                            self.gantry_robot.robot_info.is_idle =True

                        self.last_cmd_list[0].is_done = True                     
                

        if not self.effect_cmd_2.is_done:
            #判断CMD2指令类型
            if self.effect_cmd_2.type =="kitting":
                if self.effect_cmd_2.robot_name == "kitting_robot" and self.kitting_robot.robot_info.is_idle:
                    #判断机器人是否禁用
                    if not self.kitting_robot.robot_info.is_enabled:
                        #修改当前有效命令
                        self.cmd_realocation(self.effect_cmd_2,task_dispatcher) # 说明任务需要重新分配
                         
                    else:

                        self.kitting_robot.robot_info.is_idle =False
                        try:
                            thread.start_new_thread(self.kitting_robot_do_kitting,(self.effect_cmd_2,task_dispatcher,) )
                        except:
                            print ("Error: unable to start thread" )  
                            self.kitting_robot.robot_info.is_idle =True
                                          
                        self.last_cmd_list[1].is_done = True  

                elif self.effect_cmd_2.robot_name == "gantry_robot" and self.gantry_robot.robot_info.is_idle:
                    #判断机器人是否禁用
                    if not self.gantry_robot.robot_info.is_enabled:
                        #修改当前有效命令
                        self.cmd_realocation(self.effect_cmd_2,task_dispatcher) # 说明任务需要重新分配
                    else:
                        self.gantry_robot.robot_info.is_idle =False
                        try:
                            thread.start_new_thread(self.gantry_robot_do_kitting,(self.effect_cmd_2,task_dispatcher,) )
                        except:
                            print ("Error: unable to start thread" )  
                            self.gantry_robot.robot_info.is_idle =True                   
                        self.last_cmd_list[1].is_done = True  

            #判断指令类型
            elif self.effect_cmd_2.type =="assembly":
                if self.effect_cmd_2.robot_name == "gantry_robot" and self.gantry_robot.robot_info.is_idle:
                    self.gantry_robot.robot_info.is_idle =False
                    try:
                        thread.start_new_thread(self.gantry_robot_do_assembly,(self.effect_cmd_2,task_dispatcher,) )
                    except:
                        print ("Error: unable to start thread" ) 
                        self.gantry_robot.robot_info.is_idle =True
                    self.last_cmd_list[1].is_done = True  


            #判断指令类型
            elif self.effect_cmd_2.type =="withdraw":
               
                print ("cm2 receave withdraw Task !!!!!!!!!!!!!!!!")
                
                if self.effect_cmd_2.robot_name == 'kitting_robot' and self.kitting_robot.robot_info.is_idle:

                    #判断机器人是否禁用
                    if not self.kitting_robot.robot_info.is_enabled:
                        #修改当前有效命令
                        self.cmd_realocation(self.effect_cmd_2,task_dispatcher) # 说明任务需要重新分配
                    else:
                        self.kitting_robot.robot_info.is_idle =False
                        try:
                            thread.start_new_thread(self.kitting_robot_do_withdraw,(self.effect_cmd_2,task_dispatcher,) )
                        except:
                            print ("Error: unable to start thread" )
                            self.kitting_robot.robot_info.is_idle =True
                        self.last_cmd_list[1].is_done = True 

                elif self.effect_cmd_2.robot_name == "gantry_robot" and self.gantry_robot.robot_info.is_idle:
                    #判断机器人是否禁用
                    if not self.gantry_robot.robot_info.is_enabled:
                        #修改当前有效命令
                        self.cmd_realocation(self.effect_cmd_2,task_dispatcher) # 说明任务需要重新分配
                    else:
                        self.gantry_robot.robot_info.is_idle =False
                        try:
                            thread.start_new_thread(self.gantry_robot_do_withdraw,(self.effect_cmd_2,task_dispatcher,) )
                        except:
                            print ("Error: unable to start thread" )
                            self.gantry_robot.robot_info.is_idle =True
                        self.last_cmd_list[1].is_done = True                                    
            elif self.effect_cmd_2.type =="adjust":
                print ("cm2 receive adjust task!!!!!!!!!!")
                if self.effect_cmd_2.robot_name == 'kitting_robot' and self.kitting_robot.robot_info.is_idle:

                    #判断机器人是否禁用
                    if not self.kitting_robot.robot_info.is_enabled:
                        #修改当前有效命令
                        self.cmd_realocation(self.effect_cmd_2,task_dispatcher) # 说明任务需要重新分配
                    else:
                        self.kitting_robot.robot_info.is_idle =False
                        print ("kitting robot do withdraw")
                        try:
                            thread.start_new_thread(self.kitting_robot_do_adjust,(self.effect_cmd_2,task_dispatcher,) )
                        except:
                            print ("Error: unable to start thread" )
                            self.kitting_robot.robot_info.is_idle =True
                        self.last_cmd_list[1].is_done = True

                elif self.effect_cmd_2.robot_name == "gantry_robot" and self.gantry_robot.robot_info.is_idle:
                    #判断机器人是否禁用
                    if not self.gantry_robot.robot_info.is_enabled:
                        #修改当前有效命令
                        self.cmd_realocation(self.effect_cmd_2,task_dispatcher) # 说明任务需要重新分配
                    else:
                        self.gantry_robot.robot_info.is_idle =False
                        try:
                            thread.start_new_thread(self.gantry_robot_do_adjust,(self.effect_cmd_2,task_dispatcher,) )
                        except:
                            print ("Error: unable to start thread")
                            self.gantry_robot.robot_info.is_idle =True

                        self.last_cmd_list[1].is_done = True      

            elif self.effect_cmd_2.type == "pick_tray":
                print ("cm2 receive pick_tray task!!!!!!!!!!")
                if self.gantry_robot.robot_info.is_idle:
                    if not self.gantry_robot.robot_info.is_enabled:
                        #修改当前有效命令
                        self.cmd_realocation(self.effect_cmd_2,task_dispatcher) # 说明任务需要重新分配
                    else:
                        self.gantry_robot.robot_info.is_idle = False
                        try:
                            thread.start_new_thread(self.gantry_robot_do_pick_tray,(self.effect_cmd_2,task_dispatcher,) )
                        except:
                            print ("Error: unable to start thread")
                            self.gantry_robot.robot_info.is_idle =True

                        self.last_cmd_list[1].is_done = True                          




        if self.kitting_robot.robot_info.is_idle and self.kitting_robot.robot_info.is_enabled \
            and rospy.get_time()-self.kitting_robot.kitting_robot_idle_time>5:
            # print('self.kitting_robot.kitting_robot_idle_time',self.kitting_robot.kitting_robot_idle_time)
            # print(rospy.get_time())
            # print('rospy.get_time()-self.kitting_robot.kitting_robot_idle_time',rospy.get_time()-self.kitting_robot.kitting_robot_idle_time)
            park_location = self.kitting_robot_nearest_agv_location()
            if self.kitting_robot.robot_info.location!= park_location:
                self.kitting_robot.robot_info.is_idle =False
                self.kitting_move_collision(park_location)
                self.kitting_robot.move_to(park_location)
                self.kitting_robot.robot_info.work_state = "standby"
                self.kitting_robot.robot_info.is_idle =True
                self.kitting_robot.kitting_robot_idle_time = rospy.get_time()
            else:
                self.kitting_move_collision(park_location)
                #self.kitting_robot.move_to(park_location)
                self.kitting_robot.robot_info.work_state = "standby"
                self.kitting_robot.robot_info.is_idle =True
                self.kitting_robot.kitting_robot_idle_time = rospy.get_time()
        
        
        if self.gantry_robot.robot_info.is_idle and self.gantry_robot.robot_info.is_enabled \
            and rospy.get_time()-self.gantry_robot.gantry_robot_idle_time>5:
            # print('self.gantry_robot.gantry_robot_idle_time',self.gantry_robot.gantry_robot_idle_time)
            # print(rospy.get_time())
            # print('rospy.get_time()-self.gantry_robot.gantry_robot_idle_time',rospy.get_time()-self.gantry_robot.gantry_robot_idle_time)
            if self.gantry_robot.robot_info.location != 'gripper_cs':
                self.gantry_robot.robot_info.is_idle =False
                _,path = dijkstra(edges, self.gantry_robot.robot_info.location, "gripper_cs")
                for next_piont in path:
                    self.gantry_robot.move_to(next_piont) 
                    
                self.gantry_robot.robot_info.work_state = "standby"
                self.gantry_robot.robot_info.is_idle =True
                self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
            else:
                self.gantry_robot.robot_info.work_state = "standby"
                self.gantry_robot.robot_info.is_idle =True
                self.gantry_robot.gantry_robot_idle_time = rospy.get_time()


            # if self.kitting_robot_nearest_agv_location!='bin1':
            #     self.kitting_robot.robot_info.is_idle =False
            #     self.kitting_move_collision("bin1")
            #     self.kitting_robot.move_to("bin1")
            #     self.kitting_robot.robot_info.work_state = "standby"
            #     self.kitting_robot.robot_info.is_idle =True
            #     self.kitting_robot.kitting_robot_idle_time = rospy.get_time()
            # if self.kitting_robot_nearest_agv_location!='can':
            #     self.kitting_robot.robot_info.is_idle =False
            #     self.kitting_move_collision("can")
            #     self.kitting_robot.move_to("can")
            #     self.kitting_robot.robot_info.work_state = "standby"
            #     self.kitting_robot.robot_info.is_idle =True
            #     self.kitting_robot.kitting_robot_idle_time = rospy.get_time()


                

    def gantry_move_in_conveyor(self,pick_location = "conveyor"):
        self.gantry_gripeper_check(gripper_type = 'gripper_part') 
        path = None
        work_state = ["picking", "placing", "flipping"]
        kitting_zone=['bin1', 'bin2','bin6','bin5','bin4','bin3','bin7','bin8','agv1_ks1_tray','agv2_ks2_tray','agv3_ks3_tray','agv4_ks4_tray','can']
        first_safe = False
        if self.gantry_robot.robot_info.location in kitting_zone:
            distance = abs(self.gantry_robot.robot_info.position.y - self.kitting_robot.robot_info.position.y) 
            if distance >=2.0:
                first_safe = True
        
        wait_time = 0.0
        while (self.kitting_robot.robot_info.work_state in work_state) and self.kitting_robot.robot_info.is_enabled \
            and not first_safe and not rospy.is_shutdown() and wait_time<MAX_WAIT_TIME:
            print ("waiting kitting:", self.kitting_robot.robot_info.work_state)
            rospy.sleep(0.5) 
            wait_time = wait_time+0.5



        if not self.kitting_robot.robot_info.is_enabled: #kitting 故障了,避免kitting所在的地方
            if self.kitting_robot.robot_info.position.y <= agv_loaction["agv3_ks3_tray"][1]:
                l3,path3 = dijkstra(edges, self.gantry_robot.robot_info.location, "can")
                l4,path4 = dijkstra(edges, self.gantry_robot.robot_info.location, "bin2")
                l5,path5 = dijkstra(edges, self.gantry_robot.robot_info.location, "bin1")
                length = [l3,l4,l5]
                paths = [path3,path4,path5]
                short =length.index(min(length))
                path = paths[short]
            if self.kitting_robot.robot_info.position.y > agv_loaction["agv3_ks3_tray"][1] and \
                self.kitting_robot.robot_info.position.y <= agv_loaction["agv2_ks2_tray"][1] :
                l1,path1 = dijkstra(edges, self.gantry_robot.robot_info.location, "bin5")
                l2,path2 = dijkstra(edges, self.gantry_robot.robot_info.location, "bin6")
                l4,path4 = dijkstra(edges, self.gantry_robot.robot_info.location, "bin2")
                l5,path5 = dijkstra(edges, self.gantry_robot.robot_info.location, "bin1")
                length = [l1,l2,l4,l5]
                paths = [path1,path2,path4,path5]
                short =length.index(min(length))
                path = paths[short]

            if self.kitting_robot.robot_info.position.y >= agv_loaction["agv2_ks2_tray"][1]:
                l1,path1 = dijkstra(edges, self.gantry_robot.robot_info.location, "bin5")
                l2,path2 = dijkstra(edges, self.gantry_robot.robot_info.location, "bin6")
                l3,path3 = dijkstra(edges, self.gantry_robot.robot_info.location, "can")
                length = [l1,l2,l3]
                paths = [path1,path2,path3]
                short =length.index(min(length))
                path = paths[short]
        else: #kitting 没有发生故障

            l1,path1 = dijkstra(edges, self.gantry_robot.robot_info.location, "bin5")
            l2,path2 = dijkstra(edges, self.gantry_robot.robot_info.location, "bin6")
            l3,path3 = dijkstra(edges, self.gantry_robot.robot_info.location, "can")
            l4,path4 = dijkstra(edges, self.gantry_robot.robot_info.location, "bin2")
            l5,path5 = dijkstra(edges, self.gantry_robot.robot_info.location, "bin1")
            length = [l1,l2,l3,l4,l5]
            paths = [path1,path2,path3,path4,path5]
            short =length.index(min(length))
            path = paths[short]

        for next_piont in path:
            self.gantry_robot.move_to(next_piont)
        self.gantry_robot.robot_info.location = path[-1]
        self.gantry_robot.robot_info.next_park_location = "conveyor"





        #判断kitting是否在要经过的点附件做事情
        path_safe_flag = False
        wait_time = 0.0
        inner_wait_time =0.0
        while not path_safe_flag and self.kitting_robot.robot_info.is_enabled and not rospy.is_shutdown() and wait_time<MAX_WAIT_TIME:

            while self.kitting_robot.robot_info.work_state == "moving" and self.kitting_robot.robot_info.is_enabled and not rospy.is_shutdown()\
                 and inner_wait_time<MAX_WAIT_TIME:


                gantry_y = kitting_robot_park_location[self.gantry_robot.robot_info.location][1]

                if self.kitting_robot.robot_info.location == "conveyor":
                    next_kitting_y = self.kitting_robot.kitting_base_y
                    kitting_y = self.kitting_robot.kitting_base_y
                else:      
                    next_kitting_y = kitting_robot_park_location[self.kitting_robot.robot_info.next_park_location][1]
                    kitting_y = kitting_robot_park_location[self.kitting_robot.robot_info.location][1]
                min_y = min(next_kitting_y,kitting_y)
                max_y = max(next_kitting_y,kitting_y)

                if gantry_y<=min_y or gantry_y>=max_y:
                    path_safe_flag = True

                    self.gantry_robot.move_to(pick_location)
                    self.gantry_robot.robot_info.location ="conveyor"
                    break
                
                # if self.kitting_robot.robot_info.position.y > self.gantry_robot.robot_info.position.y+1.3: # safe
                #     path_safe_flag = True
                #     self.gantry_robot.move_to(pick_location)
                #     break
                # elif self.kitting_robot.robot_info.position.y < self.gantry_robot.robot_info.position.y-1.30: # safe
                #     path_safe_flag = True
                #     self.gantry_robot.move_to(pick_location)
                #     break

                print (" gantry waiting enter conveyor")
                rospy.sleep(0.5)
                inner_wait_time = inner_wait_time+0.5
            
            if self.kitting_robot.robot_info.position.y > self.gantry_robot.robot_info.position.y+0.05: # safe
                path_safe_flag = True
                self.gantry_robot.move_to(pick_location)
                self.gantry_robot.robot_info.location ="conveyor"

            elif self.kitting_robot.robot_info.position.y < self.gantry_robot.robot_info.position.y-1.10: # safe
                path_safe_flag = True
                self.gantry_robot.move_to(pick_location)
                self.gantry_robot.robot_info.location ="conveyor"

            else:
                print ("wait kitting for moving! ")
                rospy.sleep(0.5)
                wait_time = wait_time+0.5

        if not path_safe_flag: #说明在等待过程中，kitting robot 故障了，重新规划
            if self.kitting_robot.robot_info.position.y <= agv_loaction["agv3_ks3_tray"][1]:
                l3,path3 = dijkstra(edges, self.gantry_robot.robot_info.location, "can")
                l4,path4 = dijkstra(edges, self.gantry_robot.robot_info.location, "bin2")
                l5,path5 = dijkstra(edges, self.gantry_robot.robot_info.location, "bin1")
                length = [l3,l4,l5]
                paths = [path3,path4,path5]
                short =length.index(min(length))
                path = paths[short]
            if self.kitting_robot.robot_info.position.y > agv_loaction["agv3_ks3_tray"][1] and \
                self.kitting_robot.robot_info.position.y <= agv_loaction["agv2_ks2_tray"][1] :
                l1,path1 = dijkstra(edges, self.gantry_robot.robot_info.location, "bin5")
                l2,path2 = dijkstra(edges, self.gantry_robot.robot_info.location, "bin6")
                l4,path4 = dijkstra(edges, self.gantry_robot.robot_info.location, "bin2")
                l5,path5 = dijkstra(edges, self.gantry_robot.robot_info.location, "bin1")
                length = [l1,l2,l4,l5]
                paths = [path1,path2,path4,path5]
                short =length.index(min(length))
                path = paths[short]

            if self.kitting_robot.robot_info.position.y >= agv_loaction["agv2_ks2_tray"][1]:
                l1,path1 = dijkstra(edges, self.gantry_robot.robot_info.location, "bin5")
                l2,path2 = dijkstra(edges, self.gantry_robot.robot_info.location, "bin6")
                l3,path3 = dijkstra(edges, self.gantry_robot.robot_info.location, "can")
                length = [l1,l2,l3]
                paths = [path1,path2,path3]
                short =length.index(min(length))
                path = paths[short]
            
            for next_piont in path:
                self.gantry_robot.move_to(next_piont,eps=0.05)
            self.gantry_robot.robot_info.location = path[-1]
            self.gantry_robot.robot_info.next_park_location = "conveyor"    
            self.gantry_robot.move_to("conveyor")
            self.gantry_robot.robot_info.location ="conveyor"


    def gantry_move_out_conveyor(self):
        best_park_location = None
        wait_time = 0.0
        while (self.kitting_robot.robot_info.work_state=="flipping") and self.kitting_robot.robot_info.is_enabled \
            and not rospy.is_shutdown() and wait_time< MAX_WAIT_TIME:
            rospy.sleep(0.5) 
            wait_time = wait_time+0.5


            
        if not self.kitting_robot.robot_info.is_enabled:
            if self.kitting_robot.robot_info.location =="agv4_ks4_tray":
                l1 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["can"][1])
                l2 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin1"][1])
                l3 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin2"][1])
                l4 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin6"][1])
                length = [l1,l2,l3,l4]
                locations= ["can","bin1","bin2","bin6"]
                short =length.index(min(length))
                best_park_location = locations[short]
            elif self.kitting_robot.robot_info.location =="agv3_ks3_tray":

                l1 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin5"][1])
                l2 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin1"][1])
                l3 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin2"][1])
                length = [l1,l2,l3]
                locations= ["bin5","bin1","bin2"]
                short =length.index(min(length))
                best_park_location = locations[short]
    
            elif self.kitting_robot.robot_info.location =="agv2_ks2_tray":    
               
                l4 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin5"][1])
                l5 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin6"][1])
                l6 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["can"][1])
                length = [l4,l5,l6]
                locations= ["bin5","bin6","can"]
                short =length.index(min(length))
                best_park_location = locations[short]

            elif self.kitting_robot.robot_info.location =="can":    
                l2 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin1"][1])
                l3 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin2"][1])
                l4 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin5"][1])
                l5 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin6"][1])
                length = [l2,l3,l4,l5]
                locations= ["bin1","bin2","bin5","bin6"]
                short =length.index(min(length))
                best_park_location = locations[short]

            elif self.kitting_robot.robot_info.location =="agv1_ks1_tray":    
                l2 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["can"][1])
                l3 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin2"][1])
                l4 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin5"][1])
                l5 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin6"][1])
                length = [l2,l3,l4,l5]
                locations= ["can","bin2","bin5","bin6"]
                short =length.index(min(length))
                best_park_location = locations[short]
            else:
                l1 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["can"][1])
                l2 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin1"][1])
                l3 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin2"][1])
                l4 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin5"][1])
                l5 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin6"][1])
                length = [l1,l2,l3,l4,l5]
                locations= ["can","bin1","bin2","bin5","bin6"]
                short =length.index(min(length))
                best_park_location = locations[short]
        else:#kitting机器人没有故障
            # print "self.kitting_robot.robot_info.is_enabled",self.kitting_robot.robot_info.is_enabled
            l1 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["can"][1])
            l2 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin1"][1])
            l3 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin2"][1])
            l4 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin5"][1])
            l5 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin6"][1])
            length = [l1,l2,l3,l4,l5]
            locations= ["can","bin1","bin2","bin5","bin6"]
            short =length.index(min(length))
            best_park_location = locations[short]

        ################准备移动##################
        self.gantry_robot.robot_info.next_park_location = best_park_location
        path_safe_flag = False
        wait_time = 0.0
        while not path_safe_flag and self.kitting_robot.robot_info.is_enabled and not rospy.is_shutdown() and wait_time<MAX_WAIT_TIME:
            if self.kitting_robot.robot_info.work_state!="moving":
                if best_park_location == "bin5":
                    if self.kitting_robot.robot_info.location in bin5_safe_zone:  
                        path_safe_flag = True
                    else:
                        print ("waiting for kitting robot moving")
                elif best_park_location =="bin6":
                    if self.kitting_robot.robot_info.location in bin6_safe_zone:  
                        path_safe_flag = True
                    else:
                        print ("waiting for kitting robot moving")
                elif best_park_location == "can":
                    if self.kitting_robot.robot_info.location in can_safe_zone: 
                        path_safe_flag = True
                    else:
                        print ("waiting for kitting robot moving")
                elif best_park_location == "bin2":
                    if self.kitting_robot.robot_info.location in bin2_safe_zone: 
                        path_safe_flag = True
                    else:
                        print ("waiting for kitting robot moving")
                elif best_park_location == "bin1":
                    if self.kitting_robot.robot_info.location in bin1_safe_zone: 
                        path_safe_flag = True
                    else:
                        print ("waiting for kitting robot moving")
                else:
                    pass
            rospy.sleep(0.1)
            wait_time=wait_time+0.1

        if not path_safe_flag:# 说明kitting机器人故障了

            if self.kitting_robot.robot_info.location =="agv4_ks4_tray":
                l1 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["can"][1])
                l2 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin1"][1])
                l3 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin2"][1])
                l4 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin6"][1])
                length = [l1,l2,l3,l4]
                locations= ["can","bin1","bin2","bin6"]
                short =length.index(min(length))
                best_park_location = locations[short]
            elif self.kitting_robot.robot_info.location =="agv3_ks3_tray":

                l1 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin5"][1])
                l2 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin1"][1])
                l3 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin2"][1])
                length = [l1,l2,l3]
                locations= ["bin5","bin1","bin2"]
                short =length.index(min(length))
                best_park_location = locations[short]
    
            elif self.kitting_robot.robot_info.location =="agv2_ks2_tray":    
                l3 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin2"][1])
                l4 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin5"][1])
                l5 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin6"][1])
                length = [l3,l4,l5]
                locations= ["bin2","bin5","bin6"]
                short =length.index(min(length))
                best_park_location = locations[short]
            elif self.kitting_robot.robot_info.location =="can":    
                l3 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin2"][1])
                l4 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin5"][1])
                l5 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin6"][1])
                length = [l3,l4,l5]
                locations= ["bin2","bin5","bin6"]
                short =length.index(min(length))
                best_park_location = locations[short]
            elif self.kitting_robot.robot_info.location =="agv1_ks1_tray":    
                l2 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["can"][1])
                l3 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin2"][1])
                l4 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin5"][1])
                l5 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin6"][1])
                length = [l2,l3,l4,l5]
                locations= ["can","bin2","bin5","bin6"]
                short =length.index(min(length))
                best_park_location = locations[short]
            else:
                l1 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["can"][1])
                l2 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin1"][1])
                l3 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin2"][1])
                l4 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin5"][1])
                l5 = abs(self.gantry_robot.robot_info.position.y-gantry_robot_park_location["bin6"][1])
                length = [l1,l2,l3,l4,l5]
                locations= ["can","bin1","bin2","bin5","bin6"]
                short =length.index(min(length))
                best_park_location = locations[short]
    
        return best_park_location



    def gantry_pick_place_collison(self,gantry_location):
        bin12 = ["bin1","bin2"]
        bin56 = ["bin5","bin6"]
        danger_zone = ['bin1', 'bin2','bin6','bin5','agv1_ks1_tray','agv2_ks2_tray','agv3_ks3_tray','agv4_ks4_tray']
        work_state = ["picking", "placing", "flipping"]

        # print "gantry_pick_place_collison is checking!"
        is_safty = False
        # self.gantry_robot.robot_info.work_state = "placing"
        while self.kitting_robot.robot_info.location == gantry_location \
            and (self.kitting_robot.robot_info.work_state in work_state) and \
            not rospy.is_shutdown() and self.kitting_robot.robot_info.is_enabled and\
            self.gantry_robot.robot_info.is_enabled:
            # print "waiting for kitting:", self.kitting_robot.robot_info.work_state
            is_safty = True
            rospy.sleep(0.1)

        while (self.kitting_robot.robot_info.location == "bin2" and gantry_location =="bin1") \
            and (self.kitting_robot.robot_info.work_state in work_state) and \
            not rospy.is_shutdown() and self.kitting_robot.robot_info.is_enabled and\
            self.gantry_robot.robot_info.is_enabled:
            # print "waiting for kitting:", self.kitting_robot.robot_info.work_state
            is_safty = True
            rospy.sleep(0.1)

        while (self.kitting_robot.robot_info.location == "bin5" and gantry_location =="bin6") \
            and (self.kitting_robot.robot_info.work_state in work_state) and \
            not rospy.is_shutdown() and self.kitting_robot.robot_info.is_enabled and\
            self.gantry_robot.robot_info.is_enabled:
            # print "waiting for kitting:", self.kitting_robot.robot_info.work_state
            is_safty = True
            rospy.sleep(0.1)
        # lhr加的 ，当kitting移动时，gantry需要再箱子12上翻转零件，这时候需要等待
        while (self.kitting_robot.robot_info.work_state == "moving") and (gantry_location in (bin12 or bin56)) \
            and self.gantry_robot.robot_info.work_state == "flipping":
            next_kitting_y = kitting_robot_park_location[self.kitting_robot.robot_info.next_park_location][1]
            kitting_y = kitting_robot_park_location[self.kitting_robot.robot_info.location][1]
            gantry_y = kitting_robot_park_location[gantry_location][1]
            min_y = min(next_kitting_y,kitting_y)
            max_y = max(next_kitting_y,kitting_y)
            while gantry_y>=min_y and gantry_y<=max_y:
                    rospy.sleep(0.01)
                    print('waitting kitting moving!!!')
                    if gantry_y <= kitting_robot_park_location[self.kitting_robot.robot_info.location][1]:
                        break
                    else:
                        pass
            if gantry_y <= kitting_robot_park_location[self.kitting_robot.robot_info.location][1]:
                is_safty = True
                break

        if self.kitting_robot.robot_info.is_alive and not self.kitting_robot.robot_info.is_enabled:
            self.kitting_robot.robot_info.is_alive = False
            rospy.sleep(2)
 
        

        if self.kitting_robot.robot_info.next_park_location ==gantry_location and \
            self.kitting_robot.robot_info.work_state =="moving":
            is_safty = True
            rospy.sleep(0.1)


        if (self.kitting_robot.robot_info.location ==gantry_location or \
             (self.kitting_robot.robot_info.location in bin12 and gantry_location in bin12)\
            or(self.kitting_robot.robot_info.location in bin56 and gantry_location in bin56)):
            is_safty = True

        if gantry_location in danger_zone and self.kitting_robot.robot_info.work_state =="moving":
            gantry_y = kitting_robot_park_location[gantry_location][1]
            if self.kitting_robot.robot_info.location == "conveyor":
                next_kitting_y = self.kitting_robot.kitting_base_y
                kitting_y = self.kitting_robot.kitting_base_y
            else:      
                next_kitting_y = kitting_robot_park_location[self.kitting_robot.robot_info.next_park_location][1]
                kitting_y = kitting_robot_park_location[self.kitting_robot.robot_info.location][1]
            min_y = min(next_kitting_y,kitting_y)
            max_y = max(next_kitting_y,kitting_y)
            if gantry_y>=min_y and gantry_y<=max_y:
                    is_safty = True
            # if not target_part.is_flip:
            #     if gantry_y>=min_y and gantry_y<=max_y:
            #         is_safty = True
            # else:
            #     while gantry_y>=min_y and gantry_y<=max_y:###lhr加的 原来的上面两行错误
            #         rospy.sleep(0.05)
            #         print('waitting kitting moving!!!')
            #         if gantry_y <= kitting_robot_park_location[self.kitting_robot.robot_info.location][1]:
            #             break
            #         else:
            #             pass
            #     if gantry_y <= kitting_robot_park_location[self.kitting_robot.robot_info.location][1]:
            #         is_safty = True
            # print('is_safty',is_safty)
        rospy.sleep(1)
 
        return is_safty

    def gantry_robot_do_kitting(self,cmd,task_dispatcher):
        #机器人状态更新 记得加锁
        print("gantry_robot is do kitting")
        self.threadLock.acquire()
        self.gantry_robot.robot_info.is_idle = False
        self.threadLock.release()

        #命令解析
        pick_part_on = cmd.pick_part_on
        target_part = cmd.pick_part
        target_agv_tray = cmd.target_position
        print("gantry do kitting part:",target_part.i_d)
        print("target agv :", target_agv_tray)
        print("pick location:", pick_part_on)
        if pick_part_on == None:
            print('cant search part')
            self.gantry_robot.robot_info.is_idle = True
            self.gantry_robot.robot_info.work_state = "standby"
            self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
            self.cmd_realocation(cmd,task_dispatcher)
            return False
        check_location = None 
        check_agv = None
        #转换成驻点地图
        pick_location = pick_part_on
        place_location = target_agv_tray
        path = []

        self.gantry_check_has_been_on_agv(target_agv_tray,target_part)

        #动作标志位
        pick_success_flag = False
        quality_check_flag = False
        place_success_flag = False   
        check_result = False # faulty part = False
        #做一次路径规划到达要pick零件的驻点
        if pick_location =="conveyor":
            self.gantry_gripeper_check(gripper_type = 'gripper_part') 
            #寻找最佳进入点
            if self.gantry_robot.robot_info.location!="conveyor":
                self.gantry_move_in_conveyor(pick_location)
        else:
            if self.gantry_robot.robot_info.location =="conveyor":#先移动到最近的驻点
                best_park_location = self.gantry_move_out_conveyor()
                self.gantry_robot.robot_info.next_park_location = best_park_location
                if self.gantry_robot_is_enabled(cmd,task_dispatcher):
                    self.gantry_robot.move_to(best_park_location)
                else:
                    return False
            
            self.gantry_gripeper_check(gripper_type = 'gripper_part') 
            _,path = dijkstra(edges, self.gantry_robot.robot_info.location, pick_location)
            for next_piont in path:
                if self.gantry_robot_is_enabled(cmd,task_dispatcher):
                    self.gantry_robot.move_to(next_piont,eps=0.05)
                else:
                    return False
            if self.gantry_robot_is_enabled(cmd,task_dispatcher):        
                self.gantry_robot.move_to(pick_location)
            else:
                return False    

        #已经到达pick的驻点

        ############################ pick 阶段 ###############################
        # 是否在传送带上抓取
        if pick_location =="conveyor":
           #1.1先抓起来
           # 抓之前，先判断kitting是否在conveyor上做抓取
            self.gantry_pick_place_collison(pick_location)
            attempts = 0
            MAX_ATTEMPTS = 3
            while not pick_success_flag and attempts<= MAX_ATTEMPTS and not rospy.is_shutdown():
                attempts = attempts+1
                part_on_conveyor = self.sensor_system.search_part_on_conveyor(target_part.type)
                # print "part_on_conveyor number:", len(part_on_conveyor)
                min_distance = 1000
                it = 0
                if part_on_conveyor: #选择距离最近的,并且可以抓的
                    #可以抓的
                    useful_parts =[]
                    for part in part_on_conveyor:
                        #计算抓取距离
                        # =计算最佳的抓取时间
                        current_time = rospy.get_time()
                        target_part_time = part.time_stamp
                        target_part_y = part.pose.position.y
                        c_y = -(current_time - target_part_time)*conveyor_vel+target_part_y
                        if c_y <= self.gantry_robot.gantry_base_y:
                            t_best = (self.gantry_robot.gantry_base_y- c_y)/(gantry_velocity - conveyor_vel)  
                        else:
                            t_best = (c_y - self.gantry_robot.gantry_base_y)/(gantry_velocity + conveyor_vel) 
                        f_y = c_y - t_best*conveyor_vel
                        if f_y >= conveyor_end+0.1:#说明可以抓
                            useful_parts.append(part)


                    for i in range (0, len(useful_parts)):
                        dis = abs(self.gantry_robot.gantry_base_y-(useful_parts[i].pose.position.y- \
                            (rospy.get_time()-useful_parts[i].time_stamp)*conveyor_vel))
                        if dis <= min_distance:
                            min_distance = dis
                            it = i
                    # print "it:", it
                    if len(useful_parts)>0:
                        target_part_on_con = useful_parts[it]
                        if self.gantry_robot_is_enabled(cmd,task_dispatcher):
                            pick_success_flag = self.gantry_robot.pick_part_on_conveyor(target_part_on_con)
                        else:
                            return
                    else:
                        print('cant search part')
                        self.gantry_robot.robot_info.work_state = "standby"
                        self.gantry_robot.robot_info.is_idle = True
                        self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
                        self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配
                        return False
                else:
                    print('cant search part')
                    self.gantry_robot.robot_info.work_state = "standby"
                    self.gantry_robot.robot_info.is_idle = True
                    self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
                    self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配
                    return False

            #如果尝试3次失败之后，就反馈给任务分配系统
            if not pick_success_flag:
                print('pick error')
                self.gantry_robot.robot_info.work_state = "standby"
                self.gantry_robot.robot_info.is_idle = True
                self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
                self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配
                return False   
            ##############################先回到驻点######################################
            #寻找最佳点
            best_park_location = self.gantry_move_out_conveyor()
            self.gantry_robot.robot_info.next_park_location = best_park_location
            if self.gantry_robot_is_enabled(cmd,task_dispatcher):
                self.gantry_robot.move_to(best_park_location)
            else:
                return False

            # 1.2 判断是否需要flip
            if pick_success_flag and target_part.is_flip: #如果需要，先放到一个空箱子上，或者可以放的位置
                #先找一个位置
                place_part_on = None
                flip_places =[]

                for bin in bins:
                    if not task_dispatcher.parts_on_warehouse_location[bin]:
                        flip_places.append(bin)
                if len(flip_places)>0:
                    place_part_on = self.gantry_robot_opt_short_path(self.gantry_robot.robot_info.position,flip_places,target_agv_tray)
                # min_place_distance = 99999.0        
                # for place in flip_places:
                #     x_d = abs(gantry_robot_park_location[place][0]-self.gantry_robot.robot_info.position.x)
                #     y_d = abs(gantry_robot_park_location[place][1]-self.gantry_robot.robot_info.position.y)
                #     dis = x_d+y_d
                #     if min_place_distance>dis:
                #         min_place_distance =dis
                #         place_part_on = place


                if place_part_on == None:    
                    for bin in bins:
                        if len(task_dispatcher.parts_on_warehouse_location[bin])<4:
                            flip_places.append(bin)     
                    if len(flip_places)>0:
                        place_part_on = self.gantry_robot_opt_short_path(self.gantry_robot.robot_info.position,flip_places,target_agv_tray)


                if place_part_on == None:#如果找不到，就不flip了
                    pass
                else:
                    #找到了，确定放置姿态
                    self.gantry_robot.robot_info.work_state = "flipping"
                    tmp_part = copy.deepcopy(target_part)
                    tmp_part.pose =  self.withdraw_place_position(place_part_on,target_part.type)
                   
                    _,path = dijkstra(edges, self.gantry_robot.robot_info.location, place_part_on)
                    for next_piont in path:
                        if self.gantry_robot_is_enabled(cmd,task_dispatcher):
                            self.gantry_robot.move_to(next_piont)
                        else:
                            return False
                    ##################判断可不可以放################
                    # while self.kitting_robot.robot_info.location == place_part_on and not rospy.is_shutdown() and \
                    #     self.kitting_robot.robot_info.work_state != "standby" and self.kitting_robot.robot_info.is_enabled:
                    #     print "waiting..."
                    #     rospy.sleep(0.5)
                    state = self.gantry_pick_place_collison(place_part_on)
                    # if state:
                    #     self.place_part_on_bin_agv(place_part_on,tmp_part)
                    # else:
                    if self.gantry_robot_is_enabled(cmd,task_dispatcher):
                        self.gantry_robot.place_part_on_bin_agv(place_part_on,tmp_part)
                    else:
                        return False
                    ##################################
                    #此时需要做flip#
                    if self.sensor_system.is_alive():
                        new_part = self.sensor_system.new_part_dict[place_part_on]
                        new_part = self.sensor_system.search_part_use_part(new_part)
                        tmp_part = new_part
                    else:
                        new_part = tmp_part

                    flip_flag = False
                    attempts = 0
                    MAX_ATTEMPTS = 3
                    while not flip_flag and attempts<= MAX_ATTEMPTS and not rospy.is_shutdown():
                        if self.gantry_robot_is_enabled(cmd,task_dispatcher):
                            self.gantry_robot.move_to(place_part_on)  
                        else:
                            return False
                        attempts = attempts+1

                        self.gantry_pick_place_collison(place_part_on)


                        if self.gantry_robot_is_enabled(cmd,task_dispatcher):
                            flip_flag = self.gantry_robot.flip(place_part_on,new_part)
                        else:
                            return False


                    if not flip_flag:
                        print('flip error')
                        self.gantry_robot.robot_info.work_state = "standby"
                        self.gantry_robot.robot_info.is_idle = True
                        self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
                        self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配
                        return False

                    else: # flip 成功之后重新抓取
                        if self.sensor_system.is_alive():
                            new_part  = self.sensor_system.new_part_dict[place_part_on]
                            new_part = self.sensor_system.search_part_use_part(new_part)
                        else:
                            new_part  = part_pose_after_flip(tmp_part)

                        repick_flag = False
                        attempts = 0 
                        MAX_ATTEMPTS = 3
                        if self.gantry_robot_is_enabled(cmd,task_dispatcher):
                            self.gantry_robot.move_to(place_part_on,flip=True)
                        else:
                            return False
                        #self.gantry_robot.move_to(place_part_on)
                        while not repick_flag and attempts < MAX_ATTEMPTS and not rospy.is_shutdown():
                            attempts = attempts+1
                            if self.sensor_system.is_alive():
                                new_part  = self.sensor_system.new_part_dict[place_part_on]
                                new_part = self.sensor_system.search_part_use_part(new_part)
                            else:
                                new_part  = part_pose_after_flip(tmp_part)

                            if self.gantry_robot_is_enabled(cmd,task_dispatcher):
                                repick_flag = self.gantry_robot.pick_part_on_bin_agv(place_part_on,new_part)
                            else:
                                return False

                        self.gantry_robot.robot_info.work_state = "standby"

                        if not repick_flag:
                            print('pick error')
                            self.gantry_robot.robot_info.is_idle = True
                            self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
                            self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配
                            return False  

        #不在传送带上抓取
        else:
            #1.3 先判断是否需要flip
            if target_part.is_flip: 
                print ('part need flip')
                pick_success_flag = False
                flip_flag = False 
                attempts = 0
                MAX_ATTEMPTS = 3
                target_part_on_bin_agv = None

                #先判断能不能抓
                ##################判断可不可以放################
                # while self.kitting_robot.robot_info.location == place_part_on and not rospy.is_shutdown() and \
                #     self.kitting_robot.robot_info.work_state != "standby" and self.kitting_robot.robot_info.is_enabled:
                #     print "waiting..."
                #     rospy.sleep(0.5)
                self.gantry_pick_place_collison(pick_location)       
                self.gantry_robot.robot_info.work_state = "flipping"
                while not flip_flag and attempts<= MAX_ATTEMPTS and not rospy.is_shutdown():
                    attempts = attempts+1
                    # self.gantry_robot.move_to(pick_location)  
                    target_part_list =  self.sensor_system.search_part_location_type(pick_location, target_part.type)
                    if target_part_list:
                        target_part_on_bin_agv = target_part_list[0]

                        self.gantry_pick_place_collison(pick_location)

                        if self.gantry_robot_is_enabled(cmd,task_dispatcher):
                            flip_flag = self.gantry_robot.flip(pick_location, target_part_on_bin_agv)
                        else:
                            return False
                        #print "self.gantry_robot.robot_info.work_state",self.gantry_robot.robot_info.work_state
                    else:
                        print('cant search part')
                        self.gantry_robot.robot_info.is_idle = True
                        self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
                        self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配                   
                        return False 
                     
                #如果尝试3次失败之后，就反馈给任务分配系统
                if not flip_flag:
                    print('flip error')
                    self.gantry_robot.robot_info.work_state = "standby"   
                    self.gantry_robot.robot_info.is_idle = True
                    self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
                    self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配
                    return False 
                else:
                    #重新抓取
                    print ("repick")
                    #print "self.gantry_robot.robot_info.work_state",self.gantry_robot.robot_info.work_state
                    if self.sensor_system.is_alive():
                        new_part  = self.sensor_system.new_part_dict[pick_location]
                        new_part = self.sensor_system.search_part_use_part(new_part)
                    else:
                        new_part  = part_pose_after_flip(target_part_on_bin_agv)

                    pick_success_flag = False
                    attempts = 0 
                    MAX_ATTEMPTS = 3
                    if self.gantry_robot_is_enabled(cmd,task_dispatcher):
                        self.gantry_robot.move_to(pick_location,flip=True)
                    else:
                        return False
                    
                    while not pick_success_flag and attempts < MAX_ATTEMPTS and not rospy.is_shutdown():
                        attempts = attempts+1
                        print ('repicking ...')
                        if self.gantry_robot_is_enabled(cmd,task_dispatcher):
                            pick_success_flag = self.gantry_robot.pick_part_on_bin_agv(pick_location,new_part)
                        else:
                            return False
                        #print "self.gantry_robot.robot_info.work_state",self.gantry_robot.robot_info.work_state
                        
                    self.gantry_robot.robot_info.work_state = "standby"   
                    if not pick_success_flag:
                        print('pick error')
                        self.gantry_robot.robot_info.is_idle = True
                        self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
                        self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配
                        return False 
                    else:
                        pass 
            else:
                                
                ##################判断可不可以放################
                # while self.kitting_robot.robot_info.location == pick_location and not rospy.is_shutdown() and \
                #     self.kitting_robot.robot_info.work_state != "standby" and self.kitting_robot.robot_info.is_enabled:
                #     print "waiting..."
                #     rospy.sleep(0.5)
                state = self.gantry_pick_place_collison(pick_location)
                attempts = 0
                MAX_ATTEMPTS = 3
                pick_success_flag = False
                self.gantry_robot.robot_info.work_state = "picking" 
                while not pick_success_flag and attempts<= MAX_ATTEMPTS and not rospy.is_shutdown():
                    attempts = attempts+1
                    target_part_list =  self.sensor_system.search_part_location_type(pick_location, target_part.type)
                    if target_part_list:
                        target_part_on_bin_agv = target_part_list[0]
                        # if state:
                        #     pick_success_flag = self.S_pick_part_on_bin_agv(pick_location,target_part_on_bin_agv)
                        # else:
                        if self.gantry_robot_is_enabled(cmd,task_dispatcher):
                            pick_success_flag = self.gantry_robot.pick_part_on_bin_agv(pick_location,target_part_on_bin_agv)
                        else:
                            return False
                    else:
                        print('cant search part')
                        self.gantry_robot.robot_info.is_idle = True
                        self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
                        self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配                   
                        return False 
                self.gantry_robot.robot_info.work_state = "standby" 
            #如果尝试3次失败之后，就反馈给任务分配系统
            if not pick_success_flag: 
                print('pick error')
                self.gantry_robot.robot_info.work_state = "standby" 
                self.gantry_robot.robot_info.is_idle = True
                self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
                self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配
                return False    

        ############################ place 阶段 ###############################
        r1,r2 = False,False
        if pick_success_flag:
            #先判断agv的位置是否在ks位置
            if 'ks' in target_agv_tray:
                check_location = place_location
                check_agv = target_agv_tray
            else:
                #不会发生
                pass
            #     check_location, check_agv = self.find_best_check_location(self.gantry_robot.robot_info.location)
            # if check_location == None:
            #    quality_check_flag = True
            #    check_result = True
            # else:
            _,path = dijkstra(edges, self.gantry_robot.robot_info.location, check_location)
            for next_piont in path:
                if self.gantry_robot_is_enabled(cmd,task_dispatcher):
                    self.gantry_robot.move_to(next_piont)
                else:
                    return False

            self.gantry_robot.robot_info.location = check_location
            ##################判断可不可以放################
            # rospy.sleep(1000)
            self.gantry_robot.robot_info.work_state = "placing"
            state = self.gantry_pick_place_collison(check_location)

            # self.gantry_robot.robot_info.work_state = "placing"
            # if state:
            #     r1,r2 = self.gantry_robot.check_part(check_location, target_part)
            # else:
            if self.gantry_robot_is_enabled(cmd,task_dispatcher):
                r1,r2 = self.gantry_robot.check_part(check_location, target_part) 
            else:
                return False  
            #1 1 ---检查完毕，零件放到agv上
            #1 0 ---零件是坏的，已经拿起来了  
            #0 0 ----传感器失效，不知道掉落在哪里了,也不知道是不是坏的零件  
            self.gantry_robot.robot_info.work_state = "standby"
            self.gantry_robot.robot_info.location = check_location
        if r1 and r2:
            if 'ks' in target_agv_tray: 
                print("gantry do kitting :cmd is finished!")
                self.cmd_feed_back(cmd, task_dispatcher) 
                rospy.sleep(0.05)
                self.threadLock.acquire()
                cmd.is_done = True
                self.gantry_robot.robot_info.is_idle = True 
                self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
                self.threadLock.release()
                return True 
            # else:
            #     # 官方声明不可能发生
            #     print("cmd is finished!")
            #     cmd.is_done = True
            #     self.threadLock.acquire()
            #     self.gantry_robot.robot_info.is_idle =True
            #     self.cmd_feed_back(cmd, task_dispatcher)
            #     self.threadLock.release()
            #     return True
        
        if r1 and not r2:
            # self.gantry_robot.robot_info.is_idle = True
            # self.gantry_robot.robot_info.work_state = "standby"
            
            self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配  
            _,path = dijkstra(edges, self.gantry_robot.robot_info.location, "can")
            for next_piont in path:
                if self.gantry_robot_is_enabled(cmd,task_dispatcher):
                    self.gantry_robot.move_to(next_piont) 
                else:
                    return False

            ##################判断可不可以放################
            # while self.kitting_robot.robot_info.location == "can" and not rospy.is_shutdown() and \
            #     self.kitting_robot.robot_info.work_state != "standby" and self.kitting_robot.robot_info.is_enabled:
            #     print "waiting..."
            #     rospy.sleep(0.5)   
            self.gantry_pick_place_collison("can")
            if self.gantry_robot_is_enabled(cmd,task_dispatcher):
                self.gantry_robot.place_part_on_can()
            else:
                return False
            self.threadLock.acquire()
            self.gantry_robot.robot_info.is_idle = True 
            self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
            self.threadLock.release()
            # self.gantry_robot.robot_info.is_idle = True
            # self.gantry_robot.robot_info.work_state = "standby"
            # self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配  
            return False

        if not r1 and not r2:
            print("gantry not do kitting :cmd is finished!")
            self.gantry_robot.robot_info.work_state = "standby"
            self.cmd_feed_back(cmd, task_dispatcher)
            rospy.sleep(0.05)
            self.threadLock.acquire()
            self.gantry_robot.robot_info.is_idle =True
            self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
            cmd.is_done = True
            self.threadLock.release()
            #self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配  
            return False

    def gantry_robot_do_assembly(self, cmd, task_dispatcher):
        #机器人状态更新 记得加锁
        print("gantry_robot is do assembly")
        
        self.threadLock.acquire()
        self.gantry_robot.robot_info.is_idle =False
        self.threadLock.release()

        #命令解析
        pick_part_on = cmd.pick_part_on
        target_part = cmd.pick_part
        station_id = cmd.target_position  # station_id

        print ("gantry assmble", target_part.type)
        
        check_location = None 
        check_agv = None
        #转换成驻点地图
        pick_location =pick_part_on
        place_location = station_id
        path = []

        #动作标志位
        pick_success_flag = False
        quality_check_flag = False
        place_success_flag = False   
        check_result = False # faulty part = False
        ##############做一次路径规划到达要pick零件的驻点#########################
        #先判断再哪里抓
        if pick_location == "conveyor":
            #寻找最佳进入点
            if self.gantry_robot.robot_info.location!="conveyor":
                self.gantry_move_in_conveyor(pick_location)
        else:
            if self.gantry_robot.robot_info.location =="conveyor":#先移动到最近的驻点
                best_park_location = self.gantry_move_out_conveyor()
                self.gantry_robot.robot_info.next_park_location = best_park_location
                self.gantry_robot.move_to(best_park_location)

            self.gantry_gripeper_check(gripper_type = 'gripper_part') 
            _,path = dijkstra(edges, self.gantry_robot.robot_info.location, pick_location)
            for next_piont in path:
                self.gantry_robot.move_to(next_piont,eps=0.05)
            self.gantry_robot.move_to(pick_location)    

        ############################ pick 阶段 ###############################
        # 是否在传送带上抓取
        if pick_location =="conveyor":
           #1.1先抓起来
           # 抓之前，先判断kitting是否在conveyor上做抓取
            self.gantry_pick_place_collison(pick_location)
            attempts = 0
            MAX_ATTEMPTS = 3
            while not pick_success_flag and attempts<= MAX_ATTEMPTS and not rospy.is_shutdown():
                attempts = attempts+1
                part_on_conveyor = self.sensor_system.search_part_on_conveyor(target_part.type)
                # print "part_on_conveyor number:", len(part_on_conveyor)
                min_distance = 1000
                it = 0
                if part_on_conveyor: #选择距离最近的,并且可以抓的
                    #可以抓的
                    useful_parts =[]
                    for part in part_on_conveyor:
                        #计算抓取距离
                        # =计算最佳的抓取时间
                        current_time = rospy.get_time()
                        target_part_time = part.time_stamp
                        target_part_y = part.pose.position.y
                        c_y = -(current_time - target_part_time)*conveyor_vel+target_part_y
                        if c_y <= self.gantry_robot.gantry_base_y:
                            t_best = (self.gantry_robot.gantry_base_y- c_y)/(gantry_velocity - conveyor_vel)  
                        else:
                            t_best = (c_y - self.gantry_robot.gantry_base_y)/(gantry_velocity + conveyor_vel) 
                        f_y = c_y - t_best*conveyor_vel
                        if f_y >= conveyor_end+0.1:#说明可以抓
                            useful_parts.append(part)

                    for i in range (0, len(useful_parts)):
                        dis = abs(self.gantry_robot.gantry_base_y-(useful_parts[i].pose.position.y- \
                            (rospy.get_time()-useful_parts[i].time_stamp)*conveyor_vel))
                        if dis <= min_distance:
                            min_distance = dis
                            it = i
                    
                    if len(useful_parts)>0:
                        target_part_on_con = useful_parts[it]
                        pick_success_flag = self.gantry_robot.pick_part_on_conveyor(target_part_on_con)
                    else:
                        print('cant search part')
                        self.gantry_robot.robot_info.work_state = "standby"
                        self.gantry_robot.robot_info.is_idle = True
                        self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
                        self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配
                        return False
                else:
                    print('cant search part')
                    self.gantry_robot.robot_info.work_state = "standby"
                    self.gantry_robot.robot_info.is_idle = True
                    self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
                    self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配
                    return False

            #如果尝试3次失败之后，就反馈给任务分配系统
            if not pick_success_flag:
                print('pick error')
                self.gantry_robot.robot_info.work_state = "standby"
                self.gantry_robot.robot_info.is_idle = True
                self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
                self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配
                return False   
            ##############################先回到驻点######################################
            #寻找最佳点
            best_park_location = self.gantry_move_out_conveyor()
            self.gantry_robot.robot_info.next_park_location = best_park_location
            self.gantry_robot.move_to(best_park_location)
            # ###############################从驻点到AS位置########################################
            # _,path = dijkstra(edges, self.gantry_robot.robot_info.location, place_location)
            # for next_piont in path:
            #     self.gantry_robot.move_to(next_piont,eps=0.05)
            # self.gantry_robot.move_to(pick_location)

        else:# 不在传送带上抓取
            
            state = self.gantry_pick_place_collison(pick_location)
            attempts = 0
            MAX_ATTEMPTS = 3
            pick_success_flag = False
            self.gantry_robot.robot_info.work_state = "picking" 
            while not pick_success_flag and attempts<= MAX_ATTEMPTS and not rospy.is_shutdown():
                attempts = attempts+1
                target_part_list =  self.sensor_system.search_part_location_type(pick_location, target_part.type)
                if target_part_list:
                    target_part_on_bin_agv = target_part_list[0]
                    # if state:
                    #     pick_success_flag = self.S_pick_part_on_bin_agv(pick_location,target_part_on_bin_agv)
                    # else:
                    pick_success_flag = self.gantry_robot.pick_part_on_bin_agv(pick_location,target_part_on_bin_agv)
                else:
                    print('cant search part')
                    self.gantry_robot.robot_info.is_idle = True
                    self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配                   
                    return False 
            self.gantry_robot.robot_info.work_state = "standby" 
            #如果尝试3次失败之后，就反馈给任务分配系统
            if not pick_success_flag:
                print('pick error') 
                self.gantry_robot.robot_info.work_state = "standby" 
                self.gantry_robot.robot_info.is_idle = True
                self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
                self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配
                return False   


        ############################ check 阶段 ###############################

        if "agv" in pick_part_on:#从agv上pick不需要质检
            quality_check_flag = True
        else:#需要质检
            ####################  寻找质检位置  #####################
            #如果传感器失效：1、如果是从bin上抓取的，等待 30s；2、如果是从传送带抓取的，等待传感器恢复，最大100s
            if not self.sensor_system.is_alive() and not pick_location =="conveyor"and not rospy.is_shutdown():
                waitting_time = 0.0
                while not self.sensor_system.is_alive() and waitting_time < 30: 
                    rospy.sleep(0.1)
                    waitting_time = waitting_time+0.1
                if self.sensor_system.is_alive():
                    quality_check_flag = False
                else:
                    quality_check_flag = True
            elif not self.sensor_system.is_alive() and pick_location =="conveyor":

                waitting_time = 0.0
                while not self.sensor_system.is_alive() and waitting_time < 100 and not rospy.is_shutdown(): 
                    rospy.sleep(0.1)
                    waitting_time = waitting_time+0.1

                if self.sensor_system.is_alive():
                    quality_check_flag = False
                else:
                    quality_check_flag = True



            check_location = self.find_best_check_location(self.gantry_robot.robot_info.location)

            if not check_location:
                quality_check_flag = True



        if quality_check_flag ==False:
            _,path = dijkstra(edges, self.gantry_robot.robot_info.location, check_location)
            for next_piont in path:
                self.gantry_robot.move_to(next_piont,eps=0.05)
            self.gantry_robot.move_to(check_location)
            state = self.gantry_pick_place_collison(check_location)
            self.gantry_robot.robot_info.work_state = "placing"

            ################### 定义一下质检的姿态 ###################

            check_part = copy.deepcopy(target_part)
            check_part.pose.position.x = 0.0
            check_part.pose.position.y = 0.0
            check_part.pose.position.z = 0.0
            check_part.pose.orientation.x = 0.0
            check_part.pose.orientation.y = 0.0
            check_part.pose.orientation.z = 0.0
            check_part.pose.orientation.w = 1.0
            agv_name = check_location.split("_")[0]
            check_part.location = check_location
            check_part = target_pose_to_world(check_part,agv_name,check_location)

            # if state:
            #     r1,r2 = self.S_check_part(check_location, check_part)
            # else:


            
            r1,r2 = self.gantry_robot.check_part(check_location, check_part, is_assembly = True)   

            #1 1 ---检查完毕，零件放到agv上
            #1 0 ---零件是坏的，已经拿起来了  
            #0 0 ----传感器失效，不知道掉落在哪里了,也不知道是不是坏的零件  
            # self.gantry_robot.robot_info.work_state = "standby"

            if r1 and r2:
                quality_check_flag = True
                if self.sensor_system.is_alive():
                    check_part = self.sensor_system.new_part_dict[check_location]
                    check_part = self.sensor_system.search_part_use_part(check_part)

                pick_flag =  self.gantry_robot.pick_part_on_bin_agv(check_location,check_part)
                if not pick_flag:
                    if self.sensor_system.is_alive():
                        check_part = self.sensor_system.new_part_dict[check_location]
                        check_part = self.sensor_system.search_part_use_part(check_part)
                    pick_flag =  self.gantry_robot.pick_part_on_bin_agv(check_location,check_part)
                if not pick_flag:  
                    print('flip error')
                    quality_check_flag = False
                    self.gantry_robot.robot_info.work_state = "standby"
                    self.gantry_robot.robot_info.is_idle = True
                    self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
                    self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配  
                    return False

            if r1 and not r2:
                print('move to can')
                quality_check_flag = False
                _,path = dijkstra(edges, self.gantry_robot.robot_info.location, "can")
                for next_piont in path:
                    self.gantry_robot.move_to(next_piont)   
                self.gantry_pick_place_collison("can")
                self.gantry_robot.place_part_on_can()
                self.gantry_robot.robot_info.is_idle = True
                self.gantry_robot.robot_info.work_state = "standby"
                self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
                self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配  
                return False

            if not r1 and not r2:
                quality_check_flag = False
                print('error')
                self.gantry_robot.robot_info.work_state = "standby"
                self.gantry_robot.robot_info.is_idle = True
                self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
                self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配  
                return False

        ################################# 从驻点到AS位置 装配阶段 ##################################   
        if quality_check_flag:#质检没问题
            _,path = dijkstra(edges, self.gantry_robot.robot_info.location, place_location)
            for next_piont in path:
                self.gantry_robot.move_to(next_piont,eps=0.05)
            self.gantry_robot.move_to(place_location)
            if "battery" in target_part.type:
                self.gantry_robot.assemble_battery(place_location,target_part)
            elif "pump" in target_part.type:
                self.gantry_robot.assemble_pump(place_location,target_part)
            elif "sensor" in target_part.type:
                self.gantry_robot.assemble_sensor(place_location,target_part)
            elif "regulator" in target_part.type:
                self.gantry_robot.assemble_regulator(place_location,target_part)
            else:                
                pass
            
            print("gantry do assembly: cmd is finished!")
            self.cmd_feed_back(cmd, task_dispatcher)
            rospy.sleep(0.05)
            self.threadLock.acquire()
            cmd.is_done = True
            self.gantry_robot.robot_info.is_idle =True
            self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
            self.threadLock.release()
            return True
        else:
            print('cmd error')
            self.gantry_robot.robot_info.work_state = "standby"
            self.gantry_robot.robot_info.is_idle = True
            self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
            self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配  
            return False

    def gantry_robot_do_withdraw(self, cmd, task_dispatcher):
        #机器人状态更新 记得加锁
        print("gantry_robot is do withdraw")
        self.gantry_robot_do_withdraw_flag = 1
        self.threadLock.acquire()
        self.gantry_robot.robot_info.is_idle =False
        self.threadLock.release()

        #命令解析
        pick_part_on = cmd.pick_part_on
        target_part = cmd.pick_part
        place_position = cmd.target_position  #

        check_location = None 
        check_agv = None
        #转换成驻点地图
        pick_location = pick_part_on
        place_location = place_position
        path = []

        #动作标志位
        pick_success_flag = False
        quality_check_flag = False
        place_success_flag = False   
        check_result = False # faulty part = False
        #做一次路径规划到达要pick零件的驻点：
        # 如果gantry在传送带上，先到达最近的驻点然后在去withdraw
        if self.gantry_robot.robot_info.location == "conveyor":
            best_park_location = self.gantry_move_out_conveyor()
            self.gantry_robot.robot_info.next_park_location = best_park_location
            self.gantry_robot.move_to(best_park_location)
        #直接去withdaw的位置
        self.gantry_gripeper_check(gripper_type = 'gripper_part') 
        _,path = dijkstra(edges, self.gantry_robot.robot_info.location, pick_location)
        for next_piont in path:
            self.gantry_robot.move_to(next_piont)   

        #已经到达withdraw的地点
        attempts = 0
        MAX_ATTEMPTS = 3
        self.gantry_pick_place_collison(pick_part_on)
        while not pick_success_flag and attempts<= MAX_ATTEMPTS and not rospy.is_shutdown():
            attempts = attempts+1
            target_part.location = pick_part_on
            pick_success_flag = self.gantry_robot.pick_part_on_bin_agv(pick_part_on,target_part)

        if pick_success_flag:
            
            self.gantry_place_part_on_near_bin(target_part.type)
          
            # if self.gantry_robot.robot_info.location != place_location:
            #         _,path = dijkstra(edges, self.gantry_robot.robot_info.location, place_location)
            #         for next_piont in path:
            #             self.gantry_robot.move_to(next_piont)

            # if 'bin' in place_position: # 不知道放哪里 放置姿态需要进一步确定
            #     tmp_part = copy.deepcopy(target_part)
            #     tmp_part.pose =  self.withdraw_place_position(place_position,target_part.type)
            #     state = self.gantry_pick_place_collison(place_position)
            #     # if state:
            #     #     self.S_place_part_on_bin_agv(place_position,tmp_part) 
            #     # else:
            #     self.gantry_robot.place_part_on_bin_agv(place_position,tmp_part) 

            #     print("gantry do withdaw:cmd is finished!")
            #     cmd.is_done = True
            #     self.cmd_feed_back(cmd, task_dispatcher)
            #     self.gantry_robot.robot_info.is_idle =True
            #     return True

            # else:
                # self.gantry_robot.move_to("can")
                # self.gantry_robot.place_part_on_can()

            print("gantry do withdaw:cmd is finished!")
            cmd.is_done = True
            self.cmd_feed_back(cmd, task_dispatcher)
            rospy.sleep(0.05)
            self.threadLock.acquire()
            self.gantry_robot.robot_info.is_idle =True
            self.threadLock.release()
            return True
        else:
            print('cmd error')
            self.gantry_robot.robot_info.work_state = "standby"
            self.gantry_robot.robot_info.is_idle = True
            self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
            self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配  
            return False           


    def kitting_move_collision(self,kitting_next_location):
        ##########################此时判断要不要给gantry让路##############################
        # 进入到传送带
        work_state = ["picking", "placing", "flipping"]
        # print "kitting_next_location",kitting_next_location
        # print "self.gantry_robot.robot_info.location",self.gantry_robot.robot_info.location
        # print "self.gantry_robot.robot_info.work_state",self.gantry_robot.robot_info.work_state

        safe_flag = True
        if self.gantry_robot.robot_info.next_park_location == "conveyor":
            if self.gantry_robot.robot_info.location =="conveyor":#说明gantry等着抓取,或者正在抓取，safe
                safe_flag = True
            else:# 说明gantry等着进conveyor，此时判断应不应该给gantry让路
                safe_flag = False
                if self.gantry_robot.robot_info.location =="bin5" and self.kitting_robot.robot_info.location not in bin5_safe_zone:
                    safe_flag = True
                    if self.kitting_robot.robot_info.location =="bin5":
                        self.kitting_robot.move_to("agv4_ks4_tray")
                        self.kitting_robot.robot_info.work_state = "standby"
                        self.kitting_robot.robot_info.location = "agv4_ks4_tray"
                    else:
                        self.kitting_robot.move_to("agv3_ks3_tray")
                        self.kitting_robot.robot_info.work_state = "standby"
                        self.kitting_robot.robot_info.location = "agv3_ks3_tray"
                elif self.gantry_robot.robot_info.location =="bin6" and self.kitting_robot.robot_info.location not in bin6_safe_zone:
                    safe_flag = True
                    self.kitting_robot.move_to("agv3_ks3_tray")
                    self.kitting_robot.robot_info.work_state = "standby"
                    self.kitting_robot.robot_info.location = "agv3_ks3_tray"
                elif self.gantry_robot.robot_info.location =="can" and self.kitting_robot.robot_info.location not in can_safe_zone:
                    safe_flag = True
                    if self.kitting_robot.robot_info.location =="can":
                        self.kitting_robot.move_to("agv2_ks2_tray")
                        self.kitting_robot.robot_info.work_state = "standby"
                        self.kitting_robot.robot_info.location = "agv2_ks2_tray"
                    else:
                        self.kitting_robot.move_to("bin6")
                        self.kitting_robot.robot_info.work_state = "standby"
                        self.kitting_robot.robot_info.location = "bin6"
                elif self.gantry_robot.robot_info.location =="bin2" and self.kitting_robot.robot_info.location not in bin2_safe_zone:
                    safe_flag = True
                    if self.kitting_robot.robot_info.location =="bin2":
                        self.kitting_robot.move_to("agv2_ks2_tray")
                        self.kitting_robot.robot_info.work_state = "standby"
                        self.kitting_robot.robot_info.location = "agv2_ks2_tray"
                    else:
                        self.kitting_robot.move_to("agv1_ks1_tray")
                        self.kitting_robot.robot_info.work_state = "standby"
                        self.kitting_robot.robot_info.location = "agv1_ks1_tray"
                elif self.gantry_robot.robot_info.location =="bin1" and self.kitting_robot.robot_info.location not in bin1_safe_zone:
                    safe_flag = True
                    if self.kitting_robot.robot_info.location =="bin2":
                        self.kitting_robot.move_to("agv2_ks2_tray")
                        self.kitting_robot.robot_info.work_state = "standby"
                        self.kitting_robot.robot_info.location = "agv2_ks2_tray"
                    else:
                        self.kitting_robot.move_to("agv1_ks1_tray")
                        self.kitting_robot.robot_info.work_state = "standby"
                        self.kitting_robot.robot_info.location = "agv1_ks1_tray"
                else:
                    pass

            while not safe_flag and self.gantry_robot.robot_info.location != "conveyor":
                print ("waiting gantry  enter conveyor")
                rospy.sleep(0.5)
         # 离开传送带 
        elif self.gantry_robot.robot_info.next_park_location != "conveyor":
            if self.gantry_robot.robot_info.location =="conveyor":#说明gantry等着出去
                safe_flag = False
                if self.gantry_robot.robot_info.next_park_location =="bin5" and self.kitting_robot.robot_info.location not in bin5_safe_zone:
                    safe_flag = True
                    if self.kitting_robot.robot_info.location =="bin5":
                        self.kitting_robot.move_to("agv4_ks4_tray")
                        self.kitting_robot.robot_info.work_state = "standby"
                        self.kitting_robot.robot_info.location = "agv4_ks4_tray"
                    else:
                        self.kitting_robot.move_to("agv3_ks3_tray")
                        self.kitting_robot.robot_info.work_state = "standby"
                        self.kitting_robot.robot_info.location = "agv3_ks3_tray"
                elif self.gantry_robot.robot_info.next_park_location =="bin6" and self.kitting_robot.robot_info.location not in bin6_safe_zone:
                    safe_flag = True
                    self.kitting_robot.move_to("agv3_ks3_tray")
                    self.kitting_robot.robot_info.work_state = "standby"
                    self.kitting_robot.robot_info.location = "agv3_ks3_tray"
                elif self.gantry_robot.robot_info.next_park_location =="can" and self.kitting_robot.robot_info.location not in can_safe_zone:
                    safe_flag = True
                    if self.kitting_robot.robot_info.location =="can":
                        self.kitting_robot.move_to("agv2_ks2_tray")
                        self.kitting_robot.robot_info.work_state = "standby"
                        self.kitting_robot.robot_info.location = "agv2_ks2_tray"
                    else:
                        self.kitting_robot.move_to("bin6")
                        self.kitting_robot.robot_info.work_state = "standby"
                        self.kitting_robot.robot_info.location = "bin6"
                elif self.gantry_robot.robot_info.next_park_location =="bin2" and self.kitting_robot.robot_info.location not in bin2_safe_zone:
                    safe_flag = True
                    if self.kitting_robot.robot_info.location =="bin2":
                        self.kitting_robot.move_to("agv2_ks2_tray")
                        self.kitting_robot.robot_info.work_state = "standby"
                        self.kitting_robot.robot_info.location = "agv2_ks2_tray"
                    else:
                        self.kitting_robot.move_to("agv1_ks1_tray")
                        self.kitting_robot.robot_info.work_state = "standby"
                        self.kitting_robot.robot_info.location = "agv1_ks1_tray"
                elif self.gantry_robot.robot_info.next_park_location =="bin1" and self.kitting_robot.robot_info.location not in bin1_safe_zone:
                    safe_flag = True
                    if self.kitting_robot.robot_info.location =="bin2":
                        self.kitting_robot.move_to("agv2_ks2_tray")
                        self.kitting_robot.robot_info.work_state = "standby"
                        self.kitting_robot.robot_info.location = "agv2_ks2_tray"
                    else:
                        self.kitting_robot.move_to("agv1_ks1_tray")
                        self.kitting_robot.robot_info.work_state = "standby"
                        self.kitting_robot.robot_info.location = "agv1_ks1_tray"
                else:
                    pass


        wait_time = 0.0
        while not safe_flag and self.gantry_robot.robot_info.location == "conveyor" and not self.gantry_robot.robot_info.work_state in work_state\
            and wait_time<MAX_WAIT_TIME and not rospy.is_shutdown():
            print ("waiting gantry out conveyor")
            rospy.sleep(0.1)    #1
            wait_time = wait_time+0.1
        #print "self.gantry_robot.robot_info.work_state",self.gantry_robot.robot_info.work_state
            ####################################################################################
        wait_time = 0.0  
        while (self.gantry_robot.robot_info.work_state=="flipping") and self.gantry_robot.robot_info.is_enabled \
            and not rospy.is_shutdown() and self.gantry_robot.robot_info.location in kitting_robot_park_location and wait_time < MAX_WAIT_TIME:
            print ("waiting gantry flipping")
            rospy.sleep(0.1)    #0.5
            wait_time = wait_time+0.1
            # gantry 正在目标点工作
        # if self.gantry_robot.robot_info.location == kitting_next_location:
        #     while (self.gantry_robot.robot_info.work_state in work_state) and self.gantry_robot.robot_info.is_enabled \
        #         and not rospy.is_shutdown():
        #         print "waiting gantry working"
        #         rospy.sleep(0.5) 
        # else:
        #     pass   
        ############################################
        if self.gantry_robot.robot_info.is_alive and not self.gantry_robot.robot_info.is_enabled:
            self.gantry_robot.robot_info.is_alive = False                                           


    def kitting_pick_place_collison(self, kitting_location):
        bin12 = ["bin1","bin2"]
        bin56 = ["bin5","bin6"]
        work_state = ["picking", "placing", "flipping"]
        # print ("kitting_pick_place_collison is checking!")

        # while self.gantry_robot.robot_info.next_park_location ==kitting_location and \
        #     self.gantry_robot.robot_info.work_state =="moving" and not rospy.is_shutdown() \
        #     and self.gantry_robot.robot_info.is_enabled:
        #     print "waiting for gantry moving"
        #     rospy.sleep(0.1)

        # print('self.gantry_robot.robot_info.location',self.gantry_robot.robot_info.location)
        # print('kitting_location',kitting_location)
        # print('self.gantry_robot.robot_info.work_state',self.gantry_robot.robot_info.work_state)
        while self.gantry_robot.robot_info.location ==kitting_location \
            and (self.gantry_robot.robot_info.work_state in work_state) and \
            not rospy.is_shutdown() and self.gantry_robot.robot_info.is_enabled:
            # print "waiting for gantry picking"
            # print('self.gantry_robot.robot_info.work_state',self.gantry_robot.robot_info.work_state)
            rospy.sleep(0.1)


        while (self.gantry_robot.robot_info.location =="bin1" and kitting_location =="bin2") \
            and (self.gantry_robot.robot_info.work_state in work_state) and \
            not rospy.is_shutdown() and self.gantry_robot.robot_info.is_enabled:
            # print "waiting for gantry picking"
            rospy.sleep(0.1)

        while (self.gantry_robot.robot_info.location =="bin6" and kitting_location =="bin5") \
            and (self.gantry_robot.robot_info.work_state in work_state) and \
            not rospy.is_shutdown() and self.gantry_robot.robot_info.is_enabled:
            # print "waiting for gantry picking"
            rospy.sleep(0.1)

        if self.gantry_robot.robot_info.is_alive and not self.gantry_robot.robot_info.is_enabled:
            self.gantry_robot.robot_info.is_alive = False
            rospy.sleep(2)
            print ("sleeped")


    def kitting_robot_do_kitting(self,cmd,task_dispatcher):
        #机器人状态更新 记得加锁
        print("kitting_robot is do kitting")
        self.threadLock.acquire()
        self.kitting_robot.robot_info.is_idle = False
        self.threadLock.release()        
        
        #命令解析
        pick_part_on = cmd.pick_part_on
        target_part = cmd.pick_part
        target_agv_tray = cmd.target_position
        print ("kitting part:",target_part.i_d)
        print ("target agv :", target_agv_tray)
        if pick_part_on == None:
            print('cant search part')
            self.kitting_robot.robot_info.is_idle = True
            self.kitting_robot.robot_info.work_state = "standby"
            self.kitting_robot.kitting_robot_idle_time = rospy.get_time()
            self.cmd_realocation(cmd,task_dispatcher)
            return False
        #转换成驻点地图
        pick_location = pick_part_on
        place_location = target_agv_tray

        self.kitting_check_has_been_on_agv(target_agv_tray,target_part)

        path = []
        #动作标志位
        pick_success_flag = False
        quality_check_flag = False
        place_success_flag = False   
        check_result = False # faulty part = False

        ##################### 抓取阶段 #########################
        if "conveyor" in pick_part_on:

            ###########判断是否避障################
            while self.gantry_robot.robot_info.location =="conveyor" and self.gantry_robot.robot_info.work_state !="standby" and \
                not rospy.is_shutdown():
                print ("waiting for gantry picking")
                rospy.sleep(0.1) 
            
            if self.gantry_robot.robot_info.is_alive and not self.gantry_robot.robot_info.is_enabled:
                self.gantry_robot.robot_info.is_alive = False

            self.kitting_robot.robot_info.location = "conveyor"
            self.kitting_robot.robot_info.work_state = "picking"
            attempts = 0
            MAX_ATTEMPTS = 3
            while not pick_success_flag and attempts<= MAX_ATTEMPTS and not rospy.is_shutdown():
                attempts = attempts+1
                part_on_conveyor = self.sensor_system.search_part_on_conveyor(target_part.type)
                min_distance = 1000
                it = 0
                if part_on_conveyor: #选择距离最近的
                    for i in range (0, len(part_on_conveyor)):
                        dis = abs(self.kitting_robot.kitting_base_y-(part_on_conveyor[i].pose.position.y- \
                            (rospy.get_time()-part_on_conveyor[i].time_stamp)*conveyor_vel))
                        if dis <= min_distance:
                            min_distance = dis
                            it = i
                    target_part_on_con = part_on_conveyor[it]
                    
                    if self.kitting_robot_is_enabled(cmd,task_dispatcher):
                        pick_success_flag = self.kitting_robot.pick_part_on_conveyor(target_part_on_con)
                    else:
                        return False

                else:
                    print('cant search part')
                    self.kitting_robot.robot_info.is_idle = True
                    self.kitting_robot.kitting_robot_idle_time = rospy.get_time()
                    self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配
                    return False
            
            ####################去最近的驻点####################
            close_location = kitting_close_park_location(self.kitting_robot.robot_info.position.y)
            
            
            ##########lhr加######
            print('target_agv_tray',target_agv_tray)
            print('close_location',close_location)
            if target_agv_tray == close_location:
                if target_agv_tray == 'agv1_ks1_tray':
                    close_location = 'bin1'
                if target_agv_tray == 'agv2_ks2_tray':
                    close_location = 'bin2'
                if target_agv_tray == 'agv3_ks3_tray':
                    close_location = 'bin6'
                if target_agv_tray == 'agv4_ks4_tray':
                    close_location = 'bin5'
            else:
                pass
            #####################   


            
            self.kitting_robot.robot_info.next_park_location = close_location
            self.kitting_move_collision(close_location)
            if self.kitting_robot_is_enabled(cmd,task_dispatcher):
                self.kitting_robot.move_to(close_location,eps=0.05)
            else:
                return False
            self.kitting_robot.robot_info.location = close_location
            self.kitting_robot.robot_info.work_state = "standby"

            #如果尝试3次失败之后，就反馈给任务分配系统
            if not pick_success_flag:
                print('flip error')
                self.kitting_robot.robot_info.is_idle = True
                self.kitting_robot.kitting_robot_idle_time = rospy.get_time()
                self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配
                return False   

            ##########################此时判断要不要给gantry让路############################
            self.kitting_move_collision(close_location)
            #判断需不需要flip
            if pick_success_flag and target_part.is_flip: #如果需要，先放到一个空箱子上，或者可以放的位置
                #先找一个位置
                place_part_on = None
                flip_places =[]

                for bin in kitting_bins:
                    if not task_dispatcher.parts_on_warehouse_location[bin]:
                        flip_places.append(bin)
                if len(flip_places)>0:
                    place_part_on = self.kitting_robot_opt_short_path(self.kitting_robot.robot_info.position,flip_places,target_agv_tray)

                if place_part_on == None:    
                    for bin in kitting_bins:
                        if len(task_dispatcher.parts_on_warehouse_location[bin])<4:
                            flip_places.append(bin)     
                    if len(flip_places)>0:
                        place_part_on = self.kitting_robot_opt_short_path(self.kitting_robot.robot_info.position,flip_places,target_agv_tray)

                if place_part_on == None:#如果找不到，就不flip了
                    pass
                else:
                    #找到了，确定放置姿态
                    tmp_part = copy.deepcopy(target_part)
                    tmp_part.pose =  self.withdraw_place_position(place_part_on,target_part.type)

                    ###########判断是否避障################
                    ##########################此时判断要不要给gantry让路############################
                    self.kitting_robot.robot_info.next_park_location = place_part_on
                    self.kitting_move_collision(place_part_on)
                    if self.kitting_robot_is_enabled(cmd,task_dispatcher):
                        self.kitting_robot.move_to(place_part_on)
                    else:                          
                        return False
                    self.kitting_robot.robot_info.work_state = "standby"
                   ###########判断是否避障################
                    self.kitting_pick_place_collison(place_part_on)
                    self.kitting_robot.robot_info.work_state = "placing"
                    if self.kitting_robot_is_enabled(cmd,task_dispatcher):
                        self.kitting_robot.PLACE(tmp_part)
                    else:                          
                        return False
                    ##################################
                    #此时需要做flip#
                    if self.sensor_system.is_alive():
                        rospy.sleep(0.5)
                        new_part = self.sensor_system.new_part_dict[place_part_on]
                        new_part = self.sensor_system.search_part_use_part(new_part)
                        tmp_part = new_part
                    else:
                        new_part = tmp_part

                    flip_flag = False
                    attempts = 0
                    MAX_ATTEMPTS = 3
                    self.kitting_robot.robot_info.work_state = "placing"
                    ###########判断是否避障###############
                    while not flip_flag and attempts<= MAX_ATTEMPTS and not rospy.is_shutdown():
                        attempts = attempts+1

                        if self.sensor_system.is_alive():
                            rospy.sleep(0.5)
                            new_part = self.sensor_system.new_part_dict[place_part_on]
                            new_part = self.sensor_system.search_part_use_part(new_part)
                            tmp_part = new_part
                        else:
                            new_part = tmp_part

                        if self.kitting_robot_is_enabled(cmd,task_dispatcher):
                            self.kitting_robot.move_to(place_part_on) 
                        else:
                            return False

                        self.kitting_pick_place_collison(place_part_on)
                        if self.kitting_robot_is_enabled(cmd,task_dispatcher):
                            flip_flag = self.kitting_robot.flip(place_part_on,new_part)
                        else:                          
                            return False
                    if not flip_flag:
                        print('flip error')
                        self.kitting_robot.robot_info.work_state = "standby"
                        self.kitting_robot.robot_info.is_idle = True
                        self.kitting_robot.kitting_robot_idle_time = rospy.get_time()
                        self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配
                        return False

                    else: # flip 成功之后重新抓取
                        self.kitting_robot.robot_info.work_state = "picking"
                        if self.sensor_system.is_alive():
                            new_part  = self.sensor_system.new_part_dict[place_part_on]
                            new_part = self.sensor_system.search_part_use_part(new_part)
                        else:
                            new_part  = part_pose_after_flip(tmp_part)

                        repick_flag = False
                        attempts = 0 
                        MAX_ATTEMPTS = 3
                        if self.kitting_robot_is_enabled(cmd,task_dispatcher):
                            self.kitting_robot.move_to(place_part_on)
                        else:
                            return False
                        while not repick_flag and attempts < MAX_ATTEMPTS and not rospy.is_shutdown():
                            attempts = attempts+1
                            if self.kitting_robot_is_enabled(cmd,task_dispatcher):
                                repick_flag = self.kitting_robot.pick_part_on_bin_agv(place_part_on,new_part)
                            else:                          
                                return False
                        self.kitting_robot.robot_info.work_state = "standby"
                        if not repick_flag:
                            print('pick error')
                            self.kitting_robot.robot_info.is_idle = True
                            self.kitting_robot.kitting_robot_idle_time = rospy.get_time()
                            self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配
                            return False  

        # 在bin或者agv上kitting                                   
        else: # 在bin或者agv上kitting
            ###########判断是否避障################
            self.kitting_robot.robot_info.next_park_location = pick_location
            self.kitting_move_collision(pick_location)
            if self.kitting_robot_is_enabled(cmd,task_dispatcher):
                self.kitting_robot.move_to(pick_location) 
            else:
                print('robot disable')
                return False
            #判断是否需要flip
            if target_part.is_flip: 
                print ('kitting part need flip')
                pick_success_flag = False
                flip_flag = False 
                attempts = 0
                MAX_ATTEMPTS = 3
                target_part_on_bin_agv = None
                ###########判断是否避障################
                self.kitting_pick_place_collison(pick_location)
                while not flip_flag and attempts<= MAX_ATTEMPTS and not rospy.is_shutdown():
                    attempts = attempts+1
                    if self.kitting_robot_is_enabled(cmd,task_dispatcher):
                        self.kitting_robot.move_to(pick_location)  
                    else:
                        return False

                    target_part_list =  self.sensor_system.search_part_location_type(pick_location, target_part.type)
                    # print len(target_part_list)
                    
                    if target_part_list:
                        target_part_on_bin_agv = target_part_list[0]
                        #print "target_part_on_bin_agv",target_part_on_bin_agv.pose

                        self.kitting_pick_place_collison(pick_location)

                        if self.kitting_robot_is_enabled(cmd,task_dispatcher):
                            flip_flag = self.kitting_robot.flip(pick_location, target_part_on_bin_agv)
                            print ("kitting_flip_flag",flip_flag)
                        else:
                            return False
                    else:
                        print('cant search part1')
                        self.kitting_robot.robot_info.is_idle = True
                        self.kitting_robot.kitting_robot_idle_time = rospy.get_time()
                        self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配                   
                        return False 
                #如果尝试3次失败之后，就反馈给任务分配系统
                if not flip_flag:
                    print('flip error')
                    self.kitting_robot.robot_info.is_idle = True
                    self.kitting_robot.kitting_robot_idle_time = rospy.get_time()
                    self.kitting_robot.robot_info.work_state = "standby"
                    self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配
                    return False 
                else:
                    #重新抓取
                    print ("kitting repick")
                    if self.sensor_system.is_alive():
                        new_part  = self.sensor_system.new_part_dict[pick_location]
                        new_part = self.sensor_system.search_part_use_part(new_part)
                    else:
                        new_part  = part_pose_after_flip(target_part_on_bin_agv)
                        
                    pick_success_flag = False
                    attempts = 0 
                    MAX_ATTEMPTS = 3
                    if self.kitting_robot_is_enabled(cmd,task_dispatcher):
                        self.kitting_robot.move_to(pick_location,flip=True)
                    else:
                        return False
                    while not pick_success_flag and attempts < MAX_ATTEMPTS and not rospy.is_shutdown():
                        attempts = attempts+1
                        if self.sensor_system.is_alive():
                            new_part  = self.sensor_system.new_part_dict[pick_location]
                            new_part = self.sensor_system.search_part_use_part(new_part)
                        else:
                            new_part  = part_pose_after_flip(target_part_on_bin_agv)

                        if self.kitting_robot_is_enabled(cmd,task_dispatcher):
                            pick_success_flag = self.kitting_robot.pick_part_on_bin_agv(pick_location,new_part)
                        else:
                            return False
                    print ("kitting repick, and result" , pick_success_flag) 

                    if not pick_success_flag:
                        print('pick error')
                        self.kitting_robot.robot_info.work_state = "standby" 
                        self.kitting_robot.robot_info.is_idle = True
                        self.kitting_robot.kitting_robot_idle_time = rospy.get_time()
                        self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配
                        return False 
                    else:
                        pass 
            else:
                ###########判断是否避障################
                self.kitting_pick_place_collison(pick_location) 
                attempts = 0
                MAX_ATTEMPTS = 3
                pick_success_flag = False
                while not pick_success_flag and attempts<= MAX_ATTEMPTS and not rospy.is_shutdown():
                    attempts = attempts+1
                    target_part_list =  self.sensor_system.search_part_location_type(pick_location, target_part.type)
                    if target_part_list:
                        target_part_on_bin_agv = target_part_list[0]
                        if self.kitting_robot_is_enabled(cmd,task_dispatcher):
                            pick_success_flag = self.kitting_robot.pick_part_on_bin_agv(pick_location,target_part_on_bin_agv)
                        else:
                            return False
                    else:
                        print('cant search part')
                        self.kitting_robot.robot_info.is_idle = True
                        self.kitting_robot.kitting_robot_idle_time = rospy.get_time()
                        self.kitting_robot.robot_info.work_state = "standby"
                        self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配                   
                        return False 
                self.kitting_robot.robot_info.work_state = "standby"
                #如果尝试3次失败之后，就反馈给任务分配系统
                if not pick_success_flag:
                    print('pick error')
                    self.kitting_robot.robot_info.is_idle = True
                    self.kitting_robot.kitting_robot_idle_time = rospy.get_time()
                    self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配
                    return False    

        ##################### place阶段 #########################
        r1,r2 =False,False
        
        if pick_success_flag:
             ###########判断是否避障################
            self.kitting_robot.robot_info.work_state = "standby"
            self.kitting_robot.robot_info.next_park_location = target_agv_tray
            
            # self.kitting_move_collision(target_agv_tray)
            if self.kitting_robot_is_enabled(cmd,task_dispatcher):
                self.kitting_tray_check(target_agv_tray)
                self.kitting_move_collision(target_agv_tray)###lhr加的，有一种情况会碰撞，当kitting与目标位置很远时，上面2598行做的移动检查就会无效，可能上述检查完之后gantry正好去做零件翻转，所以必须在移动之前的前一行增加一个移动蔽障检查
                self.kitting_robot.move_to(target_agv_tray)  
            else:
                return False
            self.kitting_robot.robot_info.work_state = "standby"

             ###########判断是否避障################
            # rospy.sleep(0.2)
            self.kitting_pick_place_collison(target_agv_tray)
            if self.kitting_robot_is_enabled(cmd,task_dispatcher):
                r1,r2 = self.kitting_robot.check_part(target_agv_tray, target_part) # if check_result = faulty , pick up and back to park 
            else:
                return False

            #有可能在check的过程中，发生掉落的情况 //check 的过程就是一个place的过程
           #1--没有drop；2-发生了drop，官方给出说明，零件不会掉落在agv外面 返回结果 
           # 1 1 ---检查完毕，零件放到agv上
           #1 0 ---零件是坏的，已经拿起来了  
           #0 0 ----传感器失效，不知道掉落在哪里了,也不知道是不是坏的零件
        self.kitting_robot.robot_info.work_state = "standby" 

        # self.kitting_move_collision(target_agv_tray)  

        if r1 and r2:
            print("kitting do kitting :cmd is finished!")
            self.kitting_robot.robot_info.work_state = "standby"
            cmd.is_done = True
            self.cmd_feed_back(cmd, task_dispatcher)
            rospy.sleep(0.05)
            self.threadLock.acquire()
            self.kitting_robot.robot_info.is_idle = True
            self.kitting_robot.kitting_robot_idle_time = rospy.get_time()
            self.threadLock.release()
            return True 
       
        if r1 and not r2:
            self.threadLock.acquire()
            # self.kitting_robot.robot_info.work_state = "standby"
            # self.kitting_robot.robot_info.is_idle = True
            # self.kitting_robot.kitting_robot_idle_time = rospy.get_time()
            self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配  
            self.threadLock.release()
            if self.kitting_robot_is_enabled(cmd,task_dispatcher):
                self.kitting_robot.move_to("can") 
                self.kitting_robot.place_part_on_can()
            else:
                return False

            self.threadLock.acquire()
            self.kitting_robot.robot_info.work_state = "standby"
            self.kitting_robot.robot_info.is_idle = True
            self.kitting_robot.kitting_robot_idle_time = rospy.get_time()
            self.threadLock.release()
            return False
        if not r1 and not r2:   
            print("kitting not do kitting :cmd is finished!")
            self.kitting_robot.robot_info.work_state = "standby"
            self.cmd_feed_back(cmd, task_dispatcher)
            rospy.sleep(0.05)
            self.threadLock.acquire()
            cmd.is_done = True
            self.kitting_robot.robot_info.is_idle = True
            self.kitting_robot.kitting_robot_idle_time = rospy.get_time()
            self.threadLock.release()
           
            # self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配  
            return False

    def kitting_robot_do_withdraw(self,cmd,task_dispatcher):
        #机器人状态更新 记得加锁
        print("kitting_robot is do withdraw")
        self.kitting_robot_do_withdraw_flag = 1
        self.threadLock.acquire()
        self.kitting_robot.robot_info.is_idle =False
        self.threadLock.release()
        #命令解析

        pick_part_on = cmd.pick_part_on
        target_part = cmd.pick_part
        target_place_on = cmd.target_position
       
        #转换成驻点地图 kitting 没有
        pick_location = pick_part_on
        place_location =target_place_on
        path = []
        #动作标志位
        pick_success_flag = False
        self.kitting_move_collision(pick_part_on)
        if self.kitting_robot_is_enabled(cmd,task_dispatcher):
            self.kitting_robot.move_to(pick_part_on)
        else:
            return False
        self.kitting_pick_place_collison(pick_part_on)
        #已经到达withdraw的地点
        attempts = 0
        MAX_ATTEMPTS = 3
        while not pick_success_flag and attempts<= MAX_ATTEMPTS and not rospy.is_shutdown():
            attempts = attempts+1
            part_on_ks = self.sensor_system.search_part_location_type(pick_part_on,target_part.type)
            target_part.location = pick_part_on
            if self.kitting_robot_is_enabled(cmd,task_dispatcher):
                pick_success_flag = self.kitting_robot.pick_part_on_bin_agv(pick_part_on,target_part)
            else:
                return False

        if not pick_success_flag:
            print('pick error')
            self.kitting_robot.robot_info.work_state ="standby"
            self.kitting_robot.robot_info.is_idle = True
            self.kitting_robot.kitting_robot_idle_time = rospy.get_time()
            self.cmd_realocation(cmd,task_dispatcher) # 说明任务需要重新分配  
            return False

        if pick_success_flag:
            
            self.kitting_place_part_on_near_bin(target_part.type)
            # if target_place_on =="can":
            #     self.kitting_move_collision(target_place_on)
            #     if self.kitting_robot_is_enabled(cmd,task_dispatcher):
            #         self.kitting_robot.move_to("can")
            #         self.kitting_robot.place_part_on_can()
            #     else:
            #         return False
            #     print("withdraw_cmd is finished!")
            #     cmd.is_done = True
            #     self.cmd_feed_back(cmd, task_dispatcher)
            #     self.kitting_robot.robot_info.is_idle =True
            #     self.kitting_robot.kitting_robot_idle_time = rospy.get_time()
            #     self.kitting_robot.robot_info.work_state ="standby"
            #     return True 
            # else:
            #     self.kitting_move_collision(target_place_on)
            #     if self.kitting_robot_is_enabled(cmd,task_dispatcher):
            #         self.kitting_robot.move_to(target_place_on) 
            #     else:
            #         return False 
            #     #找到了，确定放置姿态
            #     tmp_part = copy.deepcopy(target_part)
            #     tmp_part.pose =  self.withdraw_place_position(target_place_on,target_part.type) 

            #     self.kitting_pick_place_collison(target_place_on)
            #     if self.kitting_robot_is_enabled(cmd,task_dispatcher):
            #         self.kitting_robot.PLACE(tmp_part)
            #     else:
            #         return False
            print("withdraw_cmd is finished!")
            self.cmd_feed_back(cmd, task_dispatcher)
            rospy.sleep(0.05)
            self.threadLock.acquire()
            cmd.is_done = True
            self.kitting_robot.robot_info.is_idle =True
            self.kitting_robot.kitting_robot_idle_time = rospy.get_time()
            self.kitting_robot.robot_info.work_state ="standby"
            self.threadLock.release()

            return True 
   
    def kitting_robot_is_enabled(self,cmd,task_dispatcher):
        if not self.kitting_robot.robot_info.is_enabled:
            self.kitting_robot.STOP()
            self.cmd_realocation(cmd,task_dispatcher)
            return False
        else:
            return True

    def gantry_robot_is_enabled(self, cmd,task_dispatcher):
        if not self.gantry_robot.robot_info.is_enabled:
            self.gantry_robot.STOP()
            print ("gantry stop!")
            self.cmd_realocation(cmd,task_dispatcher)
            return False
        else:
            return True        


    ########################################################################  \
    # gantry 的特殊操作             
    ########################################################################  

    def kitting_robot_nearest_agv_location(self):
        position_y = self.kitting_robot.robot_info.position.y
        l1 = abs(position_y -kitting_robot_park_location['agv1_ks1_tray'][1])
        l4 = abs(position_y -kitting_robot_park_location['agv2_ks2_tray'][1])-0.07
        l6 = abs(position_y -kitting_robot_park_location['agv3_ks3_tray'][1])
        l9 = abs(position_y -kitting_robot_park_location['agv4_ks4_tray'][1])
        length = [l1,l4,l6,l9]
        locations = ['agv1_ks1_tray','agv2_ks2_tray','agv3_ks3_tray','agv4_ks4_tray']
        short =length.index(min(length))
        location = locations[short]
        return location


    def kitting_place_part_on_near_bin(self, part_type = "assembly_pump_red"):
        #根据与机器人底座y坐标距离为bin排序，优先考虑离传送带进的bin
        bin_y_location = [3.379920, 2.565006, -3.379920, -2.565006]
        bin_list_1 = ["bin1","bin2", "bin5", "bin6"]
        bin_near_sort = []

        for bin_num in range(0, len(bin_y_location)):
            if len(bin_near_sort) == 0:
                bin_near_sort.append(bin_num)
            for temp_num in range(0, len(bin_near_sort)):
                if abs(self.kitting_robot.kitting_base_y - bin_y_location[bin_num]) < abs(self.kitting_robot.kitting_base_y - bin_y_location[bin_near_sort[temp_num]]):
                    bin_near_sort.insert(temp_num, bin_num)
        bin_list_sort_1 = []
        for temp_num in bin_near_sort:
            bin_list_sort_1.append(bin_list_1[temp_num])

        bin_near_sort_list = []

        bin_near_sort_list = bin_list_sort_1
        #寻找可以放零件的空位置
        #bin区域零件划分
        #    4    3
        #    1    2

        bin_part_list = {
                        "bin1": self.sensor_system.bin1_parts,
                        "bin2": self.sensor_system.bin2_parts,
                        "bin5": self.sensor_system.bin5_parts,
                        "bin6": self.sensor_system.bin6_parts,
                        }
        region_find_flag = False
        region_find_result = []

        for bin_name in bin_near_sort_list:
             
            if not region_find_flag:
                bin_region = [False, False, False, False]
                for part in bin_part_list[bin_name]:
                    temp_num_n = 0.05
                    if part.pose.position.x >= (bin_location[bin_name][0] - temp_num_n) and part.pose.position.y <= bin_location[bin_name][1]:
                        bin_region[0] = True
                    elif part.pose.position.x >= (bin_location[bin_name][0] - temp_num_n) and part.pose.position.y > bin_location[bin_name][1]:
                        bin_region[1] = True
                    elif part.pose.position.x < (bin_location[bin_name][0] - temp_num_n) and part.pose.position.y >= bin_location[bin_name][1]:
                        bin_region[2] = True
                    elif part.pose.position.x < (bin_location[bin_name][0] - temp_num_n) and part.pose.position.y < bin_location[bin_name][1]:
                        bin_region[3] = True

                for region_num in range(0, len(bin_region)):
                    if bin_region[region_num] == False:
                        if [bin_name, region_num] == self.gantry_robot_move_to:
                            continue
                        else:
                            region_find_flag = True
                            region_find_result = [bin_name, region_num]
                            break
            else:
                break

        part_position_bin = [[0.1, -0.1], [0.1, 0.1], [-0.1, 0.1], [-0.1, -0.1]]

        if region_find_flag:
            # #放到找到的bin上
            self.kitting_robot_move_to = region_find_result
            self.kitting_move_collision(region_find_result[0])
            self.kitting_robot.move_to(region_find_result[0])
            ############################################
            #part_type = part_type
            pose = Pose()
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1

            pose.position.x = bin_position[region_find_result[0]][0] + part_position_bin[region_find_result[1]][0] - 0.07
            pose.position.y = bin_position[region_find_result[0]][1] + part_position_bin[region_find_result[1]][1]
            if "regulator" in part_type:
                pose.position.z = bins_product_height['regulator'] - logic_camera_height_error
            if "pump" in part_type:
                pose.position.z = bins_product_height['pump'] - logic_camera_height_error            
            if "battery" in part_type:
                pose.position.z = bins_product_height['battery'] - logic_camera_height_error
            if "sensor" in part_type:
                pose.position.z = bins_product_height['sensor'] - logic_camera_height_error

            target_part = sPart(part_type, region_find_result[0], pose)
            self.kitting_pick_place_collison(region_find_result[0])
            self.kitting_robot.PLACE(target_part)
            # self.check_part(region_find_result[0], target_part)
        else:
            #沒找到，放到垃圾桶中（待定）
            self.kitting_move_collision("can")
            self.kitting_robot.move_to("can")
            self.kitting_robot.place_part_on_can()

        self.kitting_robot_move_to = [None, 0]
        return True

    def gantry_place_part_on_near_bin(self, part_type = "assembly_pump_red"):
        #根据与机器人底座y坐标距离为bin排序，优先考虑离传送带进的bin
        #默认机器人已经抓到零件
        bin_y_location = [3.379920, 2.565006, -3.379920, -2.565006]
        bin_list_1 = ["bin1","bin2", "bin5", "bin6"]
        bin_list_2 = ["bin4","bin3", "bin8", "bin7"]
        bin_near_sort = []

        for bin_num in range(0, len(bin_y_location)):
            if len(bin_near_sort) == 0:
                bin_near_sort.append(bin_num)
            for temp_num in range(0, len(bin_near_sort)):
                if abs(self.gantry_robot.gantry_base_y - bin_y_location[bin_num]) < abs(self.gantry_robot.gantry_base_y - bin_y_location[bin_near_sort[temp_num]]):
                    bin_near_sort.insert(temp_num, bin_num)
        bin_list_sort_1 = []
        bin_list_sort_2 = []
        for temp_num in bin_near_sort:
            bin_list_sort_1.append(bin_list_1[temp_num])
            bin_list_sort_2.append(bin_list_2[temp_num])
        bin_near_sort_list = []

        bin_near_sort_list = bin_list_sort_1 + bin_list_sort_2
        #寻找可以放零件的空位置
        #bin区域零件划分
        #    4    3
        #    1    2

        bin_part_list = {
                        "bin1": self.sensor_system.bin1_parts,
                        "bin2": self.sensor_system.bin2_parts,
                        "bin3": self.sensor_system.bin3_parts,
                        "bin4": self.sensor_system.bin4_parts,
                        "bin5": self.sensor_system.bin5_parts,
                        "bin6": self.sensor_system.bin6_parts,
                        "bin7": self.sensor_system.bin7_parts,
                        "bin8": self.sensor_system.bin8_parts,
                        }
        region_find_flag = False
        region_find_result = []
        for bin_name in bin_near_sort_list:
            if not region_find_flag:
                bin_region = [False, False, False, False]
                for part in bin_part_list[bin_name]:
                    temp_num_n = 0.05
                    if part.pose.position.x >= (bin_location[bin_name][0] - temp_num_n) and part.pose.position.y <= bin_location[bin_name][1]:
                        bin_region[0] = True
                    elif part.pose.position.x >= (bin_location[bin_name][0]- temp_num_n) and part.pose.position.y > bin_location[bin_name][1]:
                        bin_region[1] = True
                    elif part.pose.position.x < (bin_location[bin_name][0]- temp_num_n) and part.pose.position.y >= bin_location[bin_name][1]:
                        bin_region[2] = True
                    elif part.pose.position.x < (bin_location[bin_name][0]- temp_num_n) and part.pose.position.y < bin_location[bin_name][1]:
                        bin_region[3] = True

                for region_num in range(0, len(bin_region)):
                    if bin_region[region_num] == False:
                        if [bin_name, region_num] == self.kitting_robot_move_to:
                            continue
                        else:                        
                            region_find_flag = True
                            region_find_result = [bin_name, region_num]
                            break
            else:
                break

        part_position_bin = [[0.1, -0.1], [0.1, 0.1], [-0.1, 0.1], [-0.1, -0.1]]
        if region_find_flag:
            # #放到找到的bin上
            self.gantry_robot_move_to = region_find_result


            if self.gantry_robot.robot_info.location == "conveyor":
                best_park_location = self.gantry_move_out_conveyor()
                self.gantry_robot.robot_info.next_park_location = best_park_location
                self.gantry_robot.move_to(best_park_location)
            #直接去withdaw的位置
            self.gantry_gripeper_check(gripper_type = 'gripper_part') 
            _,path = dijkstra(edges, self.gantry_robot.robot_info.location, region_find_result[0])
            for next_piont in path:
                self.gantry_robot.move_to(next_piont)

            self.gantry_robot.move_to(region_find_result[0])
            #################################################
            #part_type = part_type
            pose = Pose()
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1

            pose.position.x = bin_position[region_find_result[0]][0] + part_position_bin[region_find_result[1]][0] - 0.07
            pose.position.y = bin_position[region_find_result[0]][1] + part_position_bin[region_find_result[1]][1]
            if "regulator" in part_type:
                pose.position.z = bins_product_height['regulator'] - logic_camera_height_error
            if "pump" in part_type:
                pose.position.z = bins_product_height['pump'] - logic_camera_height_error            
            if "battery" in part_type:
                pose.position.z = bins_product_height['battery'] - logic_camera_height_error
            if "sensor" in part_type:
                pose.position.z = bins_product_height['sensor'] - logic_camera_height_error

            target_part = sPart(part_type, region_find_result[0], pose)
            self.gantry_pick_place_collison(region_find_result[0])
            self.gantry_robot.place_part_on_bin_agv(region_find_result[0], target_part)
            # self.check_part(region_find_result[0], target_part)
        else:
            #沒找到，放到垃圾桶中（待定）
            self.gantry_robot.move_to("can")
            self.gantry_robot.place_part_on_can()

        self.gantry_robot_move_to = [None, 0]
        return True

    def kitting_robot_do_part_check(self, kitting_shipment):
        print("kitting_robot is do part check on agv:",kitting_shipment.agv_id )
        self.threadLock.acquire()
        self.kitting_robot.robot_info.is_idle = False
        self.threadLock.release()     
        self.sensor_system.all_parts_list_old_update()
        rospy.sleep(1.0)

        quality_control_sensor_n = self.sensor_system.read_faulty_list_by_agv_id(kitting_shipment.agv_id)
        agv_parts_list_n = self.sensor_system.read_agv_list_by_agv_id(kitting_shipment.agv_id)

        while quality_control_sensor_n:
            for faulty_part in quality_control_sensor_n:
                faulty_search = self.sensor_system.Faulty_Part_Search(faulty_part.pose.position, agv_parts_list_n)
                if faulty_search != False:
                    self.kitting_move_collision(faulty_search.location)
                    self.kitting_robot.move_to(faulty_search.location)
                    self.kitting_pick_place_collison(faulty_search.location)
                    if not self.kitting_robot.pick_part_on_bin_agv(faulty_search.location, faulty_search):
                        self.kitting_robot.pick_part_on_bin_agv(faulty_search.location, faulty_search)
                    self.kitting_move_collision("can")
                    self.kitting_robot.move_to('can')
                    self.kitting_robot.place_part_on_can()  
                    self.kitting_robot.robot_info.is_idle = False
            if self.sensor_system.is_alive():
                quality_control_sensor_n = self.sensor_system.read_faulty_list_by_agv_id(kitting_shipment.agv_id)
                agv_parts_list_n = self.sensor_system.read_agv_list_by_agv_id(kitting_shipment.agv_id)
            else:  
                break    

        agv_present_location = self.agv_locations[kitting_shipment.agv_id] 
        
        #对合格的零件做标记
        parts_no_pass_list = []
        parts_no_pass_dict_list = []
        agv_parts_list_n = self.sensor_system.read_agv_list_by_agv_id(kitting_shipment.agv_id)
        self.sensor_system.Reset_Part_Final_Check(agv_parts_list_n)
        for part_num in range(len(kitting_shipment.products)):                    
            target_part = target_pose_to_world(kitting_shipment.products[part_num], kitting_shipment.agv_id, agv_present_location)            
            if not self.sensor_system.Part_Final_Check(target_part, agv_parts_list_n):
                parts_no_pass_list.append(target_part)
                parts_no_pass_dict_list.append(part_num)
        #零件调整
        print('Check 1:kitting no pass')
        print(len(parts_no_pass_list))

        for part_num1 in range(len(parts_no_pass_list)): 
            part_n = parts_no_pass_list[part_num1]
            parts_repick_flag = False
            agv_parts_list_n = self.sensor_system.read_agv_list_by_agv_id(kitting_shipment.agv_id)
            for part_n1 in agv_parts_list_n:
                if part_n1.type == part_n.type and part_n1.final_check == False:

                    print(part_n1.type)
                    rpy_temp1 = euler_from_quaternion([part_n1.pose.orientation.x,part_n1.pose.orientation.y,\
                        part_n1.pose.orientation.z,part_n1.pose.orientation.w])       

                    if "pump" in part_n1.type and not (abs(rpy_temp1[0])<=0.05 or abs(abs(rpy_temp1[0])-pi)<=0.1):
                        print("kitting check1:faulty flip")
                        self.kitting_move_collision(part_n1.location)
                        self.kitting_robot.move_to(part_n1.location)
                        self.kitting_pick_place_collison(part_n1.location)
                        if not self.kitting_robot.pick_part_on_bin_agv(part_n1.location, part_n1):
                            self.kitting_robot.pick_part_on_bin_agv(part_n1.location, part_n1)
                        self.kitting_move_collision("can")
                        self.kitting_robot.move_to("can")
                        self.kitting_robot.place_part_on_can()  
                        break  
                    else:

                        print("kitting check1:adjust part is:",part_n1.type)
                        self.kitting_move_collision(part_n1.location)
                        self.kitting_robot.move_to(part_n1.location)
                        self.kitting_pick_place_collison(part_n1.location)
                        self.kitting_robot.adjust_part_on_bin_agv(part_n1.location, part_n1, part_n)
                        self.kitting_robot.robot_info.is_idle =False
                        target_part_temp = self.sensor_system.new_part_dict[part_n1.location]
                        self.sensor_system.change_part_check_part_use_id(agv_parts_list_n, target_part_temp)
                        parts_repick_flag = True
                        target_part_temp = self.sensor_system.new_part_dict[part_n1.location]
                        self.sensor_system.change_part_check_part_use_id(agv_parts_list_n, target_part_temp)
                        break




        if self.sensor_system.is_alive():
            agv_parts_list_n = self.sensor_system.read_agv_list_by_agv_id(kitting_shipment.agv_id)
            self.sensor_system.Reset_Part_Final_Check(agv_parts_list_n)
            for part_num in range(len(kitting_shipment.products)):                    
                target_part = target_pose_to_world(kitting_shipment.products[part_num], kitting_shipment.agv_id, agv_present_location)            
                self.sensor_system.Part_Final_Check(target_part, agv_parts_list_n)

            agv_parts_list_n = self.sensor_system.read_agv_list_by_agv_id(kitting_shipment.agv_id)
            for part_n1 in agv_parts_list_n:
                if part_n1.final_check == False:
                    print(part_n1.type)
                    rpy_temp1 = euler_from_quaternion([part_n1.pose.orientation.x,part_n1.pose.orientation.y,\
                        part_n1.pose.orientation.z,part_n1.pose.orientation.w])                    
                    if "pump" in part_n1.type and not (abs(rpy_temp1[0])<=0.05 or abs(abs(rpy_temp1[0])-pi)<=0.1):
                        print("kitting check2:faulty flip")
                        self.kitting_move_collision(part_n1.location)
                        self.kitting_robot.move_to(part_n1.location)
                        self.kitting_pick_place_collison(part_n1.location)
                        if not self.kitting_robot.pick_part_on_bin_agv(part_n1.location, part_n1):
                            self.kitting_robot.pick_part_on_bin_agv(part_n1.location, part_n1)
                        self.kitting_move_collision("can")
                        self.kitting_robot.move_to("can")
                        self.kitting_robot.place_part_on_can()                        
                    else:
                        print("kitting check2:place on near bin")
                        self.kitting_move_collision(part_n1.location)
                        self.kitting_robot.move_to(part_n1.location)
                        self.kitting_pick_place_collison(part_n1.location)
                        if not self.kitting_robot.pick_part_on_bin_agv(part_n1.location, part_n1):
                            self.kitting_robot.pick_part_on_bin_agv(part_n1.location, part_n1)
                        self.kitting_place_part_on_near_bin(part_type = part_n1.type)

            self.order_system.change_kitting_shipment_part_flag_id(kitting_shipment, part_check_flag=2)
        else:
            self.order_system.change_kitting_shipment_part_flag_id(kitting_shipment, part_check_flag=0)

        self.threadLock.acquire()
        self.kitting_robot.robot_info.is_idle = True
        self.threadLock.release()              
        self.kitting_robot.kitting_robot_idle_time = rospy.get_time()


                
    def gantry_robot_do_part_check(self, kitting_shipment):
        print("gantry_robot is do part check on agv:",kitting_shipment.agv_id)
        self.threadLock.acquire()
        self.gantry_robot.robot_info.is_idle =False
        self.threadLock.release()
        self.sensor_system.all_parts_list_old_update()

        quality_control_sensor_n = self.sensor_system.read_faulty_list_by_agv_id(kitting_shipment.agv_id)
        agv_parts_list_n = self.sensor_system.read_agv_list_by_agv_id(kitting_shipment.agv_id)
        
        while quality_control_sensor_n:
            
            for faulty_part in quality_control_sensor_n:
                faulty_search = self.sensor_system.Faulty_Part_Search(faulty_part.pose.position, agv_parts_list_n)
                if faulty_search != False:
                    if self.gantry_robot.robot_info.location == "conveyor":
                        best_park_location = self.gantry_move_out_conveyor()
                        self.gantry_robot.robot_info.next_park_location = best_park_location
                        self.gantry_robot.move_to(best_park_location)
                    
                    self.gantry_gripeper_check(gripper_type = 'gripper_part') 

                    self.gantry_robot.move_to(faulty_search.location)
                    self.gantry_pick_place_collison(faulty_search.location)
                    if not self.gantry_robot.pick_part_on_bin_agv(faulty_search.location, faulty_search):
                        self.gantry_robot.pick_part_on_bin_agv(faulty_search.location, faulty_search)

                    _,path = dijkstra(edges, self.gantry_robot.robot_info.location, "can")
                    for next_piont in path:
                        self.gantry_robot.move_to(next_piont) 
                    # self.gantry_robot.move_to('can')
                    self.gantry_robot.place_part_on_can()
                    self.kitting_robot.robot_info.is_idle =False
            if self.sensor_system.is_alive():
                quality_control_sensor_n = self.sensor_system.read_faulty_list_by_agv_id(kitting_shipment.agv_id)
                agv_parts_list_n = self.sensor_system.read_agv_list_by_agv_id(kitting_shipment.agv_id)
            else:  
                break       
            
                
        agv_present_location = self.agv_locations[kitting_shipment.agv_id]        
        #对合格的零件做标记
        parts_no_pass_list = []
        parts_no_pass_dict_list = []
        agv_parts_list_n = self.sensor_system.read_agv_list_by_agv_id(kitting_shipment.agv_id)
        self.sensor_system.Reset_Part_Final_Check(agv_parts_list_n)
        for part_num in range(len(kitting_shipment.products)):                    
            target_part = target_pose_to_world(kitting_shipment.products[part_num], kitting_shipment.agv_id, agv_present_location)            
            if not self.sensor_system.Part_Final_Check(target_part, agv_parts_list_n):
                parts_no_pass_list.append(target_part)
                parts_no_pass_dict_list.append(part_num)
        #零件调整
        print('gantry no pass')
        print(len(parts_no_pass_list))
        for part_num1 in range(len(parts_no_pass_list)): 
            part_n = parts_no_pass_list[part_num1]
            parts_repick_flag = False
            agv_parts_list_n = self.sensor_system.read_agv_list_by_agv_id(kitting_shipment.agv_id)
            for part_n1 in agv_parts_list_n:
                if part_n1.type == part_n.type and part_n1.final_check == False:
                    print(part_n1.type)
                    rpy_temp1 = euler_from_quaternion([part_n1.pose.orientation.x,part_n1.pose.orientation.y,\
                        part_n1.pose.orientation.z,part_n1.pose.orientation.w])                    
                    #if "pump" in part_n1.type and ((rpy_temp1[0] % pi) > pi/4 and (rpy_temp1[0] % pi) < 3*pi/4):
                    if "pump" in part_n1.type and not (abs(rpy_temp1[0])<=0.05 or abs(abs(rpy_temp1[0])-pi)<=0.1):
                        print("gantry check 1: faulty flip")
                        if self.gantry_robot.robot_info.location == "conveyor":
                            best_park_location = self.gantry_move_out_conveyor()
                            self.gantry_robot.robot_info.next_park_location = best_park_location
                            self.gantry_robot.move_to(best_park_location)     
                        
                        self.gantry_gripeper_check(gripper_type = 'gripper_part') 
                        _,path = dijkstra(edges, self.gantry_robot.robot_info.location, part_n1.location)
                        for next_piont in path:
                            self.gantry_robot.move_to(next_piont) 
                        self.gantry_robot.move_to(part_n1.location)

                        self.gantry_pick_place_collison(part_n1.location)
                        if not self.gantry_robot.pick_part_on_bin_agv(part_n1.location, part_n1):
                            self.gantry_robot.pick_part_on_bin_agv(part_n1.location, part_n1)

                        if self.gantry_robot.robot_info.location == "conveyor":
                            best_park_location = self.gantry_move_out_conveyor()
                            self.gantry_robot.robot_info.next_park_location = best_park_location
                            self.gantry_robot.move_to(best_park_location)
                        
                        _,path = dijkstra(edges, self.gantry_robot.robot_info.location, "can")
                        for next_piont in path:
                            self.gantry_robot.move_to(next_piont) 
                        self.gantry_robot.move_to("can")     
                        self.gantry_robot.place_part_on_can()   
                        break

                    else:
                        
                    
                        print("gantry check1 :adjust part is:",part_n1.type)
                        if self.gantry_robot.robot_info.location == "conveyor":
                            best_park_location = self.gantry_move_out_conveyor()
                            self.gantry_robot.robot_info.next_park_location = best_park_location
                            self.gantry_robot.move_to(best_park_location)
                    
                        self.gantry_gripeper_check(gripper_type = 'gripper_part') 
                        _,path = dijkstra(edges, self.gantry_robot.robot_info.location, part_n1.location)
                        for next_piont in path:
                            self.gantry_robot.move_to(next_piont)                     
                        self.gantry_robot.move_to(part_n1.location)
                        self.gantry_pick_place_collison(part_n1.location)
                        self.gantry_robot.adjust_part_on_bin_agv(part_n1.location, part_n1, part_n)
                        self.gantry_robot.robot_info.is_idle =False
                        parts_repick_flag = True
                        
                        target_part_temp = self.sensor_system.new_part_dict[part_n1.location]
                        self.sensor_system.change_part_check_part_use_id(agv_parts_list_n, target_part_temp)
                        break

        if self.sensor_system.is_alive():
            agv_parts_list_n = self.sensor_system.read_agv_list_by_agv_id(kitting_shipment.agv_id)
            self.sensor_system.Reset_Part_Final_Check(agv_parts_list_n)
            for part_num in range(len(kitting_shipment.products)):                    
                target_part = target_pose_to_world(kitting_shipment.products[part_num], kitting_shipment.agv_id, agv_present_location)            
                self.sensor_system.Part_Final_Check(target_part, agv_parts_list_n)   
            
            agv_parts_list_n = self.sensor_system.read_agv_list_by_agv_id(kitting_shipment.agv_id)


            for part_n1 in agv_parts_list_n:
                if part_n1.final_check == False:
                    print(part_n1.type)
                    rpy_temp1 = euler_from_quaternion([part_n1.pose.orientation.x,part_n1.pose.orientation.y,\
                        part_n1.pose.orientation.z,part_n1.pose.orientation.w])                    
                    #if "pump" in part_n1.type and ((rpy_temp1[0] % pi) > pi/4 and (rpy_temp1[0] % pi) < 3*pi/4):
                    if "pump" in part_n1.type and not (abs(rpy_temp1[0])<=0.05 or abs(abs(rpy_temp1[0])-pi)<=0.1):
                        print("gantry check 2:faulty flip")
                        if self.gantry_robot.robot_info.location == "conveyor":
                            best_park_location = self.gantry_move_out_conveyor()
                            self.gantry_robot.robot_info.next_park_location = best_park_location
                            self.gantry_robot.move_to(best_park_location)
                    
                        self.gantry_gripeper_check(gripper_type = 'gripper_part') 
                        _,path = dijkstra(edges, self.gantry_robot.robot_info.location, part_n1.location)
                        for next_piont in path:
                            self.gantry_robot.move_to(next_piont) 
                        self.gantry_robot.move_to(part_n1.location)
                        self.gantry_pick_place_collison(part_n1.location)
                        if not self.gantry_robot.pick_part_on_bin_agv(part_n1.location, part_n1):
                            self.gantry_robot.pick_part_on_bin_agv(part_n1.location, part_n1)

                        # if self.gantry_robot.robot_info.location == "conveyor":
                        #     best_park_location = self.gantry_move_out_conveyor()
                        #     self.gantry_robot.robot_info.next_park_location = best_park_location
                        #     self.gantry_robot.move_to(best_park_location)

                        #self.gantry_gripeper_check(gripper_type = 'gripper_part')            
                        _,path = dijkstra(edges, self.gantry_robot.robot_info.location, "can")
                        for next_piont in path:
                            self.gantry_robot.move_to(next_piont) 
                        self.gantry_robot.move_to('can')
                        self.gantry_robot.place_part_on_can()                       
                    else:
                        print("gantry check 2:place on near bin")
                        if self.gantry_robot.robot_info.location == "conveyor":
                            best_park_location = self.gantry_move_out_conveyor()
                            self.gantry_robot.robot_info.next_park_location = best_park_location
                            self.gantry_robot.move_to(best_park_location)

                        self.gantry_gripeper_check(gripper_type = 'gripper_part')                     
                        _,path = dijkstra(edges, self.gantry_robot.robot_info.location, part_n1.location)
                        for next_piont in path:
                            self.gantry_robot.move_to(next_piont)                     
                        self.gantry_robot.move_to(part_n1.location)
                        self.gantry_pick_place_collison(part_n1.location)
                        if not self.gantry_robot.pick_part_on_bin_agv(part_n1.location, part_n1):
                            self.gantry_robot.pick_part_on_bin_agv(part_n1.location, part_n1)

                        self.gantry_place_part_on_near_bin(part_type = part_n1.type)


            self.order_system.change_kitting_shipment_part_flag_id(kitting_shipment, part_check_flag=2)
        else:
            self.order_system.change_kitting_shipment_part_flag_id(kitting_shipment, part_check_flag=0)

        self.threadLock.acquire()
        self.gantry_robot.robot_info.is_idle = True
        self.threadLock.release()        
     


    def gantry_check_has_been_on_agv(self, agv_present_location,target_part):
        # part_on_expect_agv = self.sensor_system.search_part_location_type(agv_present_location,target_part.type)
        part_on_expect_agv = self.sensor_system.read_agv_list_by_agv_id(agv_present_location)


        if part_on_expect_agv:
            for part in part_on_expect_agv:
                if abs(part.pose.position.x - target_part.pose.position.x)<= 0.05 and abs(part.pose.position.y - target_part.pose.position.y)<=0.05:
                    #先到地方再挪走
                    print('invalid part')
                    if self.gantry_robot.robot_info.location == "conveyor":
                        best_park_location = self.gantry_move_out_conveyor()
                        self.gantry_robot.robot_info.next_park_location = best_park_location
                        self.gantry_robot.move_to(best_park_location)
                    #直接去withdaw的位置
                    self.gantry_gripeper_check(gripper_type = 'gripper_part') 
                    _,path = dijkstra(edges, self.gantry_robot.robot_info.location, agv_present_location)
                    for next_piont in path:
                        self.gantry_robot.move_to(next_piont)
                    self.gantry_pick_place_collison(agv_present_location)
                    if not self.gantry_robot.pick_part_on_bin_agv(agv_present_location,part):
                        self.gantry_robot.pick_part_on_bin_agv(agv_present_location,part) 
                    #放到最近的bin上
                    self.gantry_place_part_on_near_bin(part_type = target_part.type)
                    return True  
        else:
            return True


    def kitting_check_has_been_on_agv(self, agv_present_location,target_part):

        # part_on_expect_agv = self.sensor_system.search_part_location_type(agv_present_location,target_part.type)
        part_on_expect_agv = self.sensor_system.read_agv_list_by_agv_id(agv_present_location)

        if part_on_expect_agv:
            for part in part_on_expect_agv:
                if abs(part.pose.position.x - target_part.pose.position.x)<= 0.05 and abs(part.pose.position.y - target_part.pose.position.y)<=0.05:
                    #先到地方再挪走
                    print('invalid part')
                    self.kitting_move_collision(agv_present_location)
                    self.kitting_robot.move_to(agv_present_location)
                    self.kitting_pick_place_collison(agv_present_location)
                    if not self.kitting_robot.pick_part_on_bin_agv(agv_present_location,part):
                        self.kitting_robot.pick_part_on_bin_agv(agv_present_location,part)    
                    #放到最近的bin上
                    self.kitting_place_part_on_near_bin(part_type = target_part.type)
                    return True  
        else:
            return True

        
    # def kitting_robot_do_adjust():

        
    def kitting_robot_do_adjust(self,cmd,task_dispatcher):
        #机器人状态更新 记得加锁
        print("kitting_robot is do adjusting")
        self.threadLock.acquire()
        self.kitting_robot.robot_info.is_idle =False
        self.threadLock.release()
        
        #命令解析
        pick_part_on = cmd.pick_part_on
        origion_part = copy.deepcopy(cmd.pick_part)
        target_part = copy.deepcopy(cmd.pick_part)
        target_part.pose = cmd.pick_part.adjust_pose
        target_agv_tray = cmd.target_position
    
        print ("adjusting part:",target_part.i_d)
        print ("target agv :", target_agv_tray)
        

        #转换成驻点地图
        pick_location = pick_part_on
        place_location = target_agv_tray

        path = []
        #动作标志位
        pick_success_flag = False
        quality_check_flag = False
        place_success_flag = False   
        check_result = False # faulty part = False
        ##################### 抓取阶段 #########################
       
        #####################判断是否避障################
        self.kitting_robot.robot_info.next_park_location = pick_location
        self.kitting_move_collision(pick_location)
        if self.kitting_robot_is_enabled(cmd,task_dispatcher):
            self.kitting_robot.move_to(pick_location) 
        else:
            return False
        self.kitting_pick_place_collison(pick_part_on)
        self.kitting_robot.adjust_part_on_bin_agv(pick_part_on,origion_part,target_part)

        
        print("adjust_cmd is finished!")
        self.cmd_feed_back(cmd, task_dispatcher)
        rospy.sleep(0.05)
        self.threadLock.acquire()
        cmd.is_done = True
        self.kitting_robot.robot_info.is_idle =True
        self.kitting_robot.kitting_robot_idle_time = rospy.get_time()
        self.kitting_robot.robot_info.work_state ="standby"
        self.threadLock.release()
        return True     


    def gantry_robot_do_adjust(self,cmd,task_dispatcher):
        #机器人状态更新 记得加锁
        print("gantry_robot is do adjusting")
        self.threadLock.acquire()
        self.gantry_robot.robot_info.is_idle =False
        self.threadLock.release()
   
        #命令解析
        pick_part_on = cmd.pick_part_on
        origion_part = copy.deepcopy(cmd.pick_part)
        target_part = copy.deepcopy(cmd.pick_part)
        target_part.pose = cmd.pick_part.adjust_pose
        target_agv_tray = cmd.target_position
        
        print ("adjusting part:",target_part.i_d)
        print ("target agv :", target_agv_tray)
        

        #转换成驻点地图
        pick_location = pick_part_on
        place_location = target_agv_tray

        path = []
        #动作标志位
        pick_success_flag = False
        quality_check_flag = False
        place_success_flag = False   
        check_result = False # faulty part = False
        ##################### 抓取阶段 #########################

        if self.gantry_robot.robot_info.location =="conveyor":#先移动到最近的驻点
            best_park_location = self.gantry_move_out_conveyor()
            self.gantry_robot.robot_info.next_park_location = best_park_location
            if self.gantry_robot_is_enabled(cmd,task_dispatcher):
                self.gantry_robot.move_to(best_park_location)
            else:
                return False

        self.gantry_gripeper_check(gripper_type = 'gripper_part') 
        _,path = dijkstra(edges, self.gantry_robot.robot_info.location, pick_location)
        for next_piont in path:
            if self.gantry_robot_is_enabled(cmd,task_dispatcher):
                self.gantry_robot.move_to(next_piont,eps=0.05)
            else:
                return False
        if self.gantry_robot_is_enabled(cmd,task_dispatcher):        
            self.gantry_robot.move_to(pick_location)
        else:
            return False    

    
        self.gantry_pick_place_collison(pick_part_on)
        self.gantry_robot.adjust_part_on_bin_agv(pick_part_on,origion_part,target_part)

        
        print("adjust_cmd is finished!")
        self.cmd_feed_back(cmd, task_dispatcher)
        rospy.sleep(0.05)
        self.threadLock.acquire()
        cmd.is_done = True
        self.gantry_robot.robot_info.is_idle =True
        self.gantry_robot.robot_info.work_state ="standby"
        self.threadLock.release()
        return True     

   
    def gantry_robot_do_pick_tray(self,cmd,task_dispatcher):
        #机器人状态更新 记得加锁
        print("gantry_robot is do pick_tray")
        self.threadLock.acquire()
        self.gantry_robot.robot_info.is_idle =False
        self.threadLock.release()

        #命令解析
        pick_part_on = cmd.pick_part_on
        target_part = cmd.pick_part
        target_agv_tray = cmd.target_position
        print("gantry do kitting part:",target_part.i_d)
        print("target agv :", target_agv_tray)
        print("pick location:", pick_part_on)
        if pick_part_on == None:
            print('cant search part')
            self.gantry_robot.robot_info.is_idle = True
            self.gantry_robot.robot_info.work_state = "standby"
            self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
            self.cmd_realocation(cmd,task_dispatcher)
            return False

        #转换成驻点地图
        pick_location = 'tray_table'
        place_location = target_agv_tray
        path = []

        #self.gantry_check_has_been_on_agv(target_agv_tray,target_part)

        #动作标志位
        pick_success_flag = False
        quality_check_flag = False
        place_success_flag = False   
        check_result = False # faulty part = False
        # #做一次路径规划到达要pick零件的驻点
        # if pick_location =="conveyor":
        #     #寻找最佳进入点
        #     if self.gantry_robot.robot_info.location!="conveyor":
        #         self.gantry_move_in_conveyor(pick_location)
        # else:
        #     if self.gantry_robot.robot_info.location =="conveyor":#先移动到最近的驻点
        #         best_park_location = self.gantry_move_out_conveyor()
        #         self.gantry_robot.robot_info.next_park_location = best_park_location
        #         if self.gantry_robot_is_enabled(cmd,task_dispatcher):
        #             self.gantry_robot.move_to(best_park_location)
        #         else:
        #             return False

        #     self.gantry_gripeper_check(gripper_type = 'gripper_tray') 
        #     self.gantry_robot.robot_arm_init

        #     _,path = dijkstra(edges, self.gantry_robot.robot_info.location, pick_location)
        #     for next_piont in path:
        #         if self.gantry_robot_is_enabled(cmd,task_dispatcher):
        #             self.gantry_robot.move_to(next_piont,eps=0.05)
        #         else:
        #             return False
        #     if self.gantry_robot_is_enabled(cmd,task_dispatcher):        
        #         self.gantry_robot.move_to(pick_location)
        #     else:
        #         return False 
        if self.gantry_robot.robot_info.location =="conveyor":#先移动到最近的驻点
                best_park_location = self.gantry_move_out_conveyor()
                self.gantry_robot.robot_info.next_park_location = best_park_location
                if self.gantry_robot_is_enabled(cmd,task_dispatcher):
                    self.gantry_robot.move_to(best_park_location)
                else:
                    return False

        if self.gantry_robot.gantry_gripper_type != 'gripper_tray':
            self.gantry_gripeper_check(gripper_type = 'gripper_tray') 
            self.gantry_robot.robot_arm_init
        else:
            pass

        _,path = dijkstra(edges, self.gantry_robot.robot_info.location, pick_location)
        for next_piont in path:
            if self.gantry_robot_is_enabled(cmd,task_dispatcher):
                self.gantry_robot.move_to(next_piont,eps=0.05)
            else:
                return False
        if self.gantry_robot_is_enabled(cmd,task_dispatcher):        
            self.gantry_robot.move_to(pick_location)
        else:
            return False

        target_part_type = target_part.type
        tray_needed = self.sensor_system.search_tray_by_type(target_part_type)                    

        self.gantry_robot.pick_tray('tray_table', tray_needed, target_agv_tray)
        target_place = target_agv_tray
        self.gantry_robot.move_to('gripper_cs')
        self.gantry_robot.move_to(target_place)

        target_tray = tray_needed

        if 'agv1' in target_place:
            agv_kitting_location_n = agv1_kitting_location
        elif 'agv2' in target_place:
            agv_kitting_location_n = agv2_kitting_location
        elif 'agv3' in target_place:
            agv_kitting_location_n = agv3_kitting_location
        elif 'agv4' in target_place:
            agv_kitting_location_n = agv4_kitting_location
        
        target_tray.pose.position.x = agv_kitting_location_n[target_place][0]
        target_tray.pose.position.y = agv_kitting_location_n[target_place][1]
        target_tray.pose.position.z = agv_kitting_location_n[target_place][2]

        target_tray.pose.orientation.x = 0.0
        target_tray.pose.orientation.y = 0.0
        target_tray.pose.orientation.z = -0.707106781187
        target_tray.pose.orientation.w = 0.707106781187

        self.gantry_pick_place_collison(target_place)
        if 'agv2' in target_place or 'agv3' in target_place:
            self.gantry_robot_place_tray_collison(target_place)
        else:
            pass
        self.gantry_robot.place_part_on_bin_agv(target_place, target_tray, distance=0.5)
        # near_park = gantry_location_near_ks(target_place)
        # if self.gantry_robot.robot_info.is_enabled:
        #     self.gantry_robot.move_to(near_park)

        

        # rospy.sleep(5)
        if self.tray_lockflag[target_agv_tray] == False:
            agv_tay_id = 'kit_tray_' + target_agv_tray[3]
            if lock_agv_tray(agv_tay_id):
                print('lock', agv_tay_id)
                self.tray_lockflag[target_agv_tray] = True

        self.cmd_feed_back(cmd, task_dispatcher)
        rospy.sleep(0.05)
        self.threadLock.acquire()
        cmd.is_done = True
        self.gantry_robot.robot_info.is_idle =True
        self.gantry_robot.robot_info.work_state ="standby"
        self.gantry_robot.gantry_robot_idle_time = rospy.get_time()
        self.threadLock.release()
        return True  
    def gantry_robot_place_tray_collison(self,gantry_location):
        # kitting_y = self.kitting_robot.robot_info.location[1]
        kitting = self.kitting_robot.get_kitting_robot_joint_states()
        kitting_y = kitting[1]
        gantry_y = kitting_robot_park_location[gantry_location][1]
        # print('kitting_y',kitting_y)
        # print('gantry_y',gantry_y)
        while self.kitting_robot.robot_info.location == "conveyor" and float(abs(kitting_y - gantry_y)) <= 1.0:
            # print('等待kitting移动到隔壁的箱子')
            kitting = self.kitting_robot.get_kitting_robot_joint_states()
            kitting_y = kitting[1]
            # print('abs(kitting_y - gantry_y)',abs(kitting_y - gantry_y))
            rospy.sleep(0.1)
        while self.kitting_robot.robot_info.location == gantry_location and self.kitting_robot.robot_info.is_enabled:
            self.kitting_robot.move_to('can')
            rospy.sleep(0.01)
        print('safety')


    def gantry_gripeper_check(self, gripper_type = 'gripper_tray'):
        # self.threadLock.acquire()
        self.gantry_robot.robot_info.is_idle =False
        # self.threadLock.release()
        while self.gantry_robot.gantry_gripper_type != gripper_type:
            #print('change gripper_type', gripper_type)
            self.gantry_robot.move_to('gripper_cs')
            rospy.wait_for_service('/ariac/gantry/arm/gripper/change')
            rospy.ServiceProxy('/ariac/gantry/arm/gripper/change', ChangeGripper)(gripper_type)
            rospy.sleep(0.5)  

            self.gantry_robot.robot_info.location = 'gripper_cs'

    def kitting_tray_check(self, agv_id):

        if 'agv1' in agv_id:
            needed_tray = self.order_system.agv_1_needed_tray
            if self.sensor_system.AGV1_movable_tray != None:
                AGV_movable_tray = self.sensor_system.AGV1_movable_tray.type
            else:
                AGV_movable_tray = None
        elif 'agv2' in agv_id:
            needed_tray = self.order_system.agv_2_needed_tray
            if self.sensor_system.AGV2_movable_tray != None:
                AGV_movable_tray = self.sensor_system.AGV2_movable_tray.type
            else:
                AGV_movable_tray = None
        elif 'agv3' in agv_id:
            needed_tray = self.order_system.agv_3_needed_tray
            if self.sensor_system.AGV3_movable_tray != None:
                AGV_movable_tray = self.sensor_system.AGV3_movable_tray.type
            else:
                AGV_movable_tray = None
        elif 'agv4' in agv_id:
            needed_tray = self.order_system.agv_4_needed_tray
            if self.sensor_system.AGV4_movable_tray != None:
                AGV_movable_tray = self.sensor_system.AGV4_movable_tray.type
            else:
                AGV_movable_tray = None
        # print(agv_id)
        # print(needed_tray)
        # print(AGV_movable_tray)
        if needed_tray != AGV_movable_tray:
            nearest_bin = self.kitting_robot_find_nearest_bin(agv_id)
            self.kitting_robot.move_to(nearest_bin,eps=0.05)


        while needed_tray != AGV_movable_tray:
            rospy.sleep(0.5)
            if 'agv1' in agv_id:
                needed_tray = self.order_system.agv_1_needed_tray
                if self.sensor_system.AGV1_movable_tray != None:
                    AGV_movable_tray = self.sensor_system.AGV1_movable_tray.type
                else:
                    AGV_movable_tray = None
            elif 'agv2' in agv_id:
                needed_tray = self.order_system.agv_2_needed_tray
                if self.sensor_system.AGV2_movable_tray != None:
                    AGV_movable_tray = self.sensor_system.AGV2_movable_tray.type
                else:
                    AGV_movable_tray = None
            elif 'agv3' in agv_id:
                needed_tray = self.order_system.agv_3_needed_tray
                if self.sensor_system.AGV3_movable_tray != None:
                    AGV_movable_tray = self.sensor_system.AGV3_movable_tray.type
                else:
                    AGV_movable_tray = None
            elif 'agv4' in agv_id:
                needed_tray = self.order_system.agv_4_needed_tray
                if self.sensor_system.AGV4_movable_tray != None:
                    AGV_movable_tray = self.sensor_system.AGV4_movable_tray.type
                else:
                    AGV_movable_tray = None
    def kitting_robot_find_nearest_bin (self,agv_id):
        kitting = self.kitting_robot.get_kitting_robot_joint_states()
        kitting_y = kitting[1]
        if 'agv1' in agv_id and kitting_y <= agv1_ks1_tray[1]:
            nearest_bin = 'bin1'
        elif 'agv2' in agv_id and kitting_y <= agv2_ks2_tray[1]:
            nearest_bin = 'can'
        elif 'agv2' in agv_id and kitting_y >= agv2_ks2_tray[1]:
            nearest_bin = 'bin2'
        elif 'agv3' in agv_id and kitting_y <= agv3_ks3_tray[1]:
            nearest_bin = 'bin6'
        elif 'agv3' in agv_id and kitting_y >= agv3_ks3_tray[1]:
            nearest_bin = 'can'
        elif 'agv4' in agv_id and kitting_y >= agv4_ks4_tray[1]:
            nearest_bin = 'bin5'
        else:
            nearest_bin = 'can'
        return nearest_bin
        
            
            
        
    



if __name__ == '__main__':

    rospy.init_node("ariac_example_node")

    lock_agv_tray('kit_tray_1')
