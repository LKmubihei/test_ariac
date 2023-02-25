#!/usr/bin/env python
#coding:utf-8
## version v0.1

import rospy
import time

from Sensor_System import Sensor_System
from Order_System import Order_Processing
#from Order_System import Order_Processing

from Task_Dispatcher import *
from Robot_System import *
from std_msgs.msg import String
from std_srvs.srv import Trigger
import sys

global game_end_flag
game_end_flag = False

def start_competition():
    rospy.wait_for_service('/ariac/start_competition')
    rospy.ServiceProxy('/ariac/start_competition', Trigger)()
    return rospy.get_time()

def end_competition_callBack(msg):
    if msg.data == "done":
        global game_end_flag
        game_end_flag = True  
        
    
if __name__ == '__main__':

    end_competition_sub = rospy.Subscriber("/ariac/competition_state",String,end_competition_callBack)
    rospy.init_node("ariac_example_node")
    start_time = start_competition()
    sensor_system = Sensor_System(start_time)
    time.sleep(1)
    order_system = Order_Processing()
    robot_system = Robot_System(sensor_system, order_system)
    
    result = robot_system.robot_system_init()


    if result:
        print("robot init success!")
        task_dispatcher = Task_Dispatcher(order_system,sensor_system,robot_system)
        print(sensor_system.AGV1_movable_tray)
    else:
        print('\033[31m robot init failed, please check!!!')
        sys.exit(0)
    
   
    # step one: initialization

    
    task_dispatcher.read_agv_location()
    task_dispatcher.command_list_init()
    task_dispatcher.get_all_part_on_warehouse()

    while not rospy.is_shutdown() and not game_end_flag:
        task_dispatcher.read_order_list()
#        cmd_list = task_dispatcher.run()
        cmd_list = task_dispatcher.test_run()
        robot_system.run(cmd_list,task_dispatcher)
    print ("competition end!")
    sys.exit(0)