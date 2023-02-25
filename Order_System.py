#!/usr/bin/env python
#coding:utf-8

import rospy
from nist_gear.msg import Orders
from std_srvs.srv import Trigger
from tf.transformations import *
from public_data import *

def start_competition():
    rospy.wait_for_service('/ariac/start_competition')
    rospy.ServiceProxy('/ariac/start_competition', Trigger)()

class Part:
    def __init__(self):
        self.type = None
        self.i_d = ""
        self.pose = None
        self.is_done = False
        self.is_flip = False
        self.time_lack = None
        self.u_id = None
        self.location = None
        self.target_agv_id = None
        self.is_adjust = False
        self.adjust_pose = None

class Kitting_shipments:
     def __init__(self):
         self.kitting_shipment = []
         self.is_done = None

class Assembly_shipments:
     def __init__(self):
         self.assembly_shipment = []
         self.is_done = None
         
class Withdraw_shipments:
     def __init__(self):
         self.withdraw_shipment = []
         self.is_done = None         

class Kitting_shipment:
    def __init__(self):
        self.shipment_type = None
        self.agv_id = None
        self.station_id = None
        self.movable_tray = None
        self.products = []
        self.is_done = False
        self.is_submit = False 
        #0:未检测 1：检测中 2:检测通过
        self.check_flag = 0  

class Assembly_shipment:
    def __init__(self):
        self.shipment_type = None
        self.station_id = None
        self.products = []
        self.is_done = False
        self.is_submit = False
        
class Withdraw_shipment:
    def __init__(self):
        self.shipment_type = None
        self.agv_id = None
        self.station_id = None
        self.products = []
        self.is_done = False
        self.is_submit = False     

class My_order:
    def __init__(self,order_id):
        self.order_id = order_id
        self.withdraw_shipments = None
        self.kitting_shipments = None
        self.assembly_shipments = None
        self.is_done = False
        self.abandon = False
        self.priority = None    

class Order_Processing:
    def __init__(self):

        self.reaper_order_0 = None
        self.reaper_order_0_update = None
        self.reaper_order_1 = None
        self.reaper_order_1_update = None

        self.agv_1_needed_tray = None
        self.agv_2_needed_tray = None
        self.agv_3_needed_tray = None
        self.agv_3_needed_tray = None        
        
        self.order_list = []
        self.order_subscriber_= rospy.Subscriber("/ariac/orders", Orders, self.order_callback)
        print("hello, this is order_publisher")
      
#        self.order_subscriber_2= rospy.Subscriber("/ariac/orders", Order, self.order_callback2)
        self.ship_numb = 0
        
    def order_callback(self, msg):

        if msg.order_id == "order_0":
            self.order_0_init(msg)
            self.order_list.append(self.reaper_order_0)
            print("hello, this is order_0")
        elif msg.order_id == "order_0_update":
            print("TOPIC order_0_update")
            print(rospy.get_time())
#            rospy.sleep(5)
            self.order_0_update_init(msg)
            self.order_list.append(self.reaper_order_0_update)
            self.order_state = "order_0_update"
            print("hello, this is order_0_update")
#            rospy.sleep(20)
        elif msg.order_id == "order_1":
            self.order_1_init(msg)
            self.order_list.append(self.reaper_order_1)
            print("hello, this is order_1")
        elif msg.order_id == "order_1_update":
            self.order_1_update_init(msg)
            self.order_list.append(self.reaper_order_1_update)
            print("hello, this is order_1_update")
            
        if self.order_list != []:
            for order in self.order_list:
                if order.order_id == "order_0_update":
                    for order_ in self.order_list:
                        if order_.order_id == "order_0":
                            self.order_list.remove(order_)
            for order in self.order_list:
                if order.order_id == "order_1_update":
                    for order_ in self.order_list:
                        if order_.order_id == "order_1":
                            self.order_list.remove(order_)
                            
        #将"order_0_update" or "order_0"放到self.order_list的第一个位置。
        if len(self.order_list) ==1:
            pass
        elif len(self.order_list) == 2:
            order_list_temp = [0, 0]
            for order_ in self.order_list:
                if "order_0" in order_.order_id:
                    order_list_temp[0] = order_
                if "order_1" in order_.order_id:
                    order_list_temp[1] = order_
            self.order_list = []
            self.order_list = order_list_temp

    def return_order_list(self):                     
        return(self.order_list)

    def order_0_init(self,msg):

        self.reaper_order_0 = My_order(msg.order_id)
        self.reaper_order_0.is_done =False
        self.reaper_order_0.abandon = False
        self.reaper_order_0.priority = 1
        
        tmp_withdraw_shipments = Withdraw_shipments()    ## all withdraw parts
        tmp_withdraw_shipments.is_done = True
        self.reaper_order_0.withdraw_shipments = tmp_withdraw_shipments
        
        if msg.kitting_shipments == []:
            tmp_kitting_shipments = Kitting_shipments()
            tmp_kitting_shipments.is_done = True
            self.reaper_order_0.kitting_shipments = tmp_kitting_shipments
        else:
            ###parse
            tmp_kitting_shipments = Kitting_shipments()
            tmp_kitting_shipments.is_done = False
            
            for shipment_iter in range (0, len(msg.kitting_shipments)):
                tmp_kitting_shipment = Kitting_shipment()
                tmp_kitting_shipment.shipment_type = msg.kitting_shipments[shipment_iter].shipment_type
                tmp_kitting_shipment.agv_id = msg.kitting_shipments[shipment_iter].agv
                tmp_kitting_shipment.station_id = msg.kitting_shipments[shipment_iter].assembly_station
                tmp_kitting_shipment.movable_tray = Part()
                tmp_kitting_shipment.movable_tray.type = msg.kitting_shipments[shipment_iter].movable_tray.movable_tray_type
                tmp_kitting_shipment.movable_tray.pose = msg.kitting_shipments[shipment_iter].movable_tray.pose
                
                if tmp_kitting_shipment.agv_id == 'agv1':
                    self.agv_1_needed_tray = tmp_kitting_shipment.movable_tray.type
                elif tmp_kitting_shipment.agv_id == 'agv2':
                    self.agv_2_needed_tray = tmp_kitting_shipment.movable_tray.type
                elif tmp_kitting_shipment.agv_id == 'agv3':
                    self.agv_3_needed_tray = tmp_kitting_shipment.movable_tray.type
                elif tmp_kitting_shipment.agv_id == 'agv4':
                    self.agv_4_needed_tray = tmp_kitting_shipment.movable_tray.type

                tmp_kitting_shipment.products = msg.kitting_shipments[shipment_iter].products
                tmp_kitting_shipment.is_done  = False
                tmp_kitting_shipment.is_submit = False
                for part_itr in range(0, len(tmp_kitting_shipment.products)):
                    tmp_part = Part()
                    tmp_part.type = tmp_kitting_shipment.products[part_itr].type
                    tmp_part.pose = tmp_kitting_shipment.products[part_itr].pose
                    tmp_part.i_d  = tmp_kitting_shipment.shipment_type + ":"+tmp_part.type +":"+str(part_itr)
                    tmp_part.is_done = False     ###dqw temp
#                    tmp_part.is_done = True
                    tmp_kitting_shipment.products[part_itr] = tmp_part
                tmp_kitting_shipments.kitting_shipment.append(tmp_kitting_shipment)
            self.reaper_order_0.kitting_shipments = tmp_kitting_shipments
            self.order_0_kit_part_fin = [0 for _ in range(len(self.reaper_order_0.kitting_shipments.kitting_shipment))]
        
        if msg.assembly_shipments == []:
            tmp_assembly_shipments = Assembly_shipments()
            tmp_assembly_shipments.is_done = True
            self.reaper_order_0.assembly_shipments = tmp_assembly_shipments
        else:
            ###parse
            tmp_assembly_shipments = Assembly_shipments()
            tmp_assembly_shipments.is_done = False
            
            for shipment_iter in range (0, len(msg.assembly_shipments)):
                tmp_assembly_shipment = Assembly_shipment()
                tmp_assembly_shipment.shipment_type = msg.assembly_shipments[shipment_iter].shipment_type
                tmp_assembly_shipment.station_id = msg.assembly_shipments[shipment_iter].station_id
                tmp_assembly_shipment.products = msg.assembly_shipments[shipment_iter].products
                tmp_assembly_shipment.is_done  = False
                tmp_assembly_shipment.is_submit = False
                for part_itr in range(0, len(tmp_assembly_shipment.products)):
                    tmp_part = Part()
                    tmp_part.type = tmp_assembly_shipment.products[part_itr].type
                    tmp_part.pose = tmp_assembly_shipment.products[part_itr].pose
                    tmp_part.i_d  = tmp_assembly_shipment.shipment_type + ":"+tmp_part.type +":"+str(part_itr)
                    tmp_part.is_done = False    ###dqw
#                    tmp_part.is_done = True
                    tmp_assembly_shipment.products[part_itr] = tmp_part
                tmp_assembly_shipments.assembly_shipment.append(tmp_assembly_shipment)
            self.reaper_order_0.assembly_shipments = tmp_assembly_shipments
            self.order_0_ass_part_fin = [0 for _ in range(len(self.reaper_order_0.assembly_shipments.assembly_shipment))]
    
    def order_0_update_init(self,msg):
#        print("hello, is_done of the first part in order_0 is:  ")
#        print(self.reaper_order_0.kitting_shipments.kitting_shipment[0].products[0].is_done)
        self.reaper_order_0_update = My_order(msg.order_id)
        self.reaper_order_0_update.is_done =False
        self.reaper_order_0_update.abandon = False
        self.reaper_order_0_update.priority = 2
        
        tmp_withdraw_shipments = Withdraw_shipments()    ## all withdraw parts
        tmp_withdraw_shipments.is_done = False
        self.reaper_order_0_update.withdraw_shipments = tmp_withdraw_shipments

###withdraw_shipments
        
#将所有做完的零件添加到tmp_withdraw_shipments中。
#add all parts with "is_done = True " to tmp_withdraw_shipments        
        if msg.kitting_shipments == [] and msg.assembly_shipments == [] :   ##it doesn't happen.
            tmp_withdraw_shipments = Withdraw_shipments()
            tmp_withdraw_shipments.is_done = True
            self.reaper_order_0_update.withdraw_shipments = tmp_withdraw_shipments 
            
        else:
            tmp_withdraw_shipments = Withdraw_shipments()
            tmp_withdraw_shipments.is_done = False
            
            for shipment_iter in range(0, len(self.reaper_order_0.kitting_shipments.kitting_shipment)):
                tmp_withdraw_shipment = Withdraw_shipment()
                tmp_withdraw_shipment_ = Withdraw_shipment()

                #shipment_type要改
                tmp_withdraw_shipment.shipment_type = "order_0update_withdraw_shipment_" + str(shipment_iter)
                tmp_withdraw_shipment.agv_id = self.reaper_order_0.kitting_shipments.kitting_shipment[shipment_iter].agv_id
                tmp_withdraw_shipment.station_id = self.reaper_order_0.kitting_shipments.kitting_shipment[shipment_iter].station_id
                tmp_withdraw_shipment_.products = self.reaper_order_0.kitting_shipments.kitting_shipment[shipment_iter].products    ##all in 
                tmp_withdraw_shipment.is_done  = False
                tmp_withdraw_shipment.is_submit = False
                
                self.order_0_itr = 0
                for part_itr in range(0, len(tmp_withdraw_shipment_.products)):
                    tmp_part = Part()
                    if tmp_withdraw_shipment_.products[part_itr].is_done == True:    #part.is_done  == True
                        self.order_0_itr = self.order_0_itr + 1
                        tmp_part.type = tmp_withdraw_shipment_.products[part_itr].type
                        tmp_part.pose = tmp_withdraw_shipment_.products[part_itr].pose
                        #tmp_part.i_d要改
                        tmp_part.i_d  = tmp_withdraw_shipment.shipment_type + ":" + tmp_withdraw_shipment_.products[part_itr].type + ":" + str(self.order_0_itr)
                        tmp_part.is_done = False
                        tmp_withdraw_shipment.products.append(tmp_part)
                           #零件做完了，我们才把它临时加入到要撤回的订单中。   
                tmp_withdraw_shipments.withdraw_shipment.append(tmp_withdraw_shipment)
            self.reaper_order_0_update.withdraw_shipments = tmp_withdraw_shipments
            
                #对同一个kitting_shipment中的零件，肯定要加入到同一个withdraw_shipment中
            self.order_0_update_wit_part_fin = [0 for _ in range(len(self.reaper_order_0_update.withdraw_shipments.withdraw_shipment))]
           
###   kitting_shipments

        if msg.kitting_shipments == []:
            tmp_kitting_shipments = Kitting_shipments()
            tmp_kitting_shipments.is_done = True
            self.reaper_order_0_update.kitting_shipments = tmp_kitting_shipments
        else:
            ###parse
            tmp_kitting_shipments = Kitting_shipments()
            tmp_kitting_shipments.is_done = False
            
            for shipment_iter in range (0, len(msg.kitting_shipments)):
                tmp_kitting_shipment = Kitting_shipment()
                shipment_type_temp = msg.kitting_shipments[shipment_iter].shipment_type   #
                tmp_kitting_shipment.shipment_type = shipment_type_temp.replace("order_0","order_0update")  #这两行是为了修改shipment_type名
#                tmp_kitting_shipment.shipment_type = msg.kitting_shipments[shipment_iter].shipment_type
                tmp_kitting_shipment.agv_id = msg.kitting_shipments[shipment_iter].agv
                tmp_kitting_shipment.station_id = msg.kitting_shipments[shipment_iter].assembly_station
                tmp_kitting_shipment.movable_tray = Part()
                tmp_kitting_shipment.movable_tray.type = msg.kitting_shipments[shipment_iter].movable_tray.movable_tray_type
                tmp_kitting_shipment.movable_tray.pose = msg.kitting_shipments[shipment_iter].movable_tray.pose

                tmp_kitting_shipment.products = msg.kitting_shipments[shipment_iter].products
                tmp_kitting_shipment.is_done  = False
                tmp_kitting_shipment.is_submit = False
                for part_itr in range(0, len(tmp_kitting_shipment.products)):
                    tmp_part = Part()
                    tmp_part.type = tmp_kitting_shipment.products[part_itr].type
                    tmp_part.pose = tmp_kitting_shipment.products[part_itr].pose
                    tmp_part.i_d  = tmp_kitting_shipment.shipment_type + ":"+tmp_part.type +":"+str(part_itr)
                    tmp_part.is_done = False
                    ##TODO: self.reaper_order_0_update.withdraw_shipments.withdraw_shipment可能为[],但是没影响

                    for shipment_iter_inside in range(0, len(self.reaper_order_0_update.withdraw_shipments.withdraw_shipment)):       #
                        for part_itr_inside in range(0, len(self.reaper_order_0_update.withdraw_shipments.withdraw_shipment[shipment_iter_inside].products)):
                            tmp_withdraw_shipment = self.reaper_order_0_update.withdraw_shipments.withdraw_shipment[shipment_iter_inside]
                            #对一个withdraw_shipment
                            if (tmp_kitting_shipment.products[part_itr].type == tmp_withdraw_shipment.products[part_itr_inside].type) and \
                                (tmp_kitting_shipment.products[part_itr].pose == tmp_withdraw_shipment.products[part_itr_inside].pose):
                                    #意味着这两个零件相同，即该零件在新订单中出现，但是已经做完了。
                                    #将更新后的订单中对应的零件标志为置为True，将回撤订单中对应零件的标志位置为True.
                                tmp_part.is_done = True
                                tmp_withdraw_shipment.products[part_itr_inside].is_done = True
                            if (tmp_kitting_shipment.products[part_itr].type == tmp_withdraw_shipment.products[part_itr_inside].type) and \
                                (not (tmp_kitting_shipment.products[part_itr].pose == tmp_withdraw_shipment.products[part_itr_inside].pose)):
                                    #意味着这两个零件相同，但是零件在新订单中要求的位置有更改。
                                    #将更新后的订单中对应的零件标志为置为True，将回撤订单中对应零件需要调整的标志位　置为True，并将要调整到的位姿赋值。
                                tmp_part.is_done = True
                                tmp_withdraw_shipment.products[part_itr_inside].is_adjust = True
                                tmp_withdraw_shipment.products[part_itr_inside].adjust_pose = tmp_part.pose

                    tmp_kitting_shipment.products[part_itr] = tmp_part
                tmp_kitting_shipments.kitting_shipment.append(tmp_kitting_shipment)
            self.reaper_order_0_update.kitting_shipments = tmp_kitting_shipments    
            
            self.order_0_update_kit_part_fin = [0 for _ in range(len(self.reaper_order_0_update.kitting_shipments.kitting_shipment))]
            
###    assembly_shipments
        
        if msg.assembly_shipments == []:
            tmp_assembly_shipments = Assembly_shipments()
            tmp_assembly_shipments.is_done = True
            self.reaper_order_0_update.assembly_shipments = tmp_assembly_shipments
        else:
            ###parse
            tmp_assembly_shipments = Assembly_shipments()
            tmp_assembly_shipments.is_done = False
            
            for shipment_iter in range (0, len(msg.assembly_shipments)):
                tmp_assembly_shipment = Assembly_shipment()
                shipment_type_temp = msg.assembly_shipments[shipment_iter].shipment_type
                tmp_assembly_shipment.shipment_type = shipment_type_temp.replace("order_0","order_0update")
                tmp_assembly_shipment.station_id = msg.assembly_shipments[shipment_iter].station_id
                tmp_assembly_shipment.products = msg.assembly_shipments[shipment_iter].products
                tmp_assembly_shipment.is_done  = False
                tmp_assembly_shipment.is_submit = False
                for part_itr in range(0, len(tmp_assembly_shipment.products)):
                    tmp_part = Part()
                    tmp_part.type = tmp_assembly_shipment.products[part_itr].type
                    tmp_part.pose = tmp_assembly_shipment.products[part_itr].pose
                    tmp_part.i_d  = tmp_assembly_shipment.shipment_type + ":"+tmp_part.type +":"+str(part_itr)
                    tmp_part.is_done = False      
                    
                    tmp_assembly_shipment.products[part_itr] = tmp_part

                tmp_assembly_shipments.assembly_shipment.append(tmp_assembly_shipment)
            self.reaper_order_0_update.assembly_shipments = tmp_assembly_shipments
            self.order_0_update_ass_part_fin = [0 for _ in range(len(self.reaper_order_0_update.assembly_shipments.assembly_shipment))]
            
        print("First, the withdraw_shipment.is_done of order_0_update is:")
        print(self.reaper_order_0_update.withdraw_shipments.is_done)
        
        
    def order_1_init(self,msg):

        self.reaper_order_1 = My_order(msg.order_id)
        self.reaper_order_1.is_done =False
        self.reaper_order_1.abandon = False
        self.reaper_order_1.priority = 3
        
        tmp_withdraw_shipments = Withdraw_shipments()    ## all withdraw parts
        tmp_withdraw_shipments.is_done = True
        self.reaper_order_1.withdraw_shipments = tmp_withdraw_shipments
        
        if msg.kitting_shipments == []:
            tmp_kitting_shipments = Kitting_shipments()
            tmp_kitting_shipments.is_done = True
            self.reaper_order_1.kitting_shipments = tmp_kitting_shipments
        else:
            ###parse
            tmp_kitting_shipments = Kitting_shipments()
            tmp_kitting_shipments.is_done = False
            
            for shipment_iter in range (0, len(msg.kitting_shipments)):
                tmp_kitting_shipment = Kitting_shipment()
                tmp_kitting_shipment.shipment_type = msg.kitting_shipments[shipment_iter].shipment_type
                tmp_kitting_shipment.agv_id = msg.kitting_shipments[shipment_iter].agv
                tmp_kitting_shipment.station_id = msg.kitting_shipments[shipment_iter].assembly_station
                tmp_kitting_shipment.movable_tray = Part()
                tmp_kitting_shipment.movable_tray.type = msg.kitting_shipments[shipment_iter].movable_tray.movable_tray_type
                tmp_kitting_shipment.movable_tray.pose = msg.kitting_shipments[shipment_iter].movable_tray.pose

                if tmp_kitting_shipment.agv_id == 'agv1':
                    self.agv_1_needed_tray = tmp_kitting_shipment.movable_tray.type
                elif tmp_kitting_shipment.agv_id == 'agv2':
                    self.agv_2_needed_tray = tmp_kitting_shipment.movable_tray.type
                elif tmp_kitting_shipment.agv_id == 'agv3':
                    self.agv_3_needed_tray = tmp_kitting_shipment.movable_tray.type
                elif tmp_kitting_shipment.agv_id == 'agv4':
                    self.agv_4_needed_tray = tmp_kitting_shipment.movable_tray.type

                tmp_kitting_shipment.products = msg.kitting_shipments[shipment_iter].products
                tmp_kitting_shipment.is_done  = False
                tmp_kitting_shipment.is_submit = False
                for part_itr in range(0, len(tmp_kitting_shipment.products)):
                    tmp_part = Part()
                    tmp_part.type = tmp_kitting_shipment.products[part_itr].type
                    tmp_part.pose = tmp_kitting_shipment.products[part_itr].pose
                    tmp_part.i_d  = tmp_kitting_shipment.shipment_type + ":"+tmp_part.type +":"+str(part_itr)
                    tmp_part.is_done = False
                    tmp_kitting_shipment.products[part_itr] = tmp_part
                tmp_kitting_shipments.kitting_shipment.append(tmp_kitting_shipment)
            self.reaper_order_1.kitting_shipments = tmp_kitting_shipments
            self.order_1_kit_part_fin = [0 for _ in range(len(self.reaper_order_1.kitting_shipments.kitting_shipment))]
        
        if msg.assembly_shipments == []:
            tmp_assembly_shipments = Assembly_shipments()
            tmp_assembly_shipments.is_done = True
            self.reaper_order_1.assembly_shipments = tmp_assembly_shipments
        else:
            ###parse
            tmp_assembly_shipments = Assembly_shipments()
            tmp_assembly_shipments.is_done = False
            
            for shipment_iter in range (0, len(msg.assembly_shipments)):
                tmp_assembly_shipment = Assembly_shipment()
                tmp_assembly_shipment.shipment_type = msg.assembly_shipments[shipment_iter].shipment_type
                tmp_assembly_shipment.station_id = msg.assembly_shipments[shipment_iter].station_id
                tmp_assembly_shipment.products = msg.assembly_shipments[shipment_iter].products
                tmp_assembly_shipment.is_done  = False
                tmp_assembly_shipment.is_submit = False
                for part_itr in range(0, len(tmp_assembly_shipment.products)):
                    tmp_part = Part()
                    tmp_part.type = tmp_assembly_shipment.products[part_itr].type
                    tmp_part.pose = tmp_assembly_shipment.products[part_itr].pose
                    tmp_part.i_d  = tmp_assembly_shipment.shipment_type + ":" + tmp_part.type + ":" + str(part_itr)
                    tmp_part.is_done = False
                    tmp_assembly_shipment.products[part_itr] = tmp_part
                tmp_assembly_shipments.assembly_shipment.append(tmp_assembly_shipment)
            self.reaper_order_1.assembly_shipments = tmp_assembly_shipments
            self.order_1_ass_part_fin = [0 for _ in range(len(self.reaper_order_1.assembly_shipments.assembly_shipment))]
            
    def order_1_update_init(self,msg):

        self.reaper_order_1_update = My_order(msg.order_id)
        self.reaper_order_1_update.is_done =False
        self.reaper_order_1_update.abandon = False
        self.reaper_order_1_update.priority = 4
        tmp_withdraw_shipments = Withdraw_shipments()    ## all withdraw parts
        tmp_withdraw_shipments.is_done = False
        self.reaper_order_1_update.withdraw_shipments = tmp_withdraw_shipments

###withdraw_shipments
#add all parts with "is_done = True " to tmp_withdraw_shipments
        
        if msg.kitting_shipments == [] and msg.assembly_shipments == [] :
            tmp_withdraw_shipments = Withdraw_shipments()
            tmp_withdraw_shipments.is_done = True
            self.reaper_order_1_update.withdraw_shipments = tmp_withdraw_shipments 
            
        else:
            tmp_withdraw_shipments = Withdraw_shipments()
            tmp_withdraw_shipments.is_done = False
            
            for shipment_iter in range(0, len(self.reaper_order_1.kitting_shipments.kitting_shipment)):
                tmp_withdraw_shipment = Withdraw_shipment()
                tmp_withdraw_shipment_ = Withdraw_shipment()
                
                tmp_withdraw_shipment.shipment_type = "order_1update_withdraw_shipment_" + str(shipment_iter)
                tmp_withdraw_shipment.agv_id = self.reaper_order_1.kitting_shipments.kitting_shipment[shipment_iter].agv_id
                tmp_withdraw_shipment.station_id = self.reaper_order_1.kitting_shipments.kitting_shipment[shipment_iter].station_id
                tmp_withdraw_shipment_.products = self.reaper_order_1.kitting_shipments.kitting_shipment[shipment_iter].products
                tmp_withdraw_shipment.is_done  = False
                tmp_withdraw_shipment.is_submit = False
                
                self.order_1_itr = 0
                for part_itr in range(0, len(tmp_withdraw_shipment_.products)):
                    tmp_part = Part()
                    if tmp_withdraw_shipment_.products[part_itr].is_done == True:
                        self.order_1_itr = self.order_1_itr + 1
                        tmp_part.type = tmp_withdraw_shipment_.products[part_itr].type
                        tmp_part.pose = tmp_withdraw_shipment_.products[part_itr].pose
#                        tmp_part.i_d  = tmp_withdraw_shipment.shipment_type + ":" + tmp_withdraw_shipment_.products[part_itr].type + ":" + str(part_itr)
                        tmp_part.i_d  = tmp_withdraw_shipment.shipment_type + ":" + tmp_withdraw_shipment_.products[part_itr].type + ":" + str(self.order_1_itr)
                        tmp_part.is_done = False
                        tmp_withdraw_shipment.products.append(tmp_part)
                        
                tmp_withdraw_shipments.withdraw_shipment.append(tmp_withdraw_shipment)
            self.reaper_order_1_update.withdraw_shipments = tmp_withdraw_shipments
            self.order_1_update_wit_part_fin = [0 for _ in range(len(self.reaper_order_1_update.withdraw_shipments.withdraw_shipment))]
            
###   kitting_shipments

        if msg.kitting_shipments == []:
            tmp_kitting_shipments = Kitting_shipments()
            tmp_kitting_shipments.is_done = True
            self.reaper_order_1_update.kitting_shipments = tmp_kitting_shipments
        else:
            ###parse
            tmp_kitting_shipments = Kitting_shipments()
            tmp_kitting_shipments.is_done = False
            
            for shipment_iter in range (0, len(msg.kitting_shipments)):
                tmp_kitting_shipment = Kitting_shipment()
                shipment_type_temp = msg.kitting_shipments[shipment_iter].shipment_type
                tmp_kitting_shipment.shipment_type = shipment_type_temp.replace("order_1","order_1update")
#                tmp_kitting_shipment.shipment_type = msg.kitting_shipments[shipment_iter].shipment_type
                tmp_kitting_shipment.agv_id = msg.kitting_shipments[shipment_iter].agv
                tmp_kitting_shipment.station_id = msg.kitting_shipments[shipment_iter].assembly_station
                tmp_kitting_shipment.movable_tray = Part()
                tmp_kitting_shipment.movable_tray.type = msg.kitting_shipments[shipment_iter].movable_tray.movable_tray_type
                tmp_kitting_shipment.movable_tray.pose = msg.kitting_shipments[shipment_iter].movable_tray.pose

                tmp_kitting_shipment.products = msg.kitting_shipments[shipment_iter].products
                tmp_kitting_shipment.is_done  = False
                tmp_kitting_shipment.is_submit = False
                for part_itr in range(0, len(tmp_kitting_shipment.products)):
                    tmp_part = Part()
                    tmp_part.type = tmp_kitting_shipment.products[part_itr].type
                    tmp_part.pose = tmp_kitting_shipment.products[part_itr].pose
                    tmp_part.i_d  = tmp_kitting_shipment.shipment_type + ":"+tmp_part.type +":"+str(part_itr)
                    tmp_part.is_done = False
                    
                    for shipment_iter_inside in range(0, len(self.reaper_order_1_update.withdraw_shipments.withdraw_shipment) ):       ###dqw
                        for part_itr_inside in range(0, len(self.reaper_order_1_update.withdraw_shipments.withdraw_shipment[shipment_iter_inside].products)):
                            tmp_withdraw_shipment = self.reaper_order_1_update.withdraw_shipments.withdraw_shipment[shipment_iter_inside]
                            if (tmp_kitting_shipment.products[part_itr].type == tmp_withdraw_shipment.products[part_itr_inside].type) and \
                                (tmp_kitting_shipment.products[part_itr].pose == tmp_withdraw_shipment.products[part_itr_inside].pose):
                                tmp_part.is_done = True
                                tmp_withdraw_shipment.products[part_itr_inside].is_done = True
                            if (tmp_kitting_shipment.products[part_itr].type == tmp_withdraw_shipment.products[part_itr_inside].type) and \
                                (not (tmp_kitting_shipment.products[part_itr].pose == tmp_withdraw_shipment.products[part_itr_inside].pose)):
                                tmp_part.is_done = True
                                tmp_withdraw_shipment.products[part_itr_inside].is_adjust = True
                                tmp_withdraw_shipment.products[part_itr_inside].adjust_pose = tmp_part.pose
                                
                    tmp_kitting_shipment.products[part_itr] = tmp_part
                
                tmp_kitting_shipments.kitting_shipment.append(tmp_kitting_shipment)
            self.reaper_order_1_update.kitting_shipments = tmp_kitting_shipments 
            self.order_1_update_kit_part_fin = [0 for _ in range(len(self.reaper_order_1_update.kitting_shipments.kitting_shipment))]

###    assembly_shipments
        
        if msg.assembly_shipments == []:
            tmp_assembly_shipments = Assembly_shipments()
            tmp_assembly_shipments.is_done = True
            self.reaper_order_1_update.assembly_shipments = tmp_assembly_shipments
        else:
            ###parse
            tmp_assembly_shipments = Assembly_shipments()
            tmp_assembly_shipments.is_done = False
            
            for shipment_iter in range (0, len(msg.assembly_shipments)):
                tmp_assembly_shipment = Assembly_shipment()
                shipment_type_temp = msg.assembly_shipments[shipment_iter].shipment_type
                tmp_assembly_shipment.shipment_type = shipment_type_temp.replace("order_1","order_1update")
#                tmp_assembly_shipment.shipment_type = msg.assembly_shipments[shipment_iter].shipment_type
                tmp_assembly_shipment.station_id = msg.assembly_shipments[shipment_iter].station_id
                tmp_assembly_shipment.products = msg.assembly_shipments[shipment_iter].products
                tmp_assembly_shipment.is_done  = False
                tmp_assembly_shipment.is_submit = False
                for part_itr in range(0, len(tmp_assembly_shipment.products)):
                    tmp_part = Part()
                    tmp_part.type = tmp_assembly_shipment.products[part_itr].type
                    tmp_part.pose = tmp_assembly_shipment.products[part_itr].pose
                    tmp_part.i_d  = tmp_assembly_shipment.shipment_type + ":"+tmp_part.type +":"+str(part_itr)
                    tmp_part.is_done = False
                
                    tmp_assembly_shipment.products[part_itr] = tmp_part
                tmp_assembly_shipments.assembly_shipment.append(tmp_assembly_shipment)
            self.reaper_order_1_update.assembly_shipments = tmp_assembly_shipments
            self.order_1_update_ass_part_fin = [0 for _ in range(len(self.reaper_order_1_update.assembly_shipments.assembly_shipment))]

            
    def part_update(self, part_id_, is_done_ = True):
        # set the part as True
        #eg. part_id_ = "order_0_assembly_shipment_0:assembly_battery_green:0"
        #eg. part_id = "order_0update_assembly_shipment_0:assembly_battery_green:0"
        # print("hello, this is part update")
        # print rospy.get_time()
        #is_done_ = True or False
        
        first_ = part_id_.index("_")
        second_ = part_id_.index("_", first_ + 1)
        third_ = part_id_.index("_", second_ + 1)
        fourth_ = part_id_.index("_", third_ + 1)
        fifth_ = part_id_.index(":", fourth_ + 1)
        sixth_ = part_id_.index(":", fifth_ + 1)
#        seventh = part_id_.index("_", sixth_ + 1)
            
        order_ = part_id_[0 : second_]       #"order_0","order_0update","order_1","order_1_update"
        shipment_type_1 = part_id_[second_ + 1 : fourth_]

        if order_ == "order_0":
            order_id_tmp ="order_0"
        if order_ == "order_0update":
            order_id_tmp ="order_0_update"
        if order_ == "order_1":
            order_id_tmp ="order_1"
        if order_ == "order_1update":
            order_id_tmp ="order_1_update"
            
            
        if order_id_tmp == "order_0":
            if shipment_type_1 == "kitting_shipment":    ##dqw
                for shipment_itr in range(0, len(self.reaper_order_0.kitting_shipments.kitting_shipment)):
                    for part_itr in range(0, len(self.reaper_order_0.kitting_shipments.kitting_shipment[shipment_itr].products)):
                        if part_id_ == self.reaper_order_0.kitting_shipments.kitting_shipment[shipment_itr].products[part_itr].i_d:
                            self.reaper_order_0.kitting_shipments.kitting_shipment[shipment_itr].products[part_itr].is_done = is_done_   #零件标志位更新
                            
                            #以下是对shipment进行更新。
                            for order in self.order_list:     #先判断是不是有订单更新
                                if order.order_id == "order_0_update":
                                    pass    
                                elif order.order_id == "order_0":          #如果没有订单更新   #遍历该shipment中的所有的零件，检查标志位是不是为True。如果全为True,更新shipment标志位
                                    for shipment_itr_ in range(0, len(self.reaper_order_0.kitting_shipments.kitting_shipment)):
                                        self.order_0_kit_part_fin[shipment_itr_] = 0
#                                        amount = 0
                                        for part_itr_ in range(0, len(self.reaper_order_0.kitting_shipments.kitting_shipment[shipment_itr_].products)):
                                            if self.reaper_order_0.kitting_shipments.kitting_shipment[shipment_itr_].products[part_itr_].is_done == True:
                                                self.order_0_kit_part_fin[shipment_itr_] = self.order_0_kit_part_fin[shipment_itr_] + 1
                                                
                                                
                                                if self.order_0_kit_part_fin[shipment_itr_] == len(self.reaper_order_0.kitting_shipments.kitting_shipment[shipment_itr_].products):
                                                    self.reaper_order_0.kitting_shipments.kitting_shipment[shipment_itr_].is_done = True   #shipment的标志位更新
                                                    # print("self.reaper_order_0.kitting_shipments.kitting_shipment[shipment_itr_].is_done" , \
                                                    #       self.reaper_order_0.kitting_shipments.kitting_shipment[shipment_itr_].is_done )
            elif shipment_type_1 == "assembly_shipment":
                for shipment_itr in range(0, len(self.reaper_order_0.assembly_shipments.assembly_shipment)):
                    for part_itr in range(0, len(self.reaper_order_0.assembly_shipments.assembly_shipment[shipment_itr].products)):
                        if part_id_ == self.reaper_order_0.assembly_shipments.assembly_shipment[shipment_itr].products[part_itr].i_d:
                            self.reaper_order_0.assembly_shipments.assembly_shipment[shipment_itr].products[part_itr].is_done = is_done_
                            #以下是对shipment进行更新。
                            for order in self.order_list:     #先判断是不是有订单更新
                                if order.order_id == "order_0_update":
                                    pass    
                                elif order.order_id == "order_0":          #如果没有订单更新   #遍历该shipment中的所有的零件，检查标志位是不是为True。如果全为True,更新shipment标志位
                                    for shipment_itr_ in range(0, len(self.reaper_order_0.assembly_shipments.assembly_shipment)):
                                        self.order_0_ass_part_fin[shipment_itr_] = 0
                                        for part_itr_ in range(0, len(self.reaper_order_0.assembly_shipments.assembly_shipment[shipment_itr_].products)):
                                            if self.reaper_order_0.assembly_shipments.assembly_shipment[shipment_itr_].products[part_itr_].is_done == True:
                                                self.order_0_ass_part_fin[shipment_itr_] = self.order_0_ass_part_fin[shipment_itr_] + 1
                                                if self.order_0_ass_part_fin[shipment_itr_] == len(self.reaper_order_0.assembly_shipments.assembly_shipment[shipment_itr_].products):
                                                    self.reaper_order_0.assembly_shipments.assembly_shipment[shipment_itr_].is_done = True   #shipment的标志位更新

        elif order_id_tmp == "order_0_update":   #
            if shipment_type_1 == "kitting_shipment":    ##dqw
                for shipment_itr in range(0, len(self.reaper_order_0_update.kitting_shipments.kitting_shipment)):
                    for part_itr in range(0, len(self.reaper_order_0_update.kitting_shipments.kitting_shipment[shipment_itr].products)):
                        if part_id_ == self.reaper_order_0_update.kitting_shipments.kitting_shipment[shipment_itr].products[part_itr].i_d:
                            self.reaper_order_0_update.kitting_shipments.kitting_shipment[shipment_itr].products[part_itr].is_done = is_done_  
                            #以下对shipment进行更新。
                            for shipment_itr_ in range(0, len(self.reaper_order_0_update.kitting_shipments.kitting_shipment)):
                                self.order_0_update_kit_part_fin[shipment_itr_] = 0
                                for part_itr_ in range(0, len(self.reaper_order_0_update.kitting_shipments.kitting_shipment[shipment_itr_].products)):
                                    if self.reaper_order_0_update.kitting_shipments.kitting_shipment[shipment_itr_].products[part_itr_].is_done == True:
                                        self.order_0_update_kit_part_fin[shipment_itr_] = self.order_0_update_kit_part_fin[shipment_itr_] + 1

                                        if self.order_0_update_kit_part_fin[shipment_itr_] == len(self.reaper_order_0_update.kitting_shipments.kitting_shipment[shipment_itr_].products):
                                            self.reaper_order_0_update.kitting_shipments.kitting_shipment[shipment_itr_].is_done = True   #shipment的标志位更新

                                            
            elif shipment_type_1 == "assembly_shipment":
                for shipment_itr in range(0, len(self.reaper_order_0_update.assembly_shipments.assembly_shipment)):
                    for part_itr in range(0, len(self.reaper_order_0_update.assembly_shipments.assembly_shipment[shipment_itr].products)):
                        if part_id_ == self.reaper_order_0_update.assembly_shipments.assembly_shipment[shipment_itr].products[part_itr].i_d:
                            self.reaper_order_0_update.assembly_shipments.assembly_shipment[shipment_itr].products[part_itr].is_done = is_done_
                            #以下对shipment进行更新。
                            for shipment_itr_ in range(0, len(self.reaper_order_0_update.assembly_shipments.assembly_shipment)):
                                self.order_0_update_ass_part_fin[shipment_itr_] = 0
                                for part_itr_ in range(0, len(self.reaper_order_0_update.assembly_shipments.assembly_shipment[shipment_itr_].products)):
                                    if self.reaper_order_0_update.assembly_shipments.assembly_shipment[shipment_itr_].products[part_itr_].is_done == True:
                                        self.order_0_update_ass_part_fin[shipment_itr_] = self.order_0_update_ass_part_fin[shipment_itr_] + 1
                                        if self.order_0_update_ass_part_fin[shipment_itr_] == len(self.reaper_order_0_update.assembly_shipments.assembly_shipment[shipment_itr_].products):
                                            self.reaper_order_0_update.assembly_shipments.assembly_shipment[shipment_itr_].is_done = True   #shipment的标志位更新

#要修改的地方

            elif shipment_type_1 == "withdraw_shipment":
                for shipment_itr in range(0, len(self.reaper_order_0_update.withdraw_shipments.withdraw_shipment)):
                    for part_itr in range(0, len(self.reaper_order_0_update.withdraw_shipments.withdraw_shipment[shipment_itr].products)):                        
                        if part_id_ == self.reaper_order_0_update.withdraw_shipments.withdraw_shipment[shipment_itr].products[part_itr].i_d:
                            self.reaper_order_0_update.withdraw_shipments.withdraw_shipment[shipment_itr].products[part_itr].is_done = is_done_  
                            #以下对shipment进行更新。
                            
                            for shipment_itr_ in range(0, len(self.reaper_order_0_update.withdraw_shipments.withdraw_shipment)):
                                self.order_0_update_wit_part_fin[shipment_itr_] = 0
                                for part_itr_ in range(0, len(self.reaper_order_0_update.withdraw_shipments.withdraw_shipment[shipment_itr_].products)):
                                    if self.reaper_order_0_update.withdraw_shipments.withdraw_shipment[shipment_itr_].products[part_itr_].is_done == True:
                                        self.order_0_update_wit_part_fin[shipment_itr_] = self.order_0_update_wit_part_fin[shipment_itr_] + 1
      
                                        if self.order_0_update_wit_part_fin[shipment_itr_] == len(self.reaper_order_0_update.withdraw_shipments.withdraw_shipment[shipment_itr_].products):
                                            self.reaper_order_0_update.withdraw_shipments.withdraw_shipment[shipment_itr_].is_done = True   #shipment的标志位更新
                                            
                                            #为了检查上边改的这段代码有没有问题
#                                            print("self.reaper_order_0_update.withdraw_shipments.withdraw_shipment" + str(shipment_itr_) + ".is_done:",
#                                                  self.reaper_order_0_update.withdraw_shipments.withdraw_shipment[shipment_itr_].is_done)
        
        elif order_id_tmp == "order_1":
            if shipment_type_1 == "kitting_shipment":    ##dqw
                for shipment_itr in range(0, len(self.reaper_order_1.kitting_shipments.kitting_shipment)):
                    for part_itr in range(0, len(self.reaper_order_1.kitting_shipments.kitting_shipment[shipment_itr].products)):
                        if part_id_ == self.reaper_order_1.kitting_shipments.kitting_shipment[shipment_itr].products[part_itr].i_d:
                            self.reaper_order_1.kitting_shipments.kitting_shipment[shipment_itr].products[part_itr].is_done = is_done_                 
                            #以下是对shipment进行更新。
                            for order in self.order_list:     #先判断是不是有订单更新
                                if order.order_id == "order_1_update":
                                    pass    
                                elif order.order_id == "order_1":          #如果没有订单更新   #遍历该shipment中的所有的零件，检查标志位是不是为True。如果全为True,更新shipment标志位
                                    for shipment_itr_ in range(0, len(self.reaper_order_1.kitting_shipments.kitting_shipment)):
                                        self.order_1_kit_part_fin[shipment_itr_] = 0
                                        for part_itr_ in range(0, len(self.reaper_order_1.kitting_shipments.kitting_shipment[shipment_itr_].products)):
                                            if self.reaper_order_1.kitting_shipments.kitting_shipment[shipment_itr_].products[part_itr_].is_done == True:
                                                self.order_1_kit_part_fin[shipment_itr_] = self.order_1_kit_part_fin[shipment_itr_] + 1
                                                if self.order_1_kit_part_fin[shipment_itr_] == len(self.reaper_order_1.kitting_shipments.kitting_shipment[shipment_itr_].products):
                                                    self.reaper_order_1.kitting_shipments.kitting_shipment[shipment_itr_].is_done = True   #shipment的标志位更新
            elif shipment_type_1 == "assembly_shipment":
                for shipment_itr in range(0, len(self.reaper_order_1.assembly_shipments.assembly_shipment)):
                    for part_itr in range(0, len(self.reaper_order_1.assembly_shipments.assembly_shipment[shipment_itr].products)):
                        if part_id_ == self.reaper_order_1.assembly_shipments.assembly_shipment[shipment_itr].products[part_itr].i_d:
                            self.reaper_order_1.assembly_shipments.assembly_shipment[shipment_itr].products[part_itr].is_done = is_done_
                            #以下是对shipment进行更新。
                            for order in self.order_list:     #先判断是不是有订单更新
                                if order.order_id == "order_1_update":
                                    pass    
                                elif order.order_id == "order_1":          #如果没有订单更新   #遍历该shipment中的所有的零件，检查标志位是不是为True。如果全为True,更新shipment标志位
                                    for shipment_itr_ in range(0, len(self.reaper_order_1.assembly_shipments.assembly_shipment)):
                                        self.order_1_ass_part_fin[shipment_itr_] = 0
                                        for part_itr_ in range(0, len(self.reaper_order_1.assembly_shipments.assembly_shipment[shipment_itr_].products)):
                                            if self.reaper_order_1.assembly_shipments.assembly_shipment[shipment_itr_].products[part_itr_].is_done == True:
                                                self.order_1_ass_part_fin[shipment_itr_] = self.order_1_ass_part_fin[shipment_itr_] + 1
                                                if self.order_1_ass_part_fin[shipment_itr_] == len(self.reaper_order_1.assembly_shipments.assembly_shipment[shipment_itr_].products):
                                                    self.reaper_order_1.assembly_shipments.assembly_shipment[shipment_itr_].is_done = True   #shipment的标志位更新

        elif order_id_tmp == "order_1_update":
            if shipment_type_1 == "kitting_shipment":    ##dqw
                for shipment_itr in range(0, len(self.reaper_order_1_update.kitting_shipments.kitting_shipment)):
                    for part_itr in range(0, len(self.reaper_order_1_update.kitting_shipments.kitting_shipment[shipment_itr].products)):
                        if part_id_ == self.reaper_order_1_update.kitting_shipments.kitting_shipment[shipment_itr].products[part_itr].i_d:
                            self.reaper_order_1_update.kitting_shipments.kitting_shipment[shipment_itr].products[part_itr].is_done = is_done_   
                            #以下对shipment进行更新。
                            for shipment_itr_ in range(0, len(self.reaper_order_1_update.kitting_shipments.kitting_shipment)):
                                self.order_1_update_kit_part_fin[shipment_itr_] = 0
                                for part_itr_ in range(0, len(self.reaper_order_1_update.kitting_shipments.kitting_shipment[shipment_itr_].products)):
                                    if self.reaper_order_1_update.kitting_shipments.kitting_shipment[shipment_itr_].products[part_itr_].is_done == True:
                                        self.order_1_update_kit_part_fin[shipment_itr_] = self.order_1_update_kit_part_fin[shipment_itr_] + 1
                                        if self.order_1_update_kit_part_fin[shipment_itr_] == len(self.reaper_order_1_update.kitting_shipments.kitting_shipment[shipment_itr_].products):
                                            self.reaper_order_1_update.kitting_shipments.kitting_shipment[shipment_itr_].is_done = True   #shipment的标志位更新
                            
            elif shipment_type_1 == "assembly_shipment":
                for shipment_itr in range(0, len(self.reaper_order_1_update.assembly_shipments.assembly_shipment)):
                    for part_itr in range(0, len(self.reaper_order_1_update.assembly_shipments.assembly_shipment[shipment_itr].products)):
                        if part_id_ == self.reaper_order_1_update.assembly_shipments.assembly_shipment[shipment_itr].products[part_itr].i_d:
                            self.reaper_order_1_update.assembly_shipments.assembly_shipment[shipment_itr].products[part_itr].is_done = is_done_
                            
                            #以下对shipment进行更新。
                            for shipment_itr_ in range(0, len(self.reaper_order_1_update.assembly_shipments.assembly_shipment)):
                                self.order_1_update_ass_part_fin[shipment_itr_] =0
                                for part_itr_ in range(0, len(self.reaper_order_1_update.assembly_shipments.assembly_shipment[shipment_itr_].products)):
                                    if self.reaper_order_1_update.assembly_shipments.assembly_shipment[shipment_itr_].products[part_itr_].is_done == True:
                                        self.order_1_update_ass_part_fin[shipment_itr_] = self.order_1_update_ass_part_fin[shipment_itr_] + 1
                                        if self.order_1_update_ass_part_fin[shipment_itr_] == len(self.reaper_order_1_update.assembly_shipments.assembly_shipment[shipment_itr_].products):
                                            self.reaper_order_1_update.assembly_shipments.assembly_shipment[shipment_itr_].is_done = True   #shipment的标志位更新

            elif shipment_type_1 == "withdraw_shipment":
                for shipment_itr in range(0, len(self.reaper_order_1_update.withdraw_shipments.withdraw_shipment)):
                    for part_itr in range(0, len(self.reaper_order_1_update.withdraw_shipments.withdraw_shipment[shipment_itr].products)):
                        if part_id_ == self.reaper_order_1_update.withdraw_shipments.withdraw_shipment[shipment_itr].products[part_itr].i_d:
                            self.reaper_order_1_update.withdraw_shipments.withdraw_shipment[shipment_itr].products[part_itr].is_done = is_done_
                            #以下对shipment进行更新。印象中,每个withdraw_shipment中只有一个零件。该零件执行完，即可更新shipment的标志。 #可再确定一下。
                            
                            for shipment_itr_ in range(0, len(self.reaper_order_1_update.withdraw_shipments.withdraw_shipment)):
                                self.order_1_update_wit_part_fin[shipment_itr_] = 0
                                for part_itr_ in range(0, len(self.reaper_order_1_update.withdraw_shipments.withdraw_shipment[shipment_itr_].products)):
                                    if self.reaper_order_1_update.withdraw_shipments.withdraw_shipment[shipment_itr_].products[part_itr_].is_done == True:
                                        self.order_1_update_wit_part_fin[shipment_itr_] = self.order_1_update_wit_part_fin[shipment_itr_] + 1
                                        if self.order_1_update_wit_part_fin[shipment_itr_] == len(self.reaper_order_1_update.withdraw_shipments.withdraw_shipment[shipment_itr_].products):
                                            self.reaper_order_1_update.withdraw_shipments.withdraw_shipment[shipment_itr_].is_done = True   #shipment的标志位更新

        if is_done_ == True:
            self.withdraw_update(part_id_)
        else:
            pass
        
                
    def shipments_is_done(self, order_id, shipments_type):
        #order_id: "order_0","order_0_update", "order_0","order_0_update"
        #shipments_type: "kitting_shipments", "assembly_shipments", "withdraw_shipments"
        
        if order_id == "order_0":
            if shipments_type == "kitting_shipments":
                self.reaper_order_0.kitting_shipments.is_done = True
            if shipments_type == "assembly_shipments":
                self.reaper_order_0.assembly_shipments.is_done = True
                
        if order_id == "order_0_update":
            if shipments_type == "kitting_shipments":
                self.reaper_order_0_update.kitting_shipments.is_done = True
            if shipments_type == "assembly_shipments":
                self.reaper_order_0_update.assembly_shipments.is_done = True
            if shipments_type == "withdraw_shipments":
                self.reaper_order_0_update.withdraw_shipments.is_done = True

        if order_id == "order_1":
            if shipments_type == "kitting_shipments":
                self.reaper_order_1.kitting_shipments.is_done = True
            if shipments_type == "assembly_shipments":
                self.reaper_order_1.assembly_shipments.is_done = True
                
        if order_id == "order_1_update":
            if shipments_type == "kitting_shipments":
                self.reaper_order_1_update.kitting_shipments.is_done = True
            if shipments_type == "assembly_shipments":
                self.reaper_order_1_update.assembly_shipments.is_done = True
            if shipments_type == "withdraw_shipments":
                self.reaper_order_1_update.withdraw_shipments.is_done = True
                
                
                
    def shipment_is_submit(self, shipment_type):
        #shipment_type是唯一的。
        #eg: shipment_ = "order_0_kitting_shipment_0"
        #eg: shipment_ = "order_0update_withdraw_shipment_0"
           # shipment_type = order_0_assembly_shipment_0

        print("shipment_type",shipment_type)
        first_ = shipment_type.index("_")
        second_ = shipment_type.index("_", first_ + 1)
        third_ = shipment_type.index("_", second_ + 1)
        fourth_ = shipment_type.index("_", third_ + 1)
        
        order_ = shipment_type[0 : second_]
        shipment_type_ = shipment_type[second_ + 1 : fourth_]     #kitting_shipment
        shipment_type_itr = int(shipment_type[fourth_+1 : ])
        print("shipment_type_itr",shipment_type_itr)
        
        if order_ == "order_0":
            order_id_tmp ="order_0"
        if order_ == "order_0update":
            order_id_tmp ="order_0_update"
        if order_ == "order_1":
            order_id_tmp ="order_1"
        if order_ == "order_1update":
            order_id_tmp ="order_1_update"
            
        if order_id_tmp == "order_0":
            if shipment_type_ == "kitting_shipment":
                self.reaper_order_0.kitting_shipments.kitting_shipment[shipment_type_itr].is_submit == True
            if shipment_type_ == "assembly_shipment":
                self.reaper_order_0.assembly_shipments.assembly_shipment[shipment_type_itr].is_submit == True
                
        if order_id_tmp == "order_0_update":
            if shipment_type_ == "kitting_shipment":
                self.reaper_order_0_update.kitting_shipments.kitting_shipment[shipment_type_itr].is_submit == True
            if shipment_type_ == "assembly_shipment":
                self.reaper_order_0_update.assembly_shipments.assembly_shipment[shipment_type_itr].is_submit == True
        
        if order_id_tmp == "order_1":
            if shipment_type_ == "kitting_shipment":
                self.reaper_order_1.kitting_shipments.kitting_shipment[shipment_type_itr].is_submit == True
            if shipment_type_ == "assembly_shipment":
                self.reaper_order_1.assembly_shipments.assembly_shipment[shipment_type_itr].is_submit == True
                
        if order_id_tmp == "order_1_update":
            if shipment_type_ == "kitting_shipment":
                self.reaper_order_1_update.kitting_shipments.kitting_shipment[shipment_type_itr].is_submit == True
            if shipment_type_ == "assembly_shipment":
                self.reaper_order_1_update.assembly_shipments.assembly_shipment[shipment_type_itr].is_submit == True
        
    def order_is_done(self, order_id):
        #order.id = "order_0","order_0_update"
        #将指定的order设置为True.
        if order_id == "order_0":
            self.reaper_order_0.is_done = True
        if order_id == "order_0_update":
            self.reaper_order_0_update.is_done = True
        if order_id == "order_1":
            self.reaper_order_1.is_done = True
        if order_id == "order_1_update":
            self.reaper_order_1_update.is_done = True
    
    def withdraw_update(self, part_id_):
#        print("hello, this is withdraw update!")
#        print(part_id_)
#        print rospy.get_time()
        #eg. part_id_ = "order_0_assembly_shipment_0:assembly_battery_green:0"
        #order.id = "order_0","order_0_update"
        
        first_ = part_id_.index("_")        
        second_ = part_id_.index("_", first_ + 1)
        third_ = part_id_.index("_", second_ + 1)
        fourth_ = part_id_.index("_", third_ + 1)
        fifth_ = part_id_.index(":", fourth_ + 1)
        sixth_ = part_id_.index(":", fifth_ + 1)
#        seventh = part_id_.index("_", sixth_ + 1)

        order_ = part_id_[0 : second_]     #order_0
        shipment_type_ = part_id_[second_ + 1 : fourth_]  #assembly_shipment
        
        if order_ == "order_0":
            order_id_tmp ="order_0"
        if order_ == "order_0update":
            order_id_tmp ="order_0_update"
        if order_ == "order_1":
            order_id_tmp ="order_1"
        if order_ == "order_1update":
            order_id_tmp ="order_1_update"

        for order in self.order_list:
            if order.order_id == "order_0_update":       #意味着order_0的更新订单已经下发
                if order_id_tmp == "order_0":            #却还要更新旧订单中的零件，将该零件添加到withdraw中。
                    if shipment_type_ == "kitting_shipment":       #如果更新的零件是kitting_shipment
                        for shipment_itr in range(0, len(self.reaper_order_0.kitting_shipments.kitting_shipment)):  #１、找到该零件
                            shipment_temp = self.reaper_order_0.kitting_shipments.kitting_shipment[shipment_itr]
                            for part_itr in range(0, len(shipment_temp.products)):
                                if shipment_temp.products[part_itr].i_d == part_id_:
                                    withdraw_part = Part()
                                    withdraw_part = shipment_temp.products[part_itr]


                                    withdraw_part.is_done = False
                                    #判断，如果零件在更新订单中出现，则将标志位置为True，否则置为False。
                                    for kit_shipment_itr in range(len(self.reaper_order_0_update.kitting_shipments.kitting_shipment)):
                                        for kit_part_itr in range(len(self.reaper_order_0_update.kitting_shipments.kitting_shipment[kit_shipment_itr].products)):
                                            if (withdraw_part.type == self.reaper_order_0_update.kitting_shipments.kitting_shipment[kit_shipment_itr].products[kit_part_itr].type) and \
                                               (withdraw_part.pose == self.reaper_order_0_update.kitting_shipments.kitting_shipment[kit_shipment_itr].products[kit_part_itr].pose):
                                                   withdraw_part.is_done = True
                                                   self.reaper_order_0_update.kitting_shipments.kitting_shipment[kit_shipment_itr].products[kit_part_itr].is_done = True
                                                   
                                            if (withdraw_part.type == self.reaper_order_0_update.kitting_shipments.kitting_shipment[kit_shipment_itr].products[kit_part_itr].type) and \
                                               (not (withdraw_part.pose == self.reaper_order_0_update.kitting_shipments.kitting_shipment[kit_shipment_itr].products[kit_part_itr].pose)):
#                                                   withdraw_part.is_done = True
                                                   self.reaper_order_0_update.kitting_shipments.kitting_shipment[kit_shipment_itr].products[kit_part_itr].is_done = True
                                                   withdraw_part.is_adjust = True
                                                   withdraw_part.adjust_pose = self.reaper_order_0_update.kitting_shipments.kitting_shipment[kit_shipment_itr].products[kit_part_itr].pose
                                                   
                                    #查找withdraw_shipment中是不是有同一类型的。
                                    self.order_0_update_kit_same_wit_shipment = False
                                    for withdraw_shipment_itr in range(len(self.reaper_order_0_update.withdraw_shipments.withdraw_shipment)):
                                        if (shipment_temp.agv_id == self.reaper_order_0_update.withdraw_shipments.withdraw_shipment[withdraw_shipment_itr].agv_id) and \
                                           (shipment_temp.station_id == self.reaper_order_0_update.withdraw_shipments.withdraw_shipment[withdraw_shipment_itr].station_id):
                                            self.order_0_update_kit_same_wit_shipment = True
                                            self.order_0_kit_wit_shipment_itr = withdraw_shipment_itr
                                        else:
                                            pass
                                        
                                    if self.order_0_update_kit_same_wit_shipment == True:   #如果有，将part放入其中的products中。
                                        withdraw_part.i_d = "order_0update_withdraw_shipment_" + str(self.order_0_kit_wit_shipment_itr) + ":" + withdraw_part.type + ":" + \
                                                            str(len(self.reaper_order_0_update.withdraw_shipments.withdraw_shipment[self.order_0_kit_wit_shipment_itr].products))
                                        self.reaper_order_0_update.withdraw_shipments.withdraw_shipment[self.order_0_kit_wit_shipment_itr].products.append(withdraw_part)
                                        self.reaper_order_0_update.withdraw_shipments.withdraw_shipment[self.order_0_kit_wit_shipment_itr].is_done = False
                                        self.reaper_order_0_update.withdraw_shipments.withdraw_shipment[self.order_0_kit_wit_shipment_itr].is_submit = False
                                        self.reaper_order_0_update.withdraw_shipments.is_done = False
                                        
                                    elif self.order_0_update_kit_same_wit_shipment == False:           #没有对应的withdraw_shipment。新建withdraw_shipment,并将part添加进来
                                        tmp_withdraw_shipment = Withdraw_shipment()  
#                                        tmp_withdraw_shipment.shipment_type = "order_0update_withdraw_shipment_" + str(self.ship_numb)
                                        tmp_withdraw_shipment.shipment_type = "order_0update_withdraw_shipment_" + \
                                                            str(len(self.reaper_order_0_update.withdraw_shipments.withdraw_shipment))                                
                                        tmp_withdraw_shipment.agv_id = self.reaper_order_0.kitting_shipments.kitting_shipment[shipment_itr].agv_id
                                        tmp_withdraw_shipment.station_id = self.reaper_order_0.kitting_shipments.kitting_shipment[shipment_itr].station_id
                                        withdraw_part.i_d = tmp_withdraw_shipment.shipment_type + ":" + withdraw_part.type + ":" + "0"
                                        tmp_withdraw_shipment.products.append(withdraw_part)
                                        tmp_withdraw_shipment.is_done  = False
                                        tmp_withdraw_shipment.is_submit = False
                                        self.reaper_order_0_update.withdraw_shipments.withdraw_shipment.append(tmp_withdraw_shipment)
                                        self.reaper_order_0_update.withdraw_shipments.is_done = False
                                        self.order_0_update_wit_part_fin.append(0)
#                                        self.ship_numb = self.ship_numb + 1
                                    print("hello, this is reaper_order_0_update.withdraw_shipments update. ")

            if order.order_id == "order_1_update":       #意味着order_1的更新订单已经下发
                if order_id_tmp == "order_1":            #却还要更新旧订单中的零件，将该零件添加到withdraw中。１、找到该零件
                    if shipment_type_ == "kitting_shipment":       #如果更新的零件是kitting_shipment，只可能是kitting_shipment更新。
                        for shipment_itr in range(0, len(self.reaper_order_1.kitting_shipments.kitting_shipment)):  #
                            shipment_temp = self.reaper_order_1.kitting_shipments.kitting_shipment[shipment_itr]
                            for part_itr in range(0, len(shipment_temp.products)):
                                if shipment_temp.products[part_itr].i_d == part_id_:
                                    withdraw_part = Part()
                                    withdraw_part = shipment_temp.products[part_itr]
                                    withdraw_part.is_done = False

                                    #判断，如果零件在更新订单中出现，则将标志位置为True，否则置为False。
                                    for kit_shipment_itr in range(len(self.reaper_order_1_update.kitting_shipments.kitting_shipment)):
                                        for kit_part_itr in range(len(self.reaper_order_1_update.kitting_shipments.kitting_shipment[kit_shipment_itr].products)):
                                            if (withdraw_part.type == self.reaper_order_1_update.kitting_shipments.kitting_shipment[kit_shipment_itr].products[kit_part_itr].type) and \
                                               (withdraw_part.pose == self.reaper_order_1_update.kitting_shipments.kitting_shipment[kit_shipment_itr].products[kit_part_itr].pose):
                                                   withdraw_part.is_done = True
                                                   self.reaper_order_1_update.kitting_shipments.kitting_shipment[kit_shipment_itr].products[kit_part_itr].is_done = True
                                            
                                            if (withdraw_part.type == self.reaper_order_1_update.kitting_shipments.kitting_shipment[kit_shipment_itr].products[kit_part_itr].type) and \
                                               (not (withdraw_part.pose == self.reaper_order_1_update.kitting_shipments.kitting_shipment[kit_shipment_itr].products[kit_part_itr].pose)):
#                                                   withdraw_part.is_done = True
                                                   self.reaper_order_1_update.kitting_shipments.kitting_shipment[kit_shipment_itr].products[kit_part_itr].is_done = True
                                                   withdraw_part.is_adjust = True
                                                   withdraw_part.adjust_pose = self.reaper_order_1_update.kitting_shipments.kitting_shipment[kit_shipment_itr].products[kit_part_itr].pose
                 
                                    #查找withdraw_shipment中是不是有同一类型的。
                                    self.order_1_update_kit_same_wit_shipment = False
                                    for withdraw_shipment_itr in range(len(self.reaper_order_1_update.withdraw_shipments.withdraw_shipment)):
                                        if (shipment_temp.agv_id == self.reaper_order_1_update.withdraw_shipments.withdraw_shipment[withdraw_shipment_itr].agv_id) and \
                                           (shipment_temp.station_id == self.reaper_order_1_update.withdraw_shipments.withdraw_shipment[withdraw_shipment_itr].station_id):
                                            self.order_1_update_kit_same_wit_shipment = True
                                            self.order_1_kit_wit_shipment_itr = withdraw_shipment_itr
                                        else:
                                            pass
                                        
                                    if self.order_1_update_kit_same_wit_shipment == True:   #如果有，将part放入其中的products中。
                                        withdraw_part.i_d = "order_1update_withdraw_shipment_" + str(self.order_1_kit_wit_shipment_itr) + ":" + withdraw_part.type + ":" + \
                                                            str(len(self.reaper_order_1_update.withdraw_shipments.withdraw_shipment[self.order_1_kit_wit_shipment_itr].products))
                                        self.reaper_order_1_update.withdraw_shipments.withdraw_shipment[self.order_1_kit_wit_shipment_itr].products.append(withdraw_part)
                                        self.reaper_order_1_update.withdraw_shipments.withdraw_shipment[self.order_1_kit_wit_shipment_itr].is_done = False
                                        self.reaper_order_1_update.withdraw_shipments.withdraw_shipment[self.order_1_kit_wit_shipment_itr].is_submit = False
                                        self.reaper_order_1_update.withdraw_shipments.is_done = False
                                        
                                    elif self.order_1_update_kit_same_wit_shipment == False:           #没有对应的withdraw_shipment。新建withdraw_shipment,并将part添加进来
                                        tmp_withdraw_shipment = Withdraw_shipment()  
#                                        tmp_withdraw_shipment.shipment_type = "order_0update_withdraw_shipment_" + str(self.ship_numb)
                                        tmp_withdraw_shipment.shipment_type = "order_1update_withdraw_shipment_" + \
                                                            str(len(self.reaper_order_1_update.withdraw_shipments.withdraw_shipment))                                
                                        tmp_withdraw_shipment.agv_id = self.reaper_order_1.kitting_shipments.kitting_shipment[shipment_itr].agv_id
                                        tmp_withdraw_shipment.station_id = self.reaper_order_0.kitting_shipments.kitting_shipment[shipment_itr].station_id
                                        withdraw_part.i_d = tmp_withdraw_shipment.shipment_type + ":" + withdraw_part.type + ":" + "0"
                                        tmp_withdraw_shipment.products.append(withdraw_part)
                                        tmp_withdraw_shipment.is_done  = False
                                        tmp_withdraw_shipment.is_submit = False
                                        self.reaper_order_1_update.withdraw_shipments.withdraw_shipment.append(tmp_withdraw_shipment)
                                        self.reaper_order_1_update.withdraw_shipments.is_done = False
                                        self.order_1_update_wit_part_fin.append(0)
#                                        self.ship_numb = self.ship_numb + 1
                                    print("hello, this is reaper_order_1_update.withdraw_shipments update. ")
                                    
    def change_kitting_shipment_part_flag_id(self, kitting_shipment, part_check_flag=0):
        if len(self.order_list) >= 2:
            shipments_list = self.order_list[1].kitting_shipments.kitting_shipment
            for kitting_shipment_num in range(0, len(shipments_list)):
                if shipments_list[kitting_shipment_num].shipment_type == kitting_shipment.shipment_type:
                    shipments_list[kitting_shipment_num].check_flag = part_check_flag
        if len(self.order_list) >= 1:
            shipments_list = self.order_list[0].kitting_shipments.kitting_shipment
            for kitting_shipment_num in range(0, len(shipments_list)):
                if shipments_list[kitting_shipment_num].shipment_type == kitting_shipment.shipment_type:
                    shipments_list[kitting_shipment_num].check_flag = part_check_flag    

    def reset_kitting_shipment_part_done_id(self, kitting_shipment, agv_part_list, agv_present_location, can_not_find_part_type):
        success_part_num = 0
        if len(self.order_list) >= 2:
            shipments_list = self.order_list[1].kitting_shipments.kitting_shipment
            for kitting_shipment_num in range(0, len(shipments_list)):
                if shipments_list[kitting_shipment_num].shipment_type == kitting_shipment.shipment_type:
                    
                    for part_num in range(0, len(shipments_list[kitting_shipment_num].products)):
                        part = shipments_list[kitting_shipment_num].products[part_num]
                        agv_id_temp = shipments_list[kitting_shipment_num].agv_id
                        target_part = target_pose_to_world(part, agv_id_temp, agv_present_location)

                        self.part_update(part.i_d, is_done_ = False)
                        if part.type in can_not_find_part_type:
                            print ("can not find part type",part.type)
                            self.part_update(part.i_d, is_done_ = True)
                            success_part_num = success_part_num + 1
                            continue                                                                                     
                        for part_n in agv_part_list:
                            rpy = euler_from_quaternion([target_part.pose.orientation.x,target_part.pose.orientation.y,\
                                target_part.pose.orientation.z,target_part.pose.orientation.w])
                            rpy_n = euler_from_quaternion([part_n.pose.orientation.x,part_n.pose.orientation.y,\
                                part_n.pose.orientation.z,part_n.pose.orientation.w])
                            # #######################################
                            # if part_n.type == target_part.type and "pump" in part_n.type and abs(abs(rpy[0])-pi)< 0.3:#flip part
                            #     if part_n.type == target_part.type and xyz_check(part_n, target_part) and rpy_check(rpy,rpy_n):
                            #         self.part_update(part.i_d, is_done_ = True)
                            #         success_part_num = success_part_num + 1
                            #         continue
                            # ##################################
                            if part_n.type == target_part.type and xyz_check(part_n, target_part) and rpy_check(rpy,rpy_n):
                                
                                self.part_update(part.i_d, is_done_ = True)
                                success_part_num = success_part_num + 1
                    if success_part_num == len(shipments_list[kitting_shipment_num].products):
                        print("success_part_num")
                        print(success_part_num)
                        return True  
        if len(self.order_list) >= 1:
            shipments_list = self.order_list[0].kitting_shipments.kitting_shipment
            for kitting_shipment_num in range(0, len(shipments_list)):
                if shipments_list[kitting_shipment_num].shipment_type == kitting_shipment.shipment_type:
                    
                    for part_num in range(0, len(shipments_list[kitting_shipment_num].products)):
                        part = shipments_list[kitting_shipment_num].products[part_num]
                        agv_id_temp = shipments_list[kitting_shipment_num].agv_id
                        target_part = target_pose_to_world(part, agv_id_temp, agv_present_location)
#                        part.is_done = False                      
                        self.part_update(part.i_d, is_done_ = False)
                        if part.type in can_not_find_part_type:
                            self.part_update(part.i_d, is_done_ = True)
                            success_part_num = success_part_num + 1
                            continue                           
                        for part_n in agv_part_list:
                            rpy = euler_from_quaternion([target_part.pose.orientation.x,target_part.pose.orientation.y,\
                                target_part.pose.orientation.z,target_part.pose.orientation.w])
                            rpy_n = euler_from_quaternion([part_n.pose.orientation.x,part_n.pose.orientation.y,\
                                part_n.pose.orientation.z,part_n.pose.orientation.w])

                            # #######################################
                            # if part_n.type == target_part.type and "pump" in part_n.type and abs(abs(rpy[0])-pi)< 0.3:#flip part
                            #     if part_n.type == target_part.type and xyz_check(part_n, target_part) and rpy_check(rpy,rpy_n):
                            #         self.part_update(part.i_d, is_done_ = True)
                            #         success_part_num = success_part_num + 1
                            #         continue
                            # ##################################

                            if part_n.type == target_part.type and xyz_check(part_n, target_part) and rpy_check(rpy,rpy_n):
#                                part.is_done = True
                                self.part_update(part.i_d, is_done_ = True)
                                success_part_num = success_part_num + 1

                    if success_part_num == len(shipments_list[kitting_shipment_num].products):
                        print("success_part_num")
                        print(success_part_num)
                        return True  
        print("success_part_num")
        print(success_part_num)                              
        return False

    def change_kitting_shipment_submit_flag_id(self, kitting_shipment, submit_flag=False):
        if len(self.order_list) >= 2:
            shipments_list = self.order_list[1].kitting_shipments.kitting_shipment
            for kitting_shipment_num in range(0, len(shipments_list)):
                if shipments_list[kitting_shipment_num].shipment_type == kitting_shipment.shipment_type:
                    shipments_list[kitting_shipment_num].is_submit = submit_flag
        if len(self.order_list) >= 1:
            shipments_list = self.order_list[0].kitting_shipments.kitting_shipment
            for kitting_shipment_num in range(0, len(shipments_list)):
                if shipments_list[kitting_shipment_num].shipment_type == kitting_shipment.shipment_type:
                    shipments_list[kitting_shipment_num].is_submit = submit_flag  

#    def change_kitting_shipment_part_done_id(self, kitting_shipment, part_id, done_flag = True):
#
#        if len(self.order_list) >= 2:
#            shipments_list = self.order_list[1].kitting_shipments.kitting_shipment
#            for kitting_shipment_num in range(0, len(shipments_list)):
#                if shipments_list[kitting_shipment_num].shipment_type == kitting_shipment.shipment_type:
#                    
#                    for part_num in range(0, len(shipments_list[kitting_shipment_num].products)):
#                        part = shipments_list[kitting_shipment_num].products[part_num]
#                        if part.i_d == part_id:
#                            part.is_done = done_flag
#        if len(self.order_list) >= 1:
#            shipments_list = self.order_list[0].kitting_shipments.kitting_shipment
#            for kitting_shipment_num in range(0, len(shipments_list)):
#                if shipments_list[kitting_shipment_num].shipment_type == kitting_shipment.shipment_type:                    
#                    for part_num in range(0, len(shipments_list[kitting_shipment_num].products)):
#                        part = shipments_list[kitting_shipment_num].products[part_num]
#                        if part.i_d == part_id:
#                            part.is_done = done_flag

if __name__ == '__main__':
    rospy.init_node("Order_Processor")
    start_competition()
    rospy.sleep(1)
    order_processing = Order_Processing()
    rospy.sleep(1)
    order_list = order_processing.return_order_list()


#    order_processing.reaper_order_0_update
    for order in order_list:
        print(order.order_id)
        #print(order.kitting_shipments)
        for kit_shipment in order.kitting_shipments.kitting_shipment:
            print(kit_shipment.movable_tray.type)
#     order_processing.part_update("order_0_kitting_shipment_0:assembly_sensor_blue:0")
# #    order_processing.shipment_update("order_0_kitting_shipment_0")
#     order_processing.shipment_update("order_0_kitting_shipment_0")
#     print("ahhhhhhh, search is_done of assembly_sensor_blue: 0")

    
    #rospy.spin()
    
    
    
          

   