#!/usr/bin/env python
#coding:utf-8
from logging import error
import rospy
from geometry_msgs.msg import Point
from nist_gear.msg import *
from nist_gear.srv import *
from math import pi
import numpy
import copy

from tf.transformations import *
PI = pi

#############放置高度参数###############
kitting_place_height = 0.05
gantry_place_height = 0.05

agv_as_location_x1_0 = -5.60
agv_as_location_x1_1 = -6.265685
agv_as_location_x2_0 = -10.590274
agv_as_location_x1_1 = -11.265685

################避障最大等待时间#####################

MAX_WAIT_TIME = 20


######### Part 有关数据 ########
gantry_pick_part_heights_con= { #所有种类的零件高度都经过了测试，wgx 2021-04-17

    'assembly_sensor_red': 0.035,
    'assembly_sensor_blue': 0.035,
    'assembly_sensor_green': 0.035,

    'assembly_regulator_red': 0.0345, 
    'assembly_regulator_blue': 0.0345,
    'assembly_regulator_green': 0.0345,

    'assembly_pump_red': 0.0555,
    'assembly_pump_blue':0.0555,
    'assembly_pump_green':0.0555,

    'assembly_battery_red':0.030, 
    'assembly_battery_blue':0.030, 
    'assembly_battery_green':0.030,
    'can':0.15,
}

gantry_pick_part_heights_bin_agv= {

    'assembly_sensor_red': 0.0350,
    'assembly_sensor_blue': 0.0350,
    'assembly_sensor_green': 0.0350,

    'assembly_regulator_red': 0.0310, 
    'assembly_regulator_blue': 0.0310,
    'assembly_regulator_green': 0.0310,

    'assembly_pump_red': 0.054,
    'assembly_pump_blue':0.054,
    'assembly_pump_green':0.054,

    'assembly_battery_red':0.029, #2021-04-15
    'assembly_battery_blue':0.029, 
    'assembly_battery_green':0.029,
    'can':0.15,
    'movable_tray_dark_wood':0.007,
    'movable_tray_light_wood':0.007,
    'movable_tray_metal_rusty':0.007,
    'movable_tray_metal_shiny':0.007,

}

kitting_pick_part_heights_con= {

    'assembly_sensor_red': 0.03638,
    'assembly_sensor_blue': 0.03638,
    'assembly_sensor_green': 0.03638,

    'assembly_regulator_red': 0.036, 
    'assembly_regulator_blue': 0.036,
    'assembly_regulator_green': 0.036,

    'assembly_pump_red': 0.0575,
    'assembly_pump_blue':0.0575,
    'assembly_pump_green':0.0575,

    'assembly_battery_red':0.0305, 
    'assembly_battery_blue':0.0305, 
    'assembly_battery_green':0.0305,
    'can':0.15,
}

kitting_pick_part_heights_on_bin_agv= {

    'assembly_sensor_red': 0.0355,
    'assembly_sensor_blue': 0.0355,
    'assembly_sensor_green': 0.0355,

    'assembly_regulator_red': 0.0367,
    'assembly_regulator_blue': 0.0367,
    'assembly_regulator_green': 0.0367,

    'assembly_pump_red': 0.059,
    'assembly_pump_blue':0.059,
    'assembly_pump_green':0.059,

    'assembly_battery_red':0.032, 
    'assembly_battery_blue':0.032, 
    'assembly_battery_green':0.032,
    'can':0.15,
}
######### Conveyor 有关数据 ########
conveyor_vel = 0.20
conveyor_begin = 4.26
conveyor_end = -4.15
######### Kitting_Robot 有关数据 ########

kitting_robot_park_location = {
    'agv1_ks1_tray':[-1.30, 4.675404, 0],
    'bin1':[-1.30, 3.379920, 0],
    'bin2':[-1.30, 2.425006, 0],#'bin2':[-1.30, 2.565006, 0]
    'agv2_ks2_tray':[-1.30, 1.367643, 0],
    'can': [-1.30, -0.014119, 0],
    'agv3_ks3_tray':[-1.30, -1.333917, 0],
    'bin6':[-1.30, -2.565006, 0],
    'bin5':[-1.30, -3.379920, 0],
    'agv4_ks4_tray':[-1.30, -4.696062, 0],
}

kitting_velocity = 0.8  #
kitting_angle_velocity = 2.00
controller_respond_time = 0.1
ASEND = 0.01 #机器人的指令周期
vacuum_gripper_height = 0.01
#kitting_robot一次指令最大可滑行距离
kitting_robot_slide_throld = 10.50

# A_m是修正矩阵
A_k = numpy.matrix([[-1.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00],
                    [0.00000000e+00, -1.00000000e+00, 0.00000000e+00,0.00000000e+00],
                    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00, 0.00000000e+00],
                    [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.000000000000]])



######### Gantry_Robot 有关数据 ########
gantry_velocity = 0.8

gantry_angle_velocity = 2.00
# A_g是修正矩阵
A_g = numpy.matrix([[ 1.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00],
                [0.00000000e+00, 1.00000000e+00, 0.00000000e+00,0.00000000e+00],
                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00, 0.00000000e+00],
                [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.000000000000]])


bin_location = {
    'bin1': [-1.898993, 3.379920, 0],
    'bin2': [-1.898993, 2.565006, 0],
    'bin3': [-2.651690, 2.565006, 0],
    'bin4': [-2.651690, 3.379920, 0],
    'bin5': [-1.898993, -3.379920, 0],
    'bin6': [-1.898993, -2.565006, 0],
    'bin7': [-2.651690, -2.565006, 0],
    'bin8': [-2.651690, -3.379920, 0],
}

agv_loaction={

    'agv1_ks1_tray': [-2.265685,4.675404,0],
    'agv2_ks2_tray': [-2.265685,1.367643, 0],
    'agv3_ks3_tray': [-2.265685,-1.333917, 0],
    'agv4_ks4_tray': [-2.265685, -4.696062, 0],

    'agv1_as1_tray': [-5.60,4.675404,0],
    'agv2_as1_tray': [-5.60,1.367643, 0],
    'agv3_as3_tray': [-5.60,-1.333917, 0],
    'agv4_as3_tray': [-5.60, -4.696062, 0],

    'agv1_as2_tray': [-10.590274,4.675404,0],
    'agv2_as2_tray': [-10.590274,1.367643, 0],
    'agv3_as4_tray': [-10.590274,-1.333917, 0],
    'agv4_as4_tray': [-10.590274, -4.696062, 0],

}

bin_offset_x = -1.898993 +2.85
agv_offset_x = 0.75 #0.8
agv_offset_y = 0.23 # 0.23
agv_offset_x_ = -0.95
agv_offset_y_ = 0.25
as_offset_x = 2.05

gantry_robot_park_location ={

    'as1': [-7.3+as_offset_x,   3.0, pi/2],
    'as2': [-12.3+as_offset_x,  3.0, pi/2],
    'as3': [-7.3+as_offset_x,  -3.0, pi/2],
    'as4': [-12.3+as_offset_x, -3.0, pi/2],

    'agv1_ks1_tray':[agv_loaction['agv1_ks1_tray'][0] - agv_offset_x+0.08, agv_loaction['agv1_ks1_tray'][1]-agv_offset_y,  -pi/2+pi/8],
    'agv2_ks2_tray':[agv_loaction['agv2_ks2_tray'][0] - agv_offset_x, agv_loaction['agv2_ks2_tray'][1],  -pi/2], 
    'agv3_ks3_tray':[agv_loaction['agv3_ks3_tray'][0] - agv_offset_x, agv_loaction['agv3_ks3_tray'][1],  -pi/2], 
    'agv4_ks4_tray':[agv_loaction['agv4_ks4_tray'][0] - agv_offset_x+0.08, agv_loaction['agv4_ks4_tray'][1]+agv_offset_y,  -pi/2-pi/8], 

    'agv1_as1_tray':[agv_loaction['agv1_as1_tray'][0] - agv_offset_x_, agv_loaction['agv1_as1_tray'][1]-agv_offset_y_, pi/2],
    'agv2_as1_tray':[agv_loaction['agv2_as1_tray'][0] - agv_offset_x_, agv_loaction['agv2_as1_tray'][1], pi/2], 
    'agv3_as3_tray':[agv_loaction['agv3_as3_tray'][0] - agv_offset_x_, agv_loaction['agv3_as3_tray'][1], pi/2], 
    'agv4_as3_tray':[agv_loaction['agv4_as3_tray'][0] - agv_offset_x_, agv_loaction['agv4_as3_tray'][1]+agv_offset_y_, pi/2],

    'agv1_as2_tray':[agv_loaction['agv1_as2_tray'][0] - agv_offset_x_, agv_loaction['agv1_as2_tray'][1]-agv_offset_y_, pi/2],
    'agv2_as2_tray':[agv_loaction['agv2_as2_tray'][0] - agv_offset_x_, agv_loaction['agv2_as2_tray'][1], pi/2], 
    'agv3_as4_tray':[agv_loaction['agv3_as4_tray'][0] - agv_offset_x_, agv_loaction['agv3_as4_tray'][1], pi/2], 
    'agv4_as4_tray':[agv_loaction['agv4_as4_tray'][0] - agv_offset_x_, agv_loaction['agv4_as4_tray'][1]+agv_offset_y_, pi/2], 


    'bin1': [-1.898993 - bin_offset_x, 3.379920, -pi/2],
    'bin2': [-1.898993 - bin_offset_x, 2.565006, -pi/2],
    'bin3': [-2.651690 - bin_offset_x, 2.565006, -pi/2],
    'bin4': [-2.651690 - bin_offset_x, 3.379920, -pi/2],
    'bin5': [-1.898993 - bin_offset_x, -3.379920, -pi/2],
    'bin6': [-1.898993 - bin_offset_x, -2.565006, -pi/2],
    'bin7': [-2.651690 - bin_offset_x, -2.565006, -pi/2],
    'bin8': [-2.651690 - bin_offset_x, -3.379920, -pi/2],

    'can':[-2.186829 - bin_offset_x,0.000000,-pi/2],
    "init_1":[-4.55,0.00, 0],
    "init_2":[-9.54,0.00, pi/2], 
    'gripper_cs' : [-3.90, 6.20, -pi/2],
    'tray_table' : [-6.09, 5.50, -pi/2]
}


############装配相关数据#############

############装配相关数据#############

battery_offset_x = 0.003
battery_offset_y = 0.000
# regulator_offset_x = 0.043 
# regulator_offset_y = 0.115
regulator_offset_x = 0.0405#0.044
regulator_offset_y = -0.0025#0.109

sensor_offset_x = -0.1172#-0.11677
#sensor_offset_y = -0.0459
sensor_offset_y = -0.0430
sensor_offset_z = 0.0060
pump_offset_x = 0.003

as1_part = {
    # [x, y, z, r, p, y]
    'assembly_battery_red': [-7.252468+battery_offset_x, 3.270679+battery_offset_y, 1.287989, 0.0,0.0,0.0],
    'assembly_battery_blue':[-7.252468+battery_offset_x, 3.270679+battery_offset_y, 1.287989, 0.0,0.0,0.0],
    'assembly_battery_green':[-7.252468+battery_offset_x, 3.270679+battery_offset_y, 1.287989, 0.0,0.0,0.0],

    'assembly_regulator_red':  [-7.435080+regulator_offset_x, 2.930840+regulator_offset_y, 1.406471, 1.5*pi, pi/2, 1.5*pi],
    'assembly_regulator_blue': [-7.435080+regulator_offset_x, 2.930840+regulator_offset_y, 1.406471, 1.5*pi, pi/2, 1.5*pi],
    'assembly_regulator_green':[-7.435080+regulator_offset_x, 2.930840+regulator_offset_y, 1.406471, 1.5*pi, pi/2, 1.5*pi],
     
    'assembly_sensor_red':[-6.808+sensor_offset_x, 3.2597+sensor_offset_y,1.300+sensor_offset_z,pi/2,0,0],
    'assembly_sensor_blue':[-6.808+sensor_offset_x, 3.2597+sensor_offset_y,1.300+sensor_offset_z,pi/2,0,0],
    'assembly_sensor_green':[-6.808+sensor_offset_x, 3.2597+sensor_offset_y,1.300+sensor_offset_z,pi/2,0,0],

    'assembly_pump_red': [-7.187915+pump_offset_x, 2.943007, 1.274424, 0.0,0.0,0.0],
    'assembly_pump_blue':[-7.187915+pump_offset_x, 2.943007, 1.274424, 0.0,0.0,0.0],
    'assembly_pump_green':[-7.187915+pump_offset_x, 2.943007, 1.274424, 0.0,0.0,0.0],

}

as2_part = {
    # [x, y, z, r, p, y]
    'assembly_battery_red': [-12.252468+battery_offset_x, 3.270679+battery_offset_y, 1.287989, 0.0,0.0,0.0],
    'assembly_battery_blue':[-12.252468+battery_offset_x, 3.270679+battery_offset_y, 1.287989, 0.0,0.0,0.0],
    'assembly_battery_green':[-12.252468+battery_offset_x, 3.270679+battery_offset_y, 1.287989, 0.0,0.0,0.0],

    'assembly_regulator_red':  [-12.435080+regulator_offset_x, 2.930840+regulator_offset_y, 1.406471, 1.5*pi, pi/2, 1.5*pi],
    'assembly_regulator_blue': [-12.435080+regulator_offset_x, 2.930840+regulator_offset_y, 1.406471, 1.5*pi, pi/2, 1.5*pi],
    'assembly_regulator_green':[-12.435080+regulator_offset_x, 2.930840+regulator_offset_y, 1.406471, 1.5*pi, pi/2, 1.5*pi],

    'assembly_sensor_red':[-11.808+sensor_offset_x, 3.2597+sensor_offset_y,1.300+sensor_offset_z,pi/2,0,0],
    'assembly_sensor_blue':[-11.808+sensor_offset_x, 3.2597+sensor_offset_y,1.300+sensor_offset_z,pi/2,0,0],
    'assembly_sensor_green':[-11.808+sensor_offset_x, 3.2597+sensor_offset_y,1.300+sensor_offset_z,pi/2,0,0],

    'assembly_pump_red': [-12.187915+pump_offset_x, 2.943007, 1.274424, 0.0,0.0,0.0],
    'assembly_pump_blue':[-12.187915+pump_offset_x, 2.943007, 1.274424, 0.0,0.0,0.0],
    'assembly_pump_green':[-12.187915+pump_offset_x, 2.943007, 1.274424, 0.0,0.0,0.0],
}

as3_part = {
    # [x, y, z, r, p, y]
    'assembly_battery_red': [-7.252468+battery_offset_x, -2.729321+battery_offset_y, 1.287989, 0.0,0.0,0.0],
    'assembly_battery_blue':[-7.252468+battery_offset_x, -2.729321+battery_offset_y, 1.287989, 0.0,0.0,0.0],
    'assembly_battery_green':[-7.252468+battery_offset_x, -2.729321+battery_offset_y, 1.287989, 0.0,0.0,0.0],

    'assembly_regulator_red':  [-7.435080+regulator_offset_x, -3.06916+regulator_offset_y, 1.406471, 1.5*pi, pi/2, 1.5*pi],
    'assembly_regulator_blue': [-7.435080+regulator_offset_x, -3.06916+regulator_offset_y, 1.406471, 1.5*pi, pi/2, 1.5*pi],
    'assembly_regulator_green':[-7.435080+regulator_offset_x, -3.06916+regulator_offset_y, 1.406471, 1.5*pi, pi/2, 1.5*pi],

    'assembly_sensor_red':[-6.808+sensor_offset_x,  -2.7403+sensor_offset_y,1.300+sensor_offset_z,pi/2,0,0],
    'assembly_sensor_blue':[-6.808+sensor_offset_x, -2.7403+sensor_offset_y,1.300+sensor_offset_z,pi/2,0,0],
    'assembly_sensor_green':[-6.808+sensor_offset_x,-2.7403+sensor_offset_y,1.300+sensor_offset_z,pi/2,0,0],

    'assembly_pump_red': [-7.187915+pump_offset_x, -3.056993, 1.274424, 0.0,0.0,0.0],
    'assembly_pump_blue':[-7.187915+pump_offset_x, -3.056993, 1.274424, 0.0,0.0,0.0],
    'assembly_pump_green':[-7.187915+pump_offset_x, -3.056993, 1.274424, 0.0,0.0,0.0],
}

as4_part = {
    # [x, y, z, r, p, y]
    'assembly_battery_red': [-12.252468+battery_offset_x, -2.729321+battery_offset_y, 1.287989, 0.0,0.0,0.0],
    'assembly_battery_blue':[-12.252468+battery_offset_x, -2.729321+battery_offset_y, 1.287989, 0.0,0.0,0.0],
    'assembly_battery_green':[-12.252468+battery_offset_x, -2.729321+battery_offset_y,1.287989, 0.0,0.0,0.0],

    'assembly_regulator_red':  [-12.435080+regulator_offset_x, -3.06916+regulator_offset_y, 1.406471, 1.5*pi, pi/2, 1.5*pi],
    'assembly_regulator_blue': [-12.435080+regulator_offset_x, -3.06916+regulator_offset_y, 1.406471, 1.5*pi, pi/2, 1.5*pi],
    'assembly_regulator_green':[-12.435080+regulator_offset_x, -3.06916+regulator_offset_y, 1.406471, 1.5*pi, pi/2, 1.5*pi],

    'assembly_sensor_red':[-11.808+sensor_offset_x, -2.7403+sensor_offset_y,1.300+sensor_offset_z,pi/2,0,0],
    'assembly_sensor_blue':[-11.808+sensor_offset_x, -2.7403+sensor_offset_y,1.300+sensor_offset_z,pi/2,0,0],
    'assembly_sensor_green':[-11.808+sensor_offset_x, -2.7403+sensor_offset_y,1.300+sensor_offset_z,pi/2,0,0],

    'assembly_pump_red': [-12.187915+pump_offset_x, -3.056993, 1.274424, 0.0,0.0,0.0],
    'assembly_pump_blue':[-12.187915+pump_offset_x, -3.056993, 1.274424, 0.0,0.0,0.0],
    'assembly_pump_green':[-12.187915+pump_offset_x, -3.056993, 1.274424, 0.0,0.0,0.0],
}


as_data = {
    'as1':as1_part,
    'as2':as2_part,
    'as3':as3_part,
    'as4':as4_part,
}


delta_x = 2.05
delta_y = 0.05
as_location = {
    'as1': [-7.3+delta_x,   3.0, pi/2],
    'as2': [-12.3+delta_x,  3.0, pi/2],
    'as3': [-7.3+delta_x,  -3.0, pi/2],
    'as4': [-12.3+delta_x, -3.0, pi/2],
}

delta_x_regul = 1.64#0.35
delta_y_regul = -0.35#1.40
regulator_as_location = {
    'as1': [-7.3+ delta_x_regul,   3.0+delta_y_regul, pi/2],#pi
    'as2': [-12.3+delta_x_regul,  3.0+delta_y_regul, pi/2],
    'as3': [-7.3+ delta_x_regul,  -3.0+delta_y_regul, pi/2],
    'as4': [-12.3+delta_x_regul, -3.0+delta_y_regul, pi/2],
}

gantry_assembly_offset_y = -0.1543
gantry_assembly_offset_x = 1.15

bins = ['bin1', 'bin2','bin6','bin5','bin4','bin3','bin7','bin8']

kitting_bins = ['bin1', 'bin2','bin6','bin5']

bin5_safe_zone = ['bin1', 'bin2','agv1_ks1_tray','agv2_ks2_tray','agv3_ks3_tray','can']

bin6_safe_zone = ['bin1', 'bin2','agv1_ks1_tray','agv2_ks2_tray','agv3_ks3_tray','agv4_ks4_tray','can']

can_safe_zone =['bin1', 'bin2','bin5', 'bin6','agv1_ks1_tray','agv2_ks2_tray','agv4_ks4_tray']

bin2_safe_zone = ['bin5','bin6','agv1_ks1_tray','agv2_ks2_tray','agv3_ks3_tray','agv4_ks4_tray','can']

bin1_safe_zone = ['bin5','bin6','agv1_ks1_tray','agv2_ks2_tray','agv3_ks3_tray','agv4_ks4_tray','can']


agv_ks_location={
    "agv1":"agv1_ks1_tray",
    "agv2":"agv2_ks2_tray",
    "agv3":"agv3_ks3_tray",
    "agv4":"agv4_ks4_tray",
}











agv_as_location_act_x1_0 = -5.45
agv_as_location_act_x1_1 = -6.115691
agv_as_location_act_x2_0 = -10.440274
agv_as_location_act_x2_1 = -11.115701




# 下面的数据主要用来target_pose_to_world做坐标转换
# 下面的数据测量是tray的中心点位置，不是agv的
agv_tray_heigh = 0.750
agv1_kitting_location={
    'agv1_ks1_tray':[-2.115685,4.675936,agv_tray_heigh],
    'agv1_as1_tray':[-5.45, 4.675936,agv_tray_heigh],#-5.45
    'agv1_as2_tray':[-10.440274,4.675936,agv_tray_heigh],#-10.440274
}
agv2_kitting_location={
    'agv2_ks2_tray':[-2.115685, 1.368175,agv_tray_heigh],
    'agv2_as1_tray':[-5.45, 1.368175,agv_tray_heigh],
    'agv2_as2_tray':[-10.440274, 1.368175,agv_tray_heigh],
}

agv3_kitting_location={
    'agv3_ks3_tray':[-2.115685,-1.333385,agv_tray_heigh],
    'agv3_as3_tray':[-5.45,-1.333385,agv_tray_heigh],
    'agv3_as4_tray':[-10.440274,-1.333385,agv_tray_heigh],
}

agv4_kitting_location={
    'agv4_ks4_tray':[-2.115685,-4.69553,agv_tray_heigh],
    'agv4_as3_tray':[-5.45,-4.69553,agv_tray_heigh],
    'agv4_as4_tray':[-10.440274,-4.69553,agv_tray_heigh],
}


AGV_Kitting_location ={
    'agv1':agv1_kitting_location,
    'agv2':agv2_kitting_location,
    'agv3':agv3_kitting_location,
    'agv4':agv4_kitting_location,
}

AS_AGV = {
    'as1':['agv1','agv2'],
    'as2':['agv1','agv2'],
    'as3':['agv3','agv4'],
    'as4':['agv3','agv4'],
}
AS_AGV_location = {
    'as1':['agv1_as1_tray','agv2_as1_tray'],
    'as2':['agv1_as2_tray','agv2_as2_tray'],
    'as3':['agv3_as3_tray','agv4_as3_tray'],
    'as4':['agv3_as4_tray','agv4_as4_tray'],
}


container_position = {
    'bin1': [-1.898993, 3.379920, 0],
    'bin2': [-1.898993, 2.565006, 0],
    'bin3': [-2.651690, 2.565006, 0],
    'bin4': [-2.651690, 3.379920, 0],
    'bin5': [-1.898993, -3.379920, 0],
    'bin6': [-1.898993, -2.565006, 0],
    'bin7': [-2.651690, -2.565006, 0],
    'bin8': [-2.651690, -3.379920, 0], 
    'agv1_ks1_tray': [-2.115644, 4.675404, 0],
    'agv2_ks2_tray': [-2.115644, 1.367643, 0],
    'agv3_ks3_tray': [-2.115644, -1.333917, 0],
    'agv4_ks4_tray': [-2.115644, -4.696062, 0],
    'agv1_as1_tray': [-5.445, 4.675404, 0],#-5.445
    'agv1_as2_tray': [-10.435, 4.675404, 0],#-10.435
    'agv2_as1_tray': [-5.445, 1.367643, 0],
    'agv2_as2_tray': [-10.435, 1.367643, 0],
    'agv3_as3_tray': [-5.445, -1.333917, 0],
    'agv3_as4_tray': [-10.435, -1.333917, 0],   
    'agv4_as3_tray': [-5.445, -4.696062, 0],
    'agv4_as4_tray': [-10.435, -4.696062, 0],     
}


AGV_Orientation = quaternion_from_euler(0, 0, -pi/2)       

def target_pose_to_world(part,agv_id, agv_present_location):
    '''
    订单中的零件位置转换成，要放置在agv托盘上的世界坐标位置
    '''
    new_part = copy.deepcopy(part)
    new_part.pose.position.x = AGV_Kitting_location[agv_id][agv_present_location][0] + part.pose.position.y
    new_part.pose.position.y = AGV_Kitting_location[agv_id][agv_present_location][1] - part.pose.position.x
    #   part_roll = atan2(2 * (part.pose.orientation.x * part.pose.orientation.w + part.pose.orientation.y * part.pose.orientation.z), \
    #                   1.0 - 2 * (part.pose.orientation.x ** 2 + part.pose.orientation.y **2))
    
    new_part.pose.position.z = AGV_Kitting_location[agv_id][agv_present_location][2] + kitting_pick_part_heights_on_bin_agv[part.type]
    p = euler_from_quaternion([part.pose.orientation.x,part.pose.orientation.y,part.pose.orientation.z,part.pose.orientation.w])

    if abs(abs(p[0]) - pi)<=0.3 and ("pump" in part.type):
        new_part.is_flip = True
    q = quaternion_from_euler(p[0],p[1],p[2],"sxyz")
    part.pose.orientation.x = q[0]
    part.pose.orientation.y = q[1]
    part.pose.orientation.z = q[2]
    part.pose.orientation.w = q[3]

    q = quaternion_multiply(AGV_Orientation,  \
                    [part.pose.orientation.x,part.pose.orientation.y,part.pose.orientation.z,part.pose.orientation.w])
    new_part.pose.orientation.x = q[0]
    new_part.pose.orientation.y = q[1]
    new_part.pose.orientation.z = q[2]
    new_part.pose.orientation.w = q[3]
    
    return new_part
        

class Robot_Info:
    '''
    work_state: standby, moving, picking, placing, flipping
    '''
    def __init__(self,robot_name):
        self.name = robot_name
        self.position = Point()
        self.location = None
        self.next_park_location = None
        self.is_enabled = False
        self.is_idle = False
        self.work_state= "standby"
        self.has_been_disabled = False
        self.is_alive = True #place 避障时用
        self.pick_part = None



class GripperManager():
    def __init__(self, ns):
        self.ns = ns

    def activate_gripper(self):
        rospy.wait_for_service(self.ns + 'control')
        rospy.ServiceProxy(self.ns + 'control', VacuumGripperControl)(True)

    def deactivate_gripper(self):
        rospy.wait_for_service(self.ns + 'control')
        rospy.ServiceProxy(self.ns + 'control', VacuumGripperControl)(False)

    def is_object_attached(self):
        status = rospy.wait_for_message(self.ns + 'state', VacuumGripperState)
        return status.attached

def limit_joint(angle, threshold):
    while angle>=threshold:
        angle = angle-threshold
    while angle<=-threshold:
        angle = angle+threshold
    return angle

def part_pose_after_flip(part):
    rpy =  euler_from_quaternion([part.pose.orientation.x,part.pose.orientation.y,\
            part.pose.orientation.z,part.pose.orientation.w])
    flip_rpy = [rpy[0]+pi,rpy[1],rpy[2]-pi]
    q = quaternion_from_euler(flip_rpy[0],flip_rpy[1],flip_rpy[2],"sxyz")
    part.pose.orientation.x = q[0]
    part.pose.orientation.y = q[1]
    part.pose.orientation.z = q[2]
    part.pose.orientation.w = q[3]
    part.pose.position.z = 0.779888
    return part
def kitting_close_park_location(position_y):
    l1 = abs(position_y -kitting_robot_park_location['agv1_ks1_tray'][1])
    l2 = abs(position_y -kitting_robot_park_location['bin1'][1])
    l3 = abs(position_y -kitting_robot_park_location['bin2'][1])
    l4 = abs(position_y -kitting_robot_park_location['agv2_ks2_tray'][1])
    l5 = abs(position_y -kitting_robot_park_location['can'][1])
    l6 = abs(position_y -kitting_robot_park_location['agv3_ks3_tray'][1])
    l7 = abs(position_y -kitting_robot_park_location['bin6'][1])
    l8 = abs(position_y -kitting_robot_park_location['bin5'][1])
    l9 = abs(position_y -kitting_robot_park_location['agv4_ks4_tray'][1])
    length = [l1,l2,l3,l4,l5,l6,l7,l8,l9]
    locations = ['agv1_ks1_tray','bin1','bin2','agv2_ks2_tray','can','agv3_ks3_tray','bin6','bin5','agv4_ks4_tray']
    short =length.index(min(length))
    location = locations[short]
    return location

def gantry_location_near_ks(ks_location):
    if ks_location == 'agv1_ks1_tray':
        return "bin4"
    if ks_location == "agv2_ks2_tray":
        return "can"
    if ks_location == "agv3_ks3_tray":
        return "can"
    if ks_location == "agv4_ks4_tray":
        return "bin8"

def angle_limit(angle):
    angle = angle%(2*pi)
    return angle


def rpy_check(rpy, rpy_1):
    error_0 = abs((angle_limit(rpy[0]) - angle_limit(rpy_1[0])))
    error_1 = abs((angle_limit(rpy[1]) - angle_limit(rpy_1[1])))
    error_2 = abs((angle_limit(rpy[2]) - angle_limit(rpy_1[2])))

    # print("angle_limit(rpy[0])",angle_limit(rpy[0]))
    # print("angle_limit(rpy_1[0])",angle_limit(rpy_1[0]))
    # print("abs((angle_limit(rpy[0]) - angle_limit(rpy_1[0]))",error_0)

    # print("angle_limit(rpy[1])",angle_limit(rpy[1]))
    # print("angle_limit(rpy_1[1])",angle_limit(rpy_1[1]))
    # print("abs((angle_limit(rpy[1]) - angle_limit(rpy_1[1]))",error_1)


    # print("angle_limit(rpy[2])",angle_limit(rpy[2]))
    # print("angle_limit(rpy_1[2])",angle_limit(rpy_1[2]))
    # print("abs((angle_limit(rpy[2]) - angle_limit(rpy_1[2]))",error_2)



    if (error_0 <= 0.1 or error_0 >= 2*pi-0.1) and (error_1 <= 0.1 or error_1 >= 2*pi-0.1)\
        and (error_2 <= 0.1 or error_2 >= 2*pi-0.1):
        return True
    else:
        return False

max_delta = 0.00
def xyz_check(part_1, part_2):
    if  abs(part_1.pose.position.x - part_2.pose.position.x) < 0.03 + max_delta \
    and abs(part_1.pose.position.y - part_2.pose.position.y) < 0.03 + max_delta \
    and abs(part_1.pose.position.z - part_2.pose.position.z) < 0.03 + max_delta :
        return True
    else:
        return False




# yaw_region = {
#               '1' : [3*pi/4 , 7*pi/4, 0, True],
#               '2' : [pi/4 , 5*pi/4, pi, True],
#               '3' : [3*pi/4 , 7*pi/4, pi, True],
#               '4' : [pi/4 , 5*pi/4, 0, True],
#               }

# def recognize_region(part_position, container_position):
#     if part_position.x >= container_position[0] and part_position.y <= container_position[1]:
#         return '1'
#     elif part_position.x > container_position[0] and part_position.y > container_position[1]:
#         return '2'
#     elif part_position.x < container_position[0] and part_position.y > container_position[1]:
#         return '3'
#     elif part_position.x < container_position[0] and part_position.y < container_position[1]:
#         return '4'

# def flip_rotation(part_yaw, yaw_region):
#     yaw_region_n = copy.deepcopy(yaw_region)
#     #判断抓取零件左侧或右侧，返回值加上零件yaw即gantry需要旋转角度
#     part_yaw =   part_yaw% (2*pi)
#     # part_yaw = limit_joint(part_yaw, 2*pi)

#     if part_yaw >= yaw_region[0] and part_yaw <= yaw_region[1]:
#         yaw_region_n[2] = yaw_region[2] % (2*pi)
#         # yaw_region[2] = limit_joint(yaw_region[2], 2*pi)
#         yaw_region_n[3] = True
#     else:
#         yaw_region_n[2] = (yaw_region[2] + pi) % (2*pi)

#         # yaw_region[2] = limit_joint((yaw_region[2]+pi),2*pi)

#         yaw_region_n[3] = False
#     return yaw_region_n