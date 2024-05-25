#!/usr/bin/python3

import rospy
import random
from genpy import Duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import threading
import math
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

position_data = {
    'first': {'positions': [{'x': -2.0083794068041274, 'y': 0.002338294012231058, 'z': 0.0}], 'orientations': [{'x': -1.9399690280080383e-06, 'y': -1.0518423969491801e-05, 'z': 8.031014868166898e-05, 'w': 0.9999999967179396}], 'clock': 0},
    'second': {'positions': [{'x': -2.0138764034993515, 'y': 2.0208428098168696, 'z': 0.0}], 'orientations': [{'x': -3.8907407718515964e-05, 'y': 6.475626761260795e-06, 'z': 0.0044462487502659985, 'w': 0.9999901146093049}], 'clock': Duration(13, 461000000)},
    'third': {'positions': [{'x': 0.01763928818057012, 'y': 2.0141679536888937, 'z': 0.0}], 'orientations': [{'x': -2.306742541640864e-05, 'y': 2.7065745450088338e-05, 'z': -0.7006720714291966, 'w': 0.7134834595521174}], 'clock': Duration(13, 484000000)},
    'fourth': {'positions': [{'x': 0.07379384321743233, 'y': -2.0872262985759362, 'z': 0.0}], 'orientations': [{'x': 2.4959683247965742e-05, 'y': -3.0643468082788792e-06, 'z': 0.0023634007055060246, 'w': 0.9999972068484637}], 'clock': Duration(24, 51000000)},
    'fifth': {'positions': [{'x': 2.097789108523263, 'y': -2.053325009807202, 'z': 0.0}], 'orientations': [{'x': 5.2373513535312625e-06, 'y': 6.601838572871925e-06, 'z': 0.7173041880430429, 'w': 0.6967601464958346}], 'clock': Duration(13, 515000000)},
    'sixth': {'positions': [{'x': 1.9713053339206263, 'y': 2.015670852551459, 'z': 0.0}], 'orientations': [{'x': -2.9445606408695255e-05, 'y': 4.7927619089697194e-08, 'z': 0.021829964864704888, 'w': 0.9997616974894366}], 'clock': Duration(23, 891000000)},
    'seventh': {'positions': [{'x': 2.5604163216658957, 'y': 2.0337162449041575, 'z': 0.0}], 'orientations': [{'x': -0.0003429489425843032, 'y': -0.001672994267028399, 'z': 0.015088335143032316, 'w': 0.999884706163074}], 'clock': Duration(3, 10000000)}
    }

current_position = None
cmd_vel_publisher = None
risky_timepoint = None
risky_movement_param = False
risky_movement_in_process = False
current_setpoint = 'first'
rospy.set_param('stop', 1)

rospy.set_param('risky', 0)

def create_twist(linear_vel=0.0, angular_vel=0.0):
    twist = Twist()
    twist.linear.x = linear_vel
    twist.angular.z = angular_vel
    return twist

def rotation_cl_risky(publisher):
    angular_speed = -3
    rotation_time = math.pi / 2 / abs(angular_speed)
    
    twist = create_twist(angular_vel=angular_speed)
    end_time = rospy.Time.now() + rospy.Duration(rotation_time)
    while rospy.Time.now() < end_time and risky_movement_param:
        publisher.publish(twist)
    publisher.publish(create_twist())

def rotation_cl(publisher):
    angular_speed = -0.5
    rotation_time = math.pi / 2 / abs(angular_speed)
    
    twist = create_twist(angular_vel=angular_speed)
    end_time = rospy.Time.now() + rospy.Duration(rotation_time)
    while rospy.Time.now() < end_time and not risky_movement_param and not risky_movement_in_process:
        publisher.publish(twist)
        risky_movement_timer()
    if risky_movement_param and not risky_movement_in_process:
        risky_movement_timer()
    if not risky_movement_param:
        publisher.publish(create_twist())

def rotation_c_cl(publisher):
    angular_speed = 0.5
    rotation_time = math.pi / 2 / abs(angular_speed)
    
    twist = create_twist(angular_vel=angular_speed)
    end_time = rospy.Time.now() + rospy.Duration(rotation_time)
    while rospy.Time.now() < end_time and not risky_movement_param and not risky_movement_in_process:
        publisher.publish(twist)
        risky_movement_timer()
    if risky_movement_param and not risky_movement_in_process:
        risky_movement_timer()
    if not risky_movement_param:
        publisher.publish(create_twist())

def move(publisher, distance, speed=0.2):
    move_time = distance / speed
    
    twist = create_twist(linear_vel=speed)
    end_time = rospy.Time.now() + rospy.Duration(move_time)
    while rospy.Time.now() < end_time and not risky_movement_param and not risky_movement_in_process:
        publisher.publish(twist)
        risky_movement_timer()
    if risky_movement_param and not risky_movement_in_process:
        risky_movement_timer()
    if not risky_movement_param:
        publisher.publish(create_twist())

def move_final(publisher, distance, speed=0.2):
    move_time = distance / speed
    
    twist = create_twist(linear_vel=speed)
    end_time = rospy.Time.now() + rospy.Duration(move_time)
    while rospy.Time.now() < end_time:
        publisher.publish(twist)

    publisher.publish(create_twist())

def move_risky(publisher, distance, speed=2.0):
    
    twist = create_twist(linear_vel=speed)
    while risky_movement_param and rospy.get_param('stop')==1:
        publisher.publish(twist)
    publisher.publish(create_twist())

def get_current_segment(current_position):
    min_distance = float('inf')
    closest_segment_key = None

    for key, value in position_data.items():
        pos = value['positions'][0]
        distance = math.sqrt((pos['x'] - current_position.position.x)**2 + 
                             (pos['y'] - current_position.position.y)**2)

        if distance < min_distance:
            min_distance = distance
            closest_segment_key = key

    return closest_segment_key

def risky_movement(cmd_vel_publisher):
    risky_movement_in_process = True
    print("Performing risky movement...")
    rospy.set_param('risky', 1)
    rotation_cl_risky(cmd_vel_publisher)
    move_risky(cmd_vel_publisher, distance=1.5)
    print("Risky movement completed.")
    rospy.set_param('risky', 0)
    rospy.sleep(2)
    move_to_closest_position()

def move_to_closest_position():
    global current_position, position_data, risky_movement_param
    closest_key = get_current_segment(current_position)
    
    if closest_key:
        closest_position = position_data[closest_key]
        print(f"Teleporting to the closest position: {closest_position}")
        try:
            rospy.wait_for_service('/gazebo/set_model_state', timeout=5)
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            model_state = ModelState()
            model_state.model_name = 'robotont'
            model_state.pose.position.x = closest_position['positions'][0]['x']
            model_state.pose.position.y = closest_position['positions'][0]['y']
            model_state.pose.position.z = 0
 
            model_state.pose.orientation.x = closest_position['orientations'][0]['x']
            model_state.pose.orientation.y = closest_position['orientations'][0]['y']
            model_state.pose.orientation.z = closest_position['orientations'][0]['z']
            model_state.pose.orientation.w = closest_position['orientations'][0]['w']
            
            resp = set_state(model_state)
            print("Teleportation success:", resp.success)
            risky_movement_param = False
            risky_movement_in_process = False
            current_setpoint = closest_key
            calculate_next_risky_timepoint(current_setpoint)
        except rospy.ServiceException as e:
            print("Service call failed:", e)
    else:
        print("Could not determine the closest segment.")

def calculate_next_risky_timepoint(current_setpoint):
    global position_data, risky_timepoint
    setpoint_index = list(position_data.keys()).index(current_setpoint) + 1
    if setpoint_index != 6:
        remaining_duration = sum(position['clock'].to_sec() for key, position in list(position_data.items())[setpoint_index:] if position['clock'])/2
    else:
        remaining_duration = sum(position['clock'].to_sec() for key, position in list(position_data.items())[setpoint_index:] if position['clock'])

    random_timepoint_within_duration = random.uniform(0, remaining_duration)
    print(f"Random timepoint for risky movement: {random_timepoint_within_duration} seconds from the start of {current_setpoint}")
    
    risky_timepoint = rospy.get_time() + random_timepoint_within_duration
    movement_along_line(current_setpoint)

def risky_movement_timer():
    global risky_timepoint, risky_movement_param
    current_time = rospy.get_time()
    if risky_timepoint and current_time >= risky_timepoint:
        risky_movement_param = True
        risky_movement(cmd_vel_publisher)

def movement_along_line(current_setpoint):
    if current_setpoint == 'first':
        rotation_c_cl(cmd_vel_publisher)

        move(cmd_vel_publisher, distance=2.05)
        rotation_cl(cmd_vel_publisher)
    
    if current_setpoint == 'first' or current_setpoint == 'second':
        move(cmd_vel_publisher, distance=2.05)
        rotation_cl(cmd_vel_publisher)
    
    if current_setpoint == 'first' or current_setpoint == 'second' or current_setpoint == 'third':
        move(cmd_vel_publisher, distance=4.155)
        rotation_c_cl(cmd_vel_publisher)
    
    if current_setpoint == 'first' or current_setpoint == 'second' or current_setpoint == 'third' or current_setpoint == 'fourth':
        move(cmd_vel_publisher, distance=2.05)
        rotation_c_cl(cmd_vel_publisher)
    
    if current_setpoint == 'first' or current_setpoint == 'second' or current_setpoint == 'third' or current_setpoint == 'fourth' or current_setpoint == 'fifth':
        move(cmd_vel_publisher, distance=4.125)
        rotation_cl(cmd_vel_publisher)

    if current_setpoint == 'first' or current_setpoint == 'second' or current_setpoint == 'third' or current_setpoint == 'fourth' or current_setpoint == 'fifth' or current_setpoint == 'sixth':
        move_final(cmd_vel_publisher, distance=0.6)

def odom_callback(msg):
    global current_position
    current_position = msg.pose.pose

def main():
    global cmd_vel_publisher
    rospy.init_node('movement', anonymous=True)
    cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.sleep(1)

    calculate_next_risky_timepoint(current_setpoint)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass