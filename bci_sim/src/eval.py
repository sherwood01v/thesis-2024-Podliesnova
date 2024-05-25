#!/usr/bin/python3

import rospy

rospy.init_node('param_monitor')

previous_risky = rospy.get_param('risky', 0)
last_change_time = rospy.get_time()
three_second_mark = True

TP = 0
FP = 0
TN = 0
FN = 0

while not rospy.is_shutdown():
    risky = rospy.get_param('risky')
    stop = rospy.get_param('stop')

    #checking the current state and increment counters
    if risky == 0 and three_second_mark:
        if stop == 1:
            TN += 1
        elif stop == 0:
            FP += 1

    #checking state change from 0 to 1 in risky
    if previous_risky == 0 and risky == 1:
        last_change_time = rospy.get_time()
        general = False
        three_second_mark = False  #reset the three-second condition check

    #after a state change, checking stop condition for three seconds
    if not three_second_mark and rospy.get_time() - last_change_time <= 2:
        if stop == 0:
            TP += 1
            three_second_mark = True  #avoid counting TN multiple times
    elif rospy.get_time() - last_change_time > 3 and not three_second_mark:
        FN += 1
        three_second_mark = True  #mark that the check period has passed

    #updating previous risky
    previous_risky = risky
    rospy.sleep(0.1)

    rospy.loginfo(f"TP: {TP}, FP: {FP}, TN: {TN}, FN: {FN}")