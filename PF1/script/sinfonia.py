#!/usr/bin/env python


import rospy
import smach
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Empty

# from sensor_msgs.msg import Image
# from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Header
from std_msgs.msg import String

from geometry_msgs.msg import Twist

import math
import time

# from ros_beginner.srv import Kyolo,KyoloResponse

# from sync_sub.msg import some_position
# from sync_sub.msg import some_position2
import actionlib
# from ros_beginner.msg import Practice2Action
# from ros_beginner.msg import Practice2Goal
import csv
import tf
import pandas as pd
import os
import sys, select, os
import tty, termios
from std_msgs.msg import Int8, String



class Navigation(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'exit'])

    def execute(self, userdata):
        key = raw_input("key: ")
        print("Input m -> manual mode...")
        if key == 'm':
                ### manual mode ###
                return 'done'
            else:
                ### finish ###
            return 'exit'


class Manual(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'exit'])

        self.LIN_VEL_STEP_SIZE = 0.157 # [m/s]
        self.ANG_VEL_STEP_SIZE = 0.060 # [rad/s]
        self.LIMIT_LIN_VEL     = 0.833 # [m/s]
        self.LIMIT_ANG_VEL     = 0.800 # [rad/s]
        
        self.msg = """
        ---------------------------
        Moving around:
                w
           a    s    d
                x
        
        w/x : increase/decrease linear velocity
        a/d : increase/decrease angular velocity
        
        space key, s : force stop
        
        CTRL-C to quit
        """
        
        self.e = """
        Communications Failed
        """
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Manual')
        print("Waiting input key...")

        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)

        ### publisher ###
        publish_driving_mode = rospy.Publisher('driving_mode', Int8 , queue_size = 10)
        publish_cmd_vel      = rospy.Publisher('ackermann_steering_controller/cmd_vel'     , Twist, queue_size = 10)

        ### parameter ###
        status              = 0
        target_linear_vel   = 0
        target_angular_vel  = 0
        control_linear_vel  = 0
        control_angular_vel = 0
        driving_mode        = 1
        change_mode_flag    = 0

        try:
            print(self.msg)
            while(1):
                ### key options ###
                key = self.getKey()
                if key == '1' :
                    ### nomal mode ###
                    if( driving_mode == 2 or driving_mode == 3 ):
                        change_mode_flag = 1
                    driving_mode        = 1
                    target_linear_vel   = 0
                    control_linear_vel  = 0
                    target_angular_vel  = 0
                    control_angular_vel = 0
                    print("Nomal mode...")
                    print(self.vels(target_linear_vel, target_angular_vel))

                elif key == '2' :
                    ### rotetion mode ###
                    if( driving_mode == 1 or driving_mode == 3 ):
                        change_mode_flag = 1
                    driving_mode        = 2
                    target_linear_vel   = 0
                    control_linear_vel  = 0
                    target_angular_vel  = 0
                    control_angular_vel = 0
                    print("Rotate mode...")
                    print(self.vels(target_linear_vel, target_angular_vel))

                elif key == '3' :
                    ### sliding mode ###
                    if( driving_mode == 1 or driving_mode == 2 ):
                        change_mode_flag = 1
                    driving_mode        = 3
                    target_linear_vel   = 0
                    control_linear_vel  = 0
                    target_angular_vel  = 0
                    control_angular_vel = 0
                    print("Slide mdoe...")
                    print(self.vels(target_linear_vel, target_angular_vel))

                elif key == 'w' :
                    ### up ###
                    target_linear_vel = self.checkLinearLimitVelocity(target_linear_vel + self.LIN_VEL_STEP_SIZE)
                    status            = status + 1
                    if( driving_mode == 2):
                        print(self.vels(0.0, target_linear_vel))
                    else:
                        print(self.vels(target_linear_vel, target_angular_vel))
                    
                elif key == 'x' :
                    ### down ###
                    target_linear_vel = self.checkAngularLimitVelocity(target_linear_vel - self.LIN_VEL_STEP_SIZE)
                    status            = status + 1
                    if( driving_mode == 2):
                        print(self.vels(0.0, target_linear_vel))
                    else:
                        print(self.vels(target_linear_vel, target_angular_vel))

                elif key == 'a' :
                    ### left ###
                    if (driving_mode == 2 or driving_mode == 3) :
                        target_angular_vel = 0.0
                    else :
                        target_angular_vel = self.checkAngularLimitVelocity(target_angular_vel + self.ANG_VEL_STEP_SIZE)
                    status = status + 1
                    print(self.vels(target_linear_vel, target_angular_vel))

                elif key == 'd' :
                    ### right ###
                    if (driving_mode == 2 or driving_mode == 3) :
                        target_angular_vel = 0.0
                    else :
                        target_angular_vel = self.checkAngularLimitVelocity(target_angular_vel - self.ANG_VEL_STEP_SIZE)
                    status = status + 1
                    print(self.vels(target_linear_vel, target_angular_vel))

                elif key == ' ' or key == 's' :
                    ### stop ###
                    target_linear_vel   = 0
                    control_linear_vel  = 0
                    target_angular_vel  = 0
                    control_angular_vel = 0
                    print(self.vels(target_linear_vel, target_angular_vel))

                elif key == 'n':
                    ### start navigation ###
                    return 'done'

                elif key == 'i' :
                    ### finish operation ###
                    return 'exit'

                else :
                    ### finish ###
                    if (key == '\x03'):
                        break

                ### print msg ###
                if status == 20 :
                    print(self.msg)
                    status = 0
                else :
                    pass

                ### publish cmd_vel ###
                twist = Twist()

                if (driving_mode == 2) :
                    control_linear_vel = self.makeSimpleProfile(control_linear_vel, target_linear_vel, self.LIN_VEL_STEP_SIZE)
                    twist.linear.x     = control_linear_vel
                    twist.linear.y     = 0
                    twist.linear.z     = 0

                    # add
                    # target_angular_vel = self.checkAngularLimitVelocity(target_angular_vel + self.ANG_VEL_STEP_SIZE)
                    # status = status + 5 # 1
                    twist.linear.x     = 0.2
                    twist.angular.z     = 0.65 #target_angular_vel
                    # print(self.vels(target_linear_vel, target_angular_vel))
                    print(twist)

                elif(driving_mode == 3):
                    control_linear_vel = self.makeSimpleProfile(control_linear_vel, target_linear_vel, self.LIN_VEL_STEP_SIZE)
                    twist.linear.x     = control_linear_vel
                    twist.linear.y     = 0
                    twist.linear.z     = 0

                else:
                    control_linear_vel  = self.makeSimpleProfile(control_linear_vel, target_linear_vel, self.LIN_VEL_STEP_SIZE)
                    control_angular_vel = self.makeSimpleProfile(control_angular_vel, target_angular_vel, self.ANG_VEL_STEP_SIZE)
                    twist.linear.x      = control_linear_vel
                    twist.linear.y      = 0
                    twist.linear.z      = 0
                    twist.angular.x     = 0
                    twist.angular.y     = 0
                    twist.angular.z     = control_angular_vel

                publish_driving_mode.publish(driving_mode)
                publish_cmd_vel.publish(twist)

                ### wait if robot change mode ###
                if ( change_mode_flag == 1):
                    print("Waiting...")
                    rospy.sleep(4)
                    change_mode_flag = 0
                    print("Ready.")
                else:
                    pass

        except:
            print(self.e)


    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


    def vels(self, target_linear_vel, target_angular_vel):
        return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)


    def makeSimpleProfile(self, output, input, slop):
        if input > output:
            output = min( input, output + slop )
        elif input < output:
            output = max( input, output - slop )
        else:
            output = input
        return output


    def constrain(self, input, low, high):
        if input < low:
          input = low
        elif input > high:
          input = high
        else:
          input = input
        return input


    def checkLinearLimitVelocity(self, vel):
        vel = self.constrain(vel, -self.LIMIT_LIN_VEL, self.LIMIT_LIN_VEL)
        return vel


    def checkAngularLimitVelocity(self, vel):
        vel = self.constrain(vel, -self.LIMIT_ANG_VEL, self.LIMIT_ANG_VEL)
        return vel


def main():
    rospy.init_node('navigation')

    sm_top = smach.StateMachine(outcomes=['succeeded'])
    with sm_top:
        smach.StateMachine.add('manual_operation',         Manual(), transitions={'done':'autonomous_operation', 'exit':'succeeded'})
	smach.StateMachine.add('autonomous_operation', Navigation(), transitions={'done':'manual_operation'    , 'exit':'succeeded'})

    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/navigation')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()