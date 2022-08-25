#!/usr/bin/env python

import rospy
import smach
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Empty

from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Header
from std_msgs.msg import String

from geometry_msgs.msg import Twist

import math
import time

from ros_beginner.srv import Kyolo,KyoloResponse

from sync_sub.msg import some_position
from sync_sub.msg import some_position2
import actionlib
from ros_beginner.msg import Practice2Action
from ros_beginner.msg import Practice2Goal
import csv
import tf
import pandas as pd


#a = 0
#class set_route(smach.State):

#    def __init__(self):
#        smach.State.__init__(self, outcomes=['done'])


#    def execute(self, userdata):

#        global route_name
#        global route_name2

#        print("Please input Crossroad Place...")
#        route_name = str(input("Crossroad: "))
#        route_name2 = str(input("Crossroad2: "))

#        return 'done'

class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'exit', 'crossroad', 'detectarea'])
        self.listener = tf.TransformListener()
        self.A = True
        self.B = True
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction) 
        self.client.wait_for_server()
        self.listener.waitForTransform("map", "/base_link", rospy.Time(), rospy.Duration(4.0))
        self.num = True
        self.num2 = True
        self.num3 = True
        self.counter = 0


        with open('/root/catkin_ws/src/ros_beginner/src/Tsukubacharenge2021/T.csv', 'r') as f:
            
            alltxt = f.readlines() 
            f.close() 
            endgyou = len(alltxt) 
            print(endgyou)
            self.endtxt = alltxt[endgyou-1].strip() 
            print(self.endtxt)
            print(self.endtxt[0])






            rospy.loginfo('LAST POINT : {}' .format(self.endtxt))
            if self.endtxt[1] != ',':
                self.ENDTXT = self.endtxt[0] + self.endtxt[1]
                rospy.loginfo('ENDTXT : {}' .format(self.ENDTXT))
            else:
                self.ENDTXT = self.endtxt[0]


            print(' GOAL POINT : {} ' .format(self.ENDTXT))

        print(" Please input First Crossroad Place... ")
        print(" Except GOAL POINT , Countinuous Point ")
        self.Crossroad = str(input("Crossroad: ")) #'2'
        while int(self.Crossroad) >= int(self.ENDTXT)-3:
            print('error!?')
            self.Crossroad = str(input("Crossroad: ")) #'2'
        print(' First Crossroad POINT : {} ' .format(self.Crossroad))


        self.Crossroad2 = str(input("Crossroad2: ")) #'3'
        while int(self.Crossroad2) >= int(self.ENDTXT)-1 or int(self.Crossroad) == int(self.Crossroad2):
            print('error!')
            self.Crossroad2 = str(input("Crossroad2: ")) #'3'
        print(' Second Crossroad POINT : {} ' .format(self.Crossroad2))
            

        self.DetectArea = str(input("DetectArea: ")) #'3'
        while int(self.DetectArea) >= int(self.ENDTXT)-1 or int(self.DetectArea) == int(self.Crossroad) or int(self.DetectArea) == int(self.Crossroad2) or int(self.DetectArea) == int(self.Crossroad)-1 or int(self.DetectArea) == int(self.Crossroad)+1 or int(self.DetectArea) == int(self.Crossroad2)-1 or int(self.DetectArea) == int(self.Crossroad2)+1:
            print('error!')
            self.DetectArea = str(input("DetectArea: ")) #'3'
        print(' Detect POINT : {} ' .format(self.DetectArea))
       
        self.df = pd.read_csv('/root/catkin_ws/src/ros_beginner/src/Tsukubacharenge2021/T2.csv', skiprows=int(self.Crossroad))
        
        #print(self.df)


    def execute(self, userdata):

        while self.A:
            with open('/root/catkin_ws/src/ros_beginner/src/Tsukubacharenge2021/T2.csv', 'r') as f:
                reader = csv.reader(f)
                print(' reader : {} ' .format(reader))
                
         
                for pose in reader:
                    print(' POSE 0 : {} ' .format(pose[0]))
                    print(' POSE  : {} ' .format(pose))
                    print(' Crossroad2 : {} ' .format(self.Crossroad2))
                    print(' endtxt  : {} ' .format(self.endtxt))

                    if not self.num:
                        if int(pose[0]) <= int(self.Crossroad):
                            print('skip!!!!!')
                            continue

                    if not self.num2:
                        if int(pose[0]) <= int(self.Crossroad2):
                            print('skip2222!!!!!')
                            continue


                    if not self.num3:
                        if int(pose[0]) <= int(self.DetectArea):
                            print('SKIP3!!!!!')
                            continue

                    print("\nHeading to {}!".format(pose[0]))
                    goal = self.goal_pose(pose)
                    self.client.send_goal(goal)
                    while True:
                        now = rospy.Time.now()
                        self.listener.waitForTransform("map", "/base_link", now, rospy.Duration(4.0))
                        position, quaternion = self.listener.lookupTransform("map", "/base_link", now)


                        ####----TEST----####
                        if pose[0] == self.Crossroad or pose[0] == self.Crossroad2:
                            print('Crossroad -> continue')
                            if(math.sqrt((position[0]-goal.target_pose.pose.position.x)**2 + (position[1]-goal.target_pose.pose.position.y)**2 ) <= 0.0):
                                break
                            #continue
                        if(math.sqrt((position[0]-goal.target_pose.pose.position.x)**2 + (position[1]-goal.target_pose.pose.position.y)**2 ) <= 0.5):

                            

                            print("->Reached {}! Next head to {}!".format(pose[0],int(pose[0])+1))
                            #rospy.loginfo(pose[0])
                            break
                        else:
                            rospy.sleep(0.5)

                    if pose[0] == self.Crossroad:    #Crossroad  self.crossroad == '2'
                        rospy.loginfo('reached Crossroad!') 
           
                        print(pose)
                        if self.num:
                         self.num = False
                         #return 'done'
                         return 'crossroad'

                    if pose[0] == self.Crossroad2:    #Crossroad  self.crossroad == '2'
                        rospy.loginfo('reached Crossroad!') 
           
                        #print(pose)
                        if self.num2:
                         self.num2 = False
                         return 'crossroad'

                    if pose[0] == self.DetectArea:    #Crossroad  self.crossroad == '2'
                        rospy.loginfo('reached Detect area!') 
           
                        print(pose)
                        if self.num3:
                         self.num3 = False
                         #return 'done'
                         return 'detectarea'
                         

                    if pose[0] == self.ENDTXT: #'4':
                        self.A = False
                            
                rospy.loginfo("Finish")

        return 'exit'
        

    def goal_pose(self, pose):

        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.pose.position.x = map(float,pose)[1]
        goal_pose.target_pose.pose.position.y = map(float,pose)[2]
        goal_pose.target_pose.pose.position.z = map(float,pose)[3]
        goal_pose.target_pose.pose.orientation.x = map(float,pose)[4]
        goal_pose.target_pose.pose.orientation.y = map(float,pose)[5]
        goal_pose.target_pose.pose.orientation.z = map(float,pose)[6]
        goal_pose.target_pose.pose.orientation.w = map(float,pose)[7]

        return goal_pose



class YOLO(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'crossroad'])
        self._vel_pub = rospy.Publisher( 'diff_drive_controller/cmd_vel', Twist, queue_size = 10 )
        self._vel = Twist()
        

    def execute(self, userdata):
        self.a = False
        self._sub1 = rospy.Subscriber('chatter1', some_position2,self.callback)    #Class nomi xmax hanasi
            #self.callback()
            #rospy.sleep(3.0)
        
            #def callback(self,data):
            #while data.Class == 'traffic light':
        rospy.loginfo('Executing state YOLO START...')
        rospy.sleep(3.0)
        rospy.loginfo('waiting service...')
    
        rospy.wait_for_service('bbox')
        try:
            service = rospy.ServiceProxy('bbox', Kyolo)
            response = service() #client -> server -> client
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        print '->',response.Class
        #print '->',response.position     #1

        
        if self.a:     #response.Class == 'traffic light':#'Process Started':
            rospy.loginfo(' Detect traffic light ! STOP !')
            self._vel.angular.z = self.Velocity
            self._vel_pub.publish(self._vel)

            self._vel.linear.x = 0
            self._vel_pub.publish(self._vel)
            rospy.sleep(5.0)
            return 'done'
        else:
            rospy.loginfo(' retry!!!! ')
            self._vel.angular.z = 0.8 #0.5
            self._vel_pub.publish(self._vel)    ####----ADD----####
            #rospy.loginfo(self._x)
            return 'crossroad'


    def callback(self, data):
        self.Width=(data.xmax-data.xmin)
        self.Velocity=(((4.00-(((data.xmin + data.xmax)*0.01)/2))/2)*0.25*math.pi)
        #rospy.loginfo(data.Class)                              #0
        if data.Class == "traffic light":
            #print('Detect :{}'.format(data.Class))
            self.a = True
            
            




class Detect(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed', 'aborted'])
        self.counter = 0
        
    def execute(self, userdata):
        rospy.loginfo(' Executing state DETECT START... ')
        rospy.sleep(2.0)
        action_client = actionlib.SimpleActionClient( 'action', Practice2Action )
        action_client.wait_for_server() # Wait until the server is ready     
        # Set GoUntilBumperGaol's instance
        goal = Practice2Goal() 

        #goal.target_vel.linear.x = 1.0 #0.8
        goal.timeout_sec = 15 #10
    
        action_client.send_goal( goal ) # Send data ( Publish to topic bumper_action/goal )
        action_client.wait_for_result() # wait for result
        
        result = action_client.get_result()
        if result.bumper_hit: 
            rospy.loginfo( ' bumper hit! ' )
            return 'succeeded'
            
        else:
            rospy.loginfo( 'faild!' )
            #self.counter += 1
            if self.counter < 3:
                self.counter += 1
                return 'failed'
            else:
                rospy.loginfo(' Try 10 sec * 3 times but it failed ! ')
                rospy.loginfo(' Aborted ! ')
                return 'aborted'
                #return 'failed'


        
        
class Integration(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        
        self._vel_pub = rospy.Publisher('diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self._vel = Twist()
        self._red_area = 0
        #rate = rospy.Rate(50)  IMPORTANT roop velocity
        #rate.sleep()           last

    def execute(self, userdata):

        rospy.loginfo('Executing state YOLO START...')
        #rospy.sleep(5.0)
        rospy.loginfo('waiting service...')
        self._sub2 = rospy.Subscriber('chatter2', some_position , self.call)
        self._sub1 = rospy.Subscriber('chatter1', some_position2 ,self.callback)


        return 'done'

    def callback(self, data):
        #rospy.loginfo(data.Class)                              #0
        if data.Class == "traffic light":
            if self._red_area > 500:
                rospy.loginfo("STOP!")                   #2
                self._vel.linear.x = 0
                self._vel_pub.publish(self._vel)


    def call(self,msg1):
        #rospy.loginfo('red_area red_area red_area = %d' %(msg1.position[1]))  #3
        self._red_area = msg1.position[1]
    



# main
def main():
    rospy.init_node('navigation')

    sm_top = smach.StateMachine(outcomes=['succeeded'])
    with sm_top:
        
        smach.StateMachine.add('Navigation', Navigation(), transitions={'done':'Navigation', 'exit':'succeeded','crossroad':'YOLO','detectarea':'Detect'})
        smach.StateMachine.add('YOLO', YOLO(), transitions={'done':'Navigation', 'crossroad':'YOLO'})    #, 'done2', 'Detect'
        smach.StateMachine.add('Detect', Detect(), transitions={'succeeded':'Navigation', 'failed':'Detect', 'aborted':'Navigation'})    # , 'done2':'YOLO'


        #rennzoku deha dekinai


        #smach.StateMachine.add('Integration', Integration(), transitions={'done':'Navigation'})
        #smach.StateMachine.add('goal_navigation', goal_navigation(), transitions={'next':'target_navigation', 'done':'succeeded'})

    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/navigation')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
