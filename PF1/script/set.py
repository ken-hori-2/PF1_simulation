#!/usr/bin/env python

import rospy
import csv
from move_base_msgs.msg import MoveBaseActionGoal
# from sync_sub.msg import some_position
# from sync_sub.msg import some_position2
from std_msgs.msg import Empty
from geometry_msgs.msg import Quaternion, PointStamped, PoseWithCovarianceStamped, TransformStamped, PoseStamped, Twist


class Set(object):
    def __init__(self):

        #self.rospy.init_node('goal_sub', anonymous=True)

        #rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self.callback)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.callback)
        self._pub = rospy.Publisher('set_coordinate', Empty, queue_size=10)
        self._msg = Empty()
        self._num = 0

        # self._f = open('/root/catkin_ws/src/pf1_simulation/PF1/script/test.csv', 'w')
        self._f = open('/root/catkin_ws/src/pf1_simulation/PF1/script/tsukuba.csv', 'w')
        self._writer = csv.writer(self._f, lineterminator='\n')
        self._csvlist = []
        self._array = []
        self._a = []



    def callback(self,data):
        if self._num == 0:
            self._array = 'num,x,y,z,q1,q2,q3,q4'
            print(self._array)

        self.pos = data.pose.pose

        self._array = (self._num,self.pos.position.x,self.pos.position.y,0.0,0.0,0.0,self.pos.orientation.z,self.pos.orientation.w)


        self._csvlist.append(self._array)

        self._a = self._csvlist[self._num]

        print(self._a)
        self._writer.writerow(self._csvlist[self._num])

        self._num +=1

       
        #self._f.close()



if __name__ == '__main__':
    #listener()
    rospy.init_node( 'kyolo_set' )
    goal_sub = Set()
    rospy.spin()