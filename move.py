#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import math
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Int32

class move1():
  def __init__(self):
    
    # Name this node, it must be unique
    rospy.init_node('move1', anonymous=True)
        
    # Enable shutdown in rospy
#    rospy.on_shutdown(self.shutdown)

    self.vertices = np.array([1,9,17,25,33,41,49,57,65,66,67,59])
    self.v = self.vertices.tolist()
    print self.v
   # Establish publishers and subscribers
    self.goal_pub = rospy.Publisher("/goal",Int32,queue_size = 1) 
    self.currentnode_sub = rospy.Subscriber("/current_node",String,self.callback)
#    self.goal_pub = rospy.Publisher("/goal",Int32,queue_size = 1)    

  def callback(self,data):

    while not rospy.is_shutdown():
      j = self.v.index(int(data.data))
      print j
      for x in (j,self.vertices.size-1):
#      for x in (int(self.vertices[np.where(data.data)]),self.vertices.size-1):
        print x
        while(self.vertices[x+1] != data.data):
          self.goal_pub.publish(self.vertices[x+1])
          rospy.sleep(5)
        j = self.v.index(int(data.data))

      rospy.sleep(5)

#    def shutdown(self):
#      rospy.sleep(0.1)

if __name__ == '__main__':
  try:
    move1()
    rospy.spin()
  except rospy.ROSInterruptException:
    rospy.loginfo("Goal reached.")
