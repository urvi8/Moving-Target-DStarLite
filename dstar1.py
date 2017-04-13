#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import math
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Int32

class dstar():
  def __init__(self):
    
    # Name this node, it must be unique
    rospy.init_node('dstar', anonymous=True)
        
    # Enable shutdown in rospy
    rospy.on_shutdown(self.shutdown)

    # Define parameters
    #self.cost = np.array
    self.vertices = np.array([1,9,17,25,33,41,49,57,65,66,67,59])
    self.open = []
    self.close = []
    self.pred = np.array([][])
    self.succ = np.array([][])
    self.k_m = 0
    self.
    print self.vertices
    print self.vertices.size

    
    # Establish publishers and subscribers
    
    self.currentnode_sub = rospy.Subscriber("/current_node",String,self.callback)
    self.goal_pub = rospy.Publisher("/goal",Int32,queue_size = 1)    
    
  def shutdown(self):
    rospy.sleep(0.1)

  def getG(u):
    if (u==Xg):
      return 0
    else:
      m = succ(u)
      
      

#  def callback(self,data):
    
#    for x in (0,self.vertices.size-1):
      
#      print data.data
#      print x 
      #while(data.data != self.vertices[x]):
#       self.goal_pub.publish(self.vertices[x])

#       rospy.spin()
    
    
  def update_vertex(u):

    if (u!= Xg):
      rhs[u] = min


    
# initialize open, close as lists
# predecessors, successors, g, rhs as two-dimensional arrays and cost as one-dimensional array.
# k_m, k_old ??
# heuritsic will be a function



if __name__ == '__main__':
  try:
    dstar()
    rospy.spin()
  except rospy.ROSInterruptException:
    rospy.loginfo("Goal reached.")
