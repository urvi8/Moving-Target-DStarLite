#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import math
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Int32

class dstar1():
  def __init__(self):
    
    # Name this node, it must be unique
    rospy.init_node('dstar', anonymous=True)
        
    # Enable shutdown in rospy
#    rospy.on_shutdown(self.shutdown)

    # Read input data
    self.source = []
    self.dest = []
    self.cost = []
    cnt = len(open('/home/urvi/mini_project/src/ROS_MINI_PROJECT/src/ros_demo/src/input.txt').readlines(  ))

    f1 = open('/home/urvi/mini_project/src/ROS_MINI_PROJECT/src/ros_demo/src/input.txt','r')
    self.n = int(f1.readline())
    self.Xo = int(f1.readline())
    self.Xg = int(f1.readline())

    self.source = np.empty([cnt-3,1])
    self.dest = np.empty([cnt-3,1])
    self.cost = np.empty([cnt-3,1])
    self.source,self.dest,self.cost = np.loadtxt(f1, unpack=True)

#    for l in f1:
#      row=l.split()
#      self.source.append(row[0])
#      self.dest.append(row[1])
#      self.cost.append(row[2])
    f1.close()

    self.x_co = np.empty([self.n,1])
    self.y_co = np.empty([self.n,1])

    f1 = open('/home/urvi/mini_project/src/ROS_MINI_PROJECT/src/ros_demo/src/coords.txt')
#    for l in f1:
#      row=l.split()
#      self.x_co.append(row[0])
#      self.y_co.append(row[1])
#    f1.close()

    self.x_co,self.y_co = np.loadtxt(f1, unpack=True)

    # Define parameters

    self.vertices = np.array([1,9,17,25,33,41,49,57,65,66,67,59])
    self.open = []
    self.close = []
    self.k_m = 0

    # Establish publishers and subscribers
    
    self.currentnode_sub = rospy.Subscriber("/current_node",String,self.callback)
#    self.goal_pub = rospy.Publisher("/goal",Int32,queue_size = 1)    


#  def main1(self):
#    self.n1 = getNeighbor(self.vertices[0],self)
#    return self.n1
#    self.H = h(self.Xo,self.Xg,self)
#    return self.H, self.n1

#  def getG(u,self):
#    if (u==Xg):
#      return 0
#    else:
#      m =
      
  
  def callback(self,data):
    
#    for x in (1,self.vertices.size-1):
      # returns neighbors as the index in dest array 
    def getNeighbor(u,self):

      self.ind = [k for (k,val) in enumerate(self.source) if val == u]
      return self.ind

    def h(x1,x2,self):

      indx1 = self.x_co[int(x1)-1]
      indx2 = self.x_co[int(x2)-1]
      indy1 = self.y_co[int(x1)-1]
      indy2 = self.y_co[int(x2)-1]

      return math.sqrt((self.x_co[indx1]-self.x_co[indx2])**2 + (self.y_co[indy1]-self.y_co[indy2])**2)


    def shutdown(self):
      rospy.sleep(0.1)

#      while(data.data != self.vertices[x+1]):
#       self.goal_pub.publish(self.vertices[x+1])

#       rospy.sleep(1)
    
#    while not rospy.is_shutdown():
#      n1 = getNeighbor(self.vertices[1],self)
#      H = h(self.Xo,self.Xg,self)
     
#  def update_vertex(u):

#    if (u!= Xg):
#      rhs[u] = min


#  def main(self):
#    self.vertices[-1] = Xo
#    initialize()
#    ComputeShortestPath()
#    while(Xo != Xg)
      

#  def initialize(self):
    

#  def ComputeShortestPath(self):
    


if __name__ == '__main__':
  try:
    dstar1()
    rospy.spin()
  except rospy.ROSInterruptException:
    rospy.loginfo("Goal reached.")
