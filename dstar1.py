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


   # returns updated pose 
  def callback(msg):
    
#    print self.pose
    pose_x = msg.pose.pose.position.x
    pose_y = msg.pose.pose.position.y

    quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    pose_th = euler[2]

    pose = [pose_x, pose_y, pose_th]
    return pose

  # upadtes the edge costs
  def cost_update(data,self):

    if self.up_flag:
      data = data.data.split()
      indx = np.where(self.source==int(data[0]))

      for i in range(len(data)/3):
        self.dest[indx+i] = int(data[i*3 + 1])
        if data[i*3+2] == 'inf':
          self.cost[indx +i*3+ 2] = np.Inf
        else: 
          self.cost[indx +i*3+ 2] = int(data[i*3 + 2])

        update_vertex(self.source[indx+i],self)
    return

     # returns neighbors as the index in dest array 
  def getNeighbor(u,self):

    self.ind = [k for (k,val) in enumerate(self.source) if val == u]
    return self.ind


  def __init__(self):
    
    # Name this node, it must be unique
    rospy.init_node('dstarlite', anonymous=True)

    # Establish publishers and subscribers
    #self.pose = 
    rospy.Subscriber("odom", Odometry, callback)
    self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    #self.currentnode_sub = rospy.Subscriber("/current_node",String,self.callback)
    rospy.Subscriber("edge_costs", String, cost_update)

    rospy.sleep(1.0)        
    # Enable shutdown in rospy
#    rospy.on_shutdown(self.shutdown)

    # Read input data

    # Read nodes, start, goal position and edge costs from input.txt file
#    self.source = []
#    self.dest = []
#    self.cost = []
    cnt = len(open('/home/urvi/mini_project/ROS_MINI_PROJECT/src/ros_demo/src/input.txt').readlines(  ))

    f1 = open('/home/urvi/mini_project/ROS_MINI_PROJECT/src/ros_demo/src/input.txt','r')
    self.n = int(f1.readline())
    self.Xo = int(f1.readline())
    self.Xg = int(f1.readline())

    self.source = np.empty([cnt-3,1])
    self.dest = np.empty([cnt-3,1])
    self.cost = np.empty([cnt-3,1])
    self.source,self.dest,self.cost = np.loadtxt(f1, unpack=True)

    f1.close()

    # Read coordinates from coord.txt file
    self.x_co = np.empty([self.n,1])
    self.y_co = np.empty([self.n,1])

    f1 = open('/home/urvi/mini_project/ROS_MINI_PROJECT/src/ros_demo/src/coords.txt')

    self.x_co,self.y_co = np.loadtxt(f1, unpack=True)

    # Define parameters

    self.vertices = np.array([1,9,17,25,33,41,49,57,65,66,67,59])
    self.open = []
    self.up_flag = 0
    self.k_m = 0
    self.g  = np.empty([self.n+1,1])
    self.rhs = np.empty([self.n+1,1])
    self.key = np.empty([self.n+1,1])

    self.g  = np.Inf#*([self.n+1,1])
    self.rhs = np.Inf#*([self.n+1,1])
    self.key = np.Inf#*([self.n+1,1])
    self.pose = [0.0, 0.0, 0.0]
    slast = self.Xo

#    self.pose = rospy.Subscriber("odom", Odometry, callback_pose)
#    self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    #self.currentnode_sub = rospy.Subscriber("/current_node",String,self.callback)
#    rospy.Subscriber("edge_costs", String, cost_update)

    rospy.sleep(1.0)
    
    self.rhs[self.Xg] = 0
    self.open.append(self.Xg)
    self.key[self.Xg] = calculate_key(self.Xg,self)
    self.up_flag = 1

    rospy.sleep(4.0)
    self.up_flag = 0

    ComputeShortestPath(self)

    while(self.Xo!= self.Xg):
      if (getG(self.Xo,self)==np.Inf):
        print 'No path found \n'
        exit()
################################################# CHECK FROM HERE ##################################
      index1 = getNeighbor(self.Xo,self)
      for l in len(index1):
        tmp.append = self.cost[index1(l)] + self.g[index1(l)]
      ind = np.argmin(tmp)
      self.Xo = getNeighbor(ind,self)
      print 'Move to: ', self.Xo

      self.move = [self.x_co[self.Xo], self.y_co[self.Xo]]

      while not at_goal(self):
         goToGoal(self)
         rospy.sleep(0.1)

      twist=Twist()
      twist.linear.x= 0
      twist.angular.z = 0
      self.vel_pub.publish(twist)   
      print 'Reached'

      self.k_m = self.k_m + h(slast,self.Xo,self)
      slast = self.Xo
      self.up_flag = 1
      rospy.sleep(2.0)
      self.up_flag = 0
      if self.open != []:            
        ComputeShortestPath(self)

    print "Reached the Goal"      

    # checks if reached the goal 
  def at_goal(self):
    
    distThresh = 0.1
    
    # goal location
    xd = self.move[0]
    yd = self.move[1]
    
    # current robot location
    xc = self.pose[0]
    yc = self.pose[1]
    
    #check if we have reached goal point
    d = np.sqrt(pow((xd - xc),2) + pow((yd - yc),2))
    
    if d <= distThresh:
        print "Reached goal"
        return True
    else:
        return False

   #The Go to goal controller
  def goToGoal(self):

    #Controller parameters
    Kp = 0.02
       
    # goal location
    xd = self.move[0]
    yd = self.move[1]
    
    # current robot location
    xc = self.pose[0]
    yc = self.pose[1]
    phi = self.pose[2]
    
    # determine how far to rotate to face the goal point
    # ANGLES ARE IN RADIANS
    dt = (np.arctan2((yd - yc), (xd -xc))) - phi

    #restrict angle to (-pi,pi)
    dt = ((dt + np.pi)%(2.0*np.pi)) - np.pi
    dt = ((dt*180.0)/np.pi)
        
    # control input for angular velocity
    W = (Kp*dt) 
  
    # distance to goal
    d = np.sqrt(pow((xd - xc),2) + pow((yd - yc),2))
    
    #control input for linear velocity
    if d > 0.05 :
        V = 0.2
    else :
        V = 0.0
    
    twist=Twist()
    twist.linear.x= V
    twist.angular.z = W
    self.vel_pub.publish(twist)   

    return

    # returns eucildean distance as heuristic
  def h(x1,x2,self):

    indx1 = self.x_co[int(x1)-1]
    indx2 = self.x_co[int(x2)-1]
    indy1 = self.y_co[int(x1)-1]
    indy2 = self.y_co[int(x2)-1]

    return math.sqrt((self.x_co[indx1]-self.x_co[indx2])**2 + (self.y_co[indy1]-self.y_co[indy2])**2)

    # returns g estimate
  def getG(u,self):

    if (u == Xg):
      return 0
    else:
      index = getNeighbor(u,self)
      for l in len(index):
        g_new.append = self.cost[index(l)] + self.g[index(l)]
      self.g[u] = np.amin(g_new)
      return self.g[u]
#        print self.g[u]

    # returns rhs estimate
  def getrhs(u,self):

    if (u == Xg):
      return 0
    else:
      index = getNeighbor(u,self)
      for l in len(index):
        rhs_new.append = self.cost[index(l)] + self.g[index(l)]
      self.rhs[u] = np.amin(rhs_new)
      return self.rhs[u]

   # returns key
  def calculate_key(u,self):
#      self.key = min(getG[u,self) , getrhs(u,self) + h(self.start,u)+ self.km]
    return min(getG(u,self) , getrhs(u,self) + h(self.Xo,u)+ self.km)

  def update_vertex(u,self):
    if u!=Xg:
      self.rhs[u] = getrhs(u,self)
    if u in open:
      self.open.remove(u)
    if(self.g[u]!= self.rhs[u]):
      self.open.append(u)
      self.key[u] = calculate_key(u,self)

  def ComputeShortestPath(self):
    while ((min(self.key[x] for x in self.open)< calculate_key(self.Xo,self)) or (self.rhs[self.Xo] != self.g[self.Xo])):
      k_old = min(self.key[x] for x in self.open) 
      u = [x for x in self.open if self.key[x] == min(self.key[x] for x in self.open)][0]
#      print u
      self.open.remove(u)
      if k_old < calculate_key(u,self):
        self.open.append(u)
        self.key[u] = calculate_key(u,self)
      elif getG(u,self) > getrhs(u,self):
        self.g[u] = self.rhs[u]
        index = getNeighbor(u,self)
        index.append(u)
        for x in len(index):
          update_vertex(index(x),self)     
      else:
        self.g[u] = np.Inf
        index = getNeighbor(u,self)
        index.append(u)
        for x in len(index):
          update_vertex(index(x),self)     

      if (self.open == []) and (getrhs(u,self) == getG(u,self)):
          break


#  def shutdown(self):
#    rospy.sleep(0.1)



if __name__ == '__main__':
  try:
    dstar1()
    rospy.spin()
  except rospy.ROSInterruptException:
    rospy.loginfo("Goal reached.")
