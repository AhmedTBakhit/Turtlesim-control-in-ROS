#!/usr/bin/env python3
# license removed for brevity
from cmath import sqrt
from colorsys import TWO_THIRD
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from turtlesim.srv import Spawn 
import math
import numpy as np



# make a node in a single class
class Turtle1ControllerNode:
    
    # It is common in Python to put all your class variables at the 
    # very top of the class.
    turtlex = 0
    turtley = 0
    turtler = 0

    desired_x = 0
    desired_y = 0

    # Every python class should have an __init__ function. For those who
    # know C/C++, it is like the constructor for a C++ class
    def __init__(self) -> None:
        
        # In python, class variables are accessed using the 'self' object
        self.desired_x = 5
        self.desired_y = 5

        # Note how I moved all the initialization code for the node into the
        # __init__ function of this class.  
        self.cmd_vel_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        #self.cmd_vel_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/turtle2/pose', Pose, self.on_turtle1_pose_updated)
        self.set_pen_svc = rospy.ServiceProxy('turtle2/set_pen', SetPen)
        #self.spawn_svc = rospy.ServiceProxy('turtle1/spawn', Spawn)
        rospy.init_node('turtle_face', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz

    # this is the callback function that gets called when we subscribe to
    # the Pose topic of the turtlesim. this function is not inside the class and can use the member
    # variables for the turtle position
    # IMPORTANT: Every python class function has to have 'self' as the first
    #            argument
    def on_turtle1_pose_updated(self, new_pose):
        self.turtlex = new_pose.x
        self.turtley = new_pose.y
        self.turtler = new_pose.theta

    # A simple function for computing the appropriate linear and 
    # angular velocity of the turtle to reach the desired point 
    # Note again that self is the first argument and that I used
    # the self object to access the class variables
    def compute_turtle_velocity(self):
        
        linear_gain = 0.25
        angular_gain = 0.5

        deltax = self.desired_x-self.turtlex
        deltay = self.desired_y-self.turtley
        self.distance_to_goal = math.sqrt(deltax*deltax + deltay*deltay)

        R = np.array([[np.cos(self.turtler),-np.sin(self.turtler)],[np.sin(self.turtler),np.cos(self.turtler)]])
        T = np.array([[self.turtlex],[self.turtley]])

        Pd_w = np.array([[self.desired_x],[self.desired_y]])
        Rt = np.transpose(R)
        
        Pd_r = Rt@Pd_w - Rt@T

        #print(f'Rw:({self.turtlex},{self.turtley}) Pw:({self.desired_x},{self.desired_y}) Pr:({Pd_r[0]},{Pd_r[1]}) ')
        
        angle = math.atan2(Pd_r[1], Pd_r[0])

        cmd_vel = Twist()
        cmd_vel.linear.x = linear_gain * self.distance_to_goal
        cmd_vel.angular.z = angular_gain * angle

        return cmd_vel

    # This is a function for using the ROS service to set the pen state
    # Note that I made the first non-self argument required and the others
    # optional with default values.
    def set_pen_off(self,off,r=255,g=0,b=0,width=2):
        self.set_pen_svc(r,g,b,width,off)

    def spawn_turtle(self):
        self.spawn_svc(3,3,0,"")    

    # This is a useful utility function to know when we are close enough
    # to one point to move on to the next point
    def get_distance_to_goal(self):
        return self.distance_to_goal

    # This allows us to change the point we are trying to drive to
    def set_desired_point(self, x, y):
        self.desired_x = x
        self.desired_y = y

        # Need to update the distance to goal when a new goal is made
        deltax = self.desired_x-self.turtlex
        deltay = self.desired_y-self.turtley
        self.distance_to_goal = math.sqrt(deltax*deltax + deltay*deltay)

    # This function runs the controller and publishes the updated velocities
    def run_turtle_controller(self):
        cmd_vel = self.compute_turtle_velocity()
        self.cmd_vel_pub.publish(cmd_vel)

    #def rotate(self,vel_pub,ang_speed,relative_angle_degree,clockwise):
        #velocity_message = Twist()
        #angular_speed = math.radians(abs(ang_speed))

        #if (clockwise):
            #velocity_message.angular.z   

#draws the first half of the eye
    def draw_circle1(self):
        #self.set_desired_point(5,1)
        self.set_pen_off(False)
        self.set_desired_point(4,7)        
        while not rospy.is_shutdown() and self.get_distance_to_goal() > 0.1:
            self.run_turtle_controller()
            self.sleep()

#draws the second half of the eye
    def draw_circle2(self):
        #self.set_desired_point(5,1)
        self.set_pen_off(False)
        self.set_desired_point(4,8)        
        while not rospy.is_shutdown() and self.get_distance_to_goal() > 0.1:
            self.run_turtle_controller()
            self.sleep() 

#draws the first half of the second eye
    def draw_circle3(self):
        #self.set_desired_point(5,1)
        self.set_pen_off(False)
        self.set_desired_point(6,7)        
        while not rospy.is_shutdown() and self.get_distance_to_goal() > 0.1:
            self.run_turtle_controller()
            self.sleep()  

#draws the second half of the second eye
    def draw_circle4(self):
        #self.set_desired_point(5,1)
        self.set_pen_off(False)
        self.set_desired_point(6,8)        
        while not rospy.is_shutdown() and self.get_distance_to_goal() > 0.1:
            self.run_turtle_controller()
            self.sleep()                                
# goes to starting point for the eye
    def starting_point(self):
        node.set_pen_off(True)
        node.set_desired_point(4,8)
        while not rospy.is_shutdown() and self.get_distance_to_goal() > 0.1:
            self.run_turtle_controller()
            self.sleep()

#goes to starting point of the ssecond eye
    def starting_point2(self):
        node.set_pen_off(True)
        node.set_desired_point(6,8)
        while not rospy.is_shutdown() and self.get_distance_to_goal() > 0.1:
            self.run_turtle_controller()
            self.sleep() 

#goes to the starting point of the nose
    def starting_point3(self):
        node.set_pen_off(True)
        node.set_desired_point(4,6)
        while not rospy.is_shutdown() and self.get_distance_to_goal() > 0.1:
            self.run_turtle_controller()
            self.sleep()  

#draws the first part part of the nose
    def draw_circle5(self):
        #self.set_desired_point(5,1)
        self.set_pen_off(False)
        self.set_desired_point(3.5,5.5)        
        while not rospy.is_shutdown() and self.get_distance_to_goal() > 0.1:
            self.run_turtle_controller()
            self.sleep()   
            
#draws the second part of the nose
    def draw_circle6(self):
        #self.set_desired_point(5,1)
        self.set_pen_off(False)
        self.set_desired_point(4,5)        
        while not rospy.is_shutdown() and self.get_distance_to_goal() > 0.1:
            self.run_turtle_controller()
            self.sleep()                        

    def sleep(self):
        self.rate.sleep()

if __name__ == '__main__':
    try:
        node = Turtle1ControllerNode()
        # node.set_pen_off(True)
        # node.set_desired_point(5,1)
        # node.draw_circle1

        # Create a desired location and then move until you are "close enough"
        # Drives with the pen lifted
        
        node.spawn_turtle

        node.starting_point()
        node.draw_circle1()
        node.draw_circle2()
        node.starting_point2()
        node.draw_circle3()
        node.draw_circle4()
        node.starting_point3()
        node.draw_circle5()
        node.draw_circle6()

        
        


    except rospy.ROSInterruptException:
        pass