#!/usr/bin/env python3
# license removed for brevity
#!/usr/bin/env python3
# license removed for brevity

#importing necessary libraries
from __future__ import print_function
from logging import error
import math
import rospy
import sys
import cv2
import numpy as np
from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
import roslib
roslib.load_manifest('vision_based_manipulation')
from cv_bridge import CvBridge, CvBridgeError
from csv import writer

class image_converter:

    def __init__(self):                                                                                       #defining global variables and subscribers
        self.bridge = CvBridge()                                                          
        self.image_sub = rospy.Subscriber("/vbmbot/camera1/image_raw",Image,self.img_callback)               
        self.joint_state_sub = rospy.Subscriber("/vbmbot/joint_states",JointState,self.joint_state_callback)
        self.x_green = 0                                                                                      #centre coordinates for each cylinder initialised as 0
        self.y_green = 0                  
        self.x_blue = 0
        self.y_blue = 0
        self.x_red = 0
        self.y_red = 0
        self.x_purple = 0
        self.y_purple = 0
        self.s = 0                                                                                            #variable to capture current position
        self.s_star = 0                                                                                       #variable to capture reference position
        self.theta1 = 0                                                                                       #joint angle1
        self.theta2 = 0                                                                                       #joint angle2
        self.flag = 0                                                                                         #Boolean flag for switching from position to velocity controllers
        self.rate = rospy.Rate(1000)
        
    def talker(self):		
        pub_q1_pos = rospy.Publisher('/vbmbot/joint1_position_controller/command', Float64, queue_size=10)    #Publishing q1,q2 pos to move robot arm
        pub_q2_pos = rospy.Publisher('/vbmbot/joint2_position_controller/command', Float64, queue_size=10)
        
        rospy.sleep(5) 

        #Arm position 1
        q1_pos = 0.0
        q2_pos = -0.78
        pub_q1_pos.publish(q1_pos)
        pub_q2_pos.publish(q2_pos)
        rospy.sleep(5)

        # Reference position
        self.s_star = np.array([self.x_green, self.y_green, self.x_blue, self.y_blue, 
                                self.x_red, self.y_red, self.x_purple, self.y_purple])
        print("Reference cylinder centre positions are", self.s_star)

        #Arm position 2
        q1_pos = 0.52
        q2_pos = -0.52
        pub_q1_pos.publish(q1_pos)
        pub_q2_pos.publish(q2_pos)
        rospy.sleep(5)

        # Current position
        self.s = np.array([self.x_green, self.y_green, self.x_blue, self.y_blue, 
                            self.x_red, self.y_red, self.x_purple, self.y_purple])
        print("Current cylinder centre positions are", self.s)

        rospy.wait_for_service('/vbmbot/controller_manager/switch_controller')                                 #inbuilt switch controller to switch from position to velocity controller

        try:
            sc_service = rospy.ServiceProxy('/vbmbot/controller_manager/switch_controller', SwitchController)  #switching controllers
            start_controllers = ['joint1_velocity_controller','joint2_velocity_controller']
            stop_controllers = ['joint1_position_controller','joint2_position_controller']
            strictness = 2
            start_asap = False
            timeout = 0.0
            self.flag = 1                                                                                      
            res = sc_service(start_controllers,stop_controllers, strictness, start_asap,timeout)
        except rospy.ServiceException as e:
            print("Service Call Failed")


    def thresholding_color_mask(self, img, lower_color, upper_color):                                          #function for thresholding
        hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)                                                    
        color_hsv_mask = cv2.inRange(hsv_image, lower_color, upper_color)
        m = cv2.moments(color_hsv_mask)
        x_color = int(m["m10"] / m["m00"])
        y_color = int(m["m01"] / m["m00"])
        return [x_color,y_color]

    def img_callback(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.finding_centres()             #calling finding centers function to give centre points for each cylinder when robot traversing from current to reference position
            if self.flag == 1:                 #conditional flag which is set to 1 when switched to velocity controller 
                self.s = np.array([self.x_green, self.y_green, self.x_blue, self.y_blue,                     #Following this, all the current position will be stored in an accumulator
                            self.x_red, self.y_red, self.x_purple, self.y_purple])
                accumulator = []                                                                             #accumulator will be used to plot the feature's xy positional behavior
                accumulator.append(self.s[0])
                accumulator.append(self.s[1])
                accumulator.append(self.s[2])
                accumulator.append(self.s[3])
                accumulator.append(self.s[4])
                accumulator.append(self.s[5])
                accumulator.append(self.s[6])
                accumulator.append(self.s[7])
                file_n = "XYplot.csv"                                                                       #creating csv file of stored values- to be used in matlab for plotting
                with open(file_n, 'a') as f_object:                                                         #code to create csv file
                    accu = writer(f_object)
                    accu.writerow(accumulator)
                    f_object.close()
                self.visual_servoing()                                                                      #function visual servoing initiated
        except CvBridgeError as e:
            print(e)

    def finding_centres(self):                                                                              #function for calculating centres of cylinders
        lower_green = (55, 250, 45)
        upper_green = (65, 255, 255)
        lower_blue = (115, 250, 45)
        upper_blue = (125, 255, 255)
        lower_red = (0, 250, 45)
        upper_red =  (10, 255, 255)
        lower_purple = (145, 250, 45)
        upper_purple = (155, 255, 255)

        [self.x_green, self.y_green]= self.thresholding_color_mask(self.cv_image, lower_green, upper_green)
        [self.x_blue, self.y_blue] = self.thresholding_color_mask(self.cv_image, lower_blue, upper_blue)
        [self.x_red, self.y_red] = self.thresholding_color_mask(self.cv_image, lower_red, upper_red)
        [self.x_purple, self.y_purple] = self.thresholding_color_mask(self.cv_image, lower_purple, upper_purple)

        marking_image = self.cv_image.copy()

        cv2.circle(marking_image, ((int)(self.x_green), (int)(self.y_green)), 3, (0,0,0), -1)
        cv2.circle(marking_image, ((int)(self.x_blue), (int)(self.y_blue)), 3, (0,0,0), -1)
        cv2.circle(marking_image, ((int)(self.x_red), (int)(self.y_red)), 3, (0,0,0), -1)
        cv2.circle(marking_image, ((int)(self.x_purple), (int)(self.y_purple)), 3, (0,0,0), -1)

        cv2.imshow("circle_centres_image", marking_image)
        cv2.waitKey(5)

    def joint_state_callback(self, msg):
        self.theta1 = msg.position[0]
        self.theta2 = msg.position[1]
        
    def visual_servoing(self):
        self.pub_q1_vel = rospy.Publisher('/vbmbot/joint1_velocity_controller/command', Float64, queue_size=10)  #publishing joint velocities to joints q1 and q2
        self.pub_q2_vel = rospy.Publisher('/vbmbot/joint2_velocity_controller/command', Float64, queue_size=10)
        
        e = np.subtract(self.s, self.s_star)                                                                     #calculating error
        print("Error is", e)
        print("s", self.s)

        f = 1
        l = 0.0005
        z = 1

        l_array = -np.array([[l, 0], [0, l]])
        Le = np.array([[-f/z, 0], [0, -f/z], [-f/z, 0], [0, -f/z], [-f/z, 0], [0, -f/z], [-f/z, 0], [0, -f/z]]) #image jacobian
        c = np.matmul(l_array, np.linalg.pinv(Le))
        vc = np.matmul(c, e)                                                                                    #camera velocity

        print("camera velocity", vc)

        l1 = 0.5    #length of link 1
        l2 = 0.5    #length of link 2

        q1 = self.theta1
        q2 = self.theta2
        jacobian_matrix = np.array([[-l1*math.sin(q1)-l2*math.sin(q1+q2), -l2*math.sin(q1+q2)], 
                                    [l1*math.cos(q1)+l2*math.cos(q1+q2), l2*math.cos(q1+q2)]])
        inverse_jacobian_matrix = np.linalg.pinv(jacobian_matrix)
        print ("Jacobian inverse is", inverse_jacobian_matrix)
        print("\n")

        joint_angle_velocity = np.matmul(inverse_jacobian_matrix, vc)
        print("Joint angle velocity is", joint_angle_velocity)
        print("\n")

        q1_vel = joint_angle_velocity[0]
        q2_vel = joint_angle_velocity[1]

        self.pub_q1_vel.publish(q1_vel)
        self.pub_q2_vel.publish(q2_vel)

if __name__ == '__main__':	
    rospy.init_node('joint_manip_talker', anonymous=True)
    try:
        ic = image_converter()
        rospy.wait_for_service('/vbmbot/controller_manager/switch_controller')
        try:
            sc_service = rospy.ServiceProxy('/vbmbot/controller_manager/switch_controller', SwitchController) #This script is written so that Gazebo doesn't need to restarted everytime while running this node
            stop_controllers = ['joint1_velocity_controller','joint2_velocity_controller']
            start_controllers = ['joint1_position_controller','joint2_position_controller']
            strictness = 2
            start_asap = False
            timeout = 0.0
            res = sc_service(start_controllers,stop_controllers, strictness, start_asap,timeout)
        except rospy.ServiceException as e:
            print("Service Call Failed")
        ic.talker()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)