#!/usr/bin/env python3

from __future__ import print_function
from os import wait

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
from math import pi, tau, dist, fabs, cos, sin
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, Bool
import math
import numpy


transformer = tf.TransformerROS(True, rospy.Duration(10))

jointNameToIndices = {'map': 0, 
                      'world': 1, 
                      'panda_link0': 2, 
                      'panda_link1': 3, 
                      'panda_link2': 4, 
                      'panda_link3': 5, 
                      'panda_link4': 6, 
                      'panda_link5': 7, 
                      'panda_link6': 8, 
                      'panda_link7': 9,
                      'panda_hand' : 10,
                      'panda_rightfinger' : 11,
                      'panda_leftfinger' : 12} 
jointPositions = []
transforms = {}

def getTransformation(a, d, theta, alpha):
    A = [[0 for i in range(4)] for j in range(4)]
    A[0][0] = cos(theta)
    A[0][1] = -sin(theta)*cos(alpha)
    A[0][2] = sin(theta)*sin(alpha)
    A[0][3] = a*cos(theta)
    A[1][0] = sin(theta)
    A[1][1] = cos(theta)*cos(alpha)
    A[1][2] = -cos(theta)*sin(alpha)
    A[1][3] = a*sin(theta)
    A[2][0] = 0
    A[2][1] = sin(alpha)
    A[2][2] = cos(alpha)
    A[2][3] = d
    A[3][0] = 0
    A[3][1] = 0
    A[3][2] = 0
    A[3][3] = 1


'''
header: 
  seq: 0
  stamp: 
    secs: 1911
    nsecs: 832000000
  frame_id: "world"
child_frame_id: "panda_link0"
transform: 
  translation: 
    x: 0.0
    y: 0.0
    z: 0.0
  rotation: 
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
'''
statics = []
def transformsStaticListener(data):
    global statics
    msg = data.transforms
    for i in msg:
        transformer.setTransform(i)
    statics = msg
    # print(msg)
    # while(True):
    #     for i in msg:
    #         i.header.stamp = rospy.Time.now()
    #         transformer.setTransform(i)
    #     # print(msg)
    #     rospy.sleep(0.08)

def transformsListener(data):
    
    msg = data.transforms
    for i in msg:
        transformer.setTransform(i)
    for j in statics:
        # print(j)
        j.header.stamp = rospy.Time.now()
        # print("=================")
        # print(j)
        transformer.setTransform(j)
        # key = (jointNameToIndices[i.header.frame_id], jointNameToIndices[i.child_frame_id])
        # transforms[key] = i.transform
    # print("========================================")
    # print(transforms)

    # print(transformer.allFramesAsDot())
    # print(transformer.canTransform("panda_camera_optical_link","world", rospy.Time.now()))
    # transformer.waitForTransform("panda_link7","world", rospy.Time(0), rospy.Duration(10))   
    # while(not transformer.canTransform("panda_link7","world", rospy.Time(0))):
    #     pass
    # if(transformer.canTransform("panda_camera_optical_link","world", rospy.Time.now())):
    #     print("=====================================================")
        # header = Header()
        # header.frame_id = "panda_link7"
        # header.stamp = rospy.Time.now()
        # mat = transformer.asMatrix("world",header)
        # print(mat)


        # pose_target.pose.position.x = 0.0259903
        # pose_target.pose.position.y = -0.0419027
        # pose_target.pose.position.z = 0.450229
        # quaternion = tf.transformations.quaternion_from_euler(1.825868, -0.772411, 2.923791)
        # pose_target.pose.orientation.x = quaternion[0]
        # pose_target.pose.orientation.y = quaternion[1]
        # pose_target.pose.orientation.z = quaternion[2]
        # pose_target.pose.orientation.w = quaternion[3]
        # pose_target.header.frame_id = "panda_camera_optical_link"
        
        # p = transformer.transformPose("world",pose_target)
        # print(p)
        # move_group.set_pose_target(pose_target)
        # move_group.go(wait=True)
        # rospy.sleep(10)
        # while(1):
        #     pass)

# def normalize(v):
#     norm = numpy.linalg.norm(v)
#     if norm == 0: 
#        return v
#     return v / norm

# def poseListener(data):
#     pose_rcvd = data.pose
#     pose_target = geometry_msgs.msg.PoseStamped()
#     pose_target.header.stamp = rospy.Time.now()

#     while(not transformer.canTransform("panda_camera_optical_link","world", rospy.Time.now())):
#         continue
#     pose_target.pose.position.x = pose_rcvd.position.x
#     pose_target.pose.position.y = pose_rcvd.position.y
#     pose_target.pose.position.z = pose_rcvd.position.z

#     a = pose_rcvd.orientation.x
#     b = pose_rcvd.orientation.y
#     c = pose_rcvd.orientation.z

#     axisY = numpy.array([a, b, c])
#     axisX = numpy.array([0, 0, 1])
#     axisZ = numpy.cross(axisX,axisY)

#     axisX = normalize(axisX)
#     axisY = normalize(axisY)
#     axisZ = normalize(axisZ)

#     rot = numpy.array([axisX, axisY, axisZ])
#     print(rot)
#     quat = tf.transformations.quaternion_from_matrix(rot)
#     pose_target.pose.orientation = quat
#     print(pose_target)
#     # quaternion = tf.transformations.quaternion_from_euler(1.825868, -0.772411, 2.923791)
#     # pose_target.pose.orientation.x = quaternion[0]
#     # pose_target.pose.orientation.y = quaternion[1]
#     # pose_target.pose.orientation.z = quaternion[2]
#     # pose_target.pose.orientation.w = quaternion[3]
#     pose_target.header.frame_id = "panda_camera_optical_link"
    
#     p = transformer.transformPose("world",pose_target)
#     print(p)

# def jointsListener(data):
#     jointPositions = data.position
#     # print(jointPositions)
#     rospy.sleep(0.01)

rospy.init_node("move_franka", anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
pub = rospy.Publisher("/py_moveit_node/capture_pcl", Bool, queue_size=1)
# rospy.Subscriber("/tf", TFMessage, transformsListener)
# rospy.Subscriber("/tf_static", TFMessage, transformsStaticListener)
# rospy.Subscriber("/pcl_node/pose", PoseStamped, poseListener)
# rospy.Subscriber("/panda/joint_states", JointState, jointsListener)
# rospy.spin()


if __name__ == '__main__':

    pose_target = geometry_msgs.msg.Pose()

    # main
    # quaternion = tf.transformations.quaternion_from_euler(1.977813, -0.791686, 2.910482)
    # quaternion = tf.transformations.quaternion_from_euler(-3.1412, 0.0735, 0.002658)


    quaternion = tf.transformations.quaternion_from_euler(1.979445, -0.791860, 2.909927 - (15*pi/180))
    pose_target.position.x = -0.075468
    pose_target.position.y = 0.145216
    pose_target.position.z = 0.765713
    pose_target.orientation.x = quaternion[0]
    pose_target.orientation.y = quaternion[1]
    pose_target.orientation.z = quaternion[2]
    pose_target.orientation.w = quaternion[3]
    move_group.set_pose_target(pose_target)
    move_group.go(wait=True)

    # rospy.sleep(5)
    pub.publish(True)
    rospy.sleep(1)
    pub.publish(False)


    quaternion = tf.transformations.quaternion_from_euler(1.979445, -0.791860, 2.909927 + (15*pi/180))
    pose_target.position.x = 0.175468
    pose_target.position.y = 0.145216
    pose_target.position.z = 0.765713

    

    # Main
    # pose_target.position.x = 0.011974
    # pose_target.position.y = 0.134315
    # pose_target.position.z = 0.766702

    # pose_target.position.x = 0.0355959
    # pose_target.position.y = -0.0628825
    # pose_target.position.z = -0.423587

    # pose_target.position.x = 0.115024
    # pose_target.position.y = 0.000172
    # pose_target.position.z = 1.031304
    pose_target.orientation.x = quaternion[0]
    pose_target.orientation.y = quaternion[1]
    pose_target.orientation.z = quaternion[2]
    pose_target.orientation.w = quaternion[3]
    move_group.set_pose_target(pose_target)
    move_group.go(wait=True)
    # rospy.sleep(5)
    pub.publish(True)
    rospy.sleep(1)
    pub.publish(False)
    # move_group.stop()
    # move_group.clear_pose_targets()

    # leftfinger = rospy.Publisher('/panda/panda_finger1_controller/command',Float64,queue_size=1)
    # rightfinger = rospy.Publisher('/panda/panda_finger2_controller/command',Float64,queue_size=1)
    # obj = Float64()
    # obj.data = 0.04
    # leftfinger.publish(obj)
    # rightfinger.publish(obj)
    # rospy.spin()
    # print("checkflag")

    move_group.stop()
    move_group.clear_pose_targets()

    rospy.sleep(5)
    moveit_commander.roscpp_shutdown()


