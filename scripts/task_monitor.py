#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
import time
from std_srvs.srv import Empty

class management():

    def __init__(self, manipulator_pos_cmd_topic, goal_topic, gripper_on_service, gripper_off_service):

        self.use_aruco = True

        # Subscriber to the success flag to know if the robot has reached the goal

        self.controller_check = rospy.Subscriber("/success_control", Bool, self.controller_flag)

        # Publisher to send sequences of position commands

        self.seq_pub = rospy.Publisher(manipulator_pos_cmd_topic, Float64MultiArray, queue_size=1)

        # Publish arm joint velocities

        self.goto_pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=1) 

        # vacuum control services 

        self.gripper_on_service = gripper_on_service

        self.gripper_off_service = gripper_off_service


        self.counter = 0

        self.goal_position = None

        self.flag_1 = False # Flag to check if the controller has finished
        self.flag_2 = False # Flag to check if the first pick and place sequence has finished
        self.flag_3 = False # Flag to check if the robot has reached the delivery position with the end effector
        self.flag_4 = False # Flag to check if the second pick and place sequence has finished at the delivery position


    def deliver_pkg(self):
        # Send a new goal for the robot to reach and deliver the Aruco object
        goto_psoition = PoseStamped()
        goto_psoition.pose.position.x = 0
        goto_psoition.pose.position.y = 0
        goto_psoition.pose.position.z = 0.2
        goto_psoition.pose.orientation.x = 0
        goto_psoition.pose.orientation.y = 0
        goto_psoition.pose.orientation.z = 0
        goto_psoition.pose.orientation.w = 1
        self.goto_pub.publish(goto_psoition)

    def hold(self):
        # Function to hold an object with the vacuum gripper
        rospy.wait_for_service(self.gripper_on_service )
        gripper_hold = rospy.ServiceProxy(self.gripper_on_service , Empty)
        gripper_hold()

    def release(self):
        # Function to release an object from the vacuum gripper
        rospy.wait_for_service(self.gripper_off_service)
        gripper_release = rospy.ServiceProxy(self.gripper_off_service, Empty)
        gripper_release()

    def pickup_arm(self):

        sequence = Float64MultiArray()
        
        self.hold()

        # joints= [-1.401, 1.171, 1.127, 0.0]   # on the cube 
        # sequence.data = joints
        # self.seq_pub.publish(sequence)

        joints= [-1.491, -0.136, -0.082, 0.0]   # saftey above the cube
        sequence.data = joints
        self.seq_pub.publish(sequence)
        time.sleep(3)

        joints= [1.543, -0.658, 0.0, 0.0]    # saftey above the box
        sequence.data = joints
        self.seq_pub.publish(sequence)
        time.sleep(3)
        self.release() # Release the aruco cube

        joints= [-1.491, -0.136, -0.082, 0.0]   # saftey above the cube
        sequence.data = joints
        self.seq_pub.publish(sequence)
        time.sleep(3)

        print(" pickup 1 ends  ")


        self.flag_2 = True

    def delivery_arm(self):

        sequence = Float64MultiArray()

        joints= [-1.57, -0.501, -0.223, 0.0]   # saftey above the cube
        sequence.data = joints
        self.seq_pub.publish(sequence)
        time.sleep(3)


        joints= [1.57, -0.501, -0.223, 0.0]   # saftey above the box
        sequence.data = joints
        self.seq_pub.publish(sequence)
        time.sleep(3)

        joints= [1.59, -0.6, 0.1, 0.0]    # on the box
        sequence.data = joints
        self.seq_pub.publish(sequence)
        time.sleep(3)
        self.hold() # Hold the aruco cube


        joints= [1.543, -0.658, 0.0, 0.0]    # saftey above the box
        sequence.data = joints
        self.seq_pub.publish(sequence)
        time.sleep(3)

        
        joints= [-1.491, -0.136, -0.082, 0.0]   # saftey above the cube
        sequence.data = joints
        self.seq_pub.publish(sequence)
        time.sleep(3)


        joints = [-1.45, 1.137, 1.3, 0.0]   # on the cube 
        sequence.data = joints
        self.seq_pub.publish(sequence)
        time.sleep(3)
        self.release() # Release the aruco cube


        joints= [-1.491, -0.136, -0.082, 0.0]   # saftey above the cube
        sequence.data = joints
        self.seq_pub.publish(sequence)
        time.sleep(3)

        print(" Delivery ends  ")

        self.flag_4 = True

    def controller_flag(self, confirmation):
        if self.counter == 0:
            self.flag_1 =  confirmation.data
            print("the controller has finished", self.flag_1)
            self.counter += 1
        elif self.counter == 1:
            self.flag_3 =  confirmation.data
            print("the controller has finished for second time", self.flag_3)
            self.counter += 1

    def sequencial_execution(self):

        #Wait fot he controller to finish its execution #########
        while(not self.flag_1):
            pass
        print("1. controller done")
        ###########################################################
    
        self.pickup_arm()

        # Wait for arm to execute a sequence of actions to grab an object
        while(not self.flag_2):
            pass

        print("2. Pick-up onedone")
        # ###########################################################

        self.deliver_pkg()

        # # Wait for the robot to reach a location to drop an object
        while(not self.flag_3):
            pass

        print("3. Reaching delivering position")
        # ###########################################################

        self.delivery_arm()

        # # Wait for arm to execute a sequence of actions to realse an object on the ground
        while(not self.flag_4):
            pass

        print("4. Delivery Done")
        # ###########################################################


if __name__=="__main__":

    print("TASK MONITOR NODE IGNITED")

    rospy.init_node('task_monitor', anonymous=True)

    # required topics and services
    manipulator_pos_cmd_topic = rospy.get_param("/tbot_manipulator_pos_cmd_topic")
    goal_topic = rospy.get_param("/std_goal_topic")
    gripper_on_service  = rospy.get_param("/vacuum_on_service")
    gripper_off_service = rospy.get_param("/vacuum_off_service")

    node = management(manipulator_pos_cmd_topic, goal_topic, gripper_on_service, gripper_off_service)

    node.sequencial_execution()


    rospy.spin()