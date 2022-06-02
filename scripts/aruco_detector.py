#!/usr/bin/env python3
import rospy
import tf
from fiducial_msgs.msg import FiducialTransformArray
import numpy as np
from geometry_msgs.msg import PoseStamped


class aruco_pose_detection(object):

    def __init__(self, aruco_detect_topic, aruco_goto_topic):

        # Subscribe to the Aruco detect node to receiv e the transofrmations of the detected Arucos

        self.pose_sub = rospy.Subscriber(aruco_detect_topic, FiducialTransformArray, self.get_pose, queue_size = 1) 

        # Define tf listener

        self.tf_listener = tf.TransformListener()

        # Define the transforomation fromt he camera to the Aruco read from the /fiducial_transforms topic

        self.Aruco_T = np.zeros((4,4))

        # Define the publisher to send the goal position where the aruco is located

        self.aruco_position_pub = rospy.Publisher(aruco_goto_topic, PoseStamped, queue_size=1) # Publish arm joint velocities


    def get_pose(self, msg):
        
        for transform in msg.transforms:
            if transform.fiducial_id == 30:

                print("Aruco detected ...")

                aruco_rot = tf.transformations.quaternion_matrix([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])  # Get transformation matrix defining the orientation of the realsense_link wrt the odom frame

                self.Aruco_T = aruco_rot

                self.Aruco_T[0,3] = transform.transform.translation.x; self.Aruco_T[1,3] = transform.transform.translation.y; self.Aruco_T[2,3] = transform.transform.translation.z                 # Store the translation vector of the detected Aruco

                (trans,rot) =  self.tf_listener.lookupTransform('odom', 'realsense_link', rospy.Time(0))
                
                T = tf.transformations.quaternion_matrix(rot)                                                                                                                                       # Get transformation matrix defining the orientation of the realsense_link wrt the odom frame
                
                T[0,3] = trans[0]; T[1,3] = trans[1]; T[2,3] = trans[2]                                                                                                                             # Add the translation between the realsense camera and the odom frame

                wTa = T@self.Aruco_T                                                                                                                                                                # Transform the position of the Aruco to the global frame

                aruco_position = PoseStamped()
                aruco_position.pose.position.x = wTa[0,3]
                aruco_position.pose.position.y = wTa[1,3] 
                aruco_position.pose.position.z = wTa[2,3] 
                aruco_position.pose.orientation.x = 0
                aruco_position.pose.orientation.y = 0
                aruco_position.pose.orientation.z = 0
                aruco_position.pose.orientation.w = 1
                self.aruco_position_pub.publish(aruco_position)                                                                                                                                     # Publish the go-to position for the robot to pick an Object with an Aruco marker

                break




if __name__ == "__main__":

    print("ARUCO POSE DETECTOR NODE IGNITED")

    rospy.init_node('aruco_pose_detector_node')   

    aruco_goto_topic = rospy.get_param("/aruco_goal_topic")

    aruco_detect_topic = rospy.get_param("/aruco_detect_pkg_topic")

    node = aruco_pose_detection(aruco_detect_topic, aruco_goto_topic)
    
    rospy.spin()