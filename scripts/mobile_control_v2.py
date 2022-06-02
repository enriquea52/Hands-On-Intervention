#!/usr/bin/env python3


from numpy import float64
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from nav_msgs.msg import Odometry
import tf

from mobile_tasks import *

class TP_Control():

    def __init__(self, manipulator_vel_cmd_topic, 
                       base_vel_cmd_topic, 
                       success_flag_topic, 
                       joint_states_topic, 
                       odometry_topic,
                       aruco_topic,
                       goal_topic):

        # Global Variables
        
        self.odometry = np.zeros((3))
        
        self.sigma_d = None 
        self.weights = np.array([[0.3, 0.7, 0.05, 1, 1, 1]])


        self.j_vel_limits= np.array([[0.2],[0.2],[0.2],[0.2],[0.2],[0.2]]) #joint velocity limits

        self.tasks = [
                JointPosition("Joint-Position", -np.pi/2, joint_index = 3, gain=5),
                JointLimits("Joints limit task", np.array([0, np.pi/2]), np.array([0.02, 0.05]), joint_index = 4),
                JointLimits("Joints limit task", np.array([1.5, 0.1]), np.array([0.02, 0.05]), joint_index = 5),
                # JointLimits("Joints limit task", np.array([0, -np.pi/2]), np.array([0.02, 0.05]), joint_index=3),
                # BaseOrientation("Base orientation Task", 0),
                Position3D("End-effector position", self.sigma_d, gain=2.3)

                ] 

        self.error_array = np.array([])
        

        self.stop_flag = False

        # Publishers

        self.pub_joints = rospy.Publisher(manipulator_vel_cmd_topic, Float64MultiArray, queue_size=1) # Publish arm joint velocities

        self.pub_base = rospy.Publisher(base_vel_cmd_topic, Twist, queue_size=1)                      # publish twist velocities

        self.succ_pub = rospy.Publisher(success_flag_topic, Bool, queue_size=1)                       # publish succes oon picking up the aruco object

        self.error_norm_pub = rospy.Publisher("/error_norms", Float64MultiArray, queue_size=1) # Publish arm joint velocities


        # Subcribers

        self.control_sub = rospy.Subscriber(joint_states_topic, JointState, self.control)              # subscribe to the joint state

        self.odom_sub = rospy.Subscriber(odometry_topic, Odometry, self.get_odom)                      # subscribe to odom

        self.aruco_goal_sub = rospy.Subscriber(aruco_topic, PoseStamped, self.aruco_get_goal_psoition) # subscribe to the goal position from the aruco node to pick up the aruco cube

        self.goal_sub = rospy.Subscriber(goal_topic, PoseStamped, self.get_goal_psoition)              # subscribe to the goal position


    def control(self, joint_states):


        if (len(joint_states.name) == 4 and self.sigma_d is not None and not self.stop_flag):

            q  =  np.asarray(joint_states.position)


            dq =  np.zeros((6,1))
            P  =  np.eye(6)

            for task in self.tasks:                                                        # Loop over tasks
                task.update(self.odometry, q)                                              # Update task state
                if task.isActive() != 0:                                                   # Check if task is active
                    Ji = task.getJacobian()                                                # Retrieved the current Jacobian
                    x_dot = task.getfeedforward() + task.getMatrixK()@task.getError()  # Compute the error taking into account the feedforward action and the K gain matrix
                    Ji_bar = Ji @ P   
                    Ji_bar_DLS = W_DLS(Ji_bar,0.08, self.weights); Ji_bar_pinv = np.linalg.pinv(Ji_bar)    # Invert the augmented Jacobian using DLS and pseudoinverse# Compute augmented Jacobian
                    #Ji_bar_DLS = DLS(Ji_bar,0.05); Ji_bar_pinv = np.linalg.pinv(Ji_bar)    # Invert the augmented Jacobian using DLS and pseudoinverse
                    dq = dq + Ji_bar_DLS @ (task.active * x_dot - Ji @ dq)                 # Compute task velocity and Accumulate velocity
                    P = P - Ji_bar_pinv @ Ji_bar
                
                self.error_array  = np.append(self.error_array, [np.linalg.norm(task.getError())])


            # Publishing error of every task

            error_norm_array = Float64MultiArray()
            error_norm_array.data = self.error_array
            self.error_norm_pub.publish(error_norm_array)
            self.error_array = np.asarray([])


            # Limit velocities of every DoF

            s=abs(dq)/self.j_vel_limits

            S=np.max(s)
            if S>1:
                dq = dq/S

            # Publish joint Velocities

            j_vel = Float64MultiArray()

            j_vel.data = dq[2:6, 0].reshape(4).astype(float)

            self.pub_joints.publish(j_vel)

            # Publish joint Velocities

            mobile_vel = Twist()

            mobile_vel.linear.x = dq[1, 0]

            mobile_vel.angular.z = dq[0, 0]

            self.pub_base.publish(mobile_vel)

            # Stop the control system by setting sigma_d to None

            print("error:", np.linalg.norm(self.tasks[-1].getError()))
            
            #if np.linalg.norm(self.tasks[-1].getError()) < 0.035:     
            if np.abs(self.tasks[-1].getError()[2,0]) < 0.02:     

                print("controller deactivated")

                success = Bool()

                success.data = True

                self.succ_pub.publish(success.data) # Publish success data to notify the task monitor node about having finished the current task
                
                self.sigma_d = None


                # Publish joint Velocities

                j_vel = Float64MultiArray()

                j_vel.data = np.asarray([0., 0., 0., 0.], dtype=float64)

                self.pub_joints.publish(j_vel)      # Publish zero velocities on each joint of the robotic arm

                # Publish joint Velocities

                mobile_vel = Twist()

                mobile_vel.linear.x = 0.0

                mobile_vel.angular.z = 0.0

                self.pub_base.publish(mobile_vel)   # Publish zeros velocities to stop the robotic base

                self.aruco_goal_sub.unregister()    # unsubscribe to the \aruco_goto_position topic

                self.stop_flag = True               # Stop listening to set goal positions



    def get_odom(self,odom_msg):

        # Subroutine to retrieve odometry information and know the current state of the mobile base

        self.odometry[0] = odom_msg.pose.pose.position.x

        self.odometry[1] = odom_msg.pose.pose.position.y

        _, _, yaw = tf.transformations.euler_from_quaternion([odom_msg.pose.pose.orientation.x, 
                                                              odom_msg.pose.pose.orientation.y,
                                                              odom_msg.pose.pose.orientation.z,
                                                              odom_msg.pose.pose.orientation.w])
        self.odometry[2] = yaw

    def aruco_get_goal_psoition(self, position_msg):

        # retrieve a goal from the aruco_detector node based on the position of an Aruco

        x = position_msg.pose.position.x; y = position_msg.pose.position.y; z = position_msg.pose.position.z

        self.sigma_d = np.asarray([[x],[y],[0.08]])
        
        self.tasks[-1].setDesired(self.sigma_d)


    def get_goal_psoition(self, position_msg):

        # retrieve a goal from the task monitor node based on the position of an Aruco

        x = position_msg.pose.position.x; y = position_msg.pose.position.y; z = position_msg.pose.position.z

        self.sigma_d = np.asarray([[x],[y],[0.14]])
        
        self.tasks[-1].setDesired(self.sigma_d)

        self.stop_flag = False


if __name__ == "__main__":

    print("CONTROL NODE FOR THE MOBILE MANIPULATOR SYSTEM IGNITED")

    rospy.init_node('controller', anonymous=True)

    # Required Topics

    manipulator_vel_cmd_topic = rospy.get_param("/tbot_manipulator_vel_cmd_topic")
    base_vel_cmd_topic = rospy.get_param("/tbot_base_vel_cmd_topic")
    success_flag_topic = rospy.get_param("/tbot_success_flag_topic")
    joint_states_topic = rospy.get_param("/tbot_joint_states_topic")
    odometry_topic = rospy.get_param("/tbot_odometry_topic")
    aruco_topic = rospy.get_param("/aruco_goal_topic")
    goal_topic = rospy.get_param("/std_goal_topic")

    TPC_node = TP_Control(manipulator_vel_cmd_topic, 
                          base_vel_cmd_topic, 
                          success_flag_topic, 
                          joint_states_topic, 
                          odometry_topic, 
                          aruco_topic, 
                          goal_topic)

    rospy.spin()

    # rosbag recording terminal command
    # rosbag record -O subset /error_norms /turtlebot/manipulator/commands/velocity /turtlebot/odom