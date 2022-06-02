import numpy as np


'''
    Kinematics of the robotic arm structure
'''
def arm_kinematics(theta1, theta2, theta3, theta4):
    return np.array([[np.cos(theta1)*np.cos(theta4) - np.sin(theta1)*np.sin(theta4), -np.cos(theta1)*np.sin(theta4) - np.sin(theta1)*np.cos(theta4), 0, np.cos(theta1)*(0.0697 + 0.142*np.sin(theta2) + 0.1588*np.cos(theta3))],
                    [np.sin(theta1)*np.cos(theta4) - np.cos(theta1)*np.sin(theta4), np.cos(theta1)*np.cos(theta4) - np.sin(theta1)*np.sin(theta4), 0, np.sin(theta1)*(0.0697 + 0.142*np.sin(theta2) + 0.1588*np.cos(theta3))],
                    [0, 0, 1, 0.0358 + 0.142*np.cos(theta2) - 0.1588*np.sin(theta3)],
                    [0, 0, 0, 1]])

'''
    kinematics of the full mobile-manipulator structure
'''
def mobile_manipulator_kinematics(x, y, psi, theta1, theta2, theta3, theta4):
    return np.array([[ np.cos(theta1 + theta4 + psi), -np.sin(theta1 + theta4 + psi), 0, x - 0.0697*np.sin(psi + theta1)-0.071*np.cos(-theta2 + psi + theta1)+0.071*np.cos(theta2 + psi + theta1)-0.0794*np.sin(theta3 + psi + theta1)-0.0794*np.sin(-theta3 + psi + theta1) + 0.037*np.cos(psi)],
                     [ np.sin(theta1 + theta4 + psi),  np.cos(theta1 + theta4 + psi), 0, y + 0.0697*np.cos(psi + theta1)+0.071*np.sin(theta2 + psi + theta1)-0.071*np.sin(-theta2 + psi + theta1)+0.0794*np.cos(-theta3 + psi + theta1)+0.0794*np.cos(theta3 + psi + theta1) + 0.037*np.sin(psi)],
                     [0, 0, 1, 0.1828 + 0.142*np.cos(theta2) - 0.1588*np.sin(theta3)],
                     [0, 0, 0, 1]])

    # Kinematics assuming EE orientation is aligned with the world's y axis
    # return np.array([[-np.sin(theta1 + theta4 + psi), -np.cos(theta1 + theta4 + psi), 0, x - 0.0697*np.sin(psi + theta1)-0.071*np.cos(-theta2 + psi + theta1)+0.071*np.cos(theta2 + psi + theta1)-0.0794*np.sin(theta3 + psi + theta1)-0.0794*np.sin(-theta3 + psi + theta1) + 0.037*np.cos(psi)],
    #                  [ np.cos(theta1 + theta4 + psi), -np.sin(theta1 + theta4 + psi), 0, y + 0.0697*np.cos(psi + theta1)+0.071*np.sin(theta2 + psi + theta1)-0.071*np.sin(-theta2 + psi + theta1)+0.0794*np.cos(-theta3 + psi + theta1)+0.0794*np.cos(theta3 + psi + theta1) + 0.037*np.sin(psi)],
    #                  [0, 0, 1, 0.1828 + 0.142*np.cos(theta2) - 0.1588*np.sin(theta3)],
    #                  [0, 0, 0, 1]])


'''
    Jacobian of the full mobile-manipulator structure
'''
def jacobian(psi, D, theta1, theta2, theta3, theta4):

    m11 = -np.sin(psi + np.pi/2 + theta1)*D;    m12 = np.cos(psi)

    m21 = np.cos(psi + np.pi/2 + theta1)*D;     m22 = np.sin(psi)

    m31 = 0;                                    m32 = 0

    m41 = 0;                                    m42 = 0

    m51 = 0;                                    m52 = 0

    m61 = 1;                                    m62 = 0

    j11 = -np.sin(theta1)*(0.0697+ 0.142*np.sin(theta2) + 0.1588*np.cos(theta3)); j12 = 0.142*np.cos(theta1)*np.cos(theta2); j13 = -0.1588*np.cos(theta1)*np.sin(theta3); j14 = 0.

    j21 = np.cos(theta1)*(0.0697+ 0.142*np.sin(theta2) + 0.1588*np.cos(theta3)); j22 = 0.142*np.sin(theta1)*np.cos(theta2); j23 = -0.1588*np.sin(theta1)*np.sin(theta3); j24 = 0.

    j31 = 0; j32 = -0.142*np.sin(theta1); j33 = -0.1588*np.cos(theta3); j34 = 0.

    j41 = 0.; j42 = 0.; j43 = 0.; j44 = 0.

    j51 = 0.; j52 = 0.; j53 = 0.; j54 = 0.

    j61 = 1.; j62 = 0.; j63 = 0.; j64 = 1.

    return np.array([[m11, m12, j11, j12, j13, j14],
                     [m21, m22, j21, j22, j23, j24],
                     [m31, m32, j31, j32, j33, j34],
                     [m41, m42, j41, j42, j43, j44],
                     [m51, m52, j51, j52, j53, j54],
                     [m61, m62, j61, j62, j63, j64]])
'''
    Damped Lest Squares Implementation
'''
def DLS(A, damping):
    return np.transpose(A)@np.linalg.inv(A@np.transpose(A) + (damping**2)*np.eye(len(A))) # Implement the formula to compute the DLS of matrix A.


'''
    Weighted Damped Lest Squares Implementation
'''
def W_DLS(A, damping, weights):

    W = np.eye(weights.shape[1])*weights.T

    return np.linalg.inv(W)@np.transpose(A)@np.linalg.inv(A@np.transpose(A) + (damping**2)*np.eye(len(A))) # Implement the formula to compute the DLS of matrix A.

'''
    Base class representing an abstract Task.
'''
class Task:
    '''
        Constructor.

        Arguments:
        name (string): title of the task
        desired (Numpy array): desired sigma (goal)
    '''
    def __init__(self, name, desired):
        self.name = name           # Task title
        self.sigma_d = desired     # Desired sigma
        self.ff = None             # Feedforward velocity vector.
        self.K = None              # Gain matrix K
        self.error_over_time = []  # Error over time
        self.K_gain = 1            # K gain matrix diagonal gains
    '''
        Method updating the task variables (abstract).

        Arguments:
        robot (object of class Manipulator): reference to the manipulator
    '''
    def update(self, robot):
        pass
    ''' 
        Method setting the desired sigma.

        Arguments:
        value(Numpy array): value of the desired sigma (goal)
    '''
    def setDesired(self, value):
        self.sigma_d = value
    '''
        Method returning the desired sigma.
    '''
    def getDesired(self):
        return self.sigma_d
    '''
        Method returning the task Jacobian.
    '''
    def getJacobian(self):
        return self.J
    ''' 
        Method setting the feedforward velocity vector.
    '''
    def setfeedforward(self, value):
        self.ff = value
    ''' 
        Method returning the feedforward velocity vector.
    '''
    def getfeedforward(self):
        return self.ff
    ''' 
        Method setting the Gain Matrix K.
    '''
    def setMatrixK(self, matrix):
        self.K =  matrix
    ''' 
        Method returning the Gain Matrix K.
    '''
    def getMatrixK(self):
        return self.K*self.K_gain
    '''
        Method returning the task error (tilde sigma).
    '''    
    def getError(self):
        return self.err
    '''
        Method returning the jacobian.
    '''   
    def getEEJacobian(self, odometry, q):

        D = np.linalg.norm(arm_kinematics(q[0], q[1], q[2], q[3])[0:2, 3])
        
        return jacobian(odometry[2], D, q[0], q[1], q[2], q[3])
    '''
        Method returning the ff kinematics of the robot.
    '''   
    def getEEKinematics(self,odometry, q):


        return mobile_manipulator_kinematics(odometry[0], odometry[1], odometry[2], q[0], q[1], q[2], q[3])

    '''
        sgn method used for the calculation of quaternions (from a given rotation matrix)
    '''    
    def sgn(self, x):
        if x >= 0:
            return 1
        else:
            return -1
    def isActive(self):
        pass

    '''
        quaternion from given euler angles
    '''   
    def get_quaternion_from_euler(self,roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return np.array([[qx], [qy], [qz]]), qw
    '''
        quaternion from a given rotation matrix
    '''   
    def get_quaternion_from_rot_mat(self,rot):
        r11 = rot[0,0]; r12 = rot[0,1]; r13 = rot[0,2]  
        r21 = rot[1,0]; r22 = rot[1,1]; r23 = rot[1,2]
        r31 = rot[2,0]; r32 = rot[2,1]; r33 = rot[2,2]
        epsilon1 = (0.5) * self.sgn(r32 - r23) * np.sqrt(round(r11-r22-r33+1,5))
        epsilon2 = (0.5) * self.sgn(r13 - r31) * np.sqrt(round(r22-r33-r11+1,5))
        epsilon3 = (0.5) * self.sgn(r21 - r12) * np.sqrt(round(r33-r11-r22+1,5))
        w = (0.5) * np.sqrt(r11+r22+r33+1)
        return np.array([[epsilon1], [epsilon2], [epsilon3]]), w
'''
    Subclass of Task, representing the 2D position task.
'''
class Position3D(Task):
    def __init__(self, name, desired, gain = 1):
        super().__init__(name, desired)
        self.err = np.zeros((3,1))                          # Initialize error vector with proper dimensions
        self.setfeedforward(np.zeros(self.err.shape))       # Initialize feedforward vector with proper dimensions
        self.K_gain = gain
        self.setMatrixK(np.eye(len(self.err)))             # Initialize gains matrix K with proper dimensions
        self.active = 1
    def update(self, odometry, q):
        self.J = self.getEEJacobian(odometry,q)[0:3,:]                                             # Update task Jacobian
        self.err = (self.sigma_d  - self.getEEKinematics(odometry,q)[0:3,3].reshape(3,1))          # Update task error

        print("kinematics:\n", self.getEEKinematics(odometry,q)[0:3,3].reshape(3,1))


    def isActive(self):
        return self.active

'''
    Subclass of Task, representing the 2D orientation task.
'''
class Orientation3D(Task):
    def __init__(self, name, desired):
        super().__init__(name, desired)
        self.err = np.zeros((3,1))                               # Initialize error vector with proper dimensions
        self.setfeedforward(np.zeros(self.err.shape))            # Initialize feedforward vector with proper dimensions
        self.setMatrixK(np.eye(len(self.err)))                   # Initialize gains matrix K with proper dimensions
        self.active = 1
    def update(self, odometry, q):
        self.J = self.getEEJacobian(odometry, q)[3:6,:]          # Update task Jacobian
        # Desired orientation quaternion
        epsilon_d, w_d = self.get_quaternion_from_euler(0, 0, self.sigma_d)
        # Current orientation quaternion
        rot_mat =  self.getEEKinematics(odometry, q)[0:3,0:3]
        epsilon, w = self.get_quaternion_from_rot_mat(rot_mat)
        # Error computation based on quaternions of the current oreintation and the desired orientation
        self.err = self.K @ (w*epsilon_d - w_d*epsilon - np.cross(epsilon, epsilon_d, axis = 0)) # Update task error

    def isActive(self):
        return self.active


'''
    Subclass of Task, representing the 2D configuration task.
'''
class Configuration3D(Task):
    def __init__(self, name, desired):
        super().__init__(name, desired)
        self.err = np.zeros((6,1))                                         # Initialize with proper dimensions
        self.setfeedforward(np.zeros(self.err.shape))                      # Initialize feedforward vector with proper dimensions
        self.setMatrixK(np.eye(len(self.err)))                             # Initialize gains matrix K with proper dimensions
        self.active = 1

    def update(self, odometry, q):
        self.J =  self.getEEJacobian(odometry, q)                                            # Update task Jacobian
        nd = self.sigma_d[0:3].reshape(3,1)                                                  # Desired end-effector position
        n = self.getEEKinematics(odometry, q)[0:3,3].reshape(3,1)                            # Current end-effector position
        position_err = nd - n                                                                # Position error of the end-effector
        # Desired orientation quaternion
        epsilon_d, w_d = self.get_quaternion_from_euler(0, 0, self.sigma_d[3])               # Converting euler angle to quaternion
        # Current orientation quaternion
        rot_mat = self.getEEKinematics(odometry, q)[0:3,0:3]                                 # End effector rotation matrix
        epsilon, w = self.get_quaternion_from_rot_mat(rot_mat)                               # Converting rotation matrix to quaternion
        orientation_err = w*epsilon_d - w_d*epsilon - np.cross(epsilon, epsilon_d, axis = 0) # Update task error (orientation)
        self.err = np.append(position_err,orientation_err, axis = 0)                         # Update task error (position and orientation of the end effector)

    def isActive(self):
        return self.active



''' 
    Subclass of Task, representing the joint position task.
'''
class JointPosition(Task):
    def __init__(self, name, desired, joint_index = 1, gain = 1):
        super().__init__(name, desired)
        self.err = np.zeros((1,1))                        # Initialize with proper dimensions
        self.setfeedforward(np.zeros(self.err.shape))     # Initialize feedforward vector with proper dimensions
        self.K_gain = gain
        self.setMatrixK(np.eye(len(self.err)))            # Initialize gains matrix K with proper dimensions
        self.active = 1
        self.joint_index = joint_index

    def update(self, odometry, q):
        self.J =  np.zeros((1,6))                         # Update task Jacobian
        self.J[0, int(self.joint_index-1)] = 1  
        q = q[int(self.joint_index-3)]                    # Current joint angular position
        qd = self.sigma_d                                 # Desired joint angular position
        self.err = self.K @ (np.asarray([[qd - q]]))      # Joint angular position error

    def isActive(self):
        return self.active

''' 
    Subclass of Task, representing the joint limits task.
'''
class JointLimits(Task):
    def __init__(self, name, desired, switching, joint_index = 1):
        super().__init__(name, desired)
        self.err = np.zeros((1,1))                                # Initialize with proper dimensions
        self.setfeedforward(np.zeros(self.err.shape))             # Initialize feedforward vector with proper dimensions
        self.setMatrixK(np.eye(len(self.err)))                    # Initialize gains matrix K with proper dimensions

        self.active = 0                                           # Activation fucntion
        self.alpha = switching[0]; self.delta = switching[1]
        self.q_max = desired[0]; self.q_min = desired[1]
        self.joint_index = joint_index

    def update(self, odometry, q):
        self.J =  np.zeros((1,6))                                  # Update task Jacobian
        self.J[0, self.joint_index - 1] = 1  
        
        self.q = q[int(self.joint_index-3)]                        # Current joint angular position
        self.err = np.array([[1]])                                 # Joint angular position error

        # Activation function update
        if (self.active == 0) and (self.q >= self.q_max - self.alpha):
            self.active = -1
        if (self.active == 0) and (self.q <= self.q_min + self.alpha):
            self.active = 1
        if (self.active == -1) and (self.q <= self.q_max - self.delta):
            self.active = 0
        if (self.active == 1) and (self.q >= self.q_min + self.delta):
            self.active = 0

    def isActive(self):                                           # Activation Function
        return self.active


''' 
    Subclass of Task, representing the base orientation task.
'''
class BaseOrientation(Task):
    def __init__(self, name, desired):
        super().__init__(name, desired)
        self.err = np.zeros((1,1))                        # Initialize with proper dimensions
        self.setfeedforward(np.zeros(self.err.shape))     # Initialize feedforward vector with proper dimensions
        self.setMatrixK(np.eye(len(self.err)))            # Initialize gains matrix K with proper dimensions
        self.active = 1

    def update(self, odometry, q):
        self.J =  np.zeros((1,6))                         # Update task Jacobian
        self.J[0, 0] = 1  
        q = odometry[2]                                   # Current mobile base orientation position
        qd = self.sigma_d                                 # Desired joint angular position
        self.err = self.K @ (np.asarray([[qd - q]]))      # Joint angular position error

    def isActive(self):
        return self.active