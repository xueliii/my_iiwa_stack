import sys
import rospy
from iiwa_msgs.srv import *
import iiwa_msgs.msg

class SetControlMode():
    def __init__(self):
        rospy.loginfo("====== Import control mode library successfully! =============")
        rospy.wait_for_service("/iiwa/configuration/ConfigureControlMode")
        self.mode = rospy.ServiceProxy("/iiwa/configuration/ConfigureControlMode", ConfigureControlMode)
        

    def callService(self):
        pass

    def initJointImpedance(self, stiffness, damping):
        joint_impedance = iiwa_msgs.msg.JointImpedanceControlMode()
        joint_impedance.joint_stiffness.a1 = stiffness[0]
        joint_impedance.joint_stiffness.a2 = stiffness[1]
        joint_impedance.joint_stiffness.a3 = stiffness[2]
        joint_impedance.joint_stiffness.a4 = stiffness[3]
        joint_impedance.joint_stiffness.a5 = stiffness[4]
        joint_impedance.joint_stiffness.a6 = stiffness[5]
        joint_impedance.joint_stiffness.a7 = stiffness[6]
        return joint_impedance
    
    def setJointImpedanceMode(self, stiffness, damping):
        '''
        stiffness: [0,1,2,3,4,5,6]   >= 0.0
        damping:   [0,1,2,3,4,5,6]   0.0 ... 1.0
        '''
        control_mode = iiwa_msgs.msg.ControlMode.JOINT_IMPEDANCE
        joint_impedance = self.initJointImpedance(stiffness, damping)
        resp = self.mode(control_mode = control_mode, joint_impedance = joint_impedance)

    def initCartesianImpedance(self, cartesian_stiffness, cartesian_damping):
        '''

        '''
        joint_impedance = iiwa_msgs.msg.JointImpedanceControlMode()
        joint_impedance.joint_stiffness.a1 = stiffness[0]
        joint_impedance.joint_stiffness.a2 = stiffness[1]
        joint_impedance.joint_stiffness.a3 = stiffness[2]
        joint_impedance.joint_stiffness.a4 = stiffness[3]
        joint_impedance.joint_stiffness.a5 = stiffness[4]
        joint_impedance.joint_stiffness.a6 = stiffness[5]
        joint_impedance.joint_stiffness.a7 = stiffness[6]
        return cartesian_impedance
    
    def setCartesianImpedanceMode(self, cartesian_stiffness, cartesian_damping,\
                                  nullspace_stiffness = -1, nullspace_damping = 0.7):
        '''
        stiffness: [0,1,2,3,4,5,6]   >= 0.0
        damping:   [0,1,2,3,4,5,6]   0.0 ... 1.0
        '''
        control_mode = iiwa_msgs.msg.ControlMode.CARTESIAN_IMPEDANCE
        cartesian_impedance = self.initCartesianImpedance(cartesian_stiffness, cartesian_damping)
        resp = self.mode(control_mode = control_mode, cartesian_impedance = cartesian_impedance)

if __name__ == '__main__':
    setmode = SetControlMode()
