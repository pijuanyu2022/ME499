#!/usr/bin/env python
import rospy
import time
import math
import sys
import os
import rospkg
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
sys.path.append(os.path.join(rospkg.RosPack().get_path('me_cs301_robots'), 'scripts'))
from robot_control import RobotControl

'''
MotorIDstrings for Hexapod follow the convention legN_jM, where N can be 1,2,3,4 (for each of the 4 legs) and M can 1,2,3. M=1 is the joint closest to the body and M=3 is the joint farthest from the body.

Available Robot API commands:

1. setMotorTargetJointPosition(motor_id_string, target_joint_angle) - send the target position for motor_id_string to CoppeliaSim
2. getSensorValue(sensor_type), where sensor_type can be from ['front', 'left', 'right'] - retrieves the current reading of the sensor sensor_type
3. getMotorCurrentJointPosition(motor_id_string) - retrieves the current angle for motor motor_id_string
4. getRobotWorldLocation() - returns (position, orientation) of the robot with respect to the world frame. Note, that orientation is represented as a quaternion.
5. getCurrentSimTime() - returns the current simulation time

Helper functions

degToRad() - Converts degrees to radians

Note that, this list of API functions could potentially grow. You will be notified via Canvas if anything is updated
'''

class RWControl(RobotControl):
    def __init__(self):
        super(RWControl, self).__init__(robot_type='rollerwalker')
        self.hold_neutral()
        rospy.loginfo("setup complete")
        time.sleep(2.0)

        #main control loop
        while not rospy.is_shutdown():
            # self.hold_neutral() #remove if not necessary
            print(".........................................................................")

            # The current simulation time
            current_time = self.getCurrentSimTime()
            print("The current simulation time is %f"%current_time)

            # print the sensor value
            V_front = self.getSensorValue('front')
            print("The front sensor value is %f"%V_front)
            V_right = self.getSensorValue('right')
            print("The right sensor value is %f"%V_right)
            V_left = self.getSensorValue('left')
            print("The left sensor value is %f"%V_left)

            # print the joint value
            leg1_1 = self.getMotorCurrentJointPosition('leg1_j1')
            print("The joint leg1_j1 value is %f"%leg1_1)
            leg1_2 = self.getMotorCurrentJointPosition('leg1_j2')
            print("The joint leg1_j2 value is %f"%leg1_2)
            leg1_3 = self.getMotorCurrentJointPosition('leg1_j3')
            print("The joint leg1_j3 value is %f"%leg1_3)

            leg2_1 = self.getMotorCurrentJointPosition('leg2_j1')
            print("The joint leg2_j1 value is %f"%leg2_1)
            leg2_2 = self.getMotorCurrentJointPosition('leg2_j2')
            print("The joint leg2_j2 value is %f"%leg2_2)
            leg2_3 = self.getMotorCurrentJointPosition('leg2_j3')
            print("The joint leg2_j3 value is %f"%leg2_3)

            leg3_1 = self.getMotorCurrentJointPosition('leg3_j1')
            print("The joint leg3_j1 value is %f"%leg3_1)
            leg3_2 = self.getMotorCurrentJointPosition('leg3_j2')
            print("The joint leg3_j2 value is %f"%leg3_2)
            leg3_3 = self.getMotorCurrentJointPosition('leg3_j3')
            print("The joint leg3_j3 value is %f"%leg3_3)

            leg4_1 = self.getMotorCurrentJointPosition('leg4_j1')
            print("The joint leg4_j1 value is %f"%leg4_1)
            leg4_2 = self.getMotorCurrentJointPosition('leg4_j2')
            print("The joint leg4_j2 value is %f"%leg4_2)
            leg4_3 = self.getMotorCurrentJointPosition('leg4_j3')
            print("The joint leg4_j3 value is %f"%leg4_3)

            # print the robot location respect to the world frame
            World_location = self.getRobotWorldLocation()
            print("The world location of the robot is")
            print(World_location) 


            print(".........................................................................")
            time.sleep(0.1) # change the sleep time to whatever is the appropriate control rate for simulation
            
    def hold_neutral(self):
        # --- simple example of a behavior ---- #
        self.setMotorTargetJointPosition('leg1_j1', 1.0)
        self.setMotorTargetJointPosition('leg1_j2', 0.0)
        self.setMotorTargetJointPosition('leg1_j3', self.degToRad(90.0)) #note that the API also provides a helper function to convert degrees to Radians

        self.setMotorTargetJointPosition('leg2_j1', -1.0)
        self.setMotorTargetJointPosition('leg2_j2', 0.0)
        self.setMotorTargetJointPosition('leg2_j3', math.pi/2)

        self.setMotorTargetJointPosition('leg3_j1', 1.0)
        self.setMotorTargetJointPosition('leg3_j2', 0.0)
        self.setMotorTargetJointPosition('leg3_j3', math.pi/2)
        
        self.setMotorTargetJointPosition('leg4_j1', -1.0)
        self.setMotorTargetJointPosition('leg4_j2', 0.0)
        self.setMotorTargetJointPosition('leg4_j3', math.pi/2)


if __name__ == "__main__":
    q = RWControl()
    rospy.spin()
