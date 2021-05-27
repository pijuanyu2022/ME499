#!/usr/bin/env python
import rospy
import time
import math
import sys
import os
import rospkg
import numpy as np
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

#import modern_robotics as mr
sys.path.append(os.path.join(rospkg.RosPack().get_path('me_cs301_robots'), 'scripts'))
from robot_control import RobotControl

'''
MotorIDstrings for UR5 follow the convention UR5_jM, where M can 1,2,3,4,5,6,7. 1-6 are to control UR5 joints and 7 is to open RG2 or close RG2.

Available Robot API commands:

1. setMotorTargetJointPosition(motor_id_string, target_joint_angle) - send the target position for motor_id_string to CoppeliaSim
2. getSensorValue(sensor_type), where sensor_type can be from ['RG2_front1', 'RG2_front2', 'RG2_touch', 'Conveyor_belt'] - retrieves the current reading of the sensor sensor_type
3. getMotorCurrentJointPosition(motor_id_string) - retrieves the current angle for motor motor_id_string
4. getRobotWorldLocation() - returns (position, orientation) of the robot with respect to the world frame. Note, that orientation is represented as a quaternion.
5. getCurrentSimTime() - returns the current simulation time

Helper functions

degToRad() - Converts degrees to radians


'''

class UR5Control(RobotControl):

    def __init__(self):
        super(UR5Control, self).__init__(robot_type='UR5')
        rospy.loginfo("UR5 setup complete")
        self.hold_neutral()
        time.sleep(2.0)

        #main control loop
        while not rospy.is_shutdown():
            print(".........................................................................") 
            # The current simulation time
            current_time = self.getCurrentSimTime()
            print("The current simulation time is %f"%current_time)

            # print the sensor value

            V_Left_RG2_front1 = self.getSensorValue('Left_RG2_front1')
            V_Left_RG2_front2 = self.getSensorValue('Left_RG2_front2')
            V_Left_RG2_touch = self.getSensorValue('Left_RG2_touch')
            V_Left_Conveyor_belt = self.getSensorValue('Left_Conveyor_belt')

            
            #print("The Left_RG2_front1 sensor value     is %f"%V_Left_RG2_front1)
            #print("The Left_RG2_front2 sensor value     is %f"%V_Left_RG2_front2)
            #print("The Left_RG2_touch sensor value     is %f"%V_Left_RG2_touch)
            #print("The Left_Conveyor_belt sensor value is %f"%V_Left_Conveyor_belt)

            # print the joint value
            Left_UR5_j1 = self.getMotorCurrentJointPosition('Left_UR5_j1')
            Left_UR5_j2 = self.getMotorCurrentJointPosition('Left_UR5_j2')
            Left_UR5_j3 = self.getMotorCurrentJointPosition('Left_UR5_j3')
            Left_UR5_j4 = self.getMotorCurrentJointPosition('Left_UR5_j4')
            Left_UR5_j5 = self.getMotorCurrentJointPosition('Left_UR5_j5')
            Left_UR5_j6 = self.getMotorCurrentJointPosition('Left_UR5_j6')
            Left_UR5_j7 = self.getMotorCurrentJointPosition('Left_UR5_j7')
        
            print("The joint Left_UR5_j1 value is %f"%Left_UR5_j1)
            print("The joint Left_UR5_j2 value is %f"%Left_UR5_j2)    
            print("The joint Left_UR5_j3 value is %f"%Left_UR5_j3)
            print("The joint Left_UR5_j4 value is %f"%Left_UR5_j4)
            print("The joint Left_UR5_j5 value is %f"%Left_UR5_j5)
            print("The joint Left_UR5_j6 value is %f"%Left_UR5_j6)
            print("The joint Left_UR5_j7 value is %f"%Left_UR5_j7) 
            print(".........................................................................") 
            V_Right_RG2_front1 = self.getSensorValue('Right_RG2_front1')
            V_Right_RG2_front2 = self.getSensorValue('Right_RG2_front2')
            V_Right_RG2_touch = self.getSensorValue('Right_RG2_touch')
            V_Right_Conveyor_belt = self.getSensorValue('Right_Conveyor_belt')

            # print the joint value
            Right_UR5_j1 = self.getMotorCurrentJointPosition('Right_UR5_j1')
            Right_UR5_j2 = self.getMotorCurrentJointPosition('Right_UR5_j2')
            Right_UR5_j3 = self.getMotorCurrentJointPosition('Right_UR5_j3')
            Right_UR5_j4 = self.getMotorCurrentJointPosition('Right_UR5_j4')
            Right_UR5_j5 = self.getMotorCurrentJointPosition('Right_UR5_j5')
            Right_UR5_j6 = self.getMotorCurrentJointPosition('Right_UR5_j6')
            Right_UR5_j7 = self.getMotorCurrentJointPosition('Right_UR5_j7')

            print("The joint Right_UR5_j1 value is %f"%Right_UR5_j1)
            print("The joint Right_UR5_j2 value is %f"%Right_UR5_j2)
            print("The joint Right_UR5_j3 value is %f"%Right_UR5_j3)
            print("The joint Right_UR5_j4 value is %f"%Right_UR5_j4)
            print("The joint Right_UR5_j5 value is %f"%Right_UR5_j5)
            print("The joint Right_UR5_j6 value is %f"%Right_UR5_j6)
            print("The joint Right_UR5_j7 value is %f"%Right_UR5_j7) 


            # Segment 1

            # Right arm move
            # when the conveyor belt stops, the right arm goes to the pick up position
            if V_Right_RG2_front1 < 0.01 and V_Right_RG2_front2 < 0.01 and Right_UR5_j1 < 0.0001 and Right_UR5_j1 > -0.0001 and Right_UR5_j2 < 0.0001 and Right_UR5_j2 > -0.0001: 

                print (".........................................................................")
                print("/////////....Segment 1 : Right robot arm goes to the ready position....../////////")

                # set the value of N and Tf
                N = 2000
                Tf = 3

                # set the start point 
                Right_thetastart_1 = np.array([0, 0, 0, 0, 0, 0])

                # set the end effector position of right arm
                Right_end_effector1 = [0.20188, -1.2325,  0.66934, 0, 0, 1.57]

                # get the end position of right arm
                R1= np.array(self.get_joint_value(Right_end_effector1))
                Right_thetaend_1 = np.array([R1[0], R1[1]+1.57, R1[2], R1[3]+1.57, R1[4], R1[5]+1.57])
                self.simple(Right_thetaend_1)
                
                # open the gripper
                Right_RG2_1 = 0

                # move to the pick up position
                self.Right_move_position(Right_thetastart_1, Right_thetaend_1, N, Tf, Right_RG2_1)

                print("//////////////......Complete Right arm movement......//////////////")
                print (".........................................................................")

            # Left arm move
            # when the conveyor belt stops, the Left arm goes to the pick up position
            if V_Left_RG2_front1 < 0.01 and V_Left_RG2_front2 < 0.01 and Right_UR5_j1 < 0.0001 and Right_UR5_j1 > -0.0001 and Right_UR5_j2 < 0.0001 and Right_UR5_j2 > -0.0001:
                print (".........................................................................")
                print("/////////....Segment 1 : Left robot arm goes to the ready position....../////////")

                # set the value of N and Tf
                N = 2000
                Tf = 3

                # set the start point 
                Left_thetastart_1 = np.array([0, 0, 0, 0, 0, 0])

                # set the end effector position of left arm
                Left_end_effector1 = [0.20188, -1.1325,  0.66934, 0, 0, 1.57]

                # get the end position of left arm
                L1= np.array(self.get_joint_value(Left_end_effector1))
                Left_thetaend_1 = np.array([L1[0], L1[1]+1.57, L1[2], L1[3]+1.57, L1[4], L1[5]+1.57])
                self.simple(Left_thetaend_1)

                # open the gripper
                Left_RG2_1 = 0

                # move to the pick up position
                self.Left_move_position(Left_thetastart_1, Left_thetaend_1, N, Tf, Left_RG2_1)
                print("//////////////......Complete Left arm movement......//////////////")
                print (".........................................................................")

            # Segment 2

            # Left arm move
            if V_Left_RG2_front1 > 0.1 and V_Left_RG2_front2 > 0.1 and V_Left_RG2_touch < 0.01 and Left_UR5_j4 > 0.01:
                print (".........................................................................")
                print("////////....Segment 2 : Picking up the first left component......//////////////")
                
                # set the value of N and Tf
                N = 1500
                Tf = 3

                # first step, open the gripper, close to the component

                # In the first step, the start position is the end position of segment 1
                Left_thetastart_2_1 = Left_thetaend_1

                # Set the end effector positions of left arm  to close to the component
                Left_end_effector2 = [0.20188, -1.4045,  0.66934, 0, 0, 1.57]

                # get the end position of left arm
                L2 = np.array(self.get_joint_value(Left_end_effector2))
                Left_thetaend_2_1 = np.array([L2[0], L2[1]+1.57, L2[2], L2[3]+1.57, L2[4], L2[5]+1.57])
                self.simple(Left_thetaend_2_1)

                # open the gripper
                Left_RG2_2 = 0.03

                # close to the component, when the distance between the base of gripper and the component is less than 0.055, the robot arm stop
                N = int(N)
                timegap = Tf / (N - 1.0)
                traj = np.zeros((len(Left_thetastart_2_1), N))
                for i in range(N):
                    s = 10 * (1.0 * timegap * i / Tf) ** 3 - 15 * (1.0 * timegap * i / Tf) ** 4 \
                        + 6 * (1.0 * timegap * i / Tf) ** 5
                    traj[:, i] = s * np.array(Left_thetaend_2_1) + (1 - s) * np.array(Left_thetastart_2_1)
                    A = traj[:, i]
                    B = self.getSensorValue('Left_RG2_front1')
                    if B > 0.055: 
                        C = [A[0], A[1], A[2], A[3], A[4], A[5], B]
                        Left_joint_pos = [C[0], C[1], C[2], C[3], C[4], C[5], Left_RG2_2]
                        self.Left_move(Left_joint_pos) 
                # second step, close the gripper
                Left_thetastart_2_2 = [C[0], C[1], C[2], C[3], C[4], C[5]]
                Left_thetaend_2_2 = [C[0], C[1], C[2], C[3], C[4], C[5]]

                # set the RG2 value to -0.05 to close the gripper
                self.Left_move_position(Left_thetastart_2_2, Left_thetaend_2_2, 500, Tf, -0.05)
                print("//////////////......Complete Left arm movement......//////////////")
                print (".........................................................................")

            # Right arm move
            if V_Right_RG2_front1 > 0.1 and V_Right_RG2_front2 > 0.1 and V_Right_RG2_touch < 0.01 :
                print (".........................................................................")
                print("////////....Segment 2 : Picking up the Right component......//////////////")
                print(Right_UR5_j1)
                
                # set the value of N and Tf
                N = 1500
                Tf = 3

                # first step, open the gripper, close to the Right component

                # In the first step, the start position is the end position of segment 1
                Right_thetastart_2_1 = Right_thetaend_1

                # Set the end effector positions of Right arm to close to the Right component
                Right_end_effector2 = [0.20188, -1.4045,  0.66934, 0, 0, 1.57]

                # get the end position of Right arm
                R2 = np.array(self.get_joint_value(Right_end_effector2))
                Right_thetaend_2_1 = np.array([R2[0], R2[1]+1.57, R2[2], R2[3]+1.57, R2[4], R2[5]+1.57])
                self.simple(Right_thetaend_2_1)

                # open the gripper
                Right_RG2_2 = 0.03

                # close to the component, when the distance between the base of gripper and the component is less than 0.055, the robot arm stop
                N = int(N)
                timegap = Tf / (N - 1.0)
                traj = np.zeros((len(Right_thetastart_2_1), N))
                for i in range(N):
                    s = 10 * (1.0 * timegap * i / Tf) ** 3 - 15 * (1.0 * timegap * i / Tf) ** 4 \
                        + 6 * (1.0 * timegap * i / Tf) ** 5
                    traj[:, i] = s * np.array(Right_thetaend_2_1) + (1 - s) * np.array(Right_thetastart_2_1)
                    A = traj[:, i]
                    B = self.getSensorValue('Right_RG2_front1')
                    if B > 0.055: 
                        C = [A[0], A[1], A[2], A[3], A[4], A[5], B]
                        Right_joint_pos = [C[0], C[1], C[2], C[3], C[4], C[5], Right_RG2_2]
                        self.Right_move(Right_joint_pos) 
                # second step, close the gripper
                Right_thetastart_2_2 = [C[0], C[1], C[2], C[3], C[4], C[5]]
                Right_thetaend_2_2 = [C[0], C[1], C[2], C[3], C[4], C[5]]

                # set the RG2 value to -0.05 to close the gripper
                self.Right_move_position(Right_thetastart_2_2, Right_thetaend_2_2, 1000, Tf, -0.05)
                print("//////////////......Complete Right arm movement......//////////////")
                print (".........................................................................")

            
            # Segment 3
 
            # Right robot arm go to the assemble position 
            if V_Right_RG2_front1 < 0.06 and V_Right_RG2_front1 > 0.01 and Right_UR5_j6 < 0 and Left_UR5_j7 < 0:
                print (".........................................................................")
                print("////////....Segment 3 : Right arm Go to the assmeble position......//////////////")

                # set the value of N and Tf
                N = 2000
                Tf = 6

                # the start position is the end position of step2 in segment 2
                Right_thetastart_3 = Right_thetaend_2_2

                # Set the end effector positions of Right arm to go to the assemble position
                Right_end_effector3 = [-0.1651, -0.1, 0.71214, 0, 0, 1.57]

                # get the end position of the Right arm
                R3 = np.array(self.get_joint_value(Right_end_effector3))
                Right_thetaend_3 = np.array([R3[0], R3[1]+1.57, R3[2], R3[3]+1.57, R3[4], R3[5]+1.57])
                self.simple(Right_thetaend_3)

                # close the gripper
                Right_RG2_3 = -0.05

                # go to the end position from the start position
                self.Right_move_position(Right_thetastart_3, Right_thetaend_3, N, Tf, Right_RG2_3) 
                print("//////////////......Complete Right arm movement......//////////////")
                print (".........................................................................")

 
            # left robot arm go to the assemble position           
            if V_Left_RG2_front1 < 0.06 and V_Left_RG2_front1 > 0.01 and Right_UR5_j6 > 0.01: 
                print (".........................................................................")
                print("////////....Segment 3 : Left arm Go to the assmeble position......//////////////")
 

                # set the value of N and Tf
                N = 2000
                Tf = 4

                # Step 1: Go to the assemble position
                print("Step 1: Go to the assemble position")

                # the start position is the end position of step2 in segment 2
                Left_thetastart_3 = Left_thetaend_2_2

                # get the end position of the left arm
                L3 = np.array([4.90051988361323, -1.7967510018164237, -0.8045866852132709, 2.5960736050210276, 1.7576228779485685, -3.136109635911185])
                Left_thetaend_3 = np.array([L3[0], L3[1]+1.57, L3[2], L3[3]+3.14, L3[4], L3[5]+10*math.pi/180])
                self.simple(Left_thetaend_3)

                # close the gripper
                Left_RG2_3 = -0.05

                # go to the end position from the start position
                self.Left_move_position(Left_thetastart_3, Left_thetaend_3, N, Tf, Left_RG2_3) 


                # step 2: drop up the component
                print("Step 2: drop up the first component")
                self.Left_move_position(Left_thetaend_3, Left_thetaend_3, 500, 2, 0.01) 

                # step 3
                print("Step 3: return to the ready position")
                Left_thetastart_3_3 =  Left_thetaend_3

                Left_thetaend_3_3 = Left_thetaend_1 # go to the pick up position
                self.Left_move_position(Left_thetastart_3_3, Left_thetaend_3_3, 2000, 4, 0.01) 



                # Step 4
                # first step, open the gripper, close to the component
                print("Step 4: close to the second component")
                # In the first step, the start position is the end position of segment 1
                Left_thetastart_3_4 = Left_thetaend_3_3

                # Set the end effector positions of left arm  to close to the component
                Left_end_effector_3_4 = [0.20188, -1.4045,  0.66934, 0, 0, 1.57]

                # get the end position of left arm
                L3_4 = np.array(self.get_joint_value(Left_end_effector_3_4))
                Left_thetaend_3_4 = np.array([L3_4[0], L3_4[1]+1.57, L3_4[2], L3_4[3]+1.57, L3_4[4], L3_4[5]+1.57])
                self.simple(Left_thetaend_3_4)

                # open the gripper
                Left_RG2_3_4 = 0.03

                # close to the component, when the distance between the base of gripper and the component is less than 0.055, the robot arm stop
                N = int(N)
                timegap = Tf / (N - 1.0)
                traj = np.zeros((len(Left_thetastart_3_4), N))
                for i in range(N):
                    s = 10 * (1.0 * timegap * i / Tf) ** 3 - 15 * (1.0 * timegap * i / Tf) ** 4 \
                        + 6 * (1.0 * timegap * i / Tf) ** 5
                    traj[:, i] = s * np.array(Left_thetaend_3_4) + (1 - s) * np.array(Left_thetastart_3_4)
                    A = traj[:, i]
                    B = self.getSensorValue('Left_RG2_front1')
                    if B > 0.055: 
                        C = [A[0], A[1], A[2], A[3], A[4], A[5], B]
                        Left_joint_pos = [C[0], C[1], C[2], C[3], C[4], C[5], Left_RG2_3_4]
                        self.Left_move(Left_joint_pos) 

                # Step 5
                print("Step 5: pick up the second compoennt")
                Left_thetastart_3_5 = [C[0], C[1], C[2], C[3], C[4], C[5]]
                Left_thetaend_3_5 = [C[0], C[1], C[2], C[3], C[4], C[5]]

                # set the RG2 value to -0.05 to close the gripper
                self.Left_move_position(Left_thetastart_3_5, Left_thetaend_3_5, 500, Tf, -0.05)


                # Step 6
                print("Step 6: go to the assemble position")
                Left_thetastart_3_6 = Left_thetaend_3_5


                # get the end position of the left arm
                L3_6 = np.array([4.90051988361323, -1.7967510018164237, -0.8045866852132709, 2.5960736050210276, 1.7576228779485685, -3.136109635911185])
                Left_thetaend_3_6 = np.array([L3_6[0], L3_6[1]+1.57, L3_6[2], L3_6[3]+3.14, L3_6[4], L3_6[5]+10*math.pi/180])
                self.simple(Left_thetaend_3_6)

                # close the gripper
                Left_RG2_3_6 = -0.05

                # go to the end position from the start position
                self.Left_move_position(Left_thetastart_3_6, Left_thetaend_3_6, N, Tf, Left_RG2_3_6) 

                # Step 7
                print("Step 7: open the gripper")
                self.Left_move_position(Left_thetaend_3_6, Left_thetaend_3_6, 500, 2, 0.01) 
                # step 8
                print("Step 8: go back to the original position")
                Left_thetaend_3_8 = np.array([0, 0, 0, 0, 0, 0])
                self.Left_move_position(Left_thetaend_3_6, Left_thetaend_3_8, 1000, 2, 0.001) 

                print("//////////////......Complete Left arm movement......//////////////")
                print (".........................................................................")
                
            # Segment 4
            if V_Right_RG2_front1 < 0.06 and V_Right_RG2_front1 > 0.01 and Right_UR5_j6 > 0.01 and Left_UR5_j7 > 0 and Right_UR5_j7 < 0:
                print (".........................................................................")
                print("////////....Segment 4 : Right arm Go to the end position......//////////////")

                # set the value of N and Tf
                N = 2000
                Tf = 4       

                # Step 1
                # the start position is the end position of segment 3
                Right_thetastart_4_1 = Right_thetaend_3 

                # Set the end effector positions of Right arm to go to the assemble position
                #Right_end_effector_4_1 = [0.71907, -0.77598, 0.71214, 0, 0, 1.57]
                #Right_end_effector_4_1 = [0.71907, -0.77598, 0.67714, 0, 0, 1.57]

                # get the end position of the Right arm
                R4_1 = np.array([0.0001689220917272363, -0.6403662497876675, 1.0965222844492437, -3.5981198373078187, -0.0009431754663393743, 6.282232957807842])
                Right_thetaend_4_1 = np.array([R4_1[0]+3.14, R4_1[1]+1.57, R4_1[2], R4_1[3]+1.57, R4_1[4]-1.57, R4_1[5]+1.57])
                self.simple(Right_thetaend_4_1)

                # close the gripper
                Right_RG2_4_1 = -0.05

                self.Right_move_position(Right_thetastart_4_1, Right_thetaend_4_1, N, Tf, Right_RG2_4_1) 

                # step 2
                Right_thetastart_4_2 = Right_thetaend_4_1
                Right_thetaend_4_2 = np.array([Right_thetaend_4_1[0], Right_thetaend_4_1[1] + 10*math.pi/180,Right_thetaend_4_1[2] - 10*math.pi/180,\
                    Right_thetaend_4_1[3],Right_thetaend_4_1[4],Right_thetaend_4_1[5]])
                self.Right_move_position(Right_thetastart_4_2, Right_thetaend_4_2, 2000, Tf, 0.01) 

                # step 3
                Right_thetastart_4_3 = Right_thetaend_4_2
                Right_thetaend_4_3 = np.array([Right_thetaend_4_2[0], Right_thetaend_4_2[1] - 25*math.pi/180, Right_thetaend_4_2[2] + 45*math.pi/180,\
                    Right_thetaend_4_2[3] - 20*math.pi/180, Right_thetaend_4_2[4],Right_thetaend_4_2[5]])
                self.Right_move_position(Right_thetastart_4_3, Right_thetaend_4_3, 500, Tf, 0.01) 

                # step 4 
                Right_thetastart_4_4 = Right_thetaend_4_3
                Right_thetaend_4_4 = np.array([0, 0, 0, 0, 0, 0])
                self.Right_move_position(Right_thetastart_4_4, Right_thetaend_4_4, 2000, Tf, 0.01) 

                print("//////////////......Complete Right arm movement......//////////////")
                print (".........................................................................")

            time.sleep(0.1) # change the sleep time to whatever is the appropriate control rate for simulation
            


    def hold_neutral(self):

        self.setMotorTargetJointPosition('Right_UR5_j1', 0.0)
        self.setMotorTargetJointPosition('Right_UR5_j2', 0.0)
        self.setMotorTargetJointPosition('Right_UR5_j3', 0.0)

        self.setMotorTargetJointPosition('Right_UR5_j4', 0.0)
        self.setMotorTargetJointPosition('Right_UR5_j5', 0.0)
        self.setMotorTargetJointPosition('Right_UR5_j6', 0.0)
        self.setMotorTargetJointPosition('Right_UR5_j7', 0.0)

        self.setMotorTargetJointPosition('Left_UR5_j1', 0.0)
        self.setMotorTargetJointPosition('Left_UR5_j2', 0.0)
        self.setMotorTargetJointPosition('Left_UR5_j3', 0.0)

        self.setMotorTargetJointPosition('Left_UR5_j4', 0.0)
        self.setMotorTargetJointPosition('Left_UR5_j5', 0.0)
        self.setMotorTargetJointPosition('Left_UR5_j6', 0.0)
        self.setMotorTargetJointPosition('Left_UR5_j7', 0.0)

    def Right_move(self, joint_pos):
        self.setMotorTargetJointPosition('Right_UR5_j1', joint_pos[0])
        self.setMotorTargetJointPosition('Right_UR5_j2', joint_pos[1])
        self.setMotorTargetJointPosition('Right_UR5_j3', joint_pos[2])

        self.setMotorTargetJointPosition('Right_UR5_j4', joint_pos[3])
        self.setMotorTargetJointPosition('Right_UR5_j5', joint_pos[4])
        self.setMotorTargetJointPosition('Right_UR5_j6', joint_pos[5])
        self.setMotorTargetJointPosition('Right_UR5_j7', joint_pos[6]) 

    def Left_move(self, joint_pos):
        self.setMotorTargetJointPosition('Left_UR5_j1', joint_pos[0])
        self.setMotorTargetJointPosition('Left_UR5_j2', joint_pos[1])
        self.setMotorTargetJointPosition('Left_UR5_j3', joint_pos[2])

        self.setMotorTargetJointPosition('Left_UR5_j4', joint_pos[3])
        self.setMotorTargetJointPosition('Left_UR5_j5', joint_pos[4])
        self.setMotorTargetJointPosition('Left_UR5_j6', joint_pos[5])
        self.setMotorTargetJointPosition('Left_UR5_j7', joint_pos[6]) 

    def Right_move_position(self, thetastart, thetaend, N, Tf, RG2):
        method = 5  
        N = int(N)
        timegap = Tf / (N - 1.0)
        traj = np.zeros((len(thetastart), N))
        for i in range(N):
            if method == 3:
                s = 3 * (1.0 * timegap * i / Tf) ** 2 - 2 * (1.0 * timegap * i / Tf) ** 3
            else:
                s = 10 * (1.0 * timegap * i / Tf) ** 3 - 15 * (1.0 * timegap * i / Tf) ** 4 \
                    + 6 * (1.0 * timegap * i / Tf) ** 5
            traj[:, i] = s * np.array(thetaend) + (1 - s) * np.array(thetastart)
            A = traj[:, i]
            joint_pos = [A[0], A[1], A[2], A[3], A[4], A[5], RG2]
            self.Right_move(joint_pos)

    def Left_move_position(self, thetastart, thetaend, N, Tf, RG2):
        method = 5  
        N = int(N)
        timegap = Tf / (N - 1.0)
        traj = np.zeros((len(thetastart), N))
        for i in range(N):
            if method == 3:
                s = 3 * (1.0 * timegap * i / Tf) ** 2 - 2 * (1.0 * timegap * i / Tf) ** 3
            else:
                s = 10 * (1.0 * timegap * i / Tf) ** 3 - 15 * (1.0 * timegap * i / Tf) ** 4 \
                    + 6 * (1.0 * timegap * i / Tf) ** 5
            traj[:, i] = s * np.array(thetaend) + (1 - s) * np.array(thetastart)
            A = traj[:, i]
            joint_pos = [A[0], A[1], A[2], A[3], A[4], A[5], RG2]
            self.Left_move(joint_pos)


    def get_joint_value(self, end_effector):
        group_name = "manipulator"
        group = moveit_commander.MoveGroupCommander(group_name)
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = end_effector[0] + 0.05612  # 0.20188 + 0.05612 = 0.258        # 0.77519
        pose_goal.position.y = end_effector[1] + 0.8852   # (-1.2325) + 0.8852 = -0.3473     # 0.10922
        pose_goal.position.z = end_effector[2] - 0.4184   # 0.66934 -0.4193 = 0.25 
        roll_angle = end_effector[3]
        pitch_angle = end_effector[4]
        yaw_angle = end_effector[5]
        # Pose Orientation
        quaternion = quaternion_from_euler(roll_angle, pitch_angle, yaw_angle)
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = 1
        j = 0
        for j in range(300):
            j += 1
            (plan) = group.plan(pose_goal)
            i = len(plan.joint_trajectory.points)-1
            A1 = plan.joint_trajectory.points[i].positions[0]
            A2 = plan.joint_trajectory.points[i].positions[1]
            A3 = plan.joint_trajectory.points[i].positions[2]
            A4 = plan.joint_trajectory.points[i].positions[3]
            A5 = plan.joint_trajectory.points[i].positions[4]+1.57
            A6 = plan.joint_trajectory.points[i].positions[5]
            if A1 > -3.1415*2 and A1 < 3.1415*2 and \
                A2 > -3.1415*2 and A2 < 3.1415*2 and \
                A3 > -3.1415*2 and A3 < 3.1415*2 and \
                A4 > -3.1415*2 and A4 < 3.1415*2 and \
                A5 > -3.1415*2 and A5 < 3.1415*2 and \
                A6 > -3.1415*2 and A6 < 3.1415*2:
                H = [A1, A2, A3, A4, A5, A6]
                break

        return H
    
    def simple(self, theta):
        if theta[0] > 3.1415:
            theta[0] = theta[0] - 3.1415*2
        if theta[0] < -3.1415:
            theta[0] = theta[0] + 3.1415*2

        if theta[1] > 3.1415:
            theta[1] = theta[1] - 3.1415*2
        if theta[1] < -3.1415:
            theta[1] = theta[1] + 3.1415*2

        if theta[2] > 3.1415:
            theta[2] = theta[2] - 3.1415*2
        if theta[2] < -3.1415:
            theta[2] = theta[2] + 3.1415*2
        
        if theta[3] > 3.1415:
            theta[3] = theta[3] - 3.1415*2
        if theta[3] < -3.1415:
            theta[3] = theta[3] + 3.1415*2
        
        if theta[4] > 3.1415:
            theta[4] = theta[4] - 3.1415*2
        if theta[4] < -3.1415:
            theta[4] = theta[4] + 3.1415*2
        
        if theta[5] > 3.1415:
            theta[5] = theta[5] - 3.1415*2
        if theta[5] < -3.1415:
            theta[5] = theta[5] + 3.1415*2


        
if __name__ == "__main__":
    q = UR5Control()
    rospy.spin()