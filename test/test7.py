#!/usr/bin/env python
import rospy
import time
import math
import sys
import os
import rospkg
import numpy as np
import copy
import sim

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

#import modern_robotics as mr
sys.path.append(os.path.join(rospkg.RosPack().get_path('dual_arm_assemble'), 'scripts'))
from robot_control import RobotControl

clientID=sim.simxStart('127.0.0.1',20005,True,True,5000,5)
if clientID!=-1:
    print ('Connected to remote API server')
jointhandles={}
for i in range (0,6):
    er,jointhandles[i]=sim.simxGetObjectHandle(clientID,('UR5_joint'+str(i+1)),sim.simx_opmode_blocking)

print ("============ Starting Moveit setup")
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('UR5controller', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name="manipulator"
group = moveit_commander.MoveGroupCommander(group_name)
scene = moveit_commander.PlanningSceneInterface()
scene.remove_world_object("ground")
rospy.sleep(2)
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = "world"
box_pose.pose.orientation.w = 1.0
box_pose.pose.position.y=0
box_pose.pose.position.x=0
box_pose.pose.position.z = -0.5 
box_name = "ground"
scene.add_box(box_name, box_pose, size=(10, 10, 1))

'''
MotorIDstrings for UR5 follow the convention UR5_jM, where M can 1,2,3,4,5,6,7. 1-6 are to control UR5 joints and 7 is to open RG2 or close RG2.

Available Robot API commands:

1. setMotorTargetJointPosition(motor_id_string, target_joint_angle) - send the target position for motor_id_string to CoppeliaSim
2. getSensorValue(sensor_type), where sensor_type can be from ['RG2_front1', 'RG2_front2', 'RG2_touch', 'Conveyor_belt'] - retrieves the current reading of the sensor sensor_type
3. getMotorCurrentJointPosition(motor_id_string) - retrieves the current angle for motor motor_id_string
4. getRobotWorldLocation() - returns (position, orientation) of the robot with respect to the world frame. Note, that orientation is represented as a quaternion.
5. getCurrentSimTime() - returns the current simulation time
6, 


'''

class UR5Control(RobotControl):

    def __init__(self):
        super(UR5Control, self).__init__(robot_type='UR5')
        rospy.loginfo("UR5 setup complete")
        time.sleep(2.0)

        # main control loop
        while not rospy.is_shutdown():
            # The current simulation time
            current_time = self.getCurrentSimTime()

            # print the sensor value

            V_Left_RG2_front1 = self.getSensorValue('Left_RG2_front1')
            V_Left_RG2_front2 = self.getSensorValue('Left_RG2_front2')
            V_Left_RG2_touch = self.getSensorValue('Left_RG2_touch')
            V_Left_Conveyor_belt = self.getSensorValue('Left_Conveyor_belt')
            Left_UR5_j7 = self.getMotorCurrentJointPosition('Left_UR5_j7')
            #print(Left_UR5_j7)
            #print(V_Left_RG2_front1)
            #pose=group.get_current_pose().pose
            #print(pose)
            
            V_Right_RG2_front1 = self.getSensorValue('Right_RG2_front1')
            V_Right_RG2_front2 = self.getSensorValue('Right_RG2_front2')
            V_Right_RG2_touch = self.getSensorValue('Right_RG2_touch')
            V_Right_Conveyor_belt = self.getSensorValue('Right_Conveyor_belt')
            Right_UR5_j7 = self.getMotorCurrentJointPosition('Right_UR5_j7')
            print("The Right_RG2_front1 sensor value     is %f"%V_Right_RG2_front1)
            print("The Right_RG2_front2 sensor value     is %f"%V_Right_RG2_front2)
            print("The RIght RG2 value is %f"%Right_UR5_j7)

            #print("The Right_RG2_touch sensor value     is %f"%V_Right_RG2_touch)
            #print("The Right_Conveyor_belt sensor value is %f"%V_Right_Conveyor_belt)

            # Step 1
            if V_Right_RG2_front1 < 0.01 and V_Right_RG2_front2 < 0.01:
                pickuppos=[math.pi/2, math.pi, math.pi/2, 0.2732, -0.3562, 0.228]
                #print(pickuppos)
                p = [0.02, 0]
                plan = self.trajgen(pickuppos)
                self.execute_traj(plan, p)

            if V_Right_RG2_front1 > 0.01 and V_Right_RG2_front2 > 0.01 and Right_UR5_j7 > 0:
                A = V_Right_RG2_front1-0.050
                p = [0.02, 0]
                plan = self.plan_cartesian_path(0,-A,0)
                self.execute_cartesian(plan, p)
                p = [-0.015, 0] 
                plan = self.plan_cartesian_path(0,0,0)
                self.execute_traj(plan, p)

            if V_Right_RG2_front1 > 0.01 and V_Right_RG2_front2 > 0.01 and Right_UR5_j7 < -0.0149:
                p = [-0.03, 0] 
                plan = self.plan_cartesian_path(0,0,0.05)
                self.execute_cartesian(plan, p)
                pickuppos = [1.57, -0.785, 0.785, 0, 1.57, 1.57]
                plan = self.anglegen(pickuppos)
                self.execute_traj(plan, p)

            time.sleep(0.1) # change the sleep time to whatever is the appropriate control rate for simulation
            


    def execute_traj(self, data, p):
        traj=data.joint_trajectory.points
        for j in range (1,len(traj)):
            targetpos=traj[j].positions
            sim.simxSetJointTargetPosition(clientID,jointhandles[0],targetpos[0]-math.pi/2,sim.simx_opmode_streaming)
            sim.simxSetJointTargetPosition(clientID,jointhandles[1],targetpos[1]+math.pi/2,sim.simx_opmode_streaming)
            sim.simxSetJointTargetPosition(clientID,jointhandles[2],targetpos[2],sim.simx_opmode_streaming)
            sim.simxSetJointTargetPosition(clientID,jointhandles[3],targetpos[3]+math.pi/2,sim.simx_opmode_streaming)
            sim.simxSetJointTargetPosition(clientID,jointhandles[4],targetpos[4],sim.simx_opmode_streaming)
            sim.simxSetJointTargetPosition(clientID,jointhandles[5],targetpos[5],sim.simx_opmode_oneshot_wait)
            self.setMotorTargetJointPosition('Right_UR5_j7', p[0]) 
            self.setMotorTargetJointPosition('Left_UR5_j7', p[1]) 
        self.simt((traj[-1].time_from_start)/1.5)
        print('execution complete')


    def execute_cartesian(self, data, p):
        traj=data.joint_trajectory.points
        for j in range (1,len(traj)):
            targetpos=traj[j].positions
            sim.simxSetJointTargetPosition(clientID,jointhandles[0],targetpos[0]-math.pi/2,sim.simx_opmode_streaming)
            sim.simxSetJointTargetPosition(clientID,jointhandles[1],targetpos[1]+math.pi/2,sim.simx_opmode_streaming)
            sim.simxSetJointTargetPosition(clientID,jointhandles[2],targetpos[2],sim.simx_opmode_streaming)
            sim.simxSetJointTargetPosition(clientID,jointhandles[3],targetpos[3]+math.pi/2,sim.simx_opmode_streaming)
            sim.simxSetJointTargetPosition(clientID,jointhandles[4],targetpos[4],sim.simx_opmode_streaming)
            i = len(traj)-1
            sim.simxSetJointTargetPosition(clientID,jointhandles[5],traj[i].positions[5],sim.simx_opmode_oneshot_wait)
            self.setMotorTargetJointPosition('Right_UR5_j7', p[0]) 
            self.setMotorTargetJointPosition('Left_UR5_j7', p[1]) 
        self.simt((traj[-1].time_from_start)/1.5)
        print('execution complete')
    
    def control_RG2(self, p):
        self.setMotorTargetJointPosition('Right_UR5_j7', p[0]) 
        self.setMotorTargetJointPosition('Left_UR5_j7', p[1]) 

    
    def anglegen(self, p):
        group.clear_pose_targets()
        group.set_start_state_to_current_state()
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = p[0]
        joint_goal[1] = p[1]
        joint_goal[2] = p[2]
        joint_goal[3] = p[3]
        joint_goal[4] = p[4]
        joint_goal[5] = p[5]
        plan = group.plan(joint_goal)
        group.stop
        return plan
    
    def trajgen(self, p):
        group.clear_pose_targets()
        group.set_start_state_to_current_state()
        quaternion = quaternion_from_euler(p[0],p[1], p[2])
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]
        pose_goal.position.x = p[3]
        pose_goal.position.y = p[4]
        pose_goal.position.z = p[5]
        group.set_pose_target(pose_goal)
        plan = group.plan()
        group.stop
        return plan

    def plan_cartesian_path(self, xscale,yscale,zscale):
        group.clear_pose_targets()
        waypoints=[]
        group.set_start_state_to_current_state()
        pose=geometry_msgs.msg.Pose()
        pose=group.get_current_pose().pose
        quaternion = quaternion_from_euler(math.pi/2, math.pi, math.pi/2)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        pose.position.x +=xscale*1
        pose.position.y +=yscale*1
        pose.position.z +=zscale*1
        waypoints.append(copy.deepcopy(pose))
        (plan, fraction) = group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
        group.stop
        return plan
    
    def simt(self,dt):
        t1=sim.simxGetFloatSignal(clientID,'mySimulationTime',sim.simx_opmode_blocking)[1]
        while rospy.Duration(secs=(sim.simxGetFloatSignal(clientID,'mySimulationTime',sim.simx_opmode_blocking)[1]-t1)) < dt:
            pass





        
if __name__ == "__main__":
    q = UR5Control()
    rospy.spin()