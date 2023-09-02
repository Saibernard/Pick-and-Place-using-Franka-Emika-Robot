import sys
import numpy as np
from copy import deepcopy
import math
from math import pi
import time

import rospy
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds
from lib.calculateFK import FK
from lib.calculateIK6 import IK

def blockApproach(q_start, q_end):
    q_curr = deepcopy(q_start)
    step1 = [0, 2, 4, 6]
    q_curr[step1] = q_end[step1]

    arm.safe_move_to_position(q_curr)

    step2 = [1, 3, 5]
    q_curr[step2] = q_end[step2]

    arm.safe_move_to_position(q_curr)

def blockRetreat(q_start, q_end):
    q_curr = deepcopy(q_start)
    step1 = [1, 3, 5]
    q_curr[step1] = q_end[step1]

    arm.safe_move_to_position(q_curr)

    step2 = [0, 2, 4, 6]
    q_curr[step2] = q_end[step2]

    arm.safe_move_to_position(q_curr)

def getDynamicGoal(default_goal_position, stack_size):
    falling_height = 0.005


    _, T07_goal = FK.forward(FK(), default_goal_position)

    ik_goal = {'R':T07_goal[0:3, 0:3], 't':T07_goal[:3, 3]}
    ik_goal['t'][2] += (stack_size) * 0.05 + falling_height

    new_goal_config = IK.panda_ik(IK(), ik_goal)
    return new_goal_config[0]

def blue_routine():

    start_position = np.array([-0.07421933, -0.10978325, 0.29795374, -1.78864852, 0.05834795, 1.66149177, 0.97243972])
    arm.safe_move_to_position(start_position)

    # get the transform from camera to panda_end_effector
    H_ee_camera = detector.get_H_ee_camera()
    print(H_ee_camera)

    # Detect some blocks...
    for (name, pose) in detector.get_detections():
         print(name,'\n',pose)
         print(type(name))
         print(type(pose))

    pose_list_for_i = detector.get_detections()
    print(len(pose_list_for_i)) # to set the number of blocks as i

    i = 0
    while i < len(pose_list_for_i):
    # for i in range(4):
        pose_list = detector.get_detections()
        print("updated_i", i)



            # if not target_pose:
            #     break

        if not pose_list:
            print("No blocks detected")
            # arm.safe_move_to_position(start_position)
            break

        if(pose_list[0][1][2:3,3]) > 0.36:
            if len(pose_list) > 1:
                target_name = pose_list[1][0]
                target_pose = pose_list[1][1]
                print(target_pose)
            else:
                break


        # Get target name and pose in camera space

        if(pose_list[0][1][2:3,3]) < 0.36:
            target_name = pose_list[0][0]
            target_pose = pose_list[0][1]

        # Get target pose in end-effector space
        target_pose_e = H_ee_camera @ target_pose

        # Get target pose in world space
        _, T07 = FK.forward(FK(), start_position)
        target_pose_w = T07 @ target_pose_e
        print("Target pose",target_pose_w)

        # Find config where arm grabs target
        ik_input = {'R': T07[0:3,0:3], 't': target_pose_w[:3, 3]}
        print(ik_input)
        ik_input['t'][2] -= 0.012 # Make arm get closer to block during grab  # old value = 0.012
        new_config = IK.panda_ik(IK(), ik_input)
        ik_input_approach = ik_input

        # Calculation of end effector rotation angle

        if np.round(target_pose_w[2:3,0]) == -1 or np.round(target_pose_w[2:3,0]) == 1.00 :
            angle = math.atan(-target_pose_w[1:2,1] / target_pose_w[0:1,1]) # change to - and see
            print("end efe angle1", angle)


        if np.round(target_pose_w[2:3,1]) == -1 or np.round(target_pose_w[2:3,1]) == 1.00:
            angle = math.atan(-(target_pose_w[1:2,0]/target_pose_w[0:1,0]))
            print("end efe angle2", angle)

        if np.round(target_pose_w[2:3,2]) == -1.00 or np.round(target_pose_w[2:3,2]) == 1.00 :
            print("hi")
            angle = math.atan(-(target_pose_w[1:2,0]/target_pose_w[0:1,0]))
            print("end efe angle3", angle)



        ik_input_approach['t'][2] += 0.03 # prepare to grab position
        new_config_approach = IK.panda_ik(IK(), ik_input_approach)
        arm.exec_gripper_cmd(10,50) # Open gripper
        # new_config_approach[0][6] = pi/4


        # Condition for rotating the end effector
        difference = pi/4-new_config[0][6]

        if angle <0.01 and angle >-0.01:
            print("angle less than zero", angle)
            new_config_approach[0][6] = new_config_approach[0][6] + angle

        else:
            print("angle positive", angle)
            # new_config[0][6] = new_config_approach[0][6] + angle + pi/4
            new_config_approach[0][6] = new_config_approach[0][6] + angle - difference - 0.15


        arm.safe_move_to_position(new_config_approach[0])# Go to object
        print("6th angle",new_config_approach[0][6])
        print("6th angle", new_config[0][6])

        #Changing the angle so the end effector angle rotates before reaching the block
        new_config[0][6] = new_config_approach[0][6]

        arm.safe_move_to_position(new_config[0]) # Go to object
        arm.exec_gripper_cmd(0.052,50) # minimal close for adjusting blocks # need to be tuned
        arm.exec_gripper_cmd(10,50) # open gripper
        arm.exec_gripper_cmd(0, 50) #close gripper
        # print("new", new_config)

        # Move arm
         # Close gripper
        # arm.safe_move_to_position(start_position) # Return to start_position

        if (int(arm.get_gripper_state()['position'][0] * 100) == 0 or int(arm.get_gripper_state()['position'][1] * 100) == 0):
            i = i
            # print(i)
            arm.safe_move_to_position(start_position)
            continue

        # Move object to destination
        above_goal_position = np.array([0.053, 0.119,   -0.3349, -1.383, 0.067, 1.528, 0.5026])
        arm.safe_move_to_position(above_goal_position)

        goal_position = np.array([0.053, 0.253,  -0.3349, -2.0347, 0.067, 2.282, 0.5026])
        stack_goal_position = getDynamicGoal(goal_position, i)

        arm.safe_move_to_position(stack_goal_position)

        arm.exec_gripper_cmd(10, 70)

        print(start_position)
        if i <=2:
            blockRetreat(stack_goal_position, start_position)
            i = i+1
        else:
            print('entered and ready for goal position')
            above_goal_position = np.array([0.053, 0.119,   -0.3349, -1.383, 0.067, 1.528, 0.5026])
            arm.safe_move_to_position(above_goal_position)
            i = i+1



    # DYNAMIC BLOCKS APPROACH



    overlook_turntable = np.array([-np.pi/2, np.pi/6, 0, -1, 0, 0.1 + (np.pi/2), np.pi/4])

    # j = 4;
    while i<7:
    # for i in range (4, 8):
        arm.safe_move_to_position(overlook_turntable)
        pose_list = detector.get_detections()
        print('IIIII', i)

        # condition added
        if not pose_list:
            time.sleep(10) # wait for some time and do it.
            pose_list = detector.get_detections()

        if not pose_list:
            time.sleep(5) # wait for some time and do it.
            pose_list = detector.get_detections()

        if not pose_list:
            print("No blocks detected")
            arm.safe_move_to_position(start_position)
            break



        if(pose_list[0][1][2:3,3]) > 0.38:
            if len(pose_list) > 1:
                target_name = pose_list[1][0]
                target_pose = pose_list[1][1]
                if(target_pose[2:3,3]) > 0.38:
                    if len(pose_list) > 1:
                        target_name = pose_list[2][0]
                        target_pose = pose_list[2][1]
                        if (target_pose[2:3,3]) > 0.38:
                            if len(pose_list) > 1:
                                target_name = pose_list[3][0]
                                target_pose = pose_list[3][1]
            else:
                continue




        for block_pose in pose_list:
            # Get target name and pose in camera space
            if(block_pose[1][2:3,3]) < 0.38:
                target_name = block_pose[0]
                target_pose = block_pose[1]

            # Get target pose in end-effector space
            target_pose_e = H_ee_camera @ target_pose

            # Get target pose in world space
            _, T07 = FK.forward(FK(), overlook_turntable)
            target_pose_w = T07 @ target_pose_e

            # Find config where arm grabs target
            arm_waiting_conf = np.array([-1.76, 1.325, -0.167, -0.679, 1.521, 1.628, -1.185])
            _, waiting_T = FK.forward(FK(), arm_waiting_conf)
            print(waiting_T)
            target_pose_w[2, 3] -= 0.01 # Add height offset to avoid collisions # initially 0.01
            ik_waiting_setup = {'R':waiting_T[:3,:3], 't':target_pose_w[:3,3]}
            print("HERE!!!")
            print(ik_waiting_setup)
            final_arm_waiting_conf = IK.panda_ik(IK(), ik_waiting_setup)

            final_arm_waiting_conf = final_arm_waiting_conf[final_arm_waiting_conf[:,0] < 0] # Isolate solutions with negative joint_1 angles
            print(final_arm_waiting_conf)

            if len(final_arm_waiting_conf) == 0 and final_arm_waiting_conf.size == 0:
                continue

            print(np.min(final_arm_waiting_conf[:,0]))
            final_arm_waiting_conf = final_arm_waiting_conf[final_arm_waiting_conf[:,0] == np.min(final_arm_waiting_conf[:,0])] # Isolate solutions with positive joint_1 angles

            # Move arm
            arm.exec_gripper_cmd(10,70) # Open gripper
            temp_final_arm_waiting_conf = deepcopy(final_arm_waiting_conf[0])
            temp_final_arm_waiting_conf[0] = -2.897


            blockApproach(overlook_turntable, temp_final_arm_waiting_conf) # Go to object
            arm.safe_move_to_position(final_arm_waiting_conf[0])
            time.sleep(2) # wait for a bit
            arm.exec_gripper_cmd(0, 70) # Close gripper

            grabbed_block = False
            attempt_counter = 0
            while (not(grabbed_block)):
                if (attempt_counter == 3):
                    break
                print(arm.get_gripper_state())
                if (int(arm.get_gripper_state()['position'][0] * 100) != 0 or int(arm.get_gripper_state()['position'][1] * 100) != 0):
                    grabbed_block = True
                else:
                    attempt_counter += 1
                    arm.exec_gripper_cmd(10,70) # Open gripper
                    time.sleep(5);
                    arm.exec_gripper_cmd(0, 70) # Close gripper

            if (attempt_counter == 3):
                blockRetreat(final_arm_waiting_conf[0], above_goal_position)
                break
            arm.exec_gripper_cmd(10, 70) # Open gripper
            time.sleep(1)
            arm.exec_gripper_cmd(0, 70) # Close gripper

            # Move object to destination
            above_goal_position = np.array([0.053, 0.119, -0.3349, -1.383, 0.067, 1.528, 0.5026])
            blockRetreat(final_arm_waiting_conf[0], above_goal_position)

            if (int(arm.get_gripper_state()['position'][0] * 100) == 0 or int(arm.get_gripper_state()['position'][1] * 100) == 0):
                i = i
                break

            goal_position = np.array([0.053, 0.253,  -0.3349, -2.0347, 0.067, 2.282, 0.5026])
            stack_goal_position = getDynamicGoal(goal_position, i)

            arm.safe_move_to_position(stack_goal_position)

            arm.exec_gripper_cmd(10, 70)
            above_goal_position = np.array([0.053, 0.119,   -0.3349, -1.383, 0.067, 1.528, 0.5026])
            arm.safe_move_to_position(above_goal_position)
            i = i+1
            print("j_updated", i)
            # break out of block pose loop to get block positions again
            break


    # END STUDENT CODE


def red_routine():

    start_position = np.array([-0.07421933, -0.19, -0.29795374, -1.78864852, -0.05834795, 1.66149177, 0.42])
    arm.safe_move_to_position(start_position)

    # get the transform from camera to panda_end_effector
    H_ee_camera = detector.get_H_ee_camera()
    print(H_ee_camera)

    # Detect some blocks...
    for (name, pose) in detector.get_detections():
         print(name,'\n',pose)
         print(type(name))
         print(type(pose))

    pose_list_for_i = detector.get_detections()
    print(len(pose_list_for_i)) # to set the number of blocks as i

    i = 0
    while i < len(pose_list_for_i):
    # for i in range(4):
        pose_list = detector.get_detections()
        print("updated_i", i)



            # if not target_pose:
            #     break

        if not pose_list:
            print("No blocks detected")
            # arm.safe_move_to_position(start_position)
            break

        if(pose_list[0][1][2:3,3]) > 0.39:
            if len(pose_list) > 1:
                target_name = pose_list[1][0]
                target_pose = pose_list[1][1]
                print(target_pose)
            else:
                break


        # Get target name and pose in camera space
        if(pose_list[0][1][2:3,3]) < 0.39:
            target_name = pose_list[0][0]
            target_pose = pose_list[0][1]

        # Get target pose in end-effector space
        target_pose_e = H_ee_camera @ target_pose

        # Get target pose in world space
        _, T07 = FK.forward(FK(), start_position)
        target_pose_w = T07 @ target_pose_e
        print("Target pose",target_pose_w)

        # Find config where arm grabs target
        ik_input = {'R': T07[0:3,0:3], 't': target_pose_w[:3, 3]}
        print(ik_input)
        ik_input['t'][2] -= 0.012 # Make arm get closer to block during grab  # old value = 0.012
        new_config = IK.panda_ik(IK(), ik_input)
        ik_input_approach = ik_input


        # Calculation of end effector rotation angle

        if np.round(target_pose_w[2:3,0]) == -1 or np.round(target_pose_w[2:3,0]) == 1.00 :
            angle = math.atan(-target_pose_w[1:2,1] / target_pose_w[0:1,1]) # change to - and see
            print("end efe angle1", angle)


        if np.round(target_pose_w[2:3,1]) == -1 or np.round(target_pose_w[2:3,1]) == 1.00:
            angle = math.atan(-(target_pose_w[1:2,0]/target_pose_w[0:1,0]))
            print("end efe angle2", angle)

        if np.round(target_pose_w[2:3,2]) == -1.00 or np.round(target_pose_w[2:3,2]) == 1.00 :
            print("hi")
            angle = math.atan(-(target_pose_w[1:2,0]/target_pose_w[0:1,0]))
            print("end efe angle3", angle)


        ik_input_approach['t'][2] += 0.03 # prepare to grab position
        new_config_approach = IK.panda_ik(IK(), ik_input_approach)
        arm.exec_gripper_cmd(10,50) # Open gripper
        # new_config_approach[0][6] = pi/4


        # Condition for rotating the end effector
        difference = pi/4-new_config[0][6]

        if angle <0.01 and angle >-0.01:
            print("angle less than zero", angle)
            new_config_approach[0][6] = new_config_approach[0][6] + angle
        else:
            print("angle positive", angle)
            # new_config[0][6] = new_config_approach[0][6] + angle + pi/4
            new_config_approach[0][6] = new_config_approach[0][6] + angle - difference + 0.15


        arm.safe_move_to_position(new_config_approach[0])# Go to object
        print("6th angle",new_config_approach[0][6])
        print("6th angle", new_config[0][6])

        #Changing the angle so the end effector angle rotates before reaching the block
        new_config[0][6] = new_config_approach[0][6]

        arm.safe_move_to_position(new_config[0]) # Go to object
        arm.exec_gripper_cmd(0.052,50) # minimal close for adjusting blocks # need to be tuned
        arm.exec_gripper_cmd(10,50) # open gripper
        arm.exec_gripper_cmd(0, 50) #close gripper
        # print("new", new_config)

        # Move arm
         # Close gripper
        # arm.safe_move_to_position(start_position) # Return to start_position

        if (int(arm.get_gripper_state()['position'][0] * 100) == 0 or int(arm.get_gripper_state()['position'][1] * 100) == 0):
            i = i
            # print(i)
            arm.safe_move_to_position(start_position)
            continue

        # Move object to destination
        above_goal_position = np.array([-0.053, 0.119,   0.3349, -1.383, -0.067, 1.528, 1.08])
        arm.safe_move_to_position(above_goal_position)

        goal_position = np.array([-0.053, 0.253,  0.3349, -2.0347, -0.067, 2.282, 1.08])
        stack_goal_position = getDynamicGoal(goal_position, i)

        arm.safe_move_to_position(stack_goal_position)

        arm.exec_gripper_cmd(10, 70)

        print(start_position)
        if i <=2:
            blockRetreat(stack_goal_position, start_position)
            i = i+1
        else:
            print('entered and ready for goal position')
            above_goal_position = np.array([-0.053, 0.119,   0.3349, -1.383, -0.067, 1.528, 1.08])
            arm.safe_move_to_position(above_goal_position)
            i = i+1



    #DYNAMIC BLOCKS APPROACH
    overlook_turntable = np.array([np.pi/2, np.pi/6, 0, -1, 0, 0.1 + (np.pi/2), np.pi/4])

    # j = 4;
    while i<7:
    # for i in range (4, 8):
        arm.safe_move_to_position(overlook_turntable)


        pose_list = detector.get_detections()

        # condition added
        if not pose_list:
            time.sleep(10) # wait for some time and do it.
            pose_list = detector.get_detections()

        if not pose_list:
            time.sleep(5) # wait for some time and do it.
            pose_list = detector.get_detections()

        if not pose_list:
            print("No blocks detected")
            arm.safe_move_to_position(start_position)
            break

        if(pose_list[0][1][2:3,3]) > 0.39:
            if len(pose_list) > 1:
                target_name = pose_list[1][0]
                target_pose = pose_list[1][1]
                if(target_pose[2:3,3]) > 0.39:
                    if len(pose_list) > 1:
                        target_name = pose_list[2][0]
                        target_pose = pose_list[2][1]
                        if (target_pose[2:3,3]) > 0.39:
                            if len(pose_list) > 1:
                                target_name = pose_list[3][0]
                                target_pose = pose_list[3][1]
            else:
                continue

        for block_pose in pose_list:
            # Get target name and pose in camera space
            if(block_pose[1][2:3,3]) < 0.39:
                target_name = block_pose[0]
                target_pose = block_pose[1]

            # Get target pose in end-effector space
            target_pose_e = H_ee_camera @ target_pose

            # Get target pose in world space
            _, T07 = FK.forward(FK(), overlook_turntable)
            target_pose_w = T07 @ target_pose_e

            # Find config where arm grabs target
            arm_waiting_conf = np.array([1.76, 1.325, -0.167, -0.679, 1.521, 1.628, -1.185])
            _, waiting_T = FK.forward(FK(), arm_waiting_conf)
            print(waiting_T)
            # target_pose_w[2, 3] += 0.001 # Add height offset to avoid collisions #HERE!!!!!!!!
            ik_waiting_setup = {'R':waiting_T[:3,:3], 't':target_pose_w[:3,3]}

            final_arm_waiting_conf = IK.panda_ik(IK(), ik_waiting_setup)
            final_arm_waiting_conf = final_arm_waiting_conf[final_arm_waiting_conf[:,0] > 0] # Isolate solutions with positive joint_1 angles

            print(final_arm_waiting_conf)
            if len(final_arm_waiting_conf) == 0 and final_arm_waiting_conf.size == 0:
                continue

            print(np.min(final_arm_waiting_conf[:,0]))
            final_arm_waiting_conf = final_arm_waiting_conf[final_arm_waiting_conf[:,0] == np.min(final_arm_waiting_conf[:,0])] # Isolate solutions with positive joint_1 angles

            # Move arm
            arm.exec_gripper_cmd(10,70) # Open gripper
            temp_final_arm_waiting_conf = deepcopy(final_arm_waiting_conf[0])
            temp_final_arm_waiting_conf[0] = np.pi/7
            print(temp_final_arm_waiting_conf)


            blockApproach(overlook_turntable, temp_final_arm_waiting_conf) # Go to object
            arm.safe_move_to_position(final_arm_waiting_conf[0])
            time.sleep(2) # wait for a bit
            arm.exec_gripper_cmd(0, 70) # Close gripper

            grabbed_block = False
            attempt_counter = 0
            while (not(grabbed_block)):
                if (attempt_counter == 3):
                    break
                print(arm.get_gripper_state())
                if (int(arm.get_gripper_state()['position'][0] * 100) != 0 or int(arm.get_gripper_state()['position'][1] * 100) != 0):
                    grabbed_block = True
                else:
                    attempt_counter += 1
                    arm.exec_gripper_cmd(10,70) # Open gripper
                    time.sleep(5);
                    arm.exec_gripper_cmd(0, 70) # Close gripper

            if (attempt_counter == 3):
                blockRetreat(final_arm_waiting_conf[0], above_goal_position)
                break
            arm.exec_gripper_cmd(10, 70) # Open gripper
            time.sleep(1)
            arm.exec_gripper_cmd(0, 70) # Close gripper

            # Move object to destination
            above_goal_position = np.array([-0.053, 0.119,   0.3349, -1.383, -0.067, 1.528, 1.08])
            blockRetreat(final_arm_waiting_conf[0], above_goal_position)

            if (int(arm.get_gripper_state()['position'][0] * 100) == 0 or int(arm.get_gripper_state()['position'][1] * 100) == 0):
                i = i
                break

            goal_position = np.array([-0.053, 0.253,  0.3349, -2.0347, -0.067, 2.282, 1.08])
            stack_goal_position = getDynamicGoal(goal_position, i)

            arm.safe_move_to_position(stack_goal_position)

            arm.exec_gripper_cmd(10, 70)
            above_goal_position = np.array([-0.053, 0.119,   0.3349, -1.383, -0.067, 1.528, 1.08])
            arm.safe_move_to_position(above_goal_position)
            i = i+1
            print("j_updated", i)
            # arm.safe_move_to_position(overlook_turntable)

            # break out of block pose loop to get block positions again
            break


if __name__ == "__main__":
    try:
        team = rospy.get_param("team") # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    rospy.init_node("team_script")
    arm = ArmController()
    detector = ObjectDetector()


    beforestart_position =  np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866])
    arm.safe_move_to_position(beforestart_position) # on your mark!



    print("\n****************")
    if team == 'blue':
        print("** BLUE TEAM  **")
    else:
        print("**  RED TEAM  **")
    print("****************")
    input("\nWaiting for start... Press ENTER to begin!\n") # get set!
    print("Go!\n") # go!

    # STUDENT CODE HERE
    if (team == 'blue'):
        blue_routine()
    elif (team == 'red'):
        red_routine()
    # END STUDENT CODE
