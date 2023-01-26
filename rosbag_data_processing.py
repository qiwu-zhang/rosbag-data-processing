import rosbag
import numpy as np
import roboticstoolbox
from roboticstoolbox import RevoluteDH
import tf
global robot

pi = 3.141592653589798

bag = rosbag.Bag('test.bag')



def rosbag_to_npz():

    ############## get message size for joint states and gripper topic ######################
    size = bag.get_message_count('/joint_states')
    joint_states = np.zeros((size, 6))
    end_effector_position = np.zeros((size, 3))
    end_effector_orientation = np.zeros((size, 3))
    gripper_data_size = bag.get_message_count('icl_gripper/gripper_cmd/output')
    gripper_state = np.zeros((gripper_data_size, 7))

    ############## get numpy array for joint states ##########################################
    msg_num = 0
    for topic, msg, t in bag.read_messages(topics = ['/joint_states']):
        attribute_value = np.array(msg.position)
        #print(attribute_value)
        for j in range(6):
            joint_states[msg_num][j] = attribute_value[j]
        msg_num += 1

   
    ############## get numpy array for gripper ################################################
    i = 0
    for topic, msg, t in bag.read_messages(topics = ['icl_gripper/gripper_cmd/output']):
        gripper_state[i][0] = str(t)
        gripper_state[i][1] = msg.rACT
        gripper_state[i][2] = msg.rGTO
        gripper_state[i][3] = msg.rATR
        gripper_state[i][4] = msg.rPR
        gripper_state[i][5] = msg.rSP
        gripper_state[i][6] = msg.rFR
        i += 1

    bag.close()

    ########################## forward kinematics calculation #########################################
    # Create a UR5 robot
    ur5 = roboticstoolbox.models.UR5()
    # Create a new link object for the end effector
    end_effector = RevoluteDH(a=0, alpha=pi/2, d=0.1, offset=0, qlim=[-pi, pi])
    # Attach the end effector link to the last link of the UR5 arm
    ur5.links.append(end_effector)

    for i, theta in enumerate(joint_states):
        #print(theta)
        #Perform forward kinematics
        T = (ur5.fkine(theta)).A # theta is the input joint angle
        #print(T)
        R = T[0:3, 0:3]
        euler = tf.transformations.euler_from_matrix(R)
        end_effector_orientation[i][0] = euler[0]
        end_effector_orientation[i][1] = euler[1]
        end_effector_orientation[i][2] = euler[2]

        col4 = T[:, -1]
        position = col4[:3]
        #print(first_three)
        end_effector_position[i][0] = position[0]
        end_effector_position[i][1] = position[1]
        end_effector_position[i][2] = position[2]
   
    np.savez('data.npz', joint_states, gripper_state, end_effector_position, end_effector_orientation)


if __name__ == '__main__':
    rosbag_to_npz()
    #topics = bag.get_type_and_topic_info()[1].keys()
    #print(topics)
