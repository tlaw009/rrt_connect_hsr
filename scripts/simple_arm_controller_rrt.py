#!/usr/bin/env python
import controller_manager_msgs.srv
import rospy
import trajectory_msgs.msg
import geometry_msgs.msg
import hsrb_interface, rospkg, numpy, yaml, sys
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import *
from rrt_connect_hsr.srv import *
from tf import TransformListener
import numpy as np

#some init stuff
rospy.init_node('test')
robot = hsrb_interface.robot.Robot()
omni_base = robot.get('omni_base')
tf = TransformListener()

# initialize ROS publisher
pub = rospy.Publisher('/hsrb/arm_trajectory_controller/command',
                      trajectory_msgs.msg.JointTrajectory, queue_size=10)


#model service handle
rospy.wait_for_service('/gazebo/get_model_state')
rospy.wait_for_service('drake_ik_service')

ik_handle = rospy.ServiceProxy('drake_ik_service', drake_ik)
model_state_handle = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

#move to position
omni_base.go_abs(x=-0.019396511273080175, y=1.1920592651905912,yaw = 1.57)

#test extracting peach model pose
peach_model_state = model_state_handle(model_name = 'task1_food_ycb_004_sugar_box_8')

# print(type(peach_model_state.pose))
pre_pose = geometry_msgs.msg.PoseStamped(pose = peach_model_state.pose, header = peach_model_state.header)
pre_pose.header.frame_id = '/gazebo_world'
pre_pose.header.stamp = rospy.Time(0)
tf.waitForTransform('/base_link', pre_pose.header.frame_id, rospy.Time(0), rospy.Duration(5.0))
post_pose = tf.transformPose('/base_link', pre_pose)
print(post_pose)
ik_pose =  post_pose.pose

# testing sampled orientation

ik_sol = ik_handle(ik_pose)
if not len(ik_sol.joint_positions) == 1:
    print(ik_sol.joint_positions)

# # wait to establish connection between the controller
# while pub.get_num_connections() == 0:
#     rospy.sleep(0.1)

# # make sure the controller is running
# rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
# list_controllers = (
#     rospy.ServiceProxy('/hsrb/controller_manager/list_controllers',
#                        controller_manager_msgs.srv.ListControllers))
# running = False
# while running is False:
#     rospy.sleep(0.1)
#     for c in list_controllers().controller:
#         if c.name == 'arm_trajectory_controller' and c.state == 'running':
#             running = True

# #init trajectories

# #planning
# def joint_velocities_trajectory_cb(joint_states):
#     # print('traj')
#     traj = trajectory_msgs.msg.JointTrajectory()
#     traj.joint_names = ["arm_lift_joint", "arm_flex_joint",
#                         "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

#     p = trajectory_msgs.msg.JointTrajectoryPoint()
#     p.positions = [joint_states.position[1],joint_states.position[0],joint_states.position[2],joint_states.position[len(joint_states.position)-2], joint_states.position[len(joint_states.position)-1]]
#     p.velocities = [0, 0.1, 0, 0.4, 0]
#     p.time_from_start = rospy.Time(0.1)
#     traj.points = [p]
#     # pub.publish(traj)



# rospy.Subscriber("/hsrb/joint_states", JointState, joint_velocities_trajectory_cb)
# rospy.spin()
