#!/usr/bin/env python
import controller_manager_msgs.srv
import rospy
import trajectory_msgs.msg
import geometry_msgs.msg
import hsrb_interface, rospkg, numpy, yaml, sys
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import *
from rrt_connect_hsr.srv import *
from tf import TransformListener, transformations
import numpy as np

# some init stuff
rospy.init_node('hsrb_rrt_test')
robot = hsrb_interface.robot.Robot()
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')
gripper = robot.get('gripper')

tf = TransformListener()

# initialize joint traj pub
pub = rospy.Publisher('/hsrb/arm_trajectory_controller/command',
                      trajectory_msgs.msg.JointTrajectory, queue_size=10)

# wait for services
rospy.wait_for_service('/gazebo/get_model_state')
rospy.wait_for_service('drake_fk_service')

# init service handles
fk_handle = rospy.ServiceProxy('drake_fk_service', drake_fk)
model_state_handle = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

# # move to position and open gripper
# omni_base.go_abs(x=-0.019396511273080175, y=1.1920592651905912,yaw = 1.57)
whole_body.move_to_go()
gripper.command(0.5)

# test extracting peach model pose
peach_model_state = model_state_handle(model_name = 'task1_food_ycb_015_peach_9')
tf.waitForTransform('/base_link', '/gazebo_world', rospy.Time(0), rospy.Duration(5))
peach_model_state.header.frame_id = '/gazebo_world'
peach_model_state.header.stamp = rospy.Time(0)
peach_pose = tf.transformPose('/base_link', peach_model_state)
print(peach_pose)

# wait to establish connection between the controller
while pub.get_num_connections() == 0:
    rospy.sleep(0.1)

# make sure the controller is running
rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
list_controllers = (
    rospy.ServiceProxy('/hsrb/controller_manager/list_controllers',
                       controller_manager_msgs.srv.ListControllers))
running = False
while running is False:
    rospy.sleep(0.1)
    for c in list_controllers().controller:
        if c.name == 'arm_trajectory_controller' and c.state == 'running':
            running = True




class rrt_node(object):
    """docstring for rrt_node"""
    def __init__(self, joint_names, joint_pos, joint_range, goal_pose, delta, fkin_handle):
        self.parent = None
        self.joint_names = joint_names
        self.joint_pos = joint_pos
        self.joint_range = joint_range
        self.goal_pose = goal_pose
        self.delta = delta
        self.fkin_handle = fkin_handle

    def distance_to_goal(self):
        fk_sol = self.fkin_handle(self.joint_pos, self.joint_names)
        goal_trans = [self.goal_pose.position.x, self.goal_pose.position.y, self.goal_pose.position.z]
        return np.linalg.norm(goal_trans - np.array(fk_sol.translation))

    def goal_test(self):
        if self.distance_to_goal() < 0.05: 
            return True
        else:
            return False

    def Sample(self, n):
        samples = []
        for i in range(n):
            dynamic_delta = self.delta
            bad_child_counter = 0
            while True:
                if bad_child_counter > 9 and dynamic_delta < 0.1:
                    dynamic_delta = dynamic_delta * float((100+bad_child_counter)/100)
                    bad_child_counter = 0
                valid_js = False
                while valid_js == False:
                    child_joint_pos = np.zeros(len(self.joint_names))
                    for j in range(len(self.joint_names)):
                        child_joint_pos[j] = self.joint_pos[j] + (2*np.random.rand() - 1)*dynamic_delta*self.joint_range[j]
                    # print(child_joint_pos)
                    fk_sol = self.fkin_handle(child_joint_pos, self.joint_names)
                    if not (len(fk_sol.translation) == 1 and fk_sol.translation[0] == 0 and len(fk_sol.rotation) == 1 and fk_sol.rotation[0] == 0):
                        valid_js = True
                child = rrt_node(self.joint_names, child_joint_pos, self.joint_range, self.goal_pose, self.delta, self.fkin_handle)
                child.parent = self
            # print(child.cost_plus_estim())
                if child.distance_to_goal() < self.distance_to_goal():
                    break
                else:
                    bad_child_counter += 1

            samples.append(child)
        return samples

    def retrieve_joint_traj(self, duration_delta):
        if self.goal_test():
            current = self
            duration = 0
            traj = trajectory_msgs.msg.JointTrajectory()
            traj.joint_names = current.joint_names
            trajp = []
            while not current == None:
                p = trajectory_msgs.msg.JointTrajectoryPoint()
                p.positions = current.joint_pos
                p.velocities = np.zeros(len(current.joint_names))
                trajp.insert(0, p)
                current = current.parent
            for p in trajp:
                duration += duration_delta
                p.time_from_start = rospy.Duration(duration)
            traj.points = trajp
            return traj
        else:
            return 0

def rrt_planner(joint_names, initial_joint_positions, joint_range, goal_pose, joint_delta, fkin_handle, duration_delta, sample_size):
    current = rrt_node(joint_names,initial_joint_positions, joint_range, goal_pose,joint_delta,fkin_handle)
    if current.distance_to_goal() > 1:
        print('Error: out of reach')
    traj_stat = current.retrieve_joint_traj(duration_delta)
    while traj_stat == 0:
        samples = current.Sample(sample_size)
        local_min = samples[0]
        for n in samples:
            if n.distance_to_goal() < local_min.distance_to_goal():
                local_min = n
        current = local_min
        print(current.distance_to_goal())
        traj_stat = current.retrieve_joint_traj(duration_delta)
    return traj_stat
     

# planning

joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
joint_limits = [0.69, 2.62, 5.93, 3.14, 5.59]
joint_leverage = [3, 3, 2, 1, 2]
joint_range = np.zeros(len(joint_limits))
for i in range(len(joint_limits)):
    joint_range[i] = joint_limits[i]*joint_leverage[i]
init_jp = [0.049997300972243856, -3.5690600109994364e-05, -1.5700074967788669, -1.5712994827285982, 1.9394659714677687e-05]
traj = rrt_planner(joint_names, init_jp, joint_range, peach_pose.pose, 0.05, fk_handle, 2, 20)
print(traj)
pub.publish(traj)
# gripper.apply_force(1.0)

# def joint_velocities_trajectory_cb(joint_states):
#     global planning_started, fk_handle

#     if planning_started == 0:

#         planning_started = 1
#     else:
#         pass


#     # p = trajectory_msgs.msg.JointTrajectoryPoint()
#     # p.positions = [joint_states.position[1],joint_states.position[0],joint_states.position[2],joint_states.position[len(joint_states.position)-2], joint_states.position[len(joint_states.position)-1]]
#     # p.velocities = [0, 0.1, 0, 0.4, 0]
#     # fk_sol = fk_handle(p.positions, traj.joint_names)
#     # tf.waitForTransform('/base_link', '/hand_palm_link', rospy.Time(0), rospy.Duration(5))
#     # print(tf.lookupTransform('/base_link', '/hand_palm_link', rospy.Time(0)))
#     # p.time_from_start = rospy.Time(0.1)
#     # traj.points = [p]
#     # pub.publish(traj)


# rospy.Subscriber("/hsrb/joint_states", JointState, joint_velocities_trajectory_cb)
# rospy.spin()
