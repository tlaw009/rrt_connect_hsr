#!/usr/bin/env python3
import sys
from pydrake.common import FindResourceOrThrow
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, BasicVector
from pydrake.systems.meshcat_visualizer import ConnectMeshcatVisualizer
from pydrake.math import RollPitchYaw , RotationMatrix, RigidTransform
from pydrake.common.eigen_geometry import Isometry3, Quaternion
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.solvers.mathematicalprogram import Solve
import pydot
import os
import numpy as np
import rospy
from pathlib import Path
from rrt_connect_hsr.srv import drake_fk, drake_fkResponse

rospy.init_node('drake_fk_solver')

# init builder and load multibody
builder = DiagramBuilder()
plant, _ = AddMultibodyPlantSceneGraph(builder, 1e-4)
hsr = Parser(plant,_).AddModelFromFile(str(Path.home())+"/ros_drake_ws/src/rrt_connect_hsr/robots/hsrb4s_arm.obj.urdf")
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_footprint", hsr))
plant.Finalize()
print('multibody plant finalized...')

# connect meshcat, build diagram and draw context
### running local meshcat server REQUIRED ###
meshcat = ConnectMeshcatVisualizer(builder, _, zmq_url='tcp://127.0.0.1:6000')
diagram = builder.Build()
context = diagram.CreateDefaultContext()
print('meshcat visualization server connected...')

# load meshcat and visualize
meshcat.load()
diagram.Publish(context)
print('initial model published...')

# draw multibody context
plant_context = plant.GetMyMutableContextFromRoot(context)
print('multibody context drawn...')

# parse and hash joint with indices
hsr = plant.GetModelInstanceByName('hsrb')
joint_indices = plant.GetJointIndices(hsr)
mutable_joint_index = 0
joint_index_dict = {}
for i in joint_indices:
    if not plant.get_joint(i).type_name() =='weld':
        joint_index_dict.update({plant.get_joint(i).name():mutable_joint_index})
        mutable_joint_index += 1
print('mutable joints extracted from robot: ', joint_index_dict)

# retrieve gripper body
gripper_body = plant.GetBodyByName('hand_palm_link')
print('retrieved gripper_body...')

# set up joint positions and limits
m_joint_lower = plant.GetPositionLowerLimits()
m_joint_upper = plant.GetPositionUpperLimits()
print('lower joint limits: ', m_joint_lower)
print('upper joint limits: ', m_joint_upper)

# service handle
def fk_srv_handle(joint_pos_name_pair):
    # parse joint values
    requested_joint_names = joint_pos_name_pair.joint_names
    requested_joint_positions = joint_pos_name_pair.joint_positions
    # init valid joint positions
    valid_joint_positions = np.zeros(len(requested_joint_positions))
    # check joint positions validity
    for i in range(len(requested_joint_names)):
        jpv = requested_joint_positions[i]
        ji = joint_index_dict[requested_joint_names[i]]
        if  m_joint_lower[ji] < jpv < m_joint_upper[ji]:
            valid_joint_positions[ji] = jpv
        else:
            print('ERROR: invalid joint positions...')
            return drake_fkResponse(np.zeros(1),np.zeros(1))

    # set valid joint positions and solve forward kinematics
    plant.SetPositions(plant_context,valid_joint_positions)
    pose = plant.EvalBodyPoseInWorld(plant_context, gripper_body)
    # visualize state in meshcat
    diagram.Publish(context)
    # construct pose response
    pose_trans = pose.translation()
    pose_quat = np.zeros(4)
    quat = pose.rotation().ToQuaternion()
    pose_quat[0] = quat.x()
    pose_quat[1] = quat.y()
    pose_quat[2] = quat.z()
    pose_quat[3] = quat.w()
    print('returned pose: ', [pose_trans, pose_quat])
    return drake_fkResponse(pose_trans,pose_quat)

# start service server
ik_s = rospy.Service('drake_fk_service', drake_fk, fk_srv_handle)
print('drake fk solver online')
rospy.spin()
