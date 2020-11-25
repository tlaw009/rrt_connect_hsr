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
from rrt_connect_hsr.srv import drake_ik, drake_ikResponse

rospy.init_node('drake_ik_solver')

# init builder and load multibody
builder = DiagramBuilder()
plant, _ = AddMultibodyPlantSceneGraph(builder, 1e-4)
hsr = Parser(plant,_).AddModelFromFile(str(Path.home())+"/ros_drake_ws/src/rrt_connect_hsr/robots/hsrb4s_arm.obj.urdf")
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_footprint", hsr))
plant.Finalize()

# connect meshcat, build diagram and draw context
### running local meshcat server REQUIRED ###
meshcat = ConnectMeshcatVisualizer(builder, _, zmq_url='tcp://127.0.0.1:6000')
diagram = builder.Build()
context = diagram.CreateDefaultContext()

# load meshcat and visualize
meshcat.load()
diagram.Publish(context)

# draw multibody context, record initial joint positions and initialize joint input values
plant_context = plant.GetMyMutableContextFromRoot(context)
q0 = plant.GetPositions(plant_context)
# plant.get_actuation_input_port().FixValue(plant_context, np.zeros(13))

# retrieve gripper frame
gripper_frame = plant.GetFrameByName('hand_palm_link')

# quaternion normalization
def quat_norm(quat):
    d = np.sqrt(np.square(quat.w) + np.square(quat.x) + np.square(quat.y) + np.square(quat.z))
    if d == 0:
        return 0
    nw = quat.w/d
    nx = quat.x/d
    ny = quat.y/d
    nz = quat.z/d
    return {'w':nw, 'x':nx, 'y':ny, 'z':nz}

# noisy end effect translation helper
# def noisy_end_effect_trans(trans):
#     trans_lower = [trans[0]-0.01*np.random.rand(), trans[1]-0.01*np.random.rand(), trans[2]-0.01*np.random.rand()]
#     trans_upper = [trans[0]+0.01*np.random.rand(), trans[1]+0.01*np.random.rand(), trans[2]+0.01*np.random.rand()]
#     return{'lower': trans_lower, 'upper': trans_upper}

# service handle function
def ik_srv_handle(pose):
    # init pos and rot
    gripper_end_effect_position = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
    print('pose translation: ', gripper_end_effect_position)
    # noisy_gripper_end_effect_position = noisy_end_effect_trans(gripper_end_effect_position)
    # print('pose translation lower bound: ', noisy_gripper_end_effect_position['lower'])
    # print('pose translation upper bound: ', noisy_gripper_end_effect_position['upper'])
    nquat = quat_norm(pose.pose.orientation)
    if nquat == 0:
        return np.zeros(1)
    quat = Quaternion(nquat['w'], nquat['x'], nquat['y'], nquat['z'])
    gripper_end_effect_rotation = RotationMatrix(quat)
    print('pose rotation: ', RollPitchYaw(gripper_end_effect_rotation).vector())
    # init ik program with constraints
    ik = InverseKinematics(plant,plant_context)
    # ik.AddPositionConstraint(gripper_frame, [0,0,0], plant.world_frame(), noisy_gripper_end_effect_position['lower'] , noisy_gripper_end_effect_position['upper'])
    ik.AddPositionConstraint(gripper_frame, [0,0,0], plant.world_frame(), gripper_end_effect_position , gripper_end_effect_position)
    ik.AddOrientationConstraint(gripper_frame, RotationMatrix(), plant.world_frame(), gripper_end_effect_rotation, 0.0)
    # collision constraint to be added later
    # ik.AddMinimumDistanceConstraint(0.001, 0.1)
    prog = ik.get_mutable_prog()
    q = ik.q()
    prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
    prog.SetInitialGuess(q, q0)
    # solve ik
    result = Solve(ik.prog())
    # check result validity
    if not result.is_success():
        print("IK failed: ", result.get_solution_result())
        return np.zeros(1)
    print('joint positions: ', result.GetSolution())
    return drake_ikResponse(result.GetSolution()) 



# start service server
ik_s = rospy.Service('drake_ik_service', drake_ik, ik_srv_handle)
print('drake ik solver online')
rospy.spin()
