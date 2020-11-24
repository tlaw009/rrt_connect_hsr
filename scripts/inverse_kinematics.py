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
from drake_ik.srv import pose, joint_positions

rospy.init_node('drake_ik_solver')
# init builder and load multibody

builder = DiagramBuilder()
plant, _ = AddMultibodyPlantSceneGraph(builder, 1e-4)
hsr = Parser(plant,_).AddModelFromFile(str(Path.home())+"/ros_drake_ws/src/RRT_connect_hsr/robots/hsrb4s.obj.urdf")
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
plant.get_actuation_input_port().FixValue(plant_context, np.zeros(13))

# retrieve gripper body and frame
gripper = plant.GetBodyByName('hand_palm_link')
gripper_frame = plant.GetFrameByName('hand_palm_link')

# testing suite 1
"""
pose = plant.EvalBodyPoseInWorld(plant_context, gripper)

print(pose.translation())
print(pose.rotation().matrix())
pt = [pose.translation()[0]+0.2,pose.translation()[1],pose.translation()[2]]
"""
# service handle function
def ik_srv_handle(pose):
    #init pos and rot
    gripper_end_effect_position = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
    quat = Quaternion(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z)
    gripper_end_effect_rotation = RotationMatrix(quat)
    # init ik program with constraints
    ik = InverseKinematics(plant,plant_context)
    ik.AddPositionConstraint(gripper.body_frame(), [0,0,0], plant.world_frame(), gripper_end_effect_position, gripper_end_effect_position)
    ik.AddOrientationConstraint(gripper.body_frame(), RotationMatrix(), plant.world_frame(), gripper_end_effect_rotation, 0.0)
    prog = ik.get_mutable_prog()
    q = ik.q()
    prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
    prog.SetInitialGuess(q, q0)
    #solve ik
    result = Solve(ik.prog())
    #check result validity
    if not result.is_success():
        print("IK failed")
        return np.zeros(1)
    
    return result.GetSolution()




ik_s = rospy.Service('drake_ik_service', pose, ik_srv_handle)
print('drake ik solver online')
rospy.spin()



# IK = DifferentialInverseKinematicsIntegrator(plant, gripper.body_frame(), 0.1, DifferentialInverseKinematicsParameters(len(plant.GetPositions(plant_context)),len(plant.GetVelocities(plant_context))), plant_context, True)
# print(plant.GetPositions(plant_context))
# # IK.SetPositions(plant_context,plant.GetPositions(plant_context))
# pose_1 = IK.ForwardKinematics(plant_context)
# joint_v = DoDifferentialInverseKinematics(plant, plant_context,gripper_end_effect_pose, gripper.body_frame(),DifferentialInverseKinematicsParameters(len(plant.GetPositions(plant_context)),len(plant.GetVelocities(plant_context))))
# print(joint_v.status)
# print(joint_v.joint_velocities)


# advance sims

# simulator = Simulator(diagram, context)
# simulator.set_target_realtime_rate(1)
# simulator.AdvanceTo(5.0)

