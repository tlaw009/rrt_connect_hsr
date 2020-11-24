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
from pydrake.common.eigen_geometry import Isometry3
from pydrake.manipulation.planner import DifferentialInverseKinematicsIntegrator, DifferentialInverseKinematicsParameters, DoDifferentialInverseKinematics
from IPython.display import display, SVG
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.solvers.mathematicalprogram import Solve
import pydot
import os
import numpy as np
from pathlib import Path

# class inverse_kin:
# 	"""TODO: put plant and context ino the class"""
# 	def __init__(self, arg):
# 		super(inverse_kin, self).__init__()
# 		self.arg = arg

# init builder and load multibody
builder = DiagramBuilder()
plant, _ = AddMultibodyPlantSceneGraph(builder, 1e-4)
hsr = Parser(plant,_).AddModelFromFile(str(Path.home())+"/ros_drake_ws/src/RRT_connect_hsr/robots/hsrb4s.obj.urdf")
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_footprint", hsr))

plant.Finalize()

# display(SVG(pydot.graph_from_dot_data(plant.GetTopologyGraphvizString())[0].create_svg()))
# connect sim, build diagram and draw context
meshcat = ConnectMeshcatVisualizer(builder, _, zmq_url='tcp://127.0.0.1:6000')
diagram = builder.Build()
context = diagram.CreateDefaultContext()

# load sim
meshcat.load()
diagram.Publish(context)

# draw multibody context
plant_context = plant.GetMyMutableContextFromRoot(context)
q0 = plant.GetPositions(plant_context)
plant.get_actuation_input_port().FixValue(plant_context, np.zeros(13))

gripper = plant.GetBodyByName('hand_palm_link')

pose = plant.EvalBodyPoseInWorld(plant_context, gripper)

print(pose.translation())
print(pose.rotation().matrix())
pt = [pose.translation()[0]+0.2,pose.translation()[1],pose.translation()[2]]

gripper_end_effect_position = pt
gripper_end_effect_rotation = pose.rotation()

ik = InverseKinematics(plant,plant_context)
ik.AddPositionConstraint(gripper.body_frame(), [0,0,0], plant.world_frame(), gripper_end_effect_position, gripper_end_effect_position)
ik.AddOrientationConstraint(gripper.body_frame(), RotationMatrix(), plant.world_frame(), gripper_end_effect_rotation, 0.0)

prog = ik.get_mutable_prog()
q = ik.q()

prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
prog.SetInitialGuess(q, q0)

result = Solve(ik.prog())

if not result.is_success():
	print("IK failed")

print(result.GetSolution())
# IK = DifferentialInverseKinematicsIntegrator(plant, gripper.body_frame(), 0.1, DifferentialInverseKinematicsParameters(len(plant.GetPositions(plant_context)),len(plant.GetVelocities(plant_context))), plant_context, True)
# print(plant.GetPositions(plant_context))
# # IK.SetPositions(plant_context,plant.GetPositions(plant_context))
# pose_1 = IK.ForwardKinematics(plant_context)
# joint_v = DoDifferentialInverseKinematics(plant, plant_context,gripper_end_effect_pose, gripper.body_frame(),DifferentialInverseKinematicsParameters(len(plant.GetPositions(plant_context)),len(plant.GetVelocities(plant_context))))
# print(joint_v.status)
# print(joint_v.joint_velocities)
# # # # advance sims

# simulator = Simulator(diagram, context)
# simulator.set_target_realtime_rate(1)
# simulator.AdvanceTo(5.0)