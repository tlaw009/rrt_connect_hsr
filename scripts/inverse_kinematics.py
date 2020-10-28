#!/usr/bin/env python3
import sys
from pydrake.common import FindResourceOrThrow
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.all import ConnectMeshcatVisualizer
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
Parser(plant,_).AddModelFromFile(str(Path.home())+"/ros_drake_ws/src/RRT_connect_hsr/robots/hsrb_pub_release.urdf")
plant.Finalize()

# connect sim, build diagram and draw context
meshcat = ConnectMeshcatVisualizer(builder, _, zmq_url='tcp://127.0.0.1:6000')
diagram = builder.Build()
context = diagram.CreateDefaultContext()

# load sim
meshcat.load()
diagram.Publish(context)

# draw multibody context
plant_context = plant.GetMyMutableContextFromRoot(context)

plant.get_actuation_input_port().FixValue(plant_context, np.zeros(13))

# advance sims
simulator = Simulator(diagram, context)
simulator.set_target_realtime_rate(0.1)
simulator.AdvanceTo(5.0)
