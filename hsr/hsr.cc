#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {

using Eigen::Vector3d;
using Eigen::VectorXd;

using geometry::SceneGraph;
using multibody::Frame;
using multibody::LinearBushingRollPitchYaw;
using multibody::MultibodyPlant;
using multibody::Parser;
using multibody::RevoluteJoint;
using math::RigidTransformd;
using systems::Context;
using systems::DiagramBuilder;
using systems::Simulator;

namespace examples {
namespace multibody {
namespace hsr {
namespace {

DEFINE_double(simulation_time, 10.0, "Duration of the simulation in seconds.");

int do_main() {
  // Build a generic MultibodyPlant and SceneGraph.
  DiagramBuilder<double> builder;

  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(
      &builder, std::make_unique<MultibodyPlant<double>>(0.001));

  // Make and add the hsr model from an URDF model.
  const std::string relative_name =
      "drake/examples/hsr/models/hsrb4s.obj.urdf";
  const std::string full_name = FindResourceOrThrow(relative_name);

  Parser parser(&plant);
  parser.AddModelFromFile(full_name);

  plant.WeldFrames(plant.world_frame(),
                   plant.GetBodyByName("base_footprint").body_frame(),
                   RigidTransformd::Identity());

  // We are done defining the model. Finalize and build the diagram.
  plant.Finalize();

  ConnectDrakeVisualizer(&builder, scene_graph);
  auto diagram = builder.Build();

  // Create a context for this system and sub-context for the hsr system.
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  Context<double>& plant_context =
      plant.GetMyMutableContextFromRoot(diagram_context.get());

  plant.get_actuation_input_port().FixValue(&plant_context,
                                            VectorXd::Zero(13));

  // Create a simulator and run the simulation
  std::unique_ptr<Simulator<double>> simulator =
      MakeSimulatorFromGflags(*diagram, std::move(diagram_context));

  simulator->AdvanceTo(FLAGS_simulation_time);

  // Print some useful statistics
  PrintSimulatorStatistics(*simulator);

  return 0;
}

}  // namespace
}  // namespace hsr
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::hsr::do_main();
}
