/*****************************************************************************
 * Copyright (c) 2017, Massachusetts Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the Massachusetts Institute of Technology nor the
 *   names of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE MASSACHUSETTS INSTITUTE OF TECHNOLOGY AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE MASSACHUSETTS
 * INSTITUTE OF TECHNOLOGY OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include <random>

#include <drake/common/find_resource.h>
#include <drake/common/text_logging_gflags.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/common/trajectories/piecewise_polynomial_trajectory.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/parsers/urdf_parser.h>
#include <drake/multibody/rigid_body_plant/drake_visualizer.h>
#include <drake/multibody/rigid_body_plant/rigid_body_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/controllers/inverse_dynamics_controller.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/multiplexer.h>
#include <drake/systems/primitives/trajectory_source.h>

DEFINE_double(realtime_rate, 1.0,
              "Rate at which to run the simulation, relative to realtime");
DEFINE_double(simulation_time, 2.0, "How long to simulate");

///
/// Main function for demo.
///
int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "Simulates the Kuka iiwa arm tracking a trajectory with an inverse "
      "dynamics controller.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();

  // Instantiate LCM interface and start receiving. This is needed for
  // communicating with drake-visualizer
  auto interface = std::make_unique<drake::lcm::DrakeLcm>();
  interface->StartReceiveThread();

  // Load the Kuka model from a URDF-file
  auto iiwa = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      "/opt/drake/share/drake/drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf",
      drake::multibody::joints::kFixed, iiwa.get());

  // Create the block diagram representing the system
  // The DiagramBuilder class manages the connections between blocks.
  drake::systems::DiagramBuilder<double> builder;

  // Add the Kuka arm to the diagram. The RigidBodyPlant block has one input
  // (actuator torques) and one output (joint state).
  auto plant = builder.AddSystem<drake::systems::RigidBodyPlant<double>>(
      std::move(iiwa));

  // Add a visualizer to the diagram and connect its input to the output of the
  // plant.
  auto visualizer = builder.AddSystem<drake::systems::DrakeVisualizer>(
      plant->get_rigid_body_tree(), interface.get());
  builder.Connect(plant->get_output_port(0), visualizer->get_input_port(0));

  // Add an inverse dynamics controller. The controller will have three inputs
  //  - Current robot state: (q, v)
  //  - Reference robot state: (q*, v*)
  //  - Reference robot acceleration: (vd*)
  // and one output
  //  - Joint torques
  // For more information see
  // http://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1controllers_1_1_inverse_dynamics_controller.html
  const int kNumPositions{plant->get_num_positions()};

  // All the gains are for acceleration, not directly responsible for generating
  // torques. These are set to high values to ensure good tracking. These gains
  // are picked arbitrarily.
  drake::VectorX<double> kp{100 * drake::VectorX<double>::Ones(kNumPositions)};
  drake::VectorX<double> kd{2 * kp.cwiseSqrt()};
  drake::VectorX<double> ki{drake::VectorX<double>::Ones(kNumPositions)};
  auto controller = builder.AddSystem<
      drake::systems::controllers::InverseDynamicsController<double>>(
      plant->get_rigid_body_tree().Clone(), kp, ki, kd,
      true /*has_reference_acceleration*/);

  // Connect the controller's output to the plant's input
  drake::log()->debug("Connecting controller output to plant input");
  builder.Cascade(*controller, *plant);

  // State feedback from the plant to the controller.
  drake::log()->debug(
      "Connecting plant output to controller estimated state input");
  builder.Connect(plant->get_output_port(0),
                  controller->get_input_port_estimated_state());

  // Provide a reference trajectory for the controller to track. The trajectory
  // will be a piecwise-cubic Hermite spline (PCHIP) between a series of random
  // configurations with zero joint
  // velocities at the start and end of the trajectory.
  std::default_random_engine random_generator{1234};
  std::vector<double> t_values;
  std::vector<drake::MatrixX<double>> q_values;
  const int kNumKnots = 5;
  const double dt = FLAGS_simulation_time / (kNumKnots - 1);
  for (int i = 0; i < 5; ++i) {
    t_values.push_back(i * dt);
    q_values.push_back(
        plant->get_rigid_body_tree().getRandomConfiguration(random_generator));
  }

  drake::PiecewisePolynomialTrajectory reference_position_trajectory{
      PiecewisePolynomial<double>::Pchip(t_values, q_values,
                                         true /*zero_end_point_derivatives*/)};

  // This reference trajectory can used in the diagram by means of
  // TrajectorySource blocks.
  auto reference_state_source =
      builder.AddSystem<drake::systems::TrajectorySource<double>>(
          reference_position_trajectory, 1 /*output_derivative_order*/);
  auto reference_acceleration_source =
      builder.AddSystem<drake::systems::TrajectorySource<double>>(
          *reference_position_trajectory.derivative(2));

  // Finally, we can connect the trajectories to the inputs of the controller.
  drake::log()->debug("Connecting desired state trajectory to controller");
  builder.Connect(reference_state_source->get_output_port(),
                  controller->get_input_port_desired_state());
  drake::log()->debug(
      "Connecting desired acceleration trajectory to controller");
  builder.Connect(reference_acceleration_source->get_output_port(),
                  controller->get_input_port_desired_acceleration());

  // Instantiate the diagram.
  std::unique_ptr<drake::systems::Diagram<double>> system{builder.Build()};

  // Instantiate and configure simulator.
  drake::systems::Simulator<double> simulator{*system};
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  for (int index = 0; index < kNumPositions; index++) {
    plant->set_position(simulator.get_mutable_context(), index,
                        q_values.front()(index));
  }
  simulator.Initialize();

  // Run simulation.
  simulator.StepTo(FLAGS_simulation_time);
  return 0;
}
