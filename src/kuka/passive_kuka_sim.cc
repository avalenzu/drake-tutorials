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

#include <drake/common/find_resource.h>
#include <drake/common/text_logging_gflags.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/parsers/urdf_parser.h>
#include <drake/multibody/rigid_body_plant/drake_visualizer.h>
#include <drake/multibody/rigid_body_plant/rigid_body_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/constant_vector_source.h>

DEFINE_double(realtime_rate, 1.0,
              "Rate at which to run the simulation, relative to realtime");
DEFINE_double(simulation_time, std::numeric_limits<double>::infinity(),
              "How long to simulate");

///
/// Main function for demo.
///
int main(int argc, char* argv[]) {
  gflags::SetUsageMessage("Simulates the Kuka iiwa arm without any actuation.");
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

  // Add a constant (zero) torque source and connect its output to the input of
  // the plant.
  auto zero_torque_source =
      builder.AddSystem<drake::systems::ConstantVectorSource<double>>(
          drake::VectorX<double>::Zero(plant->get_input_size()));
  builder.Connect(zero_torque_source->get_output_port(),
                  plant->get_input_port(0));

  // Instantiate the diagram.
  std::unique_ptr<drake::systems::Diagram<double>> system{builder.Build()};

  // Instantiate and configure simulator.
  auto simulator = std::make_unique<drake::systems::Simulator<double>>(*system);
  simulator->set_target_realtime_rate(FLAGS_realtime_rate);
  simulator->Initialize();

  // Run simulation.
  simulator->StepTo(FLAGS_simulation_time);
  return 0;
}
