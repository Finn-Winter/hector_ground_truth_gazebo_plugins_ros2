// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_GROUND_TRUTH_GAZEBO_PLUGINS_ELEVATION_MAP_GROUND_TRUTH_PLUGIN_HPP
#define HECTOR_GROUND_TRUTH_GAZEBO_PLUGINS_ELEVATION_MAP_GROUND_TRUTH_PLUGIN_HPP

#include <hector_ground_truth_gazebo_plugins_msgs/srv/generate_ground_truth.hpp>
#include <rclcpp/rclcpp.hpp>
#include <map_bag_msgs/msg/map.hpp>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>

namespace hector_ground_truth_gazebo_plugins
{

/*!
 * Serves ground_truth/generate_elevation_map by projecting the world's *visual* geometry
 * straight down onto a grid, one cell holding the highest surface point within it.
 *
 * The geometry is read from the entity component manager and intersected on the CPU, so the
 * result is independent of the render engine and available without a running GUI.
 */
class ElevationMapGroundTruthPlugin :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Service<hector_ground_truth_gazebo_plugins_msgs::srv::GenerateGroundTruth>::SharedPtr service_server_;
  rclcpp::Publisher<map_bag_msgs::msg::Map>::SharedPtr publisher_;
  //! Valid only while PostUpdate spins the node; the service callback runs inside that spin.
  const gz::sim::EntityComponentManager *ecm_ = nullptr;

public:
  ElevationMapGroundTruthPlugin();

  ~ElevationMapGroundTruthPlugin() override;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override;

private:
  void GenerateGroundTruthCallback( hector_ground_truth_gazebo_plugins_msgs::srv::GenerateGroundTruth::Request::SharedPtr req,
                                    hector_ground_truth_gazebo_plugins_msgs::srv::GenerateGroundTruth::Response::SharedPtr resp );
};
}

GZ_ADD_PLUGIN(
  hector_ground_truth_gazebo_plugins::ElevationMapGroundTruthPlugin,
  gz::sim::System,
  hector_ground_truth_gazebo_plugins::ElevationMapGroundTruthPlugin::ISystemConfigure,
  hector_ground_truth_gazebo_plugins::ElevationMapGroundTruthPlugin::ISystemPostUpdate)

#endif
