// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_GROUND_TRUTH_GAZEBO_PLUGINS_WORLD_TRIANGLES_HPP
#define HECTOR_GROUND_TRUTH_GAZEBO_PLUGINS_WORLD_TRIANGLES_HPP

#include <vector>

#include <Eigen/Geometry>
#include <gz/sim/EntityComponentManager.hh>
#include <rclcpp/logger.hpp>

namespace hector_ground_truth_gazebo_plugins
{

//! A single world triangle, already transformed into the target frame.
struct Triangle {
  Eigen::Vector3d a, b, c;
};

/*!
 * Tessellates the visual geometry of every entity in the world and returns it as triangles
 * expressed in the target frame.
 *
 * Covers every SDF geometry type: box, sphere, cylinder, cone, capsule, ellipsoid, plane,
 * polyline, heightmap (image and DEM) and mesh files. Mesh and heightmap URIs are resolved
 * through Gazebo's resource lookup, which includes GZ_SIM_RESOURCE_PATH and the Fuel cache.
 *
 * \param world_to_target Transform applied to every vertex, mapping world coordinates into the
 *   frame the triangles are requested in.
 */
std::vector<Triangle> collectWorldTriangles( const gz::sim::EntityComponentManager &ecm,
                                             const Eigen::Isometry3d &world_to_target,
                                             const rclcpp::Logger &logger );
} // namespace hector_ground_truth_gazebo_plugins

#endif // HECTOR_GROUND_TRUTH_GAZEBO_PLUGINS_WORLD_TRIANGLES_HPP
