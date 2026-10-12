// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <array>
#include <cmath>
#include <limits>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <map_bag/map.hpp>
#include <map_bag_msgs/msg/map.hpp>
#include <map_bag_ros/message_conversions/map.hpp>
#include <hector_ground_truth_gazebo_plugins_msgs/srv/generate_ground_truth.hpp>
#include <hector_ground_truth_gazebo_plugins/elevation_map_ground_truth_plugin.hpp>
#include <hector_ground_truth_gazebo_plugins/world_triangles.hpp>

using namespace hector_ground_truth_gazebo_plugins;
using GenerateGroundTruth = hector_ground_truth_gazebo_plugins_msgs::srv::GenerateGroundTruth;
using gz::sim::EntityComponentManager;

namespace
{

//! Refuse requests whose grid would exhaust memory instead of letting the allocation take the
//! simulator down with it. 100 M cells is a 400 MB float grid.
constexpr long kMaxCells = 100L * 1000 * 1000;

//! The z-extent of the part of a triangle that lies inside an axis-aligned xy cell.
struct ZRange {
  double min, max;
};

//! z-extent of the part of the triangle inside the cell, or nullopt if the two are disjoint.
//!
//! Sutherland-Hodgman clip of the triangle against the cell rectangle, carrying z through the
//! edge interpolation. This is exact for vertical faces too (the clipped "polygon" degenerates
//! to a segment but its vertices still carry the right z), which point-sampled rays are not:
//! a 1 cm thick wall slab falls between any practical sample grid.
std::optional<ZRange> clipToCell( const Triangle &triangle, double min_x, double max_x,
                                  double min_y, double max_y )
{
  // Polygon vertices as (x, y, z); clipping interpolates all three components. Clipping a
  // triangle against four half-planes yields at most seven vertices.
  std::array<Eigen::Vector3d, 8> polygon = { triangle.a, triangle.b, triangle.c };
  std::array<Eigen::Vector3d, 8> clipped;
  size_t count = 3;
  // (axis, sign, bound): keep points with sign * (p[axis] - bound) <= 0.
  const struct {
    int axis;
    double sign, bound;
  } planes[4] = { { 0, -1.0, min_x }, { 0, 1.0, max_x }, { 1, -1.0, min_y }, { 1, 1.0, max_y } };
  for ( const auto &plane : planes ) {
    size_t clipped_count = 0;
    for ( size_t i = 0; i < count; ++i ) {
      const Eigen::Vector3d &current = polygon[i];
      const Eigen::Vector3d &next = polygon[( i + 1 ) % count];
      const double current_distance = plane.sign * ( current[plane.axis] - plane.bound );
      const double next_distance = plane.sign * ( next[plane.axis] - plane.bound );
      if ( current_distance <= 0 )
        clipped[clipped_count++] = current;
      if ( ( current_distance < 0 ) != ( next_distance < 0 ) &&
           std::abs( next_distance - current_distance ) > 1e-12 ) {
        const double t = current_distance / ( current_distance - next_distance );
        clipped[clipped_count++] = current + t * ( next - current );
      }
    }
    count = clipped_count;
    if ( count == 0 )
      return std::nullopt;
    polygon = clipped;
  }
  ZRange range{ polygon[0].z(), polygon[0].z() };
  for ( size_t i = 1; i < count; ++i ) {
    range.min = std::min( range.min, polygon[i].z() );
    range.max = std::max( range.max, polygon[i].z() );
  }
  return range;
}

//! Reason why a request cannot be served, or an empty string if it can.
std::string validate( const GenerateGroundTruth::Request &request )
{
  const auto positive = []( double value ) { return std::isfinite( value ) && value > 0; };
  if ( !positive( request.resolution ) )
    return "resolution must be a finite value greater than zero.";
  if ( !positive( request.x_size ) || !positive( request.y_size ) )
    return "x_size and y_size must be finite values greater than zero.";
  if ( !positive( request.truncation_distance ) )
    return "truncation_distance must be a finite value greater than zero, otherwise no surface "
           "can be represented.";
  if ( !std::isfinite( request.max_z_height ) )
    return "max_z_height must be finite.";
  const auto &position = request.origin.position;
  if ( !std::isfinite( position.x ) || !std::isfinite( position.y ) || !std::isfinite( position.z ) )
    return "origin.position must be finite.";
  const auto &orientation = request.origin.orientation;
  const double norm = Eigen::Vector4d( orientation.w, orientation.x, orientation.y, orientation.z )
                          .norm();
  if ( !std::isfinite( norm ) || norm < 1e-6 )
    return "origin.orientation is not a valid quaternion.";
  const double cells_x = std::ceil( request.x_size / request.resolution );
  const double cells_y = std::ceil( request.y_size / request.resolution );
  if ( cells_x * cells_y > kMaxCells )
    return "the requested extent and resolution would produce more than " +
           std::to_string( kMaxCells ) + " cells.";
  return {};
}

} // namespace

ElevationMapGroundTruthPlugin::ElevationMapGroundTruthPlugin() = default;

ElevationMapGroundTruthPlugin::~ElevationMapGroundTruthPlugin() = default;

void ElevationMapGroundTruthPlugin::Configure( const gz::sim::Entity &,
                                               const std::shared_ptr<const sdf::Element> &,
                                               EntityComponentManager &, gz::sim::EventManager & )
{
  if ( !rclcpp::ok() )
    rclcpp::init( 0, nullptr );
  node_ = std::make_shared<rclcpp::Node>( "gazebo_ground_truth_node" );

  using namespace std::placeholders;
  service_server_ = node_->create_service<GenerateGroundTruth>(
      "ground_truth/generate_elevation_map",
      std::bind( &ElevationMapGroundTruthPlugin::GenerateGroundTruthCallback, this, _1, _2 ) );
  publisher_ = node_->create_publisher<map_bag_msgs::msg::Map>( "ground_truth/elevation_map", 10 );

  RCLCPP_INFO( node_->get_logger(), "Plugin was started" );
}

void ElevationMapGroundTruthPlugin::GenerateGroundTruthCallback(
    GenerateGroundTruth::Request::SharedPtr req, GenerateGroundTruth::Response::SharedPtr resp )
{
  const rclcpp::Logger logger = node_->get_logger();
  const auto fail = [&]( const std::string &message ) {
    RCLCPP_ERROR( logger, "%s", message.c_str() );
    resp->success = false;
    resp->message = message;
  };

  RCLCPP_INFO( logger, "Request received" );
  const auto start_time = node_->get_clock()->now();
  if ( ecm_ == nullptr )
    return fail( "No entity component manager available." );
  if ( const std::string error = validate( *req ); !error.empty() )
    return fail( "Invalid request: " + error );

  const double resolution = req->resolution;
  const long cells_x = static_cast<long>( std::ceil( req->x_size / resolution ) );
  const long cells_y = static_cast<long>( std::ceil( req->y_size / resolution ) );
  const double min_elevation = req->max_z_height - req->truncation_distance;
  if ( req->samples > 1 )
    RCLCPP_INFO( logger, "samples is ignored: cells take the exact maximum of the geometry "
                         "overlapping them (supersedes ray sampling)." );

  // Rays run along -z in the request frame: transform the world into it.
  const Eigen::Quaterniond rotation( req->origin.orientation.w, req->origin.orientation.x,
                                     req->origin.orientation.y, req->origin.orientation.z );
  const Eigen::Vector3d origin( req->origin.position.x, req->origin.position.y,
                                req->origin.position.z );
  const Eigen::Isometry3d world_to_request =
      rotation.normalized().conjugate() * Eigen::Translation3d( -origin );

  const std::vector<Triangle> triangles = collectWorldTriangles( *ecm_, world_to_request, logger );
  RCLCPP_INFO( logger, "Rasterizing %zu triangles over %ld x %ld cells", triangles.size(), cells_x,
               cells_y );

  const double half_x = cells_x * resolution / 2.0, half_y = cells_y * resolution / 2.0;
  hector_math::GridMapf elevation_map = map_bag::createEmptyMap( cells_x, cells_y );
  constexpr float progress_step_size = 0.05f;
  float next_progress_step = progress_step_size;

  for ( size_t t = 0; t < triangles.size(); ++t ) {
    if ( static_cast<float>( t ) / triangles.size() >= next_progress_step ) {
      RCLCPP_INFO( logger, "Heightmap Progress: %3.0f%%", next_progress_step * 100 );
      next_progress_step += progress_step_size;
    }
    const Triangle &triangle = triangles[t];
    const double min_x = std::min( { triangle.a.x(), triangle.b.x(), triangle.c.x() } );
    const double max_x = std::max( { triangle.a.x(), triangle.b.x(), triangle.c.x() } );
    const double min_y = std::min( { triangle.a.y(), triangle.b.y(), triangle.c.y() } );
    const double max_y = std::max( { triangle.a.y(), triangle.b.y(), triangle.c.y() } );
    const double min_z = std::min( { triangle.a.z(), triangle.b.z(), triangle.c.z() } );
    const double max_z = std::max( { triangle.a.z(), triangle.b.z(), triangle.c.z() } );
    if ( max_x < -half_x || min_x > half_x || max_y < -half_y || min_y > half_y )
      continue;
    // Below the truncation distance the virtual ray never reached anything, above max_z_height
    // it has not started yet - neither can contribute to a cell.
    if ( max_z < min_elevation || min_z > req->max_z_height )
      continue;
    const long x0 = std::max<long>( 0, std::floor( ( min_x + half_x ) / resolution ) );
    const long x1 = std::min<long>( cells_x - 1, std::floor( ( max_x + half_x ) / resolution ) );
    const long y0 = std::max<long>( 0, std::floor( ( min_y + half_y ) / resolution ) );
    const long y1 = std::min<long>( cells_y - 1, std::floor( ( max_y + half_y ) / resolution ) );
    for ( long x = x0; x <= x1; ++x ) {
      const double cell_min_x = -half_x + x * resolution;
      for ( long y = y0; y <= y1; ++y ) {
        const double cell_min_y = -half_y + y * resolution;
        const std::optional<ZRange> range = clipToCell(
            triangle, cell_min_x, cell_min_x + resolution, cell_min_y, cell_min_y + resolution );
        if ( !range || range->min > req->max_z_height )
          continue;
        // A surface reaching above the ray start is hit at the start, not at its peak.
        const auto top = static_cast<float>( std::min( range->max, req->max_z_height ) );
        if ( top < min_elevation )
          continue;
        // map_bag::Map convention: cell (row, col) center = origin + ((row - (rows-1)/2) * res,
        // (col - (cols-1)/2) * res), i.e. row 0 at minimum x - matches the request frame directly.
        float &cell = elevation_map( x, y );
        if ( map_bag::IsUnknownValue( cell ) || top > cell )
          cell = top;
      }
    }
  }

  const Eigen::Vector3d map_origin = req->keep_origin ? origin : Eigen::Vector3d::Zero();
  auto result = std::make_shared<const map_bag::Map>( elevation_map, map_origin, resolution );

  resp->map = map_bag_ros::message_conversions::mapToMapMsg( result, { "elevation" } );
  resp->map.header.frame_id = map_bag::Map::DEFAULT_FRAME;
  resp->map.header.stamp = node_->get_clock()->now();
  resp->success = true;

  RCLCPP_INFO( logger, "Height map computed in %.2f seconds",
               ( node_->get_clock()->now() - start_time ).seconds() );

  if ( !req->quiet )
    publisher_->publish( resp->map );
}

void ElevationMapGroundTruthPlugin::PostUpdate( const gz::sim::UpdateInfo &,
                                                const EntityComponentManager &_ecm )
{
  if ( !rclcpp::ok() )
    return;
  ecm_ = &_ecm;
  rclcpp::spin_some( node_ );
  ecm_ = nullptr;
}
