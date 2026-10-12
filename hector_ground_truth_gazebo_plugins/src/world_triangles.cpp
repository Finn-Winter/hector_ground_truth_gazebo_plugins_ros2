// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>
#include <string>
#include <unordered_set>

#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>
#include <gz/common/Util.hh>
#include <gz/common/geospatial/Dem.hh>
#include <gz/common/geospatial/HeightmapData.hh>
#include <gz/common/geospatial/ImageHeightmap.hh>
#include <gz/math/Helpers.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Visual.hh>
#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cone.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Heightmap.hh>
#include <sdf/Mesh.hh>
#include <sdf/Plane.hh>
#include <sdf/Polyline.hh>
#include <sdf/Sphere.hh>

#include <rclcpp/logging.hpp>

#include <hector_ground_truth_gazebo_plugins/world_triangles.hpp>

namespace hector_ground_truth_gazebo_plugins
{
namespace
{

//! Tessellation density of the curved primitives. A sphere becomes SEGMENTS * RINGS quads, i.e.
//! the surface error stays well below a typical map resolution for human-scale objects.
constexpr int kSegments = 64;
constexpr int kRings = 32;

//! Accumulates the world's triangles and the problems encountered while doing so.
struct Collector {
  std::vector<Triangle> triangles;
  //! Mesh/heightmap URIs that could not be resolved or loaded.
  std::unordered_set<std::string> unloadable;
  //! Geometry types that are known to the SDF parser but not handled here.
  std::unordered_set<std::string> unsupported;
};

//! Resolve a mesh/heightmap URI through Gazebo's resource lookup (GZ_SIM_RESOURCE_PATH, the
//! Fuel cache and any callback the running server registered). _file_path is the SDF file the
//! URI was declared in and anchors relative paths.
std::string resolveUri( const std::string &uri, const std::string &file_path )
{
  return gz::common::findFile( gz::sim::asFullPath( uri, file_path ) );
}

//! Invoke f(i0, i1, i2) for every triangle of the submesh, whatever primitive type it uses.
template<typename F>
void forEachTriangle( const gz::common::SubMesh &submesh, const F &f )
{
  const unsigned int count = submesh.IndexCount();
  switch ( submesh.SubMeshPrimitiveType() ) {
  case gz::common::SubMesh::TRIANGLES:
    for ( unsigned int i = 0; i + 2 < count; i += 3 )
      f( submesh.Index( i ), submesh.Index( i + 1 ), submesh.Index( i + 2 ) );
    break;
  case gz::common::SubMesh::TRIFANS:
    for ( unsigned int i = 1; i + 1 < count; ++i )
      f( submesh.Index( 0 ), submesh.Index( i ), submesh.Index( i + 1 ) );
    break;
  case gz::common::SubMesh::TRISTRIPS:
    for ( unsigned int i = 0; i + 2 < count; ++i )
      f( submesh.Index( i ), submesh.Index( i + 1 ), submesh.Index( i + 2 ) );
    break;
  default: // Points and lines have no surface.
    break;
  }
}

void appendMesh( Collector &collector, const Eigen::Isometry3d &transform,
                 const gz::common::Mesh &mesh, const Eigen::Vector3d &scale,
                 const std::string &submesh_name = {}, bool center_submesh = false )
{
  for ( unsigned int s = 0; s < mesh.SubMeshCount(); ++s ) {
    const auto submesh = mesh.SubMeshByIndex( s ).lock();
    if ( !submesh )
      continue;
    if ( !submesh_name.empty() && submesh->Name() != submesh_name )
      continue;
    Eigen::Vector3d offset = Eigen::Vector3d::Zero();
    if ( center_submesh && !submesh_name.empty() ) {
      const gz::math::Vector3d center = ( submesh->Min() + submesh->Max() ) / 2;
      offset = -Eigen::Vector3d( center.X(), center.Y(), center.Z() );
    }
    const auto vertex = [&]( unsigned int index ) {
      const gz::math::Vector3d v = submesh->Vertex( index );
      return transform * ( ( Eigen::Vector3d( v.X(), v.Y(), v.Z() ) + offset ).cwiseProduct( scale ) );
    };
    forEachTriangle( *submesh, [&]( unsigned int i0, unsigned int i1, unsigned int i2 ) {
      collector.triangles.push_back( { vertex( i0 ), vertex( i1 ), vertex( i2 ) } );
    } );
  }
}

//! A tessellated primitive of unit dimensions, created once and reused for every instance.
//! _create is only invoked the first time the name is requested.
template<typename Create>
const gz::common::Mesh *cachedMesh( const std::string &name, const Create &create )
{
  gz::common::MeshManager *manager = gz::common::MeshManager::Instance();
  if ( !manager->HasMesh( name ) )
    create( manager, name );
  return manager->MeshByName( name );
}

//! Meshes are cached under our own names so that the tessellation density is ours, independent
//! of any unit primitives the render engine may have registered with the same manager.
constexpr const char *kMeshPrefix = "hector_ground_truth/";

void appendSphere( Collector &collector, const Eigen::Isometry3d &transform, double radius )
{
  const gz::common::Mesh *mesh =
      cachedMesh( std::string( kMeshPrefix ) + "unit_sphere",
                  []( gz::common::MeshManager *manager, const std::string &name ) {
                    manager->CreateSphere( name, 1.0f, kRings, kSegments );
                  } );
  appendMesh( collector, transform, *mesh, Eigen::Vector3d::Constant( radius ) );
}

void appendEllipsoid( Collector &collector, const Eigen::Isometry3d &transform,
                      const gz::math::Vector3d &radii )
{
  const gz::common::Mesh *mesh =
      cachedMesh( std::string( kMeshPrefix ) + "unit_sphere",
                  []( gz::common::MeshManager *manager, const std::string &name ) {
                    manager->CreateSphere( name, 1.0f, kRings, kSegments );
                  } );
  appendMesh( collector, transform, *mesh, Eigen::Vector3d( radii.X(), radii.Y(), radii.Z() ) );
}

void appendCylinder( Collector &collector, const Eigen::Isometry3d &transform, double radius,
                     double length )
{
  const gz::common::Mesh *mesh =
      cachedMesh( std::string( kMeshPrefix ) + "unit_cylinder",
                  []( gz::common::MeshManager *manager, const std::string &name ) {
                    manager->CreateCylinder( name, 1.0f, 1.0f, kRings, kSegments );
                  } );
  appendMesh( collector, transform, *mesh, Eigen::Vector3d( radius, radius, length ) );
}

void appendCone( Collector &collector, const Eigen::Isometry3d &transform, double radius,
                 double length )
{
  const gz::common::Mesh *mesh =
      cachedMesh( std::string( kMeshPrefix ) + "unit_cone",
                  []( gz::common::MeshManager *manager, const std::string &name ) {
                    manager->CreateCone( name, 1.0f, 1.0f, kRings, kSegments );
                  } );
  appendMesh( collector, transform, *mesh, Eigen::Vector3d( radius, radius, length ) );
}

void appendCapsule( Collector &collector, const Eigen::Isometry3d &transform, double radius,
                    double length )
{
  // The cap radius and the cylinder length scale independently, so a unit capsule cannot be
  // reused; cache one mesh per distinct size instead.
  std::ostringstream name;
  name << kMeshPrefix << "capsule_" << radius << '_' << length;
  const gz::common::Mesh *mesh =
      cachedMesh( name.str(), [&]( gz::common::MeshManager *manager, const std::string &n ) {
        manager->CreateCapsule( n, radius, length, kRings, kSegments );
      } );
  appendMesh( collector, transform, *mesh, Eigen::Vector3d::Ones() );
}

void appendBox( Collector &collector, const Eigen::Isometry3d &transform,
                const gz::math::Vector3d &size )
{
  const double x = size.X() / 2, y = size.Y() / 2, z = size.Z() / 2;
  const Eigen::Vector3d corners[8] = { { -x, -y, -z }, { x, -y, -z }, { x, y, -z }, { -x, y, -z },
                                       { -x, -y, z },  { x, -y, z },  { x, y, z },  { -x, y, z } };
  static constexpr int faces[12][3] = { { 0, 2, 1 }, { 0, 3, 2 },   // bottom
                                        { 4, 5, 6 }, { 4, 6, 7 },   // top
                                        { 0, 1, 5 }, { 0, 5, 4 },   // -y
                                        { 2, 3, 7 }, { 2, 7, 6 },   // +y
                                        { 1, 2, 6 }, { 1, 6, 5 },   // +x
                                        { 3, 0, 4 }, { 3, 4, 7 } }; // -x
  for ( const auto &face : faces )
    collector.triangles.push_back( { transform * corners[face[0]], transform * corners[face[1]],
                                     transform * corners[face[2]] } );
}

void appendPlane( Collector &collector, const Eigen::Isometry3d &transform,
                  const sdf::Plane &plane )
{
  const gz::math::Vector3d n = plane.Normal();
  const Eigen::Vector3d normal = Eigen::Vector3d( n.X(), n.Y(), n.Z() ).normalized();
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
  if ( normal.z() < 1 - 1e-9 ) {
    rotation = normal.z() > -1 + 1e-9
                   ? Eigen::Quaterniond( Eigen::AngleAxisd(
                         std::acos( normal.z() ),
                         Eigen::Vector3d::UnitZ().cross( normal ).normalized() ) )
                   : Eigen::Quaterniond( 0, 1, 0, 0 ); // Upside down, flip about x.
  }
  const Eigen::Isometry3d oriented = transform * rotation;
  const double x = plane.Size().X() / 2, y = plane.Size().Y() / 2;
  const Eigen::Vector3d corners[4] = { { -x, -y, 0 }, { x, -y, 0 }, { x, y, 0 }, { -x, y, 0 } };
  collector.triangles.push_back(
      { oriented * corners[0], oriented * corners[1], oriented * corners[2] } );
  collector.triangles.push_back(
      { oriented * corners[0], oriented * corners[2], oriented * corners[3] } );
}

void appendPolyline( Collector &collector, const Eigen::Isometry3d &transform,
                     const std::vector<sdf::Polyline> &polylines )
{
  if ( polylines.empty() )
    return;
  std::vector<std::vector<gz::math::Vector2d>> contours;
  std::ostringstream name;
  name << kMeshPrefix << "polyline";
  for ( const sdf::Polyline &polyline : polylines ) {
    contours.push_back( polyline.Points() );
    for ( const gz::math::Vector2d &point : polyline.Points() )
      name << '_' << point.X() << ',' << point.Y();
  }
  // All contours of one geometry share the extrusion height, matching Gazebo's rendering.
  const double height = polylines.front().Height();
  name << '_' << height;
  const gz::common::Mesh *mesh =
      cachedMesh( name.str(), [&]( gz::common::MeshManager *manager, const std::string &n ) {
        manager->CreateExtrudedPolyline( n, contours, height );
      } );
  if ( mesh == nullptr )
    return;
  appendMesh( collector, transform, *mesh, Eigen::Vector3d::Ones() );
}

std::shared_ptr<gz::common::HeightmapData> loadHeightmapData( const std::string &path )
{
  static const std::unordered_set<std::string> image_extensions = { "png", "jpg",  "jpeg", "bmp",
                                                                    "pgm", "ppm",  "tga",  "gif",
                                                                    "psd", "hdr" };
  std::string extension = path.substr( path.find_last_of( '.' ) + 1 );
  std::transform( extension.begin(), extension.end(), extension.begin(),
                  []( unsigned char c ) { return std::tolower( c ); } );
  if ( image_extensions.count( extension ) != 0 ) {
    auto image = std::make_shared<gz::common::ImageHeightmap>();
    return image->Load( path ) == 0 ? image : nullptr;
  }
  auto dem = std::make_shared<gz::common::Dem>();
  return dem->Load( path ) == 0 ? dem : nullptr;
}

void appendHeightmap( Collector &collector, const Eigen::Isometry3d &transform,
                      const sdf::Heightmap &heightmap )
{
  const std::string path = resolveUri( heightmap.Uri(), heightmap.FilePath() );
  const std::shared_ptr<gz::common::HeightmapData> data =
      path.empty() ? nullptr : loadHeightmapData( path );
  if ( data == nullptr ) {
    collector.unloadable.insert( heightmap.Uri() );
    return;
  }

  // Sample the source data at its native resolution: Gazebo's <sampling> only subdivides the
  // rendered mesh by linear interpolation, which adds vertices but no additional detail.
  const unsigned int vertices_per_row = data->Width();
  if ( vertices_per_row < 2 )
    return;
  const gz::math::Vector3d size = heightmap.Size();
  // FillHeightMap normalizes the source values to [0, 1] and multiplies by scale.z.
  const gz::math::Vector3d scale( 1, 1,
                                  gz::math::equal( data->MaxElevation(), 0.0f )
                                      ? std::abs( size.Z() )
                                      : std::abs( size.Z() ) / data->MaxElevation() );
  std::vector<float> heights;
  // flipY: the first row of the source image is the +y edge of the terrain, matching the
  // heightmap collision Gazebo's physics engine builds from the same data.
  data->FillHeightMap( 1, vertices_per_row, size, scale, true, heights );
  if ( heights.size() < static_cast<size_t>( vertices_per_row ) * vertices_per_row )
    return;

  const gz::math::Vector3d position = heightmap.Position();
  const double step_x = size.X() / ( vertices_per_row - 1 );
  const double step_y = size.Y() / ( vertices_per_row - 1 );
  const auto vertex = [&]( unsigned int x, unsigned int y ) {
    return transform * Eigen::Vector3d( position.X() - size.X() / 2 + x * step_x,
                                        position.Y() - size.Y() / 2 + y * step_y,
                                        position.Z() + heights[y * vertices_per_row + x] );
  };
  for ( unsigned int y = 0; y + 1 < vertices_per_row; ++y ) {
    for ( unsigned int x = 0; x + 1 < vertices_per_row; ++x ) {
      const Eigen::Vector3d bottom_left = vertex( x, y ), bottom_right = vertex( x + 1, y );
      const Eigen::Vector3d top_left = vertex( x, y + 1 ), top_right = vertex( x + 1, y + 1 );
      collector.triangles.push_back( { bottom_left, bottom_right, top_right } );
      collector.triangles.push_back( { bottom_left, top_right, top_left } );
    }
  }
}

void appendMeshFile( Collector &collector, const Eigen::Isometry3d &transform,
                     const sdf::Mesh &mesh_shape )
{
  const std::string path = resolveUri( mesh_shape.Uri(), mesh_shape.FilePath() );
  const gz::common::Mesh *mesh =
      path.empty() ? nullptr : gz::common::MeshManager::Instance()->Load( path );
  if ( mesh == nullptr ) {
    collector.unloadable.insert( mesh_shape.Uri() );
    return;
  }
  const gz::math::Vector3d s = mesh_shape.Scale();
  appendMesh( collector, transform, *mesh, Eigen::Vector3d( s.X(), s.Y(), s.Z() ),
              mesh_shape.Submesh(), mesh_shape.CenterSubmesh() );
}

void appendGeometry( Collector &collector, const Eigen::Isometry3d &transform,
                     const sdf::Geometry &geometry )
{
  switch ( geometry.Type() ) {
  case sdf::GeometryType::BOX:
    appendBox( collector, transform, geometry.BoxShape()->Size() );
    break;
  case sdf::GeometryType::CAPSULE:
    appendCapsule( collector, transform, geometry.CapsuleShape()->Radius(),
                   geometry.CapsuleShape()->Length() );
    break;
  case sdf::GeometryType::CONE:
    appendCone( collector, transform, geometry.ConeShape()->Radius(),
                geometry.ConeShape()->Length() );
    break;
  case sdf::GeometryType::CYLINDER:
    appendCylinder( collector, transform, geometry.CylinderShape()->Radius(),
                    geometry.CylinderShape()->Length() );
    break;
  case sdf::GeometryType::ELLIPSOID:
    appendEllipsoid( collector, transform, geometry.EllipsoidShape()->Radii() );
    break;
  case sdf::GeometryType::HEIGHTMAP:
    appendHeightmap( collector, transform, *geometry.HeightmapShape() );
    break;
  case sdf::GeometryType::MESH:
    appendMeshFile( collector, transform, *geometry.MeshShape() );
    break;
  case sdf::GeometryType::PLANE:
    appendPlane( collector, transform, *geometry.PlaneShape() );
    break;
  case sdf::GeometryType::POLYLINE:
    appendPolyline( collector, transform, geometry.PolylineShape() );
    break;
  case sdf::GeometryType::SPHERE:
    appendSphere( collector, transform, geometry.SphereShape()->Radius() );
    break;
  case sdf::GeometryType::EMPTY:
    break;
  default:
    collector.unsupported.insert( std::to_string( static_cast<int>( geometry.Type() ) ) );
    break;
  }
}

} // namespace

std::vector<Triangle> collectWorldTriangles( const gz::sim::EntityComponentManager &ecm,
                                             const Eigen::Isometry3d &world_to_target,
                                             const rclcpp::Logger &logger )
{
  Collector collector;
  ecm.Each<gz::sim::components::Visual, gz::sim::components::Geometry>(
      [&]( const gz::sim::Entity &entity, const gz::sim::components::Visual *,
           const gz::sim::components::Geometry *geometry ) -> bool {
        const gz::math::Pose3d pose = gz::sim::worldPose( entity, ecm );
        const Eigen::Isometry3d world_transform =
            Eigen::Translation3d( pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z() ) *
            Eigen::Quaterniond( pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z() );
        appendGeometry( collector, world_to_target * world_transform, geometry->Data() );
        return true;
      } );
  for ( const std::string &uri : collector.unloadable )
    RCLCPP_WARN( logger, "Could not load '%s'; its visuals are missing from the ground truth.",
                 uri.c_str() );
  for ( const std::string &type : collector.unsupported )
    RCLCPP_WARN( logger, "Skipped visuals with unsupported SDF geometry type %s.", type.c_str() );
  return collector.triangles;
}
} // namespace hector_ground_truth_gazebo_plugins
