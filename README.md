# hector_ground_truth_gazebo_plugins
Obtain a ground truth from your gazebo world (modern Gazebo / gz-sim).

## Elevation Map
To obtain a ground truth elevation (alt. height) map, add the `ElevationMapGroundTruthPlugin` system to your world:

```
<sdf version='1.11'>
  <world name='default'>
    <!-- ... -->
    <plugin filename="elevation_map_ground_truth_plugin"
            name="hector_ground_truth_gazebo_plugins::ElevationMapGroundTruthPlugin"/>
  </world>
</sdf>
```

This adds a ROS 2 service `/ground_truth/generate_elevation_map` which computes the ground truth
over a given region. The plugin reads the world's **visual** geometry from the entity component
manager and computes, per cell, the **exact maximum z of the geometry overlapping that cell**
(each triangle is clipped against the cell footprint). Because nothing is sampled, thin vertical
structures such as walls are represented at their true height rather than being missed between
sample points.

All SDF geometry types are covered: box, sphere, cylinder, cone, capsule, ellipsoid, plane,
polyline, heightmap (image and DEM) and mesh files, each with its `<pose>`, `<scale>` and
`<submesh>`. Mesh and heightmap URIs are resolved through Gazebo's resource lookup, which
includes `GZ_SIM_RESOURCE_PATH`, paths relative to the world file and the Fuel cache.

No rendering setup is required and the simulation does not have to be running; the plugin
computes inside the update loop, which Gazebo also drives while paused.

Request fields:

* **origin:** A pose marking the center of the map. The orientation defines the map plane; with
  the identity orientation the map plane is parallel to the X-Y-plane and elevations are
  relative to `origin.z`.
* **keep_origin:** Keep the requested origin in the returned map. If false, the returned map's
  origin is at (0, 0, 0).
* **max_z_height:** Height above the map plane from which the world is looked at. Surfaces above
  it are ignored (e.g. ceilings); a surface crossing it contributes at most `max_z_height`.
* **truncation_distance:** How far below `max_z_height` a surface may lie and still be
  represented. Cells whose highest geometry is below `max_z_height - truncation_distance` (or
  that hold no geometry at all) are unknown.
* **resolution:** Cell edge length in meters.
* **x_size / y_size:** Map extent in meters.
* **samples:** Ignored (kept for compatibility). Cells hold the exact geometry maximum, which
  supersedes ray sampling.
* **quiet:** If false, the resulting map is also published on `ground_truth/elevation_map`.

The response holds the elevation map as a `map_bag_msgs/Map`. If the request is rejected -
non-positive or non-finite sizes, resolution or truncation distance, an invalid orientation, or
a grid that would exceed 100 million cells - `success` is false, `message` explains why and the
map is empty.

#### Example:
```
ros2 service call /ground_truth/generate_elevation_map \
  hector_ground_truth_gazebo_plugins_msgs/srv/GenerateGroundTruth \
  "{origin: {position: {x: -1.4, y: -0.7, z: 0.0}, orientation: {w: 1.0}}, keep_origin: true,
    max_z_height: 2.0, truncation_distance: 4.0, resolution: 0.05,
    x_size: 5.0, y_size: 6.0, quiet: false}"
```

### Known limitations

* A non-identity `origin.orientation` rotates the frame the map is computed in, but
  `map_bag_msgs/Map` has no orientation - the returned map's cells are laid out along the
  rotated axes while the message describes them as axis-aligned. Only use a rotated origin if
  the consumer knows about the rotation.
* Only `<visual>` geometry is considered. Models that carry collision geometry without a visual
  are not part of the ground truth, and robots in the world are.
* The service name, topic name and node name are fixed, so only one instance of the plugin can
  run per ROS graph.

## Map Bag Writer
`map_bag_writer` subscribes to `ground_truth/elevation_map` and writes every map it receives to
its own file:

```
ros2 run hector_ground_truth_gazebo_plugins map_bag_writer \
  --ros-args -p write_path:=/tmp/maps -p file_prefix:=ground_truth
```
