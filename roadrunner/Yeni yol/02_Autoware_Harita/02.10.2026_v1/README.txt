Beytepe Campus - Autoware Map
Source: RoadRunner scene of 02.10.2026

CONTENTS
  lanelet2_map.osm          Lanelet2 HD map: 319 road lanes, 175 walkways, 6 crosswalks
  map_projector_info.yaml   Map projection: local frame, origin 39.871164 N, 32.735689 E
  pointcloud_map.pcd        Point cloud map for NDT localization (13.9 M points)

USAGE
  Pass this folder as map_path, e.g.
    ros2 launch autoware_launch planning_simulator.launch.xml map_path:=<this folder> vehicle_model:=<vehicle> sensor_model:=<sensor kit>
  Use e2e_simulator.launch.xml with AWSIM. Initialise localization with "2D Pose Estimate".

NOTES
  - Speed limit is 30 km/h on all roads.
  - The point cloud is generated from the same 3D scene that AWSIM renders and is intended for simulation.
    Localization on the real vehicle requires a lidar survey of the campus.
  - The source scene contains no traffic lights, stop lines or speed bumps, so the map has none.

KNOWN LIMITATION
  The northern road (about 2.4 km) cannot be reached by route planning in this version: the connection from
  the southern loop into the northern road (around local x=-155, y=248) is missing in the 02.10 source scene.
  It will be restored in the next revision.
