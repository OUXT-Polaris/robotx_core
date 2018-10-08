include "map_builder.lua"
include "trajectory_builder.lua"
include "pose_graph.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_odometry = false,
  use_laser_scan = true,
  use_multi_echo_laser_scan = false,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
}

TRAJECTORY_BUILDER_2D.scans_per_accumulation = 16

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 3
MAP_BUILDER.sparse_pose_graph.optimization_problem.huber_scale = 5e2
MAP_BUILDER.sparse_pose_graph.optimize_every_n_scans = 320
MAP_BUILDER.sparse_pose_graph.constraint_builder.sampling_ratio = 0.03
MAP_BUILDER.sparse_pose_graph.optimization_problem.ceres_solver_options.max_num_iterations = 10
-- Reuse the coarser 3D voxel filter to speed up the computation of loop closure
-- constraints.
MAP_BUILDER.sparse_pose_graph.constraint_builder.adaptive_voxel_filter = TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter
MAP_BUILDER.sparse_pose_graph.constraint_builder.min_score = 0.62


return options