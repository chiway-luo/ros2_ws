-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- /* Author: Darby Lim */

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",  -- 地图坐标系
  tracking_frame = "base_footprint", -- 跟踪的坐标系，可以是基坐标系、雷达或imu的坐标系
  -- published_frame = "base_footprint", -- cartographer发布的pose的坐标，一般是机器人基坐标系(一般是base_link或base_fooprint)
  published_frame = "odom", 
  odom_frame = "carto_odom",  -- cartographer 计算后优化的里程计，并非机器人本身里程计
  provide_odom_frame = false, -- 是否发布cartographer的里程计
  publish_frame_projected_to_2d = true, -- 是否转换成2d(无俯仰、滚动的情况下为 true)
  use_odometry = true, -- 是否订阅里程计数据
  use_nav_sat = false, -- 是否订阅GPS
  use_landmarks = false, -- 是否订阅路标
  num_laser_scans = 1, -- 订阅的雷达的数量
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1, -- 将激光雷达的数据拆分成多少部分发布
  num_point_clouds = 0, -- 订阅多线激光雷达的数量
  lookup_transform_timeout_sec = 0.5, -- 坐标变换超时时间
  submap_publish_period_sec = 0.5, -- 发布子图的时间间隔
  pose_publish_period_sec = 5e-3, -- 发布pose的时间间隔
  trajectory_publish_period_sec = 30e-3, -- 发布轨迹的时间间隔
  rangefinder_sampling_ratio = 1., -- 雷达采样比例
  -- odometry_sampling_ratio = 1., -- 里程计采样比例(如果里程计精度低，可以减小该设置值)
  odometry_sampling_ratio = 0.8, -- 里程计采样比例(如果里程计精度低，可以减小该设置值)
  fixed_frame_pose_sampling_ratio = 1., -- 参考坐标系采样比例
  imu_sampling_ratio = 1.,-- imu采样比例
  landmarks_sampling_ratio = 1., -- 路标采样比例
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.15 -- 最小雷达有效距离
TRAJECTORY_BUILDER_2D.max_range = 6.0 -- 最大雷达有效距离
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = false -- 是否使用 imu 数据
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true --  
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- POSE_GRAPH.optimize_every_n_nodes = 0

return options