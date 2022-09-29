/**
* Copyright 2014- Social Robotics Lab, University of Freiburg
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    # Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*    # Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*    # Neither the name of the University of Freiburg nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author Billy Okal <okal@cs.uni-freiburg.de>
*/

#include <pedsim_sensors/obstacle_point_cloud.h>
#include <pedsim_utils/geometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/opencv.hpp>
#include <random>

namespace pedsim_ros {

ObstaclePointCloud::ObstaclePointCloud(const ros::NodeHandle &node_handle,
                                       const double rate, const FoVPtr &fov)
    : PedsimSensor(node_handle, rate, fov) {
  pub_signals_local_ =
      nh_.advertise<sensor_msgs::PointCloud>("point_cloud_local", 1);
  pub_signals_global_ =
      nh_.advertise<sensor_msgs::PointCloud>("point_cloud_global", 1);
  pub_boundary_local_ =
      nh_.advertise<sidewalk_msgs::Boundary>("boundary_local", 1);

  pub_occupancy_grid_ =
      nh_.advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 1);
  pub_local_goal_ =
      nh_.advertise<geometry_msgs::PoseStamped>("local_goal", 1);

  sub_simulated_obstacles_ =
      nh_.subscribe("/pedsim_simulator/simulated_walls", 1,
                    &ObstaclePointCloud::obstaclesCallBack, this);

  sub_simulated_agents_ =
      nh_.subscribe("/pedsim_simulator/simulated_agents", 1,
                    &ObstaclePointCloud::agentStatesCallBack, this);
}

void ObstaclePointCloud::broadcast() {
  if (q_obstacles_.size() < 1 || q_agents_.size() < 1) {
    return;
  }

  const auto sim_obstacles = q_obstacles_.front();
  const auto sim_agents = q_agents_.front();

  using Cell = std::pair<float, float>;
  std::vector<Cell> all_cells;
  for (const auto &line: sim_obstacles->obstacles) {
    const auto cells = pedsim::LineObstacleToCells(line.start.x, line.start.y,
                                                   line.end.x, line.end.y);
    if (std::hypot(line.start.x - line.end.x, line.start.y - line.end.y) < 1) {
      continue;
    }
    std::copy(cells.begin(), cells.end(), std::back_inserter(all_cells));
  }

  constexpr int point_density = 1;
  const int num_points = all_cells.size() * point_density;

  std::default_random_engine generator;

  // \todo - Read params from config file.
  std::uniform_int_distribution<int> color_distribution(1, 255);
  std::uniform_real_distribution<float> height_distribution(0, 1);
  std::uniform_real_distribution<float> width_distribution(-0.5, 0.5);

  sensor_msgs::PointCloud pcd_global;
  pcd_global.header.stamp = ros::Time::now();
  pcd_global.header.frame_id = sim_obstacles->header.frame_id;
  pcd_global.points.resize(num_points);
  pcd_global.channels.resize(1);
  pcd_global.channels[0].name = "intensities";
  pcd_global.channels[0].values.resize(num_points);

  sensor_msgs::PointCloud pcd_local;
  pcd_local.header.stamp = ros::Time::now();
  pcd_local.header.frame_id = "base_footprint";
  pcd_local.points.resize(num_points);
  pcd_local.channels.resize(1);
  pcd_local.channels[0].name = "intensities";
  pcd_local.channels[0].values.resize(num_points);

  sidewalk_msgs::Boundary boundary;
  boundary.header.stamp = ros::Time::now();
  boundary.header.frame_id = "base_footprint";
  boundary.left_boundary.clear();
  boundary.right_boundary.clear();

  // prepare the transform to robot odom frame.
  try {
    transform_listener_->lookupTransform("base_footprint",
                                         robot_odom_.header.frame_id,
                                         ros::Time(0), robot_transform_);
  } catch (tf::TransformException &e) {
    ROS_WARN_STREAM_THROTTLE(5.0, "TFP lookup from ["
        << sim_obstacles->header.frame_id
        << "] to [" << robot_odom_.header.frame_id
        << "] failed. Reason: " << e.what());
    return;
  }

  size_t index = 0;
  for (const auto &cell: all_cells) {
    const int cell_color = color_distribution(generator);

    for (size_t j = 0; j < point_density; ++j) {
      if (fov_->inside(cell.first, cell.second)) {
        const tf::Vector3 point(cell.first,
                                cell.second,
                                0.);
        const auto transformed_point = transformPoint(robot_transform_, point);
        // if (transformed_point.getOrigin().x() < 0) {
        //   continue;
        // }
        if (transformed_point.getOrigin().y() > 0) {
          geometry_msgs::Point p;
          p.x = transformed_point.getOrigin().x();
          p.y = transformed_point.getOrigin().y();
          boundary.left_boundary.push_back(p);
        } else {
          geometry_msgs::Point p;
          p.x = transformed_point.getOrigin().x();
          p.y = transformed_point.getOrigin().y();
          boundary.right_boundary.push_back(p);
        }
        pcd_local.points[index].x = transformed_point.getOrigin().x();
        pcd_local.points[index].y = transformed_point.getOrigin().y();
        pcd_local.points[index].z = 0;
        pcd_local.channels[0].values[index] = 1;

        // Global observations.
        pcd_global.points[index].x = cell.first;
        pcd_global.points[index].y = cell.second;
        pcd_global.points[index].z = 0;
        pcd_global.channels[0].values[index] = 1;
      }

      index++;
    }
  }

  if (pcd_local.channels[0].values.size() > 1) {
    pub_signals_local_.publish(pcd_local);
  }
  if (pcd_global.channels[0].values.size() > 1) {
    pub_signals_global_.publish(pcd_global);
  }

  if (boundary.left_boundary.size() > 1) {
    pub_boundary_local_.publish(boundary);
  }

  // generate occupancy grid
  nav_msgs::OccupancyGrid cost_map;
  build_occupancy_grid(cost_map, sim_agents, boundary);
  pub_occupancy_grid_.publish(cost_map);

  // local goal
  auto left_goal = boundary.left_boundary[boundary.left_boundary.size()-2];
  auto right_goal = boundary.right_boundary[boundary.right_boundary.size()-2];

  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "base_footprint";
  goal.header.stamp = ros::Time::now();
  goal.pose.position.x = (left_goal.x * 0.5 + right_goal.x * 0.5);
  goal.pose.position.y = (left_goal.y * 0.5 + right_goal.y * 0.5);
  auto q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
  goal.pose.orientation = q;
  pub_local_goal_.publish(goal);

  q_obstacles_.pop();
  q_agents_.pop();
};

void ObstaclePointCloud::run() {
  ros::Rate r(rate_);

  while (ros::ok()) {
    broadcast();

    ros::spinOnce();
    r.sleep();
  }
}

void ObstaclePointCloud::obstaclesCallBack(
    const pedsim_msgs::LineObstaclesConstPtr &obstacles) {
  q_obstacles_.emplace(obstacles);
}

void ObstaclePointCloud::agentStatesCallBack(
    const pedsim_msgs::AgentStatesConstPtr &agents) {
  q_agents_.emplace(agents);
}

void ObstaclePointCloud::build_occupancy_grid(nav_msgs::OccupancyGrid &cost_map,
                                              const pedsim_msgs::AgentStatesConstPtr &agents,
                                              const sidewalk_msgs::Boundary &boundary) {

  double width_m = 32;
  double height_m = 32;

  cost_map.header.frame_id = "base_footprint";
  cost_map.header.stamp = ros::Time::now();
  cost_map.info.resolution = 0.05;         // float32
  cost_map.info.width = width_m / cost_map.info.resolution;           // uint32
  cost_map.info.height = height_m / cost_map.info.resolution;           // uint32
  cost_map.info.origin.position.y = -width_m / cost_map.info.resolution * cost_map.info.resolution * 0.5;
  cost_map.info.origin.position.x = -height_m / cost_map.info.resolution * cost_map.info.resolution * 0.5;
  std::vector<signed char> map_data(cost_map.info.width * cost_map.info.height, 1);

  build_boundary(map_data, boundary, cost_map.info.width, cost_map.info.height, 0.05);

  for (const auto &it: agents->agent_states) {
    for (double i = it.pose.position.x - 0.8; i < it.pose.position.x + 0.8; i += 0.05) {
      for (double j = it.pose.position.y - 0.8; j < it.pose.position.y + 0.8; j += 0.05) {
        const tf::Vector3 point(i, j, 0.);
        const auto transformed_point = transformPoint(robot_transform_, point);
        if (std::fabs(transformed_point.getOrigin().x()) < width_m / 2
            && std::fabs(transformed_point.getOrigin().y()) < height_m / 2) {
          auto index = calculate_index(transformed_point.getOrigin().x(),
                                       transformed_point.getOrigin().y(),
                                       cost_map.info.width,
                                       cost_map.info.height,
                                       cost_map.info.resolution);
          if (index < map_data.size() - 1) {
            double x = std::fabs(i - it.pose.position.x);
            double y = std::fabs(j - it.pose.position.y);
            if (x > 0.4 || y > 0.4) {
              map_data[index] = 101;
            } else {
              map_data[index] = 100;
            }
          }
        }
      }
    }
  }

  // for (double i = 12.5; i < 13.5; i += 0.05) {
  //   for (double j = -0.3; j < 0.3; j += 0.05) {
  //     tf::Vector3 point(i, j, 0);
  //     const auto transformed_point = transformPoint(robot_transform_, point);
  //     if (std::fabs(transformed_point.getOrigin().x()) > width_m / 2
  //         || std::fabs(transformed_point.getOrigin().y()) > height_m / 2) {
  //       continue;
  //     }
  //     auto index = calculate_index(transformed_point.getOrigin().x(),
  //                                  transformed_point.getOrigin().y(),
  //                                  cost_map.info.width,
  //                                  cost_map.info.height,
  //                                  cost_map.info.resolution);
  //     if (index < map_data.size() - 1) {
  //       map_data[index] = 100;
  //     }
  //   }
  // }
  cost_map.data = map_data;
}

void ObstaclePointCloud::build_boundary(std::vector<signed char> &map_data,
                                        const sidewalk_msgs::Boundary &boundary,
                                        int width,
                                        int height,
                                        double resolution) {
  cv::Mat src(width, height, CV_8SC1, cv::Scalar(100));

  std::vector<cv::Point> contour,hull;

  double origin_x = -width * resolution * 0.5;
  double origin_y = -height * resolution * 0.5;

  contour.reserve(boundary.left_boundary.size() + boundary.right_boundary.size());
  for (size_t i = 0; i < boundary.left_boundary.size(); ++i) {
    int index_x = std::round((boundary.left_boundary[i].x - origin_x) / resolution);
    int index_y = std::round((boundary.left_boundary[i].y - origin_y) / resolution);
    contour.emplace_back(index_x, index_y);
  }

  for (int i = boundary.right_boundary.size() - 1; i >= 0; --i) {
    int index_x = std::round((boundary.right_boundary[i].x - origin_x) / resolution);
    int index_y = std::round((boundary.right_boundary[i].y - origin_y) / resolution);
    contour.emplace_back(index_x, index_y);
  }

  std::vector<std::vector<cv::Point>> contours;
  // contours.push_back(contour);
  cv::convexHull(contour,hull);
  contours.push_back(hull);
  cv::fillPoly(src, contours, cv::Scalar(0));
  // cv::imshow("test",src);
  // cv::waitKey(1);
  map_data = std::vector<signed char>(src.reshape(1, 1));
}

size_t ObstaclePointCloud::calculate_index(double x, double y, int width, int height, double resolution) {
  double origin_x = -width * resolution * 0.5;
  double origin_y = -height * resolution * 0.5;
  int index = std::round((y - origin_y) / resolution) * width + std::round((x - origin_x) / resolution);
  return index;
}

}  // namespace

// --------------------------------------------------------------

int main(int argc, char **argv) {
  ros::init(argc, argv, "pedsim_obstacle_sensor");
  ros::NodeHandle node("~");

  double init_x = 0.0, init_y = 0.0, fov_range = 0.0;
  node.param<double>("pose_initial_x", init_x, 0.0);
  node.param<double>("pose_initial_y", init_y, 0.0);
  node.param<double>("fov_range", fov_range, 15.);

  pedsim_ros::FoVPtr circle_fov;
  circle_fov.reset(new pedsim_ros::CircularFov(init_x, init_y, fov_range));

  double sensor_rate = 0.0;
  node.param<double>("rate", sensor_rate, 25.0);

  pedsim_ros::ObstaclePointCloud pcd_sensor(node, sensor_rate, circle_fov);
  ROS_INFO_STREAM("Initialized obstacle PCD sensor with center: ("
                      << init_x << ", " << init_y << ") and range: " << fov_range);

  pcd_sensor.run();
  return 0;
}
