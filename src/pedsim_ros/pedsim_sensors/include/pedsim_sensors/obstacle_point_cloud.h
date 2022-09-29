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

#ifndef OBSTACLE_POINT_CLOUD_H
#define OBSTACLE_POINT_CLOUD_H

#include <pedsim_sensors/pedsim_sensor.h>

#include <queue>
#include <nav_msgs/OccupancyGrid.h>
#include <pedsim_msgs/LineObstacles.h>
#include <pedsim_msgs/AgentStates.h>
#include <sidewalk_msgs/Boundary.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

namespace pedsim_ros {

class ObstaclePointCloud : public PedsimSensor {
 public:
  ObstaclePointCloud(const ros::NodeHandle &node_handle, const double rate,
                     const FoVPtr &fov);
  virtual ~ObstaclePointCloud() = default;

  void broadcast() override;
  void run();
  void obstaclesCallBack(const pedsim_msgs::LineObstaclesConstPtr &obstacles);
  void agentStatesCallBack(const pedsim_msgs::AgentStatesConstPtr &agents);

 private:
  ros::Subscriber sub_simulated_obstacles_;
  ros::Subscriber sub_simulated_agents_;

  void build_occupancy_grid(nav_msgs::OccupancyGrid &cost_map,
                            const pedsim_msgs::AgentStatesConstPtr &agents,
                            const sidewalk_msgs::Boundary &boundary);

  void build_boundary(std::vector<signed char> &map_data,
                      const sidewalk_msgs::Boundary &boundary,
                      int width,
                      int height,
                      double resolution);

  size_t calculate_index(double x, double y, int width, int height, double resolution);

  std::queue<pedsim_msgs::LineObstaclesConstPtr> q_obstacles_;
  std::queue<pedsim_msgs::AgentStatesConstPtr> q_agents_;

  tf::StampedTransform robot_transform_;
};

}  // namespace pedsim_ros

#endif
