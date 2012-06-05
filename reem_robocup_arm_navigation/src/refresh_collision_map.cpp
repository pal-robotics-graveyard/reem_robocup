/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2012, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Adolfo Rodriguez Tsouroukdissian */

// C++ standard
#include <string>
#include <iomanip>
#include <cstddef>

// ROS
#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>

/// \brief Refresh the current collision map.
/// This class exposes a ROS service that when called, resets the current collision map and updates it with a snapshot
/// coming from a point cloud source.
/// The following is a list of arguments that can be remapped to configure the ROS API:
///  - \b refresh Name of service that refreshes the collision map.
///  - \b reset Name of service that resets the collision map.
///  - \b point_cloud_in Name of topic acting as point cloud source.
///  - \b point_cloud_out Name of topic containing snapshot of point cloud source.
///
/// Also, the following parameters can be specified:
///  - \b point_cloud_in_timeout Timeout used when waiting for input point cloud.
///
/// \note This class needs to be run in a node with at least two spinning threads.
class CollisionMapRefresher
{
public:
  CollisionMapRefresher()
    : msgs_to_wait_for(3),
      point_cloud_in_timeout(),
      received_msgs_(0),
      point_cloud_in_msg_()
  {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    refresh_collision_map_server_ = private_nh.advertiseService("refresh", &CollisionMapRefresher::refreshCallback, this);
    collision_reset_client_       = nh.serviceClient<std_srvs::Empty>("reset");
    pointcloud_out_publisher_     = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_out", 1);

    private_nh.param("point_cloud_in_timeout", point_cloud_in_timeout, 5.0);

    ROS_INFO("Collision map refresh node ready.");
  }

  std::size_t msgs_to_wait_for;       ///< Number of input point clouds to receive before republishing the \e last one on the output topic
  double      point_cloud_in_timeout; ///< Timeout used when waiting for input point cloud

private:
  std::size_t received_msgs_;
  sensor_msgs::PointCloud2ConstPtr point_cloud_in_msg_;
  ros::ServiceServer refresh_collision_map_server_; ///< Service server for refreshing collision map
  ros::ServiceClient collision_reset_client_;       ///< Service client for resetting collision map
  ros::Publisher     pointcloud_out_publisher_;     ///< Point cloud snapshot publisher

  void pointCloudCallback(sensor_msgs::PointCloud2::ConstPtr point_cloud_in_msg)
  {
    point_cloud_in_msg_ = point_cloud_in_msg;
    ++received_msgs_;
  }

  bool refreshCallback(std_srvs::Empty::Request  &req,
                       std_srvs::Empty::Response &res)
  {
    // Take a point cloud snapshot of the environment and store it. Get some statistics about capture time
    const ros::Time wait_begin = ros::Time::now();

    // Subscribe to point cloud stream and wait for messages to start flowing.
    // For unknown reasons, this seems to be required for an ASUS Xtion Pro live, as the first published message may
    // contain garbage
    received_msgs_ = 0;
    ros::Subscriber pointcloud_in_subscriber =
    ros::NodeHandle().subscribe("point_cloud_in", 1, &CollisionMapRefresher::pointCloudCallback, this);

    while (received_msgs_ < msgs_to_wait_for)
    {
      if ((ros::Time::now() - wait_begin).toSec() > point_cloud_in_timeout)
      {
        ROS_ERROR_STREAM("Failed to refresh collision map. Timed out (" <<
                        std::fixed << std::setprecision(2) << point_cloud_in_timeout <<
                        "s) waiting for input point cloud on topic \"" << pointcloud_in_subscriber.getTopic() << "\".");
        return false;
      }
      ros::Duration(0.1).sleep(); // NOTE: Magic value
    }
    assert(point_cloud_in_msg_ && "Input point cloud pointer is null."); // Pointcloud capture postcondition
    pointcloud_in_subscriber.shutdown();

    const double wait_duration = (ros::Time::now() - wait_begin).toSec();
    const double data_age      = (ros::Time::now() - point_cloud_in_msg_->header.stamp).toSec();

    ROS_DEBUG_STREAM("Input point cloud received with data that is " <<
                    std::fixed << std::setprecision(2) << data_age << "s old. Waited " <<
                    wait_duration << "s for data to become available.");

    // Reset collision map
    std_srvs::Empty empty_req;
    if (collision_reset_client_.call(empty_req))
    {
      ROS_DEBUG("Collision map was reset successfully.");
    }
    else
    {
      ROS_ERROR("Failed to reset collision map.");
      return false;
    }

    // Publish pointcloud to output topic
    pointcloud_out_publisher_.publish(point_cloud_in_msg_);

    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "refresh_collision_map");

  CollisionMapRefresher refresher;
  refresher.msgs_to_wait_for = 3;         // NOTE: Magic values
  refresher.point_cloud_in_timeout = 5.0;

  ros::AsyncSpinner spinner(2); // Two threads are needed by CollisionMapRefresher
  spinner.start();
  ros::waitForShutdown();

  return EXIT_SUCCESS;
}

