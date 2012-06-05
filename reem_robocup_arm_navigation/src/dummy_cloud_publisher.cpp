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

// PCL
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

// ROS
#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_cloud_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Point cloud snapshot publisher
  ros::Publisher pointcloud_out_publisher = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_out", 1);

  // Publish a dummy point cloud message to make the environment server happy and allow it to configure
  std::string point_cloud_out_frame;
  private_nh.param("point_cloud_out_frame", point_cloud_out_frame, std::string("/openni_rgb_optical_frame"));
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  PointCloud point_cloud;
  point_cloud.header.frame_id = point_cloud_out_frame;
  for (int i = 0; i < 2; ++i) // NOTE: More than one point to activate octomap
  {
    pcl::PointXYZ pointout;
    pointout.z = 1.4;
    pointout.y = 0.5;
    pointout.x = -0.1 + i*0.02;
    point_cloud.points.push_back(pointout);
  }

  ros::Rate rate(1.0);
  ROS_INFO("Start publishing dummy point cloud.");
  std::string service_name;
  private_nh.param("service_name", service_name, std::string("/environment_server/set_planning_scene_diff"));
  while (ros::ok() && !ros::service::exists(service_name, false))
  {
    // Re-stamp point cloud with current time
    point_cloud.header.stamp = ros::Time::now();

    // Publish pointcloud
    pointcloud_out_publisher.publish(point_cloud);

    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Stop publishing dummy point cloud.");

  return EXIT_SUCCESS;
}
