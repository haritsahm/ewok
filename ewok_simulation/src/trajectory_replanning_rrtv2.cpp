/**
 * This file is part of Ewok.
 *
 * Copyright 2017 Vladyslav Usenko, Technical University of Munich.
 * Developed by Vladyslav Usenko <vlad dot usenko at tum dot de>,
 * for more information see <http://vision.in.tum.de/research/robotvision/replanning>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * Ewok is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Ewok is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Ewok. If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>
#include <algorithm>
#include <chrono>
#include <map>
#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <message_filters/subscriber.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Empty.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <Eigen/Core>
#include <eigen3/Eigen/Eigen>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ewok/polynomial_3d_optimization.h>
#include <ewok/rrtstar3d.h>
#include <ewok/uniform_bspline_3d.h>
#include <ewok/uniform_bspline_3d_optimization.h>

#define bool2int (x ? 1 : 0)

const int POW = 6;

double dt;
int num_opt_points;

bool initialized = false;
bool main_debug = false;

std::ofstream f_time, opt_time;

ewok::PolynomialTrajectory3D<10, double>::Ptr traj;
ewok::EuclideanDistanceRingBuffer<POW, int16_t, double>::Ptr edrb;
ewok::RRTStar3D<POW, double>::Ptr path_planner;

ros::Publisher rrt_planner_pub, occ_marker_pub, free_marker_pub, dist_marker_pub, trajectory_pub, current_traj_pub,
    command_pt_pub, command_pt_viz_pub;
tf::TransformListener* listener;

void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  //    ROS_INFO("recieved depth image");

  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  const float fx = 554.254691191187;
  const float fy = 554.254691191187;
  const float cx = 320.5;
  const float cy = 240.5;

  tf::StampedTransform transform;

  try
  {
    listener->lookupTransform("world", msg->header.frame_id, msg->header.stamp, transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_INFO("Couldn't get transform");
    ROS_WARN("%s", ex.what());
    return;
  }

  Eigen::Affine3d dT_w_c;
  tf::transformTFToEigen(transform, dT_w_c);

  Eigen::Affine3f T_w_c = dT_w_c.cast<float>();

  float* data = (float*)cv_ptr->image.data;

  auto t1 = std::chrono::high_resolution_clock::now();

  ewok::EuclideanDistanceRingBuffer<POW, int16_t, double>::PointCloud cloud1;

  for (int u = 0; u < cv_ptr->image.cols; u += 4)
  {
    for (int v = 0; v < cv_ptr->image.rows; v += 4)
    {
      double val = data[v * cv_ptr->image.cols + u];

      // ROS_INFO_STREAM(val);

      if (std::isfinite(val))
      {
        Eigen::Vector4d p;
        p[0] = val * (u - cx) / fx;
        p[1] = val * (v - cy) / fy;
        p[2] = val;
        p[3] = 1;

        p = dT_w_c * p;

        cloud1.push_back(p);
      }
    }
  }

  Eigen::Vector3d origin = (dT_w_c * Eigen::Vector4d(0, 0, 0, 1)).head<3>();

  auto t2 = std::chrono::high_resolution_clock::now();

  if (!initialized)
  {
    Eigen::Vector3i idx;
    edrb->getIdx(origin, idx);

    ROS_INFO_STREAM("Origin: " << origin.transpose() << " idx " << idx.transpose());

    edrb->setOffset(idx);

    initialized = true;
  }
  else
  {
    Eigen::Vector3i origin_idx, offset, diff;
    edrb->getIdx(origin, origin_idx);

    offset = edrb->getVolumeCenter();
    diff = origin_idx - offset;

    while (diff.array().any())
    {
      // ROS_INFO("Moving Volume");
      edrb->moveVolume(diff);

      offset = edrb->getVolumeCenter();
      diff = origin_idx - offset;
    }
  }

  edrb->insertPointCloud(cloud1, origin);

  visualization_msgs::Marker m_occ, m_free;
  edrb->getMarkerOccupied(m_occ);
  edrb->getMarkerFree(m_free);

  occ_marker_pub.publish(m_occ);
  free_marker_pub.publish(m_free);
}

int main(int argc, char** argv)
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
  ros::init(argc, argv, "trajectory_replanning_rrtv2");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string path = ros::package::getPath("ewok_simulation") + "/benchmarking/";

  ROS_INFO_STREAM("path: " << path);

  listener = new tf::TransformListener;

  occ_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/occupied", 5);
  free_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/free", 5);
  dist_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/distance", 5);
  command_pt_viz_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/command_pt", 5);
  trajectory_pub = nh.advertise<geometry_msgs::Point>("command/point", 10);
  current_traj_pub = nh.advertise<visualization_msgs::MarkerArray>("optimal_trajectory", 1, true);
  rrt_planner_pub = nh.advertise<visualization_msgs::MarkerArray>("rrt_trajectory", 1, true);

  ros::Publisher traj_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("global_trajectory", 1, true);
  ros::Publisher traj_checker_pub = nh.advertise<visualization_msgs::Marker>("checker_trajectory", 1, true);

  message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_;
  depth_image_sub_.subscribe(nh, "camera/depth/image_raw", 5);

  tf::MessageFilter<sensor_msgs::Image> tf_filter_(depth_image_sub_, *listener, "world", 5);
  tf_filter_.registerCallback(depthImageCallback);

  double max_velocity, max_acceleration;
  pnh.param("max_velocity", max_velocity, 1.0);
  pnh.param("max_acceleration", max_acceleration, 2.0);

  Eigen::Vector4d limits(max_velocity, max_acceleration, 0, 0);

  double start_x, start_y, start_z, start_yaw;
  pnh.param("start_x", start_x, 0.0);
  pnh.param("start_y", start_y, 0.0);
  pnh.param("start_z", start_z, 0.0);
  pnh.param("start_yaw", start_yaw, 0.0);

  double middle_x, middle_y, middle_z, middle_yaw;
  pnh.param("middle_x", middle_x, 0.0);
  pnh.param("middle_y", middle_y, 0.0);
  pnh.param("middle_z", middle_z, 0.0);
  pnh.param("middle_yaw", middle_yaw, 0.0);

  double stop_x, stop_y, stop_z, stop_yaw;
  pnh.param("stop_x", stop_x, 0.0);
  pnh.param("stop_y", stop_y, 0.0);
  pnh.param("stop_z", stop_z, 0.0);
  pnh.param("stop_yaw", stop_yaw, 0.0);

  pnh.param("dt", dt, 0.5);
  pnh.param("num_opt_points", num_opt_points, 7);

  ROS_INFO("Started hovering example with parameters: start - %f %f %f %f, middle - %f %f %f %f, stop - %f %f %f %f",
           start_x, start_y, start_z, start_yaw, middle_x, middle_y, middle_z, middle_yaw, stop_x, stop_y, stop_z,
           stop_yaw);

  ROS_INFO("dt: %f, num_opt_points: %d", dt, num_opt_points);

  visualization_msgs::Marker trajectory_checker;
  trajectory_checker.lifetime = ros::Duration(0);

  /**
   * Generate Original Spline Trajectory
   */

  ewok::Polynomial3DOptimization<10, double> to(limits * 0.4);

  {
    ewok::Polynomial3DOptimization<10, double>::Vector3Array vec;

    vec.push_back(Eigen::Vector3d(start_x, start_y, start_z));
    vec.push_back(Eigen::Vector3d(middle_x, middle_y, middle_z));
    vec.push_back(Eigen::Vector3d(stop_x, stop_y, stop_z));

    traj = to.computeTrajectory(vec);

    visualization_msgs::MarkerArray traj_marker;
    traj->getVisualizationMarkerArray(traj_marker, "gt", Eigen::Vector3d(1, 0, 1));
    traj_marker_pub.publish(traj_marker);
  }

  double resolution;
  pnh.param("resolution", resolution, 0.15);

  edrb.reset(new ewok::EuclideanDistanceRingBuffer<POW, int16_t, double>(resolution, 1.0));
  // ewok::UniformBSpline3D<6, double> spline_(dt);

  path_planner.reset(new ewok::RRTStar3D<POW, double>(0.25, 1.65, 0.6, 10, dt));
  path_planner->setDistanceBuffer(edrb);
  path_planner->setPolynomialTrajectory(traj);

  for (int i = 0; i < num_opt_points; i++)
  {
    path_planner->addControlPoint(Eigen::Vector3d(start_x, start_y, start_z));
  }

  path_planner->setNumControlPointsOptimized(num_opt_points);
  path_planner->setHeight(Eigen::Vector3d(start_x, start_y, start_z));

  /**
   *
   * Waiting Gazebo Running
   *
   */

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused)
  {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused)
  {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  }
  else
  {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  /**
   * Waiting Gazebo GUI
   *
   */

  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();

  geometry_msgs::Point p;
  p.x = start_x;
  p.y = start_y;
  p.z = start_z;

  ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f, %f].", nh.getNamespace().c_str(), start_x, start_y,
           start_z, start_yaw);

  trajectory_pub.publish(p);

  ros::Duration(5.0).sleep();

  ros::Rate r(1 / dt);
  ros::Time starting = ros::Time::now();
  while (ros::ok())
  {
    r.sleep();
    tf::StampedTransform transform;

    try
    {
      listener->lookupTransform("world", "firefly/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_INFO("Couldn't get transform");
      ROS_WARN("%s", ex.what());
    }

    // Update robot position
    ROS_INFO_COND(main_debug, "Update Robot Pose");
    Eigen::Affine3d base_link;
    tf::transformTFToEigen(transform, base_link);
    path_planner->setRobotPos(base_link.translation());
    path_planner->setRobotPose(base_link);

    // process rrt
    ROS_INFO_COND(main_debug, "Process Trajectory - RRT");
    path_planner->process();

    // get trajectory checker
    ROS_INFO_COND(main_debug, "Publish Trajectory Checker");
    path_planner->TrajectoryChecker(trajectory_checker);
    traj_checker_pub.publish(trajectory_checker);

    // Publish Path Visualizer
    {
      if (path_planner->RRTVisualize())
      {
        ROS_INFO_COND(main_debug, "Publish RRT Visualizer");
        visualization_msgs::MarkerArray rrt_marker;
        rrt_marker.markers.resize(2);
        path_planner->getSolutionMarker(rrt_marker.markers[0], "rrt_trajectory_markers", 0);
        path_planner->getTreeMarker(rrt_marker.markers[1], "rrt_trajectory_markers", 1);
        if (rrt_marker.markers[0].points.size() > 0)
          rrt_planner_pub.publish(rrt_marker);

        path_planner->clearRRT();
      }
    }

    // Publish Command Point
    if ((ros::Time::now() - starting).toSec() > ros::Duration(5).toSec())
    {
      ROS_INFO_COND(main_debug, "Publish Path Visualizer");
      visualization_msgs::MarkerArray sol_traj_marker;
      path_planner->getTrajectoryMarkers(sol_traj_marker, "spline_opitimization_markers", Eigen::Vector3d(0, 1, 0),
                                          Eigen::Vector3d(0, 1, 1), true);
      current_traj_pub.publish(sol_traj_marker);

      ROS_INFO_COND(main_debug, "Publish Command");
      Eigen::Vector3d pc = path_planner->getNextPt();
      geometry_msgs::Point pp;
      pp.x = pc.x();
      pp.y = pc.y();
      pp.z = pc.z();
      trajectory_pub.publish(pp);
    }

    ros::spinOnce();
  }

  ROS_INFO_STREAM("Finished ");

  return 0;
}
