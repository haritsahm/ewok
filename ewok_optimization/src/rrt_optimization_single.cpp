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

#include <chrono>
#include <math.h>
#include <algorithm>
#include <chrono>
#include <ctime>   // localtime
#include <sstream> // stringstream
#include <iomanip> // put_time
#include <string>  // string
#include <map>
#include <thread>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/time.h>
#include <eigen3/Eigen/Eigen>
#include <Eigen/Core>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ewok/polynomial_3d_optimization.h>
#include <ewok/polynomial_trajectory_3d.h>
#include <ewok/rrtstar3d.h>
#include <ewok/uniform_bspline_3d.h>
#include <ewok/uniform_bspline_3d_optimization.h>

const int POW = 6;

double dt;
bool initialized = false;
bool main_debug = false;

ewok::PolynomialTrajectory3D<10, double>::Ptr traj;
ewok::EuclideanDistanceRingBuffer<POW, int16_t, double>::Ptr edrb;
ewok::RRTStar3D<POW, double>::Ptr path_planner;

ros::Subscriber robot_pos_subscriber;
ros::Publisher rrt_property_pub, rrt_tree_pub, rrt_solution_pub, occ_marker_pub, free_marker_pub, dist_marker_pub, trajectory_pub, current_traj_pub,
    command_pt_pub, command_pt_viz_pub;
ros::Publisher traj_marker_pub, traj_checker_pub;

void RRTPublisher(const ros::TimerEvent& event)
{
  if(path_planner->isRunning())
  {
    ROS_INFO_COND(main_debug, "Publish RRT Visualizer");
    visualization_msgs::Marker rrt_tree_marker, rrt_solution_marker;
    visualization_msgs::MarkerArray rrt_property_marker;
    path_planner->getTreeMarker(rrt_tree_marker, "rrt_tree_marker", 1);
    if(path_planner->RRTVisualize())
    {
      path_planner->getSolutionMarker(rrt_solution_marker, "rrt_solution_markers", 0, Eigen::Vector3f(0,0,1), 0.05);
      if(rrt_solution_marker.points.size() > 0)
        rrt_solution_pub.publish(rrt_solution_marker);
    }

    if (rrt_tree_marker.points.size() > 0)
      rrt_tree_pub.publish(rrt_tree_marker);

    rrt_property_marker.markers.resize(2);
    if(path_planner->solutionFound())
    {
      path_planner->getRRTProperty(rrt_property_marker.markers[0], "rrt_property_marker", 0, Eigen::Vector4f(1,1,1,0.4));
    }

    path_planner->getRRTProperty(rrt_property_marker.markers[1], "rrt_state_marker", 1, Eigen::Vector4f(1,0.5,0,0.6));
    rrt_property_pub.publish(rrt_property_marker);

  }
}

void RRTProcess(const ros::TimerEvent& event)
{
  visualization_msgs::Marker trajectory_checker;
  trajectory_checker.lifetime = ros::Duration(0);

  // process rrt
  ROS_INFO_COND(main_debug, "Process Trajectory - RRT");
  path_planner->process_TEST();

  // get trajectory checker
  ROS_INFO_COND(main_debug, "Publish Trajectory Checker");
  path_planner->TrajectoryChecker(trajectory_checker);
  traj_checker_pub.publish(trajectory_checker);

  // Publish Command Point

    ROS_INFO_COND(main_debug, "Publish Path Visualizer");
    visualization_msgs::MarkerArray sol_traj_marker;
    path_planner->getTrajectoryMarkers(sol_traj_marker, "spline_opitimization_markers", Eigen::Vector3d(0, 1, 0),
                                        Eigen::Vector3d(0, 1, 1));
    current_traj_pub.publish(sol_traj_marker);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "rrt_optimization_single");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  occ_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/occupied", 5, true);
  free_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/free", 5, true);
  dist_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/distance", 5, true);
  current_traj_pub = nh.advertise<visualization_msgs::MarkerArray>("/trajectory/optimal_trajectory", 1, true);
  rrt_tree_pub = nh.advertise<visualization_msgs::Marker>("/trajectory/rrt_tree", 1, true);
  rrt_solution_pub = nh.advertise<visualization_msgs::Marker>("/trajectory/rrt_solution", 1, true);
  traj_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/trajectory/global_trajectory", 1, true);
  traj_checker_pub = nh.advertise<visualization_msgs::Marker>("/trajectory/checker_trajectory", 1, true);
  rrt_property_pub = nh.advertise<visualization_msgs::MarkerArray>("/trajectory/rrt_property", 1, true);

  visualization_msgs::Marker trajectory_checker;
  trajectory_checker.lifetime = ros::Duration(0);

//  // Set up global trajectory
  const Eigen::Vector4d limits(0.7, 4, 0, 0);

  ewok::Polynomial3DOptimization<10, double> to(limits*0.8);

  ewok::Polynomial3DOptimization<10, double>::Vector3Array vec;
  vec.push_back(Eigen::Vector3d(-5,-5, 1));
  vec.push_back(Eigen::Vector3d(5, -2.5, 1));
  vec.push_back(Eigen::Vector3d(-5, 2.5, 1));
  vec.push_back(Eigen::Vector3d( 5, 5, 1));

  traj = to.computeTrajectory(vec);

  visualization_msgs::MarkerArray traj_marker;
  traj->getVisualizationMarkerArray(traj_marker, "trajectory", Eigen::Vector3d(1, 0, 0));
  traj_marker_pub.publish(traj_marker);

//  // Set up collision buffer
  edrb.reset(new ewok::EuclideanDistanceRingBuffer<POW, int16_t, double>(0.15, 1.0));
  ewok::EuclideanDistanceRingBuffer<POW, int16_t, double>::PointCloud cloud;

  for(double z = -2; z < 2; z += 0.05) {
    cloud.push_back(Eigen::Vector4d(0, 0.1, z, 0));
  }

  edrb->insertPointCloud(cloud, Eigen::Vector3d(0,0,0));
  edrb->insertPointCloud(cloud, Eigen::Vector3d(0,0,0));

  visualization_msgs::Marker m_occ, m_free, m_dist;
  edrb->getMarkerOccupied(m_occ);
  edrb->getMarkerFree(m_free);

  occ_marker_pub.publish(m_occ);
  free_marker_pub.publish(m_free);

  dt = 0.5;
  double step_size;
  int num_iter;
  bool save_log;
  pnh.param("step_size", step_size, 0.25);
  pnh.param("save_log", save_log, false);
  pnh.param("num_iter", num_iter, 1000);


  std::string path = ros::package::getPath("ewok_optimization") + "/logs/";
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "log-%Y-%m-%d-%X");
  std::string file_name = ss.str();
  ROS_INFO_STREAM("Writing log file " << path+file_name);


  path_planner.reset(new ewok::RRTStar3D<POW, double>(step_size, 1.15, 0.6, 4, dt, num_iter));
  path_planner->setDistanceBuffer(edrb);
  path_planner->setPolynomialTrajectory(traj);

  Eigen::Vector3d start_point(-5, -5, 1), end_point(5, 5, 1);

  for (int i = 0; i < 7; i++) {
    path_planner->addControlPoint(start_point);
  }

  bool flat_height;
  pnh.param("flat_height", flat_height, true);
  std::cout << "Flat Height " << flat_height << std::endl;

  path_planner->setLogPath(path+file_name, save_log); //save log
  path_planner->setHeight(start_point, flat_height);


  ROS_INFO("Finished setting up data");

  ros::Rate r(1/dt);

  ros::Timer timer = nh.createTimer(ros::Duration(0.002), RRTPublisher);
  ros::Timer timer2 = nh.createTimer(ros::Duration(dt), RRTProcess);

  ROS_INFO("Starting Proscess");

  ros::spin();

  ROS_INFO_STREAM("Finished RRT Test");

  return 0;
}
