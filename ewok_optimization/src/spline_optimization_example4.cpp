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

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <ewok/uniform_bspline_3d_optimization.h>
#include <ewok/polynomial_3d_optimization.h>
#include <ewok/uniform_bspline_3d.h>

void getMarkers(visualization_msgs::Marker & traj_marker, const std::string & ns,
                int id, const Eigen::Vector3d & color,
                const ros::Duration & lifetime = ros::Duration(0), double scale = 0.1)
{

}

int main(int argc, char **argv) {

  ros::init(argc, argv, "spline_optimization_example");
  ros::NodeHandle nh;

  ROS_INFO("Started spline_optimization_example");

  ros::Publisher global_traj_pub = nh.advertise<visualization_msgs::MarkerArray>(
      "global_trajectory",
      1,
      true);

  ros::Publisher spline_test_pub = nh.advertise<visualization_msgs::Marker>(
      "spline_test",
      1,
      true);

  ros::Publisher current_traj_pub = nh.advertise<visualization_msgs::MarkerArray>
          ("optimal_trajectory", 1, true);


  ros::Publisher before_opt_pub = nh.advertise<visualization_msgs::MarkerArray>(
      "before_optimization",
      1,
      true);
  ros::Publisher after_opt_pub = nh.advertise<visualization_msgs::MarkerArray>(
      "after_optimization",
      1,
      true);

  ros::Publisher occ_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/occupied", 1, true);
  ros::Publisher free_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/free", 1, true);
  ros::Publisher dist_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/distance", 1, true);

  // Set up global trajectory
  const Eigen::Vector4d limits(0.7, 4, 0, 0);

  ewok::Polynomial3DOptimization<10> po(limits*0.8);

  typename ewok::Polynomial3DOptimization<10>::Vector3Array vec;
  vec.push_back(Eigen::Vector3d(-5,-5, 1));
  vec.push_back(Eigen::Vector3d(5, -2.5, 1));
  vec.push_back(Eigen::Vector3d(-5, 2.5, 1));
  vec.push_back(Eigen::Vector3d( 5, 5, 1));

  auto traj = po.computeTrajectory(vec);

  visualization_msgs::MarkerArray traj_marker;
  traj->getVisualizationMarkerArray(traj_marker, "trajectory", Eigen::Vector3d(1, 0, 0));

  global_traj_pub.publish(traj_marker);

  // Set up collision buffer
  ewok::EuclideanDistanceRingBuffer<6>::Ptr edrb(new ewok::EuclideanDistanceRingBuffer<6>(0.15, 1));
  ewok::EuclideanDistanceRingBuffer<6>::PointCloud cloud;

  for(float z = -2; z < 2; z += 0.05) {
    cloud.push_back(Eigen::Vector4f(0, 0.1, z, 0));
  }

  edrb->insertPointCloud(cloud, Eigen::Vector3f(0,0,0));
  edrb->insertPointCloud(cloud, Eigen::Vector3f(0,0,0));

  edrb->updateDistance();

  visualization_msgs::Marker m_occ, m_free, m_dist;
  edrb->getMarkerOccupied(m_occ);
  edrb->getMarkerFree(m_free);
  edrb->getMarkerDistance(m_dist, 0.5);

  occ_marker_pub.publish(m_occ);
  free_marker_pub.publish(m_free);
  dist_marker_pub.publish(m_dist);

  // Set up spline optimization
  const int num_points = 7;
  const double dt = 0.5;

//  Eigen::Vector3d start_point(-5, -5, 0), end_point(5, 5, 0);
//  ewok::UniformBSpline3DOptimization<6> spline_opt(traj, dt);

//  for (int i = 0; i < num_points; i++) {
//    spline_opt.addControlPoint(vec[0]);
//  }

//  spline_opt.setNumControlPointsOptimized(num_points);
//  spline_opt.setDistanceBuffer(edrb);
//  spline_opt.setLimits(limits);


//  double tc = spline_opt.getClosestTrajectoryTime(Eigen::Vector3d(-3, -5, 1), 2.0);
//  ROS_INFO_STREAM("Closest time: " << tc);

  ROS_INFO("Finished setting up data");

  double current_time = 0;

  double total_opt_time = 0;
  int num_iterations = 0;

  visualization_msgs::MarkerArray before_opt_markers, after_opt_marker;
  visualization_msgs::Marker ctrl_pts_marker;

  ctrl_pts_marker.header.frame_id = "world";
  ctrl_pts_marker.ns = "spline loop test";
  ctrl_pts_marker.id = 0;
  ctrl_pts_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  ctrl_pts_marker.action = visualization_msgs::Marker::MODIFY;
  ctrl_pts_marker.scale.x = 0.1;
  ctrl_pts_marker.scale.y = 0.1;
  ctrl_pts_marker.scale.z = 0.1;
  ctrl_pts_marker.color.a = 1.0;

  ctrl_pts_marker.lifetime = ros::Duration(0);

  std_msgs::ColorRGBA c0, c1;

  c0.r = 1.0;
  c0.g = 0.5;
  c0.b = 1.0;
  c0.a = 1.0;

  c1.r = 1.0;
  c1.g = 1.0;
  c1.b = 0.0;
  c1.a = 1.0;

  std::vector<Eigen::Vector3d> ctrl_points_;
  int ctrl_points_idx = 0;
  ewok::UniformBSpline3D<6> spline_(dt);


  // Make sure initial position is static at starting point
  for (int i = 0; i < 6; i++) {
    spline_.push_back(Eigen::Vector3d(-5,-5, 1));
  }


  bool ctrl_points_ack=false;
  ros::Rate r(1/dt);

//  ROS_INFO("Rate " << r.cycleTime().toSec());

  while (ros::ok()) {

    r.sleep();

    current_time += dt;
    bool is_near = false;

    if(current_time < traj->duration())
    {
        auto ctrl_points = traj->evaluate(current_time, 0);
        ctrl_points_.push_back(ctrl_points.template cast<double>());

        is_near = edrb->isNearObstacle(ctrl_points.template cast<float>(), 1.2);

        geometry_msgs::Point p;
        p.x = ctrl_points(0);
        p.y = ctrl_points(1);
        p.z = ctrl_points(2);

        if(is_near)
        {
            std::cout << "found near" << std::endl;
            ctrl_pts_marker.colors.push_back(c0);
            ctrl_pts_marker.points.push_back(p);
        }

        else {
            ctrl_pts_marker.colors.push_back(c1);
            ctrl_pts_marker.points.push_back(p);
        }
    }


    spline_test_pub.publish(ctrl_pts_marker);

    if(current_time > ros::Duration(5).toSec())
    {
        spline_.push_back(ctrl_points_[ctrl_points_idx]);
        ctrl_points_idx++;

        if (ctrl_points_idx == ctrl_points_.size()-1)
        {
            std::cout << ctrl_points_idx << ", " << ctrl_points_.size() << std::endl;
            ctrl_points_ack=true;
        }

    }

    visualization_msgs::MarkerArray traj_marker;
    traj_marker.markers.resize(2);

    spline_.getVisualizationMarker(traj_marker.markers[0], "spline_opitimization_markers", 0,  Eigen::Vector3d(0,1,0));
    spline_.getControlPointsMarker(traj_marker.markers[1], "spline_opitimization_markers", 1,  Eigen::Vector3d(0,1,0));

    current_traj_pub.publish(traj_marker);

    ros::spinOnce();
  }

  ROS_INFO_STREAM("Finished ");

//    ros::spin();

  return 0;
}
