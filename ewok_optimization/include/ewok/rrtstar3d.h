// Copyright (C) 2020 haritsahm
// 
// PX4-TrajectoryReplanning is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// PX4-TrajectoryReplanning is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with PX4-TrajectoryReplanning. If not, see <http://www.gnu.org/licenses/>.

#ifndef RRTSTAR3D_H
#define RRTSTAR3D_H

#include <ewok/ed_ring_buffer.h>
#include <ewok/polynomial_trajectory_3d.h>
#include <ewok/uniform_bspline_3d.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Eigen>
#include <algorithm>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <queue>
#include <random>
#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <std_msgs/ColorRGBA.h>

namespace ewok
{
template <int _N, typename _Scalar = double, typename _Datatype = int16_t>
class RRTStar3D
{
public:
    typedef Eigen::Matrix<_Scalar, 3, 1> Vector3;
    typedef Eigen::Matrix<_Scalar, 3, 3> Matrix3;
    typedef Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
    typedef Eigen::Transform<_Scalar, 3, Eigen::Affine> Affine3;
    typedef Eigen::Quaternion<_Scalar> Quaternion;

    typedef Eigen::Matrix<int, 3, 1> Vector3i;
    typedef std::tuple<Vector3, Vector3, bool> Edge;
    typedef std::pair<Vector3, Vector3> PPoint;
    typedef std::pair<Vector3, bool> PointBool;
    typedef std::pair<Eigen::Vector3f, bool> PointBoolF;

    typedef std::shared_ptr<RRTStar3D<_N, _Scalar, _Datatype>> Ptr;

    struct Node
    {
        std::vector<Node*> children_;
        Node* parent_ = NULL;
        Vector3 pos_;
        _Scalar cost_;
    };

    RRTStar3D(_Scalar step_size = 0.5, _Scalar rrt_factor = 1.1, _Scalar radius = 1, _Scalar solve_tmax = 1,
              _Scalar dt = 0.5, int NUM_ITER=1000)
        : spline_(dt)
        , step_size_(step_size)
        , rrt_factor_(rrt_factor)
        , radius_(radius)
        , debugging_(false)
        , flat_height(true)
        , sampling_alpha(0.2)
        , sampling_beta(0.5)
        , max_solve_t_(solve_tmax)
        , dt_(dt)
        , flag_sol_found(false)
        , flag_rrt_running(false)
        , algorithm_(false)
        , flag_save_log_(false)
        , N_iter(NUM_ITER)
        , rng{std::random_device{}()}
    {

        current_t = 0;

        flag_rrt_started = flag_rrt_finished = false;
        flag_not_enough = false;
        flag_start_found = flag_stop_found = false;
        traj_point_counter = _N;
        flag_hold_dt = false;
        flag_vizualize_output = false;
        flag_rewire_root = false;
        flag_new_path_selected = false;
        rrt_counter=0;
    }

    void reset()
    {
        for (auto p : nodes_)
        {
            delete p;
        }
        nodes_.clear();

        x_sol_.clear();

        solution_queue.clear();

        path_point_.clear();
        edges_.clear();
    }

    void initialize()
    {
        for (auto p : nodes_)
        {
            delete p;
        }
        nodes_.clear();

        for (auto p : x_sol_)
        {
            delete p;
        }
        x_sol_.clear();

        edges_.clear();

        root_ = new Node;
        root_->parent_ = NULL;
        root_->pos_ = start_;
        root_->cost_ = 0;
        lastNode_ = root_;
        nodes_.push_back(root_);

        sub_root = new Node;
        sub_root = root_;

        goal_node = new Node;
    }

    void setLogPath(const std::string& path, bool save_log=false)
    {
        log_path_ = path;
        flag_save_log_ = save_log;
        if(flag_save_log_)
        {
            rrt_writer.open(log_path_+std::string("-rrt.csv"), std::ios::app);
            ellipsoid_writer.open(log_path_+std::string("-ellips.csv"), std::ios::app);
            rrt_path_writer.open(log_path_+std::string("-path.csv"), std::ios::app);
            rrt_tree_writer.open(log_path_+std::string("-tree.csv"), std::ios::app);

        }

    }

    void setRobotPos(const Vector3& pos)
    {
        robot_pos = pos;
    }

    void setRobotPose(const Affine3& m)
    {
        robot_pose_ = m;
    }

    void setPolynomialTrajectory(typename ewok::PolynomialTrajectory3D<10, _Scalar>::Ptr& trajectory)
    {
        trajectory_ = trajectory;
    }

    void setDistanceBuffer(typename ewok::EuclideanDistanceRingBuffer<_N, _Datatype, _Scalar>::Ptr& edrb)
    {
        edrb_ = edrb;
    }


    void addControlPoint(const Vector3& point, int num = 1)
    {
        for (int i = 0; i < num; i++)
        {
            spline_.push_back(point);
            traj_points.push_back(point);
        }
    }

    UniformBSpline3D<_N, _Scalar> getSpline()
    {
        return spline_;
    }

    void setStartPoint(const Vector3 start)
    {
        start_ = start;
    }

    bool getNextPt(Vector3 & next_point)
    {
        if(traj_point_counter == 0)
        {
            Vector3 point = traj_points[traj_point_counter];
            last_point = point;
            next_point = point;
            return true;
        }

        else if(Vector3(robot_pose_.translation() - last_point).norm() < 1)
        {
            Vector3 point = traj_points[traj_point_counter];
            if (traj_point_counter < traj_points.size()-1)
                traj_point_counter++;
            last_point = point;
            next_point = point;
            return true;
        }
        else {
            return false;
        }
    }

    void clearRRT()
    {
        flag_vizualize_output = false;
        reset();
    }

    bool RRTVisualize()
    {
        return flag_vizualize_output;
    }

    bool isRunning()
    {
        return flag_rrt_running;
    }

    void setTargetPoint(const Vector3 target)
    {
        mutex.lock();
        target_ = target;
        goal_node->pos_ = target;
        global_min_cost = distance(start_, target_);

        // informed rrt sampling
        {
            Vector3 id1 = Matrix3::Identity().col(0);

            Vector3 a_1 = (target_ - start_) / Vector3(target_ - start_).norm();
            Matrix3 M = a_1 * id1.transpose();

            Eigen::JacobiSVD<MatrixX> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);

            Eigen::DiagonalMatrix<_Scalar, 3> diag(1, 1, svd.matrixU().determinant() * svd.matrixV().transpose().determinant());

            C_rotation = svd.matrixU() * diag * svd.matrixV();
        }

        mutex.unlock();
    }

    void setHeight(const Vector3 point, bool status = true)
    {
        last_point = point;
        flat_height = status;
        height_ = point;
    }

    _Scalar getCost(const Node* n)
    {
        _Scalar cost = n->cost_;
        return cost;
    }

    _Scalar getDistCost(const Node* p, const Node* q)
    {
        return distance(q->pos_, p->pos_);
    }

    _Scalar getDistCost(const Vector3 p, const Vector3 q)
    {
        _Scalar cost = distance(q,p);
        return cost;
    }

    _Scalar distance(const Vector3 p1, const Vector3 p2)
    {
        Vector3 diff = p2-p1;
        return diff.norm();
    }

    bool isNear(const Vector3& point, _Scalar tol = 2)
    {
        if (distance(point, target_) < tol)
            return true;
        return false;
    }

    bool isCollision(const Node* p, const Node* q)
    {
        if (!edrb_.get())
        {
            ROS_INFO_COND_NAMED(debugging_, "RRT PLANNER", "EDRB ERROR");
            return true;
        }
        bool collision = false;

        std::vector<Vector3> point_check;

        Vector3 len = (q->pos_ - p->pos_);
        point_check.push_back(p->pos_);
        point_check.push_back(p->pos_ + 0.25 * len);
        point_check.push_back(p->pos_ + 0.5 * len);
        point_check.push_back(p->pos_ + 0.75 * len);
        point_check.push_back(q->pos_);

        for (Vector3 pt : point_check)
        {
            collision = edrb_->isNearObstacle(pt, radius_);
            if (collision)
                break;
        }
        return collision;
    }

    bool isCollision(const Vector3 from, const Vector3 to)
    {
        bool collision;
        collision = false;

        std::vector<Vector3> point_check;

        Vector3 len = to-from;
        point_check.push_back(from);
        point_check.push_back(from + 0.25 * len);
        point_check.push_back(from + 0.5 * len);
        point_check.push_back(from + 0.75 * len);
        point_check.push_back(to);
        for (Vector3 pt : point_check)
        {
            collision = edrb_->isNearObstacle(pt, radius_);
            if (collision)
                break;
        }

        return collision;
    }

    _Scalar getRandomNumber(const _Scalar a, const _Scalar b)
    {
        std::uniform_real_distribution<_Scalar> dist(a, b);
        _Scalar num = dist(rng);

        return num;
    }

    _Scalar getbestCost()
    {
        _Scalar min_cost = std::numeric_limits<_Scalar>::infinity();
        _Scalar min_dist = 3*step_size_;
        for (auto n : x_sol_)
        {
            if (n->cost_ <= min_cost && distance(n->pos_, target_) < min_dist)
            {
                min_cost = n->cost_;
                min_dist = distance(n->pos_, target_);
            }
        }

        return min_cost;
    }

    Node* findSolutionNode()
    {
        Node* final = new Node;
        _Scalar min_cost = std::numeric_limits<_Scalar>::infinity();
        _Scalar min_dist = std::numeric_limits<_Scalar>::infinity();
        for (auto n : x_sol_)
        {
            if (n->cost_ <= min_cost && distance(n->pos_, target_) < min_dist)
            {
                min_cost = n->cost_;
                min_dist = distance(n->pos_, target_);
                final = n;
            }
        }
        return final;
    }

    Vector3 BallSampling()
    {

        _Scalar u = (2*getRandomNumber(0, 1))-1;
        _Scalar phi = 2* M_PI * getRandomNumber(0, 1);
        _Scalar r = cbrt(getRandomNumber(-1, 1));
        _Scalar x = r* cos(phi) * sqrt((1-pow(u,2)));
        _Scalar y = r* sin(phi) * sqrt((1-pow(u,2)));
        _Scalar z = r*u;

        return Vector3(x, y, z);
    }

    Vector3 LineSampling()
    {
        Node* near_n = getNearestNode(goal_node);
        Vector3 len = goal_node->pos_ - near_n->pos_;
        len = len / len.norm();

        Vector3 point = near_n->pos_ + len * step_size_;
        if(flat_height) point.z() = height_.z();
        return point;
    }

    Vector3 UniformSampling()
    {
        Vector3 point_min, point_max, rand_point;
        Vector3i point_idx, center_idx;
        edrb_->getVolumeMinMax(point_min, point_max);
        center_idx = edrb_->getVolumeCenter();

        if (flat_height)
            rand_point = Vector3(getRandomNumber(point_min.x(), point_max.x()),
                                 getRandomNumber(point_min.y(), point_max.y()), height_.z());
        else
        {
            rand_point = Vector3(getRandomNumber(point_min.x(), point_max.x()),
                                 getRandomNumber(point_min.y(), point_max.y()),
                                 getRandomNumber(0, height_.z() + 1.5));
        }


        return rand_point;
    }

    Vector3 EllipsoidSampling(_Scalar c_max)
    {
        // Ellips Log Format : time_stamp, Vector3 starting, Vector3 target, double c_best, double c_min, r1(cmax/2), r_2
        Vector3 point_min, point_max;
        Vector3i point_idx;
        edrb_->getVolumeMinMax(point_min, point_max);
        Vector3 pos;
        if (!isinf(c_max))
        {
            _Scalar c_min = Vector3(target_-start_).norm();
            if ((c_max < c_min )|| std::fabs(c_max-c_min) < 1e-06)
                c_max = c_min + c_min/3;

            Vector3 x_center = (start_ + target_) / 2;
            _Scalar r_2 = sqrt(pow(c_max, 2) - pow(c_min, 2)) / 2;
            Eigen::DiagonalMatrix<_Scalar, 3> L((c_max / 2), r_2, r_2);


            Vector3 x_ball = BallSampling();
            pos = C_rotation * L * x_ball + x_center;

            if(flag_save_log_)
                if(ellipsoid_writer.is_open())
                    ellipsoid_writer<<std::fixed<<std::setprecision(8)<<
                    rrt_counter<<delim<<loop_counter<<delim<<toString(start_)<<
                    delim<<toString(target_)<<delim<<flag_real_target<<delim<<
                    c_max<<delim<<c_min<<delim<<r_2<<"\n";

            if (flat_height)
                pos.z() = height_.z();
            if(pos.z() < 0) pos.z() = 0;
            else if(pos.z() > height_.z() + 1.5) pos.z() = height_.z() + 1.5;
        }
        else
        {
            pos = UniformSampling();
        }

        return pos;
    }

    Node* randomSampling(_Scalar& c_max)
    {
        Vector3 pos;
        if (isinf(c_max))
        {
            _Scalar P_r = getRandomNumber(0, 1);

            if (P_r > 1 - sampling_alpha)
            {
                ROS_INFO_COND_NAMED(debugging_, "RRT PLANNER", "Line Sampling");
                pos = LineSampling();
            }

            else if (P_r <= 1 - (sampling_alpha / sampling_beta))
            {
                ROS_INFO_COND_NAMED(debugging_, "RRT PLANNER", "Uniform Sampling");
                pos = UniformSampling();
            }

            else
            {
                ROS_INFO_COND_NAMED(debugging_, "RRT PLANNER", "Ellipsoid Sampling");
                pos = EllipsoidSampling(c_max);
            }
        }
        else
        {
            ROS_INFO_COND_NAMED(debugging_, "RRT PLANNER", "Ellipsoid Sampling");
            pos = EllipsoidSampling(c_max);
        }

        Node* rand_node = new Node;
        rand_node->pos_ = pos;
        return rand_node;
    }

    Node* getNearestNode(const Node* node_)
    {
        _Scalar minDist = std::numeric_limits<_Scalar>::infinity();
        Node* closest = new Node;

        for (auto x_near: nodes_)
        {
            _Scalar dist = distance(node_->pos_, x_near->pos_);
            if (dist < minDist)
            {
                minDist = dist;
                closest = x_near;
            }
        }
        return closest;
    }

    void getNearestNodes(const Node* node, _Scalar radius, std::vector<Node*>& near)
    {
        for (auto n : nodes_)
        {
            _Scalar dist = distance(node->pos_, n->pos_);
            if (dist < radius)
                near.push_back(n);
        }
    }

    Vector3 getConfigurationNode(const Node* rand_, const Node* nearest_)
    {
        Vector3 pos;
        Vector3 rand_pos = rand_->pos_;
        Vector3 near_pos = nearest_->pos_;
        Vector3 midPos = rand_pos - near_pos;
        if (midPos.norm() > step_size_)
        {
            midPos = midPos / midPos.norm();
            pos = near_pos + step_size_ * midPos;
        }
        else
        {
            pos = rand_pos;
        }

        return pos;
    }

    bool solutionFound()
    {
        return flag_sol_found;
    }

    void process()
    {
        if (current_t < trajectory_->duration())
        {
            std::vector<Vector3> traj_pts = trajectory_->evaluates(current_t, dt_, 4, 0);
            end_segment_point = trajectory_->evaluateEndSegment(current_t, 0);
            end_seg_t = trajectory_->getEndSegmentTime(current_t);

            Vector3 robot_traj_dist = robot_pos - traj_pts.front();
            if (Vector3(robot_pos - traj_pts.front()).norm() > 2)
                flag_hold_dt = true;
            else
                flag_hold_dt = false;

            // need to update from subroot
            if(flag_rrt_running)
            {
                if(flag_sol_found)
                {

                    ROS_WARN_STREAM_COND_NAMED(algorithm_, "Proces RRT 2", "Add Root to Path");
                    ROS_WARN_COND(algorithm_, "Add root to path");
                    if (std::find(traj_points.begin(), traj_points.end(), sub_root->pos_) == traj_points.end())
                    {
                        Vector3 last_point = traj_points[traj_points.size()-1];

                        Vector3 mid_point = (last_point + sub_root->pos_)/2;

                        traj_points.push_back(mid_point);
                        spline_.push_back(mid_point);
                        traj_points.push_back(sub_root->pos_);
                        spline_.push_back(sub_root->pos_);
                    }
                    flag_rewire_root = true;
                }
                else if(flag_new_path_selected)
                {
                    flag_new_path_selected = false;
                    Vector3 last_point = traj_points[traj_points.size()-1];

                    Vector3 mid_point = (last_point + sub_root->pos_)/2;

                    traj_points.push_back(mid_point); traj_points.push_back(sub_root->pos_);
                    spline_.push_back(mid_point); spline_.push_back(sub_root->pos_);
                }
            }

            else if(!flag_rrt_running && flag_rrt_finished || flag_not_enough)
            {
                ROS_WARN_STREAM_COND_NAMED(algorithm_, "Proces RRT", "RRT FINISHED");

                Vector3 last_point = traj_points[traj_points.size()-1];
                Vector3 mid_point = (last_point + target_)/2;
                traj_points.push_back(mid_point);
                traj_points.push_back(target_);
                spline_.push_back(mid_point);
                spline_.push_back(target_);
                flag_rewire_root = false;
                flag_rrt_finished = false;
                flag_stop_found = false;
                flag_rrt_started = false;
                flag_not_enough = false;
                current_t = reset_dt_;
                if(!flag_real_target) current_t = end_seg_t;
                reset();
                delete tra_gene_thread_;
            }


            if (edrb_->insideVolume(traj_pts))
            {
                std::vector<PointBool> traj_pts_bool = edrb_->isNearObstacle2(traj_pts, radius_);

                if ((!flag_stop_found && flag_rrt_started))  // End point search
                {
                    ROS_WARN_COND_NAMED(algorithm_, "Process", "End Point Search");

                    for (int i = 0; i < traj_pts_bool.size() - 1; i++)
                    {
                        PointBool prev_pt = traj_pts_bool[i];
                        PointBool next_pt = traj_pts_bool[i + 1];

                        /*
             * To prevent multiple points inside path_checker
             */
                        if (std::find(path_checker.begin(), path_checker.end(), prev_pt) == path_checker.end())
                        {
                            path_checker.push_back(prev_pt);
                        }

                        if (std::find(path_checker.begin(), path_checker.end(), next_pt) == path_checker.end())
                        {
                            path_checker.push_back(next_pt);
                        }

                        // If multiple point is blocked, insert them to the list
                        if (prev_pt.second && next_pt.second)
                        {
                            if (std::find(obs_list.begin(), obs_list.end(), prev_pt.first) == obs_list.end())
                                obs_list.push_back(prev_pt.first);
                            if (std::find(obs_list.begin(), obs_list.end(), next_pt.first) == obs_list.end())
                                obs_list.push_back(next_pt.first);
                        }

                        // else if the second point is free, set as real target point
                        else if (prev_pt.second && !next_pt.second)
                        {
                            // less than counter
                            if (obs_list.size() < 3)
                            {
                                ROS_WARN_COND_NAMED(algorithm_, "Process", "Less Counter - Skipping");
                                flag_not_enough = true;
                                reset_dt_ = current_t + dt_ * (i+1);
                                obs_list.clear();
                                break;
                            }

                            // normal
                            else
                            {
                                ROS_WARN_COND_NAMED(algorithm_, "Process", "Found Normal Endpoint");
                                ROS_WARN_STREAM_COND_NAMED(algorithm_, "Proces",
                                                           "Starting:" << curr_start_pt.transpose()
                                                           << " Endpoint: " << next_pt.first.transpose());
                                target_ = next_pt.first;
                                setTargetPoint(next_pt.first);

                                obstacle_counter = 0;
                                flag_real_target = true;
                                flag_stop_found = true;
                                flag_not_enough = false;
                                reset_dt_ = current_t + dt_ * (i+1);
                            }
                        }
                    }
                }

                else if (!flag_rrt_started && !flag_stop_found)  // found starting point
                {
                    ROS_WARN_COND_NAMED(algorithm_, "Process", "Start Point Search");

                    for (int i = 0; i < traj_pts_bool.size() - 1; i++)  // Start point search
                    {
                        PointBool prev_pt = traj_pts_bool[i];
                        PointBool next_pt = traj_pts_bool[i + 1];

                        /*
             * If free, insert prev_pt to traj_point and spline
             */
                        if (!prev_pt.second)
                        {
                            if (std::find(traj_points.begin(), traj_points.end(), prev_pt.first) == traj_points.end())
                            {
                                traj_points.push_back(prev_pt.first);
                                spline_.push_back(prev_pt.first);
                            }
                        }

                        /*
             * To prevent multiple points inside path_checker
             */
                        if (std::find(path_checker.begin(), path_checker.end(), prev_pt) == path_checker.end())
                        {
                            path_checker.push_back(prev_pt);
                        }

                        if (std::find(path_checker.begin(), path_checker.end(), next_pt) == path_checker.end())
                        {
                            path_checker.push_back(next_pt);
                        }

                        /*
             * If the next_pt is not free, set as rrt starting point
             * and use the end of the segment point as target
             */
                        if (!prev_pt.second && next_pt.second)
                        {

                            start_ = prev_pt.first;
                            target_ = end_segment_point;

                            initialize();
                            setTargetPoint(end_segment_point);

                            ROS_WARN_STREAM_COND_NAMED(algorithm_, "Proces RRT 2",
                                                       "Proces RRT 2 Starting:" << start_.transpose() << " Endpoint: " << target_.transpose());

                            rrt_counter++;
                            tra_gene_thread_ = new boost::thread(boost::bind(&RRTStar3D::solveRRT, this));
                            for(int i=0; i < 7; i++)
                            {
                                traj_points.push_back(start_);
                                spline_.push_back(start_);
                            }
                            curr_start_pt = prev_pt.first;
                            obs_list.push_back(next_pt.first);
                            obstacle_counter = 1;
                            flag_start_found = true;
                            flag_real_target = false;
                            break;
                        }
                    }
                }

                if ((!flag_hold_dt && !flag_rrt_running)|| (flag_rrt_running && !flag_stop_found))
                    current_t += dt_;
            }
        }

    }

    void process_TEST()
    {
        if (current_t < trajectory_->duration())
        {
            std::vector<Vector3> traj_pts = trajectory_->evaluates(current_t, dt_, 4, 0);
            end_segment_point = trajectory_->evaluateEndSegment(current_t, 0);

            //            if (edrb_->insideVolume(traj_pts))
            //            {
            std::vector<PointBool> traj_pts_bool = edrb_->isNearObstacle2(traj_pts, radius_);

            if ((!flag_stop_found && flag_rrt_started))  // End point search
            {
                ROS_WARN_COND_NAMED(algorithm_, "Process", "End Point Search");

                for (int i = 0; i < traj_pts_bool.size() - 1; i++)
                {
                    PointBool prev_pt = traj_pts_bool[i];
                    PointBool next_pt = traj_pts_bool[i + 1];

                    /*
             * To prevent multiple points inside path_checker
             */
                    if (std::find(path_checker.begin(), path_checker.end(), prev_pt) == path_checker.end())
                    {
                        path_checker.push_back(prev_pt);
                    }

                    if (std::find(path_checker.begin(), path_checker.end(), next_pt) == path_checker.end())
                    {
                        path_checker.push_back(next_pt);
                    }

                    // If multiple point is blocked, insert them to the list
                    if (prev_pt.second && next_pt.second)
                    {
                        if (std::find(obs_list.begin(), obs_list.end(), prev_pt.first) == obs_list.end())
                            obs_list.push_back(prev_pt.first);
                        if (std::find(obs_list.begin(), obs_list.end(), next_pt.first) == obs_list.end())
                            obs_list.push_back(next_pt.first);
                    }

                    // else if the second point is free, set as real target point
                    else if (prev_pt.second && !next_pt.second)
                    {
                        // less than counter
                        if (obs_list.size() < 3)
                        {
                            ROS_WARN_COND_NAMED(algorithm_, "Process", "Less Counter - Skipping");
                            flag_not_enough = true;
                            reset_dt_ = current_t + dt_ * (i+1);
                            obs_list.clear();
                            break;
                        }

                        // normal
                        else
                        {
                            ROS_WARN_COND_NAMED(algorithm_, "Process", "Found Normal Endpoint");
                            ROS_WARN_STREAM_COND_NAMED(algorithm_, "Proces",
                                                       "Starting:" << curr_start_pt.transpose()
                                                       << " Endpoint: " << next_pt.first.transpose());
                            target_ = next_pt.first;
                            setTargetPoint(next_pt.first);

                            obstacle_counter = 0;
                            flag_real_target = true;
                            flag_stop_found = true;
                            flag_not_enough = false;
                            reset_dt_ = current_t + dt_ * (i+1);
                            break;
                        }
                    }
                }
            }

            else if (!flag_rrt_started && !flag_stop_found)  // found starting point
            {
                ROS_WARN_COND_NAMED(algorithm_, "Process", "Start Point Search");

                for (int i = 0; i < traj_pts_bool.size() - 1; i++)  // Start point search
                {
                    PointBool prev_pt = traj_pts_bool[i];
                    PointBool next_pt = traj_pts_bool[i + 1];

                    /*
             * If free, insert prev_pt to traj_point and spline
             */
                    if (!prev_pt.second)
                    {
                        if (std::find(traj_points.begin(), traj_points.end(), prev_pt.first) == traj_points.end())
                        {
                            traj_points.push_back(prev_pt.first);
                            spline_.push_back(prev_pt.first);
                        }
                    }

                    /*
             * To prevent multiple points inside path_checker
             */
                    if (std::find(path_checker.begin(), path_checker.end(), prev_pt) == path_checker.end())
                    {
                        path_checker.push_back(prev_pt);
                    }

                    if (std::find(path_checker.begin(), path_checker.end(), next_pt) == path_checker.end())
                    {
                        path_checker.push_back(next_pt);
                    }

                    /*
             * If the next_pt is not free, set as rrt starting point
             * and use the end of the segment point as target
             */
                    if (!prev_pt.second && next_pt.second)
                    {

                        start_ = prev_pt.first;
                        target_ = end_segment_point;

                        initialize();
                        setTargetPoint(end_segment_point);

                        ROS_WARN_STREAM_COND_NAMED(algorithm_, "Proces RRT 2",
                                                   "Proces RRT 2 Starting:" << start_.transpose() << " Endpoint: " << target_.transpose());

                        rrt_counter++;
                        tra_gene_thread_ = new boost::thread(boost::bind(&RRTStar3D::solveRRT_TEST, this));
                        for(int i=0; i < 5; i++)
                        {
                            traj_points.push_back(start_);
                            spline_.push_back(start_);
                        }
                        curr_start_pt = prev_pt.first;
                        obs_list.push_back(next_pt.first);
                        obstacle_counter = 1;
                        flag_start_found = true;
                        flag_real_target = false;
                        break;
                    }
                }
            }

            if (!flag_rrt_running || (flag_rrt_running && !flag_stop_found))
                current_t += dt_;
            //            }

        }
    }

    void solveRRT_TEST()
    {
        flag_rrt_running = true;
        flag_rrt_started = true;
        flag_rrt_finished = false;
        Node* final = NULL;
        solution_node = new Node;
        _Scalar search_radius;
        bool found = false;
        flag_sol_found = false;
        path_point_.clear();
        _Scalar free_space;
        _Scalar curr_cost;

        search_t_stamp = std::chrono::high_resolution_clock::now();
        // RRT Log Format : time_stamp, int rrt_counter, int rrt_iteration, Vector3 starting, Vector3 target, bool real_target,
        //                  double free space, double search_radius, int node_size, double best_cost, double curr_cost
        // Ellips Log Format : time_stamp, Vector3 starting, Vector3 target, int node_size, double c_best, double c_min, r1(cmax/2), r_2

        ROS_INFO_COND_NAMED(algorithm_, "RRT PLANNER", "Starting RRT");
        best_cost_ = std::numeric_limits<_Scalar>::infinity();
        int iter_counter = 0;
        while(iter_counter < N_iter)
        {
            ROS_INFO_COND_NAMED(debugging_, "RRT PLANNER", "Getting Random Node");
            mutex.lock();
            if (x_sol_.size() > 0)
                best_cost_ = getbestCost();
            Node* rand_node = randomSampling(best_cost_);
            mutex.unlock();

            if (rand_node)
            {
                ROS_INFO_COND_NAMED(debugging_, "RRT PLANNER", "Find Nearest");
                mutex.lock();
                Node* nearest_node = getNearestNode(rand_node);

                ROS_INFO_COND_NAMED(debugging_, "RRT PLANNER", "Get Conf Node");
                Node* new_node = new Node;
                new_node->pos_ = getConfigurationNode(rand_node, nearest_node);
                mutex.unlock();
                if (!isCollision(nearest_node, new_node))
                {
                    std::vector<Node*> near_nodes;
                    ROS_INFO_COND_NAMED(debugging_, "RRT PLANNER", "Get Nearest Nodes");

                    edrb_->getMapInfo(free_space);
                    rrt_gamma_ = 2*pow((1+1/3),1/3)*pow(free_space/4.189,1/3);
                    search_radius = std::min(rrt_gamma_*pow(log(nodes_.size()+1)/nodes_.size()+1, 1/3),
                                             step_size_*rrt_factor_);
                    getNearestNodes(new_node, search_radius, near_nodes);

                    ROS_INFO_COND_NAMED(debugging_, "RRT PLANNER", "Find Parent");

                    Node* min_node = nearest_node;
                    _Scalar min_cost = getCost(nearest_node) + getDistCost(nearest_node, new_node);
                    for (auto x_near : near_nodes)
                    {
                        _Scalar new_cost = getCost(x_near) + getDistCost(x_near, new_node);
                        if (!isCollision(x_near, new_node) && new_cost < min_cost)
                        {
                            min_node = x_near;
                            min_cost = new_cost;
                        }
                    }

                    new_node->parent_ = min_node;
                    new_node->cost_ = min_cost;
                    min_node->children_.push_back(new_node);
                    edges_.push_back(std::make_tuple(min_node->pos_, new_node->pos_, false));
                    nodes_.push_back(new_node);
                    lastNode_ = new_node;

                    ROS_INFO_COND_NAMED(debugging_, "RRT PLANNER", "Rewire Tree");
                    for (Node* x_near : near_nodes)
                    {
                        _Scalar min_cost = getCost(new_node) + getDistCost(new_node, x_near);
                        bool isCollisionn = !isCollision(new_node->pos_, x_near->pos_);
                        if (isCollisionn && (min_cost < x_near->cost_ ))
                        {
                            Node* n_parent = new Node;
                            n_parent = x_near->parent_;
                            n_parent->children_.erase(std::remove(n_parent->children_.begin(), n_parent->children_.end(), x_near),
                                                      n_parent->children_.end());
                            edges_.erase(std::remove(edges_.begin(), edges_.end(), std::make_tuple(n_parent->pos_, x_near->pos_, false)),
                                         edges_.end());
                            x_near->cost_ = min_cost;
                            x_near->parent_ = new_node;
                            new_node->children_.push_back(x_near);
                            edges_.push_back(std::make_tuple(new_node->pos_, x_near->pos_, false));
                        }
                    }
                }
            }

            if (isNear(lastNode_->pos_, 0.75))
            {
                ROS_WARN_COND(algorithm_, "Found Solution");
                if (std::find(x_sol_.begin(), x_sol_.end(), lastNode_) == x_sol_.end())
                {
                    x_sol_.push_back(lastNode_);
                }
                found = true;
                flag_sol_found = true;
                if(temp_solution && x_sol_.size() > 0)
                {
                    x_sol_.erase(std::remove(x_sol_.begin(), x_sol_.end(), temp_solution), x_sol_.end());
                }
            }

            std::chrono::duration<_Scalar> t_elapsed = std::chrono::high_resolution_clock::now() - search_t_stamp;

            _Scalar time_elapsed = t_elapsed.count();
            if(x_sol_.size() > 1 || iter_counter > N_iter - 5)
            {
                if(found)
                {
                    std::vector<Node*> temp_solution;
                    Node* possible_solution = findSolutionNode();

                    // if empty, add new solution to path
                    if(solution_queue.empty())
                    {
                        curr_cost = possible_solution->cost_;
                        final = possible_solution;

                        path_point_.clear();
                        while (final != NULL)
                        {
                            Vector3 pos = final->pos_;
                            path_point_.push_front(pos);
                            final = final->parent_;
                        }
                        path_point_.push_back(target_);

                    }
                    else
                    {
                        final = possible_solution;
                        while (final != NULL)
                        {
                            Vector3 pos = final->pos_;
                            path_point_.push_front(pos);
                            final = final->parent_;
                        }
                        path_point_.push_back(target_);

                    }
                }

                else {
                    Node* possible_solution = getNearestNode(goal_node);
                    final = possible_solution;
                    while (final != NULL)
                    {
                        Vector3 pos = final->pos_;
                        path_point_.push_front(pos);
                        final = final->parent_;
                    }
                    path_point_.push_back(target_);
                    found = true;
                    flag_sol_found = true;
                }

                flag_vizualize_output = true;
            }

            if(flag_save_log_)
            {
                //                ROS_INFO("Writing Log");
                _Scalar elapsed_time_mics = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - search_t_stamp).count();

                Vector3 temp_goal = target_;
                Vector3 temp_start = start_;

                if(rrt_writer.is_open())
                {
                    rrt_writer<<std::fixed<<std::setprecision(8)<<elapsed_time_mics<<","
                             <<rrt_counter<<","
                            <<iter_counter<<","
                           <<toString(temp_start)<<","
                          <<toString(temp_goal)<<","
                         <<flag_real_target<<","
                        <<free_space<<","
                       <<search_radius<<","
                      <<nodes_.size()<<","
                     <<best_cost_<<","
                    <<curr_cost << "\n";

                }
            }

            iter_counter++;
        }

        std::cout << "Done" << std::endl;
        std::vector<Vector3> final_path(path_point_.begin(), path_point_.end());

        if(flag_save_log_)
        {
            std::cout <<"Writing Path" << std::endl;

            if(rrt_path_writer.is_open())
            {
                for (int i=0; i < final_path.size(); i++)
                {
                    rrt_path_writer<<std::fixed<<std::setprecision(8)<<final_path[i].transpose()<<"\n";
                }
                rrt_path_writer<<"\n";
            }

            std::cout <<"Writing Tree" << std::endl;

            if(rrt_tree_writer.is_open())
            {
                for (int i = 0; i < edges_.size(); i++)
                {
                    Vector3 p, q;
                    p = std::get<0>(edges_[i]);
                    q = std::get<1>(edges_[i]);
                    rrt_tree_writer<<std::fixed<<std::setprecision(8)<<p.transpose()<<","<<q.transpose()<<"\n";
                }
            }
        }


        for (int i=0; i < final_path.size()-1; i++) {
            Vector3 midPoint = (final_path[i+1] + final_path[i])/2;
            spline_.push_back(final_path[i]);
            spline_.push_back(midPoint);
        }

        flag_rrt_running = false;
        flag_rrt_finished = false;
        flag_stop_found = false;
        flag_rrt_started = false;
        current_t = reset_dt_;
    }

    void solveRRT()
    {
        flag_rrt_running = true;
        flag_rrt_started = true;
        flag_rrt_finished = false;
        Node* final = NULL;
        solution_node = new Node;
        loop_counter = 0;
        _Scalar search_radius;
        bool found = false;
        flag_sol_found = false;
        path_point_.clear();
        _Scalar free_space;
        _Scalar curr_cost;

        search_t_stamp = std::chrono::high_resolution_clock::now();
        // RRT Log Format : time_stamp, int rrt_counter, int rrt_iteration, Vector3 starting, Vector3 target, bool real_target,
        //                  double free space, double search_radius, int node_size, double best_cost, double curr_cost
        // Ellips Log Format : time_stamp, Vector3 starting, Vector3 target, int node_size, double c_best, double c_min, r1(cmax/2), r_2

        ROS_INFO_COND_NAMED(algorithm_, "RRT PLANNER", "Starting RRT");
        best_cost_ = std::numeric_limits<_Scalar>::infinity();
        while (Vector3(sub_root->pos_ - goal_node->pos_).norm() > 0.5 ||
               ( Vector3(robot_pose_.translation() - goal_node->pos_).norm() > 0.5))
        {
            curr_cost = -1;

            if(solution_queue.size() > 0)
            {
                auto it_sol=find(solution_queue.begin(),solution_queue.end(),sub_root);
                int sol_pos = it_sol - solution_queue.begin();

                if(sol_pos > solution_queue.size()-3) break;
            }

            if((Vector3(robot_pos-target_).norm() < 0.5 || (Vector3(sub_root->pos_ - goal_node->pos_).norm() < 0.5)) &&
                 !isCollision(sub_root, goal_node)) break;


            // Too Short
            if (flag_not_enough)
            {
                // clear
                flag_rrt_started = false;
                break;
            }

            // rewire path solution if needed
            if(flag_rewire_root && found && solution_queue.size()>0)
            {
                ROS_INFO_COND(algorithm_, "Rewire Path");
                bool replaced = false;
                for(int i = 0; i < solution_queue.size()-1; i++)
                {

                    if(solution_queue[i]==sub_root)
                    {
                        if(Vector3(robot_pose_.translation() - sub_root->pos_).norm() < 2*step_size_)
                        {
                            sub_root = solution_queue[i+1];
                            replaced = true;
                            setTargetPoint(target_);
                            break;
                        }
                    }
                }

                if(!replaced && solution_queue.size() > 0)
                {
                    _Scalar min_dist = std::numeric_limits<_Scalar>::infinity();
                    Node* nearest_node = new Node;
                    for(auto node: solution_queue)
                    {
                        if(distance(sub_root->pos_, node->pos_) < min_dist)
                        {
                            nearest_node = node;
                            min_dist = distance(sub_root->pos_, node->pos_);
                        }
                    }
                    flag_new_path_selected = true;
                    sub_root = nearest_node;
                    setTargetPoint(target_);
                }

                flag_rewire_root = false;
            }


            ROS_INFO_COND_NAMED(debugging_, "RRT PLANNER", "Getting Random Node");
            mutex.lock();
            if (x_sol_.size() > 0)
                best_cost_ = getbestCost();
            Node* rand_node = randomSampling(best_cost_);
            mutex.unlock();

            if (rand_node)
            {
                ROS_INFO_COND_NAMED(debugging_, "RRT PLANNER", "Find Nearest");
                mutex.lock();
                Node* nearest_node = getNearestNode(rand_node);

                ROS_INFO_COND_NAMED(debugging_, "RRT PLANNER", "Get Conf Node");
                Node* new_node = new Node;
                new_node->pos_ = getConfigurationNode(rand_node, nearest_node);
                mutex.unlock();
                if (!isCollision(nearest_node, new_node))
                {
                    std::vector<Node*> near_nodes;
                    ROS_INFO_COND_NAMED(debugging_, "RRT PLANNER", "Get Nearest Nodes");

                    edrb_->getMapInfo(free_space);
                    rrt_gamma_ = 2*pow((1+1/3),1/3)*pow(free_space/4.189,1/3);
                    search_radius = std::min(rrt_gamma_*pow(log(nodes_.size()+1)/nodes_.size()+1, 1/3),
                                             step_size_*rrt_factor_);
                    getNearestNodes(new_node, search_radius, near_nodes);

                    ROS_INFO_COND_NAMED(debugging_, "RRT PLANNER", "Find Parent");

                    Node* min_node = nearest_node;
                    _Scalar min_cost = getCost(nearest_node) + getDistCost(nearest_node, new_node);
                    for (auto x_near : near_nodes)
                    {
                        _Scalar new_cost = getCost(x_near) + getDistCost(x_near, new_node);
                        if (!isCollision(x_near, new_node) && new_cost < min_cost)
                        {
                            min_node = x_near;
                            min_cost = new_cost;
                        }
                    }

                    new_node->parent_ = min_node;
                    new_node->cost_ = min_cost;
                    min_node->children_.push_back(new_node);
                    edges_.push_back(std::make_tuple(min_node->pos_, new_node->pos_, false));
                    nodes_.push_back(new_node);
                    lastNode_ = new_node;

                    ROS_INFO_COND_NAMED(debugging_, "RRT PLANNER", "Rewire Tree");
                    for (Node* x_near : near_nodes)
                    {
                        _Scalar min_cost = getCost(new_node) + getDistCost(new_node, x_near);
                        bool isCollisionn = !isCollision(new_node->pos_, x_near->pos_);
                        if (isCollisionn && (min_cost < x_near->cost_ ))
                        {
                            Node* n_parent = x_near->parent_;
                            n_parent->children_.erase(std::remove(n_parent->children_.begin(), n_parent->children_.end(), x_near),
                                                      n_parent->children_.end());
                            edges_.erase(std::remove(edges_.begin(), edges_.end(), std::make_tuple(n_parent->pos_, x_near->pos_, false)),
                                         edges_.end());

                            x_near->cost_ = min_cost;
                            x_near->parent_ = new_node;
                            new_node->children_.push_back(x_near);
                            edges_.push_back(std::make_tuple(new_node->pos_, x_near->pos_, false));
                        }
                    }
                }
            }

            if (isNear(lastNode_->pos_, 0.75))
            {
                ROS_WARN_COND(algorithm_, "Found Solution");
                if (std::find(x_sol_.begin(), x_sol_.end(), lastNode_) == x_sol_.end())
                {
                    x_sol_.push_back(lastNode_);
                }
                found = true;
                flag_sol_found = true;
                if(temp_solution && x_sol_.size() > 0)
                {
                    x_sol_.erase(std::remove(x_sol_.begin(), x_sol_.end(), temp_solution), x_sol_.end());
                }
            }

            //generate solution in time
            std::chrono::duration<_Scalar> t_elapsed = std::chrono::high_resolution_clock::now() - search_t_stamp;
            if(t_elapsed.count() > max_solve_t_)
            {
                if(found)
                {
                    std::vector<Node*> temp_solution;
                    Node* possible_solution = new Node;
                    possible_solution= findSolutionNode();

                    // if empty, add new solution to path
                    if(solution_queue.size() == 0)
                    {
                        solution_node = possible_solution;
                        curr_cost = possible_solution->cost_;
                        final = possible_solution;

                        path_point_.clear();
                        while (final != NULL)
                        {
                            Vector3 pos = final->pos_;
                            path_point_.push_front(pos);
                            solution_queue.insert(solution_queue.begin(), final);
                            final = final->parent_;
                        }
                        path_point_.push_back(target_);

                    }
                    else
                    {
                        ROS_INFO_COND(algorithm_, "looking for path");
                        final = possible_solution;

                        // check and combine path with original solution
                        while (final != NULL)
                        {
                            temp_solution.insert(temp_solution.begin(), final);
                            final = final->parent_;
                        }

                        auto it_sol=find(solution_queue.begin(),solution_queue.end(),sub_root);
                        int sol_pos = it_sol - solution_queue.begin();


                        auto it_temp=find(temp_solution.begin(),temp_solution.end(), sub_root);
                        int temp_pos = it_temp - temp_solution.begin();

                        if(temp_pos != 0 && temp_solution.size() > 0)
                        {
                            if(it_temp != temp_solution.end())
                            {
                                solution_queue.erase(solution_queue.begin(), solution_queue.end());
                                solution_queue.clear();
                                solution_queue = temp_solution;
                                curr_cost = possible_solution->cost_;
                                final = possible_solution;
                                while (final != NULL)
                                {
                                    Vector3 pos = final->pos_;
                                    path_point_.push_front(pos);
                                    final = final->parent_;
                                }
                                path_point_.push_back(target_);
                            }

                            // if there's no match or new path with smaller cost
                            // Need to rewrte the new solution method
                            else if(it_temp == temp_solution.end() && possible_solution->cost_ < solution_node->cost_)
                            {
                                bool close_path = false;
                                Node* close_node = new Node;
                                for(auto p: temp_solution)
                                {
                                    if(distance(p->pos_, sub_root->pos_) < step_size_*2.5)
                                    {
                                        if(!isCollision(p->pos_, sub_root->pos_))
                                        {
                                            close_node = p;
                                            close_path = true;
                                        }
                                    }

                                }

                                if(close_path)
                                {
                                    solution_queue.erase(solution_queue.begin(), solution_queue.end());
                                    solution_queue.clear();
                                    solution_queue = temp_solution;
                                    final = possible_solution;
                                    curr_cost = possible_solution->cost_;
                                    while (final != NULL)
                                    {
                                        Vector3 pos = final->pos_;
                                        path_point_.push_front(pos);
                                        final = final->parent_;
                                    }
                                    path_point_.push_back(target_);
                                }
                            }

                        }

                        path_point_.clear();
                        if(solution_queue.size()> 0)
                        {
                            for(int i = 0; i < solution_queue.size();i++)
                            {
                                path_point_.push_back(solution_queue[i]->pos_);
                            }

                            path_point_.push_back(target_);
                        }

                    }

                }

                else {
                    temp_solution = new Node;
                    temp_solution = findSolutionNode();
                    curr_cost = temp_solution->cost_;

                    if (std::find(x_sol_.begin(), x_sol_.end(), temp_solution) == x_sol_.end())
                    {
                        x_sol_.push_back(temp_solution);
                    }
                    found = true;
                    flag_sol_found = true;
                }

                flag_vizualize_output = true;
            }

            mutex.lock();
            if(flag_save_log_)
            {
                //                ROS_INFO("Writing Log");
                _Scalar elapsed_time_mics = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - search_t_stamp).count();

                Vector3 temp_goal = target_;
                Vector3 temp_start = start_;
                //          rrt_writer.open(log_path_+std::string("-rrt.csv"), std::ios::app);
                if(rrt_writer.is_open())
                {
                    rrt_writer<<std::fixed<<std::setprecision(8)<<elapsed_time_mics<<","
                             <<rrt_counter<<","
                            <<loop_counter<<","
                           <<toString(temp_start)<<","
                          <<toString(temp_goal)<<","
                         <<flag_real_target<<","
                        <<free_space<<","
                       <<search_radius<<","
                      <<nodes_.size()<<","
                     <<best_cost_<<","
                    <<curr_cost << "\n";

                }
                //          rrt_writer.close();

            }
            mutex.unlock();

            loop_counter++;

        }
        std::cout << "RRT FINISHED" << std::endl;
        flag_rrt_running = false;
        flag_rrt_finished = true;
    }


    void TrajectoryChecker(visualization_msgs::Marker& traj_marker, const std::string& frame = "world",
                           const std::string& ns = "trajectory_checker",
                           const Eigen::Vector3d& obs = Eigen::Vector3d(1, 0.5, 1),
                           const Eigen::Vector3d& free = Eigen::Vector3d(1, 1, 0))
    {
        traj_marker.header.frame_id = frame;
        traj_marker.ns = ns;
        traj_marker.id = 0;
        traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        traj_marker.action = visualization_msgs::Marker::MODIFY;
        traj_marker.scale.x = 0.1;
        traj_marker.scale.y = 0.1;
        traj_marker.scale.z = 0.1;
        traj_marker.color.a = 1.0;

        std_msgs::ColorRGBA c_obs, c_free;
        c_obs.r = obs.x();
        c_obs.g = obs.y();
        c_obs.b = obs.z();
        c_obs.a = 1.0;

        c_free.r = free.x();
        c_free.g = free.y();
        c_free.b = free.z();
        c_free.a = 1.0;

        if (path_checker.size() != 0)
        {
            for (auto point : path_checker)
            {
                PointBool pb = point;
                geometry_msgs::Point p;
                p.x = pb.first.x();
                p.y = pb.first.y();
                p.z = pb.first.z();

                if (pb.second)
                {
                    traj_marker.colors.push_back(c_obs);
                    traj_marker.points.push_back(p);
                }

                else
                {
                    traj_marker.colors.push_back(c_free);
                    traj_marker.points.push_back(p);
                }
            }
        }
    }

    std::string toString(const Vector3& vec)
    {
        std::stringstream ss;
        ss <<"[" << vec.x() << '|' << vec.y() << '|' << vec.z() <<"]";
        return ss.str();
    }


    void getTrajectoryMarkers(visualization_msgs::MarkerArray& traj_marker,
                              const std::string& ns = "spline_opitimization_markers",
                              const Eigen::Vector3d& color1 = Eigen::Vector3d(0, 1, 0),
                              const Eigen::Vector3d& color2 = Eigen::Vector3d(0, 1, 1))
    {
        traj_marker.markers.resize(2);
        spline_.getVisualizationMarker(traj_marker.markers[0], ns, 0, color1, traj_point_counter, 2, color2);
        spline_.getControlPointsMarker(traj_marker.markers[1], ns, 1, color1, traj_point_counter, 2, color2);

    }

    void getTreeMarker(visualization_msgs::Marker& traj_marker, const std::string& ns, int id = 0,
                       const Eigen::Vector3f& color = Eigen::Vector3f(1, 1, 0), _Scalar scale = 0.01)
    {
        if (edges_.size() > 0)
        {
            traj_marker.header.frame_id = "world";
            traj_marker.ns = ns;
            traj_marker.id = id;
            traj_marker.type = visualization_msgs::Marker::LINE_LIST;
            traj_marker.action = visualization_msgs::Marker::MODIFY;
            traj_marker.scale.x = scale;

            std_msgs::ColorRGBA c_free, c_obs;

            c_free.r = 0;
            c_free.g = 1.0;
            c_free.b = 0;
            c_free.a = 1.0;

            c_obs.r = 1.0;
            c_obs.g = 0.5;
            c_obs.b = 1.0;
            c_obs.a = 1.0;

            traj_marker.color = c_free;

            for (int i = 0; i < edges_.size() - 1; i++)
            {
                Vector3 p, q;
                p = std::get<0>(edges_[i]);
                q = std::get<1>(edges_[i]);

                geometry_msgs::Point p_, q_;
                p_.x = p.x();
                p_.y = p.y();
                p_.z = p.z();
                q_.x = q.x();
                q_.y = q.y();
                q_.z = q.z();

                if (std::get<2>(edges_[i]))
                {
                    traj_marker.points.push_back(p_);
                    traj_marker.points.push_back(q_);
                }
                else
                {
                    traj_marker.points.push_back(p_);
                    traj_marker.points.push_back(q_);
                }
            }
        }
    }

    void getSolutionMarker(visualization_msgs::Marker& traj_marker, const std::string& ns, int id = 0,
                           const Eigen::Vector3f& color = Eigen::Vector3f(0, 1, 0), _Scalar scale = 0.01)
    {
        if (path_point_.size() > 0)
        {
            traj_marker.header.frame_id = "world";
            traj_marker.ns = ns;
            traj_marker.id = id;
            traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
            traj_marker.action = visualization_msgs::Marker::MODIFY;
            traj_marker.scale.x = scale;
            traj_marker.color.a = 1.0;

            // cyan
            traj_marker.color.r = color(0);
            traj_marker.color.g = color(1);
            traj_marker.color.b = color(2);

            for (Vector3 n : path_point_)
            {
                geometry_msgs::Point p;
                p.x = n.x();
                p.y = n.y();
                p.z = n.z();

                traj_marker.points.push_back(p);
            }
        }
        flag_vizualize_output = false;

    }

    void getRRTProperty(visualization_msgs::Marker& traj_marker, const std::string& ns, int id = 0,
                        const Eigen::Vector4f& color = Eigen::Vector4f(0, 1, 0, 1), _Scalar scale = 0.01)
    {
        traj_marker.header.frame_id = "world";
        traj_marker.ns = ns;
        if(id == 0) // hyperellipsoid
        {
            traj_marker.id = id;
            traj_marker.type = visualization_msgs::Marker::CYLINDER;
            traj_marker.action = visualization_msgs::Marker::MODIFY;

            traj_marker.color.r = color(0);
            traj_marker.color.g = color(1);
            traj_marker.color.b = color(2);
            traj_marker.color.a = color(3);

            Vector3 center = (start_+target_)/2;
            Quaternion orien(C_rotation);
            if(best_cost_ < global_min_cost || (best_cost_-global_min_cost) < 1e-06)
                best_cost_ = global_min_cost + best_cost_/3;

            traj_marker.pose.position.x =center.x();
            traj_marker.pose.position.y =center.y();
            traj_marker.pose.position.z =center.z();
            traj_marker.pose.orientation.w = orien.w();
            traj_marker.pose.orientation.x = orien.x();
            traj_marker.pose.orientation.y = orien.y();
            traj_marker.pose.orientation.z = orien.z();

            traj_marker.scale.y = sqrt(pow(best_cost_,2) - pow(global_min_cost,2));
            traj_marker.scale.x = best_cost_;
            if(flat_height)
                traj_marker.scale.z = 1;
            else {
                traj_marker.scale.z = sqrt(pow(best_cost_,2) - pow(global_min_cost,2));
                traj_marker.type = visualization_msgs::Marker::SPHERE;
            }
        }

        else if(id ==1){
            traj_marker.id = id;
            traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;
            traj_marker.action = visualization_msgs::Marker::MODIFY;

            // Need to add visual for sub_root

            std_msgs::ColorRGBA root_c, subroot_c;

            root_c.a = color(3);
            root_c.r = color(0);
            root_c.g = color(1);
            root_c.b = color(2);

            subroot_c.a = 0.2;
            subroot_c.r = 0;
            subroot_c.g = 0;
            subroot_c.b = 1;

            geometry_msgs::Point point;
            point.x = start_.x();
            point.y = start_.y();
            point.z = start_.z();
            traj_marker.points.push_back(point);
            traj_marker.colors.push_back(root_c);

            point.x = target_.x();
            point.y = target_.y();
            point.z = target_.z();
            traj_marker.points.push_back(point);
            traj_marker.colors.push_back(root_c);

            point.x = sub_root->pos_.x();
            point.y = sub_root->pos_.y();
            point.z = sub_root->pos_.z();
            traj_marker.points.push_back(point);
            traj_marker.colors.push_back(subroot_c);

            traj_marker.scale.x = 0.1;
            traj_marker.scale.y = 0.1;
            traj_marker.scale.z = 0.1;
        }

    }

protected:
    typename EuclideanDistanceRingBuffer<_N, _Datatype, _Scalar>::Ptr edrb_;
    typename PolynomialTrajectory3D<10, _Scalar>::Ptr trajectory_;
    UniformBSpline3D<_N, _Scalar> spline_;

    Vector3 start_, target_, height_, robot_pos;
    Affine3 robot_pose_;

    bool flat_height;
    bool debugging_;

    boost::thread* tra_gene_thread_;
    boost::mutex mutex;
    std::chrono::high_resolution_clock::time_point search_t_stamp;

    std::list<Vector3> obs_list;
    std::vector<Vector3> traj_points;
    std::list<PointBool> path_checker;

    Vector3 curr_start_pt, end_segment_point;
    _Scalar end_seg_t;

    bool flag_rrt_started, flag_rrt_finished;
    bool flag_not_enough;
    bool flag_hold_dt;
    bool flag_start_found, flag_stop_found;
    bool algorithm_;
    int obstacle_counter, traj_point_counter;
    bool flag_vizualize_output;
    bool flag_real_target;
    _Scalar dt_, reset_dt_;
    _Scalar current_t;
    _Scalar max_solve_t_;

    // RRT
    std::list<Node*> nodes_, x_sol_;
    Node *root_, *lastNode_, *goal_node;
    _Scalar rrt_factor_, radius_;
    std::list<Vector3> path_point_;
    std::vector<Edge> edges_;
    _Scalar step_size_;
    _Scalar rrt_gamma_;

    bool flag_sol_found, flag_rrt_running, flag_rewire_root;
    bool flag_new_path_selected;
    Node* solution_node, *temp_solution, *sub_root;
    std::vector<Node*> solution_queue;
    Vector3 last_point;
    std::mt19937 rng;

    // Ellipsoid Sampling
    _Scalar sampling_alpha, sampling_beta;
    _Scalar global_min_cost, best_cost_;
    Matrix3 C_rotation;


    // Log
    std::string delim = ",";
    std::string log_path_;
    std::fstream rrt_writer, ellipsoid_writer;
    std::fstream rrt_path_writer, rrt_tree_writer;
    bool flag_save_log_;
    int N_iter;
    int rrt_counter;
    int loop_counter;

};

}  // namespace ewok

#endif  // RRTSTAR3D_H
