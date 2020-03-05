#ifndef RRTSTAR3D_H
#define RRTSTAR3D_H

#include <Eigen/Eigen>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ewok/ed_ring_buffer.h>
#include <ewok/uniform_bspline_3d.h>
#include <ewok/polynomial_trajectory_3d.h>
#include <algorithm>
#include <vector>
#include <iostream>
#include <memory>
#include <chrono>
#include <random>
#include <ros/ros.h>
#include <ros/console.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <functional>

namespace ewok {

template<int _N, typename _Scalar = float>
class RRTStar3D
{
public:
    typedef Eigen::Matrix<_Scalar, 3, 1> Vector3;
    typedef Eigen::Matrix<_Scalar, 3, 3> Matrix3;
    typedef Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;

    typedef Eigen::Matrix<int, 3, 1> Vector3i;
    typedef std::tuple<Vector3, Vector3, bool> Edge;


    typedef std::shared_ptr<RRTStar3D<_N, _Scalar>> Ptr;

    struct Node{

        std::vector<Node *> children_;
        Node *parent_ = NULL;
        Vector3 pos_;
        _Scalar cost_;
    };

    RRTStar3D(ewok::PolynomialTrajectory3D<10>::Ptr & trajectory, _Scalar step_size=0.5, _Scalar rrt_factor=1.5, _Scalar radius=1, _Scalar solve_tmax=1,_Scalar dt=0.5)
        : trajectory_(trajectory),
          spline_(dt),
          step_size_(step_size),
          rrt_factor_(rrt_factor),
          radius_(radius),
          debugging_(false),
          running_(false),
          request_(false),
          flat_height(false),
          allowed_search(false),
          sampling_alpha(0.2),
          sampling_beta(1.5),
          cp_opt_start_idx(_N),
          max_solve_t_(solve_tmax)
    {
        solved_ = false;
        immidiate_path_ = false;
        get_immidiate_path = false;

        max_neighbours_ = 5;
        max_radius_ = 1.5;
    }

    void reset()
    {
        nodes_.clear();
        path_point_.clear();
        edges_.clear();
        solved_ = false;
    }

    void initialize()
    {
        nodes_.clear();
        edges_.clear();

        root_ = new Node;
        root_->parent_ = NULL;
        root_->pos_= start_;
        root_->cost_ = 0;
        lastNode_ = root_;
        nodes_.push_back(root_);

        goal_node = new Node;
    }

    void setRobotPos(const Vector3 &pos)
    {
        robot_pos = pos;

        if(!allowed_search && running_)
            if(distance(robot_pos, start_) < 1)
                allowed_search = true;

        if(running_ && immidiate_path_)
            if(distance(robot_pos, immidiate_point_) < 1.5)
                immidiate_path_ = false;

    }

    void setNumControlPointsOptimized(int n) {
      num_cp_opt = n;
    }

    void setDistanceBuffer(EuclideanDistanceRingBuffer<6>::Ptr & edrb) {
        edrb_ = edrb;
    }

    void addControlPoint(const Vector3 &point, int num = 1) {
        for (int i = 0; i < num; i++) {
                spline_.push_back(point);
        }
    }

    void addLastControlPoint() {
        if(cp_opt_start_idx < spline_.size()-1)
            cp_opt_start_idx++;
    }

    Vector3 getFirstTrajPoint() {
        return spline_.getControlPoint(cp_opt_start_idx);
    }

    UniformBSpline3D<_N, _Scalar> getSpline(){return spline_;}

    bool isSolved() {return solved_;}

    bool isRunnning() {return running_;}

    bool immidiatePath() {return get_immidiate_path;}

    std::list<Vector3> getPathPoints() {return path_point_;}
    std::list<Vector3> getImmidiatePath() {return path_point_;}
    void resetImmidiatePath() {get_immidiate_path = false;}

    void setStartPoint(Vector3 start)
    {
        if(running_)
        {
            temp_start_ = start;
            request_ = true;
        }
        else {
            start_=start;
            edges_.clear();
        }


        initialize();

    }

    void setTargetPoint(Vector3 target, bool real_target=true)
    {

        real_target_ = real_target;
        target_=target;
        goal_node->pos_ = target;
        global_min_cost = distance(start_, target_);

//            Vector3 x_center = (start_+target_)/2;
//            Vector3 id1 = Vector3(1,0,0);

//            Vector3 a_1 = (target_-start_)/Vector3(target_-start_).norm();
//            Matrix3 M = a_1*id1.transpose();

//            Eigen::JacobiSVD<MatrixX> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);

//            Eigen::DiagonalMatrix<_Scalar, 3> diag(1, 1, svd.matrixU().determinant()*svd.matrixV().determinant());

//            C_rotation = svd.matrixU() * diag * svd.matrixV().transpose();
//        }


    }

    void setHeight(Vector3 point, bool status=true)
    {
        flat_height=status;
        height_ = point;
    }

    _Scalar getStepSize()
    {
        return step_size_;
    }

    _Scalar getCost(Node* n)
    {
        if(n->parent_ == NULL)
            return n->cost_;
        else {
            return n->cost_+getCost(n->parent_);
        }
    }

    _Scalar getDistCost(Node* p, Node* q)
    {
        return distance(q->pos_, p->pos_);
    }

    _Scalar distance(Vector3 &p1, Vector3 &p2)
    {
        return (p2-p1).norm();
    }

    bool isNear(Vector3 &point, _Scalar tol=2)
    {
        if(distance(point, target_) < tol)
            return true;
        return false;
    }

    bool isCollision(Node* p, Node* q)
    {
        mutex.lock();
        if(!edrb_.get())
        {
            ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "EDRB ERROR");
            return true;
        }
        bool collision=false;

        std::vector<Vector3> point_check;

        Vector3 len = (q->pos_-p->pos_);
        point_check.push_back(p->pos_+0*len);
        point_check.push_back(p->pos_+0.5*len);
        point_check.push_back(p->pos_+1*len);

        for(Vector3 pt: point_check)
        {
            collision = edrb_->isNearObstacle(pt, radius_);
            if(collision) break;
        }

        mutex.unlock();
    \
        return collision;
    }

    _Scalar getRandomNumber(_Scalar a, _Scalar b)
    {
        std::random_device rseed;
        std::mt19937 rng(rseed());
        std::uniform_real_distribution<_Scalar> dist(a,b);
        _Scalar num = dist(rng);

        return num;
    }

    Vector3 EllipsoidSampling()
    {
        double phi = getRandomNumber(0,2*M_PI);
        double costheta = getRandomNumber(-1,1);
        double r = std::cbrt(getRandomNumber(0,1));

        double theta = acos( costheta );

        double x = r * sin( theta) * cos( phi );
        double y = r * sin( theta) * sin( phi );
        double z = r * cos( theta );

        Node* rand = new Node;
        rand->position_ = Vector3(x,y,z);
        return  rand;
    }

    Vector3 LineSampling()
    {
        Node* near_n = getNearestNode(goal_node);
        Vector3 len = goal_node->pos_-near_n->pos_;
        len=len/len.norm();

        Vector3 point = near_n->pos_+len*getStepSize();
        Vector3i point_idx;
        edrb_->getIdxBuffer(point, point_idx);
        if(edrb_->insideVolume(point) && !edrb_->isOccupied(point_idx))
            return point;
        else {
            return UniformSampling();
        }
    }

    Vector3 UniformSampling()
    {
        Vector3 point_min, point_max, rand_point, center_point;
        Vector3i point_idx, center_idx;
        edrb_->getVolumeMinMax(point_min, point_max);
        center_idx = edrb_->getVolumeCenter();
        edrb_->getPointBuffer(center_idx, center_point);

        do{
            if(flat_height)
                rand_point = Vector3(getRandomNumber(point_min.x(), point_max.x()),
                                     getRandomNumber(point_min.y(), point_max.y()),
                                     height_.z());
            else {
                rand_point = Vector3(getRandomNumber(point_min.x(), point_max.x()),
                                     getRandomNumber(point_min.y(), point_max.y()),
                                     getRandomNumber(point_min.z(), point_max.z()));
            }
            edrb_->getIdxBuffer(rand_point, point_idx);

        }while(edrb_->isOccupied(point_idx));

        return rand_point;
    }

    Node* getRandomSampling()
    {
        Vector3 pos;
        _Scalar P_r = getRandomNumber(0,1);

        if(P_r > 1-sampling_alpha)
        {
            ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Line Sampling");
            pos = LineSampling();
        }

        else if(P_r <= (1-sampling_alpha)/sampling_beta)
        {
            ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Uniform Sampling");
            pos = UniformSampling();
        }

        else {
            ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Ellipsoid");
            pos = UniformSampling();
        }


        Node *rand_node = new Node;
        rand_node->pos_ = pos;
        return rand_node;
    }

    Node* getNearestNodeDistance(Node* node_, _Scalar max_distance=1)
    {
        _Scalar minCost = 0;
        _Scalar minDist = distance(node_->pos_, target_);
        Node* closest = new Node;

        for(int i=0; i<(int)nodes_.size(); i++)
        {
            _Scalar dist2origin = distance(node_->pos_, nodes_[i]->pos_);
            _Scalar dist2target = distance(nodes_[i]->pos_, target_);
            _Scalar cost = getCost(nodes_[i]);
            if(dist2origin < max_distance && dist2target < minDist && cost > minCost)
            {
                minDist = dist2target;
                minCost = cost;
                closest = nodes_[i];
            }
        }
        return closest;
    }

    Node* getNearestNode(Node* node_)
    {
        _Scalar minDist = std::numeric_limits<_Scalar>::max();
        Node* closest = NULL;

        for(int i=0; i<(int)nodes_.size(); i++)
        {
            _Scalar dist = distance(node_->pos_, nodes_[i]->pos_);
            if(dist < minDist)
            {
                minDist = dist;
                closest = nodes_[i];
            }
        }
        return closest;
    }


    void getNearestNodes(Node *node, _Scalar radius, std::vector<Node *> &near)
    {
        for(auto n: nodes_)
        {
            _Scalar dist = distance(node->pos_, n->pos_);
            if(dist < radius*rrt_factor_)
                near.push_back(n);
        }
    }

    Node* getConfigurationNode(Node* q, Node* nearest_)
    {
        Vector3 q_pos = q->pos_;
        Vector3 near_pos = nearest_->pos_;
        Vector3 midPos = q_pos-near_pos;
        midPos = midPos/midPos.norm();

        Node* node_ = new Node;
        node_->pos_ = near_pos+getStepSize()*midPos;
        return node_;
    }

    void findPath(int iter = 2000)
    {
        if(request_)
        {
            start_=temp_start_;
            edges_.clear();
            initialize();

            request_ = false;

            global_min_cost = distance(start_, target_);

            // generate path
            tra_gene_thread_ = new boost::thread(boost::bind(&RRTStar3D::solve, this, iter));
            delete tra_gene_thread_;

        }

        else
        {
            // generate path
            tra_gene_thread_ = new boost::thread(boost::bind(&RRTStar3D::solve, this, iter));
            delete tra_gene_thread_;
        }
    }

    Node* chooseParent(Node* nearest_node, Node* new_node, std::vector<Node*> near_nodes)
    {
        Node* min_node = nearest_node;
        _Scalar min_cost = getCost(nearest_node) + getDistCost(nearest_node, new_node);
        for(auto p: near_nodes)
        {
            _Scalar new_cost = getCost(p) + getDistCost(p, new_node);
            if(!isCollision(p, new_node) && new_cost < min_cost)
            {
                min_node = p; min_cost=new_cost;
            }
        }

        return min_node;
    }

    void reWireTree(Node* min_node, Node* new_node, std::vector<Node*> near_nodes)
    {
        for(Node* x_near: near_nodes)
        {
            _Scalar cost_old = x_near->cost_; _Scalar cost_new = getCost(new_node) + getDistCost(new_node, x_near);
            if(!isCollision(new_node,x_near) && cost_new < cost_old)
            {
                Node* n_parent = x_near->parent_;
                n_parent->children_.erase(std::remove(n_parent->children_.begin(), n_parent->children_.end(), x_near), n_parent->children_.end());
                edges_.erase(std::remove(edges_.begin(), edges_.end(), std::make_tuple(n_parent->pos_, new_node->pos_, false)), edges_.end());

                x_near->cost_=cost_new;
                x_near->parent_ = new_node;
                new_node->children_.push_back(x_near);
                edges_.push_back(std::make_tuple(x_near->pos_, new_node->pos_, false));

            }
        }
    }

    void reWireRoot()
    {
        if(sol_queue_.empty())
            sol_queue_.push_back(root_);

        do{
            ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "ReWire Root");

            _Scalar radius = sqrt((max_radius_*max_neighbours_)/M_PI*nodes_.size());
            if (radius < step_size_) radius = step_size_;

            Node* x_s = sol_queue_.front();
            std::vector<Node*> near_nodes;
            getNearestNodes(x_s, M_PI*radius*radius, near_nodes);

            for(auto x_near: near_nodes)
            {
                _Scalar old_cost = getCost(x_near);
                _Scalar new_cost = getCost(x_s) + getDistCost(x_s, x_near);
                if(!isCollision(x_s, x_near) && new_cost < old_cost)
                {
                    Node* n_parent = x_near->parent_;
                    n_parent->children_.erase(std::remove(n_parent->children_.begin(), n_parent->children_.end(), x_near), n_parent->children_.end());
                    edges_.erase(std::remove(edges_.begin(), edges_.end(), std::make_tuple(n_parent->pos_, x_near->pos_, false)), edges_.end());

                    x_near->cost_=new_cost;
                    x_near->parent_ = x_s;
                    x_s->children_.push_back(x_near);
                    edges_.push_back(std::make_tuple(x_near->pos_, x_s->pos_, false));
                }
            }

        }
        while(!sol_queue_.empty());

    }

    void reWireRandomNode()
    {

        do
        {
            _Scalar radius = sqrt((max_radius_*max_neighbours_)/M_PI*nodes_.size());
            if (radius < step_size_) radius = step_size_;

            Node* x_r = rand_queue_.front();
            std::vector<Node*> near_nodes;
            getNearestNodes(x_r, M_PI*radius*radius, near_nodes);

            for(auto x_near: near_nodes)
            {
                _Scalar old_cost = getCost(x_near);
                _Scalar new_cost = getCost(x_r) + getDistCost(x_r, x_near);
                if(!isCollision(x_r, x_near) && new_cost < old_cost)
                {
                    Node* n_parent = x_near->parent_;
                    n_parent->children_.erase(std::remove(n_parent->children_.begin(), n_parent->children_.end(), x_near), n_parent->children_.end());
                    edges_.erase(std::remove(edges_.begin(), edges_.end(), std::make_tuple(n_parent->pos_, x_near->pos_, false)), edges_.end());

                    x_near->cost_=new_cost;
                    x_near->parent_ = x_r;
                    x_r->children_.push_back(x_near);
                    edges_.push_back(std::make_tuple(x_near->pos_, x_r->pos_, false));

                    rand_queue_.push_back(x_r);
                }
            }
        }
        while(!rand_queue_.empty());
    }

    void TreeExpansionRewire()
    {
        Node* rand_node = getRandomSampling();
        if(rand_node)
        {
            ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Find Nearest");
            Node* nearest_node = getNearestNode(rand_node);

            ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Get Conf Node");
            Node* new_node = getConfigurationNode(rand_node, nearest_node);

            if(!isCollision(nearest_node, new_node))
            {
                std::vector<Node*> near_nodes;
                getNearestNodes(new_node, getStepSize(), near_nodes);

                if(near_nodes.size() < max_neighbours_ ||
                        distance(new_node, nearest_node) > getStepSize())
                {
                    Node* min_node = chooseParent(nearest_node, new_node, near_nodes);
                    InsertNode(min_node, new_node);
                    rand_queue_.push_back(new_node);
                }
                else {
                    rand_queue_.push_back(nearest_node);
                }
            }
        }
    }

    void InsertNode(Node* min_node, Node* new_node)
    {
        new_node->parent_=min_node;
        new_node->cost_ = min_node->cost_ + getDistCost(min_node, new_node);
        min_node->children_.push_back(new_node);
        edges_.push_back(std::make_tuple(min_node->pos_, new_node->pos_, false));
        nodes_.push_back(new_node);
        lastNode_ = new_node;
    }

    void solve(int iter=5000)
    {
        running_ = true;
        allowed_search = false;
        int counter=0;
        Node* final = NULL;
        path_point_.clear();
        bool found = false;
        solved_ = false;
        ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Starting RRT");

        while(!allowed_search)
        {
            ROS_WARN("WAITING FOR READY");
        }

        start_solving = std::chrono::high_resolution_clock::now();
        while(counter<iter)
        {

            ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Getting Random Node");
            mutex.lock();
            Node* rand_node = getRandomSampling();
            mutex.unlock();
            if(rand_node)
            {
                ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Find Nearest");
                mutex.lock();
                Node* nearest_node = getNearestNode(rand_node);

                ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Get Conf Node");
                Node* new_node = getConfigurationNode(rand_node, nearest_node);
                mutex.unlock();
                if(!isCollision(nearest_node, new_node))
                {
                    std::vector<Node*> near_nodes;
                    ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Get Nearest Nodes");

                    getNearestNodes(new_node, getStepSize(), near_nodes);

                    ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Find Parent");

                    Node* min_node = chooseParent(nearest_node, new_node, near_nodes);

                    InsertNode(min_node, new_node);

                    ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Rewire Tree");
                    reWireTree(min_node, new_node, near_nodes);

                }

            }

            if(real_target_)
            {
                if(isNear(lastNode_->pos_, step_size_*rrt_factor_))
                {
                    ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Target Found");
                    std::cout << "Source: \n" << start_ << std::endl;
                    std::cout << "Nearest: \n"<< lastNode_->pos_ << std::endl;
                    std::cout << "Target: \n" << target_ << std::endl;
                    final = lastNode_;
                    found = true;
                    break;
                }
//                else {
//                    Node* nearest_node = getNearestNode(goal_node);
//                    if(isNear(nearest_node->pos_, step_size_*rrt_factor_))
//                    {
//                        ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Target Found");
//                        std::cout << "Source: \n" << start_ << std::endl;
//                        std::cout << "Nearest: \n"<< lastNode_->pos_ << std::endl;
//                        std::cout << "Target: \n" << target_ << std::endl;
//                        final = nearest_node;
//                        found = true;
//                        break;
//                    }
//                }
            }

            std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start_solving;

//             Create immidiate path
            if((elapsed.count() > max_solve_t_) && !immidiate_path_ && !get_immidiate_path)
            {
                ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Create Immidiate Path");

                path_point_.clear();
                immidiate_path_ = true;
                get_immidiate_path = true;
                Node* near = new Node;
                near->pos_ = target_;
                ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Get Nearest Node By Distance");
                mutex.lock();
                Node* new_node = getNearestNodeDistance(root_, 2.5);
//                path_point_.push_back(new_node->pos_);
                final = new_node;
                mutex.unlock();

                start_ = new_node->pos_;
                immidiate_point_ = start_;

                ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Generating Path");
                while(final != NULL)
                {
                    Vector3 pos = final->pos_;
                    path_point_.push_front(pos);
                    final = final->parent_;
                }

                while(immidiate_path_)
                {
                    ROS_WARN("Moving to Immidiate Path");
                }

                ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Recreate Tree");

                nodes_.clear();
                edges_.clear();

                root_ = new Node;
                root_->parent_ = NULL;
                root_->pos_= start_;
                root_->cost_ = 0;
                lastNode_ = root_;
                nodes_.push_back(root_);
                counter = 0;
                start_solving = std::chrono::high_resolution_clock::now();

            }


            counter++;
        }


        path_point_.clear();

        if(!found)
        {
            ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Target Not Found, Get Nearest");
            Node* near = new Node;
            near->pos_ = target_;
            Node* new_node = getNearestNode(near);
            path_point_.push_back(new_node->pos_);
            final = lastNode_;
            ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Found Nearest to Target");
        }

        while(final != NULL)
        {
            ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Generating Path");
            Vector3 pos = final->pos_;
            path_point_.push_front(pos);
            final = final->parent_;
        }

            path_point_.push_front(start_);

        running_ = false;
        solved_=true;

        ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Path Ready");
    }

    void getTrajectoryMarkers(visualization_msgs::MarkerArray & traj_marker,
                              const std::string & ns = "spline_opitimization_markers",
                              const Eigen::Vector3d & color1 = Eigen::Vector3d(0,1,0),
                              const Eigen::Vector3d & color2 = Eigen::Vector3d(0,1,1)) {

        traj_marker.markers.resize(2);

        spline_.getVisualizationMarker(traj_marker.markers[0], ns, 0, color1, cp_opt_start_idx, num_cp_opt, color2);
        spline_.getControlPointsMarker(traj_marker.markers[1], ns, 1, color1, cp_opt_start_idx, num_cp_opt, color2);


    }

    void getTreeMarker(visualization_msgs::Marker &traj_marker, const std::string &ns,
                                int id=0, const Eigen::Vector3f &color = Eigen::Vector3f(1,1,0), double scale = 0.01)
    {
        if(edges_.empty()) return;
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

        for(int i=0; i < edges_.size()-1; i++)
        {
            Vector3 p,q;
            p=std::get<0>(edges_[i]); q=std::get<1>(edges_[i]);

            geometry_msgs::Point p_, q_;
            p_.x = p.x(); p_.y = p.y(); p_.z = p.z();
            q_.x = q.x(); q_.y = q.y(); q_.z = q.z();

            if(std::get<2>(edges_[i]))
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

    void getSolutionMarker(visualization_msgs::Marker &traj_marker, const std::string &ns,
                                int id=0, const Eigen::Vector3f &color = Eigen::Vector3f(0,1,0), double scale = 0.01)
    {
        if(path_point_.empty()) return;

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

        for(Vector3 n: path_point_)
        {
            geometry_msgs::Point p;
            p.x = n.x(); p.y=n.y(); p.z=n.z();

            traj_marker.points.push_back(p);
        }
    }

protected:
    EuclideanDistanceRingBuffer<6>::Ptr edrb_;
    ewok::PolynomialTrajectory3D<10>::Ptr trajectory_;

    UniformBSpline3D<_N, _Scalar> spline_;

    Vector3 start_, temp_start_, target_, temp_target_, height_, robot_pos, immidiate_point_;
    _Scalar step_size_;
    std::vector<Node* > nodes_;
    Node *root_, *lastNode_, *goal_node;
    _Scalar rrt_factor_, radius_;
    std::list<Vector3> path_point_;
    std::list<Node* > rand_queue_, sol_queue_;
    std::vector<Edge> edges_;
    bool request_, solved_, running_, immidiate_path_, get_immidiate_path, real_target_;
    bool flat_height, allowed_search;
    bool debugging_;

    boost::thread *tra_gene_thread_;
    boost::mutex mutex;

    _Scalar sampling_alpha, sampling_beta;
    int  num_cp_opt, cp_opt_start_idx;

    _Scalar max_neighbours_, max_radius_, max_solve_t_;

    _Scalar global_min_cost, global_best_cost;
    Matrix3 C_rotation;

    std::chrono::high_resolution_clock::time_point start_solving, end_solving;

};

}


#endif // RRTSTAR3D_H
