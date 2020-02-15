#ifndef RRTSTAR3D_H
#define RRTSTAR3D_H

#include <Eigen/Eigen>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ewok/ed_ring_buffer.h>
#include <ewok/uniform_bspline_3d.h>
#include <algorithm>
#include <vector>
#include <iostream>
#include <memory>
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
    typedef Eigen::Matrix<int, 3, 1> Vector3i;
    typedef std::tuple<Vector3, Vector3, bool> Edge;


    typedef std::shared_ptr<RRTStar3D<_N, _Scalar>> Ptr;

    struct Node{

        std::vector<Node *> children_;
        Node *parent_ = NULL;
        Vector3 pos_;
        _Scalar cost_;
    };

    RRTStar3D(_Scalar step_size=0.5, _Scalar rrt_factor=1.5, _Scalar radius=1, _Scalar dt=0.5)
        : spline_(dt),
          step_size_(step_size),
          rrt_factor_(rrt_factor),
          radius_(radius),
          debugging_(false),
          running_(false),
          sampling_alpha(0.2),
          sampling_beta(2)
    {
        solved_ = false;
        index_point = 0;
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
        path_point_.clear();
        edges_.clear();

        root_ = new Node;
        root_->parent_ = NULL;
        root_->pos_= start_;
        root_->cost_ = 0;
        lastNode_ = root_;
        nodes_.push_back(root_);
    }

    void setDistanceBuffer(EuclideanDistanceRingBuffer<6>::Ptr & edrb) {
        edrb_ = edrb;
    }

    void addControlPoint(const Vector3 &point, int num = 1) {
        for (int i = 0; i < num; i++) {
            spline_.push_back(point);
        }
    }

    UniformBSpline3D<_N, _Scalar> getSpline(){return spline_;}

    bool isSolved() {return solved_;}

    bool isRunnning() {return running_;}

    std::list<Vector3> getPathPoints() {return path_point_;}

    Vector3 getFirstPathPoint() {return path_point_.front();}


    void setStartPoint(Vector3 start)
    {
        start_=start;
//        for(int i=0; i < _N/2; i++)
//            spline_.push_back(start);
        edges_.clear();

        initialize();

    }

    void setTargetPoint(Vector3 target)
    {
        target_=target;
        goal_node = new Node;
        goal_node->pos_ = target;
    }

    _Scalar getStepSize()
    {
//        return step_size_;
        return getRandomNumber(0.3*step_size_, 1.2*step_size_);
    }

    _Scalar getCost(Node* n)
    {
        return n->cost_;
    }

    _Scalar getDistCost(Node* p, Node* q)
    {
        return distance(q->pos_, p->pos_);
    }

    _Scalar distance(Vector3 &p1, Vector3 &p2)
    {
        return (p2-p1).norm();
    }

    bool isNear(Vector3 &point, _Scalar tol=10)
    {
        if(distance(point, target_) < tol)
            return true;
        return false;
    }

    bool isCollision(Node* p, Node* q)
    {
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
//            std::cout << "Collision Check: " << collision << "\n" << pt << std::endl;
            if(collision) break;
        }
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

    Vector3 LineSampling()
    {
        ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Getting Line");

        Node* near_n = getNearestNode(goal_node);
        ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Getting Nearest From Goal");

        Vector3 len = goal_node->pos_-near_n->pos_;
        ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Getting Random Line Point");

        _Scalar r = getRandomNumber(0,1);

        ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Randomly find Point Between");
        return near_n->pos_+r*len;
    }

    Vector3 UniformSampling()
    {
        Vector3 point_min, point_max, rand_point, center_point;
        Vector3i point_idx, center_idx;
        edrb_->getVolumeMinMax(point_min, point_max);
        center_idx = edrb_->getVolumeCenter();
        edrb_->getPointBuffer(center_idx, center_point);

        do{
            rand_point = Vector3(getRandomNumber(0.6*point_min.x(), point_max.x()),
                                 getRandomNumber(point_min.y(), point_max.y()),
                                 center_point.z());
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
            pos = LineSampling();
        }


        Node *rand_node = new Node;
        rand_node->pos_ = pos;
        return rand_node;
    }

    Node* getNearestNode(Node* node_)
    {
        _Scalar minDist = 1e9;
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
        // generate path
        tra_gene_thread_ = new boost::thread(boost::bind(&RRTStar3D::solve, this, iter));
        delete tra_gene_thread_;
    }

    void solve(int iter=5000)
    {
        running_ = true;
        mutex.lock();
        int counter=0;
        Node* final = NULL;
        path_point_.clear();
        bool found = false;
        solved_ = false;
        std::cout << "RRT FINDING" << std::endl;
        ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Starting RRT");
        while(counter<iter)
        {
            ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Getting Random Node");
            Node* rand_node = getRandomSampling();
//            std::cout << "Found Random Node: \n" << rand_node->pos_ << std::endl;
            if(rand_node)
            {
                ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Find Nearest");
                Node* nearest_node = getNearestNode(rand_node);
//                std::cout << "Found Nearest Node: \n" << nearest_node->pos_ << std::endl;

                ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Get Conf Node");
                Node* new_node = getConfigurationNode(rand_node, nearest_node);
//                std::cout << "Found New Node: \n" << new_node->pos_ << std::endl;

                if(!isCollision(nearest_node, new_node))
                {
                    ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Collision Free");

                    std::vector<Node*> near_nodes;
                    getNearestNodes(new_node, getStepSize(), near_nodes);

                    Node* min_node = nearest_node;
                    _Scalar min_cost = getCost(nearest_node) + getDistCost(nearest_node, new_node);
                    for(auto p: near_nodes)
                    {
                        if(!isCollision(p, new_node) && (getCost(p) + getDistCost(p, new_node)) < min_cost)
                        {
                            min_node = p; min_cost=getCost(p) + getDistCost(p, new_node);
                        }
                    }

                    new_node->parent_=min_node;
                    new_node->cost_ = min_node->cost_ + getDistCost(min_node, new_node);
                    min_node->children_.push_back(new_node);
                    edges_.push_back(std::make_tuple(min_node->pos_, new_node->pos_, false));
                    nodes_.push_back(new_node);
                    lastNode_ = new_node;

                    for(auto p: near_nodes)
                    {
                        if(!isCollision(new_node,p) && (getCost(new_node) + getDistCost(new_node, p)) < p->cost_)
                        {
                            Node* n_parent = p->parent_;
                            n_parent->children_.erase(std::remove(n_parent->children_.begin(), n_parent->children_.end(), p), n_parent->children_.end());
                            edges_.erase(std::remove(edges_.begin(), edges_.end(), std::make_tuple(n_parent->pos_, new_node->pos_, false)), edges_.end());
                            p->cost_=getCost(new_node)+getDistCost(new_node, p);
                            p->parent_ = new_node;
                            new_node->children_.push_back(p);
                            edges_.push_back(std::make_tuple(p->pos_, new_node->pos_, false));

                        }
                    }

                }
//                else
//                {
//                    ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Collision Found");
//                    edges_.push_back(std::make_tuple(nearest_node->pos_, new_node->pos_, true));
//                }
            }

            if(isNear(lastNode_->pos_, step_size_))
            {
                ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Target Found");
                std::cout << "Source: \n" << start_ << std::endl;
                std::cout << "Nearest: \n"<< lastNode_->pos_ << std::endl;
                std::cout << "Target: \n" << target_ << std::endl;
                final = lastNode_;
                found = true;
                break;
            }

            counter++;
        }

        mutex.unlock();

        if(found)
            for(int i=0; i < 3; i++)
            {
                path_point_.push_back(target_);
            }

        else
        {
            ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Target Not Found, Get Nearest");
            Node* near = new Node;
            near->pos_ = target_;
            Node* new_node = getNearestNode(near);
            path_point_.push_back(new_node->pos_);
            final = lastNode_;
            ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Found Nearest to Target");
            std::cout << "Source: \n" << start_ << std::endl;
            std::cout << "Nearest: \n"<< new_node->pos_ << std::endl;
            std::cout << "Target: \n" << target_ << std::endl;

        }

        while(final != NULL)
        {
            ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Generating Path");
            Vector3 pos = final->pos_;
            path_point_.push_front(pos);
            final = final->parent_;
        }

        for(int i=0; i < 3; i++)
        {
            path_point_.push_front(start_);
        }

        running_ = false;
        solved_=true;

//        for(auto p: path_point_)
//        {
//            spline_.push_back(p);
//        }
//        for(int i=0; i < _N/2; i++)
//            spline_.push_back(target_);

        ROS_DEBUG_COND_NAMED(debugging_, "RRT PLANNER", "Path Ready");
    }

    void getTreeMarker(visualization_msgs::Marker &traj_marker, const std::string &ns,
                                int id=0, const Eigen::Vector3d &color = Eigen::Vector3d(1,1,0), double scale = 0.01)
    {
        traj_marker.header.frame_id = "world";
        traj_marker.ns = ns;
        traj_marker.id = id;
        traj_marker.type = visualization_msgs::Marker::LINE_LIST;
        traj_marker.action = visualization_msgs::Marker::MODIFY;
        traj_marker.scale.x = scale;
//        traj_marker.scale.y = scale;
//        traj_marker.scale.z = scale;

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
//                traj_marker.colors.push_back(c_obs);
            }
            else
            {
                traj_marker.points.push_back(p_);
                traj_marker.points.push_back(q_);
//                traj_marker.colors.push_back(c_free);
            }
        }
    }

    void getSolutionMarker(visualization_msgs::Marker &traj_marker, const std::string &ns,
                                int id=0, const Eigen::Vector3d &color = Eigen::Vector3d(1,1,0), double scale = 0.05)
    {
        traj_marker.header.frame_id = "world";
        traj_marker.ns = ns;
        traj_marker.id = id;
        traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
        traj_marker.action = visualization_msgs::Marker::MODIFY;
        traj_marker.scale.x = scale;
//        traj_marker.scale.y = scale;
//        traj_marker.scale.z = scale;
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

    UniformBSpline3D<_N, _Scalar> spline_;

    Vector3 start_, target_;
    _Scalar step_size_;
    std::vector<Node *> nodes_;
    Node *root_, *lastNode_, *goal_node;
    _Scalar rrt_factor_, radius_;
    std::list<Vector3> path_point_;
    std::vector<Edge> edges_;
    bool solved_, running_;
    bool debugging_;

    boost::thread *tra_gene_thread_;
    boost::mutex mutex;

    _Scalar sampling_alpha, sampling_beta;
    int index_point;




};

}


#endif // RRTSTAR3D_H
