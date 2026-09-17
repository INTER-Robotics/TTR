/*
 * This file is part of ttr.
 *
 * Copyright (C) 2025 Antoni Tauler-Rosselló <a.tauler@uib.eu> (University of the Balearic Islands)
 *
 * ttr is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ttr is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ttr. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef INCLUDE_TTR_PLANNER_H
#define INCLUDE_TTR_PLANNER_H

#include <dynamic_reconfigure/server.h>
#include <math.h>
// #include <planners_3d/a_star_plannerConfig.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdio.h>
#include <stdlib.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h> // from rpy to quaternion
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <time.h>

#include <fstream>
#include <iostream>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <queue>
#include <random>
#include <string>
#include <vector>
#include <unordered_map>

#include "common_utils.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "msc/msc"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include "ttr/node.h"
#include "ttr/branch.h"
#include "ttr/treeGraph.h"

#include <ttr/MakePlan.h>
#include <ttr/PathLengths.h>

#include <srv_mav_behaviours/StartFollowPath.h>

// #include <voxblox_ros/esdf_server.h>
#include <Eigen/Dense>
// #include <voxblox/core/common.h>
// #include <voxblox/core/tsdf_map.h>
// #include <voxblox/utils/distance_utils.h>



namespace ttr {


// Pointclouds
// pcl::PointCloud<pcl::PointXYZ>::Ptr trav_pc(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr local_pc(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr obs_pc(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr tr_gr(new pcl::PointCloud<pcl::PointXYZ>);

struct GraphNode {
    bool opened = false;
    bool computed = false;
    GraphNode* parent = nullptr;
    ttr::node node;
    double g_cost = 0.0;
    double f_cost = 0.0;
    double h_cost = 0.0;

    GraphNode() {}

    GraphNode(bool opened, bool computed, GraphNode* parent, ttr::node node,
              double g_cost, double f_cost, double h_cost)
        : opened(opened), computed(computed), parent(parent), node(node),
          g_cost(g_cost), f_cost(f_cost), h_cost(h_cost) {}

    // GraphNode(bool opened_ = false, bool computed_ = false, GraphNode* parent_ = nullptr,
    //           ttr::node node_, double g_cost_ = 0.0, double f_cost_ = 0.0, double h_cost_ = 0.0)
    //     : opened(opened_), computed(computed_), parent(parent_), node(node_), g_cost(g_cost_), f_cost(f_cost_), h_cost(h_cost_)
    // {}

    GraphNode(ttr::node node_) : node(node_) {}
    void reset(){
        opened = false;
        computed = false;
        parent = nullptr;
        g_cost = 0.0;
        f_cost = 0.0;
        h_cost = 0.0;
    }
};


struct CompareHeuristics {
    bool operator()(const GraphNode* c1, const GraphNode* c2) {
        // double equals
        if (abs(c1->f_cost - c2->f_cost) < 0.000000001) {
            return c1->h_cost > c2->h_cost;
        }
        return c1->f_cost > c2->f_cost;
    }
};


class Planner {
   public:
    explicit Planner(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private);
    virtual ~Planner();

    void configure();
    // double getMapDistance(const Eigen::Vector3d& position) const;


   private:
    // ROS variables
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    voxblox::EsdfServer voxblox_server_;
    // voxblox::TsdfServer tsdf_server_;

    // Subscribers
    //  ros::Subscriber octomap_sub_;
    ros::Subscriber obs_pc_sub_;
    ros::Subscriber tree_graph_sub_;

    // Publishers
    ros::Publisher path_pub_;
    ros::Publisher vis_path_pub_;
    ros::Publisher raw_path_pub_;
    ros::Publisher pc_pub_;

    ros::Publisher goal_pub_;


    ros::Timer timer_;
    // ros::Timer check_timer_;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    // Services
    ros::ServiceServer makePlan_;
    bool makePlan(ttr::MakePlan::Request& req, ttr::MakePlan::Response& res);
    ros::ServiceServer getFrontierPathLengths_;
    bool getFrontierPathLengths(ttr::PathLengths::Request& req, ttr::PathLengths::Response& res);

    ros::ServiceClient start_follow_path_client_;


    // Params

    // Map limits

    // Global Variables
    // std::vector<ttr::node> tree_graph;
    geometry_msgs::Point current_position;
    std::unordered_map<uint32_t, ttr::node> tree;
    std::unordered_map<uint32_t, GraphNode> tree_graph;
    std::priority_queue<GraphNode*, std::vector<GraphNode*>, CompareHeuristics> opened;
    bool new_tree_graph;
    bool obs_received = false;
    bool ground_robot;
    double proj_height;
    double max_slope;

    double init_time;

    std::string fixed_frame;
    std::string base_frame;
    double margin;
    double map_res;

    GraphNode* near_start;
    GraphNode* near_goal;
    geometry_msgs::Point start_, goal_;


    // KD-Trees
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    // pcl::KdTreeFLANN<pcl::PointXYZ> trav_kdtree;
    pcl::KdTreeFLANN<pcl::PointXYZ> local_kdtree;
    pcl::KdTreeFLANN<pcl::PointXYZ> obs_kdtree;

    // Visualization

    // Services

    // Reconfigure
    // dynamic_reconfigure::Server<ttr::validatorConfig> reconfigure_server_;
    // void dynReconfig(ttr::a_star_plannerConfig& config, uint32_t level);

    // Callbacks
    void timerClb(const ros::TimerEvent& event);
    // void traversablePointCloudClb(const sensor_msgs::PointCloud2ConstPtr& t_pc_msg);
    void localPointCloudClb(const sensor_msgs::PointCloud2ConstPtr& loc_pc_msg);
    void obstaclePointCloudClb(const sensor_msgs::PointCloud2ConstPtr& obs_pc_msg);
    void treeGraphClb(const ttr::treeGraph::ConstPtr &treegraph_msg);

    // Other methods
    nav_msgs::Path performMakePlan(geometry_msgs::Point start, geometry_msgs::Point goal, bool navigation, bool raw_path);

    std::vector<GraphNode*> getNeighbours(ttr::node node);
    bool computeNeighbours(GraphNode &g_node);
    double gCost(GraphNode parent, GraphNode neighbour);
    double hCost(GraphNode neighbour);
    void insertPointToPath(const geometry_msgs::Point& point, const std::string& fixed_fr, nav_msgs::Path& path, bool back);
    void pathGenerator(nav_msgs::Path &path);
    void pathOptimizer(nav_msgs::Path &path);
    void pathSmoothChaikin(nav_msgs::Path &path, uint8_t iterations, float percentage);

    double pathLength(const nav_msgs::Path &path);


};

}  // namespace ttr

#endif  // INCLUDE_TTR_VALIDATOR_H