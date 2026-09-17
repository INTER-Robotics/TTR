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

#ifndef INCLUDE_TTR_PERSISTENT_TREE_H
#define INCLUDE_TTR_PERSISTENT_TREE_H

#include <dynamic_reconfigure/server.h>
#include <math.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <fstream>
#include <iostream>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <queue>
#include <random>
#include <string>
#include <vector>
#include <unordered_map>

#include "common_utils.h"
#include "geometry_msgs/Point.h"
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

#include <voxblox_ros/esdf_server.h>
#include <Eigen/Dense>
#include <voxblox/core/common.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/utils/distance_utils.h>

#include <std_srvs/Empty.h>


namespace ttr {


// Pointclouds
pcl::PointCloud<pcl::PointXYZ>::Ptr trav_pc(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr obs_pc(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr tr_gr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_pc(new pcl::PointCloud<pcl::PointXYZ>);


// struct PointHasher {
//   std::size_t operator()(const geometry_msgs::Point& p) const {
//     std::size_t h1 = std::hash<double>()(p.x);
//     std::size_t h2 = std::hash<double>()(p.y);
//     std::size_t h3 = std::hash<double>()(p.z);
//     return (h1 ^ (h2 << 1)) ^ (h3 << 2);
//   }
// };

// struct PointEqual {
//   bool operator()(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) const {
//     return p1.x == p2.x && p1.y == p2.y && p1.z == p2.z;
//   }
// };

class PersistentTree {
   public:
    explicit PersistentTree(const ros::NodeHandle& nh,
                           const ros::NodeHandle& nh_private);
    virtual ~PersistentTree();

    void configure();

   private:
    // ROS variables
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    voxblox::EsdfServer voxblox_server_;

    //  ros::Subscriber octomap_sub_;
    ros::Subscriber raw_points_sub_;
    ros::Subscriber trav_pc_sub_;
    ros::Subscriber obs_pc_sub_;
    ros::Subscriber local_branch_sub_;
    ros::Subscriber exploration_state_sub_;

    ros::Publisher lines_pub_;
    ros::Publisher frontier_pub_;
    ros::Publisher graph_pub_;
    ros::Publisher points_pub_;

    ros::Timer timer_;
    ros::Timer check_timer_;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    // Params

    // Map limits

    // Global Variables
    // std::vector<Node> tree_graph;
    std::unordered_map<uint32_t, ttr::node> tree;
    // std::unordered_map<geometry_msgs::Point, ttr::node, PointHasher, PointEqual> tree;
    uint32_t last_key;

    std::vector<ttr::node> keypoints_tree;
    

    std::string fixed_frame;
    std::string base_frame;
    double max_x, min_x, max_y, min_y, max_z, min_z;  // Exploration Delimitation
    double margin;
    double step;
    double map_res;
    double rad_gain;
    double info_gain;
    bool imp_search = false;
    bool ground_robot = false;
    double proj_height;
    double max_slope;

    double init_time;
    double last_print;
    double last_trav_pc = 0.0;

    bool check_path_new_connections = false;
    bool original_add_branch = true;
    double n_factor = 1.5; // 1.5
    double f_factor = 2.0; // 2.0

    uint32_t br_cnt = 0;

    ExplorationState state = PAUSED;

    bool first_loop = true;
    bool tree_loaded = false;


    // KD-Trees
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::KdTreeFLANN<pcl::PointXYZ> trav_kdtree;
    pcl::KdTreeFLANN<pcl::PointXYZ> obs_kdtree;
    pcl::KdTreeFLANN<pcl::PointXYZ> keypoints_kdtree;
    bool trav_received = false;
    bool obs_received = false;
    //  bool octree_received = false;

    // Visualization
    visualization_msgs::Marker line;
    visualization_msgs::Marker local_branch_line;
    visualization_msgs::Marker new_connection_line;
    visualization_msgs::Marker r_points;
    visualization_msgs::Marker n_points;

    visualization_msgs::Marker keypoints_line;


    // Services
    ros::ServiceClient get_frontier_path_lengths_client_;

    // Reconfigure
    // dynamic_reconfigure::Server<ttr::validatorConfig> reconfigure_server_;
    // void dynReconfig(ttr::a_star_plannerConfig& config, uint32_t level);

    // Callbacks
    // void pointsClb(const geometry_msgs::PointStamped& point);
    void timerClb(const ros::TimerEvent& event);
    void timerCheckClb(const ros::TimerEvent& event);
    void traversablePointCloudClb(const sensor_msgs::PointCloud2ConstPtr& t_pc_msg);
    void obstaclePointCloudClb(const sensor_msgs::PointCloud2ConstPtr& obs_pc_msg);
    void localBranchClb(const ttr::branch& branch);
    void explorationStateClb(const std_msgs::UInt8::ConstPtr& state_msg);


    // Other methods
    void initialization();
    geometry_msgs::Point getRandomPoint(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z);
    void deleteBranch(const unsigned int tree_node);
    // void addBranch(std::vector<ttr::node> nodes, ttr::node near);
    bool addBranchOri(std::vector<ttr::node> nodes, geometry_msgs::PointStamped frontier);
    bool addBranch(std::vector<ttr::node> nodes, geometry_msgs::PointStamped frontier);
    bool insideNeighbours(std::vector<uint32_t> neighbours, uint32_t node_key);
    void sendGraph();

    void newConnections(double max_dist_connection);
    void newCloseConnections(double max_dist_connection, double action_radius);
    void newConnections(uint32_t node_key, double max_dist_connection);
    void performNewConnections(uint32_t node_key, double max_dist_connection);

    void keypointsStuff();

    // double getMapDistance(const Eigen::Vector3d& position) const;


};

}  // namespace ttr

#endif  // INCLUDE_TTR_PERSISTENT_TREE_H