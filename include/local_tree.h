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

#ifndef INCLUDE_TTR_LOCAL_TREE_H
#define INCLUDE_TTR_LOCAL_TREE_H

#include <dynamic_reconfigure/server.h>
#include <math.h>
// #include <planners_3d/a_star_plannerConfig.h>
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

namespace ttr {


// Pointclouds
pcl::PointCloud<pcl::PointXYZ>::Ptr trav_pc(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr obs_pc(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr tr_gr(new pcl::PointCloud<pcl::PointXYZ>);

// class Node {
//    public:
//     Node(unsigned int id_, unsigned int parent_id_, const geometry_msgs::Point& position_)
//         : id(id_), parent_id(parent_id_), position(position_) {}

//     unsigned int id;
//     unsigned int parent_id;
//     geometry_msgs::Point position;
//     std::vector<unsigned int> neighbours;

//    private:
// };

class LocalTree {
   public:
    explicit LocalTree(const ros::NodeHandle& nh,
                           const ros::NodeHandle& nh_private);
    virtual ~LocalTree();

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
    ros::Subscriber exploration_state_sub_;

    ros::Publisher lines_pub_;
    ros::Publisher frontier_pub_;
    ros::Publisher branch_pub_;
    ros::Publisher points_pub_;

    // ros::Publisher path_pub_;
    // ros::Publisher debug_path_pub_;

    ros::Timer timer_;
    // ros::Timer check_timer_;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    // Params

    // Map limits

    // Global Variables
    std::vector<ttr::node> tree_graph;
    geometry_msgs::Point current_position;
    geometry_msgs::Point last_position;


    double init_time;
    double init_time_print;
    double total_time = 0.0;
    double last_print = 0.0;
    uint32_t total_tries = 0;
    

    std::string fixed_frame;
    std::string base_frame;
    double max_x, min_x, max_y, min_y, max_z, min_z;  // Exploration Delimitation
    double margin;
    double step;
    double map_res;
    double rad_gain;
    double info_gain;
    double grow_time;
    bool ground_robot;
    double proj_height;
    double max_slope;

    double distance = 0.0;
    bool first_loop = true;

    double timing;

    uint32_t br_cnt = 0;

    std::vector<double> time_iter, time_to_again;
    ExplorationState state = PAUSED;

    // KD-Trees
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::KdTreeFLANN<pcl::PointXYZ> trav_kdtree;
    pcl::KdTreeFLANN<pcl::PointXYZ> obs_kdtree;
    bool trav_received = false;
    bool obs_received = false;
    //  bool octree_received = false;

    // Visualization
    visualization_msgs::Marker line;
    visualization_msgs::Marker r_points;
    visualization_msgs::Marker n_points;
    visualization_msgs::Marker v_frontier;

    // Services

    // Reconfigure
    // dynamic_reconfigure::Server<ttr::validatorConfig> reconfigure_server_;
    // void dynReconfig(ttr::a_star_plannerConfig& config, uint32_t level);

    // Callbacks
    void timerClb(const ros::TimerEvent& event);
    void traversablePointCloudClb(const sensor_msgs::PointCloud2ConstPtr& t_pc_msg);
    void obstaclePointCloudClb(const sensor_msgs::PointCloud2ConstPtr& obs_pc_msg);
    void explorationStateClb(const std_msgs::UInt8::ConstPtr& state_msg);

    // Other methods
    geometry_msgs::Point getRandomPoint(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z);
    void sendBranch(const geometry_msgs::PointStamped &frontier, const ttr::node &q_near);
    void treeReinitialization();
};

}  // namespace ttr

#endif  // INCLUDE_TTR_LOCAL_TREE_H