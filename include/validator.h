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

#ifndef INCLUDE_TTR_VALIDATOR_H
#define INCLUDE_TTR_VALIDATOR_H

#include <dynamic_reconfigure/server.h>
#include <math.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
// #include <planners_3d/a_star_plannerConfig.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <chrono>

#include "msc/msc"
#include "common_utils.h"

#include <fstream>
#include <iostream>
#include <queue>
#include <random>
#include <string>
#include <vector>

#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include "ttr/frontiers.h"


namespace ttr {

// class PointComparator {
//     public:
//     bool operator()(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) const {
//         if (p1.x == p2.x) {
//             if (p1.y == p2.y) {
//                 return p1.z < p2.z;
//             }
//             return p1.y < p2.y;
//         }
//         return p1.x < p2.x;
//     }
// };

class PointComparator {
public:
    double epsilon = 0.2; // Distancia máxima para considerar puntos iguales

    // explicit PointComparator(double epsilon) : epsilon(epsilon) {}

    bool operator()(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) const {
        if (std::abs(p1.x - p2.x) <= epsilon) {
            if (std::abs(p1.y - p2.y) <= epsilon) {
                if (std::abs(p1.z - p2.z) <= epsilon) {
                    return false; // Considerar puntos iguales
                }
                return p1.z < p2.z;
            }
            return p1.y < p2.y;
        }
        return p1.x < p2.x;
    }
};

// octomap::OcTree* octree;
// pcl::PointCloud<pcl::PointXYZ>::Ptr obs_pc(new pcl::PointCloud<pcl::PointXYZ>);
// pcl::PointCloud<pcl::PointXYZ>::Ptr trav_pc(new pcl::PointCloud<pcl::PointXYZ>);

class Validator {
   public:
    explicit Validator(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private);
    virtual ~Validator();

    void configure();

   private:
    // ROS variables
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    voxblox::EsdfServer voxblox_server_;

    // ros::Subscriber octomap_sub_;
    ros::Subscriber raw_points_sub_;
    // ros::Subscriber octomap_pc_sub_;
    ros::Subscriber trav_pc_sub_;
    ros::Subscriber obs_pc_sub_;
    ros::Subscriber exploration_state_sub_;

    ros::Publisher raw_points_pub_;
    ros::Publisher filtered_points_pub_;
    ros::Publisher frontiers_pub_;
    ros::Publisher raycast_gain_vis_pub_;

    // ros::Publisher path_pub_;
    // ros::Publisher debug_path_pub_;

    ros::Timer timer_;


    // Params
    

    // Map limits


    // Global Variables
    std::string fixed_frame;
    visualization_msgs::Marker raw_points, filtered;
    std::set<geometry_msgs::Point, PointComparator> raw_set;
    std::vector<geometry_msgs::Point> raw_vector, filtered_vector;
    std::vector<double> vol_gain_vector;
    double max_height;
    double ms_bandwidth;
    bool octree_received;
    double margin;
    double map_res;
    double rad_gain;
    double min_info_gain;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::KdTreeFLANN<pcl::PointXYZ> trav_kdtree;
    pcl::KdTreeFLANN<pcl::PointXYZ> obs_kdtree;
    bool trav_received = false;
    bool obs_received = false;
    
    double timing;

    ExplorationState state = PAUSED;

    // Raycast gain visualization
    visualization_msgs::Marker unk_gain, free_gain, occ_gain;
    std::vector<geometry_msgs::Point> unk_vec, free_vec, occ_vec;



    // Services

    // Reconfigure
    // dynamic_reconfigure::Server<ttr::validatorConfig> reconfigure_server_;
    // void dynReconfig(ttr::a_star_plannerConfig& config, uint32_t level);

    // Callbacks
    void pointsClb(const geometry_msgs::PointStamped& point);
    void timerClb(const ros::TimerEvent& event);
    void explorationStateClb(const std_msgs::UInt8::ConstPtr& state_msg);
    // void octomapClb(const octomap_msgs::Octomap::ConstPtr& map_msg);
    // void octomapPointCloudClb(const sensor_msgs::PointCloud2ConstPtr& pc_msg);
    // void traversablePointCloudClb(const sensor_msgs::PointCloud2ConstPtr& t_pc_msg);
    // void obstaclePointCloudClb(const sensor_msgs::PointCloud2ConstPtr& obs_pc_msg);

    // Other methods
    double getVolumetricGain(geometry_msgs::Point frontier);

};

}  // end namespace validator_3d

#endif  // INCLUDE_TTR_VALIDATOR_H