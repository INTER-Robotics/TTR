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

#ifndef INCLUDE_TTR_DECISION_MAKER_H
#define INCLUDE_TTR_DECISION_MAKER_H

#include <dynamic_reconfigure/server.h>
#include <math.h>
// #include <planners_3d/a_star_plannerConfig.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdio.h>
#include <stdlib.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <time.h>

#include <fstream>
#include <iostream>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <queue>
#include <random>
#include <string>
#include <vector>
#include <algorithm>

#include "common_utils.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "msc/msc"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "ttr/node.h"
#include "ttr/branch.h"
#include "ttr/frontiers.h"

#include <ttr/MakePlan.h>
#include <ttr/PathLengths.h>
#include <std_srvs/Empty.h>

#include "actionlib_msgs/GoalID.h"
#include "actionlib_msgs/GoalStatus.h"
#include "actionlib_msgs/GoalStatusArray.h"



namespace ttr {


// Pointclouds
// pcl::PointCloud<pcl::PointXYZ>::Ptr trav_pc(new pcl::PointCloud<pcl::PointXYZ>);
// pcl::PointCloud<pcl::PointXYZ>::Ptr obs_pc(new pcl::PointCloud<pcl::PointXYZ>);
// pcl::PointCloud<pcl::PointXYZ>::Ptr tr_gr(new pcl::PointCloud<pcl::PointXYZ>);

struct Frontier{
    double score;
    double vol_gain;
    double path_length = 0.0;
    geometry_msgs::Point position;

    Frontier() {}
    
    Frontier(double score, double vol_gain, const geometry_msgs::Point& position)
        : score(score), vol_gain(vol_gain), position(position) {}
};

auto scoreDescending = [](const Frontier& f1, const Frontier& f2) {
    return f1.score < f2.score;
};

auto scoreAscending = [](const Frontier& f1, const Frontier& f2) {
    return f1.score > f2.score;
};

auto pathLengthDescending = [](const Frontier& f1, const Frontier& f2) {
    return f1.path_length < f2.path_length;
};

class DecisionMaker {
   public:
    explicit DecisionMaker(const ros::NodeHandle& nh);
    virtual ~DecisionMaker();

    void configure();

   private:
    // ROS variables
    ros::NodeHandle nh_;

    // Subscribers
    ros::Subscriber frontiers_sub_;
    // ros::Subscriber trav_pc_sub_;
    // ros::Subscriber obs_pc_sub_;
    ros::Subscriber goal_status_sub_;

    // Publishers
    ros::Publisher front_score_pub_;
    // ros::Publisher frontier_pub_;
    // ros::Publisher branch_pub_;
    ros::Publisher cancel_navigation_pub_;

    ros::Publisher exploration_state_pub_;

    ros::Timer timer_;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    // Params

    // Map limits

    // Global Variables
    // std::vector<ttr::node> tree_graph;
    geometry_msgs::Point current_position;
    // std::vector<geometry_msgs::Point> frontiers;
    std::vector<Frontier> sorted_frontiers;

    double frequency;
    bool replan =  true;
    bool raw_path = false;
    bool frontier_reached =  false;
    int f_path_status, last_f_path_status;
    int zero_status_counter;
    int static_while_going_counter = 0;
    
    double init_time = 0.0;
    double stucked_time;
    bool first_loop = true;
    bool finished = false;
    bool finished_by_time = false;
    bool first_found = false;
    uint8_t finished_cnt = 0;
    bool ground_robot;

    uint8_t move_base_status = 255; // objective status of the robot -> 1 == in the way to goal // 3 == goal reached // 255 == no status 

    double last_best_score = 0.0;
    double last_best_lenght = 0.0;
    double last_best_gain = 0.0;
    Frontier last_frontier;

    bool new_frontiers_received = false;

    bool autonomous_paused = false;

    std::string fixed_frame;
    std::string base_frame;

    double max_vel;
    double max_mission_time;
    geometry_msgs::Point home_position;

    // Calculate travelled distance
    geometry_msgs::Point last_pose;
    double travelled_dist = 0.0;

    // Calculate average speed
    double avg_speed_sum = 0.0;
    int cnt_speed = 1;
    double last_time = 0.0;

    ExplorationState state = PAUSED;

    // double max_x, min_x, max_y, min_y, max_z, min_z;  // Exploration Delimitation
    // double margin;
    // double step;
    // double map_res;

    // KD-Trees
    // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    // pcl::KdTreeFLANN<pcl::PointXYZ> trav_kdtree;
    // pcl::KdTreeFLANN<pcl::PointXYZ> obs_kdtree;
    //  bool octree_received = false;

    // Visualization
    // visualization_msgs::Marker line;   
    visualization_msgs::MarkerArray front_score;


    // Services
    ros::ServiceClient go_to_frontier_client_;
    ros::ServiceClient cancel_follow_path_client_, pause_follow_path_client_, resume_follow_path_client_;
    ros::ServiceClient get_frontier_path_lengths_client_;

    ros::ServiceServer start_exploration_srv_, stop_exploration_srv_, pause_exploration_srv_, resume_exploration_srv_, go_home_srv_;

    bool startExploration(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool stopExploration(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool pauseExploration(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool resumeExploration(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool goHome(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    // Reconfigure
    // dynamic_reconfigure::Server<ttr::validatorConfig> reconfigure_server_;
    // void dynReconfig(ttr::a_star_plannerConfig& config, uint32_t level);

    // Callbacks
    void timerClb(const ros::TimerEvent& event);
    void frontiersClb(const ttr::frontiers::ConstPtr &frontiers_msg);
    void statusGoalCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& stat);
    // void traversablePointCloudClb(const sensor_msgs::PointCloud2ConstPtr& t_pc_msg);
    // void obstaclePointCloudClb(const sensor_msgs::PointCloud2ConstPtr& obs_pc_msg);

    // Other methods
    // double setScore(const geometry_msgs::Point& f_position);
    void setScore(std::vector<Frontier>& frontiers);

    void performGoHome();
    void performGoGoal();
    void makePlanAndExecute(ttr::MakePlan& objective);
    void performPauseExploration();
    void performStopExploration();
    double timeToHome();
    void cancelPathFollowing();
    void pausePathFollowing();
    void resumePathFollowing();


};

}  // namespace ttr

#endif  // INCLUDE_TTR_DECISION_MAKER_H;