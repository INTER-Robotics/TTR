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

#include "decision_maker.h"

// #define UINT32_MAX  ((uint32_t)-1)

namespace ttr {

DecisionMaker::DecisionMaker(const ros::NodeHandle &nh) : nh_(nh) {
    configure();
    // reconfigure_server_.setCallback(boost::bind(&DecisionMaker::dynReconfig, this, _1, _2));
    // first_loop = false;
}

DecisionMaker::~DecisionMaker() {
}

// void DecisionMaker::dynReconfig(ttr::validatorConfig &config, uint32_t level) {
//     // planner_patience = config.planner_patience;
//     // max_planner_retries = config.max_planner_retries;
//     side_margin = config.side_margin;
//     allow_unknown = config.allow_unknown;
//     allow_replan = config.allow_replan;
//     // optimize_path = config.optimize_path;

//     if (first_loop || config.depth != depth_plan || config.max_x != limits.max_x ||
//          config.min_x != limits.min_x || config.max_y != limits.max_y || config.min_y != limits.min_y ||
//          config.max_z != limits.max_z || config.min_z != limits.min_z) {
//
//         limits.max_x = config.max_x;
//         limits.min_x = config.min_x;
//         limits.max_y = config.max_y;
//         limits.min_y = config.min_y;
//         limits.max_z = config.max_z;
//         limits.min_z = config.min_z;
//         depth_plan = config.depth;

//         if (grid) delete grid;

//         grid = new Grid(limits, resolution, depth_plan, allow_unknown, side_margin);
//         ROS_INFO("GRID:");
//         // ROS_INFO("  X_init: %2.2lf", grid->init_x);
//         // ROS_INFO("  Y_init: %2.2lf", grid->init_y);
//         // ROS_INFO("  Z_init: %2.2lf", grid->init_z);
//         // ROS_INFO("  X_cells: %d", grid->cells_x);
//         // ROS_INFO("  Y_cells: %d", grid->cells_y);
//         // ROS_INFO("  Z_cells: %d", grid->cells_z);
//         ROS_INFO("Map with %d cells", grid->cells_x * grid->cells_y * grid->cells_z);
//     }
// }

void DecisionMaker::configure() {
    
    nh_.param("frequency", frequency, 1.0);
    ROS_INFO("Frequency of the node: %2.2lfHz", frequency);

    nh_.param<std::string>("fixed_frame", fixed_frame, "map");
    ROS_INFO("Fixed frame: %s", fixed_frame.c_str());

    nh_.param<std::string>("base_frame", base_frame, "base_link");
    ROS_INFO("Base frame: %s", base_frame.c_str());

    nh_.param("ground_robot", ground_robot, false);
    ROS_INFO("Ground robot: %s", ground_robot ? "true" : "false");

    nh_.param("max_vel", max_vel, 1.0);
    ROS_INFO("Maximum speed allowed: %2.2lfm/s", max_vel);
    max_vel = max_vel * 0.8; //Used to compute needed time to home (max_vel * threshold)

    nh_.param("max_mission_time", max_mission_time, 300.0);
    ROS_INFO("Maximum time mission: %2.2lfs", max_mission_time);
    

    // Publishers
    // branch_pub_ = nh_.advertise<ttr::branch>("/local_branch", 10);
    front_score_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/front_score", 1000);
    exploration_state_pub_ = nh_.advertise<std_msgs::UInt8>("/exploration_state", 1);

    if(ground_robot)cancel_navigation_pub_ = nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10);

    // Subscribers
    // trav_pc_sub_ = nh_.subscribe("/voxblox_node/traversable", 1, &DecisionMaker::traversablePointCloudClb, this);
    // obs_pc_sub_ = nh_.subscribe("/voxblox_node/surface_pointcloud", 1, &DecisionMaker::obstaclePointCloudClb, this);
    frontiers_sub_ = nh_.subscribe("/frontiers", 1, &DecisionMaker::frontiersClb, this);
    if(ground_robot)goal_status_sub_ = nh_.subscribe("/move_base/status", 1, &DecisionMaker::statusGoalCallback, this);

    // Client Services
    go_to_frontier_client_ = nh_.serviceClient<ttr::MakePlan>("/planner/make_plan");
    get_frontier_path_lengths_client_ = nh_.serviceClient<ttr::PathLengths>("/planner/get_frontier_path_lengths");
    cancel_follow_path_client_ = nh_.serviceClient<std_srvs::Empty>("/mission_manager/stop_follow_path");
    pause_follow_path_client_ = nh_.serviceClient<std_srvs::Empty>("/mission_manager/pause_follow_path");
    resume_follow_path_client_ = nh_.serviceClient<std_srvs::Empty>("/mission_manager/resume_follow_path");

    // Advertising Services
    start_exploration_srv_ = nh_.advertiseService("start_exploration", &DecisionMaker::startExploration, this);
    stop_exploration_srv_ = nh_.advertiseService("stop_exploration", &DecisionMaker::stopExploration, this);
    pause_exploration_srv_ = nh_.advertiseService("pause_exploration", &DecisionMaker::pauseExploration, this);
    resume_exploration_srv_ = nh_.advertiseService("resume_exploration", &DecisionMaker::resumeExploration, this);
    go_home_srv_ = nh_.advertiseService("go_home", &DecisionMaker::goHome, this);

    // Timers
    timer_ = nh_.createTimer(ros::Duration(1.0 / frequency), &DecisionMaker::timerClb, this);  

}


// void DecisionMaker::traversablePointCloudClb(const sensor_msgs::PointCloud2ConstPtr &t_pc_msg) {
//     // Converting ROS message to PCL
//     pcl::fromROSMsg(*t_pc_msg, *trav_pc);
//     trav_kdtree.setInputCloud(trav_pc);
// }


// void DecisionMaker::obstaclePointCloudClb(const sensor_msgs::PointCloud2ConstPtr &obs_pc_msg) {
//     // Converting ROS message to PCL
//     pcl::fromROSMsg(*obs_pc_msg, *obs_pc);
//     obs_kdtree.setInputCloud(obs_pc);
// }

// Callback to check move_base status
void DecisionMaker::statusGoalCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& stat) {
  
  if (!stat->status_list.empty()) { // we have to ensure status_list array is not empty else we have error and node die due to memory allocation error
  
    move_base_status = stat -> status_list[stat->status_list.size()-1].status; // this structure is used to always have the status updated (due to move_base way to work)
    if(move_base_status == 3)frontier_reached = true;
  
  } else {
    move_base_status = 255;
  }
//   ROS_INFO("MOVE BASE ST: %d", move_base_status);

}

// Callback to generate the tree-graph
void DecisionMaker::timerClb(const ros::TimerEvent &event) {

    if(first_loop){
        // Home position
        home_position = getRobotPosition(fixed_frame, base_frame, listener, transform);
        last_time = ros::Time::now().toSec();
        last_pose = home_position;

        // init_time = ros::Time::now().toSec(); // Duit a service callback start_exploration
        first_loop = false;
    } 

    if(finished || finished_by_time){
        if (finished)ROS_INFO_ONCE("No frontiers found. Exploration conclude in %6.2lf.", ros::Time::now().toSec() - init_time - 50/frequency);
        else ROS_INFO_ONCE("No frontiers found. Exploration conclude in %6.2lf.", ros::Time::now().toSec() - init_time);
        ROS_INFO_ONCE("Total travelled distance: %6.2lf m.", travelled_dist);
        ROS_INFO_ONCE("AVG Speed: %6.5lf m/s.", avg_speed_sum / cnt_speed);
        finished = false;
        finished_by_time = false;
        // performStopExploration();
        performGoHome();
        return;
    }

    // Exploration State
    switch (state)
    {
    case EXPLORING:
        /* code */
        break;
    case RETURNING:
        /* code */
        return;
        break;
    case PAUSED:
        /* code */
        return;
        break;
    case FINISHED:
        /* code */
        break;
    default:
        break;
    }

    double init_time_clb = ros::Time::now().toSec();
    double time_aux;

    bool compute_distance_travelled = true;
    if(compute_distance_travelled){
        double aux_travelled_dist = euclideanDistance(last_pose, getRobotPosition(fixed_frame, base_frame, listener, transform));
        travelled_dist += aux_travelled_dist;
        double aux_speed = aux_travelled_dist / (ros::Time::now().toSec() - last_time);
        if(aux_speed > 0.05){ // 0.05 m/s threshold to consider robot is moving
            avg_speed_sum += aux_speed;
            cnt_speed++;
        }
        last_pose = getRobotPosition(fixed_frame, base_frame, listener, transform);
        last_time = ros::Time::now().toSec();
    }

    if(new_frontiers_received){

        // Current position
        geometry_msgs::Point aux = getRobotPosition(fixed_frame, base_frame, listener, transform);


        // Check follow_path status
        if(!ground_robot){
        
            ros::param::get("/mission_manager/follow_path_status", f_path_status);
            // Is not following path (problem)
            if(f_path_status == 0)zero_status_counter++;
            // Following path accomplished (frontier reached)
            if(last_f_path_status == 1 && f_path_status == 0)frontier_reached = true;
            // Following path in course (and allowed)
            if(f_path_status == 1) autonomous_paused = false;
            // Following path stopped by user
            if(f_path_status == 2) autonomous_paused = true;

        } 

        // ############################# IS ROBOT STUCK??
        // The robot is not moving while exploring (ARREGLO QUE HA DE FER EN XISCO)
        if(!autonomous_paused && euclideanDistance(aux, current_position) < 0.03){ 
            static_while_going_counter++;
        }

        // The frontier is unreachable with the current path
        if(static_while_going_counter > 10){ //30
            ROS_INFO("Can't reach this frontier. Will replan.");
            static_while_going_counter = 0;
            
            // Cancel current path following
            cancelPathFollowing();
            
            raw_path = true;
            replan = true;
        }
        current_position = aux;


        // ################################### SCORING ##################################

        // Getting all frontier paths
        time_aux = ros::Time::now().toSec();
        ttr::PathLengths get_frontier_path_lengths;
        get_frontier_path_lengths.request.raw_length = false; // We want normalized lengths
        get_frontier_path_lengths.request.start = current_position;
        for(int i = 0; i < sorted_frontiers.size() ; i++){
            get_frontier_path_lengths.request.frontiers.push_back(sorted_frontiers[i].position);
        }

        if(get_frontier_path_lengths_client_.call(get_frontier_path_lengths)){
            for(int i = 0; i < sorted_frontiers.size() ; i++){
                sorted_frontiers[i].path_length = get_frontier_path_lengths.response.lengths[i];
            }
        
        } else {
            ROS_INFO("Only Euclidean Distance");
            // PRIMIGENIUS OLD WAY TO GET FRONTIER SCORE (ONLY EUCLIDEAN DISTANCE)
            for(int i = 0; i < sorted_frontiers.size() ; i++){
                sorted_frontiers[i].path_length = euclideanDistance(sorted_frontiers[i].position, current_position); 
            }
        }

        // Setting score to all frontiers
        setScore(sorted_frontiers);
        std::sort(sorted_frontiers.begin(), sorted_frontiers.end(), scoreAscending);

        // Found a better score AND is a different frontier OR the same with shorter path
        // (only if autonomous behaviour is allowed)
        if(!autonomous_paused && (1.05 * last_best_score < sorted_frontiers[0].score &&
            (2.0 < euclideanDistance(last_frontier.position, sorted_frontiers[0].position)
            || 1.5 * last_frontier.path_length < sorted_frontiers[0].path_length))){ 

            // Cancel current path following
            cancelPathFollowing();

            replan = true;
        }
        
        // // Check if frontier is reached (for move base is checked in statusGoalCallback)
        // if(!ground_robot){
        
        //     ros::param::get("/mission_manager/follow_path_status", f_path_status);
        //     // Is not following path (problem)
        //     if(f_path_status == 0)zero_status_counter++;
        //     // Following path accomplished (frontier reached)
        //     if(last_f_path_status == 1 && f_path_status == 0)frontier_reached = true;
        //     // Following path stopped by user
        //     if(f_path_status == 2) autonomous_paused = true;

        // } 

        // Execute Follow Path to Best Frontier
        // Last frontier reached OR condition OR assigned frontier too much close (zero_status_counter)
        if(!autonomous_paused && (frontier_reached || replan || zero_status_counter > 5)){

            // Time to goal and go home from the goal
            // TODO PAREIX QUE AQUESTA PETICIO TARDA ALGO I QUEDA ROBOT ATURAT MASSA TEMPS ABANS DE DECIDIR
            // TODO ALOMILLOR PODEM FER QUE CALCULI TEMPS DE TORNADA PER CADA FRONTERA I DECIDIR EN FUNCIO DE AIXO
            // TODO pot ser no es aixo
            double espected_time_to_home = timeToHome();

            // Go home
            if(espected_time_to_home > (max_mission_time - (ros::Time::now().toSec() - init_time))){
                performGoHome();
            } else { // Go to goal
                performGoGoal();
            }

            replan = false;
            raw_path = false;
            frontier_reached = false;
            zero_status_counter = 0;
            static_while_going_counter = 0;
        }

        // Visualization
        front_score_pub_.publish(front_score);

        last_f_path_status = f_path_status;
        first_found = true;
        finished_cnt = 0;


    } else if(first_found){
        if(finished_cnt >= 50){
            ROS_ERROR_ONCE("Finished");
            finished = true;
        }
        finished_cnt++;
        // if(ros::Time::now().toSec() - init_time > 1000.0){
        //     finished_by_time = true;
        // }
    }
    new_frontiers_received = false;
    
}


// Callback with the frontiers detected
void DecisionMaker::frontiersClb(const ttr::frontiers::ConstPtr &frontiers_msg) {

    sorted_frontiers.clear();
    for(int i = 0 ; i < frontiers_msg->frontiers.size() ; i++){
        sorted_frontiers.push_back(Frontier(0.0, frontiers_msg->volumetric_gain[i], frontiers_msg->frontiers[i]));
        // ROS_INFO("VOL GAIN (%2.2lf, %2.2lf, %2.2lf): %3.4lf", frontiers_msg->frontiers[i].x,
        //          frontiers_msg->frontiers[i].y, frontiers_msg->frontiers[i].z, frontiers_msg->volumetric_gain[i]);

    }
    if(sorted_frontiers.size() != 0) new_frontiers_received = true;
}

void DecisionMaker::setScore(std::vector<Frontier>& frontiers){

    visualization_msgs::Marker aux_marker;
    aux_marker.action = visualization_msgs::Marker::DELETEALL;

    int id_text = 0;
    front_score.markers.clear();
    double w_path_length = 2.0;
    double w_vol_gain = 1.0;

    // Setting score
    for(auto& frontier : frontiers){
        frontier.score = w_path_length * (double)frontier.path_length + w_vol_gain * (double)frontier.vol_gain;

        std::string text = std::to_string(w_path_length).substr(0, std::to_string(w_path_length).find(".") + 2 + 1) + " · " + 
                           std::to_string(frontier.path_length).substr(0, std::to_string(frontier.path_length).find(".") + 2 + 1) + "\n" + 
                           std::to_string(w_vol_gain).substr(0, std::to_string(w_vol_gain).find(".") + 2 + 1) + " · " + 
                           std::to_string(frontier.vol_gain).substr(0, std::to_string(frontier.vol_gain).find(".") + 2 + 1) + "\n" +
                           std::to_string(frontier.score);

        aux_marker.header.frame_id = "map";
        aux_marker.header.stamp = ros::Time::now();
        aux_marker.lifetime = ros::Duration(2.0);
        aux_marker.ns = "tasks_name";
        aux_marker.id = id_text;
        aux_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        aux_marker.action = visualization_msgs::Marker::ADD;
        aux_marker.text = text;
        aux_marker.pose.position.x = frontier.position.x;
        aux_marker.pose.position.y = frontier.position.y;
        aux_marker.pose.position.z = frontier.position.z;
        aux_marker.pose.orientation.w = 1.0;
        aux_marker.scale.x = 0.1;
        aux_marker.scale.y = 0.1;
        aux_marker.scale.z = 0.5;
        aux_marker.color.a = 1.0; // Don't forget to set the alpha!
        id_text++;

        front_score.markers.push_back(aux_marker);

    }

}


void DecisionMaker::performGoGoal(){
    ttr::MakePlan go_to_goal;
    go_to_goal.request.x = sorted_frontiers[0].position.x;
    go_to_goal.request.y = sorted_frontiers[0].position.y;
    go_to_goal.request.z = sorted_frontiers[0].position.z;
    go_to_goal.request.raw_path = raw_path;
    makePlanAndExecute(go_to_goal);
}


void DecisionMaker::performGoHome(){
    state = RETURNING;
    std_msgs::UInt8 state_msg;
    state_msg.data = RETURNING;
    exploration_state_pub_.publish(state_msg);

    ROS_WARN("Going home.");

    ttr::MakePlan go_to_home;
    go_to_home.request.x = home_position.x;
    go_to_home.request.y = home_position.y;
    go_to_home.request.z = home_position.z;
    go_to_home.request.raw_path = false;
    makePlanAndExecute(go_to_home);
}


void DecisionMaker::makePlanAndExecute(ttr::MakePlan& objective){
    if(go_to_frontier_client_.call(objective)){
        last_best_score = sorted_frontiers[0].score;
        last_frontier = sorted_frontiers[0];
    } else {
        ROS_ERROR("Error making and excuting the plan.");
    }
}


void DecisionMaker::performPauseExploration(){
    // Cridar a tots els nodes perque es pausin (reanudable)
    pausePathFollowing();
}


void DecisionMaker::performStopExploration(){
    // Cridar a tots els nodes perque s'aturin indefinidament
    cancelPathFollowing();
}


bool DecisionMaker::startExploration(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    // Començar exploració
    state = EXPLORING;
    std_msgs::UInt8 state_msg;
    state_msg.data = EXPLORING;
    exploration_state_pub_.publish(state_msg);
    // New
    init_time = ros::Time::now().toSec();
    return true;
}


bool DecisionMaker::stopExploration(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    state = FINISHED;
    std_msgs::UInt8 state_msg;
    state_msg.data = FINISHED;
    exploration_state_pub_.publish(state_msg);
    performStopExploration(); // POT SER NO FACI FALTA
    return true;
}


bool DecisionMaker::pauseExploration(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    state = PAUSED;
    std_msgs::UInt8 state_msg;
    state_msg.data = PAUSED;
    exploration_state_pub_.publish(state_msg);
    performPauseExploration();
    return true;
}


bool DecisionMaker::resumeExploration(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    // Reanudar exploració
    state = EXPLORING;
    std_msgs::UInt8 state_msg;
    state_msg.data = EXPLORING;
    exploration_state_pub_.publish(state_msg);
    resumePathFollowing();
    return true;
}


bool DecisionMaker::goHome(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    cancelPathFollowing();
    performGoHome();
    return true;
}

double DecisionMaker::timeToHome(){
    double dist2goal = 0.0;
    double dist_goal2home = 0.0;
    ttr::PathLengths get_home_path_length;
    get_home_path_length.request.raw_length = true; // We want path length
    get_home_path_length.request.start = sorted_frontiers[0].position;
    get_home_path_length.request.frontiers.push_back(current_position);
    get_home_path_length.request.frontiers.push_back(home_position);

    if(get_frontier_path_lengths_client_.call(get_home_path_length)){
        dist2goal = get_home_path_length.response.lengths[0];
        dist_goal2home = get_home_path_length.response.lengths[1];
    } else {
        // Get euclidean distance to home
        dist2goal = euclideanDistance(sorted_frontiers[0].position, current_position);
        dist_goal2home = euclideanDistance(home_position, sorted_frontiers[0].position);
    }

    // Espected time: distance / (max_vel * vel_threshold)
    double time_to_home = (dist2goal + dist_goal2home) / (max_vel);
    // ROS_INFO("Path length to goal: %2.2lf m", dist2goal);
    // ROS_INFO("Path length from goal to home: %2.2lf m", dist_goal2home);
    // ROS_INFO("Espected time to goal and return home: %2.2lf s", time_to_home);

    return time_to_home;
}

void DecisionMaker::cancelPathFollowing(){
    if(!ground_robot){
        // Call stop follow path
        std_srvs::Empty cancel_path_follow;
        cancel_follow_path_client_.call(cancel_path_follow);
    } else {
        // Cancel move_base
        actionlib_msgs::GoalID cancel_msg;
        cancel_navigation_pub_.publish(cancel_msg);
    }
}

void DecisionMaker::pausePathFollowing(){
    // if(!ground_robot){
        // Call pause follow path
        std_srvs::Empty pause_path_follow;
        pause_follow_path_client_.call(pause_path_follow);
    // } else {
    //     // Cancel move_base
    //     actionlib_msgs::GoalID cancel_msg;
    //     cancel_navigation_pub_.publish(cancel_msg);
    // }
}

void DecisionMaker::resumePathFollowing(){
    // if(!ground_robot){
        // Call resume follow path
        std_srvs::Empty resume_path_follow;
        resume_follow_path_client_.call(resume_path_follow);
    // } else {
    //     // Cancel move_base
    //     actionlib_msgs::GoalID cancel_msg;
    //     cancel_navigation_pub_.publish(cancel_msg);
    // }
}



}  // namespace ttr