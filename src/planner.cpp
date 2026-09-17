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

#include "planner.h"

// #define UINT32_MAX  ((uint32_t)-1)

namespace ttr {

Planner::Planner(const ros::NodeHandle &nh,
                 const ros::NodeHandle& nh_private)
        : nh_(nh),
          nh_private_(nh_private),
          voxblox_server_(nh_, nh_private_)
        //   tsdf_server_(nh_, nh_private_)
{
    configure();
    // reconfigure_server_.setCallback(boost::bind(&Planner::dynReconfig, this, _1, _2));
    // first_loop = false;
}

Planner::~Planner() {
}

// void Planner::dynReconfig(ttr::validatorConfig &config, uint32_t level) {
//     // planner_patience = config.planner_patience;
//     // max_planner_retries = config.max_planner_retries;
//     side_margin = config.side_margin;
//     allow_unknown = config.allow_unknown;
//     allow_replan = config.allow_replan;
//     // optimize_path = config.optimize_path;

//     if (first_loop || config.depth != depth_plan || config.max_x != limits.max_x || config.min_x != limits.min_x || config.max_y != limits.max_y || config.min_y != limits.min_y || config.max_z != limits.max_z || config.min_z != limits.min_z) {
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

void Planner::configure() {

    double freq_check = 1.0;
    double frequency;
    nh_private_.param("frequency", frequency, 100.0);
    ROS_INFO("Frequency of the node: %2.2lfHz", frequency);

    nh_private_.param<std::string>("fixed_frame", fixed_frame, "map");
    ROS_INFO("Fixed frame: %s", fixed_frame.c_str());

    nh_private_.param<std::string>("base_frame", base_frame, "base_link");
    ROS_INFO("Base frame: %s", base_frame.c_str());

    nh_private_.param("map_resolution", map_res, 0.35);
    ROS_INFO("Voxel size of the voxblox: %2.2lfm", map_res);

    nh_private_.param("margin", margin, 0.5);
    ROS_INFO("Margin per side of the path: %2.2lfm", margin);

    nh_private_.param("ground_robot", ground_robot, false);
    ROS_INFO("Ground robot: %s", ground_robot ? "true" : "false");

    if(ground_robot){
        nh_private_.param("projection_height", proj_height, 0.5);
        ROS_INFO("Height at which points are set after projection: %2.2lfm", proj_height);

        nh_private_.param("max_slope", max_slope, 20.0);
        ROS_INFO("Maximum allowed slope (degree): %2.2lf deg", max_slope);
    }


    // Publishers
    path_pub_ = nh_.advertise<nav_msgs::Path>("/path_planned", 1);
    vis_path_pub_ = nh_.advertise<nav_msgs::Path>("/VIS_path_planned", 1);
    raw_path_pub_ = nh_.advertise<nav_msgs::Path>("/RAW_path_planned", 1);
    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pointcloud_local_planner", 1);

    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    // Subscribers
    // obs_pc_sub_ = nh_.subscribe("/voxblox_node/surface_pointcloud", 1, &Planner::obstaclePointCloudClb, this);
    tree_graph_sub_ = nh_.subscribe("/planner_graph", 1, &Planner::treeGraphClb, this);

    // Wait for receive voxblox pointclouds
    // while(!obs_received){
    //     ros::spinOnce();
    //     ros::Duration(0.1).sleep();
    // }
    while(!voxblox_server_.getEsdfMapPtr() || !voxblox_server_.getTsdfMapPtr()){
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("Voxblox received.");

    // Advertising Services
    makePlan_ = nh_.advertiseService("/planner/make_plan", &Planner::makePlan, this);
    getFrontierPathLengths_ = nh_.advertiseService("/planner/get_frontier_path_lengths", &Planner::getFrontierPathLengths, this);;

    // Client Services
    start_follow_path_client_ = nh_.serviceClient<srv_mav_behaviours::StartFollowPath>("/mission_manager/start_follow_path");


    // Timers
    timer_ = nh_.createTimer(ros::Duration(1.0 / frequency), &Planner::timerClb, this);

}



// void Planner::obstaclePointCloudClb(const sensor_msgs::PointCloud2ConstPtr &obs_pc_msg) {
//     // Converting ROS message to PCL
//     pcl::fromROSMsg(*obs_pc_msg, *obs_pc);
//     obs_kdtree.setInputCloud(obs_pc);
//     obs_received = true;
// }


void Planner::treeGraphClb(const ttr::treeGraph::ConstPtr &treegraph_msg) {

    double init_tree = ros::Time::now().toSec();
    tree.clear();
    tree_graph.clear();
    // PCL for the tree-graph resize with new tree
    tr_gr->points.resize(treegraph_msg->nodes.size());
    // std::ofstream myfile;
    // myfile.open ("/home/tonitauler/nodes_planner.txt");

    // Fill PCL and unordered_map
    for(int i = 0; i < treegraph_msg->nodes.size(); i++){
        uint32_t key = treegraph_msg->keys[i];
        geometry_msgs::Point pos = treegraph_msg->nodes[i].position;
        (*tr_gr)[key].x = pos.x;
        (*tr_gr)[key].y = pos.y;
        (*tr_gr)[key].z = pos.z;

        tree_graph.emplace(key, GraphNode(treegraph_msg->nodes[i]));

        // myfile << "Node: " << key << " (" << std::fixed << std::setprecision(2) << treegraph_msg->nodes[i].position.x << ", " << treegraph_msg->nodes[i].position.y << ", " << treegraph_msg->nodes[i].position.z << ") -> [ " << treegraph_msg->nodes[i].parent_id << " ";


        //     for (int j = 0; j < treegraph_msg->nodes[i].neighbours.size(); j++){
        //         myfile << treegraph_msg->nodes[i].neighbours[j] << " ";
        //     }
        //     myfile << "]\n";

    }
    // myfile.close();

    // Generating the tree-graph KD-Tree
    kdtree.setInputCloud(tr_gr);
    new_tree_graph = true;
    // ROS_INFO("Time to generate new tree-graph(%d): %1.9lf", treegraph_msg->nodes.size(), ros::Time::now().toSec() - init_tree);

}


bool Planner::getFrontierPathLengths(ttr::PathLengths::Request& req, ttr::PathLengths::Response& res) {

    start_ = req.start;
    // nav_msgs::Path path;
    std::vector<double> raw_lengths;
    double max_length = 0.0;
    double possible_max_length = 100.0;
    double aux_time = ros::Time::now().toSec();
    // ROS_INFO("Pre all frontiers planning");

    aux_time = ros::Time::now().toSec();
    int counter = 0;
    for(int i = 0; i < req.frontiers.size(); i++){

        nav_msgs::Path path;
        double initial = ros::Time::now().toSec();
        goal_ = req.frontiers[i];

        // ROS_INFO("Pre %d frontier planning", i+1);

        // LLEVAR
        // if(req.raw_length) performMakePlan(start_, goal_, false, true);
        // else path = performMakePlan(start_, goal_, false, false);
        //LLEVAR

        // POSAR
        path = performMakePlan(start_, goal_, false, false);

        // TODO Calculate the path length during path computation (needed in gCost)
        // Will avoid calculating path length twice

        double length = pathLength(path);

        // if(req.raw_length) ROS_INFO("Path lenght: %2.4lfm with %d nodes",
        //                                 length, path.poses.size());

        // // // if(path.poses.size() == 0 || length < 0.005
        // // //         || length > possible_max_length){
        // // //     length = possible_max_length;
        // // // }
        // ARREGLAR PERQ AIXO NO PASSI (FA PINTA DE ALGO RARO AMB A* QUE NO TROBA CAMI A ALGUN DELS PUNTS)
        if(path.poses.size() == 0) length = 2000.0; // No path found

        raw_lengths.push_back(length);
    }

    // ROS_INFO("PATHS: %1.9lf", ros::Time::now().toSec() - aux_time);
    aux_time = ros::Time::now().toSec();

    // // // // To avoid in (ONLY) close frontiers the weight of path length has minor impact in decision maker
    // // // if(max_length < possible_max_length)max_length = possible_max_length;
    // double max_value = exp(1/max_length);

    // Alternative weight
    float alt_mu = -15.0;
    float line = 20.0;
    float sigma = 35.0;
    // float aux_line = (1 - exp( -pow(line - alt_mu , 2) / ( pow(2 * sigma , 2) ) ) ) / line;

    // Normalization
    float mu = -2.0;
    for (auto& value : raw_lengths) {

        if(!req.raw_length){
            // Original (LINEAL)
            // value = 1 - (value / max_length);

            // Custom (LINEAL & GAUSSIAN)
            // if(value <= 20.0) value = 1.0 - aux_line * value;
            // else value = exp( -pow(value - alt_mu , 2) / ( pow(2 * sigma , 2) ) );

            // Exponential
            // value = exp(-0.045*value);
            // value = exp(-0.02*value);

            // Gaussian
            value = exp( -pow(value - mu , 2) / ( pow(70 , 2) ) );
        }
        
        res.lengths.push_back(value);
    }
    
    return true;
}


bool Planner::makePlan(ttr::MakePlan::Request &req, ttr::MakePlan::Response &res) {

    start_ = getRobotPosition(fixed_frame, base_frame, listener, transform);

    goal_.x = req.x;
    goal_.y = req.y;
    goal_.z = req.z;

    nav_msgs::Path path;
    path = performMakePlan(start_, goal_, true, req.raw_path);
    // ROS_WARN("Start/near_start (%3.2lf/%3.2lf, %3.2lf/%3.2lf, %3.2lf/%3.2lf), Goal/Near_goal (%3.2lf/%3.2lf, %3.2lf/%3.2lf, %3.2lf/%3.2lf), Nodes: %d",
    //             start_.x, near_start->node.position.x, start_.y, near_start->node.position.y, start_.z, near_start->node.position.z,
    //             goal_.x, near_goal->node.position.x, goal_.y, near_goal->node.position.y, goal_.z, near_goal->node.position.z, path.poses.size());

    if(!ground_robot){
        // Call path following
        srv_mav_behaviours::StartFollowPath start_follow_path;
        start_follow_path.request.path = path;
        start_follow_path_client_.call(start_follow_path);
    } else {

        // Move base path follow
        geometry_msgs::PoseStamped goal_msg;
        goal_msg.header.frame_id = "map";
        goal_msg.header.stamp = ros::Time::now();
        // goal_msg.pose.position.x = goal_.x;
        // goal_msg.pose.position.y = goal_.y;
        goal_msg.pose.position.x = 0.0;
        goal_msg.pose.position.y = 0.0;
        goal_msg.pose.position.z = 0.0;

        int last_path_node = path.poses.size() - 1;
        double dx = goal_.x - path.poses[last_path_node-1].pose.position.x;
        double dy = goal_.y - path.poses[last_path_node-1].pose.position.y;
        double yaw = atan2(dy, dx);

        tf2::Quaternion quaternion;
        quaternion.setRPY( 0.0, 0.0, yaw);  // Create this quaternion from roll/pitch/yaw (in radians)
        quaternion = quaternion.normalize();
        goal_msg.pose.orientation = tf2::toMsg(quaternion);

        goal_pub_.publish(goal_msg);
    }

    // ROS_INFO("Planned");

    return true;
}

nav_msgs::Path Planner::performMakePlan(geometry_msgs::Point start, geometry_msgs::Point goal, bool navigation, bool raw_path) {

    // Load Voxblox Map
    std::shared_ptr<voxblox::EsdfMap> mapPtr = voxblox_server_.getEsdfMapPtr();
    std::shared_ptr<voxblox::TsdfMap> tsdf_mapPtr = voxblox_server_.getTsdfMapPtr();

    nav_msgs::Path path;
    bool path_found = false;

    // Clean opened priority_queue
    while (!opened.empty()) {
        opened.pop();
    }


    // Reset the values of the tree_graph structure for planning (if needed)
    if(!new_tree_graph){
        for (auto& node : tree_graph) {
            node.second.reset();
        }
    }
    new_tree_graph = false;


    // Check the nearest node of start
    pcl::PointXYZ searchPoint;
    searchPoint.x = start.x;
    searchPoint.y = start.y;
    searchPoint.z = start.z;

    std::vector<int> indices;
    std::vector<float> sq_dist;


    if(!navigation){
        kdtree.nearestKSearch(searchPoint, 1, indices, sq_dist);
        near_start = &tree_graph[indices[0]];

    } else {

        kdtree.radiusSearch(searchPoint, 2.0, indices, sq_dist, 20);
        if(indices.size() == 0) kdtree.nearestKSearch(searchPoint, 200, indices, sq_dist);

        // Look for the nearest free path node
        for(uint32_t i = 0 ; i < indices.size(); i++){
            if(freeLineMarginVB(start, tree_graph[indices[i]].node.position, mapPtr, tsdf_mapPtr, map_res, margin, ground_robot, max_slope)){
                near_start = &tree_graph[indices[i]];
                break;
            } else {
                near_start = &tree_graph[indices[0]];
                if(i == 199)ROS_WARN("Wrong start path node selected.");
            }

        }


    }


    searchPoint.x = goal.x;
    searchPoint.y = goal.y;
    searchPoint.z = goal.z;
    indices.clear();
    sq_dist.clear();

    if(!navigation){
        kdtree.nearestKSearch(searchPoint, 1, indices, sq_dist);
        near_goal = &tree_graph[indices[0]];


    } else {

        kdtree.radiusSearch(searchPoint, 2.0, indices, sq_dist, 5);
        if(indices.size() == 0) kdtree.nearestKSearch(searchPoint, 20, indices, sq_dist);

        // Look for the nearest free path node
        for(uint32_t i = 0 ; i < indices.size(); i++){
            if(freeLineMarginVB(goal, tree_graph[indices[i]].node.position, mapPtr, tsdf_mapPtr,  map_res, margin, ground_robot, max_slope)){
                near_goal = &tree_graph[indices[i]];
                break;
            } else {
                // If not free path, we select the nearest node (according KDTree)
                near_goal = &tree_graph[indices[0]];
            }

        }

    }

    // ROS_INFO("NEAR_START(%2.2lf, %2.2lf, %2.2lf), NEAR_GOAL(%2.2lf, %2.2lf, %2.2lf)", near_start->node.position.x,
    //                     near_start->node.position.y,
    //                     near_start->node.position.z,
    //                     near_goal->node.position.x,
    //                     near_goal->node.position.y,
    //                     near_goal->node.position.z);


    // Insert first Node to the heap
    opened.push(near_start);

    // ROS_WARN("Pre look");

    // Looking for a plan
    while(!path_found && opened.size() > 0){
        path_found = computeNeighbours(*opened.top());
        opened.pop();
    }

    // ROS_WARN("After look Path points: %s", path_found ? "true" : "false");
    // No path found (impossible??)
    if(!path_found)return path;


    double t_init, t_path, t_opt, t_smooth;

    t_init = ros::Time::now().toSec();

    // Path generation
    pathGenerator(path);


    // When is used only for decision making purposes
    if(!navigation){
        return path;
    }

    raw_path_pub_.publish(path); // OJOOOOOOOOOOOOOOOOOOO ES CORRECTE

    // for(int i = 0; i < path.poses.size(); i++){
    //     printf("(%2.2lf, %2.2lf, %2.2lf)", path.poses[i].pose.position.x, path.poses[i].pose.position.y, path.poses[i].pose.position.z);
    // }

    // t_path = ros::Time::now().toSec();
    // vis_path_pub_.publish(path);

    // Path optimization
    if(!raw_path) pathOptimizer(path);


    vis_path_pub_.publish(path);

    // Path smoothing
    if(path.poses.size() > 2)pathSmoothChaikin(path, 5, 0.25);

    path_pub_.publish(path);

    return path;
}



// Callback to generate the tree-graph
void Planner::timerClb(const ros::TimerEvent &event) {



}


std::vector<GraphNode*> Planner::getNeighbours(ttr::node node){

    std::vector<GraphNode*> neighbours;
    neighbours.push_back(&tree_graph[node.parent_id]);

    for(int i = 0; i < node.neighbours.size(); i++){
        neighbours.push_back(&tree_graph[node.neighbours[i]]);
    }

    return neighbours;

}


bool Planner::computeNeighbours(GraphNode &current){

    std::vector<GraphNode*> neighbours = getNeighbours(current.node);
    current.computed = true;

    // Compute A* Graph Search algorithm
    for (auto& node_ptr : neighbours) {

        // Path found
        if(euclideanDistance(node_ptr->node.position,
                near_goal->node.position) < 0.0000001){

            // The parent is assigned to the near_goal
            // node of the graph
            node_ptr->parent = &current;

            return true;
        }

        // Calculate scores for that neighbour
        double g_cost, h_cost, f_cost;
        g_cost = gCost(current, *node_ptr);
        h_cost = hCost(*node_ptr);
        f_cost = g_cost + h_cost;

        // Update a previously opened node
        if(node_ptr->opened == true && g_cost < node_ptr->g_cost){
            node_ptr->g_cost = g_cost;
            node_ptr->h_cost = h_cost;
            node_ptr->f_cost = f_cost;
            node_ptr->parent = &current;

            // Update the heap
            GraphNode updater;
            updater.f_cost = 0.0;
            updater.h_cost = 0.0;
            opened.push(&updater);
            opened.pop();

        } else if(node_ptr->opened == false) {
            node_ptr->opened = true;
            node_ptr->g_cost = g_cost;
            node_ptr->h_cost = h_cost;
            node_ptr->f_cost = f_cost;
            node_ptr->parent = &current;

            // New node in priority_queue
            opened.push(node_ptr);
        }


    }

    return false;

}


double Planner::gCost(GraphNode parent,
                        GraphNode neighbour){

    double cost = parent.g_cost;
    cost += euclideanDistance(parent.node.position,
                neighbour.node.position);

    return cost;
}


double Planner::hCost(GraphNode neighbour){

    double cost = euclideanDistance(neighbour.node.position, goal_);

    return cost;
}


void Planner::insertPointToPath(const geometry_msgs::Point& point,
                        const std::string& fixed_fr,
                        nav_msgs::Path& path, bool back){

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = path.header.stamp;
    pose.header.frame_id = fixed_fr;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = point.z;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    if(back)path.poses.push_back(pose);
    else path.poses.insert(path.poses.begin(), pose);
}

void Planner::pathGenerator(nav_msgs::Path &path){

    path.header.stamp = ros::Time::now();
    path.header.frame_id = fixed_frame;

    GraphNode* current = near_goal;

    uint32_t counter = 0;
    while (euclideanDistance(current->node.position,
                    near_start->node.position) > 0.0001
                    && counter < tree_graph.size()) {

        // ROS_ERROR("Before");

        insertPointToPath(current->node.position, fixed_frame, path, 0);

        // ROS_ERROR("After");


        current = current->parent;
        counter++;

        // ROS_ERROR("END_LOOP");
    }

    // ROS_ERROR("1");
    
    insertPointToPath(start_, fixed_frame, path, 0);

    // ROS_ERROR("2");

    // Only insert the last node of the path if path is formed by 3 or less nodes
    // if(counter <= 3) insertPointToPath(goal_, fixed_frame, path, 1);
    if(counter <= 1) insertPointToPath(goal_, fixed_frame, path, 1);

    // ROS_ERROR("deu");

    // printf("\nStart: (%2.2lf, %2.2lf, %2.2lf)\n\n", start_.x, start_.y, start_.z);
}


void Planner::pathOptimizer(nav_msgs::Path &path) {

    // ROS_WARN("START (%2.2lf, %2.2lf, %2.2lf)", path.poses[0].pose.position.x,
    //                                             path.poses[0].pose.position.y,
    //                                             path.poses[0].pose.position.z);
    // ROS_WARN("GOAL (%2.2lf, %2.2lf, %2.2lf)", path.poses[path.poses.size()-1].pose.position.x,
    //                                             path.poses[path.poses.size()-1].pose.position.y,
    //                                             path.poses[path.poses.size()-1].pose.position.z);


    nav_msgs::Path aux = path;
    nav_msgs::Path bckp = path;
    path.poses.clear();

    // Insert the start point
    path.poses.push_back(aux.poses[0]);

    // Load Voxblox Maps
    std::shared_ptr<voxblox::EsdfMap> mapPtr = voxblox_server_.getEsdfMapPtr();
    std::shared_ptr<voxblox::TsdfMap> tsdf_mapPtr = voxblox_server_.getTsdfMapPtr();

    int path_nodes = aux.poses.size();
    bool added = false;

    for(int i = 0 ; i < path_nodes - 1 ; i++){

        for(int j = path_nodes - 1 ; j > i ; j--){

            added = false;
            bool free_line = freeLineMarginVB(aux.poses[i].pose.position, aux.poses[j].pose.position,
                                                mapPtr, tsdf_mapPtr,  map_res, margin,
                                                ground_robot, max_slope);

            // ROS_WARN("From [%d](%2.2lf, %2.2lf, %2.2lf) to [%d](%2.2lf, %2.2lf, %2.2lf):",
            //         i, aux.poses[i].pose.position.x, aux.poses[i].pose.position.y, aux.poses[i].pose.position.z,                                                                       
            //         j, aux.poses[j].pose.position.x, aux.poses[j].pose.position.y, aux.poses[j].pose.position.z);

            if(free_line){
                // ROS_WARN("      Added [%d](%2.2lf, %2.2lf, %2.2lf)", j, aux.poses[j].pose.position.x,
                //                                                     aux.poses[j].pose.position.y,
                //                                                     aux.poses[j].pose.position.z);
                path.poses.push_back(aux.poses[j]);
                i = j - 1; // Because the for loop increases i after loop
                added = true;
                break;
            }

        }
        if(!added) path.poses.push_back(aux.poses[i+1]);

    }

    if(path.poses.size() < 2){
        path = bckp;
    }

}


void Planner::pathSmoothChaikin(nav_msgs::Path &path, uint8_t iterations, float percentage){

    // Load Voxblox Map
    std::shared_ptr<voxblox::EsdfMap> mapPtr = voxblox_server_.getEsdfMapPtr();
    std::shared_ptr<voxblox::TsdfMap> tsdf_mapPtr = voxblox_server_.getTsdfMapPtr();

    float percent = percentage;
    int counter = 0;
    for(uint8_t i = 0; i < iterations; i++){
        nav_msgs::Path aux = path;
        path.poses.clear();

        if(counter > 0) percent*0.5;
        counter = 0;

        // Insert the start point
        path.poses.push_back(aux.poses[0]);

        for(int i = 0 ; i < aux.poses.size()-2 ; i++){
            bool applied = false;
            uint16_t aux_cnt = 1;
            while(!applied && aux_cnt <= 5){
            geometry_msgs::Point a, b;
                a = steer(aux.poses[i].pose.position,aux.poses[i+1].pose.position, double(1-(percent/aux_cnt))*
                        euclideanDistance(aux.poses[i].pose.position, aux.poses[i+1].pose.position));
                b = steer(aux.poses[i+1].pose.position, aux.poses[i+2].pose.position, double((percent/aux_cnt))*
                        euclideanDistance(aux.poses[i+1].pose.position, aux.poses[i+2].pose.position));

            if(freeLineMarginVB(a, b, mapPtr, tsdf_mapPtr,
                                    map_res, margin,
                                    ground_robot, max_slope)){
                insertPointToPath(a, fixed_frame, path, 1);
                insertPointToPath(b, fixed_frame, path, 1);
                    applied = true;
            } else {
                counter++;
                    aux_cnt++;
                }
            }
            // Not applied smoothing at this segment
            if(!applied) path.poses.push_back(aux.poses[i+1]);
        }

        //OG
        // for(int i = 0 ; i < aux.poses.size()-2 ; i++){
        //     geometry_msgs::Point a, b;
        //     a = steer(aux.poses[i].pose.position,aux.poses[i+1].pose.position, double(1-percent)*
        //                 euclideanDistance(aux.poses[i].pose.position, aux.poses[i+1].pose.position));
        //     b = steer(aux.poses[i+1].pose.position, aux.poses[i+2].pose.position, double(percent)*
        //                 euclideanDistance(aux.poses[i+1].pose.position, aux.poses[i+2].pose.position));

        //     if(freeLineMarginVB(a, b, mapPtr, tsdf_mapPtr,
        //                             map_res, margin,
        //                             ground_robot, max_slope)){
        //         insertPointToPath(a, fixed_frame, path, 1);
        //         insertPointToPath(b, fixed_frame, path, 1);
        //         // counter++;
        //     } else {
        //         path.poses.push_back(aux.poses[i+1]);
        //         counter++;
        //     }

        // }

        // Insert the goal point
        path.poses.push_back(aux.poses[aux.poses.size()-1]);
    }
}


double Planner::pathLength(const nav_msgs::Path &path){

    double length = 0.0;

    if(path.poses.size() > 0){
        for(int i = 0; i < path.poses.size() - 1 ; i++){
            length += euclideanDistance(path.poses[i].pose.position, path.poses[i+1].pose.position);
        }

        return length;
    }
    return 0.0;
}


// double Planner::getMapDistance(const Eigen::Vector3d& position) const {
//   if (!voxblox_server_.getEsdfMapPtr()) {
//     return -1.0;
//   }
//   double distance = 0.0;
//   if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position,
//                                                               &distance)) {
//     return -2.0;
//   }
//   return distance;
// }



}  // namespace ttr
