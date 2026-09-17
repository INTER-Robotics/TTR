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

#include "persistent_tree.h"

namespace ttr {

PersistentTree::PersistentTree(const ros::NodeHandle &nh,
                 const ros::NodeHandle& nh_private)
        : nh_(nh),
          nh_private_(nh_private),
          voxblox_server_(nh_, nh_private_)
{
    configure();
    
}

PersistentTree::~PersistentTree() {
}

// void PersistentTree::dynReconfig(ttr::validatorConfig &config, uint32_t level) {
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

void PersistentTree::configure() {
    
    double freq_check = 10.0;
    double frequency;
    nh_private_.param("frequency", frequency, 20.0);
    ROS_INFO("Frequency of the node: %2.2lfHz", frequency);

    nh_private_.param<std::string>("fixed_frame", fixed_frame, "map");
    ROS_INFO("Fixed frame: %s", fixed_frame.c_str());

    nh_private_.param<std::string>("base_frame", base_frame, "base_link");
    ROS_INFO("Base frame: %s", base_frame.c_str());

    nh_private_.param("max_branch_length", step, 1.0);
    ROS_INFO("Maximum length of the tree branches: %2.2lfm", step);

    nh_private_.param("map_resolution", map_res, 0.35);
    ROS_INFO("Voxel size of the voxblox: %2.2lfm", map_res);

    nh_private_.param("max_x", max_x, 200.0);
    ROS_INFO("Limit maximum X: %2.2lfm", max_x);

    nh_private_.param("min_x", min_x, -200.0);
    ROS_INFO("Limit minimum X: %2.2lfm", min_x);

    nh_private_.param("max_y", max_y, 200.0);
    ROS_INFO("Limit maximum Y: %2.2lfm", max_y);

    nh_private_.param("min_y", min_y, -200.0);
    ROS_INFO("Limit minimum Y: %2.2lfm", min_y);

    nh_private_.param("max_z", max_z, 200.0);
    ROS_INFO("Limit maximum Z: %2.2lfm", max_z);

    nh_private_.param("min_z", min_z, -200.0);
    ROS_INFO("Limit minimum Z: %2.2lfm", min_z);

    nh_private_.param("margin", margin, 0.5);
    ROS_INFO("Distance margin for accept frontier point: %2.2lfm", margin);

    nh_private_.param("gain_radius", rad_gain, 1.0);
    ROS_INFO("Radius to obtain information gain: %2.2lfm", rad_gain);

    nh_private_.param("min_information_gain", info_gain, 0.25);
    ROS_INFO("Minimum information gain of the frontiers: %2.2lf", info_gain);

    nh_private_.param("improved_frontiers_search", imp_search, false);
    ROS_INFO("Search frontiers in graph borders: %s", imp_search ? "true" : "false");

    nh_private_.param("check_path_new_connections", check_path_new_connections, false);
    ROS_INFO("Check path length for new connections: %s", check_path_new_connections ? "true" : "false");
    
    nh_private_.param("original_add_branch", original_add_branch, true);
    ROS_INFO("Use original add branch function: %s", original_add_branch ? "true" : "false");

    nh_private_.param("ground_robot", ground_robot, false);
    ROS_INFO("Ground robot: %s", ground_robot ? "true" : "false");

    if(ground_robot){
        nh_private_.param("projection_height", proj_height, 0.5);
        ROS_INFO("Height at which points are set after projection: %2.2lfm", proj_height);

        nh_private_.param("max_slope", max_slope, 20.0);
        ROS_INFO("Maximum allowed slope (degree): %2.2lf deg", max_slope);
    }


    // Visualization
    line.header.frame_id = fixed_frame;
    line.header.stamp = ros::Time::now();
    line.ns = "global_branches";
    line.id = 0;
    line.type = line.LINE_LIST;
    line.action = line.ADD;
    line.pose.orientation.x = 0.0;
    line.pose.orientation.y = 0.0;
    line.pose.orientation.z = 0.0;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.03;
    line.color.r = 0.0 / 255.0;
    line.color.g = 255.0 / 255.0;
    line.color.b = 255.0 / 255.0;
    line.color.a = 1.0;
    line.lifetime = ros::Duration(1);

    local_branch_line.header.frame_id = fixed_frame;
    local_branch_line.header.stamp = ros::Time::now();
    local_branch_line.ns = "local_branches";
    local_branch_line.id = 1;
    local_branch_line.type = line.LINE_LIST;
    local_branch_line.action = line.ADD;
    local_branch_line.pose.orientation.x = 0.0;
    local_branch_line.pose.orientation.y = 0.0;
    local_branch_line.pose.orientation.z = 0.0;
    local_branch_line.pose.orientation.w = 1.0;
    local_branch_line.scale.x = 0.03;
    local_branch_line.color.r = 255.0 / 255.0;
    local_branch_line.color.g = 0.0 / 255.0;
    local_branch_line.color.b = 255.0 / 255.0;
    local_branch_line.color.a = 1.0;
    local_branch_line.lifetime = ros::Duration(1);

    new_connection_line.header.frame_id = fixed_frame;
    new_connection_line.header.stamp = ros::Time::now();
    new_connection_line.ns = "new_connections";
    new_connection_line.id = 2;
    new_connection_line.type = line.LINE_LIST;
    new_connection_line.action = line.ADD;
    new_connection_line.pose.orientation.x = 0.0;
    new_connection_line.pose.orientation.y = 0.0;
    new_connection_line.pose.orientation.z = 0.0;
    new_connection_line.pose.orientation.w = 1.0;
    new_connection_line.scale.x = 0.03;
    new_connection_line.color.r = 255.0 / 255.0;
    new_connection_line.color.g = 255.0 / 255.0;
    new_connection_line.color.b = 0.0 / 255.0;
    new_connection_line.color.a = 0.4;
    new_connection_line.lifetime = ros::Duration(1);

    keypoints_line.header.frame_id = fixed_frame;
    keypoints_line.header.stamp = ros::Time::now();
    keypoints_line.ns = "keypoints";
    keypoints_line.id = 3;
    keypoints_line.type = line.LINE_LIST;
    keypoints_line.action = line.ADD;
    keypoints_line.pose.orientation.x = 0.0;
    keypoints_line.pose.orientation.y = 0.0;
    keypoints_line.pose.orientation.z = 0.0;
    keypoints_line.pose.orientation.w = 1.0;
    keypoints_line.scale.x = 0.03;
    keypoints_line.color.r = 255.0 / 255.0;
    keypoints_line.color.g = 0.0 / 255.0;
    keypoints_line.color.b = 255.0 / 255.0;
    keypoints_line.color.a = 1.0;
    keypoints_line.lifetime = ros::Duration(1);


    r_points.header.frame_id = fixed_frame;
    r_points.header.stamp = ros::Time::now();
    r_points.ns = "random_point";
    r_points.id = 0;
    r_points.type = r_points.SPHERE_LIST;
    r_points.action = r_points.ADD;
    r_points.pose.orientation.x = 0.0;
    r_points.pose.orientation.y = 0.0;
    r_points.pose.orientation.z = 0.0;
    r_points.pose.orientation.w = 1.0;
    r_points.scale.x = 5;
    r_points.scale.y = 5;
    r_points.scale.z = 5;
    r_points.color.r = 255.0 / 255.0;
    r_points.color.g = 0.0 / 255.0;
    r_points.color.b = 0.0 / 255.0;
    r_points.color.a = 1.0;
    r_points.lifetime = ros::Duration(10.0);

    n_points.header.frame_id = fixed_frame;
    n_points.header.stamp = ros::Time::now();
    n_points.ns = "nearest_point";
    n_points.id = 0;
    n_points.type = n_points.SPHERE_LIST;
    n_points.action = n_points.ADD;
    n_points.pose.orientation.x = 0.0;
    n_points.pose.orientation.y = 0.0;
    n_points.pose.orientation.z = 0.0;
    n_points.pose.orientation.w = 1.0;
    n_points.scale.x = 0.5;
    n_points.scale.y = 0.5;
    n_points.scale.z = 0.5;
    n_points.color.r = 0.0 / 255.0;
    n_points.color.g = 0.0 / 255.0;
    n_points.color.b = 255.0 / 255.0;
    n_points.color.a = 1.0;
    n_points.lifetime = ros::Duration(10.0);


    // Publishers
    lines_pub_ = nh_.advertise<visualization_msgs::Marker>("/tree_graph", 10);
    frontier_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/detected_points", 10);
    graph_pub_ = nh_.advertise<ttr::treeGraph>("/planner_graph", 10);
    points_pub_ = nh_.advertise<visualization_msgs::Marker>("/points", 10);


    // Subscribers
    // trav_pc_sub_ = nh_.subscribe("/voxblox_node/traversable", 1, &PersistentTree::traversablePointCloudClb, this);
    // obs_pc_sub_ = nh_.subscribe("/voxblox_node/surface_pointcloud", 1, &PersistentTree::obstaclePointCloudClb, this);
    local_branch_sub_ = nh_.subscribe("/local_branch", 500, &PersistentTree::localBranchClb, this);
    exploration_state_sub_ = nh_.subscribe("/exploration_state", 1, &PersistentTree::explorationStateClb, this);



    // Advertising Services
    // makePlan_ = nh_.advertiseService("make_plan", &PersistentTree::makePlan, this);

    // Service clients
    get_frontier_path_lengths_client_ = nh_.serviceClient<ttr::PathLengths>("/planner/get_frontier_path_lengths");

    // Keypoints (FUTURE WORKS)
    // keypoints_tree.push_back(root_node);
    // keypoints_pc->points.resize(1);
    // (*keypoints_pc)[0].x = aux.x;
    // (*keypoints_pc)[0].y = aux.y;
    // (*keypoints_pc)[0].z = aux.z;

    // Wait for receive voxblox pointclouds
    // while(!obs_received || !trav_received){
    //     ros::spinOnce();
    //     ros::Duration(0.1).sleep();
    // } 
    // ROS_INFO("Voxblox PCs received.");

    while(!voxblox_server_.getEsdfMapPtr() || !voxblox_server_.getTsdfMapPtr()){
        ros::Duration(0.1).sleep();
    }
    // ros::Duration(10.0).sleep();
    ROS_INFO("Voxblox received.");

    init_time = ros::Time::now().toSec();
    last_print = ros::Time::now().toSec();

    // Timers
    timer_ = nh_.createTimer(ros::Duration(1.0 / frequency), &PersistentTree::timerClb, this);  
    check_timer_ = nh_.createTimer(ros::Duration(1.0 / freq_check), &PersistentTree::timerCheckClb, this);
}


void PersistentTree::initialization() { // Persistent-tree initialization
    
    std::shared_ptr<voxblox::EsdfMap> esdf_map_ptr = voxblox_server_.getEsdfMapPtr();
    if(esdf_map_ptr == nullptr){
        ROS_ERROR("NULL ESDF!");
        return;
    }
    std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr = voxblox_server_.getTsdfMapPtr();
    if(tsdf_map_ptr == nullptr){
        ROS_ERROR("NULL TSDF!");
        return;
    }

    // Seed initialization
    srand(time(NULL));

    geometry_msgs::Point aux = getRobotPosition(fixed_frame, base_frame, listener, transform);

    // tree_graph.push_back(Node(0, 65535, aux));
    ttr::node root_node;
    root_node.id = 0;
    root_node.parent_id = UINT32_MAX;
    root_node.position = aux;
    tree.emplace(0, root_node);
    last_key = 0;

    // Tree-graph KDTree generationsize()
    tr_gr->points.resize(1);
    (*tr_gr)[0].x = aux.x;
    (*tr_gr)[0].y = aux.y;
    (*tr_gr)[0].z = aux.z;

    // KD-Tree is updated
    kdtree.setInputCloud(tr_gr);
}

void PersistentTree::localBranchClb(const ttr::branch &branch){

    if(first_loop) return;

    if(original_add_branch){

        // ROS_WARN("Time received branch %d: %lf", br_cnt, ros::Time::now().toSec());
        // br_cnt++;
        if(addBranchOri(branch.nodes, branch.frontier)){
            // The new frontier is published
            frontier_pub_.publish(branch.frontier);
        }
    } else {
        if(addBranch(branch.nodes, branch.frontier)){
            // The new frontier is published
            frontier_pub_.publish(branch.frontier);
        }
    }

}


// Callback to generate the tree-graph
void PersistentTree::timerClb(const ros::TimerEvent &event) {

    // Exploration State
    switch (state)
    {
    case EXPLORING:
        if(first_loop){
            initialization();
            first_loop = false;
        }
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

    // ROS_INFO("Time from last branch try: %2.6lf", ros::Time::now().toSec() - last_print);

    // Getting voxblox ESDF and TSDF map pointer
    std::shared_ptr<voxblox::EsdfMap> esdf_map_ptr = voxblox_server_.getEsdfMapPtr();
    if(esdf_map_ptr == nullptr){
        ROS_ERROR("NULL ESDF!");
        return;
    }
    std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr = voxblox_server_.getTsdfMapPtr();
    if(tsdf_map_ptr == nullptr){
        ROS_ERROR("NULL TSDF!");
        return;
    }


    geometry_msgs::Point q_rand, q_near, q_new;

    q_rand = getRandomPoint(min_x, max_x, min_y, max_y, min_z, max_z);
    
    r_points.points.push_back(q_rand);

    // Search nearest Node in tree_graph
    pcl::PointXYZ searchPoint;
    searchPoint.x = q_rand.x;
    searchPoint.y = q_rand.y;
    searchPoint.z = q_rand.z;
    // kdtree.setInputCloud(tr_gr);

    std::vector<int> indices;
    std::vector<float> sq_dist;

    // N Neighbours 
    int desired_NN = 20;
    int num_NN;
    if(last_key+1 < desired_NN){
        num_NN = last_key+1;
    } else {
        num_NN = desired_NN;
    }
    kdtree.nearestKSearch(searchPoint, num_NN, indices, sq_dist);

    // The Nearest Neighbour
    q_near.x = (*tr_gr)[indices[0]].x;
    q_near.y = (*tr_gr)[indices[0]].y;
    q_near.z = (*tr_gr)[indices[0]].z;

    q_new = steer(q_near, q_rand, step);

    // char check = freeLineAtDistance(q_near, &q_new, obs_kdtree, trav_kdtree, map_res, margin);
    signed char check = freeLineAtDistanceVB(q_near, &q_new, esdf_map_ptr, tsdf_map_ptr,
                    map_res, margin, ground_robot, max_slope, proj_height);

    // // Try to find frontier with the number 'desired_NN' Nearest Neighbours
    if(imp_search && check != -1) {
        for(int i = 1; i < num_NN; i++){
            geometry_msgs::Point aux_point;
            
            aux_point.x = (*tr_gr)[indices[i]].x;
            aux_point.y = (*tr_gr)[indices[i]].y;
            aux_point.z = (*tr_gr)[indices[i]].z;

            geometry_msgs::Point aux_new_point = steer(aux_point, q_rand, step);

            // char aux_check = freeLineAtDistance(aux_point, &aux_new_point,
            //                       obs_kdtree, trav_kdtree, map_res, margin);
            signed char aux_check = freeLineAtDistanceVB(aux_point, &aux_new_point,
                                                    esdf_map_ptr, tsdf_map_ptr,
                                                    map_res, margin, ground_robot,
                                                    max_slope, proj_height);

            if(aux_check == -1){
                q_near = aux_point;
                q_new = aux_new_point;
                check = aux_check;
                break;
            }

        }
    }
    
    

    // Frontier Found
    if (check == -1) {
        
        // Frontier with minimum volumetric gain found
        if(ground_robot){

            // Publish Frontiers
            geometry_msgs::PointStamped frontier;
            frontier.header.stamp = ros::Time::now();
            frontier.header.frame_id = fixed_frame;
            frontier.point = q_new;
            frontier_pub_.publish(frontier);

        } else if(getInformationGain(q_new, tsdf_map_ptr, map_res, rad_gain, 1.5) >= info_gain){
            // Publish Frontiers
            geometry_msgs::PointStamped frontier;
            frontier.header.stamp = ros::Time::now();
            frontier.header.frame_id = fixed_frame;
            frontier.point = q_new;
            frontier_pub_.publish(frontier);
            // ROS_WARN("Frontier FOUND!!!!!!!!!!!!!!!!!!!!");
        }
    }

    // New tree node
    else if (check == 0) {

        ttr::node aux_node;
        last_key++;
        aux_node.id = last_key;
        // TODO algo perque si s'eliminen nodes de l'arbre l'index
        // de PC serà diferent (s'hauria d'asignar key del parent)
        aux_node.parent_id = indices[0]; 
        aux_node.position = q_new;
        tree[aux_node.parent_id].neighbours.push_back(aux_node.id);
        tree.emplace(last_key, aux_node);


        // KD_tree          
        // TODO se podria fer que quan un node s'elimina de l'arbre es canvii la
        // posició a una molt llunyana (aixi mantenint estructura de indices)
        tr_gr->points.resize(last_key + 1);
        (*tr_gr)[last_key].x = q_new.x;
        (*tr_gr)[last_key].y = q_new.y;
        (*tr_gr)[last_key].z = q_new.z;


        // Trying to add new connections
        // newConnections(last_key, 3.0);

        // KD-Tree is updated
        kdtree.setInputCloud(tr_gr);

        line.points.push_back(q_new);
        line.points.push_back(q_near);
    }

    // ROS_INFO("Graph nodes: %ld", last_key);
    lines_pub_.publish(line);
    lines_pub_.publish(local_branch_line);
    lines_pub_.publish(new_connection_line);

    last_print = ros::Time::now().toSec();

    // lines_pub_.publish(keypoints_line);

}

uint8_t cnt = 0;
void PersistentTree::timerCheckClb(const ros::TimerEvent &event) {

    if(first_loop) return;
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

    // POSAR
    newConnections(8.0);
    newCloseConnections(5.0, 3.0);

    double time_gr = ros::Time::now().toSec();
    sendGraph();
    // ROS_INFO("Time to send new tree-graph: %1.9lf", ros::Time::now().toSec() - time_gr);


    // Generate KeyPoints and its connections
    // if(cnt > 10){
    //     keypointsStuff();
    //     cnt = 0;
    // }
    // cnt++;
    // line.points.clear();
    // for(int i = 0; i < tree_graph.size() ; i++){
    //     if(obstacleAtDistance(tree_graph[i].position, 0.5, obstacles_kdtree)){
    //         // deleteBranch(i);
    //         ROS_INFO("BORRAU!!");
    //     } else {
    //         for(int j = 0; j < tree_graph[i].neighbours.size() ; j++){
    //             line.points.push_back(tree_graph[i].position);
    //             line.points.push_back(tree_graph[tree_graph[i].neighbours[j]].position);
    //         }
    //     }
    // }
    // lines_pub_.publish(line);
}

void PersistentTree::explorationStateClb(const std_msgs::UInt8::ConstPtr& state_msg) {
    state = static_cast<ExplorationState>(state_msg->data);
}

void PersistentTree::deleteBranch(const unsigned int tree_node) {

    // if(tree_graph[tree_node].neighbours.size() == 0){
    //     // Set Node to Invalid
    //     tree_graph[tree_node].parent_id = 65535;
    //     tree_graph[tree_node].neighbours.clear();
    //     return;
    // }

    // for(unsigned int i = 0 ; i < tree_graph[tree_node].neighbours.size() ; i++){
    //     unsigned int index = tree_graph[tree_node].neighbours[i];
    //     if(tree_graph[index].parent_id == tree_node){
    //         tree_graph[index].neighbours.clear();
    //     }
    // }

}

bool PersistentTree::addBranch(std::vector<ttr::node> nodes,
                                geometry_msgs::PointStamped frontier){

    std::shared_ptr<voxblox::EsdfMap> esdf_map_ptr = voxblox_server_.getEsdfMapPtr();
    std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr = voxblox_server_.getTsdfMapPtr();
    bool link_branch = false;
    bool add_new_connections = false;
    bool send_graph = false;
    int end_for = 0;
    double branch_factor = 3.5;
    std::vector<ttr::node> aux_branch;

    // Check length of branch
    double branch_length = 0.0;
    for(uint16_t i = 1; i < nodes.size() ; i++){
        branch_length += euclideanDistance(nodes[i-1].position, nodes[i].position);
        aux_branch.push_back(nodes[i-1]); // Update aux_branch to update the global tree

    }
    // Check path length using global graph
    ttr::PathLengths get_frontier_path_lengths;
    get_frontier_path_lengths.request.raw_length = true;
    get_frontier_path_lengths.request.start = nodes.front().position; // Root node of the branch
    get_frontier_path_lengths.request.frontiers.push_back(nodes.back().position);

    // TODO Arreglar això
    pcl::PointXYZ frontierPoint;
    frontierPoint.x = frontier.point.x;
    frontierPoint.y = frontier.point.y;
    frontierPoint.z = frontier.point.z;

    std::vector<int> a_indices;
    std::vector<float> a_sq_dist;
    kdtree.radiusSearch(frontierPoint, step+0.2, a_indices, a_sq_dist, 1);
    bool path;
    double path_length = 0.0; 

    // If frontier is in a zone without tree -> original add branch
    if(a_indices.size() > 0){
        path = get_frontier_path_lengths_client_.call(get_frontier_path_lengths);

        if (path) path_length = get_frontier_path_lengths.response.lengths[0];
        ROS_WARN("Branch: %2.3lf * %1.1lf = %2.3lf || Path: %2.3lf",
                    branch_length, branch_factor,
                    branch_length*branch_factor, path_length );
    } else {
        path = false;
    }

    if(path && branch_length*branch_factor < path_length ){
            
            // Add all branch
            link_branch = true;
            // add_new_connections = true;
            send_graph = true;
            end_for = 0;
            ROS_INFO("Full branch.");

    } else {
        

        // Check if frontier can be connected directly to the current tree
        // pcl::PointXYZ frontierPoint;
        // frontierPoint.x = frontier.point.x;
        // frontierPoint.y = frontier.point.y;
        // frontierPoint.z = frontier.point.z;

        std::vector<int> f_indices;
        std::vector<float> f_sq_dist;
        int8_t num_neighs = 5;
        if(tr_gr->points.size() < num_neighs) num_neighs = tr_gr->points.size();
        kdtree.radiusSearch(frontierPoint, step, f_indices, f_sq_dist, num_neighs);
        if(f_indices.size() >= 1){
            for(int8_t i = 0 ; i < f_indices.size() ; i++){
                // printf("found: %d, id: %d, (%2.3lf, %2.3lf, %2.3lf)\n",f_indices.size(), f_indices[i],
                //                              tree[f_indices[i]].position.x,
                //                              tree[f_indices[i]].position.y,
                //                              tree[f_indices[i]].position.z);
                if(freeLineMarginVB(frontier.point, tree[f_indices[i]].position,
                                            esdf_map_ptr, tsdf_map_ptr, map_res,
                                            margin, ground_robot, max_slope)){
                    return true;
                }
            }
        }

        end_for = nodes.size()-1;

        // Adding branch to global tree
        aux_branch.clear();
        link_branch = false;
        ROS_INFO("Piece of branch.");


    }

    
    // Begin from the last node of the branch (the one closest to the frontier found)
    ttr::node aux;
    for(int i = end_for; i >= 0 ; i--){

        // Check the nearest node of the branch node
        pcl::PointXYZ searchPoint;
        searchPoint.x = nodes[i].position.x;
        searchPoint.y = nodes[i].position.y;
        searchPoint.z = nodes[i].position.z;
        // kdtree.setInputCloud(tr_gr); // We do not have to update for the new branch nodes

        std::vector<int> indices;
        std::vector<float> sq_dist;
        kdtree.nearestKSearch(searchPoint, 1, indices, sq_dist);

        // Filling aux node
        aux.position = nodes[i].position;

        // If free line with set margin from current branch node to nearest global tree node -> the branch up to 
        // current branch node is added to global tree
        if(freeLineMarginVB(tree[indices[0]].position, nodes[i].position,
                                esdf_map_ptr, tsdf_map_ptr, map_res,
                                margin, ground_robot, max_slope)
            && euclideanDistance(tree[indices[0]].position,
                                    nodes[i].position) <= step){

            // Updating parent and neighbours of parent
            aux.parent_id = tree[indices[0]].id; 
            aux_branch.insert(aux_branch.begin(), aux);
        
            link_branch = true; // The branch can be added to the global tree
            break;
        
        }

        // Add node to temporal nodes vector (aux_branch)
        aux_branch.insert(aux_branch.begin(), aux);

        
    }


    // Add the branch to the global tree
    if(link_branch){

        for(int n = 0 ; n < aux_branch.size() ; n++){

            // Add node to global tree
            last_key++;
            // The id of the node is set
            aux_branch[n].id = last_key;
            // IF the node is not the ones linked to global tree
            // (for which the parent_id is set by global tree NN)
            // the parent_id is set as the last branch node added to the global tree
            if(n!=0)aux_branch[n].parent_id = aux_branch[n-1].id;
            tree.emplace(last_key, aux_branch[n]);

            // The neighbours of the last node
            // (n=0 is a global tree node, others are branch nodes)
            tree[aux_branch[n].parent_id].neighbours.push_back(aux_branch[n].id);

            // Add node to global tree KD-Tree
            tr_gr->points.resize(last_key + 1);
            (*tr_gr)[last_key].x = aux_branch[n].position.x;
            (*tr_gr)[last_key].y = aux_branch[n].position.y;
            (*tr_gr)[last_key].z = aux_branch[n].position.z;

            // Visualization
            local_branch_line.points.push_back(tree[aux_branch[n].parent_id].position);
            local_branch_line.points.push_back(tree[aux_branch[n].id].position); 

            // Add new connections
            if(add_new_connections) newConnections(last_key, 5.0);

            // Send graph
            if(send_graph) sendGraph(); 

            
        }


        // Look for new connections to the last branch node
        // float aux_rand = (float)rand()/RAND_MAX;
        // if(aux_rand < 0.1) newConnections(last_key, 3.0);

        // KD-Tree is updated
        kdtree.setInputCloud(tr_gr);

        return true;
    }


    return false;

}

bool PersistentTree::addBranchOri(std::vector<ttr::node> nodes,
                                    geometry_msgs::PointStamped frontier){

    std::shared_ptr<voxblox::EsdfMap> esdf_map_ptr = voxblox_server_.getEsdfMapPtr();
    std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr = voxblox_server_.getTsdfMapPtr();

    double max_distance_to_add = 3.0;

    // Check if frontier can be connected directly to the current tree
    pcl::PointXYZ frontierPoint;
    frontierPoint.x = frontier.point.x;
    frontierPoint.y = frontier.point.y;
    frontierPoint.z = frontier.point.z;

    std::vector<int> f_indices;
    std::vector<float> f_sq_dist;
    int8_t num_neighs = 5;
    if(tr_gr->points.size() < num_neighs) num_neighs = tr_gr->points.size();
    // kdtree.nearestKSearch(frontierPoint, num_neighs, f_indices, f_sq_dist);
    // kdtree.radiusSearch(frontierPoint, step, f_indices, f_sq_dist, num_neighs);
    kdtree.radiusSearch(frontierPoint, max_distance_to_add, f_indices, f_sq_dist, num_neighs);

    if(f_indices.size() >= 1){
        for(int8_t i = 0 ; i < f_indices.size() ; i++){
            // printf("found: %d, id: %d, (%2.3lf, %2.3lf, %2.3lf)\n",f_indices.size(), f_indices[i],  tree[f_indices[i]].position.x,
            //             tree[f_indices[i]].position.y, tree[f_indices[i]].position.z);
            if(freeLineMarginVB(frontier.point, tree[f_indices[i]].position,
                                    esdf_map_ptr, tsdf_map_ptr, map_res,
                                    margin, ground_robot, max_slope)) return true;
        }
    }


    // Adding branch to global tree
    ttr::node aux;
    std::vector<ttr::node> aux_branch;
    bool link_branch = false;

    // Begin from the last node of the branch (the one closest to the frontier found)
    for(int i = nodes.size()-1; i >= 0 ; i--){

        // Check the nearest node of the branch node
        pcl::PointXYZ searchPoint;
        searchPoint.x = nodes[i].position.x;
        searchPoint.y = nodes[i].position.y;
        searchPoint.z = nodes[i].position.z;
        // kdtree.setInputCloud(tr_gr); // We do not have to update for the new branch nodes

        std::vector<int> indices;
        std::vector<float> sq_dist;
        kdtree.nearestKSearch(searchPoint, 1, indices, sq_dist);

        // Filling aux node
        aux.position = nodes[i].position;

        // If free line with set margin from current branch node to nearest global tree node -> the branch up to 
        // current branch node is added to global tree
        if(freeLineMarginVB(tree[indices[0]].position, nodes[i].position,
                                esdf_map_ptr, tsdf_map_ptr, map_res,
                                margin, ground_robot, max_slope)
            && euclideanDistance(tree[indices[0]].position,
                                    nodes[i].position) <= max_distance_to_add){ // step

            // Updating parent and neighbours of parent                                                                          tree[indices[0]].position.z);
            aux.parent_id = tree[indices[0]].id; 
            aux_branch.insert(aux_branch.begin(), aux);
        
            link_branch = true; // The branch can be added to the global tree
            break;
        
        }

        // Add node to temporal nodes vector (aux_branch)
        aux_branch.insert(aux_branch.begin(), aux);

    }

    // Add the branch to the global tree
    if(link_branch){

        for(int n = 0 ; n < aux_branch.size() ; n++){

            // Add node to global tree
            last_key++;
            // The id of the node is set
            aux_branch[n].id = last_key;
            // IF the node is not the ones linked to global tree
            // (for which the parent_id is set by global tree NN)
            // the parent_id is set as the last branch node added to the global tree
            if(n!=0)aux_branch[n].parent_id = aux_branch[n-1].id;
            tree.emplace(last_key, aux_branch[n]);

            // The neighbours of the last node
            // (n=0 is a global tree node, others are branch nodes)
            tree[aux_branch[n].parent_id].neighbours.push_back(aux_branch[n].id);

            // Add node to global tree KD-Tree
            tr_gr->points.resize(last_key + 1);
            (*tr_gr)[last_key].x = aux_branch[n].position.x;
            (*tr_gr)[last_key].y = aux_branch[n].position.y;
            (*tr_gr)[last_key].z = aux_branch[n].position.z;

            // Visualization
            local_branch_line.points.push_back(tree[aux_branch[n].parent_id].position);
            local_branch_line.points.push_back(tree[aux_branch[n].id].position); 

            
        }

        // Look for new connections to the last branch node
        // float aux_rand = (float)rand()/RAND_MAX;
        // if(aux_rand < 0.1) newConnections(last_key, 3.0);

        // KD-Tree is updated
        kdtree.setInputCloud(tr_gr);

        return true;
    }

    return false;

}

// TODO Optimize to send only updated nodes
// TODO Save the graph occasionally at disk
void PersistentTree::sendGraph(){

    ttr::treeGraph graph;

    for (auto it = tree.begin(); it != tree.end(); ++it){
        graph.keys.push_back(it->first);
        graph.nodes.push_back(it->second);

    }

    graph_pub_.publish(graph);
}

void PersistentTree::newConnections(double max_dist_connection){

    // Select a random node of the tree with no neighbours (only have parent)
    uint32_t random;
    for(int m = 0 ; m < 1000 ; m++){
        random = rand() % (tree.size()-1 + 1);
        if(tree[random].neighbours.size() < 3)break;
        // If point not found exit
        if(m == 999) return;
    }

    performNewConnections(random, max_dist_connection);

}

void PersistentTree::newCloseConnections(double max_dist_connection,
                                            double action_radius){

    // Seed initialization
    srand(time(NULL));

    geometry_msgs::Point pose = getRobotPosition(fixed_frame, base_frame, listener, transform);

    // Select a nodes inside the sphere of radius and
    // center to robot position
    pcl::PointXYZ searchPoint;
    searchPoint.x = pose.x;
    searchPoint.y = pose.y;
    searchPoint.z = pose.z;

    std::vector<int> indices;
    std::vector<float> sq_dist;
    kdtree.radiusSearch(searchPoint, action_radius, indices, sq_dist);

    for(int i = 0 ; i < indices.size() ; i++){
        if(tree[indices[i]].neighbours.size() < 3){
            performNewConnections(indices[i], max_dist_connection);
        }
    }


}

void PersistentTree::newConnections(uint32_t node_key, double max_dist_connection){
    performNewConnections(node_key, max_dist_connection);
}

void PersistentTree::performNewConnections(uint32_t node_key,
                                    double max_dist_connection){

    int8_t tries = 10;
    geometry_msgs::Point point;

    point.x = tree[node_key].position.x;
    point.y = tree[node_key].position.y;
    point.z = tree[node_key].position.z;

    pcl::PointXYZ searchPoint;
    searchPoint.x = point.x;
    searchPoint.y = point.y;
    searchPoint.z = point.z;

    std::vector<int> indices;
    std::vector<float> sq_dist;
    // uint8_t num_neighs = tries + 1;
    // if(tr_gr->points.size() < 11)num_neighs = tr_gr->points.size();
    
    if(tr_gr->points.size() > tries){

        // Getting voxblox ESDF and TSDF map pointer
        std::shared_ptr<voxblox::EsdfMap> esdf_map_ptr = voxblox_server_.getEsdfMapPtr();
        if(esdf_map_ptr == nullptr){
            ROS_ERROR("NULL ESDF!");
            return;
        }
        std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr = voxblox_server_.getTsdfMapPtr();
        if(tsdf_map_ptr == nullptr){
            ROS_ERROR("NULL TSDF!");
            return;
        }
    
        kdtree.radiusSearch(searchPoint, max_dist_connection, indices, sq_dist);
        geometry_msgs::Point near, far; 


        bool near_connected = false;
        bool far_connected = false;
        int8_t far_ctr = 0;
        if(indices.size() < tries*2) tries = std::floor(indices.size()/2);

        // The three closest and the three farthest of the radiusSearch
        for(int i = 1 ; i <= tries; i++){

            far.x = (*tr_gr)[indices[indices.size() - i]].x;
            far.y = (*tr_gr)[indices[indices.size() - i]].y;   
            far.z = (*tr_gr)[indices[indices.size() - i]].z;

            near.x = (*tr_gr)[indices[i-1]].x;
            near.y = (*tr_gr)[indices[i-1]].y;   
            near.z = (*tr_gr)[indices[i-1]].z;              

            
            ttr::PathLengths get_frontier_path_lengths;    
            bool check_path;
            if(check_path_new_connections){

                // Check path length using global graph
                get_frontier_path_lengths.request.raw_length = true;
                get_frontier_path_lengths.request.start = point; // Root node of the branch
                get_frontier_path_lengths.request.frontiers.push_back(far);
                get_frontier_path_lengths.request.frontiers.push_back(near);

                // If can not find path -> cancel new connections
                if(!get_frontier_path_lengths_client_.call(get_frontier_path_lengths)) return;

                // ROS_WARN("Dir: %2.2lf*%1.1lf= %2.2lf < Length: %2.2lf",
                //             euclideanDistance(point, far), f_factor,
                //             euclideanDistance(point, far)*f_factor,
                //             get_frontier_path_lengths.response.lengths[0]);

                check_path = euclideanDistance(point, far)*f_factor < get_frontier_path_lengths.response.lengths[0];
            } else { check_path = true;}

            // If is not the parent and the line is free of obstacles (far points)
            if(!far_connected && tree[indices[indices.size() - i]].neighbours.size() < 5
               && check_path // new
               && !insideNeighbours(tree[indices[indices.size() - i]].neighbours, node_key)
               && tree[node_key].parent_id != indices[indices.size() - i]
               && tree[indices[indices.size() - i]].parent_id != node_key
               && freeLineMarginVB(point, far, esdf_map_ptr, tsdf_map_ptr, map_res, margin, ground_robot, max_slope)){

                tree[node_key].neighbours.push_back(indices[indices.size() - i]);
                tree[indices[indices.size() - i]].neighbours.push_back(node_key);
                new_connection_line.points.push_back(point);
                new_connection_line.points.push_back(far);
                // ROS_WARN("NEW CONNECTION FAR");
                // if(far_ctr > std::ceil(tries/5)) far_connected = true;
                // far_ctr++;
                far_connected = true;
            }

            if(check_path_new_connections){
                // ROS_WARN("Dir: %2.2lf*%1.1lf= %2.2lf < Length: %2.2lf",
                //             euclideanDistance(point, near), n_factor,
                //             euclideanDistance(point, near)*n_factor,
                //             get_frontier_path_lengths.response.lengths[1]);
                check_path = euclideanDistance(point, near)*n_factor < get_frontier_path_lengths.response.lengths[1];
            } else { check_path = true;}

            // If is not the parent and the line is free of obstacles (close points)
            if(!near_connected && tree[indices[i-1]].neighbours.size() < 3 
               && check_path // new
               && !insideNeighbours(tree[indices[i-1]].neighbours, node_key)
               && tree[node_key].parent_id != indices[i-1] && sq_dist[i-1] >= 2.0
               && tree[indices[i-1]].parent_id != node_key
               && freeLineMarginVB(point, near, esdf_map_ptr, tsdf_map_ptr, map_res, margin, ground_robot, max_slope)){

                tree[node_key].neighbours.push_back(indices[i-1]);
                tree[indices[i-1]].neighbours.push_back(node_key);
                new_connection_line.points.push_back(point);
                new_connection_line.points.push_back(near);
                // ROS_WARN("NEW CONNECTION NEAR");
                near_connected = true;
                // break;
            }
            
        }
    }
}


void PersistentTree::keypointsStuff(){

    keypoints_kdtree.setInputCloud(keypoints_pc);

    // Tree-graph initialization

    geometry_msgs::Point aux = getRobotPosition(fixed_frame, base_frame, listener, transform);

    // tree_graph.push_back(Node(0, 65535, aux));
    ttr::node node;
    node.id = keypoints_tree.size();
    node.position = aux;
    node.parent_id = keypoints_tree.size()-1; //Parent is always the last point
    keypoints_tree.push_back(node);


    pcl::PointXYZ searchPoint;
    searchPoint.x = aux.x;
    searchPoint.y = aux.y;
    searchPoint.z = aux.z;

    std::vector<int> indices;
    std::vector<float> sq_dist;
    // keypoints_kdtree.nearestKSearch(searchPoint, 5, indices, sq_dist);
    keypoints_kdtree.radiusSearch(searchPoint, 2.0, indices, sq_dist);

    for(int i = 0; i < indices.size() ; i++){
        geometry_msgs::Point near;
        near.x = (*keypoints_pc)[indices[i]].x;
        near.y = (*keypoints_pc)[indices[i]].y;
        near.z = (*keypoints_pc)[indices[i]].z;

        if(freeLineMargin(aux, near, obs_kdtree, map_res, margin)){
            keypoints_tree[node.id].neighbours.push_back(indices[i]);
            keypoints_tree[indices[i]].neighbours.push_back(node.id);
            keypoints_line.points.push_back(aux);
            keypoints_line.points.push_back(near);
        }
        
        
    }

    // Keypoints KDTree generation
    keypoints_pc->points.resize(keypoints_pc->points.size() + 1);
    (*keypoints_pc)[keypoints_pc->points.size()-1].x = aux.x;
    (*keypoints_pc)[keypoints_pc->points.size()-1].y = aux.y;
    (*keypoints_pc)[keypoints_pc->points.size()-1].z = aux.z;

    
    
}


bool PersistentTree::insideNeighbours(std::vector<uint32_t> neighbours, uint32_t node_key){
    for(uint32_t i = 0; i < neighbours.size() ; i++){
        if(node_key == neighbours[i]) return true;
    }

    return false;
}

geometry_msgs::Point PersistentTree::getRandomPoint(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z){
    static std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist_x(min_x, max_x);
    std::uniform_real_distribution<> dist_y(min_y, max_y);
    std::uniform_real_distribution<> dist_z(min_z, max_z);

    geometry_msgs::Point p;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.z = dist_z(gen);

    return p;
}


// double PersistentTree::getMapDistance(
//     const Eigen::Vector3d& position) const {
//   if (!voxblox_server_.getEsdfMapPtr()) {
//     ROS_ERROR("NO MAP");
//     return 0.0;
//   }
//   double distance = 0.0;
//   if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position,
//                                                               &distance)) {
//     ROS_WARN("NO DISTANCE");   
//     return 0.0;
//   }
//   return distance;
// }

}  // namespace ttr
