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

#include "local_tree.h"
#include <voxblox/core/common.h>

// #define UINT32_MAX  ((uint32_t)-1)

namespace ttr {

LocalTree::LocalTree(const ros::NodeHandle &nh,
                 const ros::NodeHandle& nh_private)
        : nh_(nh),
          nh_private_(nh_private),
          voxblox_server_(nh_, nh_private_)
{
    configure();
    // reconfigure_server_.setCallback(boost::bind(&LocalTree::dynReconfig, this, _1, _2));
    // first_loop = false;
}

LocalTree::~LocalTree() {
}

// void LocalTree::dynReconfig(ttr::validatorConfig &config, uint32_t level) {
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

void LocalTree::configure() {
    
    double frequency;
    nh_private_.param("frequency", frequency, 20.0);
    ROS_INFO("Frequency of the node: %2.2lfHz", frequency);

    nh_private_.param<std::string>("fixed_frame", fixed_frame, "map");
    ROS_INFO("Fixed frame: %s", fixed_frame.c_str());

    nh_private_.param<std::string>("base_frame", base_frame, "base_link");
    ROS_INFO("Base frame: %s", base_frame.c_str());

    nh_private_.param("max_branch_length", step, 1.0);
    ROS_INFO("Maximum length of the tree branches: %2.2lfm", step);

    nh_private_.param("max_tree_time_growing", grow_time, 0.05);
    ROS_INFO("Maximum time to grow the tree (s): %2.2lfs", grow_time);

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

    nh_private_.param("ground_robot", ground_robot, false);
    ROS_INFO("Ground robot: %s", ground_robot ? "true" : "false");

    if(ground_robot){
        nh_private_.param("projection_height", proj_height, 0.5);
        ROS_INFO("Height at which points are set after projection: %2.2lfm", proj_height);

        nh_private_.param("max_slope", max_slope, 20.0);
        ROS_INFO("Maximum allowed slope (degree): %2.2lf deg", max_slope);
    }



    // Visualization
    line.header.frame_id = "map";
    line.header.stamp = ros::Time::now();
    line.ns = "markers";
    line.id = 0;
    line.type = line.LINE_LIST;
    line.action = line.ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.03;
    line.color.r = 255.0 / 255.0;
    line.color.g = 0.0 / 255.0;
    line.color.b = 0.0 / 255.0;
    line.color.a = 1.0;
    line.lifetime = ros::Duration(1);

    // Debugging points
    r_points.header.frame_id = fixed_frame;
    r_points.header.stamp = ros::Time::now();
    r_points.ns = "random_point";
    r_points.id = 0;
    r_points.type = r_points.SPHERE_LIST;
    r_points.action = r_points.ADD;
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
    n_points.pose.orientation.w = 1.0;
    n_points.scale.x = 0.5;
    n_points.scale.y = 0.5;
    n_points.scale.z = 0.5;
    n_points.color.r = 0.0 / 255.0;
    n_points.color.g = 0.0 / 255.0;
    n_points.color.b = 255.0 / 255.0;
    n_points.color.a = 1.0;
    n_points.lifetime = ros::Duration(10.0);

    v_frontier.header.frame_id = fixed_frame;
    v_frontier.header.stamp = ros::Time::now();
    v_frontier.ns = "frontiers";
    v_frontier.id = 0;
    v_frontier.type = v_frontier.POINTS;
    v_frontier.action = v_frontier.ADD;
    v_frontier.pose.orientation.w = 1.0;
    v_frontier.scale.x = 0.5;
    v_frontier.scale.y = 0.5;
    v_frontier.scale.z = 0.5;
    v_frontier.color.r = 255.0 / 255.0;
    v_frontier.color.g = 0.0 / 255.0;
    v_frontier.color.b = 0.0 / 255.0;
    v_frontier.color.a = 1.0;
    v_frontier.lifetime = ros::Duration(10.0);
  

    // Publishers
    lines_pub_ = nh_.advertise<visualization_msgs::Marker>("/local_tree_graph", 10);
    // frontier_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/detected_points", 10);
    frontier_pub_ = nh_.advertise<visualization_msgs::Marker>("/local_frontiers", 10);
    branch_pub_ = nh_.advertise<ttr::branch>("/local_branch", 10);
    points_pub_ = nh_.advertise<visualization_msgs::Marker>("/points", 10);


    // Subscribers
    // trav_pc_sub_ = nh_.subscribe("/voxblox_node/traversable", 1, &LocalTree::traversablePointCloudClb, this);
    // obs_pc_sub_ = nh_.subscribe("/voxblox_node/surface_pointcloud", 1, &LocalTree::obstaclePointCloudClb, this);
    exploration_state_sub_ = nh_.subscribe("/exploration_state", 1, &LocalTree::explorationStateClb, this);
    
    // Waiting for Voxblox
    while(!voxblox_server_.getEsdfMapPtr() || !voxblox_server_.getTsdfMapPtr()){
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("Voxblox received.");


    // Seed initialization
    srand(time(NULL));

    // Tree initialization
    treeReinitialization();

    // Wait for receive voxblox pointclouds
    // while(!obs_received || !trav_received){
    //     ros::spinOnce();
    //     ros::Duration(0.1).sleep();
    // } 
    // ROS_INFO("Voxblox PCs received.");

    // Timers
    timer_ = nh_.createTimer(ros::Duration(1.0 / frequency), &LocalTree::timerClb, this);  

    init_time = ros::Time::now().toSec();
    timing = ros::Time::now().toSec();

}


// void LocalTree::traversablePointCloudClb(const sensor_msgs::PointCloud2ConstPtr &t_pc_msg) {
//     // double time_init = ros::Time::now().toSec();
//     // Converting ROS message to PCL
//     pcl::fromROSMsg(*t_pc_msg, *trav_pc);
//     // ROS_WARN("Time to load the whole trav_pc: %1.9lf", ros::Time::now().toSec() - time_init);

//     // time_init = ros::Time::now().toSec();
//     if(trav_pc->points.size() > 10){
//         trav_received = true;
//         trav_kdtree.setInputCloud(trav_pc);
//     }
//     // ROS_WARN("Kd-Tree trav_pc: %1.9lf", ros::Time::now().toSec() - time_init);

// }


// void LocalTree::obstaclePointCloudClb(const sensor_msgs::PointCloud2ConstPtr &obs_pc_msg) {
//     // double time_init = ros::Time::now().toSec();
//     // Converting ROS message to PCL
//     pcl::fromROSMsg(*obs_pc_msg, *obs_pc);
//     // ROS_WARN("Time to trans the whole obs_pc: %1.9lf", ros::Time::now().toSec() - time_init);
//     // time_init = ros::Time::now().toSec();

//     if(obs_pc->points.size() > 10){
//         obs_received = true;
//         obs_kdtree.setInputCloud(obs_pc);
//     }
//     // ROS_WARN("Kd-Tree obs_pc: %1.9lf", ros::Time::now().toSec() - time_init);

// }


// Callback to generate the tree-graph
void LocalTree::timerClb(const ros::TimerEvent &event) {

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


    // ROS_WARN("Time from last timer init: %1.9lf", ros::Time::now().toSec() - timing);
    // time_to_again.push_back(ros::Time::now().toSec() - timing);
    // timing = ros::Time::now().toSec();

    // // Print distance travelled every 10 sec
    // if(ros::Time::now().toSec() - last_print > 10){
    //     ROS_WARN("*************************************************");
    //     ROS_WARN("%6.2lf m travelled in %6.2lf", distance, ros::Time::now().toSec() - init_time_print);
    //     ROS_WARN("*************************************************");

    //     double time = 0.0;
    //     for(int i = 0 ; i < time_to_again.size() ; i++){
    //         time += time_to_again[i];
    //     }
    //     ROS_WARN("AVG TIME LAST TIMER: %1.12lf", time/time_to_again.size());

    //     time = 0.0;
    //     for(int i = 0 ; i < time_iter.size() ; i++){
    //         time += time_iter[i];
    //     }

    //     ROS_WARN("AVG TIME ITER: %1.12lf", time/time_iter.size());

    //     time_to_again.clear();
    //     time_iter.clear();

    //     last_print = ros::Time::now().toSec();
    // }


    // Reset tree by time
    // if(total_time > grow_time){
    //     tree_graph.clear();
    //     line.points.clear();
    
    //     total_time = 0.0;
    //     treeReinitialization();

    // }

    // init_time = ros::Time::now().toSec();
    // ROS_ERROR("CURRENT LOCAL TREE TIME: %1.3lf", total_time);

    tree_graph.clear();
    line.points.clear();

    treeReinitialization();

    total_time = 0.000;
    total_tries = 0;

    // ROS_ERROR("NEW LOCAL TREE");

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

    // const voxblox::Layer<voxblox::TsdfVoxel>& tsdf_layer = tsdf_map_ptr->getTsdfLayer();
    // voxblox::BlockIndexList block_list;
    // tsdf_layer.getAllUpdatedBlocks(voxblox::Update::kMap, &block_list);
    // size_t num_blocks = block_list.size();
    // ROS_ERROR("\n\n\nNum of updated blocks: %d\n\n\n", num_blocks);

    while(total_time < grow_time){

        // ROS_ERROR("LOCAL TREE TRY NEW NODE");
        total_tries += 1;
        // ROS_INFO("Total tries: %d", total_tries);
        init_time = ros::Time::now().toSec();
        geometry_msgs::Point q_rand, q_near, q_new;

        float aux_rand = (float)rand()/RAND_MAX;
        float h_range = 0.01;
        if(ground_robot){
            q_rand = getRandomPoint(min_x, max_x, min_y, max_y, current_position.z - h_range, current_position.z + h_range);
            
        } else if(aux_rand < 0.1){
            // Vertical expansion
            q_rand = getRandomPoint(std::max(current_position.x - 1.5, min_x),
                                    std::min(current_position.x + 1.5, max_x),
                                    std::max(current_position.y - 1.5, min_y),
                                    std::min(current_position.y + 1.5, max_y),
                                    std::max(min_z, current_position.z - 15),
                                    std::min(max_z, current_position.z + 15));

        } else if(aux_rand >= 0.1 && 0.25 > aux_rand){
            // Horizontal expansion
            q_rand = getRandomPoint(std::max(current_position.x - 15, min_x),
                                    std::min(current_position.x + 15, max_x),
                                    std::max(current_position.y - 15, min_y),
                                    std::min(current_position.y + 15, max_y),
                                    std::max(min_z, current_position.z - 0.5),
                                    std::min(max_z, current_position.z + 0.5));
        } else {
            // Random expansion
            // ROS_INFO("Normal RANDOM");
            q_rand = getRandomPoint(min_x, max_x, min_y, max_y, min_z, max_z);
        }

        // Debugging points
        r_points.points.push_back(q_rand);

        // Search nearest Node in tree_graph
        pcl::PointXYZ searchPoint;
        searchPoint.x = q_rand.x;
        searchPoint.y = q_rand.y;
        searchPoint.z = q_rand.z;
        kdtree.setInputCloud(tr_gr);

        std::vector<int> indices;
        std::vector<float> sq_dist;
        kdtree.nearestKSearch(searchPoint, 1, indices, sq_dist);

        q_near.x = (*tr_gr)[indices[0]].x;
        q_near.y = (*tr_gr)[indices[0]].y;
        q_near.z = (*tr_gr)[indices[0]].z;

        q_new = steer(q_near, q_rand, step);

        // ************************
        // Afegir Projecció a terra
        // ************************
        // ROS_INFO("QNEW (%2.2lf, %2.2lf, %2.2lf)", q_new.x, q_new.y, q_new.z);
        // if(!newNodeProjection(q_near, &q_new, 0.8, 20, tsdf_map_ptr)) return;
        // ROS_INFO("PROJECTED (%2.2lf, %2.2lf, %2.2lf)\n", q_new.x, q_new.y, q_new.z);


        // char check = freeLineAtDistance(q_near, &q_new, obs_kdtree, trav_kdtree, map_res, margin);
        signed char check = freeLineAtDistanceVB(q_near, &q_new, esdf_map_ptr, tsdf_map_ptr, map_res, margin, ground_robot, max_slope, proj_height);

        // Debugging points
        n_points.points.push_back(q_near);

        points_pub_.publish(r_points);
        points_pub_.publish(n_points);
        r_points.points.clear();
        n_points.points.clear();

        // Frontier Found
        if (check == -1) {        
            
            // Frontier with minimum of volumetric gain
            if(ground_robot){

                // Publish Frontiers
                geometry_msgs::PointStamped frontier;
                frontier.header.stamp = ros::Time::now();
                frontier.header.frame_id = "map";
                frontier.point = q_new;
                
                v_frontier.points.clear();
                v_frontier.points.push_back(q_new);
                frontier_pub_.publish(v_frontier);

                // Send new branch to global graph
                sendBranch(frontier, tree_graph[indices[0]]);
                total_time = 0.0;

                tree_graph.clear();
                line.points.clear();

                treeReinitialization();
                total_tries = 0;
                return;

            } else if(getInformationGain(q_new, tsdf_map_ptr, map_res, rad_gain, 1.5) >= info_gain) {
                // Publish Frontiers
                geometry_msgs::PointStamped frontier;
                frontier.header.stamp = ros::Time::now();
                frontier.header.frame_id = "map";
                frontier.point = q_new;
                
                v_frontier.points.clear();
                v_frontier.points.push_back(q_new);
                frontier_pub_.publish(v_frontier);

                // Send new branch to global graph
                sendBranch(frontier, tree_graph[indices[0]]);
                total_time = 0.0;

                tree_graph.clear();
                line.points.clear();

                treeReinitialization();
                total_tries = 0;
                return;
            }
            
        }

        // New tree node
        else if (check == 0) {
            
            // Node aux_node = Node(tree_graph.size(), indices[0], q_new);
            ttr::node aux_node;
            aux_node.id = tree_graph.size();
            aux_node.parent_id = indices[0]; // TODO algo perque si s'eliminen nodes de l'arbre l'index de PC serà diferent (s'hauria d'asignar key del parent)
            aux_node.position = q_new;
            tree_graph[indices[0]].neighbours.push_back(aux_node.id);

            tree_graph.push_back(aux_node);

            // KD_tree
            int V_last = tree_graph.size() - 1;
            tr_gr->points.resize(tree_graph.size());
            (*tr_gr)[V_last].x = q_new.x;
            (*tr_gr)[V_last].y = q_new.y;
            (*tr_gr)[V_last].z = q_new.z;

            line.points.push_back(q_new);
            line.points.push_back(q_near);

            

            
        } 
        
        total_time += ros::Time::now().toSec() - init_time;
        lines_pub_.publish(line);

        // ROS_WARN("TIMER iteration time: %1.12lf", ros::Time::now().toSec() - timing);
        // time_iter.push_back(ros::Time::now().toSec() - timing);
    }
    
}


void LocalTree::explorationStateClb(const std_msgs::UInt8::ConstPtr& state_msg) {
    state = static_cast<ExplorationState>(state_msg->data);
}


void LocalTree::sendBranch(const geometry_msgs::PointStamped &frontier, const ttr::node &q_near){
    
    ttr::branch aux_branch;
    aux_branch.frontier = frontier;
    aux_branch.nodes.push_back(q_near);
    uint32_t next_id = q_near.parent_id;

    // While next_id does not be the root node
    while(next_id != UINT32_MAX){
        // ROS_INFO("inside while %ld", next_id);
        aux_branch.nodes.insert(aux_branch.nodes.begin(), tree_graph[next_id]);
        next_id = aux_branch.nodes[0].parent_id;
    }

    // Only send branch if it has more than 1 nodes (WHY?????)
    if(aux_branch.nodes.size() > 1){
        branch_pub_.publish(aux_branch);
        // ROS_WARN("Time sent branch %d: %lf", br_cnt, ros::Time::now().toSec());
        // br_cnt++;
    }
    // Send branch and frontier
    // branch_pub_.publish(aux_branch);



}


geometry_msgs::Point LocalTree::getRandomPoint(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z){
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

void LocalTree::treeReinitialization(){

    // int temp = 0;
    // while (temp == 0) {
    //     try {
    //         temp = 1;
    //         listener.lookupTransform(fixed_frame, base_frame, ros::Time(0), transform);
    //     } catch (tf::TransformException ex) {
    //         temp = 0;
    //         ros::Duration(0.1).sleep();
    //     }
    // }

    // current_position.x = transform.getOrigin().x();
    // current_position.y = transform.getOrigin().y();
    // current_position.z = transform.getOrigin().z();
    current_position = getRobotPosition(fixed_frame, base_frame, listener, transform);

    // Clear sphereAroundRobot
    // voxblox::Point center(current_position.x, current_position.y, current_position.z);
    std::shared_ptr<voxblox::EsdfMap> esdf_map_ptr = voxblox_server_.getEsdfMapPtr();
    // voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer = esdf_map_ptr->getEsdfLayerPtr();

    // voxblox::utils::fillSphereAroundPoint(center, margin, 0.0, esdf_layer);

    if(first_loop){
        last_position.x = current_position.x;
        last_position.y = current_position.y;
        last_position.z = current_position.z;
        first_loop = false;
        init_time_print = ros::Time::now().toSec();
    }

    distance += euclideanDistance(last_position, current_position);

    last_position.x = current_position.x;
    last_position.y = current_position.y;
    last_position.z = current_position.z;

    // To allow local tree growing in narrow spaces
    // std::shared_ptr<voxblox::EsdfMap> esdf_map_ptr = voxblox_server_.getEsdfMapPtr();
    Eigen::Vector3d aux_pos(current_position.x, current_position.y, current_position.z);
    bool bad_root = false;
    uint8_t ctr = 0;
    while(getMapDistanceVB(aux_pos, esdf_map_ptr) < margin && ctr < 100){

        geometry_msgs::Point aux_rand = getRandomPoint(current_position.x - margin, current_position.x + margin, current_position.y - margin, current_position.y + margin, std::max(min_z, current_position.z - margin), std::min(max_z, current_position.z + margin));
        aux_pos.x() = aux_rand.x;
        aux_pos.y() = aux_rand.y;
        aux_pos.z() = aux_rand.z;
        bad_root = true;
        ctr++;
    }
    if(bad_root){

        current_position.x = aux_pos.x();
        current_position.y = aux_pos.y();
        current_position.z = aux_pos.z();
        // ROS_WARN("CAN'T SET LOCAL ROOT");
    }


    // if(ground_robot) current_position.z += 0.25;

    // Insert current position to the new tree
    // tree_graph.push_back(Node(0, 65535, q_new));
    ttr::node root_node;
    root_node.id = 0;
    root_node.parent_id = UINT32_MAX;
    root_node.position = current_position;
    tree_graph.push_back(root_node);

    // KD_tree re-initialization
    tr_gr->points.resize(1);
    (*tr_gr)[0].x = current_position.x;
    (*tr_gr)[0].y = current_position.y;
    (*tr_gr)[0].z = current_position.z;
}

}  // namespace ttr
