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

#include "validator.h"

template <class T>
struct Point3 {
    T x, y, z;
};

namespace msc {
template <class T>
struct Accessor<T, Point3<T>> {
    inline static const T* data(const Point3<T>& container) {
        return &container.x;
    }
};
}  // namespace msc

namespace ttr {

Validator::Validator(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      voxblox_server_(nh_, nh_private_) {
    configure();
    // reconfigure_server_.setCallback(boost::bind(&Validator::dynReconfig, this, _1, _2));
    // first_loop = false;
}

Validator::~Validator() {
}

// void Validator::dynReconfig(ttr::validatorConfig &config, uint32_t level) {
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

void Validator::configure() {
    float frequency = 10.0;

    nh_private_.param<std::string>("fixed_frame", fixed_frame, "map");
    ROS_INFO("Fixed frame: %s", fixed_frame.c_str());

    nh_private_.param("map_resolution", map_res, 0.35);
    ROS_INFO("Voxel size of the voxblox: %2.2lfm", map_res);

    nh_private_.param("height_limit", max_height, 200.0);
    ROS_INFO("Maximum height to explore: %2.2lfm", max_height);

    nh_private_.param("bandwidth", ms_bandwidth, 10.0);
    ROS_INFO("MeanShift (bw): %2.2lf", ms_bandwidth);

    nh_private_.param("margin", margin, 0.5);
    ROS_INFO("Margin for accept frontier point: %2.2lfm", margin);

    nh_private_.param("gain_radius", rad_gain, 1.0);
    ROS_INFO("Radius to check information gain: %2.2lfm", rad_gain);

    nh_private_.param("min_information_gain", min_info_gain, 0.25);
    ROS_INFO("Minimum information gain of the frontiers: %2.2lf", min_info_gain);

    // nh_private_.param<std::string>("base_frame", base_frame, "base_link");
    // ROS_INFO("Base frame: %s", base_frame.c_str());

    // nh_private_.param("depth", depth_plan, 14);
    // ROS_INFO("Octomap depth: %d", depth_plan);

    // nh_private_.param("resolution", resolution, 0.35);
    // ROS_INFO("Octomap resolution: %2.2lfm", resolution);

    // Raw detected points
    raw_points.header.frame_id = fixed_frame;
    raw_points.header.stamp = ros::Time::now();
    raw_points.ns = "raw_points";
    raw_points.id = 0;
    raw_points.type = visualization_msgs::Marker::POINTS;
    raw_points.action = visualization_msgs::Marker::ADD;
    // raw_points.pose.position.x = 1;
    // raw_points.pose.position.y = 1;
    // raw_points.pose.position.z = 1;
    // raw_points.pose.orientation.x = 0.0;
    // raw_points.pose.orientation.y = 0.0;
    // raw_points.pose.orientation.z = 0.0;
    raw_points.pose.orientation.w = 1.0;
    raw_points.scale.x = 0.1;
    raw_points.scale.y = 0.1;
    raw_points.scale.z = 0.1;
    raw_points.color.a = 0.5;  // Don't forget to set the alpha!
    raw_points.color.r = 255.0 / 255.0;
    raw_points.color.g = 255.0 / 255.0;
    raw_points.color.b = 0.0 / 255.0;
    raw_points.lifetime = ros::Duration(10);

    // Filtered detected points
    filtered.header.frame_id = fixed_frame;
    filtered.header.stamp = ros::Time::now();
    filtered.ns = "filtered_points";
    filtered.id = 0;
    filtered.type = visualization_msgs::Marker::POINTS;
    filtered.action = visualization_msgs::Marker::ADD;
    // filtered.pose.position.x = 1;
    // filtered.pose.position.y = 1;
    // filtered.pose.position.z = 1;
    // filtered.pose.orientation.x = 0.0;
    // filtered.pose.orientation.y = 0.0;
    // filtered.pose.orientation.z = 0.0;
    filtered.pose.orientation.w = 1.0;
    filtered.scale.x = 0.25;
    filtered.scale.y = 0.25;
    filtered.scale.z = 0.25;
    filtered.color.a = 1.0;  // Don't forget to set the alpha!
    filtered.color.r = 0.0 / 255.0;
    filtered.color.g = 255.0 / 255.0;
    filtered.color.b = 0.0 / 255.0;
    filtered.lifetime = ros::Duration(10);

    // // Gain visualization
    // Unknown gain voxels cubes
    unk_gain.header.frame_id = fixed_frame;
    unk_gain.header.stamp = ros::Time::now();
    unk_gain.ns = "unknown";
    unk_gain.id = 0;
    unk_gain.type = visualization_msgs::Marker::CUBE_LIST;
    unk_gain.action = visualization_msgs::Marker::ADD;
    unk_gain.pose.orientation.w = 1.0;
    unk_gain.scale.x = map_res;
    unk_gain.scale.y = map_res;
    unk_gain.scale.z = map_res;
    unk_gain.color.a = 0.4;  // Don't forget to set the alpha!
    unk_gain.color.r = 255.0 / 255.0;
    unk_gain.color.g = 255.0 / 255.0;
    unk_gain.color.b = 255.0 / 255.0;
    unk_gain.lifetime = ros::Duration(1);

    // Free gain voxels cubes
    free_gain.header.frame_id = fixed_frame;
    free_gain.header.stamp = ros::Time::now();
    free_gain.ns = "free";
    free_gain.id = 0;
    free_gain.type = visualization_msgs::Marker::CUBE_LIST;
    free_gain.action = visualization_msgs::Marker::ADD;
    free_gain.pose.orientation.w = 1.0;
    free_gain.scale.x = map_res;
    free_gain.scale.y = map_res;
    free_gain.scale.z = map_res;
    free_gain.color.a = 0.4;  // Don't forget to set the alpha!
    free_gain.color.r = 0.0 / 255.0;
    free_gain.color.g = 255.0 / 255.0;
    free_gain.color.b = 0.0 / 255.0;
    free_gain.lifetime = ros::Duration(1);

    // Occupied gain voxels cubes
    occ_gain.header.frame_id = fixed_frame;
    occ_gain.header.stamp = ros::Time::now();
    occ_gain.ns = "occupied";
    occ_gain.id = 0;
    occ_gain.type = visualization_msgs::Marker::CUBE_LIST;
    occ_gain.action = visualization_msgs::Marker::ADD;
    occ_gain.pose.orientation.w = 1.0;
    occ_gain.scale.x = map_res;
    occ_gain.scale.y = map_res;
    occ_gain.scale.z = map_res;
    occ_gain.color.a = 0.4;  // Don't forget to set the alpha!
    occ_gain.color.r = 255.0 / 255.0;
    occ_gain.color.g = 0.0 / 255.0;
    occ_gain.color.b = 0.0 / 255.0;
    occ_gain.lifetime = ros::Duration(1);

    // Publishers
    raw_points_pub_ = nh_private_.advertise<visualization_msgs::Marker>("raw_points", 10);
    filtered_points_pub_ = nh_private_.advertise<visualization_msgs::Marker>("filtered_points", 10);
    raycast_gain_vis_pub_ = nh_private_.advertise<visualization_msgs::Marker>("gain_vis", 10);
    frontiers_pub_ = nh_.advertise<ttr::frontiers>("/frontiers", 1);

    // Subscribers
    raw_points_sub_ = nh_.subscribe("/detected_points", 100, &Validator::pointsClb, this);
    // trav_pc_sub_ = nh_.subscribe("/voxblox_node/traversable", 1, &Validator::traversablePointCloudClb, this);
    // obs_pc_sub_ = nh_.subscribe("/voxblox_node/surface_pointcloud", 1, &Validator::obstaclePointCloudClb, this);
    exploration_state_sub_ = nh_.subscribe("/exploration_state", 1, &Validator::explorationStateClb, this);

    // Advertising Services
    // makePlan_ = nh_.advertiseService("make_plan", &Validator::makePlan, this);

    // Service clients
    //  request_position_control_client_ = nh_.serviceClient<srv_mav_behaviours::RequestPositionControl>("request_position_control");

    // Wait for receive voxblox pointclouds
    // while(!obs_received || !trav_received){
    //     ros::spinOnce();
    //     ros::Duration(0.1).sleep();
    // }
    // ROS_INFO("Voxblox PCs received.");

    // Waiting for Voxblox
    while(!voxblox_server_.getEsdfMapPtr() || !voxblox_server_.getTsdfMapPtr()){
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("Voxblox received.");

    // Timers
    timer_ = nh_.createTimer(ros::Duration(1.0 / frequency), &Validator::timerClb, this);  // Callback to re-check if path is in free/unknown space

    timing = ros::Time::now().toSec();
}

void Validator::pointsClb(const geometry_msgs::PointStamped& point) {

    // Pre-filter by volumetric gain
    // double time = ros::Time::now().toSec();
    // if(getVolumetricGain(point.point) > 0.1){
    raw_set.insert(point.point);
    // raw_vector.push_back(point.point);
    // }
    // ROS_WARN("Time VOL_GAIN: %1.9lf", ros::Time::now().toSec() - time);
}

// void Validator::traversablePointCloudClb(const sensor_msgs::PointCloud2ConstPtr &t_pc_msg) {
//     // Converting ROS message to PCL
//     pcl::fromROSMsg(*t_pc_msg, *trav_pc);
//     if(trav_pc->points.size() > 10){
//         trav_received = true;
//         trav_kdtree.setInputCloud(trav_pc);
//     }

// }

// void Validator::obstaclePointCloudClb(const sensor_msgs::PointCloud2ConstPtr &obs_pc_msg) {
//     // Converting ROS message to PCL
//     pcl::fromROSMsg(*obs_pc_msg, *obs_pc);
//     if(obs_pc->points.size() > 10){
//         obs_received = true;
//         obs_kdtree.setInputCloud(obs_pc);
//     }
// }

// Callback to filter and publish
void Validator::timerClb(const ros::TimerEvent& event) {
    // Aborting for only few points
    // if(raw_vector.size() < 10)return;

    // ROS_WARN("Time from last timer init: %1.9lf", ros::Time::now().toSec() - timing);
    // timing = ros::Time::now().toSec();

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

    // Getting voxblox ESDF and TSDF map pointer
    std::shared_ptr<voxblox::EsdfMap> esdf_map_ptr = voxblox_server_.getEsdfMapPtr();
    if (esdf_map_ptr == nullptr) {
        ROS_ERROR("NULL ESDF!");
        return;
    }
    std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr = voxblox_server_.getTsdfMapPtr();
    if (tsdf_map_ptr == nullptr) {
        ROS_ERROR("NULL TSDF!");
        return;
    }

    voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer = tsdf_map_ptr->getTsdfLayerPtr();

    std::vector<Point3<double>> filt;
    std::vector<geometry_msgs::Point> temp_vec;

    if(raw_set.size() < 2) return;

    for (const auto& point : raw_set) {
        // Check if is near surface
        Eigen::Vector3d vector(point.x, point.y, point.z);
        double surface_distance = getMapDistanceVB(vector, esdf_map_ptr);
        // ROS_INFO("SURFACE_DISTANCE: %1.3lf", surface_distance);

        if (surface_distance > margin && 
            isFrontierVB(point, tsdf_map_ptr, map_res)) {
                
            temp_vec.push_back(point);

            // Filling the MS container
            filt.emplace_back();
            auto& aux = filt.back();
            aux.x = point.x;
            aux.y = point.y;
            aux.z = point.z;
        }
    }
   
    // If any raw frontier is found cancel the clusterization
    if(temp_vec.size() == 0) return;
    
    raw_vector.clear();
    raw_vector = temp_vec;
    temp_vec.clear();

    // ROS_INFO("RAW_POINTS: %ld", raw_vector.size());

    double cluster_time = ros::Time::now().toSec();

    // Cluster raw_points
    const auto clusters = msc::mean_shift_cluster<double>(
        std::begin(filt), std::end(filt),
        sizeof(Point3<double>) / sizeof(double),
        msc::metrics::L2Sq(),
        msc::kernels::ParabolicSq(),
        // msc::kernels::GaussianSq(),
        msc::estimators::Constant(ms_bandwidth));

    // Fill filtered vector
    geometry_msgs::Point aux_point;
    filtered_vector.clear();
    vol_gain_vector.clear();
    int tot_cnt = 0;
    
    for (const auto& cluster : clusters) {

        int cnt = 0;
        double min_distance = 100.0;
        double max_inf_gain = 0.0;
        int id;
        bool by_vol_gain = true;
        for (const auto& index : cluster.members)
        {
            geometry_msgs::Point aux_3;
            aux_3.x = filt[index].x;
            aux_3.y = filt[index].y;
            aux_3.z = filt[index].z;

            // Representative frontier of the cluster
            if(by_vol_gain){
                // By max information gain
                // std::chrono::time_point<std::chrono::system_clock> start, end;
                // start = std::chrono::system_clock::now();
                double gain = getInformationGain(aux_3, tsdf_map_ptr, map_res, rad_gain, 1.5);
                // end = std::chrono::system_clock::now();
                // std::chrono::duration<double> elapsed_seconds = end - start;
                // ROS_WARN("Sphere gain TIME: %lf s", elapsed_seconds.count());

                if(max_inf_gain < gain){
                    max_inf_gain = gain;
                    id = index;
                }

            } else {
                // By distance
                double dist = euclideanDistance(aux_point, aux_3);
                if(min_distance > dist){
                    min_distance = dist;
                    id = index;
                }
            }
            
            // ROS_INFO("%d/%d: (%2.2lf, %2.2lf, %2.2lf) at %2.3lf",cnt, tot_cnt, filt[index].x, filt[index].y, filt[index].z, euclideanDistance(aux_point, aux_3));
            cnt++;
            tot_cnt++;
        }

        // Select the new representative frontier
        aux_point.x = filt[id].x;
        aux_point.y = filt[id].y;
        aux_point.z = filt[id].z;

        // Information Gain according to selected methodology 
        if(by_vol_gain){
            if(max_inf_gain >= min_info_gain){
                filtered_vector.push_back(aux_point);
                //SPHERE GAIN
                vol_gain_vector.push_back(max_inf_gain); // ORIGINAL

                // RAYCAST
                // std::chrono::time_point<std::chrono::system_clock> start, end;
                // start = std::chrono::system_clock::now();
                // vol_gain_vector.push_back(getInformationGainRayCast(aux_point, tsdf_map_ptr, map_res, 20.0,
                //                                 unk_vec, free_vec, occ_vec)); // Ray Cast
                // end = std::chrono::system_clock::now();
                // std::chrono::duration<double> elapsed_seconds = end - start;
                // ROS_WARN("Ray Cast gain TIME: %lf s", elapsed_seconds.count());

            }
        } else {
            filtered_vector.push_back(aux_point);
            vol_gain_vector.push_back(getInformationGain(aux_point, tsdf_map_ptr, map_res, rad_gain, 1.5));
        }
    }

    // ROS_WARN("Time to cluster %d into %d centroids: %2.6lf", raw_points.points.size(),
    //         filtered.points.size(), ros::Time::now().toSec() - cluster_time);


    // If we get a number of frontier points upper 500 we erase it and only keep the filtered points
    if (raw_vector.size() > 500) {
        raw_vector = filtered_vector;
        raw_set.clear();
        for (const auto& point : filtered_vector) {
 
            raw_set.insert(point);
        }
    }

    // Publishing cleaned raw_points
    raw_points.points.clear();
    raw_points.points = raw_vector;
    raw_points_pub_.publish(raw_points);

    // Publishing filtered_points
    filtered.points.clear();
    filtered.points = filtered_vector;
    filtered_points_pub_.publish(filtered);

    // Publish Frontiers for Decision Maker
    ttr::frontiers frontiers;
    frontiers.frontiers = filtered_vector;
    frontiers.volumetric_gain = vol_gain_vector;
    frontiers_pub_.publish(frontiers);

    // Publish Ray casting gain
    unk_gain.points.clear();
    unk_gain.points = unk_vec;
    raycast_gain_vis_pub_.publish(unk_gain);
    unk_vec.clear();
    free_gain.points.clear();
    free_gain.points = free_vec;
    raycast_gain_vis_pub_.publish(free_gain);
    free_vec.clear();
    occ_gain.points.clear();
    occ_gain.points = occ_vec;
    raycast_gain_vis_pub_.publish(occ_gain);
    occ_vec.clear();
}


void Validator::explorationStateClb(const std_msgs::UInt8::ConstPtr& state_msg) {
    state = static_cast<ExplorationState>(state_msg->data);
}


double Validator::getVolumetricGain(geometry_msgs::Point frontier) {
    double vol_gain_rad = rad_gain;

    std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr = voxblox_server_.getTsdfMapPtr();
    voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer = tsdf_map_ptr->getTsdfLayerPtr();

    voxblox::Point frontier_point(frontier.x, frontier.y, frontier.z);
    voxblox::HierarchicalIndexMap block_voxel_list;
    voxblox::utils::getSphereAroundPoint(*tsdf_layer, frontier_point, vol_gain_rad,
                                         &block_voxel_list);

    uint32_t occupied = 0;
    uint32_t free = 0;
    

    for (const std::pair<voxblox::BlockIndex, voxblox::VoxelIndexList>& kv :
         block_voxel_list) {
        // Get block -- only already existing blocks are in the list.
        const voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
            tsdf_layer->getBlockPtrByIndex(kv.first);

        if (!block_ptr) {
            continue;
        }

        for (const voxblox::VoxelIndex& voxel_index : kv.second) {
            // if (!block_ptr->isValidVoxelIndex(voxel_index)) {
            //     // if (true/*treat_unknown_as_occupied_*/) {
            //     //     // return true;
            //     // }
            //     continue;
            // }
            const voxblox::TsdfVoxel& tsdf_voxel =
                block_ptr->getVoxelByVoxelIndex(voxel_index);
            if (tsdf_voxel.weight > 1e-4 /*voxblox::kEpsilon*/) {
                
                if (tsdf_voxel.distance <= 0.0f) {
                    occupied++;
                } else {
                    free++;
                }
            }
            
        }

    }

    double total_volume = (1.3333333333333)*M_PI*pow(vol_gain_rad, 3);

    double free_volume = (free * pow(map_res, 3));
    double occupied_volume = (occupied * pow(map_res, 3))/* * 1.1*/;

    double vol_gain = total_volume - (free_volume + occupied_volume);

    // // ROS_INFO("VOL GAIN (%2.2lf, %2.2lf, %2.2lf): %3.4lf", frontier.x, frontier.y, frontier.z, vol_gain/total_volume);
    if (vol_gain < 0.0) vol_gain = 0.0;

    return vol_gain / total_volume;

}

}  // namespace ttr