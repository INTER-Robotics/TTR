#include "common_utils.h"


// euclideanDistance function
double euclideanDistance(geometry_msgs::Point point1, geometry_msgs::Point point2) {

    double distance = sqrt(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2) + pow(point2.z - point1.z, 2));


    return distance;

}

// Obtain the nearest neighbour
geometry_msgs::Point nearestNeighbour(std::vector<geometry_msgs::Point> V, geometry_msgs::Point point) {

    float min = euclideanDistance(V[0], point);
    int min_index;
    float temp;

    for (int j = 0; j < V.size(); j++) {
        temp = euclideanDistance(V[j], point);
        if (temp <= min) {
            min = temp;
            min_index = j;
        }
    }

    return V[min_index];
}

// Steer function
geometry_msgs::Point steer(geometry_msgs::Point nearest, geometry_msgs::Point rand, float eta) {

    geometry_msgs::Point new_point;
    float distAB = euclideanDistance(nearest, rand);

    if (distAB <= eta) {
        new_point = rand;

    } else {

        new_point.x = nearest.x + (eta / distAB) * (rand.x - nearest.x);
        new_point.y = nearest.y + (eta / distAB) * (rand.y - nearest.y);
        new_point.z = nearest.z + (eta / distAB) * (rand.z - nearest.z);
    }

    return new_point;
}


char freeLine(geometry_msgs::Point p_nearest, geometry_msgs::Point* p_new, octomap::OcTree* octree){

    float resolution = octree->getResolution() * 0.1; // divided by 0.1 cause have to check more than resolution
    int num_steps = ceil(euclideanDistance(p_nearest, *p_new) / resolution);

    geometry_msgs::Point stepper = p_nearest;

    for(int step = 0 ; step < num_steps ; step++){

        stepper = steer(stepper, *p_new, resolution);

        // if(obstacle(stepper, octree)){
        //     return false;
        // }

        switch(obstacle(stepper, octree)) {
            case -1:
                *p_new = stepper;
                return -1; // unknown (frontier)
                break;
            case 1:
                return 1; // obstacle
                break;
            default:
               // Free
                break;
        }

    }

    return 0; // free line
}

char freeLineAtDistance(geometry_msgs::Point p_nearest, geometry_msgs::Point* p_new, octomap::OcTree* octree,  pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree){

    float resolution = octree->getResolution() * 0.1; // divided by 0.1 cause have to check more than resolution
    int num_steps = ceil(euclideanDistance(p_nearest, *p_new) / resolution);

    geometry_msgs::Point stepper = p_nearest;

    for(int step = 0 ; step < num_steps ; step++){

        stepper = steer(stepper, *p_new, resolution);

        if(obstacleAtDistance(stepper, 0.5, kdtree)){
            return 1;
        } else {

            if (obstacle(stepper, octree)) {
                *p_new = stepper;
                return -1;
            }

        }


        // switch(obstacle(stepper, octree) * obstacleAtDistance(stepper, 0.5, octree, kdtree)) {
        //     case -1:
        //         *p_new = stepper;
        //         return -1; // unknown (frontier)
        //         break;
        //     case 1:
        //         return 1; // obstacle
        //         break;
        //     default:
        //        // Free
        //         break;
        // }

    }

    return 0; // free line
}

char freeLineAtDistance(geometry_msgs::Point p_nearest, geometry_msgs::Point* p_new,  pcl::KdTreeFLANN<pcl::PointXYZ> &obs_kdtree_, pcl::KdTreeFLANN<pcl::PointXYZ> &trav_kdtree_, double res, double margin_){

    float resolution = res * 0.1; // multiplied by 0.1 cause have to check more than resolution
    int num_steps = ceil(euclideanDistance(p_nearest, *p_new) / resolution);

    geometry_msgs::Point stepper = p_nearest;

    for(int step = 0 ; step < num_steps ; step++){

        stepper = steer(stepper, *p_new, resolution);

        if(obstacleAtDistance(stepper, margin_, obs_kdtree_)){
            // Abort this branch
            return 1;
        } else {

            if (isFrontier(stepper, trav_kdtree_, res)) {
                // p_new is the frontier found
                *p_new = stepper;
                return -1;
            }

        }

    }

    // New node to tree
    return 0; // free line
}


bool freeLineMargin(geometry_msgs::Point a, geometry_msgs::Point b,  pcl::KdTreeFLANN<pcl::PointXYZ> &obs_kdtree_, double res, double margin_){

    float resolution = res * 0.1; // multiplied by 0.1 cause have to check more than resolution
    int num_steps = ceil(euclideanDistance(a, b) / resolution);

    geometry_msgs::Point stepper = a;

    for(int step = 0 ; step < num_steps ; step++){

        stepper = steer(stepper, b, resolution);

        if(obstacleAtDistance(stepper, margin_, obs_kdtree_)){
            return false;
        }

    }

    return true;
}

bool isFrontier(geometry_msgs::Point point, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, pcl::KdTreeFLANN<pcl::PointXYZ> &obs_kdtree, double resolution, double margin){

    std::vector<int> indices;
    std::vector<float> sq_dist;
    pcl::PointXYZ searchPoint;
    searchPoint.x = point.x;
    searchPoint.y = point.y;
    searchPoint.z = point.z;

    obs_kdtree.radiusSearch(searchPoint, margin+resolution, indices, sq_dist, 1);
    if(indices.size() > 0){
        return false;
    }

    indices.clear();
    sq_dist.clear();

    kdtree.radiusSearch(searchPoint, resolution*1.33, indices, sq_dist, 8);
    if(sq_dist.size() < 6){
        return true;
    }

    return false;

}


bool isFrontier(geometry_msgs::Point point, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, double resolution){

    std::vector<int> indices;
    std::vector<float> sq_dist;
    pcl::PointXYZ searchPoint;
    searchPoint.x = point.x;
    searchPoint.y = point.y;
    searchPoint.z = point.z;

    kdtree.radiusSearch(searchPoint, resolution*1.33, indices, sq_dist, 8);
    // kdtree.radiusSearch(searchPoint, resolution*1.5, indices, sq_dist, 8);


    if(sq_dist.size() < 6){
        return true;
    }

    return false;

}


char obstacle(geometry_msgs::Point point, octomap::OcTree* octree){

    octomap::OcTreeNode* node = octree->search(point.x, point.y, point.z);
    if (node){
        // if(node->getValue() > 0.0){
        //     return 1; // Obstacle
        // } else {
        //     return 0; // Free
        // }
        if(octree->isNodeOccupied(*node)){
            return 1; // Obstacle
        } else {
            return 0; // Free
        }

    } else {
        // Unknown space
        return -1;
    }


    // if(octree){

    //     int i = 1;

        // for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
        //                         end = octree->end_leafs();
        //     it != end; ++it) {  // access node, e.g.:
        //     std::cout << "Node center(" << i <<"): " << it.getCoordinate();
        //     std::cout << " Node size: " << it.getSize();
        //     std::cout << " value: " << it->getValue();

        //     if(octree->isNodeOccupied(*it)){
        //         std::cout << " Occupied \n";
        //         //return 1;
        //     } else {
        //         std::cout << " Free \n";
        //         //return 0;
        //     }


        //     i++;
        // }
        // std::cout << "NUM LEAFS: " << i << "\n";

        // octomap::OcTreeNode* node = octree->search(0.0, 15.0, 0.0);
        // if (node){
        //     // std::cout << "Value: " << node->getValue() << "\n";
        //     printf("Value: %f \n", node->getValue());
        //     return node->getValue();
        // } else {
        //     // std::cout << "Unknown \n";
        //     printf("Unknown\n");
        //     return -1;
        // }

    //     return i;
    // }



}

bool obstacleAtDistance(geometry_msgs::Point point, double distance, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree){
    if(distance < 0.00001)return false;

    std::vector<int> indices;
    std::vector<float> sq_dist;
    pcl::PointXYZ searchPoint;
    searchPoint.x = point.x;
    searchPoint.y = point.y;
    searchPoint.z = point.z;

    kdtree.radiusSearch(searchPoint, distance, indices, sq_dist, 1);

    if(indices.size() != 0){
        return true;
    }

    return false;
}


double distanceObstacle(geometry_msgs::Point point, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree){

    std::vector<int> indices;
    std::vector<float> sq_dist;
    pcl::PointXYZ searchPoint;
    searchPoint.x = point.x;
    searchPoint.y = point.y;
    searchPoint.z = point.z;

    kdtree.radiusSearch(searchPoint, 10.0, indices, sq_dist, 1);
    // kdtree.nearestKSearch(searchPoint, 1, indices, sq_dist);

    if(indices.size() != 0){
        return sq_dist[0];
    }

    return 0.0;
}


double getMapDistanceVB(const Eigen::Vector3d& position, std::shared_ptr<voxblox::EsdfMap> mapPtr){
    if (!mapPtr) {
        // ROS_ERROR("NO MAP");
        return 0.0;
    }
    double distance = 0.0;
    if (!mapPtr->getDistanceAtPosition(position, &distance)) {
        // ROS_WARN("NO DISTANCE");
        return 0.0;
    }
    return distance;
}


bool freeLineMarginVB(geometry_msgs::Point a, geometry_msgs::Point b,
                        std::shared_ptr<voxblox::EsdfMap> mapPtr,
                        std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr,
                        double res, double margin_, bool ground_robot,
                        double max_slope){

    // Extra-check by raycasting
    // if(!freeLineVB(a, b, tsdf_map_ptr, res)){
    //     return false;
    // } 

    float resolution = res * 0.1; // multiplied by 0.1 cause have to check more than resolution
    int num_steps = ceil(euclideanDistance(a, b) / resolution);

    geometry_msgs::Point stepper = a;

    for(int step = 0 ; step < num_steps ; step++){

        stepper = steer(stepper, b, resolution);

        Eigen::Vector3d vector(stepper.x, stepper.y, stepper.z);

        if(getMapDistanceVB(vector, mapPtr) < margin_ || (ground_robot && abs(slopeDegrees(a,b)) > max_slope)){
            return false;
        }

    }

    return true;
}

bool freeLineVB(geometry_msgs::Point a, geometry_msgs::Point b,
                std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr, double res){

    const voxblox::Layer<voxblox::TsdfVoxel>& tsdf_layer = tsdf_map_ptr->getTsdfLayer();
    //  Millor si ho pas per paràmetre
    float voxel_size_ = tsdf_layer.voxel_size();
    float voxel_size_inv_ = 1/voxel_size_;
    int voxels_per_side_ = tsdf_layer.voxels_per_side();
    float voxels_per_side_inv_ = 1/voxels_per_side_;

    voxblox::Point origin(a.x, a.y, a.z);
    const voxblox::Point start_scaled = origin.cast<voxblox::FloatingPoint>() * voxel_size_inv_;

    voxblox::Point end(b.x, b.y, b.z);
    const voxblox::Point end_scaled = end.cast<voxblox::FloatingPoint>() * voxel_size_inv_;

    // Cast Ray of a Line
    voxblox::AlignedVector<voxblox::GlobalIndex> global_voxel_indices;
    voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);

    for (int i = 0; i < global_voxel_indices.size(); i++) {
        const voxblox::GlobalIndex& global_voxel_idx = global_voxel_indices[i];

        // Getting the voxel by global index
        const voxblox::TsdfVoxel *voxel = tsdf_layer.getVoxelPtrByGlobalIndex(global_voxel_idx);
        if (voxel != nullptr) {

            if (voxel->weight <= 1e-4 || (voxel->distance <= 0.1 && voxel->distance >= -0.1)) {
                // Unknown voxel
                return false;
            
            } 
        } else {
            // Treated as occupied voxel
            return false;
        }

    }

    // Free line
    return true;
}


// char freeLineAtDistanceVB(geometry_msgs::Point p_nearest, geometry_msgs::Point* p_new, std::shared_ptr<voxblox::EsdfMap> esdf_map_ptr, std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr, double res, double margin_){

//     const voxblox::Layer<voxblox::TsdfVoxel>& tsdf_layer = tsdf_map_ptr->getTsdfLayer();

//     float resolution = res * 0.1; // multiplied by 0.1 cause have to check more than resolution
//     int num_steps = ceil(euclideanDistance(p_nearest, *p_new) / resolution);

//     geometry_msgs::Point stepper = p_nearest;

//     for(int step = 0 ; step < num_steps ; step++){

//         stepper = steer(stepper, *p_new, resolution);

//         Eigen::Vector3d vector(stepper.x, stepper.y, stepper.z);

//         if(getMapDistanceVB(vector, esdf_map_ptr) < margin_){
//             // Obstacle
//             return 1;
//         } else {
//             voxblox::Point point(stepper.x, stepper.y, stepper.z);


//             // Obtiene el voxel al que pertenece el punto.
//             const voxblox::TsdfVoxel* voxel = tsdf_layer.getVoxelPtrByCoordinates(point);

//             if (voxel != nullptr) {

//                 // Getting the voxel's world coordinates
//                 // voxblox::Point vox_coord = voxblox::getCenterPointFromGridIndex(global_voxel_idx, voxel_size_);
//                 // Eigen::Vector3d vector(vox_coord.x(), vox_coord.y(), vox_coord.z());

//                 // ROS_WARN("Current voxel: %3.3lf, %3.3lf, %3.3lf", vox_coord.x(), vox_coord.y(), vox_coord.z());

//                 if (voxel->weight <= 1e-4/* && getMapDistanceVB(vector, esdf_map_ptr) > margin_*/) {
//                     // Frontier found
//                     // ROS_INFO("Unknown voxel");
//                     *p_new = stepper;
//                     // std::cout << "Frontier found at:" << vox_coord.transpose() << std::endl;
//                     // ROS_WARN("Frontier found at: %3.3lf, %3.3lf, %3.3lf", p_new->x, p_new->y, p_new->z);
//                     return -1;
//                     // std::cout << vox_coord.transpose() << std::endl;
//                 } else if (voxel->distance >= 0.0 /* && getMapDistanceVB(vector, esdf_map_ptr) > margin_ */) {
//                     // ROS_INFO("Free voxel");
//                     // std::cout << vox_coord.transpose() << std::endl;
//                 } else {
//                     // Blocked line
//                     // ROS_INFO("Occupied voxel");
//                     // std::cout << vox_coord.transpose() << std::endl;
//                     return 1;

//                 }
//             } else {
//                 // Treated as blocked line
//                 // ROS_INFO("Unknown voxel BLOCK NO EXIST");
//                 return 1;
//             }
//         }

//     }

//     // Free Line
//     return 0;

// }


// // Using Ray Casting
// char freeLineAtDistanceVB(geometry_msgs::Point p_nearest, geometry_msgs::Point* p_new, std::shared_ptr<voxblox::EsdfMap> esdf_map_ptr, std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr, double res, double margin_){

//     const voxblox::Layer<voxblox::TsdfVoxel>& tsdf_layer = tsdf_map_ptr->getTsdfLayer();

//     // Millor si ho pas per paràmetre
//     float voxel_size_ = tsdf_layer.voxel_size();
//     float voxel_size_inv_ = 1/voxel_size_;
//     int voxels_per_side_ = tsdf_layer.voxels_per_side();
//     float voxels_per_side_inv_ = 1/voxels_per_side_;

//     voxblox::Point origin(p_nearest.x, p_nearest.y, p_nearest.z);
//     const voxblox::Point start_scaled = origin.cast<voxblox::FloatingPoint>() * voxel_size_inv_;

//     // std::cout << "Origin at point (" << origin.transpose();
//     // std::cout << "VBOrigin at point (" << start_scaled.transpose() << std::endl;

//     voxblox::Point end(p_new->x, p_new->y, p_new->z);
//     const voxblox::Point end_scaled = end.cast<voxblox::FloatingPoint>() * voxel_size_inv_;

//     // std::cout << ", End at point (" << end.transpose() << std::endl;
//     // std::cout << "VBEnd at point (" << end_scaled.transpose() << std::endl;



//     // Cast Ray of a Line
//     voxblox::AlignedVector<voxblox::GlobalIndex> global_voxel_indices;
//     voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);
//     // std::cout << "num voxels in ray_cast:" << global_voxel_indices.size() << std::endl;
//     for (int i = 0; i < global_voxel_indices.size(); i++) {
//         const voxblox::GlobalIndex& global_voxel_idx = global_voxel_indices[i];

//         // Getting the voxel by global index
//         const voxblox::TsdfVoxel *voxel = tsdf_layer.getVoxelPtrByGlobalIndex(global_voxel_idx);
//         if (voxel != nullptr) {

//             // Getting the voxel's world coordinates
//             voxblox::Point vox_coord = voxblox::getCenterPointFromGridIndex(global_voxel_idx, voxel_size_);
//             Eigen::Vector3d vector(vox_coord.x(), vox_coord.y(), vox_coord.z());

//             // ROS_WARN("Current voxel: %3.3lf, %3.3lf, %3.3lf", vox_coord.x(), vox_coord.y(), vox_coord.z());

//             if (voxel->weight <= 1e-4/* && getMapDistanceVB(vector, esdf_map_ptr) > margin_*/) {
//                 // Frontier found
//                 // ROS_INFO("Unknown voxel");
//                 p_new->x = vox_coord.x();
//                 p_new->y = vox_coord.y();
//                 p_new->z = vox_coord.z();
//                 // std::cout << "Frontier found at:" << vox_coord.transpose() << std::endl;
//                 // ROS_WARN("Frontier found at: %3.3lf, %3.3lf, %3.3lf", p_new->x, p_new->y, p_new->z);
//                 return -1;
//                 // std::cout << vox_coord.transpose() << std::endl;
//             } else if (voxel->distance >= 0.0 && getMapDistanceVB(vector, esdf_map_ptr) > margin_) {
//                 // ROS_INFO("Free voxel");
//                 // std::cout << vox_coord.transpose() << std::endl;
//             } else {
//                 // Blocked line
//                 // ROS_INFO("Occupied voxel");
//                 // std::cout << vox_coord.transpose() << std::endl;
//                 return 1;

//             }
//         } else {
//             // Treated as blocked line
//             // ROS_INFO("Unknown voxel BLOCK NO EXIST");
//             return 1;
//         }

//     }

//     // Free line to add to the tree
//     return 0;

// }


// bool isFrontierVB(geometry_msgs::Point point,
//                     std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr, double resolution){

//     const voxblox::Layer<voxblox::TsdfVoxel>& tsdf_layer = tsdf_map_ptr->getTsdfLayer();
//     voxblox::Point voxel_point(point.x, point.y, point.z);

//     // Obtiene el voxel al que pertenece el punto.
//     const voxblox::TsdfVoxel* voxel_ptr = tsdf_layer.getVoxelPtrByCoordinates(voxel_point);

//     if (voxel_ptr) {
//         // El voxel existe.

//         double weight = voxel_ptr->weight;

//         if (weight > 1e-4) {
//             if(voxel_ptr->distance >= 0.0){
//                 // std::cout << "Voxel at point (" << point.transpose() << ") is free." << std::endl;
//             } else {
//                 // std::cout << "Voxel at point (" << point.transpose() << ") is occupied." << std::endl;
//             }
//             // ROS_INFO("Is not frontier anymore");
//             return false;
//         } else {
//             // std::cout << "Voxel at point (" << point.transpose() << ") does not exist." << std::endl;
//             // Is unknown voxel
//             // ROS_INFO("Is still frontier");
//             return true;
//         }
//     } else {
//         // The voxel does not exist.
//         // TRUE or FALSE???
//         // ROS_INFO("Is still frontier (NO EXIST)");
//         return true;
//     }
// }

// *********************************************************************************************************************
// *********************************************************************************************************************
// *********************************************************************************************************************
// Being frontiers in free space
// *********************************************************************************************************************
// *********************************************************************************************************************
// *********************************************************************************************************************

// // Voxel Space
// char freeLineAtDistanceVB(geometry_msgs::Point p_nearest, geometry_msgs::Point* p_new,
//                             std::shared_ptr<voxblox::EsdfMap> esdf_map_ptr,
//                             std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr,
//                             double res, double margin_){

//     const voxblox::Layer<voxblox::TsdfVoxel>& tsdf_layer = tsdf_map_ptr->getTsdfLayer();

//     // Millor si ho pas per paràmetre
//     float voxel_size_ = tsdf_layer.voxel_size();
//     float voxel_size_inv_ = 1/voxel_size_;
//     int voxels_per_side_ = tsdf_layer.voxels_per_side();
//     float voxels_per_side_inv_ = 1/voxels_per_side_;

//     voxblox::Point origin(p_nearest.x, p_nearest.y, p_nearest.z);
//     const voxblox::Point start_scaled = origin.cast<voxblox::FloatingPoint>() * voxel_size_inv_;

//     voxblox::Point end(p_new->x, p_new->y, p_new->z);
//     const voxblox::Point end_scaled = end.cast<voxblox::FloatingPoint>() * voxel_size_inv_;


//     // Cast Ray of a Line
//     voxblox::AlignedVector<voxblox::GlobalIndex> global_voxel_indices;
//     voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);
//     // std::cout << "num voxels in ray_cast:" << global_voxel_indices.size() << std::endl;

//     voxblox::Point last_vox_coord;

//     for (int i = 0; i < global_voxel_indices.size(); i++) {
//         const voxblox::GlobalIndex& global_voxel_idx = global_voxel_indices[i];

//         // Getting the voxel by global index
//         const voxblox::TsdfVoxel *voxel = tsdf_layer.getVoxelPtrByGlobalIndex(global_voxel_idx);
//         if (voxel != nullptr) {

//             // Getting the voxel's world coordinates
//             voxblox::Point vox_coord = voxblox::getCenterPointFromGridIndex(global_voxel_idx, voxel_size_);
//             Eigen::Vector3d vector(vox_coord.x(), vox_coord.y(), vox_coord.z());

//             // ROS_WARN("Current voxel: %3.3lf, %3.3lf, %3.3lf", vox_coord.x(), vox_coord.y(), vox_coord.z());

//             if (voxel->weight <= 1e-4/* && getMapDistanceVB(vector, esdf_map_ptr) > margin_*/) {
//                 // Considered as blocked (no free with unknown neighbours found)
//                 return 1;
//                 // std::cout << vox_coord.transpose() << std::endl;
//             } else if (voxel->distance >= 0.0 && getMapDistanceVB(vector, esdf_map_ptr) > margin_) {
//                 // ROS_INFO("Free voxel");
//                 // std::cout << vox_coord.transpose() << std::endl;
//                 // last_vox_coord.x() = vox_coord.x();
//                 // last_vox_coord.y() = vox_coord.y();
//                 // last_vox_coord.z() = vox_coord.z();
//                 geometry_msgs::Point aux;
//                 aux.x = vox_coord.x();
//                 aux.y = vox_coord.y();
//                 aux.z = vox_coord.z();

//                 if(isFrontierVB(aux, tsdf_map_ptr, res)){
//                     p_new->x = vox_coord.x();
//                     p_new->y = vox_coord.y();
//                     p_new->z = vox_coord.z();
//                     return -1;
//                 }

//             } else {
//                 // Blocked line
//                 // ROS_INFO("Occupied voxel");
//                 // std::cout << vox_coord.transpose() << std::endl;
//                 return 1;

//             }
//         } else {
//             // Treated as blocked line
//             // ROS_INFO("Unknown voxel BLOCK NO EXIST");
//             return 1;
//         }

//     }

//     // Free line to add to the tree
//     return 0;

// }

// Real Space
// Returns -> 0: free line, 1: not free line, -1: frontier found
signed char freeLineAtDistanceVB(geometry_msgs::Point p_nearest, geometry_msgs::Point* p_new,
                             std::shared_ptr<voxblox::EsdfMap> esdf_map_ptr,
                             std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr,
                             double res, double margin_, bool ground_robot,
                             double max_slope, double proj_height){

    const voxblox::Layer<voxblox::TsdfVoxel>& tsdf_layer = tsdf_map_ptr->getTsdfLayer();

    float resolution = res * 0.5; // multiplied by (DEF:0.1) cause have to check more than resolution
    int num_steps = ceil(euclideanDistance(p_nearest, *p_new) / resolution);

    geometry_msgs::Point stepper = p_nearest;
    double last_z = stepper.z;
    uint8_t cnt_no_surface = 0;

    for(int step = 0 ; step < num_steps ; step++){

        stepper = steer(stepper, *p_new, resolution);

        // TO ERASE
        if(ground_robot){
            if(!pointProjection(&stepper, proj_height, tsdf_map_ptr)){
                if(cnt_no_surface > 15) return 1; // To avoid extension over consecutive no surface
                stepper.z = last_z;
                cnt_no_surface++;
            } else {
                cnt_no_surface = 0;
            }
        }
        // TO ERASE

        Eigen::Vector3d vector(stepper.x, stepper.y, stepper.z);

        double dist_to_surface = getMapDistanceVB(vector, esdf_map_ptr);

        if(dist_to_surface < margin_){
            // Obstacle
            return 1;
        } else {
            voxblox::Point point(stepper.x, stepper.y, stepper.z);


            // Obtiene el voxel al que pertenece el punto.
            const voxblox::TsdfVoxel* voxel = tsdf_layer.getVoxelPtrByCoordinates(point);

            if (voxel != nullptr) {

                // Getting the voxel's world coordinates
                // voxblox::Point vox_coord = voxblox::getCenterPointFromGridIndex(global_voxel_idx, voxel_size_);
                // Eigen::Vector3d vector(vox_coord.x(), vox_coord.y(), vox_coord.z());

                // ROS_WARN("Current voxel: %3.3lf, %3.3lf, %3.3lf", vox_coord.x(), vox_coord.y(), vox_coord.z());

                if (voxel->weight <= 1e-4/* && getMapDistanceVB(vector, esdf_map_ptr) > margin_*/) {
                    // Considered as blocked (no free with unknown neighbours found)
                    return 1;
                } else if (voxel->distance >= 0.0 /*&& getMapDistanceVB(vector, esdf_map_ptr) > margin_*/) {
                    
                    if(ground_robot){
                        if(isFrontierVB(stepper, tsdf_map_ptr, res) && abs(slopeDegrees(p_nearest, stepper)) < max_slope){
                            *p_new = stepper; 

                            return -1;
                        }

                    } else {
                        // Is frontier & is near a surface (permiting exploration of surfaces in open scenarios)
                        // & is away from surfaces (avoiding the aproximations)
                        if(isFrontierVB(stepper, tsdf_map_ptr, res)/* && dist_to_surface < 1.0*//*&& dist_to_surface > 0.5*/){
                            *p_new = stepper; 

                            return -1;
                        }
                    }
                    

                } else {
                    // Blocked line
                    return 1;

                }
            } else {
                // Treated as blocked line
                return 1;
            }
        }
        last_z = stepper.z;
    }

    // Free Line
    // TO ERASE
    if(ground_robot){
        if(abs(slopeDegrees(p_nearest, stepper)) < max_slope){
            *p_new = stepper; // Amb lo de projectar
            return 0;
        }
        // Too much slope
        return 1;
    }
    // TO ERASE
    return 0;

}


bool isFrontierVB(geometry_msgs::Point point,
                    std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr, double resolution){

    // const voxblox::Layer<voxblox::TsdfVoxel>& tsdf_layer = tsdf_map_ptr->getTsdfLayer();
    voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer = tsdf_map_ptr->getTsdfLayerPtr();
    voxblox::Point voxel_point(point.x, point.y, point.z);

    // Obtiene el voxel al que pertenece el punto.
    const voxblox::TsdfVoxel* voxel_ptr = tsdf_layer->getVoxelPtrByCoordinates(voxel_point);
    // Getting the voxel's world coordinates
    // const voxblox::Point voxel_coord = voxel_point.cast<voxblox::FloatingPoint>() * (1/resolution);
    
    // Check voxel exists
    if (voxel_ptr) {

        double weight = voxel_ptr->weight;
        
        if (weight > 1e-4 && voxel_ptr->distance >= 0.0) {
            // Is in free space, can be a frontier.
        } else {
            return false;
        }
    } else {
        // The voxel does not exist, is not a frontier.
        return false;
    }

    // Now check if a neighbour is unknown
    voxblox::HierarchicalIndexMap block_voxel_list;
    // voxblox::utils::getSphereAroundPoint(*tsdf_layer, voxel_point, resolution*1.5,
    //                                      &block_voxel_list);
    voxblox::utils::getSphereAroundPoint(*tsdf_layer, voxel_point, resolution*1.0,
                                         &block_voxel_list);

    // int neighs = 0;
    // bool frontier = false;

    for (const std::pair<voxblox::BlockIndex, voxblox::VoxelIndexList>& kv :
         block_voxel_list) {
        
        // Get block -- only already existing blocks are in the list.
        const voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
            tsdf_layer->getBlockPtrByIndex(kv.first);

        if (!block_ptr) {
            return true;
            // continue;
        }

        for (const voxblox::VoxelIndex& voxel_index : kv.second) {
            if (!block_ptr->isValidVoxelIndex(voxel_index)) {
                // if (true/*treat_unknown_as_occupied_*/) {
                //     // return true;
                // }
                // Unknown voxel (untracked)
                return true;
                // continue;
            }



            const voxblox::TsdfVoxel& tsdf_voxel =
                block_ptr->getVoxelByVoxelIndex(voxel_index);
            // if (tsdf_voxel.weight > 1e-4 /*voxblox::kEpsilon*/) {
                
            //     // if (tsdf_voxel.distance <= 0.0f) {
            //     //     // occupied++;
            //     // } else {
            //     //     // free++;
            //     // }
            // } else {
            //     frontier = true;
            // }
            if (tsdf_voxel.weight <= 1e-4 /*voxblox::kEpsilon*/) return true;
            
        }

    }

    // ROS_INFO("NEIGHBOURS: %d front: %d", neighs, frontier);
    return false;

}

double getInformationGain(geometry_msgs::Point frontier,
                        std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr,
                        double map_res, double gain_rad, double occ_penal) {

    double vol_gain_rad = gain_rad;

    // std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr = voxblox_server_.getTsdfMapPtr();
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
    double occupied_volume = (occupied * pow(map_res, 3)) * occ_penal;

    double vol_gain = total_volume - (free_volume + occupied_volume);

    // // ROS_INFO("VOL GAIN (%2.2lf, %2.2lf, %2.2lf): %3.4lf", frontier.x, frontier.y, frontier.z, vol_gain/total_volume);
    if (vol_gain < 0.0) vol_gain = 0.0;

    return vol_gain / total_volume;

}

double getInformationGainRayCast(geometry_msgs::Point frontier,
                        std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr,
                        double map_res, double gain_rad, std::vector<geometry_msgs::Point>& unk_vec,
                        std::vector<geometry_msgs::Point>& free_vec, std::vector<geometry_msgs::Point>& occ_vec){


    const voxblox::Layer<voxblox::TsdfVoxel>& tsdf_layer = tsdf_map_ptr->getTsdfLayer();
    //  Millor si ho pas per paràmetre
    float voxel_size_ = tsdf_layer.voxel_size();
    float voxel_size_inv_ = 1/voxel_size_;
    int voxels_per_side_ = tsdf_layer.voxels_per_side();
    float voxels_per_side_inv_ = 1/voxels_per_side_;

    voxblox::Point origin(frontier.x, frontier.y, frontier.z);
    const voxblox::Point start_scaled = origin.cast<voxblox::FloatingPoint>() * voxel_size_inv_;

    // End points of the sphere
    int rings = 8;
    int points_per_ring = 16;
    std::vector<voxblox::Point> end_points = generatePointsOnSphere(origin, 45.0, -45.0,
                                                        gain_rad, rings, points_per_ring);

    uint32_t unknown;
    uint32_t free;
    bool free_b;
    uint32_t cnt;

    for(int r = 0; r < end_points.size() ; r++){
        const voxblox::Point end_scaled = end_points[r].cast<voxblox::FloatingPoint>() * voxel_size_inv_;

        // Cast Ray of a Line
        voxblox::AlignedVector<voxblox::GlobalIndex> global_voxel_indices;
        voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);
        cnt += global_voxel_indices.size();
        free = 0;
        free_b = false;
        for (int i = 0; i < global_voxel_indices.size(); i++) {
            const voxblox::GlobalIndex& global_voxel_idx = global_voxel_indices[i];

            // Get voxel coordinates (VIS)
            voxblox::Point vox_coord = voxblox::getCenterPointFromGridIndex(global_voxel_idx, voxel_size_);
            geometry_msgs::Point vox_point;
            vox_point.x = vox_coord.x();
            vox_point.y = vox_coord.y();
            vox_point.z = vox_coord.z();

            // Getting the voxel by global index
            const voxblox::TsdfVoxel *voxel = tsdf_layer.getVoxelPtrByGlobalIndex(global_voxel_idx);
            if (voxel != nullptr) {

                if (voxel->weight > 1e-4 /*voxblox::kEpsilon*/) {
                
                    if (voxel->distance <= 0.0f) {
                        // Occupied
                        occ_vec.push_back(vox_point);
                        break;
                    } else {
                        // Free
                        free_vec.push_back(vox_point);
                        free_b = true;
                        free++;
                        // break;
                    }
                } else {
                    unk_vec.push_back(vox_point);
                    unknown++;
                }
            } else {
                // Not already mapped, treated as unknown voxel
                unk_vec.push_back(vox_point); // ¿ERROR?
                unknown++;
            }
            // if(free_b){
            //     free_b = false;
            //     if(free > 2)break;
            // } else {
            //     free = 0;
            // }
            

        }

    }

    // Gain = num_unk_voxels / total_casted_voxels
    return unknown/double(cnt);
}

std::vector<voxblox::Point> generatePointsOnSphere(const voxblox::Point& center,
                                                    double max_v, double min_v,
                                                    double radius, int numRings, int pointsPerRing) {
    std::vector<voxblox::Point> points;
    double verticalStep = ((max_v - min_v)*M_PI/180.0) / (numRings - 1);
    double angleStep = 2 * M_PI / pointsPerRing;
    double min_v_rad = (-90.0+min_v)*M_PI/180.0;

    for (int i = 0; i < numRings; ++i) {
        double theta = i * verticalStep + min_v_rad;
        for (int j = 0; j < pointsPerRing; ++j) {
            double phi = j * angleStep;
            double x = center.x() + radius * sin(theta) * cos(phi);
            double y = center.y() + radius * sin(theta) * sin(phi);
            double z = center.z() + radius * cos(theta);
            voxblox::Point point(x, y, z);
            points.push_back(point);
        }
    }

    return points;
}

bool newNodeProjection(geometry_msgs::Point tree_node, geometry_msgs::Point* q_new,
                        double s_distance, double max_slope,
                        std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr){
                            
    // Set origin and direction of ray.
    voxblox::Point origin(q_new->x, q_new->y, q_new->z);
    voxblox::Point direction(origin.x(), origin.y(), (origin.z()-100.0));
    

    // Normalize direction vector.
    direction.normalize();
    
    // Set maximum distance for ray casting.
    double max_distance = 30.0;
    
    // Calculate surface distance along ray.
    voxblox::Point surface_intersection;
    bool success = voxblox::getSurfaceDistanceAlongRay(tsdf_map_ptr->getTsdfLayer(),
                         origin, direction, max_distance, &surface_intersection);
  

    // Is not over a surface
    if(!success) return false;

    
    // Check occupancy of the voxel
    voxblox::Point voxel_point(surface_intersection.x(),
                                surface_intersection.y(),
                                surface_intersection.z() + s_distance);

    const voxblox::TsdfVoxel* voxel_ptr = tsdf_map_ptr->getTsdfLayer().getVoxelPtrByCoordinates(voxel_point);
    // Getting the voxel's world coordinates
    // const voxblox::Point voxel_coord = voxel_point.cast<voxblox::FloatingPoint>() * (1/resolution);
    
    // Check voxel exists
    if (voxel_ptr) {

        double weight = voxel_ptr->weight;

        if (weight > 1e-4 && voxel_ptr->distance >= 0.0) {
            // Is in free space.
            q_new->x = voxel_point.x();
            q_new->y = voxel_point.y();
            q_new->z = voxel_point.z();

            return true;
        } else {
            // Occupied
            return false;
        }
    } else {
        // The voxel does not exist
        return false;
    }

    

    // geometry_msgs::Point a, b;
    // a.x = origin.x();
    // a.y = origin.y();
    // a.z = origin.z();
    // b.x = surface_intersection.x();
    // b.y = surface_intersection.y();
    // b.z = surface_intersection.z();

    // if (success) {
    //     ROS_INFO("Distance to surface along ray: %f", euclideanDistance(a, b));
    // } else {
    //     ROS_WARN("No surface intersection found within max distance.");
    // }
    
}

bool pointProjection(geometry_msgs::Point* q_new, double s_distance,
                        std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr){
    

    double resolution = 0.5;
    bool surface_found = false;
    double sum_surf_intersec = 0.0;
    uint8_t cnt_surf_found = 0;
    for(int8_t i = -1 ; i < 2 ; i++){

        for(int8_t j = -1 ; j < 2 ; j++){

            voxblox::Point origin(q_new->x + (i * resolution), q_new->y + (j * resolution), q_new->z);
            voxblox::Point direction(origin.x(), origin.y(), (origin.z()-100.0));

            // Normalize direction vector.
            direction.normalize();
            
            // Set maximum distance for ray casting.
            double max_distance = 30.0;
            
            // Calculate surface distance along ray.
            voxblox::Point surface_intersection;
            bool success = voxblox::getSurfaceDistanceAlongRay(tsdf_map_ptr->getTsdfLayer(),
                                origin, direction, max_distance, &surface_intersection);
        

            // Is not over a surface
            if(success){
                surface_found = true;
                sum_surf_intersec += surface_intersection.z();
                cnt_surf_found++;
            }

            
        }

    }
        
        if(!surface_found) return false;

        q_new->z = sum_surf_intersec / cnt_surf_found + s_distance;
        return true;

    // // Set origin and direction of ray.
    // voxblox::Point origin(q_new->x, q_new->y, q_new->z);
    // voxblox::Point direction(origin.x(), origin.y(), (origin.z()-100.0));
    
    // // ROS_INFO("STEPPER (%2.2lf, %2.2lf, %2.2lf)", q_new->x, q_new->y, q_new->z);


    // // Normalize direction vector.
    // direction.normalize();
    
    // // Set maximum distance for ray casting.
    // double max_distance = 30.0;
    
    // // Calculate surface distance along ray.
    // voxblox::Point surface_intersection;
    // bool success = voxblox::getSurfaceDistanceAlongRay(tsdf_map_ptr->getTsdfLayer(),
    //                      origin, direction, max_distance, &surface_intersection);
  

    // // Is not over a surface
    // if(!success) return false;

    // // q_new->x = surface_intersection.x();
    // // q_new->y = surface_intersection.y();
    // q_new->z = surface_intersection.z() + s_distance;

    // // ROS_INFO("PROJECTED STEPPER (%2.2lf, %2.2lf, %2.2lf)", q_new->x, q_new->y, q_new->z);
    
    // return true;
    
}


double slopeDegrees(geometry_msgs::Point a, geometry_msgs::Point b){
    return 57.2957795131 * asin( (a.z-b.z) / euclideanDistance(a,b));
}

geometry_msgs::Point getRobotPosition(std::string fixed_frame, std::string base_frame,
                        tf::TransformListener& listener, tf::StampedTransform& transform){


    int temp = 0;
    while (temp == 0) {
        try {
            temp = 1;
            listener.lookupTransform(fixed_frame, base_frame, ros::Time(0), transform);
        } catch (tf::TransformException ex) {
            temp = 0;
            ros::Duration(0.1).sleep();
        }
    }


    geometry_msgs::Point aux;
    aux.x = transform.getOrigin().x(); 
    aux.y = transform.getOrigin().y();
    aux.z = transform.getOrigin().z();

    return aux;
}
