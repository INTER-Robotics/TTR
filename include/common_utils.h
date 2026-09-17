#ifndef common_utils_H
#define common_utils_H
#include <math.h>
#include <stdlib.h>
#include <time.h>

#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <vector>

#include "geometry_msgs/Point.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <random>

#include <voxblox/interpolator/interpolator.h>
#include <voxblox/utils/layer_utils.h>
#include <voxblox/utils/planning_utils.h>
#include <voxblox/utils/planning_utils_inl.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/ros_params.h>
#include <voxblox_ros/transformer.h>
#include "voxblox/utils/distance_utils.h"

#include <std_msgs/UInt8.h>

enum ExplorationState {
    PAUSED,      
    EXPLORING,  
    RETURNING,      // Returning home
    FINISHED        // Completed or cancelled     
};

// Euclidean distance between two points 
double euclideanDistance(geometry_msgs::Point, geometry_msgs::Point);

// Nearest node of the tree
geometry_msgs::Point nearestNeighbour(std::vector<geometry_msgs::Point>, geometry_msgs::Point);

// Get point at correct distance in the line
geometry_msgs::Point steer(geometry_msgs::Point, geometry_msgs::Point, float);

char freeLine(geometry_msgs::Point, geometry_msgs::Point*, octomap::OcTree* octree);

char freeLineAtDistance(geometry_msgs::Point, geometry_msgs::Point*, octomap::OcTree* octree, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree);

char obstacle(geometry_msgs::Point, octomap::OcTree* octree);

bool obstacleAtDistance(geometry_msgs::Point point, double distance, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree);


char freeLineAtDistance(geometry_msgs::Point p_nearest, geometry_msgs::Point* p_new,  pcl::KdTreeFLANN<pcl::PointXYZ> &obs_kdtree_, pcl::KdTreeFLANN<pcl::PointXYZ> &trav_kdtree_, double res, double margin_);
bool isFrontier(geometry_msgs::Point point, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, pcl::KdTreeFLANN<pcl::PointXYZ> &obs_kdtree, double resolution, double margin);
bool isFrontier(geometry_msgs::Point point, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, double resolution);
bool freeLineMargin(geometry_msgs::Point a, geometry_msgs::Point b,  pcl::KdTreeFLANN<pcl::PointXYZ> &obs_kdtree_, double res, double margin_);

double distanceObstacle(geometry_msgs::Point point, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree);

// Using voxblox
double getMapDistanceVB(const Eigen::Vector3d& position, std::shared_ptr<voxblox::EsdfMap> mapPtr);

bool freeLineMarginVB(geometry_msgs::Point a, geometry_msgs::Point b,
                        std::shared_ptr<voxblox::EsdfMap> mapPtr,
                        std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr,
                        double res, double margin_, bool ground_robot,
                        double max_slope);

bool freeLineVB(geometry_msgs::Point a, geometry_msgs::Point b,
                std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr, double res);

signed char freeLineAtDistanceVB(geometry_msgs::Point p_nearest, geometry_msgs::Point* p_new,
                             std::shared_ptr<voxblox::EsdfMap> esdf_map_ptr,
                             std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr,
                             double res, double margin_, bool ground_robot,
                             double max_slope, double proj_height);

bool isFrontierVB(geometry_msgs::Point point,
                    std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr, double resolution);

double getInformationGain(geometry_msgs::Point frontier,
                        std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr,
                        double map_res, double gain_rad, double occ_penals);

double getInformationGainRayCast(geometry_msgs::Point frontier,
                        std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr,
                        double map_res, double gain_rad, std::vector<geometry_msgs::Point>& unk_vec,
                        std::vector<geometry_msgs::Point>& free_vec, std::vector<geometry_msgs::Point>& occ_vec);

std::vector<voxblox::Point> generatePointsOnSphere(const voxblox::Point& center,
                                                    double max_v, double min_v,
                                                    double radius, int numRings, int pointsPerRing); 

bool newNodeProjection(geometry_msgs::Point tree_node, geometry_msgs::Point* q_new,
                        double s_distance, double max_slope,
                        std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr);

bool pointProjection(geometry_msgs::Point* q_new, double s_distance,
                        std::shared_ptr<voxblox::TsdfMap> tsdf_map_ptr);


double slopeDegrees(geometry_msgs::Point a, geometry_msgs::Point b);

geometry_msgs::Point getRobotPosition(std::string fixed_frame, std::string base_frame,
                    tf::TransformListener& listener, tf::StampedTransform& transform);

#endif
