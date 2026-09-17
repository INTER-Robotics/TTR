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

int main(int argc, char** argv) {
  ros::init (argc, argv, "planner");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  voxblox::EsdfServer node(nh, nh_private);

  ttr::Planner mm(nh, nh_private);
  
  ros::spin();
  // ros::AsyncSpinner spinner(4); // Use 4 threads
  // spinner.start();
  // ros::waitForShutdown();
  return 0;
}