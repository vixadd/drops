///////////////////////////////////////////////////////////////////////////////
// main.cpp - Main for DROPS - Dynamics Realtime Obstacle Pathing System
//Copyright (C) 2015  Christopher Newport University
//
//This program is free software; you can redistribute it and/or
//modify it under the terms of the GNU General Public License
//as published by the Free Software Foundation; either version 2
//of the License, or (at your option) any later version.
//
//This program is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.
//
//You should have received a copy of the GNU General Public License
//along with this program; if not, write to the Free Software
//Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
///////////////////////////////////////////////////////////////////////////////

#include "main.hpp"
#include "communication.hpp"
#include "plan.hpp"

#include <chrono>

#include <boost/asio.hpp>

// C++ REST SDK
#include <cpprest/http_client.h>

const std::string CONFIG_FILENAME = "./src/communicator_config.txt";

using namespace web;
using namespace web::http;
using namespace web::http::client;

void print_env(env_data_t my_env_data)
{
    std::cout << "Grid"  << std::endl << std::endl;
    for(int i = 0; i < my_env_data.height; i++) {
        for(int j = 0; j < my_env_data.width; j++) {
            if(j == my_env_data.start_x && i == my_env_data.start_y) {
                std::cout << "S";
            } else if (j == my_env_data.end_x && i == my_env_data.end_y) {
                std::cout << "G";
            } else if(my_env_data.grid_2d[j + i * my_env_data.width] == 0) {
                std::cout << " ";
            } else if (my_env_data.grid_2d[j + i * my_env_data.width] < 255) {
                std::cout << (my_env_data.grid_2d[j + i * my_env_data.width] / 29);
            } else {
                std::cout << "O";
            }
        }
        std::cout << std::endl;
    }
}

void print_env_const(env_constants_t my_env_const)
{
    std::cout << std::endl << "Env Constatnts" << std::endl;
    std::cout << "obs_thresh: " << +my_env_const.obs_thresh << std::endl;
    std::cout << "cost_inscribed_thresh: " << +my_env_const.cost_inscribed_thresh << std::endl;
    std::cout << "cost_possibly_circumscribed_thresh: " << my_env_const.cost_possibly_circumscribed_thresh << std::endl;
    std::cout << "est_velocity: " << my_env_const.est_velocity << std::endl;
    std::cout << "timetoturn45degs: " << my_env_const.timetoturn45degs << std::endl;
    std::cout << "cellsize_m: " << my_env_const.cellsize_m << std::endl << std::endl;

}

void print_path(std::vector<sbpl_xy_theta_pt_t> path)
{
    for (unsigned int i = 0; i < path.size(); i++) {
        printf("%.3f %.3f %.3f\n", path.at(i).x, path.at(i).y, path.at(i).theta);
    }
}

int main(int argc, char *argv[])
{

    communicator my_communicator;
    my_communicator.import_config(CONFIG_FILENAME);

    auto start = std::chrono::system_clock::now();
    my_communicator.update_data();
    std::cout << "waiting " << std::flush;
    while(!my_communicator.is_updated()) {
        usleep(10);
    }
    auto end = std::chrono::system_clock::now();
    auto elapsed = end - start;
    std::cout << std::endl;

    env_data_t my_env_data = my_communicator.get_env_data();
    env_constants_t my_env_const = my_communicator.get_const_data();
    point_char_map moveing_obs_pts = my_communicator.get_updated_points();

    std::cout << "Height:   " << my_env_data.height << std::endl;
    std::cout << "Width:    " << my_env_data.width << std::endl;
    std::cout << "Time(ms): " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() << std::endl;

    print_env_const(my_env_const);

    std::cout << "Creating Planner" << std::endl;

    //Create a planner object
    Planner my_planner;
    {
        //Lock the grid, then intialize the planner
        std::unique_lock<std::mutex> env_grid_lock = my_communicator.get_lock_env_grid_2d();
        std::cout << "Initialize Planner" << std::endl;
        my_planner.initialize(my_env_data, my_env_const);
    }
    //Update the planner with the moving obstacles
    std::cout << "Update Planner" << std::endl;
    my_planner.update_grid_points(moveing_obs_pts);

    //Plan!
    std::cout << "Plan" << std::endl;
    int has_path = my_planner.plan();

    std::cout << "Plan returned: " << has_path << std::endl;

    if(has_path) {
        //Print the path to stdout
        print_path(my_planner.get_path());
    }

    return 0;
}
