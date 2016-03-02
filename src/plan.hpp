///////////////////////////////////////////////////////////////////////////////
// plan.h - Header for planner - Dynamics Realtime Obstacle Pathing System
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

#ifndef PLAN_H
#define PLAN_H

#include "communication.hpp" //For the types
#include <sbpl/headers.h>
#include "util.hpp"

class Planner {
public:

    static const int PATH_EXISTS = 1;

    Planner();
    virtual ~Planner();

    // TODO: Figure out the params for the following functions
    int update_grid_points(point_char_map &points);
    int initialize(env_data_t &env_data, env_constants_t &env_const);
    int plan();
    std::vector<sbpl_xy_theta_pt_t> get_path();

private:

    //Creates the planner
    int init_planner();
    //Sets up the planner for use with the current set of goal
    int set_planner_states(int start_state_id, int goal_state_id);


    //---Environment---
    //Environment settings
    EnvironmentNAVXYTHETALAT m_env;
    MDPConfig MDPCfg; // Not exactly sure what this is, but its in the example

    //---Planner---
    //Planner Settings
    double planning_time; //In seconds
    double initial_epsilon; //The initial epsilon used for planning (a multiplier on the heuristic)

    bool search_forward; //Should we search forward or backwards. defaults to backwards (less replaning)
    bool changed; //Has the environment changed

    bool last_plan_good; //True if the last plan we tried was good.

    std::vector<nav2dcell_t> changed_cells; // A vector of the cells changed this time.

    std::vector<sbpl_2Dpt_t> perimeterptsV; //The perimeters of the vehicle

    SBPLPlanner* m_planner = NULL; //By making this a pointer, we can use whatever planner we want,
    //but we need to make sure we delete it

    std::vector<sbpl_xy_theta_pt_t> xythetaPath;

};



#endif /* PLAN_H */
