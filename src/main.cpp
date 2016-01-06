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

#include "main.h"
#include "communication.h"

#include <chrono>

#include <boost/asio.hpp>

// C++ REST SDK
#include <cpprest/http_client.h>

using namespace web;
using namespace web::http;
using namespace web::http::client;

int main(int argc, char *argv[]){

  communicator my_communicator;

  auto start = std::chrono::system_clock::now();
  my_communicator.update_data();
  std::cout << "waiting " << std::flush;
  while(!my_communicator.is_updated()){
    usleep(10);
  }
  auto end = std::chrono::system_clock::now();
  auto elapsed = end - start;
  std::cout << std::endl;

  env_data_t my_env_data = my_communicator.get_env_data();

  std::cout << "Height:   " << my_env_data.height << std::endl;
  std::cout << "Width:    " << my_env_data.width << std::endl;
  std::cout << "Time(ms): " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() << std::endl;

  return 0;
}
