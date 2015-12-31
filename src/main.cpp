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

#include <boost/asio.hpp>

// C++ REST SDK
#include <cpprest/http_client.h>

#define HOST "http://private-6dd53-jamapi.apiary-mock.com/"

using namespace web;
using namespace web::http;
using namespace web::http::client;

auto grid_size_thread(http_client &my_client)
{
  return my_client.request(methods::GET, U("/api/grid")).then([](http_response resp)
  {
    return resp.extract_json();
  }).then([](json::value grid_json)
  {
    grid_t my_grid;
    if (!grid_json[U("x")].is_null()){
      my_grid.x = grid_json[U("x")].as_integer();
    } else {
      //We had an error in the request
      //For now, we will set the value to undefined
      my_grid.x = GRID_UNDEFINED;
    }
    if (!grid_json[U("y")].is_null()){
      my_grid.y = grid_json[U("y")].as_integer();
    } else {
      //We had an error in the request
      //For now, we will set the value to undefined
      my_grid.y = GRID_UNDEFINED;
    }
    return my_grid;
  });
}

int main(int argc, char *argv[]){

  http_client_config my_client_config;
  my_client_config.set_nativehandle_options([](native_handle  handle)
    {
      // I think this code should set the socket to keep_alive, not sure though
      boost::asio::ip::tcp::socket* socket = static_cast<boost::asio::ip::tcp::socket*>(handle);
      if(socket->is_open()){
        boost::asio::socket_base::keep_alive option(true);
        socket->set_option(option);
      }
    });

  http_client my_client(U(HOST), my_client_config);

  auto grid_size_worker = grid_size_thread(my_client);
  grid_t my_grid = grid_size_worker.get();

  std::cout << "X:" << my_grid.x << std::endl;
  std::cout << "Y:" << my_grid.y << std::endl;

  return 0;
}
