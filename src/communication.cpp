#include "communication.h"

#include <algorithm>


using namespace web::http;
using namespace web::http::client;


communicator::communicator():m_updated(false), m_posted(false),m_env_data(),m_task_update([](){}),m_client(U(HOST)){
  m_client_config.set_nativehandle_options([](native_handle  handle)
    {
      // I think this code should set the socket to keep_alive, not sure though
      boost::asio::ip::tcp::socket* socket = static_cast<boost::asio::ip::tcp::socket*>(handle);
      if(socket->is_open()){
        boost::asio::socket_base::keep_alive option(true);
        socket->set_option(option);
      }
    });
  m_client = http_client(U(HOST), m_client_config);

}

communicator::~communicator(){
  if(m_env_data.grid_2d != NULL){
    for (int x = 0; x < m_env_data.width; x++)
      delete[] m_env_data.grid_2d[x];
    delete[] m_env_data.grid_2d;
    m_env_data.grid_2d = NULL;
  }
}

void communicator::update_data(){
  if(update_in_progress())
    return; //Don't run an update if one is in progress
  m_updated = false;

  m_task_update = get_grid();
}

bool communicator::is_updated(){
  return m_updated;
}

bool communicator::update_in_progress(){
  return !m_task_update.is_done();
}

env_data_t communicator::get_env_data(){
  std::lock_guard<std::mutex> lock(m_env_data_mutex);
  return m_env_data;
}

pplx::task<void> communicator::get_grid(){
  return m_client.request(methods::GET, U("/api/grid")).then([](http_response resp)
    {
      return resp.extract_json();
    }).then([this](web::json::value grid_json)
    {
      // Uncomment to print out the json object we recieved
      //std::cout << grid_json.serialize() << std::endl;

      int height = 0;
      int width = 0;

      // Grid Size
      if (grid_json.at(U("grid_width")).is_number()){
        width = grid_json.at(U("grid_width")).as_integer();
      } else {
        throw web::json::json_exception(U("value grid_width is messed up"));
      }
      if (grid_json.at(U("grid_height")).is_number()){
        height = grid_json.at(U("grid_height")).as_integer();
      } else {
        throw web::json::json_exception(U("value grid_height is messed up"));
      }

      // Obstacles
      std::vector<moving_obstacle_t> moving_obstacles;
      std::vector<stationary_obstacle_t> stationary_obstacles;

      auto obstacles_json = grid_json.at(U("obstacles"));

      if (obstacles_json.at(U("moving_obstacles")).is_array()){
        std::for_each(obstacles_json.at(U("moving_obstacles")).as_array().begin(),
                      obstacles_json.at(U("moving_obstacles")).as_array().end(),
                      [&moving_obstacles](web::json::value &obstacle_json)
                      {
                        int x = obstacle_json.at(U("x")).as_integer();
                        int y = obstacle_json.at(U("y")).as_integer();
                        int rad = obstacle_json.at(U("radius")).as_integer();
                        int head = obstacle_json.at(U("heading")).as_integer();
                        int vel = obstacle_json.at(U("velocity")).as_integer();
                        moving_obstacles.push_back({x,y,rad,head,vel});
                      });
      } else {
        throw web::json::json_exception(U("value moving_obstacles not found in grid"));
      }
      if (obstacles_json.at(U("stationary_obstacles")).is_array()){
        std::for_each(obstacles_json.at(U("stationary_obstacles")).as_array().begin(),
                      obstacles_json.at(U("stationary_obstacles")).as_array().end(),
                      [&stationary_obstacles](web::json::value &obstacle_json)
                      {
                        int x = obstacle_json.at(U("x")).as_integer();
                        int y = obstacle_json.at(U("y")).as_integer();
                        int rad = obstacle_json.at(U("radius")).as_integer();
                        stationary_obstacles.push_back({x,y,rad});
                      });
      } else {
        throw web::json::json_exception(U("value stationary_obstacles not found in grid"));
      }

      //goal
      auto goal_json = grid_json.at(U("goal"));

      int goal_x = 0;
      int goal_y = 0;
      int goal_theta = 0;

      if (goal_json.at(U("x")).is_number()){
        goal_x = goal_json.at(U("x")).as_integer();
      } else {
        throw web::json::json_exception(U("value x not found in goal"));
      }
      if (goal_json.at(U("y")).is_number()){
        goal_y = goal_json.at(U("y")).as_integer();
      } else {
        throw web::json::json_exception(U("value y not found in goal"));
      }
      if (goal_json.at(U("theta")).is_number()){
        goal_theta = goal_json.at(U("theta")).as_integer();
      } else {
        throw web::json::json_exception(U("value theta not found in goal"));
      }

      //location
      auto location_json = grid_json.at(U("location"));

      int location_x = 0;
      int location_y = 0;
      int location_theta = 0;

      if (location_json.at(U("x")).is_number()){
        location_x = location_json.at(U("x")).as_integer();
      } else {
        throw web::json::json_exception(U("value x not found in location"));
      }
      if (location_json.at(U("y")).is_number()){
        location_y = location_json.at(U("y")).as_integer();
      } else {
        throw web::json::json_exception(U("value y not found in location"));
      }
      if (location_json.at(U("theta")).is_number()){
        location_theta = location_json.at(U("theta")).as_integer();
      } else {
        throw web::json::json_exception(U("value theta not found in location"));
      }

      std::lock_guard<std::mutex> lock(m_env_data_mutex);
      //Lock before editing the env data.
      //height and width
      m_env_data.height = height;
      m_env_data.width = width;

      //goal
      m_env_data.end_x = goal_x;
      m_env_data.end_y = goal_y;
      m_env_data.end_theta = goal_theta;

      //location
      m_env_data.start_x = location_x;
      m_env_data.start_y = location_y;
      m_env_data.start_theta = location_theta;

      //free memory if not null
      if(m_env_data.grid_2d != NULL){
        for (int x = 0; x < m_env_data.width; x++)
          delete[] m_env_data.grid_2d[x];
        delete[] m_env_data.grid_2d;
        m_env_data.grid_2d = NULL;
      }

      //make a new grid_2d array
      m_env_data.grid_2d = new unsigned char*[m_env_data.width];
      for (int x = 0; x < m_env_data.width; x++) {
        m_env_data.grid_2d[x] = new unsigned char[m_env_data.height];
      }

      // TODO: Convert obstacles to grid

      m_updated = true;

    }).then([](pplx::task<void> task)
    {
      //This step is just to catch exceptions
      try{
        task.get();
      }
      catch(const std::exception& ex){
        // TODO: Do something about exceptions here
        std::cout << "Caught Exception: " << ex.what() << std::endl;
      }
    });
}
