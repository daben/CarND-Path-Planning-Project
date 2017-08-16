#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include <sys/types.h>
#include <sys/stat.h>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "planner.hpp"

// #define WS_BY_REF

#ifdef WS_BY_REF
# define WS_PARAM(name) *name
# define WS_P(name) (name)
#else
# define WS_PARAM(name) name
# define WS_P(name) (&name)
#endif

using namespace std;

// for convenience
using json = nlohmann::json;

namespace {
namespace carnd {
  // Convert from json to sensor data
  void from_json(const json & j, sensor_data &sensor) {
    sensor.uid = j[0].get<int>();
    sensor.x = j[1];
    sensor.y = j[2];
    sensor.vx = j[3];
    sensor.vy = j[4];
    sensor.s = j[5];
    sensor.d = j[6];
  }

  // Convert from json to telemetry data
  void from_json(const json & j, telemetry_data &tele) {
    tele.x = j["x"];
    tele.y = j["y"];
    tele.yaw = deg2rad(j["yaw"]);
    tele.s = j["s"];
    tele.d = j["d"];
    tele.speed = mph2mps(j["speed"]);

    // Previous path data given to the Planner
    tele.previous_path.x = j["previous_path_x"].get<vector<double>>();
    tele.previous_path.y = j["previous_path_y"].get<vector<double>>();
    // Previous path's end s and d values
    tele.end_path.s = j["end_path_s"];
    tele.end_path.d = j["end_path_d"];

    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    tele.sensor_fusion = j["sensor_fusion"].get<vector<sensor_data>>();
  }
}}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
static string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

static bool file_exists(const std::string & filename) {
    struct stat info;
    return stat(filename.c_str(), &info) == 0 && ((info.st_mode & S_IFMT) == S_IFREG);
}


int main(int argc, char ** argv) {
  uWS::Hub h;

  // Path planner object
  carnd::PathPlanner planner;

  // Waypoint map to read from
  string map_file_;

  if (argc == 2) {
    map_file_ =  argv[1];
  } else {
    struct stat info;

    const vector<string> paths{"data/", "../data/", "../../data/", "../../../data/"};

    map_file_ = "highway_map.csv";
    for (int k = 0; !file_exists(map_file_) && k < paths.size(); k++) {
      map_file_ = paths[k] + "highway_map.csv";
    }
  }

  if (!file_exists(map_file_)) {
    cerr << "Error: waypoints file not found." << endl;
    return -1;
  }

  cerr << "Using: " << map_file_ << endl;

  // Initialize the path planner with the map file
  planner.initialize(map_file_);

  h.onMessage([&planner](uWS::WebSocket<uWS::SERVER> WS_PARAM(ws),
                         char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          carnd::path_t next_path;

          // j[1] is the data JSON object

          // define a path made up of (x,y) points that the car will visit
          // sequentially every .02 seconds
          planner.run(j[1], next_path, 0.02);

          json msgJson;
          msgJson["next_x"] = next_path.x;
          msgJson["next_y"] = next_path.y;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          WS_P(ws)->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        WS_P(ws)->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&planner](uWS::WebSocket<uWS::SERVER> WS_PARAM(ws), uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    planner.reset();
  });

  h.onDisconnection([](uWS::WebSocket<uWS::SERVER> WS_PARAM(ws), int code,
                      char *message, size_t length) {
    std::cout << "Disconnected" << std::endl;
    //WS_P(ws)->close();
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
