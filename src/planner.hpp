#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <functional>
#include <algorithm>
#include <cmath>
#include <random>
#include "spline.h"


#ifndef NDEBUG
#define DEBUG_LOG std::cout
#else
static struct {} debug_log_disabled_;
template<typename T>
debug_log_disabled_& operator << (debug_log_disabled_& any, T const& thing) { return any; }
#define DEBUG_LOG debug_log_disabled_
#endif


// Anonymous namespace to avoid duplicated symbols in object files
namespace {

namespace carnd // == Utils ===================================================
{
  using namespace std;

  // For converting back and forth between radians and degrees.  conestex
  constexpr double pi() { return M_PI; }
  constexpr double deg2rad(double x) { return x * (M_PI / 180.0); }
  constexpr double rad2deg(double x) { return x * (180.0 / M_PI); }

  // Infinity is useful
  constexpr double INF = std::numeric_limits<double>::infinity();

  // miles/hour to meters/second
  static constexpr double MPH2MPS = 0.44704;

  constexpr double mph2mps(double x) { return x * MPH2MPS; }
  constexpr double mps2mph(double x) { return x / MPH2MPS; }

  static constexpr double MILE2METER = 1609.34;

  constexpr double miles2meters(double x) { return x * MILE2METER; }
  constexpr double meters2miles(double x) { return x / MILE2METER; }

  // dot product
  double dot(double x1, double y1, double x2, double y2) {
    return x1 * x2 + y1 * y2;
  }

  // 2d vector norm
  double norm(double x, double y) {
    return sqrt(x * x + y * y);
  }

  // euclidean distance
  double distance(double x1, double y1, double x2, double y2) {
    return norm(x2 - x1, y2 - y1);
  }
}

namespace carnd // == Spline interpolation ====================================
{
  // interpolated curve
  struct spline_curve {
    void fit(vector<double> s, vector<double> x, vector<double> y);
    inline double x(double s) const { return s_x_(s); }
    inline double y(double s) const { return s_y_(s); }
  private:
    tk::spline s_x_;
    tk::spline s_y_;
  };

  void spline_curve::fit(vector<double> s, vector<double> x, vector<double> y) {
    // TODO: Check loops where s is not monotonic
    s_x_.set_points(s, x);
    s_y_.set_points(s, y);
  }
}

namespace carnd // === Localization ===========================================
{
  // cartesian point
  struct xy_t { double x, y; };
  // freenet point
  struct sd_t { double s, d; };

  // waypoint
  struct waypoint_t { double x, y, s, dx, dy; };
  // waypoint list as a set of vectors
  struct waypoints_list {
    vector<double> x, y, s, dx, dy;
    size_t size() const { return x.size(); }
    waypoint_t operator[](int i) const { return {x[i],y[i],s[i],dx[i],dy[i]}; }
  };


  struct RoadMap {
    constexpr static double default_fov = M_PI / 4;
    // The max s value before wrapping around the track back to 0
    const double max_s = 6945.554;

    waypoints_list waypoints;

    // Load map waypoints from a csv file
    void load(const string &filename);

    // Convert frenet to cartesian
    xy_t to_xy(double s, double d) const;
    // Convert cartesian to frenet
    sd_t to_frenet(double x, double y, double theta) const;

    // closest waypoint to x, y
    int closest_waypoint(double x, double y) const;

    // closest waypoint looking forward in the direction of theta
    int next_waypoint(double x, double y, double theta,
                      const double fov = default_fov) const;
  };

  // RoadMap definitions...

  int RoadMap::closest_waypoint(double x, double y) const {
    double min_dist = std::numeric_limits<double>::max();
    int closest = 0;
    const int n = waypoints.x.size();
    for(int i = 0; i < n; i++) {
      auto dist = distance(x, y, waypoints.x[i], waypoints.y[i]);
      if (dist < min_dist) {
        min_dist = dist;
        closest = i;
      }
    }
    return closest;
  }

  int RoadMap::next_waypoint(double x, double y, double theta, const double fov) const {
    int next_wp = closest_waypoint(x, y);
    xy_t wp = {waypoints.x[next_wp], waypoints.y[next_wp]};
    auto heading = atan2(wp.y - y, wp.x - x);
    auto angle = fabs(theta - heading);
    if (angle > fov)
      next_wp = (next_wp + 1) % waypoints.x.size();
    return next_wp;
  }

  sd_t RoadMap::to_frenet(double x, double y, double theta) const {
    const int next_ = next_waypoint(x,y, theta);
    const int prev_ = (next_ - 1) % waypoints.size();

    auto next_wp = waypoints[next_];
    auto prev_wp = waypoints[prev_];

    double n_x = next_wp.x - prev_wp.x;
    double n_y = next_wp.y - prev_wp.y;
    double x_x = x - prev_wp.x;
    double x_y = y - prev_wp.y;

    // find the projection of x onto n
    double proj_norm = dot(x_x,x_y,n_x,n_y) / dot(n_x,n_y,n_x,n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = distance(x_x, x_y, proj_x, proj_y);

    // see if d value is positive or negative by comparing it to a center point

    double center_x = 1000 - prev_wp.x;
    double center_y = 2000 - prev_wp.y;
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if(centerToPos <= centerToRef) {
      frenet_d = -frenet_d;
    }

    // calculate s value
    double frenet_s = prev_wp.s + distance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};
  }

  xy_t RoadMap::to_xy(double s, double d) const {
    int wp1 = -1;
    while((s > waypoints[wp1+1].s) && (wp1 < int(waypoints.size()-1)))
      wp1++;

    int wp2 = (wp1 + 1) % waypoints.size();

    double heading = atan2(waypoints[wp2].y - waypoints[wp1].y,
                           waypoints[wp2].x - waypoints[wp1].x);
    double seg_s = s - waypoints[wp1].s;
    double seg_x = waypoints[wp1].x + seg_s * cos(heading);
    double seg_y = waypoints[wp1].y + seg_s * sin(heading);
    double perp = heading - pi() / 2;

    double x = seg_x + d * cos(perp);
    double y = seg_y + d * sin(perp);

    return {x, y};
  }

  void RoadMap::load(const string &filename) {
    ifstream in_map_(filename, ifstream::in);

    waypoints.x.clear();
    waypoints.y.clear();
    waypoints.s.clear();
    waypoints.dx.clear();
    waypoints.dy.clear();

    string line;
    while (getline(in_map_, line)) {
      istringstream iss(line);

      double x, y, s, dx, dy;
      iss >> x >> y >> s >> dx >> dy;

      waypoints.x.push_back(x);
      waypoints.y.push_back(y);
      waypoints.s.push_back(s);
      waypoints.dx.push_back(dx);
      waypoints.dy.push_back(dy);
    }

    // Add the last point
    waypoints.s.push_back(max_s);
    waypoints.x.push_back(waypoints.x[0]);
    waypoints.y.push_back(waypoints.y[0]);
    waypoints.dx.push_back(waypoints.dx[0]);
    waypoints.dy.push_back(waypoints.dy[0]);
  }


  // Helper representing the set of lanes in the highway.
  // The origin is in the middle of the road; from there lanes
  // are numbered: 0, 1, 2, ...
  struct Track
  {
    // These are actually constants
    double speed_limit_mph = 50;
    int    lane_count = 3;
    double lane_width = 4;

    // Track width
    double width() const {
      return lane_count * lane_width;
    }

    // Lane center from the origin
    double lane_center(int lane) const {
      return (0.5 + lane) * lane_width;
    }

    // Lane center with a bias in the left and right lanes
    double safe_lane_center(int lane) const {
      double center_ = lane_center(lane);
      if (lane == 0) // left lane
        center_ += 0.05;
      else if (lane == lane_count - 1) // right lane
        center_ -= 0.05;
      return center_;
    }

    // Lane for a given frenet point
    int lane_at(sd_t frenet) const {
      return lane_at(frenet.d);
    }

    // Lane at a given distance from the road center
    int lane_at(double d) const {
      if (d < 0)
        return -1;
      if (d > width())
        return -2;
      return std::floor(d / lane_width);
    }

    double distance_to_lane(double d, int lane) {
      return lane_center(lane) - d;
    }

    // FIXME fractional version
    double lane_center_distance_at(double d) const {
      return std::fmod(d, lane_width) / (lane_width / 2) - 1;
    }
  };
} // ::carnd


namespace carnd // === Planner ====================
{
  struct path_t
  {
    vector<double> x, y;

    size_t size() const { return x.size(); }
    void append(const xy_t xy) {
      x.push_back(xy.x);
      y.push_back(xy.y);
    }
    void append(const double x_, const double y_) {
      x.push_back(x_);
      y.push_back(y_);
    }
  };

  // Sensor Fusion data represint other cars
  struct sensor_data {
    int    uid;  // unique identifier
    double x;    // x position in map coordinates
    double y;    // y position in map coordinates
    double vx;   // velocity in x direction (m/s)
    double vy;   // velocity in y direction (m/s)
    double s;    // s position in frenet coordinates
    double d;    // d position in frenet coordinates
  };

  // Telemetry
  struct telemetry_data {
    // Main car's localization Data
    double x;     // x position in map coordinates
    double y;     // y position in map coordinates
    double yaw;   // yaw angle in map coordinates (radians)
    double speed; // speed (m/s)
    double s;     // frenet
    double d;     // frenet
    // Previous path data given to the Planner
    path_t previous_path;
    // Previous path's end s and d values
    sd_t end_path;
    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    vector<sensor_data> sensor_fusion;
  };

  struct lane_info_t
  {
    int    front_car   = -1;
    int    back_car    = -1;
    double front_gap   = INF;
    double front_speed = INF;
    double back_gap    = INF;
    double back_speed  = 0;
    bool   feasible    = true;

    bool is_clear() const { return feasible && front_car < 0; }
  };

  // Path Planner
  struct PathPlanner {
    RoadMap   roadmap;
    Track     track;

    double    accel = 0.22;     // m/s^2
    double    lane_horizon = 50;   // m
    double    lane_change_front_buffer = 15; // m
    double    lane_change_back_buffer = 7;  // m

    // number of points to generate ... constant
    int       n_path_points = 50;

    // Reference position for new path - 2 points to build a tangent
    double    ref_x;
    double    ref_y;
    double    ref_x_prev;
    double    ref_y_prev;
    double    ref_yaw;
    double    ref_speed;
    double    ref_s;
    double    ref_d;
    int       ref_lane; 
    int       ref_points;  // number of steps consumed at reference 

    vector<lane_info_t> lane_info;

    // lap tracking for the ego
    size_t    ego_laps;
    size_t    ego_laps_tick;
    sd_t      ego_start_position;
    bool      ego_passed_zero_s;

    // target lane for changing lane states
    int       changing_lane = -1;
    // target lane for next path (of left=0, middle=1, right=2)
    int       target_lane  = 1;
    // target speed for next path
    double    target_speed = 0;

    enum class STATE { START, KL, PLC, LC };
    STATE     state_ = STATE::KL;
    double    state_s_;

   public:

    void initialize(const string & map_file_) {
      roadmap.load(map_file_);
    }

    // Reset planner
    void reset() {
      target_lane  = 1;
      target_speed = 0;
      ref_points = 0;
      ego_laps = 0;
      ego_laps_tick = 0;
      ego_passed_zero_s = false;
      state_ = STATE::START;
      state_s_ = 0;
    }

    // Run the planner with the given telemetry to generate the next trajectory.
    // dt is the simulator period.
    void run(const telemetry_data & ego, path_t & path, double dt);

   protected:

    void track_lap(const telemetry_data &ego);
    void compute_reference(const telemetry_data & ego, double dt);
    void process_sensor_fusion(const telemetry_data & ego, double dt);
    void create_plan(const telemetry_data & ego, double dt);
    void build_path(const telemetry_data & ego,
                    const int target_lane,
                    const double target_speed,
                    path_t & path,
                    double dt);

    int get_best_lane() const;
    void set_state(const telemetry_data & ego, STATE new_state);
  };

  // == Implementations =================================

  void PathPlanner::run(const telemetry_data & ego, path_t & path, double dt) {
    // 1a. Get reference point: where we start to build the plan
    compute_reference(ego, dt);
    // 1b. Track laps
    track_lap(ego);
    // 2. Analyse the environment
    process_sensor_fusion(ego, dt);
    // 3. Behaviour; set target lane and speed
    create_plan(ego, dt);
    // 4. Generate final trajectory
    build_path(ego, target_lane, target_speed, path, dt);
  }

  void PathPlanner::track_lap(const telemetry_data & ego) {
    if (ego_laps_tick == 0) {
      ego_start_position = {ego.s, ego.d};
    } 
    
    // EGO .. current position
    if (ego.s < ego_start_position.s) {
      ego_passed_zero_s = true;
    }

    if (ego_passed_zero_s && ego.s > ego_start_position.s) {
      ego_laps ++;
      ego_laps_tick = 0;
      ego_passed_zero_s = false;

      DEBUG_LOG << "#################################################################";
    }
    else {
      DEBUG_LOG << "_________________________________________________________________";
    }

    ego_laps_tick++;

    DEBUG_LOG
        << endl
        << endl 
        << "LAP "  << (ego_laps + 1)
        << "\tLANE " << track.lane_at(ego.d)
        << " (s="  << fixed << setprecision(1) << ego.s
        << ", d="  << fixed << setprecision(1) << ego.d << ")"
        << " PLANNED " << ego.previous_path.size() << " points"
        << endl;
  }

  void PathPlanner::compute_reference(const telemetry_data & ego, double dt) {
    const int planned_size = ego.previous_path.size();

    // If previous path almost empty, use the current ego position
    if (planned_size < 2)
    {
      ref_x = ego.x;
      ref_y = ego.y;
      ref_yaw = ego.yaw;
      ref_speed = ego.speed;
      ref_s = ego.s;
      ref_d = ego.d;

      ref_x_prev = ref_x - cos(ref_yaw);
      ref_y_prev = ref_y - sin(ref_yaw);
    }
    else // use the previous path
    {
      ref_x = *(ego.previous_path.x.end() - 1);
      ref_y = *(ego.previous_path.y.end() - 1);

      ref_x_prev = *(ego.previous_path.x.end() - 2);
      ref_y_prev = *(ego.previous_path.y.end() - 2);

      ref_yaw = std::atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
      ref_speed = distance(ref_x_prev, ref_y_prev, ref_x, ref_y) / dt;

      ref_s = ego.end_path.s;
      ref_d = ego.end_path.d;
    }

    ref_lane = track.lane_at(ref_d);

    // FIXME assuming n_path_points keeps constant. If not, we have to
    // track the size of the previous path.
    ref_points += n_path_points - planned_size;
  }

  void PathPlanner::process_sensor_fusion(const telemetry_data & ego, double dt) {
    lane_info.clear();
    lane_info.resize(track.lane_count);

    DEBUG_LOG << "SENSOR FUSION" << endl;

    const int planned_size = ego.previous_path.size();

    for(auto & car : ego.sensor_fusion) {
      int car_lane = track.lane_at(car.d);

      // vehicle in the road
      if (car_lane >= 0) {
        auto & lane = lane_info[car_lane];

        // predict car position assuming constant speed
        auto car_speed = norm(car.vx, car.vy);
        // predict car position after consumption of pending plan
        auto car_next_s = car.s + car_speed * planned_size * dt;
        // check if it's in front or behind
        bool in_front = car_next_s > ref_s;
        // absolute s-distance
        auto car_gap = fabs(car_next_s - ref_s);

        DEBUG_LOG << "  CAR "  << setw(2) << car.uid
                  << "  lane=" << car_lane
                  << "  v="    << setw(4) << mps2mph(norm(car.vx, car.vy))
                  << "  s="    << setw(6) << car.s
                  << "  s'="   << setw(6) << car_next_s
                  << "  gap="  << setw(7) << (car_next_s - ref_s)
                  << endl;

        // if getting under the buffer
        if (in_front && car_gap < lane.front_gap) {
          lane.front_car = car.uid;
          lane.front_gap = car_gap;
          lane.front_speed = car_speed;
        }
        else if (car_gap < fmin(lane.back_gap, lane_horizon)) {
          lane.back_car = car.uid;
          lane.back_gap = car_gap;
          lane.back_speed = car_speed;
        }

        // Evaluate feasibility
        lane.feasible =    (lane.front_gap > lane_change_front_buffer)
                        && (lane.back_gap > lane_change_back_buffer);
      }
    }

    #ifndef NDEBUG
    {
      auto & log_ = DEBUG_LOG;
      log_ << "LANE INFO" << endl;
      for(int i = 0; i < lane_info.size(); i++) {
        auto & l = lane_info[i];
        log_ << "  LANE " << i << ": ";
        if (l.is_clear())
          log_ << "CLEAR";
        else {
          log_ << (l.feasible ?  "FEASIBLE " : "         ");
          log_ << "[";
          log_ << fixed << setw(2) << setprecision(0);
          log_ << l.front_gap;
          if (isfinite(l.front_gap))
            log_ << "+" << mps2mph(l.front_speed) << "t";
          log_ << "; ";
          log_ << -l.back_gap;
          if (isfinite(l.back_gap))
            log_ << "+" << mps2mph(l.back_speed) << "t";
          log_ << "]";
        }
        log_ << endl;
      }
    }
    #endif
  }

  void PathPlanner::set_state(const telemetry_data & ego, STATE new_state) {
    if (state_ != new_state) {
      // Set the new state and the reference position
      state_ = new_state;
      state_s_ = ref_s;
    }
  }

  int PathPlanner::get_best_lane() const {

    if (lane_info[1].is_clear())
      return 1;

    vector<int> lanes(track.lane_count);
    iota(lanes.begin(), lanes.end(), 0);

    std::sort(lanes.begin(), lanes.end(), [&](int i, int j){
      auto & lane_i = lane_info[i];
      auto & lane_j = lane_info[j];
      
      auto i_closest_to_ref_lane = abs(i - ref_lane) <= abs(j - ref_lane);

      // prefer clear lanes
      if (lane_i.is_clear()) {
        if (lane_j.is_clear())
          return i_closest_to_ref_lane;
        else
          return true;
      } else if (lane_j.is_clear()) {
        return false;
      }


      const double i_v = lane_i.front_gap > lane_horizon 
                          ? INF
                          : lane_i.back_gap < lane_change_back_buffer
                            ? lane_i.back_speed
                            : lane_i.front_speed;
      const double j_v = lane_j.front_gap > lane_horizon 
                          ? INF
                          : lane_j.back_gap < lane_change_back_buffer
                            ? lane_j.back_speed
                            : lane_j.front_speed;

      if ((!isfinite(i_v) && !isfinite(j_v)) || fabs(i_v - j_v) < 0.5)
        return lane_i.front_gap >= lane_j.front_gap;
      else
        return i_v > j_v;
    });

    return lanes[0];
  }

  void PathPlanner::create_plan(const telemetry_data & ego, double dt) {
    DEBUG_LOG << "PLANNING" << endl;

    // Give it a safety margin
    const double road_speed_limit = mph2mps(track.speed_limit_mph) - 0.2;
    const double cte = (ref_d - track.lane_center(target_lane));

    if (state_ == STATE::START)
    {
      changing_lane = -1;
      target_lane = ref_lane;
      state_ = STATE::KL;
      state_s_ = ego_start_position.s;
    }

    int best_lane = get_best_lane();
    DEBUG_LOG << " * BEST LANE " << best_lane << endl;

    while (true)
    {
      double meters_in_state = ref_s - state_s_;
      while (meters_in_state < 0) 
        meters_in_state += roadmap.max_s;

      if (state_ == STATE::KL)
      {
        assert( target_lane == ref_lane );

        DEBUG_LOG << " * KEEPING LANE " << target_lane 
                  << " FOR " << setprecision(2) << meters2miles(meters_in_state) << " Miles"
                  << " (cte=" << setprecision(1) << setw(4) << cte << " m)" 
                  << endl;

        target_speed = road_speed_limit;
        
        // Evaluate lane change
        changing_lane = -1;

        // Evaluate lane changes if current lane is busy
        if (lane_info[target_lane].front_gap < lane_horizon
          && lane_info[target_lane].front_speed < road_speed_limit)
        {
          // Change if we are not in the best one
          if (lane_info[best_lane].front_speed > lane_info[target_lane].front_speed + 0.2) {
            changing_lane = best_lane;
            set_state(ego, STATE::PLC);
            continue;
          }
        }

        break;
      }

      else if (state_ == STATE::PLC)
      {
        assert( changing_lane != ref_lane );

        DEBUG_LOG << " * PREPARING CHANGE TO LANE " << changing_lane 
                  << " FOR " << setprecision(2) << meters2miles(meters_in_state) << " Miles"
                  << " (cte=" << setprecision(1) << setw(4) << cte << " m)" 
                  << endl;

        target_lane = ref_lane + ((changing_lane > ref_lane) ? 1 : -1);

        if (lane_info[target_lane].feasible)
        {
          set_state(ego, STATE::LC);
          continue;
        }
        else if (changing_lane != best_lane || meters_in_state > 500) 
        {
          // Waiting too much or not the best: cancel
          target_lane = ref_lane;
          set_state(ego, STATE::KL);
          continue;
        }
        else // Not feasible
        {
          if (lane_info[ref_lane].front_gap < 30) {
            // Can't perform the change ... try to slow down here
            if (lane_info[target_lane].back_gap < lane_change_back_buffer)
              target_speed = fmin(target_speed, lane_info[target_lane].back_speed - 0.5);
            else
              target_speed = fmin(target_speed, lane_info[target_lane].front_speed - 0.5);
          }
          // But wait in this lane
          target_lane = ref_lane;
        }

        break;
      }

      else if (state_ == STATE::LC)
      {
        if (ref_lane == target_lane && fabs(cte) < 0.2 && meters_in_state > 100)
        {
          // Lane change completed
          if (changing_lane >= 0 && changing_lane != ref_lane) {
            set_state(ego, STATE::PLC);
            continue;
          }

          changing_lane = -1;
          set_state(ego, STATE::KL);
          continue;
        }

        if (fmin(lane_info[target_lane].front_gap, lane_info[target_lane].back_gap) < 5)
        {
          // ABORT
          target_lane = ref_lane;
          DEBUG_LOG << " * ABORTING LANE CHANGE" << endl;
        }

        DEBUG_LOG << " * CHANGING TO LANE " << target_lane 
          << " FOR " << setprecision(2) << meters2miles(meters_in_state) << " Miles"
          << " (error=" << setprecision(1) << setw(4) << cte << " m)" 
          << endl;
        
        target_speed = road_speed_limit;
        break;
      }

      break;
    }

    // Avoid collisions
    if (lane_info[target_lane].front_gap < 30)
    {
      if (lane_info[target_lane].front_gap < 15)
        target_speed = fmin(target_speed, lane_info[target_lane].front_speed - 0.2);
      else
        target_speed = fmin(target_speed, lane_info[target_lane].front_speed);
      DEBUG_LOG << " * FOLLOWING THE LEAD (" << lane_info[target_lane].front_gap << " m)" << endl;
    }

    // Ensure the target speed is inside the limits
    // NOTE that we don't consider the possibility of moving backwards.
    target_speed = fmax(0.0, fmin(road_speed_limit, target_speed));

    // Adjust speed
    if (target_speed < ref_speed) {
      // deccelerate
      target_speed = fmax(target_speed, ref_speed - accel);
    }
    else if (target_speed > ref_speed) {
      // accelerate
      target_speed = fmin(target_speed, ref_speed + accel);
    }

    DEBUG_LOG << "TRAJECTORY" << endl
              << " * TARGET LANE " << target_lane << endl
              << " * TARGET SPEED " << setprecision(1) << mps2mph(target_speed)
              << endl;
  }


  void PathPlanner::build_path(const telemetry_data & ego,
                               const int target_lane,
                               const double target_speed,
                               path_t & path, double dt)
  {
    const auto target_d = track.safe_lane_center(target_lane);

    // trajectory points
    path_t anchors;

    // Build a path tangent to the previous end state
    anchors.append(ref_x_prev, ref_y_prev);
    anchors.append(ref_x, ref_y);

    // Add 3 more points spaced 30 m
    for(int i = 1; i <= 3; i++) {
      xy_t next_wp = roadmap.to_xy(ref_s + 30 * i, target_d);
      anchors.append(next_wp);
    }

    // change the points to the reference (ref_) coordinate
    for(int i = 0; i < anchors.size(); i++) {
      const auto dx = anchors.x[i] - ref_x;
      const auto dy = anchors.y[i] - ref_y;
      anchors.x[i] = dx * cos(-ref_yaw) - dy * sin(-ref_yaw);
      anchors.y[i] = dx * sin(-ref_yaw) + dy * cos(-ref_yaw);
    }

    // Interpolate the anchors with a cubic spline
    tk::spline spline;
    spline.set_points(anchors.x, anchors.y);

    // Now we can build the final trajectory...

    // Add previous path for continuity
    path.x.assign(ego.previous_path.x.begin(), ego.previous_path.x.end());
    path.y.assign(ego.previous_path.y.begin(), ego.previous_path.y.end());

    // distance = N * dt * speed

    // set a horizon of 30 m
    const double target_x = 30;
    const double target_y = spline(target_x);
    const double target_dist = norm(target_x, target_y);

    // t = N * dt = target_dist / target_speed
    // const double N = target_dist / (dt * target_speed);
    const double t = target_x / target_dist * dt;

    // sample the spline curve to reach the target speed
    for(int i = 1; i <= n_path_points - path.x.size(); i++) {
      double x__ = i * t * target_speed;
      double y__ = spline(x__);
      // transform back to world coordinates
      double x_ = x__ * cos(ref_yaw) - y__ * sin(ref_yaw) + ref_x;
      double y_ = x__ * sin(ref_yaw) + y__ * cos(ref_yaw) + ref_y;
      // append the trajectory points
      path.append({x_, y_});
    }
  }

} // ::carnd

} // ::(anonymous)
