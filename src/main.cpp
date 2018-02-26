#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <stdlib.h>
#include <cfloat>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

const double LEFT_LANE = 0;
const double CENTER_LANE = 1;
const double RIGHT_LANE = 2;
const double LANE_LENGTH = 4;
const double LANE_RELATIVE_CENTER = 2;

double CAR_POINT_FREQUENCY = 0.02;  // The car visits a point every 0.02 seconds.

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(const string &s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");

	if (found_null != string::npos) {
        return "";
    }

    if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }

    return "";
}

double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1) * (y2 - y1));
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y) {

    auto closestLength = DBL_MAX; //large number
	int closestWaypoint = 0;

	for (unsigned short i = 0; i < maps_x.size(); i++) {
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLength) {
			closestLength = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y) {

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2(map_y - y, map_x - x);

	double angle = abs(theta - heading);

	if (angle > pi() / 4) {
		closestWaypoint++;
	}

	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x,
                         double y,
                         double theta,
                         vector<double> maps_x,
                         vector<double> maps_y)
{
	int next_waypoint = NextWaypoint(x, y, theta, maps_x, maps_y);
	int previous_waypoint = next_waypoint - 1;

    if (next_waypoint == 0) {
		previous_waypoint = static_cast<int>(maps_x.size() - 1);
	}

	double n_x = maps_x[next_waypoint] - maps_x[previous_waypoint];
	double n_y = maps_y[next_waypoint] - maps_y[previous_waypoint];
	double x_x = x - maps_x[previous_waypoint];
	double x_y = y - maps_y[previous_waypoint];

	// find the projection of x onto n
	double projection_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
	double projection_x = projection_norm * n_x;
	double projection_y = projection_norm * n_y;

	double frenet_d = distance(x_x, x_y, projection_x, projection_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000 - maps_x[previous_waypoint];
	double center_y = 2000 - maps_y[previous_waypoint];
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, projection_x, projection_y);

	if (centerToPos <= centerToRef) {
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for (int i = 0; i < previous_waypoint; i++)
	{
		frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
	}

	frenet_s += distance(0, 0, projection_x, projection_y);

	return {frenet_s, frenet_d};
}

// Transform from Frenet s, d coordinates to Cartesian x, y
vector<double> getXY(double s,
                     double d,
                     vector<double> maps_s,
                     vector<double> maps_x,
                     vector<double> maps_y) {
	int previous_waypoint = -1;

	while (s > maps_s[previous_waypoint + 1] && (previous_waypoint < (int)(maps_s.size() - 1)))
	{
		previous_waypoint++;
	}

	int waypoint2 = (previous_waypoint + 1) % maps_x.size();

	double heading = atan2((maps_y[waypoint2] - maps_y[previous_waypoint]), (maps_x[waypoint2] - maps_x[previous_waypoint]));
	// the x, y, s along the segment
	double seg_s = (s - maps_s[previous_waypoint]);

	double seg_x = maps_x[previous_waypoint] + seg_s * cos(heading);
	double seg_y = maps_y[previous_waypoint] + seg_s * sin(heading);

	double perp_heading = heading - pi() / 2;

	double x = seg_x + d * cos(perp_heading);
	double y = seg_y + d * sin(perp_heading);

	return {x, y};
}

vector<vector<double>> loadMapWaypoints() {
    // Load up map values for waypoint's x, y, s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;

        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;

        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }

    return {map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy, map_waypoints_s};
}

double getLaneCenterDCoordinate(int lane) {
    return LANE_RELATIVE_CENTER + LANE_LENGTH * lane;
}

bool isObjectInLane(int lane, double objectD) {
    return (objectD < LANE_RELATIVE_CENTER + LANE_LENGTH * lane + LANE_RELATIVE_CENTER) &&
            (objectD > LANE_RELATIVE_CENTER + LANE_LENGTH * lane - LANE_RELATIVE_CENTER);
}

double turnLeftCost(int lane,
                   double ego_car_s,
                   double ego_car_speed,
                   int previous_size,
                   vector<vector<double>> objects_in_left_lane,
                   vector<vector<double>> objects_in_center_lane,
                   vector<vector<double>> objects_in_right_lane) {
    if (lane == LEFT_LANE) {
        return 1000000.0;
    } else {
        vector<vector<double>> objects_in_target_lane;

        if (lane == CENTER_LANE) {
            objects_in_target_lane = objects_in_left_lane;
        } else {
            objects_in_target_lane = objects_in_center_lane;
        }

        if (objects_in_target_lane.size() == 0) {
            return 0;
        }

        double cost = 1000.0;
        for (int i = 0; i < objects_in_target_lane.size(); i++) {
            vector<double> detected_object = objects_in_target_lane[i];

            double object_velocity_x = detected_object[3];
            double object_velocity_y = detected_object[4];

            double object_speed = sqrt(object_velocity_x * object_velocity_x +
                                       object_velocity_y * object_velocity_y);
            double object_s = detected_object[5];

            // If we are using previous points we can project s value into the future
            object_s += (double) previous_size * CAR_POINT_FREQUENCY * object_speed;

            if (abs(object_s - ego_car_s) < 22.5) {
                return -1;
            }

            if (cost > abs(object_s - ego_car_s)) {
                cost = abs(object_s - ego_car_s);
            }
        }

        return 1.0 / (cost + objects_in_target_lane.size());
    }
}

double turnRightCost(int lane,
                    double ego_car_s,
                    double ego_car_speed,
                    int previous_size,
                    vector<vector<double>> objects_in_left_lane,
                    vector<vector<double>> objects_in_center_lane,
                    vector<vector<double>> objects_in_right_lane) {
    if (lane == RIGHT_LANE) {
        return 1000000.0;
    } else {
        vector<vector<double>> objects_in_target_lane;

        if (lane == CENTER_LANE) {
            objects_in_target_lane = objects_in_right_lane;
        } else {
            objects_in_target_lane = objects_in_center_lane;
        }

        if (objects_in_target_lane.size() == 0) {
            return 0;
        }

        double cost = 1000.0;
        for (int i = 0; i < objects_in_target_lane.size(); i++) {
            vector<double> detected_object = objects_in_target_lane[i];

            double object_velocity_x = detected_object[3];
            double object_velocity_y = detected_object[4];

            double object_speed = sqrt(object_velocity_x * object_velocity_x +
                                       object_velocity_y * object_velocity_y);
            double object_s = detected_object[5];

            // If we are using previous points we can project s value into the future
            object_s += (double) previous_size * CAR_POINT_FREQUENCY * object_speed;

            if (abs(object_s - ego_car_s) < 22.5) {
                return -1;
            }

            if (cost > abs(object_s - ego_car_s)) {
                cost = abs(object_s - ego_car_s);
            }
        }

        return 1.0 / (cost + objects_in_target_lane.size());
    }
}

int main() {
    uWS::Hub h;

    // Load up map values for waypoint's x, y, s and d normalized normal vectors
    vector<vector<double>> map_waypoints = loadMapWaypoints();
    vector<double> map_waypoints_x = map_waypoints[0];
    vector<double> map_waypoints_y = map_waypoints[1];
    vector<double> map_waypoints_dx = map_waypoints[2];
    vector<double> map_waypoints_dy = map_waypoints[3];
    vector<double> map_waypoints_s = map_waypoints[4];

    int lane = CENTER_LANE;
    double reference_velocity = 0;  //mph

  h.onMessage([&reference_velocity, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &lane]
                      (uWS::WebSocket<uWS::SERVER != 0u> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
      bool isWebSocketMessage = data[0] == '4';
      bool isWebSocketEvent = data[1] == '2';
      bool validLength = length && length > 2;

      if (validLength && isWebSocketMessage && isWebSocketEvent) {
          auto s = hasData(data);

          if (!s.empty()) {
              auto j = json::parse(s);

              string event = j[0].get<string>();

              if (event == "telemetry") {
                  // j[1] is the data JSON object
                  // Main car's localization Data

                  double ego_car_x = j[1]["x"];
                  double ego_car_y = j[1]["y"];
                  double ego_car_s = j[1]["s"];
                  double ego_car_d = j[1]["d"];
                  double ego_car_yaw = j[1]["yaw"];
                  double ego_car_speed = j[1]["speed"];

                  // Previous path data given to the Planne
                  auto previous_path_x = j[1]["previous_path_x"];
                  auto previous_path_y = j[1]["previous_path_y"];

                  // Previous path's end s and d values
                  double end_path_s = j[1]["end_path_s"];
                  double end_path_d = j[1]["end_path_d"];

                  // Sensor Fusion Data, a list of all other cars on the same side of the road.
                  auto sensor_fusion = j[1]["sensor_fusion"];

                  int previous_size = previous_path_x.size();

                  if (previous_size > 0) {
                      ego_car_s = end_path_s;  // Our car is at the end of the previous path
                  }

                  bool too_close = false;
                  bool car_in_my_lane = false;

                  vector<vector<double>> objects_in_left_lane = {};
                  vector<vector<double>> objects_in_center_lane = {};
                  vector<vector<double>> objects_in_right_lane = {};

                  for (int i = 0; i < sensor_fusion.size(); i++) {
                      vector<double> detected_object = sensor_fusion[i];
                      double object_d = detected_object[6];

                      for (int j = 0; j < 3; j++) {
                          bool is_my_lane = j == lane;
                          if (isObjectInLane(j, object_d)) {

                              if (j == LEFT_LANE) {
                                  objects_in_left_lane.push_back(detected_object);
                              } else if (j == CENTER_LANE) {
                                  objects_in_center_lane.push_back(detected_object);
                              } else {
                                  objects_in_right_lane.push_back(detected_object);
                              }

                              if (is_my_lane) {
                                  double object_velocity_x = detected_object[3];
                                  double object_velocity_y = detected_object[4];

                                  double object_speed = sqrt(object_velocity_x * object_velocity_x +
                                                             object_velocity_y * object_velocity_y);
                                  double object_s = detected_object[5];

                                  // If we are using previous points we can project s value into the future
                                  object_s += (double) previous_size * CAR_POINT_FREQUENCY * object_speed;

                                  double is_object_in_front_of_ego_car = object_s > ego_car_s;
                                  double is_object_in_front_of_ego_car_too_close = object_s - ego_car_s < 25; // Is the gap less than 25 meters?

                                  car_in_my_lane = is_object_in_front_of_ego_car && is_object_in_front_of_ego_car_too_close;
                                  if (is_object_in_front_of_ego_car && is_object_in_front_of_ego_car_too_close) {
                                      // Do some logic here. Lower reference velocity so we don't crash into car in front of us.
                                      // Could also flag to change lanes.
                                      too_close = true;

                                      double costOfTurningLeft = turnLeftCost(lane, ego_car_s, ego_car_speed, previous_size, objects_in_left_lane, objects_in_center_lane, objects_in_right_lane);
                                      double costOfTurningRight = turnRightCost(lane, ego_car_s, ego_car_speed, previous_size, objects_in_left_lane, objects_in_center_lane, objects_in_right_lane);

                                      cout << "cost of turning LEFT " << costOfTurningLeft << endl;
                                      cout << "cost of turning RIGHT " << costOfTurningRight << endl;


                                      if (costOfTurningLeft < costOfTurningRight) {
                                          if (costOfTurningLeft >= 0 && costOfTurningLeft <= 1) {
                                              lane -= 1;
                                          } else if (costOfTurningRight >= 0 && costOfTurningRight <= 1){
                                              lane += 1;
                                          }
                                      } else {
                                          if (costOfTurningRight >= 0 && costOfTurningRight <= 1) {
                                              lane += 1;
                                          } else if (costOfTurningLeft >= 0 && costOfTurningLeft <= 1){
                                              lane -= 1;
                                          }
                                      }
                                  }
                              }
                          }
                      }
                  }

//                  cout << "car in my lane " << car_in_my_lane << endl;
//                  cout << "too_close " << too_close << endl;
//
//                  if (car_in_my_lane) {
//                      // Do some logic here. Lower reference velocity so we don't crash into car in front of us.
//                      // Could also flag to change lanes.
//                      too_close = true;
//                      if (turnLeftCost(lane, previous_size, ego_car_s, ego_car_speed, objects_in_left_lane, objects_in_center_lane, objects_in_right_lane)) {
//                          lane -= 1;
//                      } else if (turnRightCost(lane, previous_size, ego_car_s, ego_car_speed, objects_in_left_lane, objects_in_center_lane, objects_in_right_lane)) {
//                          lane += 1;
//                      } else {
//                          too_close = true;
//                      }
//                  }

                  if (too_close) {
                      reference_velocity -= 0.300;
                      // reference_velocity -= 0.224;
                  } else if (reference_velocity < 49.5) {
                      reference_velocity += 0.300;
                  }

                  // Here we are creating a list of sample points that are evenly spaced (30 m)
                  // Later we'll spline points inside this sample to smooth our transition.
                  vector<double> points_x;
                  vector<double> points_y;

                  // Our reference values are the car's current position.
                  double reference_x = ego_car_x;
                  double reference_y = ego_car_y;
                  double reference_yaw = deg2rad(ego_car_yaw);

                  // If previous path is almost empty, let's just use the car's position as starting reference.
                  if (previous_size < 2) {
                      double previous_car_x = ego_car_x - cos(ego_car_yaw);
                      double previous_car_y = ego_car_y - sin(ego_car_yaw);

                      // Create two points that are tangent to the car's angle.
                      points_x.push_back(previous_car_x);
                      points_x.push_back(ego_car_x);

                      points_y.push_back(previous_car_y);
                      points_y.push_back(ego_car_y);

                  // Given the past has enough length, we'll use the last point of it as our starting reference.
                  } else {
                      reference_x = previous_path_x[previous_size - 1];
                      reference_y = previous_path_y[previous_size - 1];

                      double reference_x_prev = previous_path_x[previous_size - 2];
                      double reference_y_prev = previous_path_y[previous_size - 2];
                      reference_yaw = atan2(reference_y - reference_y_prev, reference_x - reference_x_prev);

                      // Add two points that are tangent to the car's angle.
                      points_x.push_back(reference_x_prev);
                      points_x.push_back(reference_x);

                      points_y.push_back(reference_y_prev);
                      points_y.push_back(reference_y);
                  }

                  // Next, we are going to add 3 evenly spaced points in Frenet's coordinates.
                  unsigned short SAMPLE_POINTS = 3;
                  unsigned short DISTANCE = 30;  // Meters

                  for (unsigned short i = 1; i <= SAMPLE_POINTS; i++) {
                      double waypoint_s = ego_car_s + DISTANCE * i;
                      double waypoint_d = getLaneCenterDCoordinate(lane);
                      vector<double> next_waypoint = getXY(waypoint_s, waypoint_d, map_waypoints_s, map_waypoints_x,
                                                           map_waypoints_y);

                      // Add waypoint to path
                      points_x.push_back(next_waypoint[0]);
                      points_y.push_back(next_waypoint[1]);
                  }

                  // Now we have 5 points in our path!

                  // Here we are shifting our coordinate system to car's POV, so the last point of the previous
                  // path is located at the origin (0, 0) and the car's angle is 0°.
                  for (unsigned short i = 0; i < points_x.size(); i++) {
                      double shift_x = points_x[i] - reference_x;
                      double shift_y = points_y[i] - reference_y;

                      points_x[i] = shift_x * cos(0 - reference_yaw) - shift_y * sin(0 - reference_yaw);
                      points_y[i] = shift_x * sin(0 - reference_yaw) + shift_y * cos(0 - reference_yaw);
                  }

                  // Create a spline
                  tk::spline spline;

                  // Set (x, y) points to the spline
                  spline.set_points(points_x, points_y);

                  // These are the actual points we are going to use to feed the simulator.
                  vector<double> next_x_values;
                  vector<double> next_y_values;

                  // Start with all of the previous path points from last time
                  for (unsigned short i = 0; i < previous_path_x.size(); i++) {
                      next_x_values.push_back(previous_path_x[i]);
                      next_y_values.push_back(previous_path_y[i]);
                  }

                  // Calculate how to space spline points so that we travel at our desired reference velocity
                  double target_x = 30;  // 30 meters. This is a horizon value
                  double target_y = spline(target_x);
                  double target_x_squared = target_x * target_x;
                  double target_y_squared = target_y * target_y;
                  double target_distance = sqrt(target_x_squared + target_y_squared);

                  double x_add_on = 0;

                  int remaining_points = 50 - previous_path_x.size();
                  for (unsigned short i = 1; i <= remaining_points; i++) {
                      double number_of_points_in_spline_to_reach_target =
                              target_distance / (CAR_POINT_FREQUENCY * reference_velocity / 2.24);  // 2.24 is used to transform to meters per second.
                      double x_point = x_add_on + target_x / number_of_points_in_spline_to_reach_target;
                      double y_point = spline(x_point);

                      x_add_on = x_point;

                      double x_reference = x_point;
                      double y_reference = y_point;

                      // Rotate back to normal after rotating it earlier. Reverts car's POV
                      x_point = x_reference * cos(reference_yaw) - y_reference * sin(reference_yaw);
                      y_point = x_reference * sin(reference_yaw) + y_reference * cos(reference_yaw);

                      x_point += reference_x;
                      y_point += reference_y;

                      next_x_values.push_back(x_point);
                      next_y_values.push_back(y_point);
                  }

                  json msgJson;

                  msgJson["next_x"] = next_x_values;
                  msgJson["next_y"] = next_y_values;

                  auto msg = "42[\"control\"," +  msgJson.dump() + "]";

                  //this_thread::sleep_for(chrono::milliseconds(1000));
                  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              }
          } else {
              // Manual driving
              std::string msg = "42[\"manual\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
      }
  });

    // We don't need this since we're not using HTTP but if it's removed the
    // program doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER != 0u> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER != 0u> ws, int code, char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
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