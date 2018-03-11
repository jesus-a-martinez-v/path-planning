# Path Planner Writeup

The main goal of this project was to implement a path planer module in C++ capable of generating drivable, safe and feasible routes for the vehicle to traverse. In order to accomplish these task, we are handed localization data, map data and sensor fusion data that indicates us where the other objects in the road are located (note that we refer to detected entities as "objects", not as "cars" because an unexpected obstacle might appear on the road).

Following we describe the steps and tasks completed in order to meet each project rubric's criteria (along with code snippets, where relevant).

## The car is able to drive at least 4.32 miles without incident.

For the car to be able to drive such a distance withouth having incidents, we generated paths comprised of waypoints such as:

	- Max acceleration and jerk wasn't exceeded.
	- The car always stayed at one particular lane, with the exception of changing lanes.
    - The car can change lanes when it is safe to do so.
    - The car does not goes against traffic.
    - The car doesn't surpasses speed limit.
    - The car tries to drive as close to the speed limit as possible, without breaking it.

 For a demonstration, please watch [this video]()

## The car drives according to the speed limit

Given that the speed limit is 50 mph, we set our reference velocity to 49.5 mph. This way we will progressively accelerate the car while the current velocity hasn't reached this threshold. In particular, the code in charge of controlling the acceleration and velocity is this:

```
                  if (too_close) {
                      reference_velocity -= 0.300;
                  } else if (reference_velocity < 49.5) {
                      reference_velocity += 0.300;
                  }

```

Variable `too_close` is a flag that indicates if we'are getting too close to a vehicle in front of us in the current lane. If that's the case, we decrease our reference velocity by 0.3 mph each time we receive a sensor fusion reading (0.02 seconds). Otherwise, we increase the reference velocity by the same amount while we haven't reached our threshold of 49.5 mph.

## Max acceleration and jerk are not exceeded

The key to having smooth transitions between waypoints is to generate many of them not too far from each other, so the car isn't required to accelerate beyond comfortable boundaries. The process for generating waypoints is as follows:

First, we must extract the X, Y, S , D, yar and speed values of the ego car and also the waypoints of the previous generated path:

```
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
```

We then check the position of other objects in the road in order to determine our reference velocity and try to change lanes if needed. For more details on this, please read the subsequent sections.

We'll now create some reference points that are evenly spaced (30 meters). Then, we'll fill the gaps between these points using a spline library.

We'll first add two points tangent's to the car angle. If the previous generated path has less than two points, we'll create them with the following code: 

```
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

```

If the previously generated path has enough length, then we'll use the last two points of it as our refernce for the new path:

```
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
```

Then, to complete 5 sample points we are going to add three evenly spaced points in Frenet's coordinates.

```
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
```

Of course, before adding them to the path, we are transforming them back to Cartesian coordinates.

In order to ease our lives and use the spline library to fill the gaps between these five reference points, we'll shift the coordinate system to the car's POV. This means that the car will be at the origin with an angle of 0°.

```
                  // Here we are shifting our coordinate system to car's POV, so the last point of the previous
                  // path is located at the origin (0, 0) and the car's angle is 0°.
                  for (unsigned short i = 0; i < points_x.size(); i++) {
                      double shift_x = points_x[i] - reference_x;
                      double shift_y = points_y[i] - reference_y;

                      points_x[i] = shift_x * cos(0 - reference_yaw) - shift_y * sin(0 - reference_yaw);
                      points_y[i] = shift_x * sin(0 - reference_yaw) + shift_y * cos(0 - reference_yaw);
                  }

```

We'll use a spline library to fill the gap.

```
                  // Create a spline
                  tk::spline spline;

                  // Set (x, y) points to the spline
                  spline.set_points(points_x, points_y);
```

Then, we'll calculate the number of points the spline must generate in order for the car to travel at our desired speed:

```
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
```

Notice we're adding first the five reference points to the path. Then we are using the spline to generate the remaining points and before adding them to the path we must rotate them back to the Cartesian reference of the map instead of the car's.


## Car does not have collisions

The car does not have collisions with the cars or objects in front of him by always trying to keep a minimum distance of 25 meters. If the ego vehicle detects a gap lesser or equal than 25 meters with an object in front of it, it starts to reduce its reference velocity and evaluates the possibility of changing lanes. For that matter, we first extract the data for the ego car:

```
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

                  ...
```

Then, we'll use two variables to indicate if there is a car in the lane of the ego vehicle and if that car is too close:

```
bool too_close = false;
bool car_in_my_lane = false;
```

Then we'll proceed to keep a list of all the detected objects in each of the three lanes (LEFT, CENTER and RIGHT lanes):

```
                  vector<vector<double>> objects_in_left_lane = {};
                  vector<vector<double>> objects_in_center_lane = {};
                  vector<vector<double>> objects_in_right_lane = {};
```

For each detected object by sensor fusion we'll determine in which lane it is. For that purpose we use this helper function:

```
bool isObjectInLane(int lane, double objectD) {
    return (objectD < LANE_RELATIVE_CENTER + LANE_LENGTH * lane + LANE_RELATIVE_CENTER) &&
            (objectD > LANE_RELATIVE_CENTER + LANE_LENGTH * lane - LANE_RELATIVE_CENTER);
}
```

This function rlies on Frenet's coordinates. We are checking if the detected object's D coordinate falls within the d values range of the provided lane number (0 is LEFT, 1 is CENTER and 2 is RIGHT).

After determining which lane the detected object is in, we then check if the lane we are checking is the lane the ego car is in. If that's the case, then we calculate the distance between the ego car and the detected object. For that we need to know if the car is in front of us and, if that's the case, how big the gap among us is. If the car is too close, we set `too_close = true;` and then calculate the cost of turning either left or right. For that purpose we use the two following cost functions:

```
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
```

These functions first look at the lane we're at and then determine what are our options depending on the turn they are evaluating. Let's take `turnRightCost` for instance. If we are at the center lane, we can switch to the right lane and we should consider only the objects detected in that lane. If we are in the left lane we can switch to the center lane and we should only consider the objects detected in that lane. But if we are in the right lane, then immediatly we know a right lane change is impossible because that would mean we would abandon the road. In order to flag invalid or impossible costs, we assume values either less than 0 or greater than 1 to be prohibitive. So, for instance, the cost of turning right when we are in the right lane is 1000000. Also, the cost of turning right when we are either in the left of center lane will be -1 if there are vehicles either 22.5 meters ahead or 22.5 meters behind in the target lane because by changing to that lane our ego vehicle could cause an accident. The cost of turning to an empty lane is a no brainer and thus its cost is 0. Finally, the cost of changing to a lane with vehicles in it will be dominated by the number of vehicles in it and the distance to the closest vehicle either in front or behind. 


## The car stays in its lane, except for the time between changing lanes

The path planner always generates waypoints based on the target lane it has. So, if the target lane is 1, the path planner will generate points with D Frenet's coordinate always within the range of lane 1.

## The car is able to change lane

As we saw above, the car evaluates the possibility of changing lanes only when it detects a slower vehicle in its current lane. For that purpose, it calculates the costs of changing either left or right and selects the option with the lowest cost. If changing lanes is doable, it sets its target lane to the corresponding number or index (remember, 0 = Left, 1 = Center, 2 = Right).

## There is a reflection on how to generate paths.

You're reading it ;)


# Improvements

There are lots of ways this path planner can be improved. In particular, there are situations where it doesn't generate the most optimal paths nor takes the best decisions. There are times where it choses lanes with more cars where other lanes with less or zero cars are available. Some times it waits too much before changing lanes. Other times it changes lanes when the gap isn't big enough, thus avoiding collisions just by a few meters.
