#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <stdlib.h>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;

	std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

	string line;
	double integral_term = 0;
	double ref_vel = 0;
	double prev_ref_accel = 0;
	int lane = 1;
	int no_lane_change_counter = 0; // No lane change in the first 5 seconds
	while (getline(in_map_, line)) {
		std::istringstream iss(line);
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




	h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
		&map_waypoints_dx, &map_waypoints_dy, &integral_term, &ref_vel, &lane, &prev_ref_accel, &no_lane_change_counter]
		(uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, 
			uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		double desired_vel = 49.5; //mph
		if (length && length > 2 && data[0] == '4' && data[1] == '2') {

			auto s = hasData(data);
			
			if (s != "") {
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry") {
					// j[1] is the data JSON object

					// Main car's localization Data
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];

					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values 
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side 
					//   of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];

					json msgJson;

					// Number of points in last suggested path
					int prev_size = previous_path_x.size();

					std::cout << "Number of points used:" << 15 - prev_size << std::endl;

					if (prev_size > 0)
					{
						car_s = end_path_s;
					}

					bool too_close = false;
					double speed_loss;
					bool right_lane_is_safe = true;
					bool right_right_lane_is_safe = true;
					bool left_lane_is_safe = true;
					bool left_left_lane_is_safe = true;

					double right_lane_speed_advantage = 9999.0;
					double right_right_lane_speed_advantage = 9999.0;
					double left_lane_speed_advantage = 9999.0;
					double left_left_lane_speed_advantage = 9999.0;
					// fid ref_v to use
					double gap_size = 20;
					for (int i = 0; i < sensor_fusion.size(); ++i)
					{
						float d = sensor_fusion[i][6];
						if (d < (2 + 4 * lane + 2) && d >(2 + 4 * lane - 2))
						{
							double vx = sensor_fusion[i][3];
							double vy = sensor_fusion[i][4];
							double check_speed = sqrt(vx * vx + vy * vy);
							double check_car_s = sensor_fusion[i][5];

							check_car_s += ((double)prev_size * 0.02 * check_speed);

							double delta_s = check_car_s - car_s;

							if (delta_s > 0)
							{
								double prel_desired_vel = check_speed*2.24 - (check_speed*2.24 / 25 ) * (25 - delta_s); //mph
								if (prel_desired_vel < desired_vel) {
									desired_vel = prel_desired_vel;
									too_close = true;
									speed_loss = 49.5 - desired_vel;
								}
							}
						}// Checking right lane


						 // if lane == 0
						if (lane == 0) {
							left_lane_is_safe = false;
							left_left_lane_is_safe = false;
							//CHECK RIGHT LANE
							if (d < (2 + 4 * (lane + 1) + 2) && d >(2 + 4 * (lane + 1) - 2))
							{
								double vx = sensor_fusion[i][3];
								double vy = sensor_fusion[i][4];
								double check_speed = sqrt(vx * vx + vy * vy);
								double check_car_s = sensor_fusion[i][5];
								check_car_s += ((double)prev_size * 0.02 * check_speed);
								double delta_s = check_car_s - car_s;
								double delta_v = check_speed - ref_vel; //mps
								double desired_speed_difference = (check_speed - desired_vel/2.24) ;
								if ((delta_s < 0 && delta_v > 0 && (3 * delta_v) > (-delta_s - gap_size)) || (delta_s < gap_size && delta_s > -gap_size)) // 3 seconds for lane change and 10 meters buffer
								{
									right_lane_is_safe = false;
									right_right_lane_is_safe = false;
								}
								double speed_advantage;
								if (delta_s > 0 && desired_speed_difference < 0) {
									speed_advantage = delta_s / (1-desired_speed_difference);
									if (speed_advantage < right_lane_speed_advantage)
										right_lane_speed_advantage = speed_advantage;
								}
								else if (delta_s > 0) {
									speed_advantage = (5+desired_speed_difference * 10);
									if (speed_advantage < right_lane_speed_advantage)
										right_lane_speed_advantage = speed_advantage;

								}
															}
							//CHECK RIGHT RIGHT LANE
							if (d < (2 + 4 * (lane + 2) + 2) && d >(2 + 4 * (lane + 2) - 2))
							{
								double vx = sensor_fusion[i][3];
								double vy = sensor_fusion[i][4];
								double check_speed = sqrt(vx * vx + vy * vy);
								double check_car_s = sensor_fusion[i][5];
								check_car_s += ((double)prev_size * 0.02 * check_speed);
								double delta_s = check_car_s - car_s;
								double delta_v = check_speed - ref_vel; //mps
								double desired_speed_difference = (check_speed - desired_vel/2.24);
								if ((delta_s < 0 && delta_v > 0 && (6 * delta_v) > (-delta_s - 2*gap_size)) || (delta_s < 2*gap_size && delta_s > - 2*gap_size)) // 6 seconds for two lane changes and 2*10 meters buffer
									right_right_lane_is_safe = false;
								double speed_advantage;
								if (delta_s > 0 && desired_speed_difference < 0) {
									speed_advantage = 0.5*delta_s / (1-desired_speed_difference);
									if (speed_advantage < right_right_lane_speed_advantage)
										right_right_lane_speed_advantage = speed_advantage;

								}
								else if (delta_s > 0) {
									speed_advantage = 5 + desired_speed_difference * 5;
									if (speed_advantage < right_right_lane_speed_advantage)
										right_right_lane_speed_advantage = speed_advantage;

								}
							}

						}

						else if(lane == 1) 
						{
							right_right_lane_is_safe = false;
							left_left_lane_is_safe = false;
							//CHECK RIGHT LANE
							if (d < (2 + 4 * (lane + 1) + 2) && d >(2 + 4 * (lane + 1) - 2))
							{
								double vx = sensor_fusion[i][3];
								double vy = sensor_fusion[i][4];
								double check_speed = sqrt(vx * vx + vy * vy);
								double check_car_s = sensor_fusion[i][5];
								check_car_s += ((double)prev_size * 0.02 * check_speed);
								double delta_s = check_car_s - car_s;
								double delta_v = check_speed - ref_vel; //mps
								double desired_speed_difference = (check_speed - desired_vel / 2.24);
								if ((delta_s < 0 && delta_v > 0 && (3 * delta_v) > (-delta_s - gap_size)) || (delta_s < gap_size && delta_s > -gap_size)) // 3 seconds for lane change and 10 meters buffer
									right_lane_is_safe = false;
								double speed_advantage;
								if (delta_s > 0 && desired_speed_difference < 0) {
									speed_advantage = delta_s / (1-desired_speed_difference);
									if (speed_advantage < right_lane_speed_advantage)
										right_lane_speed_advantage = speed_advantage;

								}
								else if (delta_s > 0) {
									speed_advantage = 5 + desired_speed_difference * 10;
									if (speed_advantage < right_lane_speed_advantage)
										right_lane_speed_advantage = speed_advantage;

								}

							}
							//CHECK LEFT LANE
							if (d < (2 + 4 * (lane - 1) + 2) && d >(2 + 4 * (lane - 1) - 2))
							{
								double vx = sensor_fusion[i][3];
								double vy = sensor_fusion[i][4];
								double check_speed = sqrt(vx * vx + vy * vy);
								double check_car_s = sensor_fusion[i][5];
								check_car_s += ((double)prev_size * 0.02 * check_speed);
								double delta_s = check_car_s - car_s;
								double delta_v = check_speed - ref_vel; //mps
								double desired_speed_difference = (check_speed - desired_vel / 2.24);
								if ((delta_s < 0 && delta_v > 0 && (3 * delta_v) > (-delta_s - gap_size)) || (delta_s < gap_size && delta_s > -gap_size)) // 3 seconds for lane change and 10 meters buffer
									left_lane_is_safe = false;
								double speed_advantage;
								if (delta_s > 0 && desired_speed_difference < 0) {
									speed_advantage = delta_s / (1-desired_speed_difference);
									if (speed_advantage < left_lane_speed_advantage)
										left_lane_speed_advantage = speed_advantage;

								}
								else if (delta_s > 0) {
									speed_advantage = 5 + desired_speed_difference * 10;
									if (speed_advantage < left_lane_speed_advantage)
										left_lane_speed_advantage = speed_advantage;
								}

							}
						}

						else if (lane == 2) {
							right_lane_is_safe = false;
							right_right_lane_is_safe = false;
							//CHECK LEFT LANE
							if (d < (2 + 4 * (lane - 1) + 2) && d >(2 + 4 * (lane - 1) - 2))
							{
								double vx = sensor_fusion[i][3];
								double vy = sensor_fusion[i][4];
								double check_speed = sqrt(vx * vx + vy * vy);
								double check_car_s = sensor_fusion[i][5];
								check_car_s += ((double)prev_size * 0.02 * check_speed);
								double delta_s = check_car_s - car_s;
								double delta_v = check_speed - ref_vel; //mps
								double desired_speed_difference = (check_speed - desired_vel / 2.24);
								if ((delta_s < 0 && delta_v > 0 && (3 * delta_v) > (-delta_s - gap_size)) || (delta_s < gap_size && delta_s > -gap_size)) // 3 seconds for lane change and 10 meters buffer
								{
									left_lane_is_safe = false;
									left_left_lane_is_safe = false;
								}
								double speed_advantage;
								if (delta_s > 0 && desired_speed_difference < 0) {
									speed_advantage = delta_s / (1-desired_speed_difference);
									if (speed_advantage < left_lane_speed_advantage)
										left_lane_speed_advantage = speed_advantage;

								}
								else if (delta_s > 0) {
									speed_advantage = 5 +desired_speed_difference * 10;
									if (speed_advantage < left_lane_speed_advantage)
										left_lane_speed_advantage = speed_advantage;

								}

							}
						
							//CHECK LEFT LEFT LANE
							if (d < (2 + 4 * (lane - 2) + 2) && d >(2 + 4 * (lane - 2) - 2))
							{
								double vx = sensor_fusion[i][3];
								double vy = sensor_fusion[i][4];
								double check_speed = sqrt(vx * vx + vy * vy);
								double check_car_s = sensor_fusion[i][5];
								check_car_s += ((double)prev_size * 0.02 * check_speed);
								double delta_s = check_car_s - car_s;
								double delta_v = check_speed - ref_vel; //mps
								double desired_speed_difference = (check_speed - desired_vel / 2.24);
								if ((delta_s < 0 && delta_v > 0 && (6 * delta_v) > (-delta_s - 2*gap_size)) || (delta_s < 2*gap_size && delta_s > -2*gap_size)) // 6 seconds for two lane changes and 2*10 meters buffer
									left_left_lane_is_safe = false;
								double speed_advantage;
								if (delta_s > 0 && desired_speed_difference < 0) {
									speed_advantage = 0.5*delta_s / (1-desired_speed_difference);
									if (speed_advantage < left_left_lane_speed_advantage)
										left_left_lane_speed_advantage = speed_advantage;

								}
								else if (delta_s > 0) {
									speed_advantage = 5 + desired_speed_difference * 50;
									if (speed_advantage < left_left_lane_speed_advantage)
										left_left_lane_speed_advantage = speed_advantage;

								}
							}
						}
					}

					double threshold = 10;
					vector<double> advantages{ right_lane_speed_advantage, left_lane_speed_advantage, right_right_lane_speed_advantage, left_left_lane_speed_advantage };
					vector<bool> safe{ right_lane_is_safe, left_lane_is_safe, right_right_lane_is_safe, left_left_lane_is_safe };
					vector<int> actions{ 1,-1,1,-1 };
					int cntr = 0;
					bool keep_going = true;
					std::cout << "too_close: " << too_close << std::endl;
					std::cout << "right_lane_speed_advantage: " << right_lane_speed_advantage << std::endl;
					std::cout << "left_lane_speed_advantage: " << left_lane_speed_advantage << std::endl;
					std::cout << "right_right_lane_speed_advantage: " << right_right_lane_speed_advantage << std::endl;
					std::cout << "left_left_lane_speed_advantage: " << left_left_lane_speed_advantage << std::endl;
					std::cout << "right_lane_is_safe: " << right_lane_is_safe << std::endl;
					std::cout << "left_lane_is_safe: " << left_lane_is_safe << std::endl;
					std::cout << "right_right_lane_is_safe: " << right_right_lane_is_safe << std::endl;
					std::cout << "left_left_lane_is_safe " << left_left_lane_is_safe << std::endl;
					//std::cout << "right_lane_speed_advantage: " << right_lane_speed_advantage << std::endl;
					//std::cout << "left_lane_speed_advantage: " << left_lane_speed_advantage << std::endl;
					//std::cout << "right_right_lane_speed_advantage: " << right_right_lane_speed_advantage << std::endl;
					


					if (no_lane_change_counter == 0){
						while (cntr < 4 && keep_going && too_close) {
							int maxElementIndex = std::max_element(advantages.begin(), advantages.end()) - advantages.begin();
							double advantage = advantages[maxElementIndex];
							if (advantage > threshold && safe[maxElementIndex]) 
							{
								lane += actions[maxElementIndex];
								keep_going = false;
								no_lane_change_counter = 150; //disables lane changes for the next 2 seconds
							}
							else if (advantage < threshold) 
							{
								keep_going = false;
							}
							else 
							{
								advantages[maxElementIndex] = 0;
							}
							++cntr;
						}
					}
					std::cout << "lane " << lane << std::endl;
					std::cout << std::endl;

					//double error = desired_vel - car_speed;
					//double time = (50 - prev_size) * 0.02;
					//double differential = error / time;
					//integral_term += time * error;
					//double kp = 0.9;
					//double ki = 0.1;
					//double kd = 0.01;
					//double gain = 3;
					
					//double ref_accel = gain*(kp * error + ki*integral_term + kd*differential);
					double ref_accel;
					prev_ref_accel = ref_accel;
					double max_accel = 1.5;
					double max_decel = 1.15;
					double max_jerk = 5;
					// 3 State Machine - Accelerate, Decelerate, Keep Speed
					if (no_lane_change_counter != 0) {
						max_accel *= (1 - no_lane_change_counter / 150.0);
						max_jerk *= (1 - no_lane_change_counter / 150.0);
						max_decel *= (1 - no_lane_change_counter / 150.0);
					}

					if (desired_vel-(ref_vel*2.24) > 0)
					{ //accelerate
						ref_accel = prev_ref_accel + max_jerk * 0.02;
						if (ref_accel > max_accel)
						{
							ref_accel = max_accel;
						}
					}
					else if (desired_vel - (ref_vel*2.24) < 0)
					{ //decellerate
						ref_accel = prev_ref_accel - max_jerk * 0.02;
						if (ref_accel < -(max_decel))
						{
							ref_accel = -max_decel;
						}
					}
					else 
					{
						//Keep current speed
						ref_accel = 0;
					}


					prev_ref_accel = ref_accel;
					//ref_vel = car_speed;
					//std::cout << "Current ref_vel vehicle Speed: " << ref_vel << std::endl;
					// Vector of widely spaced waypoints

					vector<double> ptsx;
					vector<double> ptsy;

					double ref_x = car_x;
					double ref_y = car_y;
					double ref_yaw = deg2rad(car_yaw);

					if (prev_size < 2)
					{
						double prev_car_x = car_x - cos(car_yaw);
						double prev_car_y = car_y - sin(car_yaw);
						ptsx.push_back(prev_car_x);
						ptsx.push_back(car_x);
						ptsy.push_back(prev_car_y);
						ptsy.push_back(car_y);
					}

					else
					{
						ref_x = previous_path_x[prev_size - 1];
						ref_y = previous_path_y[prev_size - 1];

						double ref_x_prev = previous_path_x[prev_size - 2];
						double ref_y_prev = previous_path_y[prev_size - 2];
						ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

						ptsx.push_back(ref_x_prev);
						ptsx.push_back(ref_x);
						ptsy.push_back(ref_y_prev);
						ptsy.push_back(ref_y);
					}

					double next_d = 2+4*lane;
					double target_x;
					if (no_lane_change_counter != 0)
						target_x = 30.0;
					else
						target_x = 20.0;

					for (int i = 1; i < 4; ++i) {
						double next_s = car_s + target_x * i;
						vector<double> next_xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
						ptsx.push_back(next_xy[0]);
						ptsy.push_back(next_xy[1]);
					}

					// Turn into vehicle coordinates
					for (int i = 0; i < ptsx.size(); ++i)
					{
						double shift_x = ptsx[i] - ref_x;
						double shift_y = ptsy[i] - ref_y;
						ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
						ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
					}
					std::cout << "Calculating spline" << std::endl;
					tk::spline s;
					s.set_points(ptsx, ptsy);
					std::cout << "Calculated spline" << std::endl;
					vector<double> next_x_vals;
					vector<double> next_y_vals;

					for (int i = 0; i < previous_path_x.size(); i++)
					{
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					}
					// Calculate how to break up spline points so that we travel at our desired reference velocity
					target_x = 5;
					double target_y = s(target_x);
					double target_dist = sqrt(target_x * target_x + target_y * target_y);

					double x_add_on = 0;
					std::cout << "Calculating points using spline" << std::endl;
					for (int i = 1; i <= 15 - previous_path_x.size(); ++i)
					{
						if (no_lane_change_counter > 0)
							no_lane_change_counter -= 1;
						ref_vel = ref_vel + ref_accel * 0.02*i;
						if (ref_accel > 0 && ref_vel > (desired_vel/2.24))
						{
							ref_vel = (desired_vel/2.24);
						}
						else if (ref_accel < 0 && ref_vel < (desired_vel/2.24))
						{
							ref_vel = (desired_vel/2.24);
						}

						double N = (target_dist / (0.02 * ref_vel)); // 2.2f turn mph to mps
						double x_point = x_add_on + (target_x) / N;
						double y_point = s(x_point);

						x_add_on = x_point;
						
						double x_ref = x_point;
						double y_ref = y_point;

						// Rotate back to global coordinates
						
						x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
						y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
						x_point += ref_x;
						y_point += ref_y;

						next_x_vals.push_back(x_point);
						next_y_vals.push_back(y_point);
					}
					std::cout << "Calculated points using spline" << std::endl;
					sleep(0.05);
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\"," + msgJson.dump() + "]";

					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}  // end "telemetry" if
			}
			else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}  // end websocket if
	}); // end h.onMessage

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
		char* message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	}
	else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}

	h.run();
}
