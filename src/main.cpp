#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <fstream>
#include <vector>
#include <chrono>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
const int ERR_ACC = 10080;
double Kp = 0.15;
double Ki = 0.0007;
double Kd = 6.0;
bool record_data = false;
bool use_twiddle = false;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}
// low pass filter to smoothen data
void lowPassFilter(double prev_steer, double &steer) {
	const double CUTOFF = 5;
	const double SAMPLE_RATE = 100;
	double RC = 1.0/(CUTOFF*2*M_PI);
	double dt = 1.0/SAMPLE_RATE;
	double alpha = dt/(RC+dt);
	
	steer = prev_steer + (alpha * (steer - prev_steer));
}
// weighted average to smoothen data
void weightedAvg(vector<double> &prev_steers, double &steer) {
	steer = prev_steers[0] * 0.2 + prev_steers[1] * 0.3 + prev_steers[2] * 0.5;
}
// twiddle to attempt to optimize pid values
void twiddle(vector<double> &dp, double &error, double &best_err, int &index, bool &retry){
	// Don't forget to call `make_robot` before every call of `run`!
	vector<double> p = {Kp, Ki, Kd};
	//dp = [0.001, 0.001, 0.001]

	if (error < best_err) {
		best_err = error;
		dp[index] *= 1.1;
		retry = false;
	} else {
		if (retry) {
			p[index] += dp[index];
			dp[index] *= 0.9;
			retry = false;
		} else {
			p[index] -= 2 * dp[index];
			retry = true;
		}
	}
	if (!retry) {
		index += 1;
		if (index > 2) index = 0;
		p[index] += dp[index];
	}
	Kp = p[0];
	Ki = p[1];
	Kd = p[2];
}

int main() {
  uWS::Hub h;
  const double MAX_SPEED = 50.0;
  const double MAX_THROTTLE = 1.0;
  const double MAX_STEERING = 1.0;
  vector<double> dp = {Kp*0.3, Ki*0.3, Kd*0.1};
  vector<double> prev_steers;
  double error = 0.0;
  double best_err = 10000000.0;
  double prev_steer = 0;
  bool retry = false;
  int count = 0, index = 0;
  PID steering;
  steering.Init(Kp,Ki,Kd,false);
  PID throttle;
  throttle.Init(1.0,0.0,0.0,false);

  /**
   * TODO: Initialize the pid variable.
   */
  std::cout << "Use Twiddle: " << use_twiddle << std::endl;
  std::ofstream fout;  // Create Object of Ofstream
  string filename = std::to_string((int)MAX_SPEED)+"-" + std::to_string(Kp) + "-" + std::to_string(Ki) + "-" + std::to_string(Kd) + ".txt";
  if (record_data)
	fout.open (filename,std::ios_base::app); // Append mode

  h.onMessage([&steering, &throttle, &MAX_SPEED, &fout, &count, &dp, &error, &best_err, 
				&retry, &index, &MAX_THROTTLE, &MAX_STEERING, &prev_steer, &prev_steers](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value, throttle_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
		  steering.UpdateError(cte);
		  steer_value = steering.TotalError();
          if (steer_value > MAX_STEERING) steer_value = MAX_STEERING;
          if (steer_value < -MAX_STEERING) steer_value = -MAX_STEERING;
		  // either low pass filter or weighted average is used here.
		  // if it is commented out, it means it is not used.
		  /*
		  if (count > 1) {
			  lowPassFilter(prev_steer, steer_value);
		  }
		  prev_steer = steer_value;
		  */
		  /*
		  prev_steers.push_back(steer_value);
		  if (count > 2) {
			  prev_steers.erase(prev_steers.begin());
			  weightedAvg(prev_steers, steer_value);
		  }
		  */
          // DEBUG
          //std::cout << "speed: " << speed << " Steering Value: " << steer_value << "cte: " << cte
          //          << std::endl;
					
		  throttle.UpdateError(speed-MAX_SPEED);
		  throttle_value = throttle.TotalError();
		  if (throttle_value >MAX_THROTTLE) throttle_value =MAX_THROTTLE;
          json msgJson;
          //msgJson["steering_angle"] = steer_value/factor/factor*2.236936*2.236936;
		  msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
		  // if we are recording data, it happens here
		  if(record_data)
			fout<< "cte:	" << cte << "	steer:	" << steer_value << "	angle:	" 
				<< angle << "	throt:	" << throttle_value << "	speed:	" << speed << std::endl;// Writing data to file
		  // when twiddle is turned on, the code is used below
		  if(use_twiddle) {
			  if(count > 500) {
				error += cte*cte/ERR_ACC;
				if(count%ERR_ACC == 0) {
					// first set of data is the control (default pid values)
					if(count == ERR_ACC) {
						best_err = error;
						error = 0.0;
						Kp += dp[index];
						steering.Init(Kp,Ki,Kd,true);
						std::cout << "dp: " << dp[0] << ", " << dp[1] << ", " << dp[2] << "  Retry: " << retry << "  Index: " << index << std::endl;
						std::cout << "New PID: " << Kp << ", " << Ki << ", " << Kd << std::endl;
						std::cout << "Error: " << best_err <<std::endl;
					// increment p, i, and d to minimize cte
					} else {
						std::cout << "Error: " << error << "     Best Error: " << best_err <<std::endl;
						twiddle(dp, error, best_err, index, retry);
						steering.Init(Kp,Ki,Kd,true);
						error = 0.0;
						std::cout << "dp: " << dp[0] << ", " << dp[1] << ", " << dp[2] << "  Retry: " << retry << "  Index: " << index << std::endl;
						std::cout << "New PID: " << Kp << ", " << Ki << ", " << Kd << std::endl;
						std::cout << "Error: " << best_err <<std::endl;
						if (dp[0]/Kp + dp[1]/Ki + dp[2]/Kd < 0.01) {
							std::cout << "Twiddle Complete" << std::endl;
							use_twiddle = false;
						}
					}
				}
			  }
		  }
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
		  count += 1;
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
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