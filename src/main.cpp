#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;
using Eigen::VectorXd;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int main() {
    uWS::Hub h;

    // MPC is initialized here!
    MPC mpc;

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);

        cout << "================================" << endl;
        std::cout << sdata << std::endl;

        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            string s = hasData(sdata);
            if (s != "") {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    vector<double> ptsx = j[1]["ptsx"];
                    vector<double> ptsy = j[1]["ptsy"];
                    double px = j[1]["x"];
                    double py = j[1]["y"];
                    double psi = j[1]["psi"];
                    double v = j[1]["speed"];
                    double delta = j[1]["steering_angle"];
                    double accel = j[1]["throttle"];
                    
                    vector<double> car_ref_ptsx;
                    vector<double> car_ref_ptsy;
                    for (size_t p=0; p < ptsx.size(); ++p) {
                        Point pt = transform_to_car_coord(ptsx[p], ptsy[p], px, py, psi);
                        car_ref_ptsx.push_back(pt.x);
                        car_ref_ptsy.push_back(pt.y);
                    }

                    Eigen::Map<Eigen::VectorXd> ptsx_v(&car_ref_ptsx[0], car_ref_ptsx.size());
                    Eigen::Map<Eigen::VectorXd> ptsy_v(&car_ref_ptsy[0], car_ref_ptsy.size());

                    auto coeffs = polyfit(ptsx_v, ptsy_v, 3);
                    double cte = polyeval(coeffs, 0);
                    double epsi = atan(3*coeffs(3)*px*px + 2*coeffs(2)*px + coeffs(1)) - MPC::normalize_Psi_To_Slope_Angle(psi);

                    // initial state
                    VectorXd state(6);
                    double x_lat = v * MPC::LATENCY_SECONDS;
                    double psi_lat = -delta*deg2rad(25) * v / MPC::Lf * MPC::LATENCY_SECONDS;
                    double a_lat = v + accel * MPC::LATENCY_SECONDS;
                    double cte_lat = cte + v * sin(epsi) * MPC::LATENCY_SECONDS;
                    state << x_lat,
                            0,
                            psi_lat,
                            a_lat,
                            cte_lat,
                            psi_lat - epsi;

                    MPC::Solution soln = mpc.Solve(state, coeffs);

                    cout << "x = " << soln.xv[1] << ", "
                         << "y = " << soln.yv[1] << ", "
                         << "psi = " << soln.psi << ", "
                         << "v = " << soln.vel << ", "
                         << "cte = " << soln.cte << ", "
                         << "epsi = " << soln.epsi << ", "
                         << "delta = " << soln.delta/deg2rad(25) << ", "
                         << "a = " << soln.acc << endl;

                    /**
                      * Calculate steering angle and throttle using MPC.
                      * Both are in between [-1, 1].
                      */

                    double steer_value = soln.delta/deg2rad(25);
                    double throttle_value = soln.acc;

                    json msgJson;
                    // NOTE: Remember to divide by deg2rad(25) before you send the
                    //   steering value back. Otherwise the values will be in between
                    //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle_value;

                    /**
                     * add (x,y) points to list here, points are in reference to
                     *   the vehicle's coordinate system the points in the simulator are
                     *   connected by a Green line
                     */

                    msgJson["mpc_x"] = soln.xv;
                    msgJson["mpc_y"] = soln.yv;

                    /**
                     * add (x,y) points to list here, points are in reference to
                     *   the vehicle's coordinate system the points in the simulator are
                     *   connected by a Yellow line
                     */

                    msgJson["next_x"] = car_ref_ptsx;
                    msgJson["next_y"] = car_ref_ptsy;


                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    std::cout << msg << std::endl;
                    // Latency
                    // The purpose is to mimic real driving conditions where
                    //   the car does actuate the commands instantly.
                    //
                    // Feel free to play around with this value but should be to drive
                    //   around the track with 100ms latency.
                    //
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
                    std::this_thread::sleep_for(std::chrono::milliseconds(long(MPC::LATENCY_SECONDS*1000)));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } else {
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