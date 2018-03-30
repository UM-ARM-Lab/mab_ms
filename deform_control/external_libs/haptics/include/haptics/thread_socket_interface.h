/* thread_socket_interface.h */

#include <stdlib.h>
#include <iostream>
#include <cstring>
#include <string>
#include <cstdio>
#include <vector>
#include <Eigen/Core>

#define IP "128.32.37.98"
#define RECEIVE_PORT 9000
#define SEND_PORT 9001

void parse(std::string buf, std::vector<std::string> &vect);

void connectionInit();

bool getDeviceState (Eigen::Vector3d& start_proxy_pos, Eigen::Matrix3d& start_proxy_rot, bool start_proxybutton[], Eigen::Vector3d& end_proxy_pos, Eigen::Matrix3d& end_proxy_rot, bool end_proxybutton[]);

void sendDeviceState (const Eigen::Vector3d& start_feedback_pos, bool start_feedback_enabled, const Eigen::Vector3d& end_feedback_pos, bool end_feedback_enabled);

void getDeviceState (double start_proxyxform[], bool start_proxybutton[], double end_proxyxform[], bool end_proxybutton[]);

void sendDeviceState (double start_feedback_pos[], bool start_feedback_enabled, double end_feedback_pos[], bool end_feedback_enabled);
