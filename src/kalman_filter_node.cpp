/**
 * \file
 * \brief Kalman Filter ROS node
 * \author Andrey Stepanov
 * \version 0.1
 * \copyright Copyright (c) 2019 Andrey Stepanov \n
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * \n
 *     http://www.apache.org/licenses/LICENSE-2.0
 * \n
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "kl_kalman/kalman_filter_ros.h"
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
//#include "kl_kalman/multiarray_conversions.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "kalman_filter");
	ros::NodeHandle nh("~");
	XmlRpc::XmlRpcValue matrix;
	kl_kalman::KalmanFilterROS kalman_filter("kalman_filter");

	kalman_filter.init();
	Eigen::MatrixXd state;
	Eigen::MatrixXd covariance;
	kalman_filter.getState(state, covariance);
	ROS_INFO_STREAM("State" << std::endl << state << std::endl << "Covariance" << std::endl << covariance);

	state(2,0) = 5.;
	covariance(2,2) = 100.;
	kalman_filter.setState(state, covariance);
	Eigen::MatrixXd new_state;
	Eigen::MatrixXd new_covariance;
	kalman_filter.getState(new_state, new_covariance);
	ROS_INFO_STREAM("State" << std::endl << new_state << std::endl << "Covariance" << std::endl << new_covariance);

	return 0;
}
