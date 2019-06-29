/**
 * \file
 * \brief Kalman Filter ROS node
 * \author Andrey Stepanov
 * \version 0.2.0
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

	kl_kalman::KalmanFilterROS kalman_filter;
	kalman_filter.init();

	ROS_INFO_STREAM(" Matrix F" << std::endl << kalman_filter.getF(2.));

	return 0;
}
