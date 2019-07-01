/**
 * \file
 * \brief Kalman Filter ROS node
 * \author Andrey Stepanov
 * \version 0.3.0
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

kl_kalman::KalmanFilterROS* kalman_filter;

void publish_timer_cb(const ros::TimerEvent& te) {
	kl_kalman::matrix_type X = kalman_filter->getState();
	ROS_INFO("X = %1.1f; Vx = %1.1f", X(0,0), X(1,0));
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "kalman_filter");
	ros::NodeHandle nh("~");

	kalman_filter = new kl_kalman::KalmanFilterROS();
	kalman_filter->init();

	ros::Timer publish_timer = nh.createTimer(ros::Duration(1.), &publish_timer_cb);

	ros::spin();

	delete kalman_filter;

	return 0;
}
