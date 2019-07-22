/**
 * \file
 * \brief Kalman Filter ROS node
 * \author Andrey Stepanov
 * \version 0.4.0
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

#include <ros/ros.h>
#include <kl_kalman/kalman_filter_ros.h>

class TestKalmanFilterROS : public kl_kalman::KalmanFilterROS {
	public:
		TestKalmanFilterROS();
		void topic_callback(const kl_kalman::matrix_type vector);
		void test_subscribe(const std::string& topic_name, const kl_kalman::interest_type& interest);
};

TestKalmanFilterROS::TestKalmanFilterROS():
	kl_kalman::KalmanFilterROS()
{}

void TestKalmanFilterROS::topic_callback(const kl_kalman::matrix_type vector) {
	ROS_INFO_STREAM("Vector from topic:" << std::endl << vector << std::endl);
}

void TestKalmanFilterROS::test_subscribe(const std::string& topic_name, const kl_kalman::interest_type& interest) {
	subscribe(topic_name, 1, interest, &TestKalmanFilterROS::topic_callback, this);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "kalman_filter");
	ros::NodeHandle nh("~");

	TestKalmanFilterROS kalman_filter;

	kalman_filter.test_subscribe("test_topic", {1,2});

	ros::spin();

	return 0;
}
