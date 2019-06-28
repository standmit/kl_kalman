/**
 * \file
 * \brief ROS wrapper for Kalman Filter
 * \author Andrey Stepanov
 * \version 0.1.1
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
#include "ros/node_handle.h"

namespace kl_kalman {

KalmanFilterROS::KalmanFilterROS():
		KalmanFilter(),
		nh("~kalman_filter")
{ }

KalmanFilterROS::KalmanFilterROS(const std::string name):
		KalmanFilter(),
		nh("~" + name)
{ }

const std::string& KalmanFilterROS::getNamespace() const {
	return nh.getNamespace();
}

void KalmanFilterROS::init() {
	loadEigen("X", X);
	loadEigen("P", P);
}

bool KalmanFilterROS::loadEigen(const std::string& param_name, matrix_type& e) {
	XmlRpc::XmlRpcValue matrix_xml;
	if (nh.getParam(param_name, matrix_xml)) {
		kl_kalman::Matrix matrix_msg;
		XmlRpcToMatrixMsg(matrix_xml, matrix_msg);
		MatrixMsgToEigen(matrix_msg, e);
		return true;
	} else {
		return false;
	}
}

}
