/**
 * \file
 * \brief ROS wrapper for Kalman Filter
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

#include "kl_kalman/kalman_filter_ros.h"
#include "ros/node_handle.h"
#include <xmlrpcpp/XmlRpcException.h>

namespace kl_kalman {

KalmanFilterROS::KalmanFilterROS(const std::string name):
		KalmanFilter(),
		nh(name)
{ }

KalmanFilterROS::~KalmanFilterROS() { }

const std::string& KalmanFilterROS::getNamespace() const {
	return nh.getNamespace();
}

void KalmanFilterROS::init(bool autostart) {
	autostart = autostart && loadEigen("X", X);
	autostart = autostart && loadEigen("P", P);
	clearF();
	XmlRpc::XmlRpcValue Farray;
	if (nh.getParam("F", Farray)) {
		try {
			for (unsigned char i = 0; i < Farray.size(); i++) {
				XmlRpc::XmlRpcValue matrix = Farray[i];
				addFcomp(loadEigen(matrix["T"]), (int)matrix["pow"]);
			}
		}
		catch (XmlRpc::XmlRpcException &e) {
			autostart = false;
			ROS_ERROR("Can't load F matrix components: %s", e.getMessage().c_str());
		}
	}
	prediction_timer = nh.createTimer(
		ros::Duration(
			ros::Rate(
				nh.param("prediction_rate", 100.)
			)
		),
		&KalmanFilterROS::prediction_timer_cb,
		this,
		false,		// oneshot
		autostart	// autostart
	);
}

bool KalmanFilterROS::loadEigen(const std::string& param_name, matrix_type& e) const {
	XmlRpc::XmlRpcValue matrix_xml;
	if (nh.getParam(param_name, matrix_xml)) {
		e = loadEigen(matrix_xml);
		return true;
	} else {
		return false;
	}
}

matrix_type KalmanFilterROS::loadEigen(const XmlRpc::XmlRpcValue& x) {
	return MatrixMsgToEigen(XmlRpcToMatrixMsg(x));
}

void KalmanFilterROS::prediction_timer_cb(const ros::TimerEvent& te) {
	predict((te.current_expected - te.last_expected).toSec());
}

void KalmanFilterROS::start() {
	prediction_timer.start();
}

void KalmanFilterROS::stop() {
	prediction_timer.stop();
}

matrix_type KalmanFilterROS::parse_msg(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string& topic_name, const interest_type& interest) {
	msg_parser.registerMessageDefinition(
		topic_name,
		RosIntrospection::ROSType(msg->getDataType()),
		msg->getMessageDefinition()
	);
	std::vector<uint8_t> buffer(msg->size());
	ros::serialization::OStream stream(buffer.data(), buffer.size());
	msg->write(stream);

	RosIntrospection::FlatMessage flat_container;
	msg_parser.deserializeIntoFlatContainer(
		topic_name,
		absl::Span<uint8_t>(buffer),
		&flat_container,
		100
	);

	RosIntrospection::RenamedValues renamed_value;
	msg_parser.applyNameTransform(
		topic_name,
		flat_container,
		&renamed_value
	);

	matrix_type result(interest.size(), 1);
	for (interest_type::size_type i = 0; i < interest.size(); i++)
		result(i,0) = renamed_value[interest[i]].second.convert<double>();

	return result;
}

}
