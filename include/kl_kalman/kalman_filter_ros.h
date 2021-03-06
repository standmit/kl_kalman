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

#ifndef KL_KALMAN_KALMAN_FILTER_ROS_H_
#define KL_KALMAN_KALMAN_FILTER_ROS_H_

#include <kl_kalman/matrix_msg_conversions.h>
#include "kl_kalman/kalman_filter.h"
#include <ros/node_handle.h>
#include <topic_tools/shape_shifter.h>
#include <ros_type_introspection/ros_introspection.hpp>

namespace kl_kalman {

/**
 * \brief Kalman Filter with ROS API
 */
class KalmanFilterROS: public KalmanFilter {
	public:
		/**
		 * \param name Filter's name. Used as namespace in parameters list
		 */
		KalmanFilterROS(const std::string name = "~kalman_filter");

		virtual ~KalmanFilterROS();

		/**
		 * \brief Get namespace of Kalman Filter
		 * \return Namespace of Kalman Filter
		 */
		const std::string& getNamespace() const;

		/**
		 * \brief	Initialise KalmanFilter from parameters. Call it after ros::init
		 *
		 * \param autostart		Start prediction and measurements after initialization
		 */
		void init(bool autostart = true);

		/**
		 * \brief	Start estimation
		 */
		void start();

		/**
		 * \brief	Stop estimation
		 */
		void stop();

	protected:
		ros::NodeHandle nh; ///< \link ros::NodeHandle NodeHandle \endlink for receiving parameters and publish/subscribe

		/**
		 * \brief 					Load Matrix from ParameterServer
		 * \param[in] param_name	Name of parameter
		 * \param[out] e			\link matrix_type Eigen matrix \endlink
		 * \return					True if parameter was found
		 */
		bool loadEigen(const std::string& param_name, matrix_type& e) const;

		/**
		 * \brief		Parse matrix from \link XmlRpc::XmlRpcValue XmlRpcValue \endlink element
		 * \param x		\link XmlRpc::XmlRpcValue XmlRpcValue \endlink element
		 * \return		\link matrix_type Eigen matrix \endlink
		 */
		static matrix_type loadEigen(const XmlRpc::XmlRpcValue& x);

		/**
		 * \brief	Callback to make prediction by timer
		 *
		 * \param te	A TimerEvent record contains desired timestamp
		 */
		void prediction_timer_cb(const ros::TimerEvent& te);

		/**
		 * \brief	Subscribe to topic
		 *
		 * \details
		 * Subscribe to topic of any type and point numbers of fields that we interested in
		 *
		 * \param topic_name	Topic name
		 * \param queue_size	Queue size
		 * \param interest		Sequence of indexes of fields that need to put in vector. Indexes order like in message definition
		 * \param callback		Pointer to callback that will proceed resulting vector
		 * \param obj			Pointer to instance contains callback
		 */
		template <class T>
		void subscribe(const std::string& topic_name, const uint32_t queue_size, const interest_type& interest, void (T::*callback)(const matrix_type), T* const obj);

		typedef boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> generic_callback; ///< generic callback
		std::vector<std::pair<ros::Subscriber, generic_callback> > subscribers;	///< Generic subscribers and its callbacks

		/**
		 * \brief	Convert message to Eigen vector contains fields pointed in interest
		 *
		 * \param msg	Message
		 * \param topic_name Topic name
		 * \param interest	Sequence of indexes of fields that need to put in vector. Indexes order like in message definition
		 */
		matrix_type parse_msg(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string& topic_name, const interest_type& interest);

	private:
		ros::Timer prediction_timer; ///< Timer for calling predict

		const std::string ns; ///< ROS namespace of instance

		RosIntrospection::Parser msg_parser; ///< Generic message parser
};

template <class T>
void KalmanFilterROS::subscribe(const std::string& topic_name, const uint32_t queue_size, const interest_type& interest, void (T::*callback)(const matrix_type), T* const obj) {
	subscribers.emplace_back(
		ros::Subscriber(),
		generic_callback()
	);
	subscribers.back().second = [interest, topic_name, callback, obj, this](const topic_tools::ShapeShifter::ConstPtr& msg) {
		(obj->*callback)(this->parse_msg(msg, topic_name, interest));
	};
	subscribers.back().first = nh.subscribe(topic_name, queue_size, subscribers.back().second);
}

}

#endif
