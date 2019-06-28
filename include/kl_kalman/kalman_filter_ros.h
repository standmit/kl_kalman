/**
 * \file
 * \brief ROS wrapper for Kalman Filter
 * \author Andrey Stepanov
 * \version 0.1.2
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

namespace kl_kalman {

/**
 * \brief Kalman Filter with ROS API
 */
class KalmanFilterROS: public KalmanFilter {
	public:
		/**
		 * \brief Constructor with standard namespace 'kalman_filter'
		 */
		KalmanFilterROS();


		/**
		 * \param name Filter's name. Used as namespace in parameters list
		 */
		KalmanFilterROS(const std::string name);

		/**
		 * \brief Get namespace of Kalman Filter
		 * \return Namespace of Kalman Filter
		 */
		const std::string& getNamespace() const;

		/**
		 * \brief	Initialise KalmanFilter from parameters. Call it after ros::init
		 */
		void init();

	protected:
		const ros::NodeHandle nh; ///< NodeHandle for receiving parameters and publish/subscribe

		/**
		 * \brief 					Load Matrix from ParameterServer
		 * \param[in] param_name	Name of parameter
		 * \param[out] e			Eigen matrix
		 */
		bool loadEigen(const std::string& param_name, matrix_type& e);
};

}

#endif
