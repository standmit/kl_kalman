/**
 * \file
 * \brief Kalman Filter
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

#ifndef KL_KALMAN_KALMAN_FILTER_H_
#define KL_KALMAN_KALMAN_FILTER_H_

#include <eigen3/Eigen/Core>
#include <vector>

/**
 * \details		All stuff refered to Kalman Filter described in this namespace
 */
namespace kl_kalman {

typedef unsigned char dimension; ///< Dimensions count type


/// Kalman Filter base class
class KalmanFilter {
	protected:
		Eigen::MatrixXd X; ///< Object's state vector
		Eigen::MatrixXd P; ///< Matrix of object's state covariance

	public:
		/**
		 * \brief		Empty constructor
		 * \details		Create instance of Kalman Filter with empty model (dimensions: 0)
		 * \warning		Dimensions must be defined before using such created filter.
		 */
		KalmanFilter();

		/**
		 * \brief				Set current state of the object
		 * \param state			State of the object
		 * \param covariance	Covariance of object's state
		 */
		void setState(const Eigen::MatrixXd& state, const Eigen::MatrixXd& covariance);

		/**
		 * \brief	Get current state of system
		 */
		void getState(Eigen::MatrixXd& state, Eigen::MatrixXd& covariance) const;
};


}

#endif
