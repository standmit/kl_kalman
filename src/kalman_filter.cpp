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

#include "kl_kalman/kalman_filter.h"

namespace kl_kalman {

KalmanFilter::KalmanFilter():
		X(),
		P()
{
	X.setZero();
	P.setZero();
}

void KalmanFilter::setState(const Eigen::MatrixXd& state, const Eigen::MatrixXd& covariance) {
	X = state;
	P = covariance;
}

void KalmanFilter::getState(Eigen::MatrixXd& state, Eigen::MatrixXd& covariance) const {
	state = X;
	covariance = P;
}

}