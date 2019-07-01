/**
 * \file
 * \brief Kalman Filter
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

#include "kl_kalman/kalman_filter.h"

namespace kl_kalman {

KalmanFilter::KalmanFilter():
		X(),
		P(),
		Fcomp()
{
	X.setZero();
	P.setZero();
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::setState(const matrix_type& state, const matrix_type& covariance) {
	X = state;
	P = covariance;
}

matrix_type KalmanFilter::getState() const {
	return X;
}

void KalmanFilter::getState(matrix_type& state, matrix_type& covariance) const {
	state = X;
	covariance = P;
}

matrix_type KalmanFilter::getF(const double dt) {
	matrix_type F(P.rows(), P.cols());
	F.setZero();
	for (composite_matrix::const_iterator comp = Fcomp.begin(); comp != Fcomp.end(); comp++)
		F += comp->first * pow(dt, comp->second);
	return F;
}

void KalmanFilter::clearF() {
	Fcomp.clear();
}

void KalmanFilter::addFcomp(const matrix_type& Fi, const power_type power) {
	Fcomp.emplace_back(Fi, power);
}

void KalmanFilter::predict(const double dt) {
	matrix_type F = getF(dt);
	X = F * X;
	P = F * P * F.transpose();
}

}
