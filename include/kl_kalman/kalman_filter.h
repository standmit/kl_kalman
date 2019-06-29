/**
 * \file
 * \brief Kalman Filter
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

#ifndef KL_KALMAN_KALMAN_FILTER_H_
#define KL_KALMAN_KALMAN_FILTER_H_

#include "kl_kalman/types.h"
#include <vector>

/**
 * \details		All stuff refered to Kalman Filter described in this namespace
 */
namespace kl_kalman {

/// Kalman Filter base class
class KalmanFilter {
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
		void setState(const matrix_type& state, const matrix_type& covariance);

		/**
		 * \brief	Get current state of system
		 */
		void getState(matrix_type& state, matrix_type& covariance) const;

		/**
		 * \brief	Clear F matrix
		 * \details
		 * Delete all components of F matrx, so F matrix will be zero \n
		 * \f$ F = \begin{pmatrix} 0 & 0 & ... & 0 \\ 0 & 0 & ... & 0 \\ ... & ... & ... & ... \\ 0 & 0 & ... & 0 \end{pmatrix} \f$
		 */
		void clearF();

		/**
		 * \brief	Add component of F matrix and its power
		 * \details
		 * \f$ \sum_{i} {T_i (dt)^i} + T (dt)^{power} \f$
		 */
		void addFcomp(const matrix_type& T, const power_type power);

		/**
		 * \brief	Build F matrix
		 * \details
		 * F matrix will be built from components in Fcomp list
		 * \f$F = \sum \limits_{i} T_i (dt)^i\f$
		 */
		matrix_type getF(const double dt) const;

	protected:
		matrix_type X; ///< Object's state vector

		matrix_type P; ///< Matrix of object's state covariance

	private:
		/**
		 * \brief	Components of F matrix
		 * \details
		 * See getF for details
		 */
		composite_matrix Fcomp;
};


}

#endif
