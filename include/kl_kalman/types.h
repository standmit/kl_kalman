/**
 * \file
 * \brief Common types of kl_kalman package
 * \author Andrey Stepanov
 * \version 1.1.0
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

#ifndef KL_KALMAN_TYPES_H_
#define KL_KALMAN_TYPES_H_

#include <eigen3/Eigen/Core>
#include <vector>

namespace kl_kalman {

typedef unsigned char dimension; ///< Dimensions count type

typedef double scalar; ///< Values type

typedef Eigen::Matrix<scalar, Eigen::Dynamic, Eigen::Dynamic> matrix_type; ///< Common type of matrix

typedef short int power_type; ///< Power of number

/**
 * \brief       Component of matrix with power
 * \details
 * Contains matrix and a power of <em>dt</em>: \n
 * \f$ [M, n] \Rightarrow M (dt)^n \f$
 */
typedef std::pair<matrix_type, power_type> matrix_component;

/**
 * \brief		List of components of composite matrix
 * \details
 * Contains list of elements ::matrix_component \n
 * \f$ [M_0, 0], [M_1, 1], ..., [M_n, n] \Rightarrow \sum \limits_{i=0}^{n} M_i (dt)^i \f$
 */
typedef std::vector<matrix_component> composite_matrix;

/**
 * \brief	Sequence of numbers
 * \details
 * Used to point field numbers in messages
 */
typedef std::vector<dimension> interest_type;

}

#endif
