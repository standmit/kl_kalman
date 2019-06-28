/**
 * \file
 * \brief Common types of kl_kalman package
 * \author Andrey Stepanov
 * \version 1.0.0
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

namespace kl_kalman {

typedef unsigned char dimension; ///< Dimensions count type

typedef Eigen::MatrixXd matrix_type; ///< Common type of matrix

}

#endif
