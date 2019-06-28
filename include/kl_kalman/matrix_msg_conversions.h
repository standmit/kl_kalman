/**
 * \file
 * \brief Utilites for parsing Matrix messages from XmlRpc
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

#ifndef KL_KALMAN_MATRIX_MSG_CONVERSIONS_H_
#define KL_KALMAN_MATRIX_MSG_CONVERSIONS_H_

#include <xmlrpcpp/XmlRpcValue.h>
#include "kl_kalman/Matrix.h"
#include "kl_kalman/types.h"
#include <ros/console.h>

namespace kl_kalman {

/**
 * \brief			Convert XmlRpcValue to kl_kalman::Matrix message
 * \param[in] x		Input XmlRpcValue
 * \param[out] m	Output message
 */
void XmlRpcToMatrixMsg(XmlRpc::XmlRpcValue x, Matrix& m);

/**
 * \brief			Convert kl_kalman::Matrix message to Eigen matrix
 * \param[in] m		Input Matrix message
 * \param[out] e	Output Eigen matrix
 */
void MatrixMsgToEigen(const Matrix& m, matrix_type& e);

}

#endif
