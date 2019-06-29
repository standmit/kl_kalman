/**
 * \file
 * \brief Utilites for parsing Matrix messages from XmlRpc
 * \author Andrey Stepanov
 * \version 2.0.1
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

namespace kl_kalman {

/**
 * \brief		Convert \link XmlRpc::XmlRpcValue XmlRpcValue \endlink to kl_kalman::Matrix message
 * \param x		Input \link XmlRpc::XmlRpcValue XmlRpcValue \endlink
 * \return		Matrix message
 */
Matrix XmlRpcToMatrixMsg(XmlRpc::XmlRpcValue x);

/**
 * \brief		Convert kl_kalman::Matrix message to \link matrix_type Eigen matrix \endlink
 * \param m		Input Matrix message
 * \return		\link matrix_type Eigen matrix \endlink
 */
matrix_type MatrixMsgToEigen(const Matrix& m);

}

#endif
