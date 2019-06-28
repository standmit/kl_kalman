/**
 * \file
 * \brief Utilites for parsing Matrix messages from XmlRpc
 * \author Andrey Stepanov
 * \version 1.0.1
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

#include <kl_kalman/matrix_msg_conversions.h>

namespace kl_kalman {

void XmlRpcToMatrixMsg(XmlRpc::XmlRpcValue x, Matrix& m) {
	m.rows = (int)x["rows"];
	m.cols = (int)x["cols"];
	m.data.clear();
	XmlRpc::XmlRpcValue x_data = x["data"];
	for (int i = 0; i < x_data.size(); i++) {
		m.data.push_back((double)x_data[i]);
	}
}

void MatrixMsgToEigen(const Matrix& m, matrix_type& e) {
	e.resize(m.rows, m.cols);
	for (dimension r = 0; r < m.rows; r++)
		for (dimension c = 0; c < m.cols; c++)
			e(r,c) = m.data[r * m.cols + c];
}

}
