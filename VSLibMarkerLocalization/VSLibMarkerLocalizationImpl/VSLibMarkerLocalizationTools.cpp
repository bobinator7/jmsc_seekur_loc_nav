#include "../VSLibMarkerLocalizationTools.h"

// VEROSIM
// #include "Lib/VSM/VSMVector3.h"
// #include "Lib/VSM/VSMVector4.h"
// #include "Lib/VSM/VSMVectorN.h"

// 3rd party


/* eigen matrix manipulation (begin) */
void VSLibMarkerLocalization::Tools::removeEigenMatRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove)
{
	unsigned int numRows = matrix.rows() - 1;
	unsigned int numCols = matrix.cols();

	if (rowToRemove < numRows)
		matrix.block(rowToRemove, 0, numRows - rowToRemove, numCols) = matrix.block(rowToRemove + 1, 0, numRows - rowToRemove, numCols);

	matrix.conservativeResize(numRows, numCols);
}

void VSLibMarkerLocalization::Tools::removeEigenMatColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove)
{
	unsigned int numRows = matrix.rows();
	unsigned int numCols = matrix.cols() - 1;

	if (colToRemove < numCols)
		matrix.block(0, colToRemove, numRows, numCols - colToRemove) = matrix.block(0, colToRemove + 1, numRows, numCols - colToRemove);

	matrix.conservativeResize(numRows, numCols);
}

void VSLibMarkerLocalization::Tools::fixBrokenCorrelationMatrix(Eigen::MatrixXd& matrix)
{
	unsigned int m = matrix.cols();
	unsigned int n = matrix.rows();

	// matrix not quadratic!
	if (n != m)
		return;

	//auto tmp = matrix.ldlt();
	//Eigen::VectorXd diag = tmp.vectorD();
	//for (int ii = 0; ii < diag.size(); ii++)
	//{
	//	if (diag(ii) < 0)
	//		diag(ii) = 0.0;
	//}
	//matrix = Eigen::MatrixXd(tmp.matrixL()) * diag.asDiagonal() * Eigen::MatrixXd(tmp.matrixU());

	matrix = (matrix + matrix.transpose()) * 0.5;

}

/* eigen matrix manipulation (end) */

void VSLibMarkerLocalization::Tools::stateWrapAngle(Eigen::VectorXd& vecIn)
{
	if (vecIn.size() % 3)
		return;

	int n = vecIn.size() / 3;
	for (int ii = 0; ii < n; ii++)
	{
		while (!(vecIn(3 * ii + 2) <= M_PI && vecIn(3 * ii + 2) > -M_PI))
		{
			if (vecIn(3 * ii + 2) > M_PI)
			{
				vecIn(3 * ii + 2) = vecIn(3 * ii + 2) - 2 * M_PI;
			}
			else if (vecIn(3 * ii + 2) < -M_PI)
			{
				vecIn(3 * ii + 2) = vecIn(3 * ii + 2) + 2 * M_PI;
			}
		}
	}
}

/* marker type conversion fcn (begin) */
std::unordered_map<int, VSM::PoseVector3Quaternion> VSLibMarkerLocalization::Tools::stdUnorderedMap3DFromVsm3DMarkerMat(VSM::MatrixNxM mat_in)
{
   std::unordered_map<int, VSM::PoseVector3Quaternion> marker_map;
   for (unsigned int ii = 0; ii < mat_in.columns(); ii++)
   {
      VSM::VectorN marker_cursor = mat_in.getColumn(ii);
      int id = static_cast<int>(marker_cursor.getElement(0));
      VSM::Vector3 position(marker_cursor.getElement(1), marker_cursor.getElement(2), marker_cursor.getElement(3));
      VSM::Vector4 orientation(marker_cursor.getElement(4), marker_cursor.getElement(5), marker_cursor.getElement(6), marker_cursor.getElement(7));

      VSM::PoseVector3Quaternion marker(position, orientation);

      marker_map[id] = marker;
   }

   return marker_map;
}

VSM::MatrixNxM VSLibMarkerLocalization::Tools::vsm3DMarkerMatFromStdUnorderedMap3D(std::unordered_map<int, VSM::PoseVector3Quaternion> map_in)
{
   VSM::MatrixNxM marker_matrix_out(8,map_in.size());
   int col = 0;
   for (const auto& cursor : map_in)
   {
      VSM::PoseVector3Quaternion marker_element = cursor.second;
      VSM::Vector3 pos = cursor.second.getPosition();
      VSM::Vector4 ori = cursor.second.getOrientation().getVector4();

      VSM::VectorN cursor_column = VSM::VectorN(8);
      cursor_column.setElement(0, cursor.first);
      cursor_column.setElement(1, pos[0]);
      cursor_column.setElement(2, pos[1]);
      cursor_column.setElement(3, pos[2]);
      cursor_column.setElement(4, ori[0]);
      cursor_column.setElement(5, ori[1]);
      cursor_column.setElement(6, ori[2]);
      cursor_column.setElement(7, ori[3]);
      marker_matrix_out.setColumn(col, cursor_column);

      col++;
   }

   return marker_matrix_out;
}

VSM::MatrixNxM VSLibMarkerLocalization::Tools::vsm2DMarkerMatFromStdUnorderedMap3D(std::unordered_map<int, VSM::PoseVector3Quaternion> map_in)
{
   VSM::MatrixNxM marker_matrix_out(4, map_in.size());
   int col = 0;
   for (const auto& cursor : map_in)
   {
      VSM::PoseVector3Quaternion marker_element = cursor.second;
      VSM::Vector3 pos = cursor.second.getPosition();

		// theta angle is determined by projection of the z axis of detected marker onto the xy plane
		//double theta = cursor.second.getOrientation().getMatrix3x3().getPitch();
		VSM::Matrix3x3 matRot = cursor.second.getOrientation().getMatrix3x3();
		VSM::Vector3 vecZ(0, 0, 1);
		VSM::Vector3 vecZProjected = matRot * vecZ;
		double theta = std::atan2(vecZProjected.getY(), vecZProjected.getX());

      VSM::Vector4 cursor_column(cursor.first, pos[0], pos[1], theta);
      marker_matrix_out.setColumn(col, cursor_column);

      col++;
   }

   return marker_matrix_out;
}

VSM::MatrixNxM VSLibMarkerLocalization::Tools::vsm2DMarkerMatFromStdUnorderedMap2D(std::unordered_map<int, VSM::Vector3> map_in)
{
   VSM::MatrixNxM marker_matrix_out(4,map_in.size());
   int col = 0;
   for (const auto& cursor : map_in)
   {
      VSM::Vector3 marker_element = cursor.second;

      VSM::Vector4 cursor_column(cursor.first, marker_element[0], marker_element[1], marker_element[2]);
      marker_matrix_out.setColumn(col, cursor_column);

      col++;
   }

   return marker_matrix_out;
}

/* marker type conversion fcn (end) */