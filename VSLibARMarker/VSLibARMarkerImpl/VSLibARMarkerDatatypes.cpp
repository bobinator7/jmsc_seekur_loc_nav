#include "../VSLibARMarkerDatatypes.h"
#include <opencv2/highgui.hpp>
#include "Plugin/VSLibOpenCV/VSLibOpenCVCommon.h"

namespace VSLibARMarker {

   Marker::Marker()
   {
      cv::redirectError(VSLibOpenCV::Common::OpenCVErrorCallback);
   }

   Marker::Marker(const std::vector<cv::Point2f>& markerCorners, const int& id)
      :corners(markerCorners)
      , id(id)
   {
   }

   Marker::~Marker()
   {
   }

   std::vector<cv::Point2f> Marker::getCorners()
   {
      return corners;
   }

   int Marker::getId()
   {
      return id;
   }

   cv::Vec3d Marker::getTvec()
   {
      return tvec;
   }
   
   cv::Vec3d Marker::getRvec()
   {
      return rvec;
   }

   void Marker::setTvec(cv::Vec3d t)
   {
      tvec = t;
   }

   void Marker::setRvec(cv::Vec3d r)
   {
      rvec = r;
   }

   void Marker::asMatrix(cv::Mat& mat)
   {
      mat.create(5, 2, CV_32FC1);
      mat.at<float>(0, 0) = id;
      for (int i = 1; i < 5; ++i)
      {
         mat.at<float>(i, 0) = corners.at(i-1).x;
         mat.at<float>(i, 1) = corners.at(i-1).y;
      }
   }


   MarkerDetections::MarkerDetections()
   {
   }

   MarkerDetections::MarkerDetections(const std::vector<std::vector<cv::Point2f> >& corners, std::vector<int>& ids)
      : corners(corners)
      , ids(ids)
   {
   }

   MarkerDetections::MarkerDetections(const MarkerDetections& otherMarkerDetections)
      : corners(otherMarkerDetections.corners)
      , ids(otherMarkerDetections.ids)
      , tvecs(otherMarkerDetections.tvecs)
      , rvecs(otherMarkerDetections.rvecs)
   {
   }

   MarkerDetections::~MarkerDetections()
   {
      ids.clear();
      for (int i = 0; i < corners.size(); ++i)
      {
         corners.at(i).clear();
      }
      corners.clear();
   }

   Marker MarkerDetections::at(int i)
   {
      Marker marker(corners.at(i), ids.at(i));
      if (!tvecs.empty())
      {
         marker.setTvec(tvecs.at(i));
      }
      if (!rvecs.empty())
      {
         marker.setRvec(rvecs.at(i));
      }
      return marker;
   }

   void MarkerDetections::fromMatrix(cv::Mat_<float> mat)
   {
      corners.clear();
      ids.clear();
      tvecs.clear();
      rvecs.clear();
      if (mat.empty())
         return;
      int size = mat.cols / 2;
      for (int m = 0; m < size; ++m)
      {
         cv::Mat_<float> subMat = mat.colRange(m * 2, m * 2 + 2);
         std::vector<cv::Point2f> markerCorners;
         for (int c = 1; c < 5; ++c)
         {
            cv::Point2f p(subMat(c, 0), subMat(c, 1));
            markerCorners.push_back(p);
         }
         ids.push_back(subMat.at<float>(0, 0));
         corners.push_back(markerCorners);
      }
   }

   VSLibOpenCV::Mat MarkerDetections::asMatrix()
   {
      cv::Mat mat;
      mat.create(5, 2 * corners.size(), CV_32FC1);
      for (int m = 0; m < corners.size(); ++m)
      {
         cv::Mat subMat = mat.colRange(m * 2, m * 2 + 2);
         cv::Mat markerMat;
         at(m).asMatrix(markerMat);
         markerMat.copyTo(subMat);
      }
      return mat;
   }

   bool MarkerDetections::empty()
   {
      return corners.empty();
   }

   int MarkerDetections::size()
   {
      return corners.size();
   }

	void MarkerDetections::filterDetections(int edgeLengthThreshold, int minEdgeLengthDiff)
	{
		std::vector<unsigned int> invalidDetectionIndices;
		for (unsigned int ii = 0; ii < corners.size(); ii++)
		{
			std::vector<cv::Point2f> singleMarkerCorners = corners.at(ii);

			double minEdgeLength = HUGE_VAL;
			std::vector<double> edgeLengths;
			for (unsigned int jj = 0; jj < singleMarkerCorners.size(); jj++)
			{
				unsigned int nextIndex = jj + 1;
				if (nextIndex >= singleMarkerCorners.size())
					nextIndex = 0;

				double dist = cv::norm(singleMarkerCorners.at(jj) - singleMarkerCorners.at(nextIndex));
				edgeLengths.push_back(dist);
				if (dist < minEdgeLength)
					minEdgeLength = dist;
			}

			// check minimal edge length
			if (minEdgeLength < edgeLengthThreshold)
			{
				invalidDetectionIndices.push_back(ii);
				continue;
			}
		
			// check if edges have same length
			if (abs(edgeLengths[0] - edgeLengths[2]) + abs(edgeLengths[1] - edgeLengths[3]) < minEdgeLengthDiff)
			{
				invalidDetectionIndices.push_back(ii);
				continue;
			}
		}

		for (unsigned int kk = invalidDetectionIndices.size(); kk-- > 0; )
		{
			ids.erase(ids.begin() + invalidDetectionIndices.at(kk));
			corners.erase(corners.begin() + invalidDetectionIndices.at(kk));
			if (!rvecs.empty())
				rvecs.erase(rvecs.begin() + invalidDetectionIndices.at(kk));
			if (!tvecs.empty())
				tvecs.erase(tvecs.begin() + invalidDetectionIndices.at(kk));
		}
	}

   bool MarkerDetections::estimateMarkerPoses(double markerEdgeLength, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs)
   {
      cv::aruco::estimatePoseSingleMarkers(corners, markerEdgeLength, cameraMatrix, distCoeffs, rvecs, tvecs);
      if (corners.size() == rvecs.size() && corners.size() == tvecs.size())
         return true;
      else
         return false;
   }

   //void MarkerDetections::clear()
   //{
   //   std::vector<std::vector<cv::Point2f> >().swap(corners);
   //   //corners.clear();
   //   std::vector<int>().swap(ids);
   //   //ids.clear();
   //   //if (tvecs.size()>0)
   //   //   std::vector< cv::Vec3d >().swap(tvecs);
   //   ////tvecs.clear();
   //   //if (rvecs.size()>0)
   //   //   std::vector< cv::Vec3d >().swap(rvecs);
   //   ////rvecs.clear();
   //}

   //appends Marker to MarkerDetections (WARNING: existing rvecs and tvecs will be cleared!!!)
   void MarkerDetections::append(Marker marker)
   {
      corners.push_back(marker.getCorners());
      ids.push_back(marker.getId());
      if (!tvecs.empty())
         tvecs.clear();
      if (!rvecs.empty())
         rvecs.clear();
   }


}//end namespace