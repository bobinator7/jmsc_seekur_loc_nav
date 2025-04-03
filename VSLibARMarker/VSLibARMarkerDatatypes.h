#ifndef VSLibARMarkerDatatypesH
#define VSLibARMarkerDatatypesH

#include "VSLibARMarkerExport.h"
#include <opencv2/aruco.hpp>
#include "Plugin/VSLibOpenCV/VSLibOpenCVDatatypes.h"

namespace VSLibARMarker
{
   class Marker
   {
   public:
      Marker();
      Marker(const std::vector<cv::Point2f>& corners, const int& id);
      ~Marker();

      std::vector<cv::Point2f> getCorners();
      int getId();
      cv::Vec3d getTvec();
      cv::Vec3d getRvec();
      void setTvec(cv::Vec3d t);
      void setRvec(cv::Vec3d r);
      void asMatrix(cv::Mat& mat);

   protected:
      std::vector<cv::Point2f> corners;
      int id;
      cv::Vec3d tvec;
      cv::Vec3d rvec;
   };

   class MarkerDetections
   {
   public:
      MarkerDetections();
      MarkerDetections(const std::vector<std::vector<cv::Point2f> >& corners, std::vector<int>& ids);
      MarkerDetections(const MarkerDetections& otherMarkerDetections);
      ~MarkerDetections();

      Marker at(int i);
      void fromMatrix(cv::Mat_<float> mat);
      VSLibOpenCV::Mat asMatrix();
      bool empty();
      int size();
		void filterDetections(int edgeLengthThreshold, int minEdgeLengthDiff);
      bool estimateMarkerPoses(double markerEdgeLength, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);
      //void clear();

      //appends Marker to MarkerDetections (WARNING: existing rvecs and tvecs will be cleared!!!)
      void append(Marker marker);

   public:
      std::vector<std::vector<cv::Point2f> > corners;
      std::vector<int> ids;
      std::vector< cv::Vec3d > tvecs;
      std::vector< cv::Vec3d > rvecs;

   };


}
#endif //VSLibARMarkerDatatypesH
