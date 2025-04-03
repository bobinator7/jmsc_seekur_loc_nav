#ifndef VSLibARMarkerToolsH
#define VSLibARMarkerToolsH

#include "opencv/cv.h"
#include <opencv2/aruco.hpp>
#include "VSLibARMarkerExport.h"
#include <Lib/VSM/VSMFrame.h>
#include "VSLibARMarkerDatatypes.h"

namespace VSLibARMarker
{
   class VSLibARMarker_DECLSPEC Tools
   {
   public:
      //static int detectMarkers( const cv::Mat& image, std::vector<std::vector<cv::Point2f> >& corners, std::vector<int>& ids);
      static int detectMarkers(const cv::Mat& image, cv::OutputArrayOfArrays& corners, cv::OutputArray& ids);

      static VSM::Frame getMarkerRelFrame(const cv::Vec3d tvec, const cv::Vec3d rvec);

      static VSM::Frame getMarkerWorldFrame(const cv::Vec3d tvec, const cv::Vec3d rvec, const VSM::Frame& cameraWorldFrame);

      static VSM::Frame getCameraWorldFrame(const cv::Vec3d tvec, const cv::Vec3d rvec, const VSM::Frame& markerFrame);

      static VSM::Frame transformArucoToVerosim(const VSM::Frame& arucoFrame);
  };

};

#endif