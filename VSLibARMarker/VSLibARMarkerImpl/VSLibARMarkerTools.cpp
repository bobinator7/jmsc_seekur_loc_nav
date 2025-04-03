#include "../VSLibARMarkerTools.h"
#include "opencv2/calib3d.hpp"
#include <qthread.h>
#include "Plugin/VSLibOpenCV/VSLibOpenCVCommon.h"

using namespace VSLibARMarker;
using namespace cv;

//int Tools::detectMarkers(const cv::Mat& image, std::vector<std::vector<cv::Point2f> >& _corners, std::vector<int>& _ids)
int Tools::detectMarkers(const cv::Mat& image, OutputArrayOfArrays& corners, OutputArray& ids)
{
   Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_ARUCO_ORIGINAL);
   aruco::detectMarkers(image, dictionary, corners, ids);
   return corners.size().height;
}

VSM::Frame Tools::getMarkerRelFrame(const cv::Vec3d tvec, const cv::Vec3d rvec)
{
   cv::Mat mat;
   cv::Rodrigues(rvec,mat);
   //copy cv::Mat into VSM::Matrix3x3

   VSM::Matrix3x3 orientation = VSLibOpenCV::Common::vsmMatrix3x3FromCVMat(mat);
      //VSM::Matrix3x3( mat(0,2), mat(0,1), mat(0,0)
      //                                        , mat(1,2), mat(1,1), mat(1,0)
      //                                        , mat(2,2), mat(2,1), mat(2,0));
   //correct the yaw angle
   VSM::Matrix3x3 verosimOrientation = VSM::Matrix3x3(orientation.getRoll(), orientation.getPitch(), /*VSM::pi-*/orientation.getYaw(), true);
   //correct the axis-order of the translational vector according to VEROSIM coordinate system
   VSM::Vector3 translation = VSM::Vector3(tvec(0), tvec(1), tvec(2));
   //build frame from orientation and translation
   VSM::Frame markerRelFrame(verosimOrientation, translation);
   markerRelFrame = transformArucoToVerosim(markerRelFrame);
   return markerRelFrame;
}  

VSM::Frame Tools::getMarkerWorldFrame(const cv::Vec3d tvec, const cv::Vec3d rvec, const VSM::Frame& cameraWorldFrame)
{
   return cameraWorldFrame*getMarkerRelFrame(tvec, rvec);
}

 VSM::Frame Tools::getCameraWorldFrame(const cv::Vec3d tvec, const cv::Vec3d rvec, const VSM::Frame& markerWorldFrame)
{
   return markerWorldFrame*getMarkerRelFrame(tvec, rvec).inverse();
}


 VSM::Frame Tools::transformArucoToVerosim(const VSM::Frame& arucoFrame)
 {
    VSM::Frame correctionFrame(1, 0, 0, 0,   0, 0, 1, 0,   0, -1, 0, 0);
    VSM::Matrix3x3 rotMat(180, 0, 90);
    VSM::Frame rotationFrame(rotMat, VSM::Vector3(0, 0, 0));
    return correctionFrame*arucoFrame*rotationFrame;
 }