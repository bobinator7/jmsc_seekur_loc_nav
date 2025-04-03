#ifndef VSLibMarkerLocalizationGridmapH
#define VSLibMarkerLocalizationGridmapH


#include "VSLibMarkerLocalizationExport.h"

// VEROSIM
#include "Lib/VSM/VSMFrame.h"
#include "Lib/VSM/VSMVector3.h"

// other
#include <set>
#include <mutex>
#include <unordered_set>

struct pairhash {
public:
	template <typename T, typename U>
	std::size_t operator()(const std::pair<T, U> &x) const
	{
		size_t seed = 0;
		seed ^= std::hash<T>()(x.first) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		seed ^= std::hash<U>()(x.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		return seed;
	}
};


namespace VSLibMarkerLocalization
{
	enum Cellstate { FIXED, STATIC, DYNAMIC };

	struct Gridcell { Cellstate state; float lo; bool occupied; };

   class VSLibMarkerLocalization_DECLSPEC Gridmap
   {
	public:
		// constructor / destructor
		Gridmap();
		Gridmap(size_t width, size_t height, float resolution, std::pair<double, double> offset);
		~Gridmap();

		void resetEmptyMap();

		// get / set
		void setRangesConversionParam(double angleOffset, double angleResolution, VSM::Frame sensorToBaseTransform);

		// export / import
		QImage exportMap();
		QImage exportCostmap();
		//QImage exportBinaryOccMap();
		void importMap(size_t width, size_t height, float res, std::pair<double, double> offset, std::vector<unsigned char> data);

		// occmap
		void updateMapFromScan(std::vector<double>& ranges, VSM::Vector3& robotPoseInWorldCoords);
		std::unordered_set<std::pair<size_t, size_t>, pairhash> getOccupiedCellSet();
		std::vector<std::pair<double, double>> getOccupiedCellPointCloudInWorldCoords();
		std::vector<std::pair<double, double>> getOccupiedCellPointCloudInRobotCoords(VSM::Vector3 robotPoseInWorldCoords, bool inVincinity = false);

		// costmap
		void generateCostMap(VSM::Vector3& robotPoseInWorldCoords, VSM::Vector3& goalPoseInWorldCoords);
		double getCostmapWeighting(VSM::Vector3& pose);
		double getCostmapWeighting(std::pair<size_t, size_t>& cell);
		double getCostmapWeighting(size_t& oneDimCellIndex);
		double getGradNF1(VSM::Vector3& pose);
		double getDeltaNF1(VSM::Vector3& pose);


	private:
		// map parameters
		size_t height_, width_;
		float res_;
		std::pair<double, double> mapOffset_;
		std::vector<Gridcell> data_;

		double priorProb_;
		double lowProb_;
		double highProb_;

		float occThresh_;
		std::unordered_set<std::pair<size_t, size_t>, pairhash> occupiedCellSet_;

		// sensor parameters / fcn
		VSM::Frame sensorToBaseTransform_;
		VSM::Vector3 sensorPoseInMapCoords_;
		double angleOffset_;
		double angleResolution_;
		double maxRange_;
		void updateSensorPose();

		// robot parameters
		VSM::Vector3 robotPoseInMapCoords_;

		// convert world coord / map coord (TODO: move to tools?)
		std::pair<double, double> pointfromMapIndex(std::pair<size_t, size_t>& index);
		std::vector<std::pair<double, double>> pointCloudInWordCoordsFromMapIndex(std::pair<size_t, size_t> index);
		std::vector<std::pair<double, double>> pointCloudInRobotCoordsFromMapIndex(std::pair<size_t, size_t> index, VSM::Vector3 robotPoseInWorldCoords);

		VSM::Vector3 poseInMapCoords(VSM::Vector3& poseInWorldCoords);
		VSM::Vector3 poseInWorldCoords(VSM::Vector3& poseInMapCoords);
		std::pair<size_t, size_t> mapIndexFromMapPose(VSM::Vector3& poseInMapCoords);
		std::pair<size_t, size_t> mapIndexFromWorldPose(VSM::Vector3& poseInWorldCoords);


		// sensor model subfunction
		double getBeamRangeFinderInvModelProb(std::pair<size_t, size_t>& cellIndex, std::vector<double>& rangesForCell);
		double getLogOddsPrior();
		std::vector<double> getRangesForCell(std::pair<size_t, size_t>& cellIndex, std::vector<double> ranges);

		double getPointDistToSensor(std::pair<double, double>& point);
		double getPointAngleToSensor(std::pair<double, double>& point);

		// costmap (consider using unordered_map for performance, hash implementation necessary!)
		size_t getOneDimIndex(std::pair<size_t, size_t>& cellIndex);
		size_t getOneDimIndex(size_t x, size_t y);
		std::unordered_set<std::pair<size_t, size_t>, pairhash> getInflatedOccupiedCellSet(double inflationRadius, std::unordered_set<std::pair<size_t, size_t>, pairhash>& occupied);
		std::vector<std::pair<size_t, size_t>> getMooreNeighbours(std::pair<size_t, size_t>& cell);

		void calcGoalDistCostmap(std::pair<size_t, size_t>& robotCell, std::pair<size_t, size_t>& goalCell, std::unordered_set<std::pair<size_t, size_t>, pairhash>& occupiedInflated);
		void calcObstacleCostmap(double obstacleSafetyDist, std::unordered_set<std::pair<size_t, size_t>, pairhash>& occupiedInflated);
		void calcPathDistCostmap(std::pair<size_t, size_t>& robotCell, std::pair<size_t, size_t>& goalCell);

		double calcCostmapWeighting(size_t& oneDimCellIndex);
		void calcWeightedCostmap();
		std::vector<float> costmapGoalDist_, costmapObstacle_, costmapPathDist_, costmap_;

		// nf1 function


		// concurrency handling
		std::mutex mCostmap_;

   };
} //end namespace VSLibMarkerLocalization

#endif 
