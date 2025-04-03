#include "../VSLibMarkerLocalizationGridmap.h"

// VEROSIM
#include "../VSLibMarkerLocalizationTools.hpp"

// 3rd party
#include <unordered_map>

namespace VSLibMarkerLocalization
{
	Gridmap::Gridmap()
	{
		priorProb_ = 0.5;
		lowProb_ = 0.3;
		highProb_ = 0.7;

		occThresh_ = 0.75;

		setRangesConversionParam(0.0, 0.0, VSM::Frame::getIdentity());
		maxRange_ = 50.0;
	}

	Gridmap::~Gridmap()
	{
	}

	Gridmap::Gridmap(size_t width, size_t height, float resolution, std::pair<double, double> offset)
	{
		width_ = width;
		height_ = height;
		res_ = resolution;
		mapOffset_ = offset;

		priorProb_ = 0.5;
		lowProb_ = 0.3;
		highProb_ = 0.7;

		occThresh_ = 0.75;

		setRangesConversionParam(0.0, 0.0, VSM::Frame::getIdentity());
		maxRange_ = 50.0;

		costmapGoalDist_ = std::vector<float>(width_ * height_, 0.0);
		costmapObstacle_ = std::vector<float>(width_ * height_, 0.0);
		costmapPathDist_ = std::vector<float>(width_ * height_, 0.0);
		costmap_ = std::vector<float>(width_ * height_, 0.0);

		occupiedCellSet_ = std::unordered_set<std::pair<size_t, size_t>, pairhash>();
	}

	void Gridmap::resetEmptyMap()
	{
		std::lock_guard<std::mutex> lock(mCostmap_);

		data_.clear();
		float maxVal = std::hypot(width_, height_);
		std::fill(costmapGoalDist_.begin(), costmapGoalDist_.end(), maxVal); //costmapGoalDist_ = std::vector<float>(width_ * height_, maxVal);
		std::fill(costmapObstacle_.begin(), costmapObstacle_.end(), 0.0); //costmapObstacle_ = std::vector<float>(width_ * height_, 0.0);
		std::fill(costmapPathDist_.begin(), costmapPathDist_.end(), maxVal); //costmapPathDist_ = std::vector<float>(width_ * height_, maxVal);
		std::fill(costmap_.begin(), costmap_.end(), 0.0); //costmap_ = std::vector<float>(width_ * height_, 0.0);
		occupiedCellSet_.clear();

		// define empty cell
		Gridcell emptyCell;
		emptyCell.state = STATIC;
		emptyCell.lo = getLogOddsFromProb(lowProb_);
		emptyCell.occupied = false;

		// fill data vector with empty cells
		for (int ii = 0; ii < height_; ii++)
		{
			for (int jj = 0; jj < width_; jj++)
			{
				data_.push_back(emptyCell);
			}
		}
	}

	void Gridmap::setRangesConversionParam(double angleOffset, double angleResolution, VSM::Frame sensorToBaseTransform)
	{
		angleOffset_ = angleOffset;
		angleResolution_ = angleResolution;

		sensorToBaseTransform_ = sensorToBaseTransform;
		updateSensorPose();
	}

	void Gridmap::updateSensorPose()
	{
		sensorPoseInMapCoords_.x = sensorToBaseTransform_.getElement(0, 3) * std::cos(robotPoseInMapCoords_.z)
			- sensorToBaseTransform_.getElement(1, 3) * std::sin(robotPoseInMapCoords_.z)
			+ robotPoseInMapCoords_.x;
		sensorPoseInMapCoords_.y = sensorToBaseTransform_.getElement(0, 3) * std::sin(robotPoseInMapCoords_.z)
			+ sensorToBaseTransform_.getElement(1, 3) * std::cos(robotPoseInMapCoords_.z)
			+ robotPoseInMapCoords_.y;
		sensorPoseInMapCoords_.z = wrapAngle(std::acos(sensorToBaseTransform_.getElement(0, 0)) + robotPoseInMapCoords_.z + M_PI_2);
	}

	void Gridmap::importMap(size_t width, size_t height, float res, std::pair<double, double> offset, std::vector<unsigned char> data)
	{
		width_ = width;
		height_ = height;
		res_ = res;
		mapOffset_ = offset;

		// clear old data
		data_.clear();
		costmapGoalDist_ = std::vector<float>(width_ * height_, 0.0);
		costmapObstacle_ = std::vector<float>(width_ * height_, 0.0);
		costmapPathDist_ = std::vector<float>(width_ * height_, 0.0);
		costmap_ = std::vector<float>(width_ * height_, 0.0);
		occupiedCellSet_.clear();

		// define empty cell
		Gridcell cell;
		cell.state = STATIC;
		cell.lo = getLogOddsFromProb(lowProb_);
		cell.occupied = false;

		// fill data vector with empty cells
		for (int ii = 0; ii < height_; ii++)
		{
			for (int jj = 0; jj < width_; jj++)
			{
				double cellProb = (highProb_ - lowProb_) * (data.at(ii * width_ + jj) / 255) + lowProb_;
				cell.lo = getLogOddsFromProb(cellProb);

				if (getProbFromLogOdds(cell.lo) > occThresh_)
				{
					cell.state = FIXED;
					cell.occupied = true;
					occupiedCellSet_.insert(std::make_pair(jj, ii));
				}
				else
				{
					cell.state = STATIC;
					cell.occupied = false;
				}


				data_.push_back(cell);
			}
		}
	}

	QImage Gridmap::exportMap()
	{
		QImage outImg(width_, height_, QImage::Format_RGB32);

		for (int ii = 0; ii < width_; ii++)
		{
			for (int jj = 0; jj < height_; jj++)
			{
				int col = static_cast<int>((1 - getProbFromLogOdds(data_.at(jj * width_ + ii).lo)) * 255);
				if (data_.at(jj * width_ + ii).state == FIXED)
				{
					outImg.setPixel(ii, jj, qRgb(0, col, 0));
				}
				else
				{
					outImg.setPixel(ii, jj, qRgb(col, col, col));
				}
			}
		}
		return outImg;
	}

	QImage Gridmap::exportCostmap()
	{
		QImage outImg(width_, height_, QImage::Format_RGB32);
		for (int ii = 0; ii < width_; ii++)
		{
			for (int jj = 0; jj < height_; jj++)
			{
				//TODO change into heatmap for better visualization
				int col = static_cast<int>(costmap_[getOneDimIndex(ii, jj)] * 255.);
				if (col > 255)
					col = 255;

				col = 255 - col;
				outImg.setPixel(ii, jj, qRgb(col, col, col));
			}
		}
		return outImg;
	}

	//QImage Gridmap::exportBinaryOccMap()
	//{
	//	QImage outImg(width_, height_, QImage::Format_RGB32);

	//	for (int ii = 0; ii < width_; ii++)
	//	{
	//		for (int jj = 0; jj < height_; jj++)
	//		{

	//		}
	//	}
	//	return outImg;
	//}

	std::pair<double, double> Gridmap::pointfromMapIndex(std::pair<size_t, size_t>& index)
	{
		double x = index.first * res_ + res_ / 2;
		double y = (height_ - index.second) * res_ + res_ / 2;
		return std::make_pair(x, y);
	}

	std::vector<std::pair<double, double>> Gridmap::pointCloudInWordCoordsFromMapIndex(std::pair<size_t, size_t> index)
	{
		std::vector<std::pair<double, double>> outVec;

		double x = index.first * res_ + mapOffset_.first;
		double y = (height_ - index.second) * res_ + mapOffset_.second;

		outVec.push_back(std::make_pair(x, y));
		outVec.push_back(std::make_pair((x + res_), y));
		outVec.push_back(std::make_pair(x, (y + res_)));
		outVec.push_back(std::make_pair((x + res_), (y + res_)));

		return outVec;
	}

	std::vector<std::pair<double, double>> Gridmap::pointCloudInRobotCoordsFromMapIndex(std::pair<size_t, size_t> index, VSM::Vector3 robotPoseInWorldCoords)
	{
		VSM::Vector3 rp = poseInMapCoords(robotPoseInWorldCoords);
		VSM::Frame rf;
		rf.setPosition(VSM::Vector3(rp.x, rp.y, 0.0));
		VSM::Matrix3x3 orientation;
		orientation.setRotZ(rp.z, true);
		rf.setOrientation(orientation);

		std::vector<std::pair<double, double>> outVec;

		double x = index.first * res_;
		double y = (height_ - index.second) * res_;
		//double x = index.first * res_ + res_ / 2;
		//double y = (height_ - index.second) * res_ + res_ / 2;

		outVec.push_back(std::make_pair(x, y));
		outVec.push_back(std::make_pair((x + res_), y));
		outVec.push_back(std::make_pair(x, (y + res_)));
		outVec.push_back(std::make_pair((x + res_), (y + res_)));

		// transform into robot frame
		for (auto& it : outVec)
		{
			//double xRobotFrame = it.first * std::cos(rp.z) + it.second * std::sin(rp.z)
			//	- rp.x * std::cos(rp.z) - rp.y * std::sin(rp.z);
			//double yRobotFrame = -it.first * std::sin(rp.z) + it.second * std::cos(rp.z)
			//	+ rp.x * std::sin(rp.z) - rp.x * std::cos(rp.z);

			//it.first = xRobotFrame;
			//it.second = yRobotFrame;
			VSM::Vector3 pointInRobotFrame = rf.invTrafo(VSM::Vector3(it.first, it.second, 0.0));

			it.first = pointInRobotFrame.x;
			it.second = pointInRobotFrame.y;
		}

		return outVec;
	}

	VSM::Vector3 Gridmap::poseInMapCoords(VSM::Vector3& poseInWorldCoords)
	{
		return VSM::Vector3(poseInWorldCoords.x - mapOffset_.first, poseInWorldCoords.y - mapOffset_.second, poseInWorldCoords.z);
	}

	VSM::Vector3 Gridmap::poseInWorldCoords(VSM::Vector3& poseInMapCoords)
	{
		return VSM::Vector3(poseInMapCoords.x + mapOffset_.first, poseInMapCoords.y + mapOffset_.second, poseInMapCoords.z);
	}

	std::pair<size_t, size_t> Gridmap::mapIndexFromMapPose(VSM::Vector3& poseInMapCoords)
	{
		size_t indexX = std::floor(poseInMapCoords.x / res_);
		size_t indexY = height_ - std::floor(poseInMapCoords.y / res_);

		return std::make_pair(indexX, indexY);
	}

	std::pair<size_t, size_t> Gridmap::mapIndexFromWorldPose(VSM::Vector3& poseInWorldCoords)
	{
		return mapIndexFromMapPose(poseInMapCoords(poseInWorldCoords));
	}

	void Gridmap::updateMapFromScan(std::vector<double>& ranges, VSM::Vector3& robotPoseInWorldCoords)
	{
		// update pose in map coords
		robotPoseInMapCoords_ = poseInMapCoords(robotPoseInWorldCoords);
		updateSensorPose();

		// abort if robot outside of the map
		double maxMapWidth = width_ * res_;
		double maxMapHeight = height_ * res_;
		if (robotPoseInMapCoords_.x < 0 || robotPoseInMapCoords_.x > maxMapWidth)
			if (robotPoseInMapCoords_.y < 0 || robotPoseInMapCoords_.y > maxMapHeight)
				return;

		// for each cell update probability
		for (size_t jj = 0; jj < height_; jj++)
		{
			for (size_t ii = 0; ii < width_; ii++)
			{
				double oldLogOdd = data_.at(jj * width_ + ii).lo;

				std::pair<size_t, size_t> currentCellIndex = std::make_pair(ii, jj);
				std::vector<double> cellRanges = getRangesForCell(currentCellIndex, ranges);

				if (cellRanges.empty())
					continue;

				double invModelProb = getBeamRangeFinderInvModelProb(currentCellIndex, cellRanges);

				double newLogOdd = oldLogOdd + getLogOddsFromProb(invModelProb) - getLogOddsPrior();

				float probBoundLow = 0.1;
				float probBoundHigh = 0.9;
				if (newLogOdd < getLogOddsFromProb(probBoundLow))
					newLogOdd = getLogOddsFromProb(probBoundLow);
				if (newLogOdd > getLogOddsFromProb(probBoundHigh))
					newLogOdd = getLogOddsFromProb(probBoundHigh);

				if (getProbFromLogOdds(newLogOdd) > occThresh_)
				{
					data_.at(jj * width_ + ii).occupied = true;
					occupiedCellSet_.insert(std::make_pair(ii, jj));
				}
				else
				{
					data_.at(jj * width_ + ii).occupied = false;
					occupiedCellSet_.erase(std::make_pair(ii, jj));
				}



				data_.at(jj * width_ + ii).lo = newLogOdd;
			}
		}
	}

	double Gridmap::getBeamRangeFinderInvModelProb(std::pair<size_t, size_t>& cellIndex, std::vector<double>& rangesForCell)
	{
		// sensor model: use simplified model from Moravec1988 Fig 11 B 
		// simplify exponential function by trapeze
		// (opt.: Probabilistic Robotics Fig 6.3)

		// hardcode measurement noise param
		double d1 = 0.05;
		double d2 = 0.15;
		double d3 = 0.2;

		// get cell centre range
		std::pair<double, double> cellCentre = pointfromMapIndex(cellIndex);
		double cellRange = getPointDistToSensor(cellCentre);

		double prob = 0;

		for (auto& it : rangesForCell)
		{
			if (it >= maxRange_)
			{
				prob += lowProb_;
				continue;
			}

			double bound1 = it - d1;
			double bound2 = it + d1;
			double bound3 = it + d2;
			double bound4 = it + d3;

			//        _
			//       / \
			//      /   \_____
			// ____/

			if (cellRange < bound1)
			{
				prob += lowProb_;
			}
			else if (cellRange > bound1 && cellRange < bound2)
			{
				prob += lowProb_ + (highProb_ - lowProb_) * (cellRange - bound1) / (bound2 - bound1);
			}
			else if (cellRange > bound2 && cellRange < bound3)
			{
				prob += highProb_;
			}
			else if (cellRange > bound3 && cellRange < bound4)
			{
				prob += priorProb_ + (highProb_ - priorProb_) * (cellRange - bound3) / (bound4 - bound3);
			}
			else
			{
				prob += priorProb_;
			}

		}

		return prob / rangesForCell.size();
	}

	double Gridmap::getLogOddsPrior()
	{
		return std::log(0.5 / 0.5);
	}

	std::vector<double> Gridmap::getRangesForCell(std::pair<size_t, size_t>& cellIndex, std::vector<double> ranges)
	{
		std::pair<double, double> cellCentre = pointfromMapIndex(cellIndex);
		double cellAngle = getPointAngleToSensor(cellCentre);

		std::vector<double> outRanges;
		int numScans = 3;
		for (int ii = 0; ii < numScans; ii++)
		{
			double off = (numScans - 1) * angleResolution_ * 0.5;
			double currentAngle = cellAngle - off + ii * angleResolution_;
			if (currentAngle > angleOffset_ && currentAngle < angleOffset_ + ranges.size() * angleResolution_)
			{
				size_t index = std::floor((currentAngle - angleOffset_) / angleResolution_);

				if (index >= ranges.size())
					continue;

				outRanges.push_back(ranges.at(index));
			}
		}

		return outRanges;
	}

	double Gridmap::getPointDistToSensor(std::pair<double, double>& pointInSensorFrame)
	{
		return std::sqrt(std::pow((pointInSensorFrame.first - sensorPoseInMapCoords_.x), 2) + std::pow((pointInSensorFrame.second - sensorPoseInMapCoords_.y), 2));
	}

	double Gridmap::getPointAngleToSensor(std::pair<double, double>& pointInSensorFrame)
	{
		return wrapAngle(sensorPoseInMapCoords_.z - std::atan2((pointInSensorFrame.second - sensorPoseInMapCoords_.y), (pointInSensorFrame.first - sensorPoseInMapCoords_.x)));
	}

	/* under construction (begin) */
	inline size_t Gridmap::getOneDimIndex(std::pair<size_t, size_t>& cellIndex)
	{
		return cellIndex.second * width_ + cellIndex.first;
	}

	inline size_t Gridmap::getOneDimIndex(size_t x, size_t y)
	{
		return y * width_ + x;
	}

	std::vector<std::pair<size_t, size_t>> Gridmap::getMooreNeighbours(std::pair<size_t, size_t>& cell)
	{
		std::vector<std::pair<size_t, size_t>> outVec;

		for (int ii = -1; ii < 2; ii++)
		{
			int x = cell.first + ii;
			if (x < 0 || x >= width_)
				continue;

			for (int jj = -1; jj < 2; jj++)
			{
				int y = cell.second + jj;
				if (y < 0 || y >= height_)
					continue;

				if (ii == 0 && jj == 0)
					continue;

				outVec.push_back(std::make_pair(x, y));
			}
		}

		return outVec;
	}

	std::unordered_set<std::pair<size_t, size_t>, pairhash> Gridmap::getOccupiedCellSet()
	{
		return occupiedCellSet_;
	}

	std::vector<std::pair<double, double>> Gridmap::getOccupiedCellPointCloudInWorldCoords()
	{
		std::vector<std::pair<double, double>> outVec;
		outVec.reserve(occupiedCellSet_.size() * 4);

		for (auto& it : occupiedCellSet_)
		{
			std::vector<std::pair<double, double>> cellPointCloud = pointCloudInWordCoordsFromMapIndex(it);
			outVec.insert(outVec.end(), cellPointCloud.begin(), cellPointCloud.end());
		}

		return outVec;
	}

	std::vector<std::pair<double, double>> Gridmap::getOccupiedCellPointCloudInRobotCoords(VSM::Vector3 robotPoseInWorldCoords, bool inVincinity)
	{
		std::vector<std::pair<double, double>> outVec;
		outVec.reserve(occupiedCellSet_.size() * 4);

		for (auto& it : occupiedCellSet_)
		{
			std::vector<std::pair<double, double>> cellPointCloud = pointCloudInRobotCoordsFromMapIndex(it, robotPoseInWorldCoords);

			if (inVincinity)
			{
				double thresh1 = 0.7;
				double thresh2 = 0.6;
				auto& itCellPt = cellPointCloud.end();
				do {
					itCellPt--;

					if (std::fabs(itCellPt->first) > thresh2 || std::fabs(itCellPt->second) > thresh1)
					{
						itCellPt = cellPointCloud.erase(itCellPt);
					}
				
				} while (itCellPt != cellPointCloud.begin());

				outVec.insert(outVec.end(), cellPointCloud.begin(), cellPointCloud.end());
			}
			else
				outVec.insert(outVec.end(), cellPointCloud.begin(), cellPointCloud.end());
		}

		outVec.shrink_to_fit();

		return outVec;
	}

	std::unordered_set<std::pair<size_t, size_t>, pairhash> Gridmap::getInflatedOccupiedCellSet(double inflationRadius, std::unordered_set<std::pair<size_t, size_t>, pairhash>& occupied)
	{
		std::unordered_set<std::pair<size_t, size_t>, pairhash> outSet;

		std::unordered_map<std::pair<size_t, size_t>, float, pairhash> visited;
		std::vector<std::pair<size_t, size_t>> currentFront, nextFront;

		for (auto& itOcc : occupied)
		{
			outSet.insert(itOcc);
			nextFront.push_back(itOcc);
			visited.insert({ itOcc, 0.0});
		}

		while (!nextFront.empty())
		{
			currentFront = nextFront;
			nextFront.clear();

			for (auto& itFront : currentFront)
			{
				auto neighbourCells = getMooreNeighbours(itFront);
				for (auto& it : neighbourCells)
				{
					int xDiff = (int)it.first - (int)itFront.first;
					int yDiff = (int)it.second - (int)itFront.second;

					float newDist;
					if (xDiff == 0 || yDiff == 0)
						newDist = 1.0 * res_;
					else
						newDist = M_SQRT2 * res_;

					float tmp = visited[itFront];
					newDist += tmp;

					auto search = visited.find(it);
					if (search == visited.end())
					{
						if (newDist < inflationRadius)
						{
							visited.insert({it, newDist});
							outSet.insert(it);
							nextFront.push_back(it);
						}
					}
					else
					{
						if (newDist < search->second)
						{
							search->second = newDist;
							nextFront.push_back(it);
						}
					}
				}
			}
		}
		
		return outSet;
	}

	void Gridmap::generateCostMap(VSM::Vector3& robotPoseInWorldCoords, VSM::Vector3& goalPoseInWorldCoords)
	{
		// get cell of robot and goal
		auto robotCell = mapIndexFromWorldPose(robotPoseInWorldCoords);
		auto goalCell = mapIndexFromWorldPose(goalPoseInWorldCoords);

		// initialize costmaps
		int length = width_ * height_;
		float maxVal = std::hypot(width_, height_);

		costmapGoalDist_ = std::vector<float>(length, maxVal);
		costmapObstacle_ = std::vector<float>(length, 0.0);
		costmapPathDist_ = std::vector<float>(length, maxVal);
		costmap_ = std::vector<float>(length, 0.0);

		// inflate binary occ map by shortest edge of robot
		std::unordered_set<std::pair<size_t, size_t>, pairhash> occupiedInflated;
		occupiedInflated = getInflatedOccupiedCellSet(0.62, occupiedCellSet_);

		/* goal distance function */
		calcGoalDistCostmap(robotCell, goalCell, occupiedInflated);

		/* obstacle function */
		calcObstacleCostmap(2.0, occupiedInflated);

		/* path distance function */
		calcPathDistCostmap(robotCell, goalCell);

		/* final normed and weigthed costmap*/
		calcWeightedCostmap();
	}

	void Gridmap::calcGoalDistCostmap(std::pair<size_t, size_t>& robotCell, std::pair<size_t, size_t>& goalCell, std::unordered_set<std::pair<size_t, size_t>, pairhash>& occupiedInflated)
	{
		std::unordered_set<std::pair<size_t, size_t>, pairhash> visited;
		std::vector<std::pair<size_t, size_t>> currentFront, nextFront;

		visited.insert(goalCell);
		nextFront.push_back(goalCell);
		costmapGoalDist_[getOneDimIndex(goalCell)] = 0;

		bool done = false;
		while (!(done || nextFront.empty()))
		{
			currentFront = nextFront;
			nextFront.clear();

			for (auto& currentCell : currentFront)
			{
				// abort if robot cell is reached
				if (getOneDimIndex(currentCell) == getOneDimIndex(robotCell))
					done = true;

				float cellDist = costmapGoalDist_[getOneDimIndex(currentCell)];
				auto neighbourCells = getMooreNeighbours(currentCell);
				for (auto& it : neighbourCells)
				{
					// add neighbour cell to next wavefront if not occupied
					auto search = occupiedInflated.find(it);
					if (search != occupiedInflated.end())
						continue;

					float neighbourDist = costmapGoalDist_[getOneDimIndex(it)];

					float newDist;
					if (it.first == currentCell.first || it.second == currentCell.second)
						newDist = cellDist + 1;
					else
						newDist = cellDist + M_SQRT2;

					search = visited.find(it);
					if (search == visited.end())
					{
						visited.insert(it);
						nextFront.push_back(it);
						costmapGoalDist_[getOneDimIndex(it)] = newDist;
					}
					else
					{
						if (newDist < neighbourDist)
						{
							costmapGoalDist_[getOneDimIndex(it)] = newDist;
							nextFront.push_back(it);
						}
					}

				}
			}
		}
	}

	void Gridmap::calcObstacleCostmap(double obstacleSafetyDist, std::unordered_set<std::pair<size_t, size_t>, pairhash>& occupiedInflated)
	{
		float obstacleDistMaxVal = 1.0;

		std::unordered_set<std::pair<size_t, size_t>, pairhash> visited;
		std::vector<std::pair<size_t, size_t>> currentFront, nextFront;

		std::unordered_set<std::pair<size_t, size_t>, pairhash> pixSafety = getInflatedOccupiedCellSet(obstacleSafetyDist, occupiedInflated);

		for (auto it : pixSafety)
		{
			auto searchOcc = occupiedInflated.find(it);
			if (searchOcc != occupiedInflated.end())
			{
				costmapObstacle_[getOneDimIndex(it)] = obstacleDistMaxVal;
				continue;
			}
			else
			{
				float minDist = std::numeric_limits<float>::max();
				for (auto itOcc : occupiedInflated)
				{
					float xDiff = (int)it.first - (int)itOcc.first;
					float yDiff = (int)it.second - (int)itOcc.second;
					float distToObstacle = std::hypot(xDiff, yDiff) * res_;

					minDist = std::min(minDist, distToObstacle);
				}
				float newVal = (1. - minDist / obstacleSafetyDist) * obstacleDistMaxVal;
				costmapObstacle_[getOneDimIndex(it)] = newVal;
			}
		}
	}

	void Gridmap::calcPathDistCostmap(std::pair<size_t, size_t>& robotCell, std::pair<size_t, size_t>& goalCell)
	{
		if (costmapGoalDist_.empty())
			return;

		// get shortest path based on goal dist costmap
		std::set<std::pair<size_t, size_t>> pathToGoal;
		pathToGoal.insert(robotCell);
		float distToGoal = costmapGoalDist_[getOneDimIndex(robotCell)];
		auto currentCell = robotCell;
		while (distToGoal != 0.)
		{
			auto neighbourCells = getMooreNeighbours(currentCell);
			auto bestCell = neighbourCells.at(0);
			float bestDist = costmapGoalDist_[getOneDimIndex(bestCell)];
			for (auto& it : neighbourCells)
			{
				float newDist = costmapGoalDist_[getOneDimIndex(it)];
				if (newDist < bestDist)
				{
					bestDist = newDist;
					bestCell = it;
				}
			}

			distToGoal = bestDist;
			pathToGoal.insert(bestCell);
			currentCell = bestCell;
		}

		// calc distance from path
		std::set<std::pair<size_t, size_t>> visited;
		std::vector<std::pair<size_t, size_t>> currentFront, nextFront;

		for (auto itPath : pathToGoal)
		{
			nextFront.push_back(itPath);
			visited.insert(itPath);
			costmapPathDist_[getOneDimIndex(itPath)] = 0;
		}

		float maxDistance = 5.0;
		bool done = false;
		while (!(done || nextFront.empty()))
		{
			currentFront = nextFront;
			nextFront.clear();

			for (auto& currentCell : currentFront)
			{
				// abort if maxDistance is reached
				if (costmapPathDist_[getOneDimIndex(currentCell)] > maxDistance)
					done = true;

				float cellDist = costmapPathDist_[getOneDimIndex(currentCell)];
				auto neighbourCells = getMooreNeighbours(currentCell);
				for (auto& it : neighbourCells)
				{
					float neighbourDist = costmapPathDist_[getOneDimIndex(it)];

					float newDist;
					if (it.first == currentCell.first || it.second == currentCell.second)
						newDist = cellDist + 1;
					else
						newDist = cellDist + M_SQRT2;

					auto search = visited.find(it);
					if (search == visited.end())
					{
						visited.insert(it);
						nextFront.push_back(it);
						costmapPathDist_[getOneDimIndex(it)] = newDist;
					}
					else
					{
						if (newDist < neighbourDist)
						{
							costmapPathDist_[getOneDimIndex(it)] = newDist;
							nextFront.push_back(it);
						}
					}
				}
			}
		}

	}

	void Gridmap::calcWeightedCostmap()
	{
		std::lock_guard<std::mutex> lock(mCostmap_);

		int length = width_ * height_;
		
		for (size_t ii = 0; ii < length; ii++)
		{
			costmap_[ii] = calcCostmapWeighting(ii);
		}
	}

	double Gridmap::getCostmapWeighting(VSM::Vector3& pose)
	{
		size_t poseIndex = getOneDimIndex(mapIndexFromWorldPose(pose));

		return getCostmapWeighting(poseIndex);
	}

	double Gridmap::getCostmapWeighting(std::pair<size_t, size_t>& cell)
	{
		size_t poseIndex = getOneDimIndex(cell);

		return getCostmapWeighting(poseIndex);
	}

	double Gridmap::getCostmapWeighting(size_t& oneDimCellIndex)
	{
		std::lock_guard<std::mutex> lock(mCostmap_);

		if (oneDimCellIndex > width_ * height_)
			return 1.0;

		return costmap_[oneDimCellIndex];
	}

	double Gridmap::calcCostmapWeighting(size_t& oneDimCellIndex)
	{
		int length = width_ * height_;
		float maxVal = std::hypot(width_, height_);

		float obstacleDistMaxVal = 1.0;

		double goalDistCostNormed = costmapGoalDist_[oneDimCellIndex] / maxVal;
		double obstacleDistCostNormed = costmapObstacle_[oneDimCellIndex] / obstacleDistMaxVal;
		double pathDistCostNormed = costmapPathDist_[oneDimCellIndex] / maxVal;

		double goalDistWeight = 1.0;
		double obstacleDistWeight = 0.0;
		double pathDistWeight = 0.0;
		double weightSum = goalDistWeight + obstacleDistWeight + pathDistWeight;
		double sum = (goalDistWeight * goalDistCostNormed + obstacleDistWeight * obstacleDistCostNormed + pathDistWeight * pathDistCostNormed) / weightSum;

		return 1. - sum;
	}

	double Gridmap::getGradNF1(VSM::Vector3& pose)
	{
		std::lock_guard<std::mutex> lock(mCostmap_);

		std::pair<size_t, size_t> poseCell = mapIndexFromWorldPose(pose);
		double poseCost = costmapGoalDist_[getOneDimIndex(poseCell)];

		std::unordered_set<std::pair<size_t, size_t>, pairhash> poseCellSet;
		poseCellSet.insert(poseCell);
		auto neighbourCells = getInflatedOccupiedCellSet(0.2, poseCellSet);
		//auto neighbourCells = getMooreNeighbours(poseCell);

		double bestCost = std::numeric_limits<double>::max();
		std::pair<size_t, size_t> bestGradCell;
		for (auto it : neighbourCells)
		{
			double neighbourCost = costmapGoalDist_[getOneDimIndex(it)];

			if (neighbourCost < bestCost)
			{
				bestCost = neighbourCost;
				bestGradCell = it;
			}
		}

		if (bestCost >= poseCost)
			return 0.;

		int xDiff = static_cast<int>(bestGradCell.first) - static_cast<int>(poseCell.first);
		int yDiff = static_cast<int>(bestGradCell.second) - static_cast<int>(poseCell.second);

		double bestGradAngle = std::atan2(-yDiff, xDiff);
		double diffAngle = wrapAngle(pose.z - (bestGradAngle - M_PI_2));
		double weight = (M_PI - std::fabs(diffAngle)) / M_PI;

		return weight;
	}

	double Gridmap::getDeltaNF1(VSM::Vector3& pose)
	{
		return 0.;
	}

	/* under construction (end) */
} // end namespace