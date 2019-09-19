#ifndef _RRT_PATH_PLANNER_HH
#define _RRT_PATH_PLANNER_HH
#include "PathPlanner.h"
#include "utils.h"
#include "Tree.h"
namespace RRT_star {
	namespace path_planning {
		using namespace utils;
		class RRTPathPlanner : public PathPlanner {
		public:
			RRTPathPlanner(WorldMap map) :PathPlanner(map) {};
			void setSteps(int steps);
			virtual Path findPath(WorldPoint startPoint, WorldPoint endPoint) 
				override;
			virtual void rebuildPath(Path* path, WorldMap map) override;
			Tree getTree();
		private:
			WorldPoint startPoint;
			WorldPoint endPoint;
			Tree tree;
			int N_steps = 1000;
			double gap;//used for get newPoint from crossPoint
			WorldPoint nearestNeighbour(WorldPoint otherPoint);
			WorldPoint findStoppingState(WorldPoint nearPoint, WorldPoint randPoint);
			void initGap();
			double searchRadius(int step);
			void nearestNeighbours(std::vector<int>& nearestInd, 
				std::vector<double>& distanceToNew, WorldPoint newPoint, double r);
			void sortNearestNeighbours(std::vector<double>& sortedCosts, 
				std::vector<int>& prevSortInd, std::vector<int> nearestInd, 
				std::vector<double> distanceToNew);
			void minCostParent(double& newCost, int& iParent, WorldPoint newPoint, 
				std::vector<int> nearestInd, std::vector<double> sortedCosts, 
				std::vector<int> prevSortInd);
			void rewire(double newCost, std::vector<double> distanceToNew, 
				std::vector<int> nearestInd, WorldPoint newPoint);
			Path optimalPathPlot(int nearInd);
		};
	}
}
#endif