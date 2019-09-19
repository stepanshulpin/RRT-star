#ifndef _UTILS_HH
#define _UTILS_HH
#include "Line.h"
#include "Tree.h"
#include "WorldMap.h"
#include <string> 
namespace RRT_star {
	namespace path_planning {
		namespace utils {
			WorldPoint randomSample(WorldPoint minPlace, WorldPoint maxPlace);
			double generateRand(double min, double max);
			WorldPoint nearestNeighbour(WorldPoint otherPoint, Tree tree);
			int nearest(WorldPoint otherPoint, std::vector<WorldPoint> points);
			WorldPoint findStoppingState(WorldPoint nearPoint, WorldPoint randPoint,
				Tree tree, world::WorldMap map, double gap);
			void normalize(WorldPoint point, double gap);
			WorldPoint indent(WorldPoint pointA, WorldPoint pointB, double gap);
			void nearestNeighbours(std::vector<int>& nearestInd, 
				std::vector<double>& distanceToNew, WorldPoint newPoint, double r,
				std::vector<WorldPoint> points);
			double distance(WorldPoint pointA, WorldPoint pointB);
			void sortNearestNeighbours(std::vector<double>& sortedCosts, 
				std::vector<int>& prevSortInd, std::vector<int> nearestInd, 
				std::vector<double> distanceToNew, std::vector<double> costs);
			void minCostParent(double& newCost, int& iParent, WorldPoint newPoint,
				std::vector<int> nearestInd, std::vector<double> sortedCosts, 
				std::vector<int> prevSortInd, Tree tree, world::WorldMap map);
			void rewire(Tree tree, double newCost, 
				std::vector<double> distanceToNew, std::vector<int> nearestInd, 
				WorldPoint newPoint, world::WorldMap map);
			void findInvalidVertexes(std::vector<unsigned int> &invalidInd, Tree &tree, world::WorldMap map);
			void addChildrenToInvalid(std::vector<unsigned int> &invalidInd, Tree &tree);
		}
	}
}
#endif 

