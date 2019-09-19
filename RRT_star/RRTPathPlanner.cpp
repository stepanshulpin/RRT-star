#include "RRTPathPlanner.h"
using namespace RRT_star::path_planning;
void RRTPathPlanner::setSteps(int steps)
{
	N_steps = steps;
}
Path RRTPathPlanner::findPath(WorldPoint startPoint, WorldPoint endPoint)
{
	this->startPoint = startPoint;
	this->endPoint = endPoint;
	initGap();
	tree.init(startPoint);
	int step = 0;
	while (step < N_steps) {
		WorldPoint randPoint = randomSample(map.getMinPlace(), map.getMaxPlace());
		WorldPoint nearPoint = nearestNeighbour(randPoint);
		WorldPoint newPoint = findStoppingState(nearPoint, randPoint);
		if (newPoint != nearPoint) {
			double r = searchRadius(step);
			std::vector<int> nearestInd;
			std::vector<double> distanceToNew;
			nearestNeighbours(nearestInd, distanceToNew, newPoint, r);
			if (!nearestInd.empty())
			{
				std::vector<double> sortedCosts;
				std::vector<int> prevSortInd;
				sortNearestNeighbours(sortedCosts, prevSortInd, nearestInd,
					distanceToNew);
				double newCost;
				int iParent;
				minCostParent(newCost, iParent, newPoint, nearestInd, sortedCosts,
					prevSortInd);
				tree.add(newPoint, nearestInd[iParent], newCost);
				rewire(newCost, distanceToNew, nearestInd, newPoint);
			}
		}
		step++;
	}
	int nearGoalInd = utils::nearest(endPoint, tree.getVertexes());
	Path path = optimalPathPlot(nearGoalInd);
	return path;
}
void RRTPathPlanner::rebuildPath(Path * path, WorldMap newMap)
{
	this->map = newMap;
	std::vector<unsigned int> invalidInd;
	utils::findInvalidVertexes(invalidInd, tree, map);
	utils::addChildrenToInvalid(invalidInd, tree);

	/*Tree newTree;
	newTree.init(startPoint);
	for (unsigned int i = 0; i < tree.getSize(); i++) {
		if ((!tree.isInvalid(i)) && (tree.getParents()[i] != -1)) {
			newTree.add(tree.getVertexes()[i], tree.getParents()[i], tree.getCosts()[i]);
		}
	}
	this->tree = newTree;*/

	//int nearGoalInd = utils::nearest(endPoint, tree.getVertexes());
	//path = &(optimalPathPlot(nearGoalInd));

}
Tree RRT_star::path_planning::RRTPathPlanner::getTree()
{
	return tree;
}
WorldPoint RRTPathPlanner::nearestNeighbour(WorldPoint otherPoint)
{
	return utils::nearestNeighbour(otherPoint, tree);
}
WorldPoint RRTPathPlanner::findStoppingState(WorldPoint nearPoint, WorldPoint
	randPoint)
{
	return utils::findStoppingState(nearPoint, randPoint, tree, map, gap);
}
void RRTPathPlanner::initGap()
{
	double diagonal = utils::distance(map.getMinPlace(), map.getMaxPlace());
	gap = diagonal * 0.00001;
}
double RRTPathPlanner::searchRadius(int step)
{
	double diagonal = utils::distance(map.getMinPlace(), map.getMaxPlace());
	return diagonal*(1 - ((double)step) / ((double)N_steps));
}
void RRTPathPlanner::nearestNeighbours(std::vector<int>& nearestInd,
	std::vector<double>& distanceToNew, WorldPoint newPoint, double r)
{
	utils::nearestNeighbours(nearestInd, distanceToNew, newPoint, r,
		tree.getVertexes());
}
void RRTPathPlanner::sortNearestNeighbours(std::vector<double>& sortedCosts,
	std::vector<int>& prevSortInd, std::vector<int> nearestInd,
	std::vector<double> distanceToNew)
{
	utils::sortNearestNeighbours(sortedCosts, prevSortInd, nearestInd,
		distanceToNew, tree.getCosts());
}
void RRTPathPlanner::minCostParent(double& newCost, int& iParent,
	WorldPoint newPoint, std::vector<int> nearestInd, std::vector<double>
	sortedCosts, std::vector<int> prevSortInd)
{
	utils::minCostParent(newCost, iParent, newPoint, nearestInd, sortedCosts,
		prevSortInd, tree, map);
}
void RRTPathPlanner::rewire(double newCost, std::vector<double> distanceToNew,
	std::vector<int> nearestInd, WorldPoint newPoint)
{
	utils::rewire(tree, newCost, distanceToNew, nearestInd, newPoint, map);
}
Path RRTPathPlanner::optimalPathPlot(int nearInd)
{
	Path path;
	int ix = nearInd;
	path.addPoint(tree.getVertexes()[ix]);
	while (tree.getParents()[ix] != -1)
	{
		ix = tree.getParents()[ix];
		path.addPoint(tree.getVertexes()[ix]);
	}
	return path;
}
