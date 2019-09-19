#include "utils.h"
#include <random>
#include <algorithm>
#include "linalg.h"
using namespace RRT_star::path_planning::utils;
using namespace RRT_star::world;
WorldPoint RRT_star::path_planning::utils::randomSample(WorldPoint minPlace,
	WorldPoint maxPlace)
{
	return WorldPoint(generateRand(minPlace.first, maxPlace.first),
		generateRand(minPlace.second, maxPlace.second));
}
double RRT_star::path_planning::utils::generateRand(double min, double max)
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(min, max);
	return round(dis(gen)*100.0) / 100.0;
}
WorldPoint RRT_star::path_planning::utils::nearestNeighbour(WorldPoint otherPoint,
	Tree tree)
{
	return tree.getVertexes().at(nearest(otherPoint, tree.getVertexes()));
}
int RRT_star::path_planning::utils::nearest(WorldPoint otherPoint,
	std::vector<WorldPoint> points)
{
	std::vector<double> dist;
	for_each(points.begin(), points.end(), [&dist, otherPoint](WorldPoint point) {
		dist.push_back(distance(point, otherPoint));
	});
	return std::distance(dist.begin(), std::min_element(dist.begin(), dist.end()));
}

WorldPoint RRT_star::path_planning::utils::findStoppingState(WorldPoint nearPoint,
	WorldPoint randPoint, Tree tree, WorldMap map, double gap)
{
	Line line(nearPoint, randPoint);
	WorldPoint newPoint;
	std::vector<WorldPoint> crossPoints;
	if (map.hasCollision(line, crossPoints)) {
		WorldPoint firstCross = crossPoints.at(nearest(nearPoint, crossPoints));
		newPoint = indent(firstCross, nearPoint, gap);
	}
	else {
		newPoint = randPoint;
	}
	return newPoint;
}
void RRT_star::path_planning::utils::normalize(WorldPoint point, double gap)
{
	double len = std::sqrt(std::pow(point.first, 2) + std::pow(point.second, 2));
	point.first = point.first / len * gap;
	point.second = point.second / len * gap;
}
RRT_star::path_planning::utils::WorldPoint RRT_star::path_planning::utils::indent
(WorldPoint pointA, WorldPoint pointB, double gap)
{
	WorldPoint directionVec(pointB.first - pointA.first,
		pointB.second - pointA.second);
	normalize(directionVec, gap);
	double first = pointA.first + directionVec.first;
	double second = pointA.second + directionVec.second;
	return WorldPoint(first, second);
}
void RRT_star::path_planning::utils::nearestNeighbours(
	std::vector<int>& nearestInd, std::vector<double>& distanceToNew,
	WorldPoint newPoint, double r, std::vector<WorldPoint> points)
{
	int j = 0;
	for_each(points.begin(), points.end(), [&](WorldPoint point) {
		double dist = utils::distance(point, newPoint);
		if (dist < r)
		{
			nearestInd.push_back(j);
		}
		distanceToNew.push_back(dist);
		j++;
	});
}
double RRT_star::path_planning::utils::distance(WorldPoint pointA,
	WorldPoint pointB)
{
	return linalg::distance(pointA.first, pointA.second, pointB.first,
		pointB.second);
}
void RRT_star::path_planning::utils::sortNearestNeighbours(
	std::vector<double>& sortedCosts, std::vector<int>& prevSortInd,
	std::vector<int> nearestInd, std::vector<double> distanceToNew,
	std::vector<double> costs)
{
	typedef std::pair<double, int> CostPair;
	std::vector<CostPair> costsPair;
	int j = 0;
	int k = 0;
	for (auto &i : costs) {
		if (std::find(nearestInd.begin(), nearestInd.end(), j) != nearestInd.end())
		{
			costsPair.push_back(CostPair(i + distanceToNew[j], k));
			k++;
		}
		j++;
	}
	std::sort(costsPair.begin(), costsPair.end(), [](const auto& lhs,
		const auto& rhs) {
		return lhs.first < rhs.first;
	});
	for_each(costsPair.begin(), costsPair.end(), [&sortedCosts, &prevSortInd]
	(CostPair i) {
		sortedCosts.push_back(i.first);
		prevSortInd.push_back(i.second);
	});
}
void RRT_star::path_planning::utils::minCostParent(double& newCost, int& iParent,
	WorldPoint newPoint, std::vector<int> nearestInd,
	std::vector<double> sortedCosts, std::vector<int> prevSortInd,
	Tree tree, WorldMap map)
{
	for (size_t j = 0; j < prevSortInd.size(); j++) {
		iParent = prevSortInd[j];
		if (!map.hasCollision(Line(newPoint,
			tree.getVertexes()[nearestInd[iParent]])))
		{
			newCost = sortedCosts[j];
			break;
		}
	}
}
void RRT_star::path_planning::utils::rewire(Tree tree, double newCost,
	std::vector<double> distanceToNew, std::vector<int> nearestInd,
	WorldPoint newPoint, WorldMap map)
{
	for (const auto &j : nearestInd)
	{
		if ((newCost + distanceToNew[j]) < tree.getCosts()[j]) {
			int iNode = j;
			if (!map.hasCollision(Line(newPoint, tree.getVertexes()[iNode])))
			{
				tree.rewireVertex(iNode, newCost + distanceToNew[iNode]);
			}
		}
	}
}

void RRT_star::path_planning::utils::findInvalidVertexes(std::vector<unsigned int> &invalidInd, Tree &tree, world::WorldMap map)
{
	for (unsigned int i = 0; i < tree.getSize(); i++) {
		int parentInd = tree.getParents()[i];
		if (parentInd != -1) {
			WorldPoint parent = tree.getVertexes()[parentInd];
			WorldPoint currentPoint = tree.getVertexes()[i];
			Line line(parent, currentPoint);
			if (map.hasCollision(line)) {
				tree.unwireVertex(i);
				invalidInd.push_back(i);
			}
		}
	}
}

void RRT_star::path_planning::utils::addChildrenToInvalid(std::vector<unsigned int> &invalidInd, Tree &tree)
{
	std::vector<unsigned int> localInvalidInd;
	for (unsigned int j = 0; j < tree.getSize(); j++) {
		//TODO: new data structure
		if (std::find(invalidInd.begin(), invalidInd.end(), tree.getParents()[j]) != invalidInd.end()) {
			if (!tree.isInvalid(j)) {
				tree.unwireVertex(j);
				localInvalidInd.push_back(j);
			}
		}
	}
	if (localInvalidInd.size() != 0) {
		addChildrenToInvalid(localInvalidInd, tree);
	}
	invalidInd.insert(invalidInd.end(), localInvalidInd.begin(), localInvalidInd.end());
}
