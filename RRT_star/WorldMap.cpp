#include "WorldMap.h"
using namespace RRT_star::world::geometry;
RRT_star::world::WorldMap::WorldMap(WorldPoint _minPlace, WorldPoint _maxPlace)
{
	minPlace = _minPlace;
	maxPlace = _maxPlace;
}
void RRT_star::world::WorldMap::addObstacle(Obstacle obstacle)
{
	obstacles.push_back(obstacle);
}
bool RRT_star::world::WorldMap::hasCollision(const Line& line)
{
	return std::any_of(obstacles.cbegin(), obstacles.cend(), [line](Obstacle 
		obstacle) {
		return obstacle.hasCollision(line);
	});
}
bool RRT_star::world::WorldMap::hasCollision(const Line& line, 
	std::vector<WorldPoint>& crossPoints)
{
	bool hasCross = false;
	for_each(obstacles.cbegin(), obstacles.cend(), [line, &hasCross, &crossPoints]
	(Obstacle obstacle) {
		std::vector<WorldPoint> crossPointsLocal;
		if (obstacle.hasCollision(line, crossPointsLocal)) {
			crossPoints.insert(crossPoints.cend(), crossPointsLocal.cbegin(), 
				crossPointsLocal.cend());
			hasCross = true;
		}
	});
	return hasCross;
}
WorldPoint RRT_star::world::WorldMap::getMinPlace()
{
	return minPlace;
}
WorldPoint RRT_star::world::WorldMap::getMaxPlace()
{
	return maxPlace;
}

std::vector<Obstacle> RRT_star::world::WorldMap::getObstacles()
{
	return obstacles;
}
