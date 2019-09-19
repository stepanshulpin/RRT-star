#include "BoxObstacle.h"
RRT_star::world::geometry::BoxObstacle::BoxObstacle(WorldPoint pointA, WorldPoint pointB, WorldPoint pointC, WorldPoint pointD)
{
	lines.emplace_back(pointA, pointB);
	lines.emplace_back(pointB, pointC);
	lines.emplace_back(pointC, pointD);
	lines.emplace_back(pointD, pointA);
}
RRT_star::world::geometry::BoxObstacle::BoxObstacle(WorldPoint pointMin, WorldPoint pointMax)
{
	WorldPoint minMax(pointMin.first, pointMax.second);
	WorldPoint maxMin(pointMax.first, pointMin.second);
	lines.emplace_back(pointMin, minMax);
	lines.emplace_back(minMax, pointMax);
	lines.emplace_back(pointMax, maxMin);
	lines.emplace_back(maxMin, pointMin);
}
