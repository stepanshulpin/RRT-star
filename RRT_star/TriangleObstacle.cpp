#include "TriangleObstacle.h"
RRT_star::world::geometry::TriangleObstacle::TriangleObstacle(WorldPoint pointA, WorldPoint pointB, WorldPoint pointC)
{
	lines.emplace_back(pointA, pointB);
	lines.emplace_back(pointB, pointC);
	lines.emplace_back(pointC, pointA);
}
