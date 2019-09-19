#include "Line.h"
RRT_star::world::geometry::Line::Line(WorldPoint _pointA, WorldPoint _pointB)
{
	pointA = _pointA;
	pointB = _pointB;
}
bool RRT_star::world::geometry::Line::hasCollision(const Line & otherLine,
	WorldPoint & crossPoint)
{
	if (hasCollision(otherLine)) {
		crossPoint = RRT_star::linalg::crossPoint(pointA.first, pointA.second,
			pointB.first, pointB.second, otherLine.pointA.first,
			otherLine.pointA.second, otherLine.pointB.first,
			otherLine.pointB.second);
		return true;
	}
	else {
		return false;
	}
}
bool RRT_star::world::geometry::Line::hasCollision(const Line & otherLine)
{
	return RRT_star::linalg::hasCross(pointA.first, pointA.second, pointB.first,
		pointB.second, otherLine.pointA.first, otherLine.pointA.second,
		otherLine.pointB.first, otherLine.pointB.second);
}

RRT_star::world::geometry::WorldPoint RRT_star::world::geometry::Line::getA()
{
	return pointA;
}

RRT_star::world::geometry::WorldPoint RRT_star::world::geometry::Line::getB()
{
	return pointB;
}

