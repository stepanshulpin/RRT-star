#include "Obstacle.h"
bool RRT_star::world::geometry::Obstacle::hasCollision(const Line& otherLine, 
	std::vector<WorldPoint>& crossPoints)
{
	bool hasCross = false;
	for_each(lines.cbegin(), lines.cend(), [&hasCross, otherLine, &crossPoints]
	(Line line) {
		WorldPoint crossPoint;
		if (line.hasCollision(otherLine, crossPoint)) {
			crossPoints.push_back(crossPoint);
			hasCross = true;
		}
	});
	return hasCross;
}
bool RRT_star::world::geometry::Obstacle::hasCollision(const Line & otherLine)
{
	return std::any_of(lines.cbegin(), lines.cend(), [otherLine](Line line) 
	{ return line.hasCollision(otherLine); });
}

std::vector<RRT_star::world::geometry::Line> RRT_star::world::geometry::Obstacle::getLines()
{
	return lines;
}
