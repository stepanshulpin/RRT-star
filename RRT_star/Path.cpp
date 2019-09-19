#include "Path.h"
std::vector<RRT_star::path_planning::WorldPoint> 
RRT_star::path_planning::Path::getPoints() {
	return points;
}
void RRT_star::path_planning::Path::addPoint(RRT_star::path_planning::WorldPoint
	point) {
	points.push_back(point);
}
void RRT_star::path_planning::Path::clear() {
	points.clear();
}
