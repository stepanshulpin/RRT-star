#ifndef _PATH_HH
#define _PATH_HH
#include "Line.h"
namespace RRT_star {
	namespace path_planning {
		using namespace world::geometry;
		class Path {
		public:
			std::vector<WorldPoint> getPoints();
			void addPoint(WorldPoint point);
			void clear();
		private:
			std::vector<WorldPoint> points;
		};
	}
}
#endif