#ifndef _PATH_PLANNER_HH
#define _PATH_PLANNER_HH
#include "WorldMap.h"
#include "Path.h"
namespace RRT_star {
	namespace path_planning {
		using namespace world;
		class PathPlanner {
		public:
			PathPlanner(WorldMap& _map);
			virtual Path findPath(WorldPoint startPoint, WorldPoint endPoint) = 0;
			virtual void rebuildPath(Path* path, WorldMap map) = 0;
		protected:
			WorldMap map;
		};
	}
}
#endif