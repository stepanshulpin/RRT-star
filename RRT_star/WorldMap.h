#ifndef _WORLDMAP_HH
#define _WORLDMAP_HH
#include "TriangleObstacle.h"
#include "BoxObstacle.h"
namespace RRT_star {
	namespace world {
		using namespace geometry;
		class WorldMap {
		public:
			WorldMap(WorldPoint _minPlace = WorldPoint(-10, -10), WorldPoint _maxPlace = WorldPoint(10, 10));
			void addObstacle(Obstacle obstacle);
			bool hasCollision(const Line& line);
			bool hasCollision(const Line& line, std::vector<WorldPoint>& crossPoints);
			WorldPoint getMinPlace();
			WorldPoint getMaxPlace();
			std::vector<Obstacle> getObstacles();
		private:
			WorldPoint minPlace;
			WorldPoint maxPlace;
			std::vector<Obstacle> obstacles;
		};
	}
}
#endif
