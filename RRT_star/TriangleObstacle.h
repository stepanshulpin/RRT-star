#ifndef _TRIANGLE_OBSTACLE_HH
#define _TRIANGLE_OBSTACLE_HH
#include "Obstacle.h"
namespace RRT_star {
	namespace world {
		namespace geometry {
			class TriangleObstacle : public Obstacle {
			public:
				TriangleObstacle(WorldPoint pointA, WorldPoint pointB, WorldPoint pointC);
				~TriangleObstacle() {}
			};
		}
	}
}
#endif
