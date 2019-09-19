#ifndef _BOX_OBSTACLE_HH
#define _BOX_OBSTACLE_HH
#include "Obstacle.h"
namespace RRT_star {
	namespace world {
		namespace geometry {
			class BoxObstacle : public Obstacle {
			public:
				BoxObstacle(WorldPoint pointA, WorldPoint pointB, WorldPoint pointC, WorldPoint pointD);
				BoxObstacle(WorldPoint pointMin, WorldPoint pointMax);
				~BoxObstacle() {}
			};
		}
	}
}
#endif
