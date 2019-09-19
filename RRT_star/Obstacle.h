#ifndef _OBSTACLE_HH
#define _OBSTACLE_HH
#include "Line.h"
namespace RRT_star {
	namespace world {
		namespace geometry {
			class Obstacle {
			public:
				bool hasCollision(const Line& otherLine, 
					std::vector<WorldPoint>& crossPoints);
				bool hasCollision(const Line& otherLine);
				std::vector<Line> getLines();
				virtual ~Obstacle() {};
			protected:
				std::vector<Line> lines;
			};
		}
	}
}
#endif
