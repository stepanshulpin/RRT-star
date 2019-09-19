#ifndef _LINE_HH
#define _LINE_HH
#include "linalg.h"
namespace RRT_star {
	namespace world {
		namespace geometry {
			using WorldPoint = std::pair<double, double>;
			class Line {
			public:
				Line(WorldPoint _pointA, WorldPoint _pointB);
				bool hasCollision(const Line& otherLine, WorldPoint& crossPoint);
				bool hasCollision(const Line& otherLine);
				WorldPoint getA();
				WorldPoint getB();
			private:
				WorldPoint pointA;
				WorldPoint pointB;
			};
		}
	}
}
#endif
