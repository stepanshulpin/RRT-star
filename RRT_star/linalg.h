#ifndef _LINALG_HH
#define _LINALG_HH
#include <vector>
#include <algorithm>
namespace RRT_star {
	namespace linalg {
		double vectorProduct(double ax, double ay, double bx, double by);
		bool hasCross(double l1ax, double l1ay, double l1bx, double l1by, 
			double l2ax, double l2ay, double l2bx, double l2by);
		class LineEquation {
		public:
			double A, B, C;
			LineEquation(double ax, double ay, double bx, double by);
		};
		std::pair<double, double> crossPoint(double l1ax, double l1ay, 
			double l1bx, double l1by, double l2ax, double l2ay, double l2bx, 
			double l2by);
		double distance(double ax, double ay, double bx, double by);
	}
}
#endif
