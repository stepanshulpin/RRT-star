#include "linalg.h"
double RRT_star::linalg::vectorProduct(double ax, double ay, double bx, double by)
{
	return ax*by - bx*ay;
}
bool RRT_star::linalg::hasCross(double l1ax, double l1ay, double l1bx, 
	double l1by, double l2ax, double l2ay, double l2bx, double l2by)
{
	double v1 = vectorProduct(l2bx - l2ax, l2by - l2ay, l1ax - l2ax, l1ay - l2ay);
	double v2 = vectorProduct(l2bx - l2ax, l2by - l2ay, l1bx - l2ax, l1by - l2ay);
	double v3 = vectorProduct(l1bx - l1ax, l1by - l1ay, l2ax - l1ax, l2ay - l1ay);
	double v4 = vectorProduct(l1bx - l1ax, l1by - l1ay, l2bx - l1ax, l2by - l1ay);
	if ((v1*v2) <= 0 && (v3*v4) <= 0)
		return true;
	return false;
}
std::pair<double, double> RRT_star::linalg::crossPoint(double l1ax, double l1ay,
	double l1bx, double l1by, double l2ax, double l2ay, double l2bx, double l2by)
{
	LineEquation le1(l1ax, l1ay, l1bx, l1by);
	LineEquation le2(l2ax, l2ay, l2bx, l2by);
	std::pair<double, double> point;
	double d = le1.A*le2.B - le1.B*le2.A;
	double dx = -le1.C*le2.B + le1.B*le2.C;
	double dy = -le1.A*le2.C + le1.C*le2.A;
	point.first = dx / d;
	point.second = dy / d;
	return point;
}
double RRT_star::linalg::distance(double ax, double ay, double bx, double by)
{
	return std::sqrt(std::pow((ax - bx), 2) + std::pow((ay - by), 2));
}
RRT_star::linalg::LineEquation::LineEquation(double ax, double ay, double bx, 
	double by)
{
	A = by - ay;
	B = ax - bx;
	C = -ax*(by - ay) + ay*(bx - ax);
}
