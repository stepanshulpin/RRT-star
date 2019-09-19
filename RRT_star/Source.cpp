#include <iostream>
#include <windows.h>

#include "TriangleObstacle.h"
#include "BoxObstacle.h"
#include "WorldMap.h"
#include "RRTPathPlanner.h"

using namespace RRT_star::world;
using namespace RRT_star::world::geometry;
using namespace RRT_star::path_planning;


std::pair<int, int> getPointPosition(double x, double y, int width, int height, WorldPoint minPlace, WorldPoint maxPlace) {
	double xScale = width / (maxPlace.first - minPlace.first);
	double yScale = height / (maxPlace.second - minPlace.second);
	double xZero = -minPlace.first * xScale;
	double yZero = -minPlace.second * yScale;
	int xNew = xZero + x * xScale;
	int yNew = yZero - y * yScale;
	return std::pair<int, int>(xNew, yNew);
}

void drawLine(Line line, int width, int height, WorldPoint minPlace, WorldPoint maxPlace, HDC hdc) {
	std::pair<int, int> firstPoint = getPointPosition(line.getA().first, line.getA().second, width, height, minPlace, maxPlace);
	std::pair<int, int> secondPoint = getPointPosition(line.getB().first, line.getB().second, width, height, minPlace, maxPlace);
	MoveToEx(hdc, firstPoint.first, firstPoint.second, NULL);
	LineTo(hdc, secondPoint.first, secondPoint.second);
}


void drawObstacle(Obstacle obs, int width, int height, WorldPoint minPlace, WorldPoint maxPlace, HDC hdc) {
	for (Line &line : obs.getLines()) {
		drawLine(line, width, height, minPlace, maxPlace, hdc);
	}
}

void drawMap(WorldMap map, int width, int height, HDC hdc) {
	for (auto &obs : map.getObstacles()) {
		drawObstacle(obs, width, height, map.getMinPlace(), map.getMaxPlace(), hdc);
	}
}

void drawPoint(HDC hdc, std::pair<int, int> point) {
	Ellipse(hdc, point.first - 2, point.second + 2, point.first + 2, point.second - 2);
}

void drawStartAndEnd(WorldPoint start, WorldPoint end, WorldMap map, int width, int height, HDC hdc) {
	std::pair<int, int> startPoint = getPointPosition(start.first, start.second, width, height, map.getMinPlace(), map.getMaxPlace());
	std::pair<int, int> endPoint = getPointPosition(end.first, end.second, width, height, map.getMinPlace(), map.getMaxPlace());
	HGDIOBJ hOld = SelectObject(hdc, GetStockObject(DC_PEN));
	SetDCPenColor(hdc, RGB(0, 255, 0));
	drawPoint(hdc, startPoint);
	SetDCPenColor(hdc, RGB(255, 0, 0));
	drawPoint(hdc, endPoint);
	SelectObject(hdc, hOld);
}

void drawTree(Tree tree, WorldMap map, int width, int height, HDC hdc) {	
	for (int i = 0; i < tree.getSize(); i++) {
		WorldPoint currentPoint = tree.getVertexes()[i];
		int parentInd = tree.getParents()[i];
		if (parentInd == -1) {
			std::pair<int, int> point = getPointPosition(currentPoint.first, currentPoint.second, width, height, map.getMinPlace(), map.getMaxPlace());
			drawPoint(hdc, point);
		}
		else if (parentInd == -111) {

		}
		else {
			WorldPoint parent = tree.getVertexes()[parentInd];
			Line line(parent, currentPoint);
			drawLine(line, width, height, map.getMinPlace(), map.getMaxPlace(), hdc);
		}
	}
}

void drawPath(Path path, WorldMap map, int width, int height, HDC hdc) {
	HPEN hPen = CreatePen(PS_SOLID, 5, RGB(255, 0, 0));
	SelectObject(hdc, hPen);
	for (unsigned int i = 0; i < path.getPoints().size() - 1; i++) {
		WorldPoint currentPoint = path.getPoints()[i];
		WorldPoint nextPoint = path.getPoints()[i + 1];
		Line line(currentPoint, nextPoint);
		drawLine(line, width, height, map.getMinPlace(), map.getMaxPlace(), hdc);
	}
}

void drawNewObstacle(Obstacle obs, WorldMap map, int width, int height, HDC hdc) {
	HPEN hPen = CreatePen(PS_SOLID, 3, RGB(0, 255, 0));
	SelectObject(hdc, hPen);
	drawObstacle(obs, width, height, map.getMinPlace(), map.getMaxPlace(), hdc);
}

int main() {

	Obstacle* A = new TriangleObstacle(WorldPoint(-6, -6), WorldPoint(-3, -6), WorldPoint(-6, 0));
	Obstacle* B = new BoxObstacle(WorldPoint(-2, -2), WorldPoint(2, 2));
	Obstacle* C = new TriangleObstacle(WorldPoint(3, 4), WorldPoint(3, 8), WorldPoint(7, 8));
	Obstacle* D = new TriangleObstacle(WorldPoint(4, 2), WorldPoint(9, 2), WorldPoint(9, 7));
	Obstacle* E = new BoxObstacle(WorldPoint(3, 0), WorldPoint(6, 1));
	WorldMap map(WorldPoint(-10, -10), WorldPoint(10, 10));

	map.addObstacle(*A);
	map.addObstacle(*B);
	map.addObstacle(*C);
	map.addObstacle(*D);
	WorldPoint start(-7, -7);
	WorldPoint end(7, 7);

	RRTPathPlanner* pathPlanner = new RRTPathPlanner(map);
	pathPlanner->setSteps(1000);
	Path path = pathPlanner->findPath(start, end);

	HWND hWnd = GetConsoleWindow();

	RECT lprect;
	GetClientRect(hWnd, &lprect);
	int width = lprect.right;
	int height = lprect.bottom;
	HDC hDC = GetDC(hWnd);
	SelectObject(hDC, GetStockObject(BLACK_PEN));

	drawMap(map, width, height, hDC);
	drawStartAndEnd(start, end, map, width, height, hDC);

	Tree tree = pathPlanner->getTree();
	HPEN hPen1 = CreatePen(PS_DOT, 1, RGB(0, 0, 70));
	SelectObject(hDC, hPen1);
	drawTree(tree, map, width, height, hDC);


	std::cin.get();
	//drawPath(path, map, width, height, hDC);

	std::cin.get();

	map.addObstacle(*E);
	drawNewObstacle(*E, map, width, height, hDC);

	std::cin.get();

	pathPlanner->rebuildPath(&path, map);

	Tree newTree = pathPlanner->getTree();
	HPEN hPen2 = CreatePen(PS_SOLID, 1, RGB(50, 50, 200));
	SelectObject(hDC, hPen2);
	drawTree(newTree, map, width, height, hDC);

	std::cin.get();
	/*for (auto &i : path.getPoints()) {
		std::cout << i.first << " " << i.second << std::endl;
	}*/

	return 0;
}


