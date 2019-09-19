#include "Tree.h"
using namespace RRT_star::path_planning::utils;

void Tree::init(WorldPoint startPoint)
{
	vertexes.push_back(startPoint);
	parents.push_back(-1);
	costs.push_back(0);
	tree_size = 1;
}
void Tree::add(WorldPoint point, int parent, double cost)
{
	vertexes.push_back(point);
	parents.push_back(parent);
	costs.push_back(cost);
	tree_size++;
}
std::vector<WorldPoint> Tree::getVertexes()
{
	return vertexes;
}
std::vector<int> Tree::getParents()
{
	return parents;
}
std::vector<double> Tree::getCosts()
{
	return costs;
}
int Tree::getSize()
{
	return tree_size;
}
void Tree::rewireVertex(int i, double newCost)
{
	parents[i] = tree_size;
	costs[i] = newCost;
}

void RRT_star::path_planning::utils::Tree::unwireVertex(int i)
{
	parents[i] = UNWIRED_VERTEX;

}

void RRT_star::path_planning::utils::Tree::setParent(int i, int parent)
{
	parents[i] = parent;
}

bool RRT_star::path_planning::utils::Tree::isInvalid(unsigned int i)
{
	bool tmp = parents[i] == UNWIRED_VERTEX;
	return parents[i] == UNWIRED_VERTEX;
}

