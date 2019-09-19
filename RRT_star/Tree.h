#ifndef _TREE_HH
#define _TREE_HH
#include "Line.h"
namespace RRT_star {
	namespace path_planning {
		namespace utils {
			using namespace world::geometry;
			class Tree {
			public:
				void init(WorldPoint startPoint);
				void add(WorldPoint point, int parent, double cost);
				std::vector<WorldPoint> getVertexes();
				std::vector<int> getParents();
				std::vector<double> getCosts();
				int getSize();
				void rewireVertex(int i, double newCost);
				void unwireVertex(int i);
				void setParent(int i, int parent);
				bool isInvalid(unsigned int i);
				Tree& operator= (const Tree &other)
				{
					this->tree_size = other.tree_size;
					this->vertexes = other.vertexes;
					this->parents = other.parents;
					this->costs = other.costs;
					return *this;
				}
			private:
				const int UNWIRED_VERTEX = -111;
				std::vector<WorldPoint> vertexes;
				std::vector<int> parents;
				std::vector<double> costs;
				int tree_size;
			};
		}
	}
}
#endif 
