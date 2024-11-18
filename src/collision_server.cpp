#include <vector>
#include "collision_server.h"
#include "shapes.h"
using namespace KinSolver;

int CollisionServer::RegisterShape(Shape NewShape) {
	// Scan through shapes vector to find an empty space
	int TargetPos;
	for (int i = 0; i < Shapes.size(); i++) {
		if (Shapes[i] == nullptr) {
			TargetPos = i;
			Shapes[i] = &NewShape;
			return i;
		}
	}
	Shapes.push_back(&NewShape);
	return Shapes.size() - 1;
}
