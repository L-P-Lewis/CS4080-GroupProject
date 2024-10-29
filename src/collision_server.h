// Collision Server
// Handles the storage of collision shapes and processes their movement

#include<vector>

#ifndef KINSOLVER_COLLISIONSERVER_H_
#define KINSOLVER_COLLISIONSERVER_H_

namespace KinSolver {
	// The result of sweeping a shape through space, stopping when either the movment is exhausted, or a shape is hit.
	struct SweepResult {
		// The portion of the movement that was completed, ranges from 0.0-1.0
		double TravelPortion;
		// The ID of the shape collided with, -1 if no shape is hit
		int CollidedShape;
	};

	// TODO: Fill out this function with relevant data
	struct MoveAndSlideResult {
		
	};

	// Represents an axis alligned bounding box
	struct AABB {
		double X;
		double Y;
		double Width;
		double Height;
	};

	// The main collision server, holds a record of all physical shapes in the world and has methods for moving them
	class CollisionServer {
	public:
		// Registers a shape in the collision server. Returns the shape's ID, which can be used as a lightweight reference
		// TODO: Make this take in a shape builder
		int RegisterShape();
		// Sweeps a shape through space, returning a Sweep result on completion
		// TODO: Make this take in a velocity vector
		SweepResult SweepShape(int ShapeID);
		// Gets all shapes that overlap the given bounding box and that are on a given layer.
		std::vector<int> GetShapesInAABB(AABB BoundingBox, int Layer);
		// Move a shape through space through as much of it's velocity as possible. Sliding allong surfaces, and only completely stopping if it hits something head on.
		// TODO: Make this take in a velocity vector
		MoveAndSlideResult MoveAndSlide(int ShapeID);
};
}

#endif
