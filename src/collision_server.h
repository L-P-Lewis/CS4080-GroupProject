// Collision Server
// Handles the storage of collision shapes and processes their movement

#include<vector>

#include "shapes.h"
#ifndef KINSOLVER_COLLISIONSERVER_H_
#define KINSOLVER_COLLISIONSERVER_H_

namespace KinSolver {
	typedef std::vector<int> ShapeList;
	struct SweepResult {
		double TravelPortion;
		Vector2 HitVector;
	};
	// The result of sweeping a shape through space, stopping when either the movment is exhausted, or a shape is hit.
	struct GlobalSweepResult {
		// The portion of the movement that was completed, ranges from 0.0-1.0
		SweepResult Result;
		// The ID of the shape collided with, -1 if no shape is hit
		int CollidedShape;
	};

	// TODO: Fill out this function with relevant data
	struct MoveAndSlideResult {
		
	};


	// The main collision server, holds a record of all physical shapes in the world and has methods for moving them
	class CollisionServer {
	public:
		CollisionServer();
		// Registers a shape in the collision server. Returns the shape's ID, which can be used as a lightweight reference
		// TODO: Make this take in a shape builder
		int RegisterShape(Shape NewShape);
		// Sweeps a shape through space, returning a Sweep result on completion
		// TODO: Make this take in a velocity vector
		GlobalSweepResult SweepShape(int ShapeID, Vector2 Velocity);
		// Gets all shapes that overlap the given bounding box and that are on a given layer.
		ShapeList GetShapesInAABB(AABB BoundingBox);
		// Move a shape through space through as much of it's velocity as possible. Sliding allong surfaces, and only completely stopping if it hits something head on.
		// TODO: Make this take in a velocity vector
//		MoveAndSlideResult MoveAndSlide(int ShapeID, Vector2 Velocity);
		static SweepResult TestCollideShapes(Shape Mover, Shape Target, Vector2 Movement);
	private:
		std::vector<Shape*> Shapes;

};
}

#endif
