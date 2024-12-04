#ifndef KINSOLVER_COLLISIONSERVER_H_
#define KINSOLVER_COLLISIONSERVER_H_

#include <vector>
#include "shapes.h"

namespace KinSolver {

    typedef std::vector<int> ShapeList;

    struct SweepResult {
        double TravelPortion;    // How far along the velocity the collision occurred
        Vector2 HitVector;       // Normal of the collision surface
    };

    struct GlobalSweepResult {
        SweepResult Result;      // Details of the closest collision
        int CollidedShape;       // Index of the collided shape, or -1 if none
    };

    struct MoveAndSlideResult {
        Vector2 FinalPosition;          // Final position of the shape
        Vector2 RemainingVelocity;      // Remaining velocity after sliding
        std::vector<Vector2> HitNormals; // Collision normals encountered
        int CollisionCount;             // Number of collisions
    };

    class CollisionServer {
    public:
        CollisionServer() : Collisions(0), LastEntryDirection(Vector2(0, 0)) {}

        int RegisterShape(Shape* NewShape);
        ShapeList GetShapesInAABB(AABB BoundingBox);
        GlobalSweepResult SweepShape(int ShapeID, Vector2 Velocity);
        MoveAndSlideResult MoveAndSlide(int ShapeID, Vector2 Velocity);

        static SweepResult TestCollideShapes(Shape* Mover, Shape* Target, Vector2 Movement);

    private:
        std::vector<Shape*> Shapes;       // Holds all registered shapes
        int Collisions;                   // Collision count
        Vector2 LastEntryDirection;       // Store the last collision direction
    };

}

#endif





