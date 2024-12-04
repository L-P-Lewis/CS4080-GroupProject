/*
Names: Luke Lewis, Erick Hambardzumyan
Class: CS 4080
Assignment: Group Project- Collision Detection System
Date: 12-05-2024
*/

//collision_server.h
//class declaration for the collision server

#ifndef KINSOLVER_COLLISIONSERVER_H_
#define KINSOLVER_COLLISIONSERVER_H_

#include <vector> // For using std::vector to store shapes
#include "shapes.h" // Assuming "shapes.h" contains definitions for Shape, Vector2, AABB, etc.

namespace KinSolver {

    // ShapeList is defined as a vector of integers representing shape IDs
    typedef std::vector<int> ShapeList;

    // SweepResult contains data for tracking how far along the velocity the collision occurred
    struct SweepResult {
        double TravelPortion;    // How far along the velocity the collision occurred
        Vector2 HitVector;       // Normal of the collision surface
    };

    // GlobalSweepResult holds the closest collision details and the shape that collided
    struct GlobalSweepResult {
        SweepResult Result;      // Details of the closest collision
        int CollidedShape;       // Index of the collided shape, or -1 if no collision
    };

    // MoveAndSlideResult holds information after a move and slide operation
    struct MoveAndSlideResult {
        Vector2 FinalPosition;          // Final position of the shape after movement
        Vector2 RemainingVelocity;      // Remaining velocity after sliding
        std::vector<Vector2> HitNormals; // Collision normals encountered during the process
        int CollisionCount;             // Number of collisions detected during the movement
    };

    // CollisionServer is the core class for handling shape registration and collision logic
    class CollisionServer {
    public:
        // Constructor to initialize the server and reset collision count and last entry direction
        CollisionServer() : Collisions(0), LastEntryDirection(Vector2(0, 0)) {}

        // Method to register a shape, returns a unique shape ID
        int RegisterShape(Shape* NewShape);

        // Method to get a list of shapes that overlap with a given AABB (Axis-Aligned Bounding Box)
        ShapeList GetShapesInAABB(AABB BoundingBox);

        // Method to perform a sweep test on a shape, returning the result of the closest collision
        GlobalSweepResult SweepShape(int ShapeID, Vector2 Velocity);

        // Method to move and slide a shape along its velocity while handling collisions
        MoveAndSlideResult MoveAndSlide(int ShapeID, Vector2 Velocity);

        // Static method to check if two shapes collide and return the result of the sweep test
        static SweepResult TestCollideShapes(Shape* Mover, Shape* Target, Vector2 Movement);

    private:
        std::vector<Shape*> Shapes;       // Holds all registered shapes in the server
        int Collisions;                   // Tracks the total number of collisions detected
        Vector2 LastEntryDirection;       // Stores the last valid collision direction for movement adjustment
    };

}

#endif

