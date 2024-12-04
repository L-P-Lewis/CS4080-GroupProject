/*
Names: Luke Lewis, Erick Hambardzumyan
Class: CS 4080
Assignment: Group Project- Collision Detection System
Date: 12-05-2024
*/

//collision_server.cpp
//class implementation for the collision server

#include "collision_server.h"
#include "shapes.h"
#include <set>
#include <tuple>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <string>
#include <iomanip>
using namespace KinSolver;

#define Range std::tuple<double, double>

// RegisterShape: Adds a new shape to the Shapes vector. If a spot is empty, the shape is added there. 
// If no empty spots exist, it appends the shape to the list.
int CollisionServer::RegisterShape(Shape* NewShape) {
    for (size_t i = 0; i < Shapes.size(); ++i) {
        if (Shapes[i] == nullptr) {
            Shapes[i] = NewShape;  // Place shape in empty spot
            return static_cast<int>(i);
        }
    }
    Shapes.push_back(NewShape);  // No empty spot, append shape
    return static_cast<int>(Shapes.size() - 1);
}

// GetShapesInAABB: This function checks which shapes overlap with the given AABB (Axis-Aligned Bounding Box).
// It returns a list of shapes that are inside the bounding box.
ShapeList CollisionServer::GetShapesInAABB(AABB BoundingBox) {
    ShapeList OverlappingShapes;

    for (size_t i = 0; i < Shapes.size(); ++i) {
        if (Shapes[i] == nullptr) continue;

        // Get the AABB of the current shape.
        AABB ShapeAABB = Shapes[i]->GetSweptAABB(Vector2());

        // Check if the bounding boxes overlap.
        if (BoundingBox.X <= ShapeAABB.X + ShapeAABB.Width &&
            BoundingBox.X + BoundingBox.Width >= ShapeAABB.X &&
            BoundingBox.Y <= ShapeAABB.Y + ShapeAABB.Height &&
            BoundingBox.Y + BoundingBox.Height >= ShapeAABB.Y) {
            OverlappingShapes.push_back(static_cast<int>(i));  // Add overlapping shape to the list
        }
    }

    return OverlappingShapes;
}

// SweepShape: This function tests for the closest collision of a shape as it moves along a given velocity.
// It returns the closest collision result along with the collided shape's index.
GlobalSweepResult CollisionServer::SweepShape(int ShapeID, Vector2 Velocity) {
    if (ShapeID < 0 || ShapeID >= static_cast<int>(Shapes.size()) || Shapes[ShapeID] == nullptr) {
        throw std::runtime_error("Invalid ShapeID");  // Error if the shape ID is invalid
    }

    Shape* Mover = Shapes[ShapeID];  // Get the shape to move
    ShapeList PotentialCollisions = GetShapesInAABB(Mover->GetSweptAABB(Velocity));  // Get shapes that overlap with the swept AABB
    SweepResult ClosestHit = { 1.0, Velocity };  // Initialize the closest hit with no collision
    int ClosestObject = -1;  // Initialize no object as collided

    // Loop through the potential collisions to find the closest one
    for (int i : PotentialCollisions) {
        if (i == ShapeID) continue;  // Skip checking the shape against itself

        SweepResult Hit = TestCollideShapes(Mover, Shapes[i], Velocity);  // Test for collision
        if (Hit.TravelPortion < ClosestHit.TravelPortion) {  // Update closest collision if this one is closer
            ClosestHit = Hit;
            ClosestObject = i;
        }
    }

    return { ClosestHit, ClosestObject };  // Return the closest collision and the collided object
}

// MoveAndSlide: Moves the shape along a velocity while handling collisions. It iterates up to a maximum number of times (MaxIterations)
// and adjusts the shape's position and velocity after each collision.
MoveAndSlideResult CollisionServer::MoveAndSlide(int ShapeID, Vector2 Velocity) {
    Shape* Mover = Shapes[ShapeID];  // Get the shape to move
    Vector2 CurrentPosition = Mover->Position;  // Get current position
    Vector2 RemainingVelocity = Velocity;  // Initialize remaining velocity
    std::vector<Vector2> HitNormals;  // Store collision normals
    int Collisions = 0;  // Initialize collision counter

    const int MaxIterations = 10;  // Set max iterations for movement
    const double Epsilon = 1e-6;  // Set a threshold for minimal velocity to stop the movement
    const double PositionChangeThreshold = 1e-4;  // Set a threshold to stop the movement if the position change is too small
    bool DebugMode = false;  // Debug mode flag

    Vector2 PreviousPosition = CurrentPosition;  // Initialize previous position

    // Iterating through the movement process
    for (int i = 0; i < MaxIterations; ++i) {
        GlobalSweepResult Sweep = SweepShape(ShapeID, RemainingVelocity);  // Get the sweep result for the current velocity

        std::cout << "Iteration " << i << ":\n";
        std::cout << "  - Travel Portion: " << Sweep.Result.TravelPortion << "\n";
        std::cout << "  - Remaining Velocity Length: " << RemainingVelocity.Length() << "\n";

        if (Sweep.Result.TravelPortion >= 1.0 || RemainingVelocity.Length() < Epsilon) {
            std::cout << "  - No further collisions or minimal velocity. Exiting loop.\n";
            break;  // Exit the loop if no collision or velocity is too small
        }

        Vector2 HitNormal = Sweep.Result.HitVector.Normalized();  // Get the hit normal
        if (HitNormal.Length() > 0.0) {
            std::cout << "  - Collision Detected!\n";
            std::cout << "    * Hit Normal: ("
                << (HitNormal.X == 0.0 ? 0 : HitNormal.X) << ", "
                << (HitNormal.Y == 0.0 ? 0 : HitNormal.Y) << ")\n";  // Print 0 instead of -0.0 for hit normal
            HitNormals.push_back(HitNormal);  // Add the hit normal to the list
            Collisions++;  // Increment collision count
        }

        // Update the current position and remaining velocity after collision
        CurrentPosition = CurrentPosition + RemainingVelocity * Sweep.Result.TravelPortion;
        double OverlapDepth = std::max(Epsilon, RemainingVelocity.Length() * 0.8);  // Calculate the overlap depth
        CurrentPosition = CurrentPosition + HitNormal * OverlapDepth;  // Update position after overlap

        // Calculate the new velocity for sliding along the surface
        Vector2 PerpendicularComponent = HitNormal * Vector2::Vector2Dot(RemainingVelocity, HitNormal);
        Vector2 SlideVelocity = RemainingVelocity - PerpendicularComponent;

        // Validate the slide movement
        GlobalSweepResult ValidationSweep = SweepShape(ShapeID, SlideVelocity);
        if (ValidationSweep.Result.TravelPortion < 1.0) {
            SlideVelocity = SlideVelocity * ValidationSweep.Result.TravelPortion;  // Adjust the slide velocity
        }

        if (SlideVelocity.Length() < Epsilon) {
            RemainingVelocity = Vector2(0, 0);  // Stop moving if velocity is too small
            break;
        }

        RemainingVelocity = SlideVelocity;  // Update remaining velocity
        if ((CurrentPosition - PreviousPosition).Length() < PositionChangeThreshold) {
            std::cout << "  - Minimal position change detected. Exiting loop.\n";
            break;  // Exit if position does not change significantly
        }

        PreviousPosition = CurrentPosition;  // Update previous position

        // Check if consecutive collisions are too similar, indicating a potential deadlock
        if (HitNormals.size() > 2 &&
            std::abs(Vector2::Vector2Dot(HitNormals.back(), HitNormals[HitNormals.size() - 2])) > 0.99) {
            std::cout << "  - Similar consecutive collision normals detected. Exiting loop.\n";
            break;
        }
    }

    Mover->Position = CurrentPosition;  // Set final position

    // Output the summary of the MoveAndSlide result
    std::cout << "\nMoveAndSlide Summary:\n";
    std::cout << "  - Total Collisions: " << Collisions << "\n";
    std::cout << "  - Unique Collision Normals:\n";

    std::set<Vector2> UniqueNormals(HitNormals.begin(), HitNormals.end());
    for (const auto& Normal : UniqueNormals) {
        std::cout << "    * ("
            << (Normal.X == 0.0 ? 0 : Normal.X) << ", "  // Print 0 instead of -0.0 for collision normals
            << (Normal.Y == 0.0 ? 0 : Normal.Y) << ")\n";   // Print 0 instead of -0.0 for collision normals
    }

    std::cout << "  - Final Position: (" << CurrentPosition.X << ", " << CurrentPosition.Y << ")\n";
    std::cout << "  - Remaining Velocity: (" << RemainingVelocity.X << ", " << RemainingVelocity.Y << ")\n";

    return { CurrentPosition, RemainingVelocity, HitNormals, Collisions };
}

// TestCollideShapes: Checks for a collision between two shapes along a given movement. 
// It projects both shapes onto the axes of the current shapes and calculates the entry and exit times.
SweepResult CollisionServer::TestCollideShapes(Shape* Mover, Shape* Target, Vector2 Movement) {
    std::set<Vector2> Axes;
    auto MoverAxes = Mover->GetSeparationAxes();  // Get the separation axes for the mover
    auto TargetAxes = Target->GetSeparationAxes();  // Get the separation axes for the target

    // Combine axes from both shapes
    Axes.insert(MoverAxes.begin(), MoverAxes.end());
    Axes.insert(TargetAxes.begin(), TargetAxes.end());

    double FirstExit = 1.0;  // Fraction of movement where collision ends
    double LastEntry = 0.0; // Fraction of movement where collision starts
    Vector2 LastEntryDirection(0, 0); // Direction of the last valid collision entry

    for (const auto& Axis : Axes) {
        // Project both shapes onto the current axis
        Range MoverProjection = Mover->ProjectShape(Axis);
        Range TargetProjection = Target->ProjectShape(Axis);

        double MovementDot = Vector2::Vector2Dot(Movement, Axis);  // Calculate dot product
        // Print Axis and Projections. If component is +-0.0, print 0
        std::cout << "Axis: ("
            << (Axis.X == 0.0 ? 0 : Axis.X) << ", "
            << (Axis.Y == 0.0 ? 0 : Axis.Y) << ")\n";  // Print 0 instead of -0.0 for the axis
        std::cout << "Mover Projection: ["
            << (std::get<0>(MoverProjection) == 0.0 ? 0 : std::get<0>(MoverProjection)) << ", "
            << (std::get<1>(MoverProjection) == 0.0 ? 0 : std::get<1>(MoverProjection)) << "]\n";  // Handle projection components
        std::cout << "Target Projection: ["
            << (std::get<0>(TargetProjection) == 0.0 ? 0 : std::get<0>(TargetProjection)) << ", "
            << (std::get<1>(TargetProjection) == 0.0 ? 0 : std::get<1>(TargetProjection)) << "]\n";  // Handle projection components

        // Check for overlap on the current axis
        if (std::get<1>(MoverProjection) >= std::get<0>(TargetProjection) &&
            std::get<0>(MoverProjection) <= std::get<1>(TargetProjection)) {
            std::cout << "Projections overlap.\n";  // Overlap detected
        }
        else {
            std::cout << "Projections do not overlap.\n";
            return { 1.0, Vector2(0, 0) };  // No collision on this axis
        }

        // Handle parallel movement (dot product close to 0)
        if (std::abs(MovementDot) < 1e-6) {
            std::cout << "Small MovementDot: " << MovementDot << "\n";
            LastEntryDirection = Axis.Normalized();  // Normalize the axis direction
            return { 0.0, LastEntryDirection };  // Immediate collision if movement is parallel
        }

        // Calculate entry and exit times for this axis
        double Entry = (std::get<0>(TargetProjection) - std::get<1>(MoverProjection)) / MovementDot;
        double Exit = (std::get<1>(TargetProjection) - std::get<0>(MoverProjection)) / MovementDot;

        if (Entry > Exit) std::swap(Entry, Exit);  // Swap if entry is after exit

        FirstExit = std::min(FirstExit, Exit);  // Update the earliest exit time
        if (LastEntry < Entry) {
            LastEntry = Entry;
            LastEntryDirection = Axis;  // Update the direction of the last entry
        }

        // Check for separation on this axis
        if (FirstExit <= LastEntry) {
            std::cout << "Separation detected on axis: (" << Axis.X << ", " << Axis.Y << ")\n";
            return { 1.0, Vector2(0, 0) };  // No collision if separated
        }
    }

    if (LastEntry >= 0.0 && LastEntry < 1.0) {
        LastEntryDirection = LastEntryDirection.Normalized();
        std::cout << "Valid Collision Detected:\n";
        std::cout << "  - Last Entry: " << LastEntry << "\n";
        std::cout << "  - Direction: (" << LastEntryDirection.X << ", " << LastEntryDirection.Y << ")\n";
        return { LastEntry, LastEntryDirection };  // Return the valid collision entry and direction
    }

    return { 1.0, Vector2(0, 0) };  // No collision
}
