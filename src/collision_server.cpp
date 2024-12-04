#include "collision_server.h"
#include "shapes.h"
#include <set>
#include <tuple>
#include <algorithm>
#include <iostream>
#include <cmath>
using namespace KinSolver;

#define Range std::tuple<double, double>

int CollisionServer::RegisterShape(Shape* NewShape) {
    for (size_t i = 0; i < Shapes.size(); ++i) {
        if (Shapes[i] == nullptr) {
            Shapes[i] = NewShape;
            return static_cast<int>(i);
        }
    }
    Shapes.push_back(NewShape);
    return static_cast<int>(Shapes.size() - 1);
}

ShapeList CollisionServer::GetShapesInAABB(AABB BoundingBox) {
    ShapeList OverlappingShapes;

    for (size_t i = 0; i < Shapes.size(); ++i) {
        if (Shapes[i] == nullptr) continue;

        AABB ShapeAABB = Shapes[i]->GetSweptAABB(Vector2());
        if (BoundingBox.X <= ShapeAABB.X + ShapeAABB.Width &&
            BoundingBox.X + BoundingBox.Width >= ShapeAABB.X &&
            BoundingBox.Y <= ShapeAABB.Y + ShapeAABB.Height &&
            BoundingBox.Y + BoundingBox.Height >= ShapeAABB.Y) {
            OverlappingShapes.push_back(static_cast<int>(i));
        }
    }

    return OverlappingShapes;
}

GlobalSweepResult CollisionServer::SweepShape(int ShapeID, Vector2 Velocity) {
    if (ShapeID < 0 || ShapeID >= static_cast<int>(Shapes.size()) || Shapes[ShapeID] == nullptr) {
        throw std::runtime_error("Invalid ShapeID");
    }

    Shape* Mover = Shapes[ShapeID];
    ShapeList PotentialCollisions = GetShapesInAABB(Mover->GetSweptAABB(Velocity));
    SweepResult ClosestHit = { 1.0, Velocity };
    int ClosestObject = -1;

    for (int i : PotentialCollisions) {
        if (i == ShapeID) continue;

        SweepResult Hit = TestCollideShapes(Mover, Shapes[i], Velocity);
        if (Hit.TravelPortion < ClosestHit.TravelPortion) {
            ClosestHit = Hit;
            ClosestObject = i;
        }
    }

    return { ClosestHit, ClosestObject };
}

MoveAndSlideResult CollisionServer::MoveAndSlide(int ShapeID, Vector2 Velocity) {
    Shape* Mover = Shapes[ShapeID];
    Vector2 CurrentPosition = Mover->Position;
    Vector2 RemainingVelocity = Velocity;
    std::vector<Vector2> HitNormals;
    int Collisions = 0;

    const int MaxIterations = 10;
    const double Epsilon = 1e-6;
    const double PositionChangeThreshold = 1e-4;
    bool DebugMode = false;  // Toggle this flag to switch modes

    Vector2 PreviousPosition = CurrentPosition;

    for (int i = 0; i < MaxIterations; ++i) {
        GlobalSweepResult Sweep = SweepShape(ShapeID, RemainingVelocity);

        std::cout << "Iteration " << i << ":\n";
        std::cout << "  - Travel Portion: " << Sweep.Result.TravelPortion << "\n";
        std::cout << "  - Remaining Velocity Length: " << RemainingVelocity.Length() << "\n";

        if (Sweep.Result.TravelPortion >= 1.0 || RemainingVelocity.Length() < Epsilon) {
            std::cout << "  - No further collisions or minimal velocity. Exiting loop.\n";
            break;
        }

        Vector2 HitNormal = Sweep.Result.HitVector.Normalized();
        if (HitNormal.Length() > 0.0) {
            std::cout << "  - Collision Detected!\n";
            std::cout << "    * Hit Normal: (" << HitNormal.X << ", " << HitNormal.Y << ")\n";
            HitNormals.push_back(HitNormal);
            Collisions++;
        }

        CurrentPosition = CurrentPosition + RemainingVelocity * Sweep.Result.TravelPortion;
        double OverlapDepth = std::max(Epsilon, RemainingVelocity.Length() * 0.8);
        CurrentPosition = CurrentPosition + HitNormal * OverlapDepth;

        Vector2 PerpendicularComponent = HitNormal * Vector2::Vector2Dot(RemainingVelocity, HitNormal);
        Vector2 SlideVelocity = RemainingVelocity - PerpendicularComponent;

        GlobalSweepResult ValidationSweep = SweepShape(ShapeID, SlideVelocity);
        if (ValidationSweep.Result.TravelPortion < 1.0) {
            SlideVelocity = SlideVelocity * ValidationSweep.Result.TravelPortion;
        }

        if (SlideVelocity.Length() < Epsilon) {
            RemainingVelocity = Vector2(0, 0);
            break;
        }

        RemainingVelocity = SlideVelocity;
        if ((CurrentPosition - PreviousPosition).Length() < PositionChangeThreshold) {
            std::cout << "  - Minimal position change detected. Exiting loop.\n";
            break;
        }

        PreviousPosition = CurrentPosition;

        if (HitNormals.size() > 2 &&
            std::abs(Vector2::Vector2Dot(HitNormals.back(), HitNormals[HitNormals.size() - 2])) > 0.99) {
            std::cout << "  - Similar consecutive collision normals detected. Exiting loop.\n";
            break;
        }
    }

    Mover->Position = CurrentPosition;

    std::cout << "\nMoveAndSlide Summary:\n";
    std::cout << "  - Total Collisions: " << Collisions << "\n";
    std::cout << "  - Unique Collision Normals:\n";

    std::set<Vector2> UniqueNormals(HitNormals.begin(), HitNormals.end());
    for (const auto& Normal : UniqueNormals) {
        std::cout << "    * (" << Normal.X << ", " << Normal.Y << ")\n";
    }

    std::cout << "  - Final Position: (" << CurrentPosition.X << ", " << CurrentPosition.Y << ")\n";
    std::cout << "  - Remaining Velocity: (" << RemainingVelocity.X << ", " << RemainingVelocity.Y << ")\n";

    return { CurrentPosition, RemainingVelocity, HitNormals, Collisions };
}



SweepResult CollisionServer::TestCollideShapes(Shape* Mover, Shape* Target, Vector2 Movement) {
    std::set<Vector2> Axes;
    auto MoverAxes = Mover->GetSeparationAxes();
    auto TargetAxes = Target->GetSeparationAxes();

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

        double MovementDot = Vector2::Vector2Dot(Movement, Axis);

        std::cout << "Axis: (" << Axis.X << ", " << Axis.Y << ")\n";
        std::cout << "Mover Projection: [" << std::get<0>(MoverProjection) << ", " << std::get<1>(MoverProjection) << "]\n";
        std::cout << "Target Projection: [" << std::get<0>(TargetProjection) << ", " << std::get<1>(TargetProjection) << "]\n";

        // Check for overlap on the current axis
        if (std::get<1>(MoverProjection) >= std::get<0>(TargetProjection) &&
            std::get<0>(MoverProjection) <= std::get<1>(TargetProjection)) {
            std::cout << "Projections overlap.\n";
        }
        else {
            std::cout << "Projections do not overlap.\n";
            return { 1.0, Vector2(0, 0) };  // No collision on this axis
        }

        // Handle parallel movement (dot product close to 0)
        if (std::abs(MovementDot) < 1e-6) {
            std::cout << "Small MovementDot: " << MovementDot << "\n";
            LastEntryDirection = Axis.Normalized();
            return { 0.0, LastEntryDirection };  // Immediate collision
        }

        // Calculate entry and exit times for this axis
        double Entry = (std::get<0>(TargetProjection) - std::get<1>(MoverProjection)) / MovementDot;
        double Exit = (std::get<1>(TargetProjection) - std::get<0>(MoverProjection)) / MovementDot;

        if (Entry > Exit) std::swap(Entry, Exit);

        FirstExit = std::min(FirstExit, Exit);
        if (LastEntry < Entry) {
            LastEntry = Entry;
            LastEntryDirection = Axis;
        }

        // Check for separation on this axis
        if (FirstExit <= LastEntry) {
            std::cout << "Separation detected on axis: (" << Axis.X << ", " << Axis.Y << ")\n";
            return { 1.0, Vector2(0, 0) };  // No collision
        }
    }

    if (LastEntry >= 0.0 && LastEntry < 1.0) {
        LastEntryDirection = LastEntryDirection.Normalized();
        std::cout << "Valid Collision Detected:\n";
        std::cout << "  - Last Entry: " << LastEntry << "\n";
        std::cout << "  - Direction: (" << LastEntryDirection.X << ", " << LastEntryDirection.Y << ")\n";
        return { LastEntry, LastEntryDirection };
    }

    return { 1.0, Vector2(0, 0) };  // Default to no collision
}







