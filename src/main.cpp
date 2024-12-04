#include "collision_server.h"
#include "shapes.h"
#include <iostream>

using namespace KinSolver;

int main() {
    std::cout << "=== Collision Solver Test ===\n";

    CollisionServer Server;

    Polygon Shape1;
    Shape1.Points = {
        Vector2(-0.5, -0.5),
        Vector2(-0.5, 0.5),
        Vector2(0.5, 0.5),
        Vector2(0.5, -0.5)
    };
    Shape1.Position = Vector2(0.0, 0.0);
    int Shape1Handle = Server.RegisterShape(&Shape1);

    Polygon Shape2;
    Shape2.Points = {
        Vector2(-0.5, -0.5),
        Vector2(-0.5, 0.5),
        Vector2(0.5, 0.5),
        Vector2(0.5, -0.5)
    };
    Shape2.Position = Vector2(0.5, 0.0);  // Close enough for collision
    int Shape2Handle = Server.RegisterShape(&Shape2);

    Vector2 Velocity(-1.0, 0.0);  // Move Shape1 left towards Shape2
    MoveAndSlideResult Result = Server.MoveAndSlide(Shape1Handle, Velocity);

    std::cout << "\n=== Test Summary ===\n";
    std::cout << "Final Position: (" << Result.FinalPosition.X << ", " << Result.FinalPosition.Y << ")\n";
    std::cout << "Remaining Velocity: (" << Result.RemainingVelocity.X << ", " << Result.RemainingVelocity.Y << ")\n";
    std::cout << "Collisions Detected: " << Result.CollisionCount << "\n";

    // Print collision normals
    std::cout << "Collision Normals:\n";
    for (const auto& Normal : Result.HitNormals) {
        std::cout << "  * (" << Normal.X << ", " << Normal.Y << ")\n";
    }

    return 0;
}

