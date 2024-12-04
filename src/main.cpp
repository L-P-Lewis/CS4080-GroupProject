/*
Names: Luke Lewis, Erick Hambardzumyan
Class: CS 4080
Assignment: Group Project- Collision Detection System
Date: 12-05-2024
*/

//main.cpp
//client used for the program

#include "collision_server.h"
#include "shapes.h"
#include <iostream>
using namespace KinSolver;

int main() {
    std::cout << "=== Collision Solver Test ===\n";  // Display the start of the test

    CollisionServer Server;  // Create an instance of the CollisionServer to handle the simulation

    // Create Shape1 as a square and define its points (a rectangle with equal sides)
    Polygon Shape1;
    Shape1.Points = {
        Vector2(-0.5, -0.5),  // Bottom-left point
        Vector2(-0.5, 0.5),   // Top-left point
        Vector2(0.5, 0.5),    // Top-right point
        Vector2(0.5, -0.5)    // Bottom-right point
    };
    Shape1.Position = Vector2(0.0, 0.0);  // Set the position of Shape1 at the origin
    int Shape1Handle = Server.RegisterShape(&Shape1);  // Register Shape1 with the server

    // Create Shape2 as a square and define its points (same as Shape1 but offset in position)
    Polygon Shape2;
    Shape2.Points = {
        Vector2(-0.5, -0.5),  // Bottom-left point
        Vector2(-0.5, 0.5),   // Top-left point
        Vector2(0.5, 0.5),    // Top-right point
        Vector2(0.5, -0.5)    // Bottom-right point
    };
    Shape2.Position = Vector2(0.5, 0.0);  // Set the position of Shape2 to the right of Shape1 (close enough for a collision)
    int Shape2Handle = Server.RegisterShape(&Shape2);  // Register Shape2 with the server

    Vector2 Velocity(-1.0, 0.0);  // Set the velocity for Shape1 to move it left (towards Shape2)
    MoveAndSlideResult Result = Server.MoveAndSlide(Shape1Handle, Velocity);  // Perform the movement and collision detection for Shape1

    // Print the results of the collision test
    std::cout << "\n=== Test Summary ===\n";
    std::cout << "Final Position: (" << Result.FinalPosition.X << ", " << Result.FinalPosition.Y << ")\n";  // Display final position after movement and collisions
    std::cout << "Remaining Velocity: (" << Result.RemainingVelocity.X << ", " << Result.RemainingVelocity.Y << ")\n";  // Display remaining velocity after collision
    std::cout << "Collisions Detected: " << Result.CollisionCount << "\n";  // Display the number of collisions detected

    // Print the normals of the collisions detected
    std::cout << "Collision Normals:\n";
    for (const auto& Normal : Result.HitNormals) {
        std::cout << "  * ("
            << (Normal.X == 0.0 ? 0 : Normal.X) << ", "  // If component is + -0.0, print 0
            << (Normal.Y == 0.0 ? 0 : Normal.Y) << ")\n";   // If component is + -0.0, print 0
    }

    return 0;  // End of the program
}

