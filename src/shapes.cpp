/*
Names: Luke Lewis, Erick Hambardzumyan
Class: CS 4080
Assignment: Group Project- Collision Detection System
Date: 12-05-2024
*/

//shapes.cpp
//class implementation for the shapes used in the collision system

#include "shapes.h"
#include <algorithm>
#include <cmath>
#include <iostream>

using namespace KinSolver;

// Scalar multiplication operator for Vector2
Vector2 Vector2::operator*(double scalar) const {
    return Vector2(X * scalar, Y * scalar);  // Multiply each component of the vector by the scalar
}

// Vector addition operator for Vector2
Vector2 Vector2::operator+(const Vector2& Other) const {
    return Vector2(X + Other.X, Y + Other.Y);  // Add the corresponding components of the vectors
}

// Vector subtraction operator for Vector2
Vector2 Vector2::operator-(const Vector2& Other) const {
    return Vector2(X - Other.X, Y - Other.Y);  // Subtract the corresponding components of the vectors
}

// Less-than comparison operator for Vector2 (for sorting or comparisons)
bool Vector2::operator<(const Vector2& Other) const {
    if (X != Other.X)
        return X < Other.X;  // If X components are different, compare X
    return Y < Other.Y;  // If X components are equal, compare Y
}

// Equality comparison operator for Vector2
bool Vector2::operator==(const Vector2& Other) const {
    return X == Other.X && Y == Other.Y;  // Return true if both X and Y components are equal
}

// Normalize the vector (make its length equal to 1, if possible)
Vector2 Vector2::Normalized() const {
    double length = Length();  // Get the length (magnitude) of the vector
    if (length == 0.0) return Vector2(0.0, 0.0);  // Avoid division by zero if the vector is a zero vector
    return Vector2(X / length, Y / length);  // Divide each component by the length to normalize it
}

// Get the length (magnitude) of the vector
double Vector2::Length() const {
    return std::sqrt(X * X + Y * Y);  // Pythagorean theorem to calculate the length of the vector
}

// Calculate the dot product of two vectors
double Vector2::Vector2Dot(const Vector2& A, const Vector2& B) {
    return A.X * B.X + A.Y * B.Y;  // Multiply corresponding components and sum them
}

// Project the shape onto the given axis and return the min and max projections
std::tuple<double, double> Polygon::ProjectShape(Vector2 Axis) {
    double min = 1e9, max = -1e9;  // Initialize min and max with extreme values
    for (const auto& Point : Points) {
        double projection = Vector2::Vector2Dot(Point + Position, Axis);  // Project each point onto the axis
        min = std::min(min, projection);  // Update the minimum projection
        max = std::max(max, projection);  // Update the maximum projection
    }
    return { min, max };  // Return the min and max projections as a tuple
}

// Calculate the separation axes, by using the Separating Axis Theorem, for the polygon (normals of the edges)
std::vector<Vector2> Polygon::GetSeparationAxes() {
    std::vector<Vector2> Axes;  // Vector to store the separation axes (normals)
    for (size_t i = 0; i < Points.size(); ++i) {
        Vector2 Edge = Points[i] - Points[(i + 1) % Points.size()];  // Get the edge between consecutive points
        Vector2 Normal(-Edge.Y, Edge.X);  // The normal is perpendicular to the edge (2D cross product)
        Axes.push_back(Normal.Normalized());  // Normalize the normal and add it to the axes
    }
    return Axes;  // Return the list of separation axes
}

// Calculate the swept AABB (Axis-Aligned Bounding Box) for the polygon considering movement
AABB Polygon::GetSweptAABB(Vector2 Movement) {
    double MinX = 1e9, MinY = 1e9, MaxX = -1e9, MaxY = -1e9;  // Initialize min and max with extreme values
    for (const auto& Point : Points) {
        Vector2 AdjustedPoint = Point + Position;  // Adjust point position by the shape's position
        MinX = std::min(MinX, AdjustedPoint.X);  // Update the min X value
        MinY = std::min(MinY, AdjustedPoint.Y);  // Update the min Y value
        MaxX = std::max(MaxX, AdjustedPoint.X);  // Update the max X value
        MaxY = std::max(MaxY, AdjustedPoint.Y);  // Update the max Y value
    }

    // Adjust the AABB based on the movement (swept AABB concept)
    if (Movement.X > 0) MaxX += Movement.X;
    else MinX += Movement.X;
    if (Movement.Y > 0) MaxY += Movement.Y;
    else MinY += Movement.Y;

    std::cout << "Swept AABB: Min(" << MinX << ", " << MinY << "), Max(" << MaxX << ", " << MaxY << ")\n";
    return { MinX, MinY, MaxX - MinX, MaxY - MinY };  // Return the AABB (min, min, width, height)
}


