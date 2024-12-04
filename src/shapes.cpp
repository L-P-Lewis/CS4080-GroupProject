#include "shapes.h"
#include <algorithm>
#include <cmath>
#include <iostream>

using namespace KinSolver;

Vector2 Vector2::operator*(double scalar) const {
    return Vector2(X * scalar, Y * scalar);
}

Vector2 Vector2::operator+(const Vector2& Other) const {
    return Vector2(X + Other.X, Y + Other.Y);
}

Vector2 Vector2::operator-(const Vector2& Other) const {
    return Vector2(X - Other.X, Y - Other.Y);
}

bool Vector2::operator<(const Vector2& Other) const {
    if (X != Other.X)
        return X < Other.X;
    return Y < Other.Y;
}

bool Vector2::operator==(const Vector2& Other) const {
    return X == Other.X && Y == Other.Y;
}

Vector2 Vector2::Normalized() const {
    double length = Length();
    if (length == 0.0) return Vector2(0.0, 0.0);
    return Vector2(X / length, Y / length);
}

double Vector2::Length() const {
    return std::sqrt(X * X + Y * Y);
}

double Vector2::Vector2Dot(const Vector2& A, const Vector2& B) {
    return A.X * B.X + A.Y * B.Y;
}

std::tuple<double, double> Polygon::ProjectShape(Vector2 Axis) {
    double min = 1e9, max = -1e9;
    for (const auto& Point : Points) {
        double projection = Vector2::Vector2Dot(Point + Position, Axis);
        min = std::min(min, projection);
        max = std::max(max, projection);
    }
    return { min, max };
}

std::vector<Vector2> Polygon::GetSeparationAxes() {
    std::vector<Vector2> Axes;
    for (size_t i = 0; i < Points.size(); ++i) {
        Vector2 Edge = Points[i] - Points[(i + 1) % Points.size()];
        Vector2 Normal(-Edge.Y, Edge.X);
        Axes.push_back(Normal.Normalized());
    }
    return Axes;
}

AABB Polygon::GetSweptAABB(Vector2 Movement) {
    double MinX = 1e9, MinY = 1e9, MaxX = -1e9, MaxY = -1e9;
    for (const auto& Point : Points) {
        Vector2 AdjustedPoint = Point + Position;
        MinX = std::min(MinX, AdjustedPoint.X);
        MinY = std::min(MinY, AdjustedPoint.Y);
        MaxX = std::max(MaxX, AdjustedPoint.X);
        MaxY = std::max(MaxY, AdjustedPoint.Y);
    }

    if (Movement.X > 0) MaxX += Movement.X;
    else MinX += Movement.X;
    if (Movement.Y > 0) MaxY += Movement.Y;
    else MinY += Movement.Y;


    std::cout << "Swept AABB: Min(" << MinX << ", " << MinY << "), Max(" << MaxX << ", " << MaxY << ")\n";
    return { MinX, MinY, MaxX - MinX, MaxY - MinY };

}

