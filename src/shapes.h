/*
Names: Luke Lewis, Erick Hambardzumyan
Class: CS 4080
Assignment: Group Project- Collision Detection System
Date: 12-05-2024
*/

//shapes.h
//class declaration for the shapes used in the collision system

#ifndef KINSOLVER_SHAPES_H_
#define KINSOLVER_SHAPES_H_

#include <vector> // For using std::vector to store points in shapes
#include <tuple>  // For using std::tuple to represent projections
#include <cmath>  // For mathematical functions like length and normalization

namespace KinSolver {

    // AABB (Axis-Aligned Bounding Box) structure holds information about the box's position and size
    struct AABB {
        double X, Y, Width, Height;  // X, Y: position of the top-left corner, Width, Height: dimensions of the box
    };

    // The Vector2 class represents a 2D vector with x and y components
    class Vector2 {
    public:
        double X;  // X component of the vector
        double Y;  // Y component of the vector

        // Operator overloads to allow vector multiplication by a scalar and addition/subtraction between vectors
        Vector2 operator*(double scalar) const;
        Vector2 operator+(const Vector2& Other) const;
        Vector2 operator-(const Vector2& Other) const;

        // Comparison operators to check for vector inequality or equality
        bool operator<(const Vector2& Other) const;
        bool operator==(const Vector2& Other) const;

        // Method to normalize the vector (make its length 1)
        Vector2 Normalized() const;

        // Method to compute the length (magnitude) of the vector
        double Length() const;

        // Default constructor initializing the vector to (0,0) 
        Vector2() : X(0.0), Y(0.0) {}

        // Constructor that initializes the vector with the given X and Y values
        Vector2(double X, double Y) : X(X), Y(Y) {}

        // Static method to compute the dot product of two vectors
        static double Vector2Dot(const Vector2& A, const Vector2& B);
    };

    // The Shape class is an abstract base class for all shapes (e.g., polygons, circles)
    // It defines common methods that derived shapes must implement
    class Shape {
    public:
        Vector2 Position;  // Position of the shape

        // Constructor to initialize the shape's position to (0,0)
        Shape() : Position(Vector2()) {}

        // Pure virtual methods that must be implemented by derived classes
        virtual std::vector<Vector2> GetSeparationAxes() = 0;  // Computes the axes for Separating Axis Theorem (SAT)
        virtual std::tuple<double, double> ProjectShape(Vector2 Axis) = 0;  // Projects the shape onto an axis
        virtual AABB GetSweptAABB(Vector2 Movement) = 0;  // Computes the swept AABB (axis-aligned bounding box after movement)
    };

    // The Polygon class is a derived class from Shape representing a polygonal shape
    class Polygon : public Shape {
    public:
        std::vector<Vector2> Points;  // List of vertices (points) of the polygon

        // Implementation of ProjectShape for projecting a polygon onto an axis
        std::tuple<double, double> ProjectShape(Vector2 Axis) override;

        // Implementation of GetSeparationAxes for a polygon (returns the axes used for SAT)
        std::vector<Vector2> GetSeparationAxes() override;

        // Implementation of GetSweptAABB for a polygon (calculates the bounding box after movement)
        AABB GetSweptAABB(Vector2 Movement) override;
    };

}

#endif

