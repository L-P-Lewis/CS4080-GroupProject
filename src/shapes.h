// Shapes
//
// Defines a base shape class, as well as defining a square and polygon shape
#include <vector>
#include <tuple>


#ifndef KINSOLVER_SHAPES_H_
#define KINSOLVER_SHAPES_H_

namespace KinSolver {
	
	// Represents an axis alligned bounding box
	struct AABB {
		double X;
		double Y;
		double Width;
		double Height;
	};
	// Represents a 2d vector
	class Vector2 {
	public:
		double X;
		double Y;
		Vector2 operator*(double scalar);
		Vector2 operator+(Vector2 Other);
		Vector2 operator-(Vector2 Other);
		bool operator<(const Vector2 &Other) const {return Other.X + Other.Y < X + Y;};
		bool operator==(const Vector2 &Other) const {return Other.X == X && Other.Y == Y;};
		Vector2 Normalized();
		Vector2() : X(0.0), Y(0.0) {};
		Vector2(double X, double Y) : X(X), Y(Y) { };
		static double Vector2Dot(Vector2 A, Vector2 B);
	};
	const Vector2 VECTOR_2_ZERO = Vector2(0.0, 0.0);
	// Represents a type of convex collision shape. Mostly holds virtual functions
	class Shape {
	public:
		Vector2 Position;
		Shape() : Position(Vector2()) {};
		virtual std::vector<Vector2> GetSeperationAxes() {return std::vector<Vector2>();};
		virtual std::tuple<double, double> ProjectShape(Vector2 Axis) {return std::tuple<double, double>();} ;
		virtual AABB GetSweptAABB(Vector2 Movement) {return (AABB){0.0, 0.0, 0.0, 0.0};}; 
	};

	class Polygon : public Shape {
	public:
		std::vector<Vector2> Points;
		std::tuple<double, double> ProjectShape(Vector2 Axis);
		std::vector<Vector2> GetSeperationAxes();
		AABB GetSweptAABB(Vector2 Movement); 
	};
}

	bool AABBOverlap(KinSolver::AABB A, KinSolver::AABB B);
#endif
