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
	};

	// Represents a type of convex collision shape. Mostly holds virtual functions
	class Shape {
	public:
		Vector2 Position;
		virtual std::vector<Vector2> GetSeperationAxes();
		virtual std::tuple<double, double> ProjectShape(Vector2 Axis);
		virtual AABB GetSweptAABB(Vector2 Movement); 
	};

	class Polygon : public Shape {
	public:
		std::vector<Vector2> Points;
	};
}

#endif
