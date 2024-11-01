// Shapes
//
// Defines a base shape class, as well as defining a square and polygon shape
#include<vector>
#include<tuple>

#ifndef KINSOLVER_SHAPES_H_
#define KINSOLVER_SHAPES_H_

namespace KinSolver {
	
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
	};
}

#endif
