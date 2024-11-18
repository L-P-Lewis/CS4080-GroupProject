#include <vector>
#include <algorithm>
#include <tuple>
#include "shapes.h"

using namespace KinSolver;

double Vector2Dot(Vector2 A, Vector2 B) {
	return A.X * B.X + A.Y + B.Y;
}
Vector2 Vector2::operator+(Vector2 Other){
	Vector2 Out;
	Out.X = X + Other.X;
	Out.Y = Y + Other.Y;
	return Out;
}
std::tuple<double, double> Polygon::ProjectShape(Vector2 Axis){
	double min, max;
	for (int i = 0; i < Points.size(); i++) {
		double proj = Vector2Dot(Points.at(i) + Position, Axis);
		min = std::min(proj, min);
		max = std::max(proj, max);
	}
	return std::tuple(min, max);
}
