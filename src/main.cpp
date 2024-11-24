#include "collision_server.h"
#include "shapes.h"
#include<iostream>
#include <vector>
using namespace KinSolver;
int main() 
{
	std::cout << "Hello, World!\n";
	CollisionServer Server;

	KinSolver::Polygon Shape1;
	Shape1.Points = {Vector2(-0.5, -0.5), Vector2(-0.5, 0.5), Vector2(0.5, 0.5), Vector2(0.5, -0.5)};
	int Shape1Handle = Server.RegisterShape(&Shape1);

	std::cout << "After registering Shape 1 with handle " << Shape1Handle << "\n";

	KinSolver::Polygon Shape2;
	Shape2.Points = {Vector2(-0.5, -0.5), Vector2(-0.5, 0.5), Vector2(0.5, 0.5), Vector2(0.5, -0.5)};
	Shape2.Position.X = 1.5;
	int Shape2Handle = Server.RegisterShape(&Shape2);
	std::cout << "After registering Shape 2 with handle " << Shape2Handle << "\n";

	GlobalSweepResult Result = Server.SweepShape(Shape1Handle, Vector2(-1.0, 0.0));
	std::cout << Result.Result.TravelPortion << "\n";
	return 0;
}
