#include <vector>
#include <set>
#include <tuple>
#include "collision_server.h"
#include "shapes.h"
using namespace KinSolver;


#define Range std::tuple<double, double>


// Takes in a list of vectors and adds all of them to the given set pointing opposite movement
void AddProjectedMovementAxes(std::set<Vector2>* Set, std::vector<Vector2> Axes, Vector2 Movement) {
	for ( int i = 0; i < Axes.size(); i++) {
		if (Vector2::Vector2Dot(Axes[i], Movement) < 0.0) {
			Set -> insert(Axes[i] * -1.0);
		} else {
			Set->insert(Axes[i]);
		}
	}
}

bool TestRangeOverlap(Range A, Range B) {
	return std::get<0>(A) <=std::get<1>(B) && std::get<0>(B) <= std::get<1>(A);
}

double GetRangeEntryTime(Range Mover, Range Static, double Movement) {
	if (TestRangeOverlap(Mover, Static)) return 0.0;
	double Difference = 99999999.0;
	if (Movement > 0.0) Difference = std::get<0>(Static) - std::get<1>(Mover);
	if (Movement < 0.0) Difference = std::get<0>(Mover) - std::get<1>(Static);
	return Difference / Movement;
}

double GetRangeExitTime(Range Mover, Range Static, double Movement) {
	double Difference = 99999999.0;
	if (Movement > 0.0) Difference = std::get<1>(Static) - std::get<0>(Mover);
	if (Movement < 0.0) Difference = std::get<1>(Mover) - std::get<0>(Static);
	return Difference / Movement;
}

SweepResult CollisionServer::TestCollideShapes(Shape Mover, Shape Target, Vector2 Movement){
	// Step 1: Get all axes of potential collision
	std::set<Vector2> CollisionAxes;
	AddProjectedMovementAxes(&CollisionAxes, Mover.GetSeperationAxes(), Movement);
	AddProjectedMovementAxes(&CollisionAxes, Target.GetSeperationAxes(), Movement);
	// Step 2: Loop through axes..
	double FirstExit = 9999999.0;
	double LastEntry = 0.0;
	Vector2 LastEntryDir;
	for(std::set<Vector2>::iterator Axis = CollisionAxes.begin(); Axis != CollisionAxes.end(); Axis++) {
		// Find when the shapes will begin and end overlapping on each axis
		double Entry = GetRangeEntryTime(
			Mover.ProjectShape(*Axis),
			Target.ProjectShape(*Axis),
			Vector2::Vector2Dot(Movement, *Axis)
		);
		double Exit = GetRangeExitTime(
			Mover.ProjectShape(*Axis),
			Target.ProjectShape(*Axis),
			Vector2::Vector2Dot(Movement, *Axis)
		);
		// Find earliest time of exiting and latest time of entry
		FirstExit = std::min(Exit, Entry);
		if (LastEntry < Entry) {
			LastEntry = Entry;
			LastEntryDir = *Axis;
		}
	}
	
	// If first exit happens before the first entry then the shapes will not collide
	bool DidHit = FirstExit > LastEntry && LastEntry <= 1.0;
	// The normal of collision is the direction of the last time of entry
	//TODO: Rework to return data struct
	return (SweepResult){LastEntry, LastEntryDir};
}


int CollisionServer::RegisterShape(Shape NewShape) {
	// Scan through shapes vector to find an empty space
	int TargetPos;
	for (int i = 0; i < Shapes.size(); i++) {
		if (Shapes[i] == nullptr) {
			TargetPos = i;
			Shapes[i] = &NewShape;
			return i;
		}
	}
	Shapes.push_back(&NewShape);
	return Shapes.size() - 1;
}


ShapeList CollisionServer::GetShapesInAABB(AABB BoundingBox){
	ShapeList CollisionList;
	for (int i = 0; i < Shapes.size(); i++) {
		if (Shapes[i] == nullptr) continue;
		if (AABBOverlap(BoundingBox, Shapes[i]->GetSweptAABB(VECTOR_2_ZERO))) {
			CollisionList.push_back(i);
		}
	}
	return CollisionList;
}


GlobalSweepResult CollisionServer::SweepShape(int ShapeID, Vector2 Velocity){
	ShapeList PotentialCollisions = GetShapesInAABB(Shapes[ShapeID]->GetSweptAABB(Velocity));
	SweepResult ClosestHit = (SweepResult){1.0, Velocity};
	int ClosestObject = -1; 
	for (int i = 0; i < PotentialCollisions.size(); i++) {
		if (i == ShapeID) continue;
		SweepResult NewHit = TestCollideShapes(*Shapes[ShapeID], *Shapes[i], Velocity);
		if (NewHit.TravelPortion < ClosestHit.TravelPortion) {
			ClosestHit = NewHit;
			ClosestObject = i;
		}
	}
	return (GlobalSweepResult) {ClosestHit, ClosestObject};
}
