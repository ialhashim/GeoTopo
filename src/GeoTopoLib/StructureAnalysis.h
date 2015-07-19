#pragma once
#include "ShapeGraph.h"

class StructureAnalysis
{
public:
	static void analyzeGroups(Structure::ShapeGraph * shape, bool isDebug = false);
	static void removeFromGroups(Structure::ShapeGraph * shape, Structure::Node * node);

	// Utility:
	static std::pair<Vector3, Vector3> getReflectionalPlane(Structure::Node * n1, Structure::Node * n2);
	static Vector3 pointReflection(const Vector3 & p, const Vector3 & planePoint, const Vector3 & planeNormal);
	static void updateRelation(Structure::ShapeGraph * shape, Structure::Relation & r);
};
