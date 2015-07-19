#pragma once

#include "ShapeGraph.h"

class PropagateSymmetry
{
public:
	static void propagate(const QSet<QString> & fixedNodes, Structure::ShapeGraph * graph);
};
