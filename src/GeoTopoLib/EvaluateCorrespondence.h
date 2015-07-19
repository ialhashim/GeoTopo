#pragma once
#include "ShapeGraph.h"
#include "NanoKdTree.h"
#include "EnergyGuidedDeformation.h"

struct EvaluateCorrespondence
{
	static void prepare(Structure::ShapeGraph * shape);
	static double evaluate(Energy::SearchNode * searchNode);
	static double evaluate2(Energy::SearchNode * searchNode);

	// Utility:
	static Array1D_Vector3 spokesFromLink(Structure::ShapeGraph * shape, Structure::Link * link, bool isFindCoordClosest = false);
	static Array2D_Vector4d sampleNode(Structure::ShapeGraph * shape, Structure::Node * n, double resolution);
	static QMap<QString, NanoKdTree*> kdTreesNodes(Structure::ShapeGraph * shape);
	static QMap<QString, QMap<QString, double> > hausdroffDistance( Structure::ShapeGraph * shapeA, Structure::ShapeGraph * shapeB );
	static double RMSD(Structure::ShapeGraph * shapeA, Structure::ShapeGraph * shapeB);
	static double vectorSimilarity(QVector<double> & feature_vector);

	static int numSamples;
};

Q_DECLARE_METATYPE(Vector3);
Q_DECLARE_METATYPE(Array1D_Vector3);
Q_DECLARE_METATYPE(Array2D_Vector3);
Q_DECLARE_METATYPE(Array2D_Vector4d);
