#include "DeformEnergy.h"
#include <Eigen/Geometry>
#include "myglobals.h"
#include "disjointset.h"

#include "NanoKdTree.h"

Array2D_Vector4d DeformEnergy::sideCoordinates = Structure::ShapeGraph::computeSideCoordinates();

DeformEnergy::DeformEnergy(Structure::ShapeGraph * shapeA, Structure::ShapeGraph * shapeB,
                           const QVector<QStringList> & a_landmarks,
                           const QVector<QStringList> & b_landmarks,
                           bool debugging) : a(shapeA), b(shapeB), total_energy(0), debugging(debugging)
{
	bool isGeometryDistortion = true;
	bool isUnCorrespondend = true;
	bool isCoverage = true;
	bool isSymmetries = false;
	bool isConnections = false;

	/// (1) Deformed geometry:
	if (isGeometryDistortion)
	{
		QMap<QString,double> partError;

		for (size_t i = 0; i < a_landmarks.size(); i++)
		{
			auto la = a_landmarks[i];
			auto lb = b_landmarks[i];
			bool isOneA = la.size() == 1;
			bool isOneB = lb.size() == 1;
			bool isSheetA = a->getNode(la.front())->type() == Structure::SHEET;
			bool isSheetB = b->getNode(lb.front())->type() == Structure::SHEET;
			bool isOneToMany = (isOneA != isOneB);
			bool isOneIsSheet = (isSheetA || isSheetB);
			bool isOtherNotSheet = !isSheetA || !isSheetB;

			bool isMergedNodes = false;
			Structure::ShapeGraph * graph;
			QStringList * landmarks;
			Structure::Node * newNode;

			if (isOneToMany && isOneIsSheet && isOtherNotSheet)
			{
				graph = (isOneA) ? shapeB : shapeA;
				landmarks = (isOneA) ? &lb : &la;

                QString newnode = Structure::ShapeGraph::convertCurvesToSheet(graph, *landmarks, sideCoordinates);
				newNode = graph->getNode(newnode);

				landmarks->clear();
				landmarks->push_back(newnode);

				isMergedNodes = true;
			}

			for (size_t u = 0; u < la.size(); u++)
			{
				for (size_t v = 0; v < lb.size(); v++)
				{
					auto nodeA = a->getNode(la[u]);
					auto nodeB = b->getNode(lb[v]);

					double area = deform(nodeA, nodeB, true);

					if (isMergedNodes) area += newNode->area();

					partError[nodeA->id] = area;
				}
			}

			if (isMergedNodes)
				graph->removeNode(landmarks->front());
		}
		
		double errorGeometric = 0;

		for (auto nid : partError.keys()) errorGeometric += partError[nid];

		energyTerms["geometric"].setValue(errorGeometric);
		total_energy += errorGeometric;
	}

	/// (2) For uncorrespondend nodes:
	if (isUnCorrespondend)
	{
		double errorUncorrespond = 0;

		// Collect list of uncorrespondend
		QStringList remainingNodes;
		for (auto n : a->nodes) remainingNodes << n->id;
		for (auto l : a_landmarks) for (auto nid : l) remainingNodes.removeAll(nid);

		for (auto nid : remainingNodes)
		{
			int numSides = (shapeA->getNode(nid)->type() == Structure::SHEET) ? 4 : 1;

			double longestSide = 0;

			for (int si = 0; si < numSides; si++)
			{
				double lengthSum = 0;
				auto pnts = shapeA->getNode(nid)->getPoints(std::vector<Array1D_Vector4d>(1, sideCoordinates[si])).front();
				for (size_t i = 1; i < pnts.size(); i++) lengthSum += (pnts[i-1] - pnts[i]).norm();
				longestSide = std::max(longestSide, lengthSum);
			}

			double lengthSquared = pow(longestSide, 2);

			errorUncorrespond += (lengthSquared * numSides);
		}

		energyTerms["uncorrespond"].setValue(errorUncorrespond);
		total_energy += errorUncorrespond;
	}

	/// (3) Coverage:
	if (isCoverage)
	{
		QStringList sourceNodes;
		for (auto l : a_landmarks) for (auto nid : l) sourceNodes << nid;

		// Get shape A nodes as groups
		auto grps = shapeA->groups;
		for (auto n : shapeA->nodes){
			bool isFound = false;
			for (auto g : grps) if (g.contains(n->id)) isFound = true;
			if (!isFound) grps.push_back(QVector<QString>() << n->id);
		}

		// Count corresponded groups
		int correspondedGroups = 0;
		for (auto g : grps){
			bool isFound = false;
			for (auto nid : sourceNodes){
				if (g.contains(nid)){
					isFound = true;
					break;
				}
			}
			if (isFound) correspondedGroups++;
		}

		double ratio = double(correspondedGroups) / grps.size();
		if (ratio == 0) ratio = 0.001;

		double errorCoverage = 1.0 / ratio;

		energyTerms["coverage"].setValue(errorCoverage);
		total_energy *= errorCoverage;
	}

	/// (4) Symmetries: check for global reflectional symmetry 
	if (isSymmetries && !b_landmarks.empty())
	{
		QStringList targetNodes;
		for (auto lb : b_landmarks) for (auto nidB : lb) targetNodes << nidB;

		// For now simply use x-axis
		Vector3 plane_n(1, 0, 0);
		Vector3 plane_pos = b->bbox().center();

		// Add feature points
		NanoKdTree tree;
		for (auto nid : targetNodes)
		{
			tree.addPoint(b->position(nid, Eigen::Vector4d(0, 0, 0, 0)));
			tree.addPoint(b->position(nid, Eigen::Vector4d(0.5, 0.5, 0, 0)));
			tree.addPoint(b->position(nid, Eigen::Vector4d(1, 0, 0, 0)));
			tree.addPoint(b->position(nid, Eigen::Vector4d(1, 1, 0, 0)));
			tree.addPoint(b->position(nid, Eigen::Vector4d(0, 1, 0, 0)));
		}
		tree.build();

		double threshold = b->bbox().diagonal().x() * 0.05;
		int foundMatches = 0;
		int expectedMatches = 0;

		for (auto p : tree.cloud.pts)
		{
			double distPlane = plane_n.dot(p - plane_pos);
			if (distPlane <= 0) continue; // one side

			Vector3 reflected = p - (2 * (distPlane*plane_n));

			auto closest = tree.closest(reflected);
			double dist = (reflected - tree.cloud.pts[closest]).norm();

			expectedMatches++;
			if (dist < threshold) foundMatches++;
		}

		if (expectedMatches > 0)
		{
			double ratio = double(foundMatches) / expectedMatches;
			if (ratio == 0) ratio = 0.01;

			double errorSymmetry = 1.0 / ratio;

			energyTerms["symmetry"].setValue(errorSymmetry);
			total_energy *= errorSymmetry;
		}
	}

	/// (5) Connections:
	if (isConnections)
	{
		QStringList targetNodes;
		for(auto lb : b_landmarks) for (auto nidB : lb) targetNodes << nidB;

		DisjointSet disjoint(targetNodes.size());

		for (size_t u = 0; u < targetNodes.size(); u++){
			for (size_t v = 0; v < targetNodes.size(); v++){
				if (u != v && shapeB->shareEdge(targetNodes[u], targetNodes[v])){
					disjoint.Union(u, v);
					break;
				}
			}
		}

		int numComponenets = disjoint.Groups().size();

		double errorConnection = std::max(numComponenets, 1);

		energyTerms["connection"].setValue(errorConnection);
		total_energy *= errorConnection;
	}
}

double DeformEnergy::deform( Structure::Node * inputNodeA, Structure::Node * inputNodeB, bool isTwistTerm )
{
	Structure::Node * nodeA = inputNodeA->clone();
	Structure::Node * nodeB = inputNodeB->clone();

	// Point-to-point correspondence
	if (nodeA->type() == Structure::SHEET && nodeB->type() == Structure::SHEET)
		Structure::ShapeGraph::correspondTwoSheets((Structure::Sheet*)nodeA, (Structure::Sheet*)nodeB, b);

	if (nodeA->type() == Structure::CURVE && nodeB->type() == Structure::CURVE)
		Structure::ShapeGraph::correspondTwoCurves((Structure::Curve*)nodeA, (Structure::Curve*)nodeB, b);

	// Number of sides to compute
	int numSides = (nodeA->type() == Structure::SHEET || nodeB->type() == Structure::SHEET) ? 4 : 1;

	QVector< QVector<Vector3> > quads_pnts(numSides);

	int num_steps = 10;

	// Transformation
	{
		auto cpntsA = nodeA->controlPoints();
		auto cpntsB = nodeB->controlPoints();

		std::vector<Array1D_Vector3> cur(numSides), prev(numSides);

		for (int si = 0; si < numSides; si++)
		{
			prev[si] = nodeA->getPoints(std::vector<Array1D_Vector4d>(1, sideCoordinates[si])).front();
			cur[si] = prev[si];

			for (auto p : prev[si]) quads_pnts[si] << p;

			for (int i = 1; i < num_steps; i++)
			{
				double t = double(i) / (num_steps - 1);

				auto pI = nodeA->getPoints(std::vector<Array1D_Vector4d>(1, sideCoordinates[si])).front();
				auto pJ = nodeB->getPoints(std::vector<Array1D_Vector4d>(1, sideCoordinates[si])).front();

				for (int u = 0; u < cur[si].size(); u++)
					cur[si][u] = AlphaBlend(t, pI[u], pJ[u]);

				for (auto p : cur[si]) quads_pnts[si] << p;
			}
		}
	}

	typedef QVector<size_t> Quad;
	QVector< QVector< Quad > > quads(numSides);
	QVector< Array2D_Vector3 > rectangles(numSides);

	int num_control_points = sideCoordinates.front().size();

	for (int si = 0; si < numSides; si++)
	{
		// Build faces
		for (int i = 0; i < num_steps - 1; i++)
		{
			for (int j = 0; j < num_control_points - 1; j++)
			{
				QVector<size_t> quad;

				quad << (i * num_control_points) + j;
				quad << (i * num_control_points) + (j + 1);
				quad << ((i + 1) * num_control_points) + (j + 1);
				quad << ((i + 1) * num_control_points) + j;

				quads[si] << quad;
			}
		}

		// Collect as rectangles
		int h = 0;
		rectangles[si].resize(num_steps);
		for (int i = 0; i < num_steps; i++){
			rectangles[si][i].resize(num_control_points);
			for (int j = 0; j < num_control_points; j++)
				rectangles[si][i][j] = quads_pnts[si][h++];
		}
	}

	double area = 0.0;

	for (int si = 0; si < numSides; si++)
	{
		starlab::PolygonSoup * ps;
		if (debugging) ps = new starlab::PolygonSoup();

		double sideArea = 0;

		NormalAnalysis normals;
		double twist = 0;

		for (auto quad : quads[si])
		{
			QVector<Vector3> q;
			for (auto v : quad)	q << quads_pnts[si][v];

			auto n1 = (q[1] - q[0]).cross(q[2] - q[0]);
			auto n2 = (q[2] - q[0]).cross(q[3] - q[0]);

			double triArea1 = n1.norm() / 2.0;
			double triArea2 = n2.norm() / 2.0;
			double quadArea = triArea1 + triArea2;

			sideArea += quadArea;

			normals.addNormal(n1.normalized());
		}

		if (isTwistTerm)
		{
			double dot = abs(((quads_pnts[si][num_control_points - 1] - quads_pnts[si][0]).normalized()).dot((quads_pnts[si].back()
							- quads_pnts[si][quads_pnts[si].size() - num_control_points]).normalized()));

			double angleTerm = 1.0 - dot;

			double lengthFrom = 0, lengthTo = 0;
			int total_pnts = quads_pnts[si].size();
			for (int i = 1; i < num_control_points; i++){
				lengthFrom += (quads_pnts[si][i] - quads_pnts[si][i-1]).norm();
				lengthTo += (quads_pnts[si][total_pnts - i] - quads_pnts[si][total_pnts - i - 1]).norm();
			}
			double lengthTerm = 1.0 - (std::min(lengthFrom, lengthTo) / std::max(lengthFrom, lengthTo));

			double normalsTerm = normals.standardDeviation();

			if (angleTerm < 0.1) normalsTerm = 0;

			twist = normalsTerm + angleTerm + lengthTerm;

			if (std::isnan(twist)) twist = 1.0;

			//if (debugging) debugBox(twist);

			// Extra penalty:
			if (twist > 0.4) sideArea *= 3;

			sideArea *= (1.0 + twist);
		}

		area += sideArea;

		if (debugging)
		{
			for (auto quad : quads[si])
			{
				QVector<starlab::QVector3> pnts;
				QVector<Vector3> q;
				for (auto v : quad)	q << quads_pnts[si][v];
				for (auto p : q) pnts << p;
				ps->addPoly(pnts, starlab::qtJetColor(twist));
			}
			debug << ps;
		}
	}

	delete nodeA;
	delete nodeB;

	return area;
}
