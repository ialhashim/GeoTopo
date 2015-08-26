#include <deque>
#include <stack>
#include <unordered_set>

#include "EnergyGuidedDeformation.h"

#include "StructureAnalysis.h"
#include "DeformToFit.h"
#include "PropagateSymmetry.h"
#include "PropagateProximity.h"
#include "EvaluateCorrespondence.h"

#include "hausdorff.h"

#include "myglobals.h"

#include <omp.h>

void Energy::GuidedDeformation::preprocess(Structure::ShapeGraph * shapeA, Structure::ShapeGraph * shapeB)
{
	// Analyze symmetry groups
	StructureAnalysis::analyzeGroups(shapeA, false);
	StructureAnalysis::analyzeGroups(shapeB, false);

	// Prepare for proximity propagation
	PropagateProximity::prepareForProximity(shapeA);
	PropagateProximity::prepareForProximity(shapeB);

	// Prepare for structure distortion evaluation
	EvaluateCorrespondence::prepare(shapeA);
	EvaluateCorrespondence::prepare(shapeB);
}

void Energy::GuidedDeformation::searchAll(Structure::ShapeGraph * shapeA, Structure::ShapeGraph * shapeB, QVector<Energy::SearchNode> & roots)
{
	if (roots.empty()) return;

	// Insert given roots of search trees
	for (auto & root : roots)
		searchTrees.push_back(SearchTree(root));

	// Pre-processing
	{
		origShapeA = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*shapeA));
		origShapeB = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*shapeB));

		preprocess(origShapeA.data(), origShapeB.data());
	}

	// Start search from roots
	for (auto & searchTree : searchTrees)
	{
		auto & root = *searchTree.begin();

		// Make copies
		root.shapeA = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*origShapeA));
		root.shapeB = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*origShapeB));

		// Set as unassigned anything not set for this root
		root.unassigned = root.unassignedList();

		// Explore
		std::stack < Energy::SearchTree::iterator_base > path_stack;
		path_stack.push(searchTree.begin());

		while (!path_stack.empty())
		{
			auto pathItr = path_stack.top();
			path_stack.pop();

			auto & path = *pathItr;

			// Apply deformation given current assignment
			applyAssignment(&path, false);

			// Collect valid suggestions
			auto suggested_children = suggestChildren(path, 4);
			for (auto & child : suggested_children)
				searchTree.append_child(pathItr, child);
			path.num_children = suggested_children.size();

			// Explore each suggestion:
			SearchTree::sibling_iterator child = searchTree.begin(pathItr);
			while (child != searchTree.end(pathItr)) {
				path_stack.push(child);
				++child;
			}
		}
	}
}

void Energy::GuidedDeformation::applyAssignment(Energy::SearchNode * path, bool isSaveKeyframes)
{
	double prevEnergy = path->energy;

	QSet<QString> mappedParts;

	// Go over and apply the suggested assignments:
	for (auto ap : path->assignments)
	{
		// Get assignment pair <source, target>
		auto la = ap.first, lb = ap.second;
		assert(la.size() && lb.size());

		// Apply any needed topological operations
		topologicalOpeartions(path->shapeA.data(), path->shapeB.data(), la, lb);

		// Assigned parts will be fixed
		for (auto partID : ap.first) path->current << partID;

		// Deform the assigned
		applyDeformation(path->shapeA.data(), path->shapeB.data(), la, lb, path->fixed + path->current, isSaveKeyframes);

		// Track established correspondence
		for (size_t i = 0; i < la.size(); i++)
		{
			auto targetPartID = lb[i].split(",").front();
			path->mapping[la[i]] = targetPartID;
			mappedParts << la[i];
		}
	}

	// Evaluate distortion of shape
	double curEnergy = EvaluateCorrespondence::evaluate(path);

	path->cost = curEnergy - prevEnergy;
	path->energy = path->energy + path->cost;

	// Assign costs to mapping:
	for (auto partID : mappedParts) path->mappingCost[partID] = path->cost;
}

QVector<Energy::SearchNode> Energy::GuidedDeformation::suggestChildren(Energy::SearchNode & path, int k_top)
{
	// Hard coded thresholding to limit search space
	double candidate_threshold = 0.5;
	double cost_threshold = 0.3;
	int k_top_candidates = k_top;

	/// Suggest for next unassigned:
	QVector<Structure::Relation> candidatesA;

	// Start process from remaining unassigned parts if needed
	if (!path.unassigned.isEmpty())
	{
		std::set<QString> path_unassigned;
		for (auto s : path.unassigned) path_unassigned.insert(s);

		// Suggest for all remaining unassigned
		for (auto partID : path_unassigned)
		{
			if (!path.shapeA->hasRelation(partID)) continue;
			auto r = path.shapeA->relationOf(partID);

            bool candidates_contains_relation = false;

            for(auto & rj : candidatesA) {
                if(rj.parts == r.parts){
                    candidates_contains_relation = true;
                    break;
                }
            }

            if (!candidates_contains_relation) candidatesA << r;
		}
	}

	// Suggest for each candidate
	QVector < QPair<Structure::Relation, Structure::Relation> > pairings;
	for (auto relationA : candidatesA)
	{
		// Candidate target groups
		/// Thresholding [0]: possibly skip geometrically very different ones
		for (auto relationB : path.shapeB->relations)
			pairings.push_back(qMakePair(relationA, relationB));
	}

	// Current shapes bounding boxes
	auto boxA = path.shapeA->bbox(), boxB = path.shapeB->bbox();

	// Output:
	QList< QPair<double, Energy::SearchNode> > evaluated_children;

	//#ifndef QT_DEBUG
	//#pragma omp parallel for
	//#endif
	for (int r = 0; r < pairings.size(); r++)
	{
		auto & pairing = pairings[r];

		auto & relationA = pairing.first;
		auto & relationB = pairing.second;

		// Relative position of centroid
		auto rboxA = path.shapeA->relationBBox(relationA);
		Vector3 rboxCenterA = (rboxA.center() - boxA.min()).array() / boxA.sizes().array();

		auto rboxB = path.shapeB->relationBBox(relationB);
		Vector3 rboxCenterB = (rboxB.center() - boxB.min()).array() / boxB.sizes().array();

		/// Thresholding [1]: Skip assignment if spatially too far
		auto dist = (rboxCenterA - rboxCenterB).norm();
		if (dist < candidate_threshold)
		{
			double curEnergy = 0;
			QStringList la = relationA.parts, lb = relationB.parts;

			// Case: many-to-many, find best matching between two sets
			if (la.size() != 1 && lb.size() != 1)
			{
				//Eigen::Vector4d centroid_coordinate(0.5, 0.5, 0, 0);
				auto intrestingCentroid = [&](Structure::ShapeGraph * shape, QString nodeID){
					Eigen::Vector4d c(0, 0, 0, 0);
					auto edges = shape->getEdges(nodeID);
					if (edges.empty()) return Eigen::Vector4d(0.5,0.5,0,0);
					for (auto e : edges) c += e->getCoord(nodeID).front();
					return Eigen::Vector4d(c / edges.size());
				};

				lb.clear();

				for (size_t i = 0; i < relationA.parts.size(); i++)
				{
					auto partID = relationA.parts[i];
					auto partA = path.shapeA->getNode(partID);

					auto centroid_coordinateA = intrestingCentroid(path.shapeA.data(), partID);
					Vector3 partCenterA = (partA->position(centroid_coordinateA) - rboxA.min()).array() / rboxA.sizes().array();

					partCenterA.array() *= rboxA.diagonal().normalized().array();

					QMap<double, QString> dists;
					for (auto tpartID : relationB.parts)
					{
						auto centroid_coordinateB = intrestingCentroid(path.shapeB.data(), tpartID);
						Vector3 partCenterB = (path.shapeB->getNode(tpartID)->position(centroid_coordinateB) - rboxB.min()).array() / rboxB.sizes().array();

						partCenterB.array() *= rboxA.diagonal().normalized().array();

						dists[(partCenterA - partCenterB).norm()] = tpartID;
					}

					lb << dists.values().front();
				}
			}
			else
			{
				la = relationA.parts;
				lb = relationB.parts;
			}

			// Make copies
			auto shapeA = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*path.shapeA));
			auto shapeB = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*path.shapeB));

			// Apply then evaluate cost of current suggestion
			auto copy_la = la, copy_lb = lb;
			topologicalOpeartions(shapeA.data(), shapeB.data(), copy_la, copy_lb);
			applyDeformation(shapeA.data(), shapeB.data(), copy_la, copy_lb, path.fixed + path.current + copy_la.toSet());

			// Evaluate:
			{
				SearchNode tempNode;
				tempNode.shapeA = shapeA;
				tempNode.shapeB = shapeB;
				tempNode.fixed = path.fixed + path.current + copy_la.toSet();
				tempNode.mapping = path.mapping;
				for (size_t i = 0; i < la.size(); i++) tempNode.mapping[copy_la[i]] = copy_lb[i].split(",").front();
				tempNode.unassigned = path.unassigned;
				for (auto p : la) tempNode.unassigned.remove(p);
				curEnergy = EvaluateCorrespondence::evaluate(&tempNode);
			}

			/// Thresholding [2]: Skip really bad assignments
			double cost = curEnergy - path.energy;
			if (cost < cost_threshold)
			{
				auto modifiedShapeA = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*path.shapeA));
				auto modifiedShapeB = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*path.shapeB));

				auto unassigned = path.unassigned;
				for (auto p : la) unassigned.remove(p);

				Assignments assignment;
				assignment << qMakePair(la, lb);
				assert(la.size() && lb.size());

				//#pragma omp critical
				evaluated_children.push_back(qMakePair(cost, SearchNode(modifiedShapeA, modifiedShapeB, 
					path.fixed + path.current, assignment, unassigned, path.mapping, cost, path.energy)));
			}
		}
	}

	/// Thresholding [3] : only accept the 'k' top suggestions
	qSort(evaluated_children);
	auto sorted_children = evaluated_children.toVector();
	sorted_children.resize(std::min(sorted_children.size(), k_top_candidates));

	QVector < Energy::SearchNode > accepted_children;
	for (auto & child : sorted_children)
	{
		// Copy parent's mapping cost
		child.second.mappingCost = path.mappingCost;

		accepted_children.push_back(child.second);
	}

	// Clean up of no longer needed data:
	path.shapeA.clear();
	path.shapeB.clear();

	return accepted_children;
}

QVector<Energy::SearchNode*> Energy::GuidedDeformation::solutions()
{
	auto & t = searchTrees.front();

	QVector<Energy::SearchNode*> result;
	for (auto leaf = t.begin_leaf(); leaf != t.end_leaf(); leaf++)
		if (leaf->unassigned.isEmpty())
			result.push_back(&(*leaf));

	/* Case: All paths where prematurely terminated */
	if (result.empty())
		for (auto leaf = t.begin_leaf(); leaf != t.end_leaf(); leaf++)
			result.push_back(&(*leaf));

	return result;
}

void Energy::GuidedDeformation::topologicalOpeartions(Structure::ShapeGraph *shapeA, Structure::ShapeGraph *shapeB,
	QStringList & la, QStringList & lb)
{
	// Utility:
	auto aproxProjection = [](const Vector3 p, Structure::Sheet * sheet){
		Eigen::Vector4d bestUV(0.5, 0.5, 0, 0), minRange(0, 0, 0, 0), maxRange(1, 1, 0, 0);
		double avgEdge = sheet->avgEdgeLength(), threshold = avgEdge * 0.5;
		return sheet->surface.timeAt(p, bestUV, minRange, maxRange, avgEdge, threshold);
	};

	// NULL Special case: any-to-null
	if (lb.contains(Structure::null_part))
	{
		return;
	}

	// Many-many case:
	if (la.size() > 1 && lb.size() > 1)
	{
		// Unique parts
		QStringList la_set = la, lb_set = lb;
		
		if (shapeA->hasRelation(la.front())) la_set = shapeA->relationOf(la.front()).parts;
		if (shapeB->hasRelation(lb.front())) lb_set = shapeB->relationOf(lb.front()).parts;

		if (la.size() != la_set.size() || lb.size() != lb_set.size())
		{
			// Experimental: will allow sliding
			for (auto partID : la_set)
			{
				auto partA = shapeA->getNode(partID);
				QString situation = la_set.size() > lb_set.size() ? "isMerged" : "isSplit";
				partA->property[situation].setValue(true);
				partA->property["isManyMany"].setValue(true);
			}
		}
	}

	// CONVERT Case: curve to sheet (unrolling)
	if (la.size() == lb.size() && la.size() == 1
		&& shapeA->getNode(la.front())->type() == Structure::CURVE
		&& shapeB->getNode(lb.front())->type() == Structure::SHEET)
	{
		auto snode = shapeA->getNode(la.front());
		Array2D_Vector3 surface_cpts(4, snode->controlPoints());
		auto snode_sheet = new Structure::Sheet(NURBS::NURBSRectangled::createSheetFromPoints(surface_cpts), snode->id);
		snode_sheet->property["mesh"].setValue(snode->property["mesh"].value< QSharedPointer<SurfaceMeshModel> >());
		snode_sheet->property["solidity"].setValue(1.0);

		// Replace curve with a squashed sheet
		shapeA->nodes.replace(shapeA->nodes.indexOf(snode), snode_sheet);

		// Prepare for evaluation
		EvaluateCorrespondence::sampleNode(shapeA, snode_sheet, shapeA->property["sampling_resolution"].toDouble());

		// Fix coordinates (not robust..)
		for (auto l : shapeA->getEdges(snode->id))
		{
			Array1D_Vector4 sheet_coords(1, aproxProjection(l->position(snode->id), snode_sheet));
			l->replaceForced(snode->id, snode_sheet, sheet_coords);

			// Prepare for evaluation
			l->property["orig_spokes"].setValue(EvaluateCorrespondence::spokesFromLink(shapeA, l));
		}

		// Remove from all relations
		StructureAnalysis::removeFromGroups(shapeA, snode);

		snode_sheet->property["isConverted"].setValue(true);

		delete snode;
	}
	
	// CONVERT Case: sheet to curve (squeezing)
	if (la.size() == lb.size() && la.size() == 1
		&& shapeA->getNode(la.front())->type() == Structure::SHEET
		&& shapeB->getNode(lb.front())->type() == Structure::CURVE)
	{
		shapeA->getNode(la.front())->property["isConverted"].setValue(true);
	}

	// MERGE Case: many curves - one sheet
	if (la.size() > 1 && lb.size() == 1
		&& shapeA->getNode(la.front())->type() == Structure::CURVE
		&& shapeB->getNode(lb.front())->type() == Structure::SHEET)

	{
		auto tnode_sheet = (Structure::Sheet*)shapeB->getNode(lb.front());
		lb.clear();

		QString sheetid = Structure::ShapeGraph::convertCurvesToSheet(shapeA, la, Structure::ShapeGraph::computeSideCoordinates());
		Structure::Sheet * snode_sheet = (Structure::Sheet *)shapeA->getNode(sheetid);

		QVector<QPair<Eigen::Vector4d, Eigen::Vector4d>> coords;
		for (size_t i = 0; i < la.size(); i++)
		{
			auto snode = shapeA->getNode(la[i]);
			Eigen::Vector4d start_c(0, 0, 0, 0), end_c(1, 0, 0, 0);

			auto c1 = aproxProjection(snode->position(start_c), snode_sheet);
			auto c2 = aproxProjection(snode->position(end_c), snode_sheet);

			coords << qMakePair(c1, c2);

			snode->property["isMerged"].setValue(true);
		}

		// Should give a nice enough alignment
		Structure::ShapeGraph::correspondTwoNodes(snode_sheet->id, shapeA, tnode_sheet->id, shapeB);
		DeformToFit::registerAndDeformNodes(snode_sheet, tnode_sheet);

		// Create equivalent curves on target sheet
		for (size_t i = 0; i < la.size(); i++)
		{
			auto coord = coords[i];

			Vector3 start_point = snode_sheet->position(coord.first);
			Vector3 end_point = snode_sheet->position(coord.second);
			Vector3 direction = (end_point - start_point).normalized();

			auto tcurve = tnode_sheet->convertToNURBSCurve(start_point, direction);
			auto tcurve_id = tnode_sheet->id + "," + la[i];

			auto newCurve = shapeB->getNode(tcurve_id) ? shapeB->getNode(tcurve_id) : shapeB->addNode(new Structure::Curve(tcurve, tcurve_id));
			newCurve->property["solidity"].setValue(1.0);
			lb << newCurve->id;
		}

		// Clean up
		shapeA->removeNode(sheetid);
	}

	// MERGE Case: many curves - one curve
	if (la.size() > 1 && lb.size() == 1
		&& shapeA->getNode(la.front())->type() == Structure::CURVE
		&& shapeB->getNode(lb.front())->type() == Structure::CURVE)
	{
		auto tnodeID = lb.front();
		lb.clear();

		// Experimental: will allow sliding
		for (auto partID : la)
		{
			auto snode = shapeA->getNode(partID);
			StructureAnalysis::removeFromGroups(shapeA, snode);
			snode->property["isMerged"].setValue(true);
			lb << tnodeID;
		}
	}

	// MERGE Case: many sheets - one sheet
	if (la.size() > 1 && lb.size() == 1
		&& shapeA->getNode(la.front())->type() == Structure::SHEET
		&& shapeB->getNode(lb.front())->type() == Structure::SHEET)
	{
		auto tnodeID = lb.front();
		lb.clear();

		// Experimental: will allow sliding
		for (auto partID : la)
		{
			auto snode = shapeA->getNode(partID);
			StructureAnalysis::removeFromGroups(shapeA, snode);
			if (partID != la.front()) snode->property["isMerged"].setValue(true);
			lb << tnodeID;
		}
	}

	// MERGE Case: many sheets - one curve
	if (la.size() > 1 && lb.size() == 1
		&& shapeA->getNode(la.front())->type() == Structure::SHEET
		&& shapeB->getNode(lb.front())->type() == Structure::CURVE)
	{
		auto tnodeID = lb.front();
		lb.clear();

		// Experimental: will allow sliding
		for (auto partID : la)
		{
			auto snode = shapeA->getNode(partID);
			StructureAnalysis::removeFromGroups(shapeA, snode);
			snode->property["isMerged"].setValue(true);
			lb << tnodeID;
		}
	}

	// pseudo-SPLIT Case: one sheet - many curves
	if (la.size() == 1 && lb.size() > 1
		&& shapeA->getNode(la.front())->type() == Structure::SHEET
		&& shapeB->getNode(lb.front())->type() == Structure::CURVE)
	{
		auto snode = shapeA->getNode(la.front());
		snode->property["isSplit"].setValue(true);

		QString newtnode = Structure::ShapeGraph::convertCurvesToSheet(shapeB, lb, Structure::ShapeGraph::computeSideCoordinates());

		// Turn curves on target to a new sheet
		auto tnode_sheet = shapeB->getNode(newtnode);
		double solidity_curves = shapeB->getNode(lb.front())->property["solidity"].toDouble();
		tnode_sheet->property["solidity"].setValue(solidity_curves);

		// Effectively perform a split by moving samples used for correspondence evaluation:
		{
			// Make a copy of the sheet
			auto snode_sheet = shapeA->getNode(la.front());
			auto snode_sheet_copy = snode_sheet->clone();
			snode_sheet_copy->id += "clone";
			shapeA->addNode(snode_sheet_copy);

			// Deform the copy to target curves
			Structure::ShapeGraph::correspondTwoNodes(snode_sheet_copy->id, shapeA, tnode_sheet->id, shapeB);
			DeformToFit::registerAndDeformNodes(snode_sheet_copy, tnode_sheet);

			// Snap source sheet samples to closest target curve projection
            auto samples = snode_sheet->property["samples_coords"].value<Array2D_Vector4>();
			assert(!samples.empty());

			auto tcurve = shapeB->getNode(lb.front());
			auto start_c = aproxProjection(tcurve->position(Eigen::Vector4d(0, 0, 0, 0)), (Structure::Sheet*)snode_sheet_copy);
			auto end_c = aproxProjection(tcurve->position(Eigen::Vector4d(1, 0, 0, 0)), (Structure::Sheet*)snode_sheet_copy);
			Eigen::Vector4d range = (end_c - start_c).cwiseAbs();
			int snapIDX = range[0] > range[1] ? 1 : 0;

            auto coordSnap = [&](const Eigen::Vector4d& _coord, int dim, int samplesCount, int curvesCount){
                Eigen::Vector4d coord = _coord;
				if (curvesCount >= samplesCount) return coord;
				double interval = 1.0 / curvesCount;
				double v = coord[dim] < 0.5 ? floor(coord[dim] / interval) : ceil(coord[dim] / interval);
				coord[dim] = v * interval;
				return coord;
			};

			int samplesCount = samples.size();
			int curvesCount = lb.size();

			for (auto & row : samples){
				for (auto & c : row){
					c = coordSnap(c, snapIDX, samplesCount, curvesCount);
				}
			}

			// Replace samples
			snode_sheet->property["samples_coords"].setValue(samples);
				
			// Clean up
			shapeA->removeNode(snode_sheet_copy->id);
		}

		lb.clear();
		lb << newtnode;
	}

	// SPLIT Case: one curve - many curves
	bool one_many_curves = la.size() == 1 && lb.size() > 1
		&& shapeA->getNode(la.front())->type() == Structure::CURVE
		&& shapeB->getNode(lb.front())->type() == Structure::CURVE;

	// SPLIT Case: one sheet - many sheets
	bool one_many_sheets = la.size() == 1 && lb.size() > 1
		&& shapeA->getNode(la.front())->type() == Structure::SHEET
		&& shapeB->getNode(lb.front())->type() == Structure::SHEET;

	// Perform split:
	if (one_many_curves || one_many_sheets)
	{
		auto copy_lb = lb;

		auto snode = shapeA->getNode(la.front());

		QVector<Structure::Node *> new_nodes;
		new_nodes << snode;

		// Match the first one to its closest counter-part
		{
			auto boxA = shapeA->bbox(), boxB = shapeB->bbox();
			Vector3 snode_center = (snode->position(Eigen::Vector4d(0.5, 0.5, 0, 0)) - boxA.min()).array() / boxA.sizes().array();
			QMap<double, QString> dists;
			for (auto partID : lb){
				Vector3 tnode_center = (shapeB->getNode(partID)->position(Eigen::Vector4d(0.5, 0.5, 0, 0)) - boxB.min()).array() / boxB.sizes().array();
				dists[(tnode_center - snode_center).norm()] = partID;
			}
			QString matched_tnode = dists.values().front();
			std::swap(lb[0], lb[lb.indexOf(matched_tnode)]);
			copy_lb.removeAll(matched_tnode);
		}

		// Make remaining copies
		for (auto partID : copy_lb)
		{
			// Clone first node
			auto new_snode = snode->clone();
			new_snode->id += QString("@%1").arg(new_nodes.size());

			// Clone underlying geometry
			{
				auto orig_mesh_ptr = new_snode->property["mesh"].value< QSharedPointer<SurfaceMeshModel> >();
				if (!orig_mesh_ptr.isNull())
				{
					auto orig_mesh = orig_mesh_ptr.data();
					QSharedPointer<SurfaceMeshModel> new_mesh_ptr(orig_mesh->clone());
					new_mesh_ptr->updateBoundingBox();
					new_snode->property["mesh"].setValue(new_mesh_ptr);
				}
			}

			shapeA->addNode(new_snode);
			new_nodes << new_snode;
		}

		la.clear();

		for (auto n : new_nodes)
		{
			n->property["isSplit"].setValue(true);
			la << n->id;
		}
	}
}

void Energy::GuidedDeformation::applyDeformation(Structure::ShapeGraph *shapeA, Structure::ShapeGraph *shapeB,
	const QStringList & la, const QStringList & lb, const QSetString & fixed, bool isSaveKeyframes)
{
	// Special case: many-to-null
	if (lb.contains(Structure::null_part)) return;

	// Save initial configuration
	//if (isSaveKeyframes) shapeA->pushKeyframeDebug(new RenderObject::Text(30, 30, "now deform", 15));
	if (isSaveKeyframes) shapeA->saveKeyframe();

	// Deform part to its target
	for (size_t i = 0; i < la.size(); i++)
	{
		auto partID = la[i];
		auto tpartID = lb[i];

		// does this help? why?
		if (shapeA->getNode(partID)->type() == Structure::SHEET && shapeB->getNode(tpartID)->type() == Structure::SHEET)
			Structure::ShapeGraph::correspondTwoNodes(partID, shapeA, tpartID, shapeB);

		DeformToFit::registerAndDeformNodes(shapeA->getNode(partID), shapeB->getNode(tpartID)); if (isSaveKeyframes) shapeA->saveKeyframe();
	}

	//if (isSaveKeyframes) shapeA->pushKeyframeDebug(new RenderObject::Text(30, 30, "now symmetry", 15));
	PropagateSymmetry::propagate(fixed, shapeA); if (isSaveKeyframes) shapeA->saveKeyframe();

	// Propagate edit by applying structural constraints
	//if (isSaveKeyframes) shapeA->pushKeyframeDebug(new RenderObject::Text(30, 30, "now proximity", 15));
	PropagateProximity::propagate(fixed, shapeA); if (isSaveKeyframes) { shapeA->saveKeyframe(); }
	//PropagateSymmetry::propagate(fixed, shapeA); if (isSaveKeyframes) { shapeA->saveKeyframe(); }
	//PropagateProximity::propagate(fixed, shapeA); if (isSaveKeyframes) { shapeA->saveKeyframe(); }

	postDeformation(shapeA, fixed);
}

void Energy::GuidedDeformation::postDeformation(Structure::ShapeGraph * shape, const QSet<QString> & fixedSet)
{
	for (auto & r : shape->relations)
	{
		if (r.parts.empty() || !r.parts.toSet().intersect(fixedSet).empty()) continue;

		StructureAnalysis::updateRelation(shape, r);
	}
}

QVector<Energy::SearchNode*> Energy::GuidedDeformation::childrenOf(Energy::SearchNode * path)
{
	QVector<Energy::SearchNode*> children;

	auto & searchTree = searchTrees.front();
	auto pathItr = searchTree.begin();
	for (; pathItr != searchTree.end(); pathItr++) if (&(*pathItr) == path) break;

	SearchTree::sibling_iterator child = searchTree.begin(pathItr);
	while (child != searchTree.end(pathItr)) {
		children << &(*child);
		++child;
	}

	std::sort(children.begin(), children.end(), [&](Energy::SearchNode * a, Energy::SearchNode * b){ return a->cost < b->cost; });

	return children;
}

QVector<Energy::SearchNode*> Energy::GuidedDeformation::getEntirePath(Energy::SearchNode * path)
{
	QVector<Energy::SearchNode*> entirePath;

	auto & t = searchTrees.front();
	auto itr = t.begin();
	for (; itr != t.end(); itr++) if (&(*itr) == path) break;

	// Find ancestors
	auto current = itr;
	while (current != 0){
		auto & p = *current;
		entirePath.push_front(&p);
		current = t.parent(current);
	}

	return entirePath;
}

void Energy::GuidedDeformation::applySearchPath(QVector<Energy::SearchNode*> path)
{
	path.front()->shapeA = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*origShapeA));
	path.front()->shapeB = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*origShapeB));

	for (size_t i = 0; i < path.size(); i++)
	{
		auto p = path[i];

		// Prepare using previous step
		if (i > 0)
		{
			p->shapeA = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*path[i - 1]->shapeA.data()));
			p->shapeB = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*path[i - 1]->shapeB.data()));
			p->energy = path[i - 1]->energy;
		}

		applyAssignment(p, true);
	}
}


int scoringBySymh(std::vector<int>& symhA, int groupNumA, std::vector<int>& symhB, int groupNumB, std::vector<int>& groupsA, std::vector<int>& groupsB)
{
	std::vector<std::vector<int> > groupInds(groupNumA); //collecting groups
	for (int i = 0; i < groupsA.size(); i++)
		groupInds[symhA[groupsA[i]]].push_back(i);

	unsigned scores = 0;
	for (int i = 0; i < groupInds.size(); i++)
	{
		if (groupInds[i].size() < 2) continue;

		int gsize = groupInds[i].size();
		std::vector<int> counting(groupNumB, 0);
		for (int j = 0; j < gsize; j++)
			counting[symhB[groupsB[groupInds[i][j]]]] ++;

		int maxG = *std::max_element(counting.begin(), counting.end());
		if (maxG == gsize){
			scores += (maxG - 1) * 2;

			std::unordered_set<int> tnodes;
			for (int j = 0; j < gsize; j++)
				tnodes.insert(groupsB[groupInds[i][j]]);
			scores -= gsize - tnodes.size();
		}
		else{
			scores = -1;
			break; //bad matching... well, only if we're confident about this case, 
		}
	}
	return scores;
}
void Energy::GuidedDeformation::symhPruning(Energy::SearchNode & path, QVector < QPair<Structure::Relation, Structure::Relation> > & pairings, std::vector<int>& pairAward)
{
	if (!isInitTest){
		//string to int == part name to part id map
		for (int i = 0; i < (int)path.shapeA->relations.size(); i++)
			for (auto r : path.shapeA->relations[i].parts)
				nidMapA[r] = i;
		for (int i = 0; i < (int)path.shapeB->relations.size(); i++)
			for (auto r : path.shapeB->relations[i].parts)
				nidMapB[r] = i;

		//help manually grouping
		// 		std::vector<std::string> partNamesA, partNamesB;
		//  		for (int i = 0; i < (int)path.shapeA->relations.size(); i++)
		// 			partNamesA.push_back(path.shapeA->relations[i].parts.first().toStdString());
		//  		for (int i = 0; i < (int)path.shapeB->relations.size(); i++)
		//  			partNamesB.push_back(path.shapeB->relations[i].parts.first().toStdString());
		//for chair
		// 		int inSymhA[] = { 0, 1, 0, 1 }; //just for test
		// 		int inSymhB[] = { 1, 0, 1, 0, 0 };
		// 		symhA.assign(inSymhA, inSymhA + 4);
		// 		symhB.assign(inSymhB, inSymhB + 5);
		//for cart
// 		 		int inSymhA[] = { 0, 0, 0, 1, 1, 1, 1, 1 }; //just for test
// 		 		int inSymhB[] = { 0, 0, 1, 1, 1, 1, 0 };
// 		 		symhA.assign(inSymhA, inSymhA + 8);
// 		 		symhB.assign(inSymhB, inSymhB + 7);
		//for tricycle
// 		 		int inSymhA[] = { 0, 1, 1, 2, 3, 1, 0, 3, 2, 3, 3, 0 };
// 		 		int inSymhB[] = { 0, 0, 0, 1, 1, 1, 2, 3, 3, 3, 0, 2, 3, 3 }; //just for test
// 		 		symhB.assign(inSymhA, inSymhA + 12);
// 		 		symhA.assign(inSymhB, inSymhB + 14);

		//random grouping
		for (int i = 0; i < (int)nidMapA.size(); i++)
			symhA.push_back((i) % 4);
		for (int i = 0; i < (int)nidMapB.size(); i++)
			symhB.push_back((i) % 4);

		symhGroupSizeA = *std::max_element(symhA.begin(), symhA.end()) + 1;
		symhGroupSizeB = *std::max_element(symhB.begin(), symhB.end()) + 1;

		isInitTest = true;
	}

	//loading assigned pairs;
	std::vector<int> groupsA, groupsB;
	for (QMap<QString, QString>::iterator mapit = path.mapping.begin(); mapit != path.mapping.end(); ++mapit)
	{
		int  tkey = nidMapA[mapit.key()];
		if (std::find(groupsA.begin(), groupsA.end(), tkey) == groupsA.end())
		{
			groupsA.push_back(tkey);
			groupsB.push_back(nidMapB[mapit.value()]);
		}
	}

#pragma omp parallel for
	for (int r = 0; r < pairings.size(); r++)
	{
		std::vector<int> gA(groupsA.begin(), groupsA.end());
		std::vector<int> gB(groupsB.begin(), groupsB.end());
		gA.push_back(nidMapA[pairings[r].first.parts.first()]);
		gB.push_back(nidMapB[pairings[r].second.parts.first()]);

		pairAward[r] = std::min(
			scoringBySymh(symhA, symhGroupSizeA, symhB, symhGroupSizeB, gA, gB),
			scoringBySymh(symhB, symhGroupSizeB, symhA, symhGroupSizeA, gB, gA)
			);
	}

	//only best mapping is kept
	int maxAward = *std::max_element(pairAward.begin(), pairAward.end());
	for (int i = 0; i < pairAward.size(); i++)
		pairAward[i] -= maxAward;
}
void Energy::GuidedDeformation::propagateDP(Energy::SearchNode & path, Structure::Relation& frontParts, std::vector<Structure::Relation>& mirrors,
	std::vector<double>& costs, std::vector<Energy::SearchNode>& res)
{
	double candidate_threshold = 0.5;
	double cost_threshold = 0.3;
	int k_top_candidates = std::min(K_2, (int)mirrors.size());

	QVector < QPair<Structure::Relation, Structure::Relation> > pairings;
	for (auto & mirror : mirrors)
		pairings.push_back(qMakePair(frontParts, mirror));

	// pruning pairs by SYMH
	std::vector<int> pairAward(pairings.size(), 0);
	if (isApplySYMH)
		symhPruning(path, pairings, pairAward);

	auto boxA = path.shapeA->bbox(), boxB = path.shapeB->bbox();

	for (int r = 0; r < pairings.size(); r++)
	{
		costs[r] = DBL_MAX;
		if (pairAward[r] < 0){ continue; }

		auto & pairing = pairings[r];

		auto & relationA = pairing.first;
		auto & relationB = pairing.second;

		// Relative position of centroid
		auto rboxA = path.shapeA->relationBBox(relationA);
		Vector3 rboxCenterA = (rboxA.center() - boxA.min()).array() / boxA.sizes().array();

		auto rboxB = path.shapeB->relationBBox(relationB);
		Vector3 rboxCenterB = (rboxB.center() - boxB.min()).array() / boxB.sizes().array();

		/// Thresholding [1]: Skip assignment if spatially too far
		auto dist = (rboxCenterA - rboxCenterB).norm();
		if (dist < candidate_threshold)
		{
			double curEnergy = 0;
			QStringList la = relationA.parts, lb = relationB.parts;

			// Case: many-to-many, find best matching between two sets
			if (la.size() != 1 && lb.size() != 1)
			{
				//Eigen::Vector4d centroid_coordinate(0.5, 0.5, 0, 0);
				auto intrestingCentroid = [&](Structure::ShapeGraph * shape, QString nodeID){
					Eigen::Vector4d c(0, 0, 0, 0);
					auto edges = shape->getEdges(nodeID);
					if (edges.empty()) return Eigen::Vector4d(0.5, 0.5, 0, 0);
					for (auto e : edges) c += e->getCoord(nodeID).front();
					return Eigen::Vector4d(c / edges.size());
				};

				lb.clear();

				for (size_t i = 0; i < relationA.parts.size(); i++)
				{
					auto partID = relationA.parts[i];
					auto partA = path.shapeA->getNode(partID);

					auto centroid_coordinateA = intrestingCentroid(path.shapeA.data(), partID);
					Vector3 partCenterA = (partA->position(centroid_coordinateA) - rboxA.min()).array() / rboxA.sizes().array();

					partCenterA.array() *= rboxA.diagonal().normalized().array();

					QMap<double, QString> dists;
					for (auto tpartID : relationB.parts)
					{
						auto centroid_coordinateB = intrestingCentroid(path.shapeB.data(), tpartID);
						Vector3 partCenterB = (path.shapeB->getNode(tpartID)->position(centroid_coordinateB) - rboxB.min()).array() / rboxB.sizes().array();

						partCenterB.array() *= rboxA.diagonal().normalized().array();

						dists[(partCenterA - partCenterB).norm()] = tpartID;
					}

					lb << dists.values().front();
				}
			}
			else
			{
				la = relationA.parts;
				lb = relationB.parts;
			}

			// Make copies
			auto shapeA = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*path.shapeA));
			auto shapeB = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*path.shapeB));

			// Apply then evaluate cost of current suggestion
			auto copy_la = la, copy_lb = lb;
			topologicalOpeartions(shapeA.data(), shapeB.data(), copy_la, copy_lb);
			applyDeformation(shapeA.data(), shapeB.data(), copy_la, copy_lb, path.fixed + path.current + copy_la.toSet());

			// Evaluate:
			{
				SearchNode tempNode;
				tempNode.shapeA = shapeA;
				tempNode.shapeB = shapeB;
				tempNode.fixed = path.fixed + path.current + copy_la.toSet();
				tempNode.mapping = path.mapping;
				for (size_t i = 0; i < la.size(); i++) tempNode.mapping[copy_la[i]] = copy_lb[i].split(",").front();
				tempNode.unassigned = path.unassigned;
				for (auto p : la) tempNode.unassigned.remove(p);
				curEnergy = EvaluateCorrespondence::evaluate(&tempNode);
			}

			/// Thresholding [2]: Skip really bad assignments
			double cost = curEnergy - path.energy;
			if (cost < cost_threshold)
			{
				auto modifiedShapeA = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*path.shapeA));
				auto modifiedShapeB = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*path.shapeB));

				auto unassigned = path.unassigned;
				for (auto p : la) unassigned.remove(p);

				Assignments assignment;
				assignment << qMakePair(la, lb);
				assert(la.size() && lb.size());

				costs[r] = curEnergy;
				res[r] = SearchNode(modifiedShapeA, modifiedShapeB, path.fixed + path.current, assignment, unassigned, path.mapping, cost, path.energy);

				res[r].mappingCost = path.mappingCost;
			}
		}
	}
	std::vector<double> tc = costs;
	std::sort(tc.begin(), tc.end());
	double thresbyK = tc[k_top_candidates - 1];
	for (int i = 0; i < costs.size(); i++){
		if (costs[i]>thresbyK)
			costs[i] = DBL_MAX;
	}
}
std::vector<std::pair<int, int> > TopKAlgorithm(std::vector < std::vector<double> >& vals, int K)
//find the top K values with index;
{
	struct tempStruct
	{
		tempStruct() :val(DBL_MAX), i(-1), j(-1){};
		tempStruct(int& ti, int& tj, double& tval){ i = ti; j = tj; val = tval; };
		bool operator< (const tempStruct& t2) const
		{
			return val < t2.val;
		};
		double val;
		int i, j;
	};
	std::multiset<tempStruct> kVals;
	std::multiset<tempStruct>::iterator kit;
	for (int i = 0; i < K; i++)
		kVals.insert(tempStruct());

	for (int i = 0; i < vals.size(); i++)
	{
		for (int j = 0; j < vals[i].size(); j++)
		{
			if (vals[i][j] < kVals.rbegin()->val)
			{
				kit = kVals.end(); kit--;
				kVals.erase(kit); //replace the last one
				kVals.insert(tempStruct(i, j, vals[i][j]));
			}

		}
	}

	std::vector<std::pair<int, int> > inds(K);
	int ids = 0;
	for (kit = kVals.begin(); kit != kVals.end(); kit++)
	{
		inds[ids].first = kit->i;
		inds[ids].second = kit->j;
		ids++;
	}
	return inds;
}
void Energy::GuidedDeformation::searchDP(Structure::ShapeGraph * shapeA, Structure::ShapeGraph * shapeB, QVector<Energy::SearchNode> & roots)
{
	// Preprocessing
	{
		origShapeA = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*shapeA));
		origShapeB = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*shapeB));

		preprocess(origShapeA.data(), origShapeB.data());
	}

	//initial state
	SearchNode root;
	root.shapeA = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*origShapeA));
	root.shapeB = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*origShapeB));
	root.unassigned = root.unassignedList();
	applyAssignment(&root, false);


	int N, M;
	N = (int)root.shapeA->relations.size();
	M = (int)root.shapeB->relations.size();
	std::vector<Structure::Relation> unvisitedParts, mirrors;
	for (auto rel : root.shapeA->relations)
		unvisitedParts.push_back(rel);
	for (auto rel : root.shapeB->relations)
		mirrors.push_back(rel);

	if (K < M) K = M;
	std::vector<SearchNode> topK(K);			//stores top K solution
	std::vector<double> topKScore(K, DBL_MAX);	//and the top K best cost


	//connectivity group
	QMap<QString, int> relationIds;
	for (int i = 0; i < root.shapeA->relations.size(); i++)
		relationIds[root.shapeA->relations[i].parts.front()] = i;
	std::vector<std::vector<bool> > connectGraph(root.shapeA->relations.size(), std::vector<bool>(root.shapeA->relations.size(), false));
	std::vector<int > connectStrength(root.shapeA->relations.size(), 0);
	for (auto edg : root.shapeA->edges)
	{
		if (!root.shapeA->hasRelation(edg->n1->id)) continue;
		if (!root.shapeA->hasRelation(edg->n2->id)) continue;
		auto r1 = root.shapeA->relationOf(edg->n1->id);
		auto r2 = root.shapeA->relationOf(edg->n2->id);
		connectGraph[relationIds[r1.parts.front()]][relationIds[r2.parts.front()]] = true;
		connectGraph[relationIds[r2.parts.front()]][relationIds[r1.parts.front()]] = true;
	}
	for (int i = 0; i < connectGraph.size(); i++)
	{
		for (int j = i + 1; j < connectGraph[i].size(); j++)
		{
			if (connectGraph[i][j])
			{
				connectStrength[i]++;
				connectStrength[j]++;
			}
		}
	}
	int firstfrontId = std::max_element(connectStrength.begin(), connectStrength.end()) - connectStrength.begin();

	//initial space; DP is order sensitive, different order of visiting returns different solution
	std::vector<double> initCost(M);
	std::vector<SearchNode> initStates(M);
	propagateDP(root, unvisitedParts[firstfrontId], mirrors, initCost, initStates);

	for (int j = 0; j < M; j++)
	{
		topK[j] = initStates[j];
		topKScore[j] = initCost[j];
	}

	std::vector<Structure::Relation> visitedParts(1, unvisitedParts[firstfrontId]);
	unvisitedParts.erase(unvisitedParts.begin() + firstfrontId);

	// complexity = N * K*M; 
	std::vector< std::vector< SearchNode > > allPaths(K, std::vector< SearchNode >(M));
	while (!unvisitedParts.empty())
	{
		//find parts in the same group of visited.
		int frontId = 0;
		if (isApplySYMH && isInitTest)
		{
			std::vector<bool> usedGroups(symhGroupSizeA, false);
			for (auto & rel : visitedParts)
				usedGroups[symhA[nidMapA[rel.parts.first()]]] = true;

			for (int i = 0; i < unvisitedParts.size(); i++)
			{
				if (usedGroups[symhA[nidMapA[unvisitedParts[i].parts.first()]]])
				{
					frontId = i;
					break;
				}
			}
		}
		else
		{
			//find neighbor
			std::vector<int> frontPrior(unvisitedParts.size(), 0);
			for (int i = 0; i < unvisitedParts.size(); i++){
				for (auto & rel2 : visitedParts){
					if (connectGraph[relationIds[rel2.parts.front()]][relationIds[unvisitedParts[i].parts.front()]])
					{
						frontPrior[i] = connectStrength[relationIds[unvisitedParts[i].parts.front()]];
						break;
					}
				}
			}
			frontId = std::max_element(frontPrior.begin(), frontPrior.end()) - frontPrior.begin();
			if (frontId == 0)
			{
				std::vector<int> frontPrior2(unvisitedParts.size());
				for (int i = 0; i < unvisitedParts.size(); i++)
					frontPrior2[i] = connectStrength[relationIds[unvisitedParts[i].parts.front()]];

				frontId = std::max_element(frontPrior2.begin(), frontPrior2.end()) - frontPrior2.begin();
			}
		}
		Structure::Relation frontParts = unvisitedParts[frontId];
		visitedParts.push_back(frontParts);
		unvisitedParts.erase(unvisitedParts.begin() + frontId);

		//propagate front;
		std::vector < std::vector<double> > newCost(K, std::vector < double >(M, DBL_MAX));
#pragma omp parallel for
		for (int i = 0; i < K; i++)
		{
			if (topKScore[i] != DBL_MAX)
			{
				applyAssignment(&topK[i], false);
				propagateDP(topK[i], frontParts, mirrors, newCost[i], allPaths[i]);
			}
		}

		//top K paths;
		std::vector < std::pair<int, int> > td = TopKAlgorithm(newCost, K);
		for (int i = 0; i < K; i++)
		{
			if (td[i].first > -1 && newCost[td[i].first][td[i].second] != DBL_MAX)
			{
				topKScore[i] = newCost[td[i].first][td[i].second];
				topK[i] = allPaths[td[i].first][td[i].second];
			}
			else
			{
				topKScore[i] = DBL_MAX;
			}
		}
	}
#pragma omp parallel for
	for (int i = 0; i < K; i++)
	{
		if (topKScore[i] != DBL_MAX)
		{
			applyAssignment(&topK[i], false);
		}
	}
	roots.push_back(topK[std::min_element(topKScore.begin(), topKScore.end()) - topKScore.begin()]);
}

Energy::SearchNode Energy::GuidedDeformation::partialSelectionGreedy(const Energy::SearchNode &initpath, const Energy::SearchNode &path, int bestK)
{
	//get volume size.
	QMap<QString, double> partsToVolume;
	Structure::ShapeGraph a(*initpath.shapeA.data());
	Structure::ShapeGraph b(*initpath.shapeB.data());

	// Will compute volumes of groups
	Energy::GuidedDeformation::preprocess(&a, &b);

	// Sort by volume
	QVector < QPair<double, Structure::Relation*> > volumeRelation;

	for (auto & r : a.relations)
		volumeRelation << qMakePair(r.property["volume"].toDouble(), &r);

	for (auto vr : volumeRelation)
	{
		auto relation = vr.second;
		for (auto partID : relation->parts)
		{
			partsToVolume[partID] = vr.first/* / (double)relation->parts.size()*/;
		}
	}

	//
	QMap<QString, QString> newMapping = path.mapping;
	auto shapeA = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*path.shapeA));
	auto shapeB = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*path.shapeB));
	SearchNode newPath = SearchNode(shapeA, shapeB, QSetString(), Energy::Assignments(), QSetString(), newMapping, 0);

	int reapeatTimes = path.shapeA->relations.size() - bestK;
	//everytime, select a relation;
	while (reapeatTimes > 0)
	{
		double cost = EvaluateCorrespondence::evaluate(&newPath);

		QVector<Structure::Relation> remainRelations = path.shapeA->relations;
		for (QVector<Structure::Relation>::iterator irel = remainRelations.begin(); irel != remainRelations.end(); irel++)
		{
			if (!irel->parts.empty() && partsToVolume.find(irel->parts.first()) == partsToVolume.end())
			{
				remainRelations.erase(irel); irel--;
			}
		}

		QVector<Energy::SearchNode> newPaths;
		for (int i = 0; i < remainRelations.size(); i++)
		{
			auto shapeA = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*path.shapeA));
			auto shapeB = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*path.shapeB));

			newPaths.push_back(SearchNode(shapeA, shapeB, QSetString(), Energy::Assignments(), QSetString(), newMapping, 0));
			//remove mappings of the relation
			for (auto &p : remainRelations[i].parts)
			{
				newPaths.back().mapping.erase(newPaths[i].mapping.find(p));
			}
		}

		std::vector<double> newCosts(newPaths.size());
#pragma omp parallel for
		for (int i = 0; i < newPaths.size(); i++)
		{
			newCosts[i] = EvaluateCorrespondence::evaluate(&newPaths[i]);
			newCosts[i] = cost - newCosts[i];
		}

		//weighted cost for a possibly removed relation
		for (int i = 0; i < newCosts.size(); i++)
		{
			newCosts[i] = newCosts[i] / partsToVolume[remainRelations[i].parts.first()];
		}

		int ind = std::max_element(newCosts.begin(), newCosts.end()) - newCosts.begin();
		newMapping = newPaths[ind].mapping;

		//update volume mapping
		for (auto &p : remainRelations[ind].parts)
		{
			partsToVolume.erase(partsToVolume.find(p));
		}

		auto shapeA = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*path.shapeA));
		auto shapeB = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*path.shapeB));
		newPath = SearchNode(shapeA, shapeB, QSetString(), Energy::Assignments(), QSetString(), newMapping, 0);

		reapeatTimes--;
	}

	//everytime, select a part;
	// 	int reapeatTimes = path.mapping.size() - bestK;
	// 	while (reapeatTimes>0)
	// 	{
	// 		double cost = EvaluateCorrespondence::evaluate(&newPath);
	// 
	// 		QVector<Energy::SearchNode> newPaths(newMapping.size());
	// 		for (int i = 0; i < newPaths.size(); i++)
	// 		{
	// 			auto shapeA = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*path.shapeA));
	// 			auto shapeB = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*path.shapeB));
	// 
	// 			newPaths[i] = SearchNode(shapeA, shapeB, QSetString(), Energy::Assignments(), QSetString(), newMapping, 0);
	// 			newPaths[i].mapping.erase(newPaths[i].mapping.begin() + i);
	// 		}
	// 
	// 		std::vector<double> newCosts(newPaths.size());
	// #pragma omp parallel for
	// 		for (int i = 0; i < newPaths.size(); i++)
	// 		{
	// 			newCosts[i] = EvaluateCorrespondence::evaluate(&newPaths[i]);
	// 			newCosts[i] = cost - newCosts[i];
	// 		}
	// 
	// 		//weighted cost.
	// 		int ind = 0;
	// 		for (QMap<QString, QString>::iterator imap = newMapping.begin(); imap != newMapping.end(); imap++)
	// 		{
	// 			newCosts[ind] = newCosts[ind] / partsToVolume[imap.key()];
	// 			ind++;
	// 		}
	// 
	// 		newMapping = newPaths[std::min_element(newCosts.begin(), newCosts.end()) - newCosts.begin()].mapping;
	// 
	// 		auto shapeA = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*path.shapeA));
	// 		auto shapeB = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*path.shapeB));
	// 		newPath = SearchNode(shapeA, shapeB, QSetString(), Energy::Assignments(), QSetString(), newMapping, 0);
	// 
	// 		reapeatTimes--;
	// 	}
	return newPath;
}
