#include "PropagateProximity.h"
#include "ShapeGraph.h"
#include <array>

#include "convexhull2d.h"
#include "mean_value_coordinates.h"

Q_DECLARE_METATYPE(Vector3);

struct ProximityConstraint{
	Vector3 d;
	Structure::Node *from, *to;
	Structure::Link *link;
	ProximityConstraint(Structure::Node *from = nullptr, Structure::Link *link = nullptr) :from(from), link(link){
		if (!from || !link) return;
		to = link->otherNode(from->id);
		d = link->property[from->id].value<Vector3>();
	}
	inline Vector3 start(){ return link->getNode(from->id)->position(link->getCoord(from->id).front()); }
	inline Vector3 start2(){ return link->getNode(from->id)->position(link->getCoord(from->id).back()); }
	inline Vector3 delta(){ return d; }
	inline Eigen::Vector4d coord(){ return link->getCoord(to->id).front(); }
	inline Array1D_Vector4d coords(){ return link->getCoord(to->id); }
};

void PropagateProximity::prepareForProximity(Structure::Graph * graph)
{
	for (auto & edge : graph->edges){
		auto p1 = edge->position(edge->n1->id), p2 = edge->position(edge->n2->id);
		edge->property[edge->n1->id].setValue(Vector3(p2 - p1));
		edge->property[edge->n2->id].setValue(Vector3(p1 - p2));
	}
}

void PropagateProximity::propagate(const QSet<QString> &_fixedNodes, Structure::ShapeGraph *graph)
{
	// Convert
	std::set<QString> fixedNodes;
	for (auto s : _fixedNodes) fixedNodes.insert(s);

	// Constraints per part
	QMap < QString, QVector< ProximityConstraint > > constraints;

	// Initialize propagation state
	for (auto n : graph->nodes){
		n->property["propagated"].setValue(false);
		n->property["fixed"].setValue(fixedNodes.count(n->id) != 0);
	}

	/// Find propagation levels:
	// First level:
	QVector < QVector<QString> > propagationLevel(1);
	for (auto nid : fixedNodes) {
		auto n = graph->getNode(nid);
		n->property["propagated"].setValue(true);
		propagationLevel.front() << nid;

		// complement with same height grouping:
		if (false){
			for (auto nid : n->property["height_siblings"].toStringList()){
				propagationLevel.front() << nid;
			}
		}
	}
	// Remaining levels:
	forever{
		QVector<QString> curLevel;
		for (auto & nid : propagationLevel.back())
		{
			for (auto & edge : graph->getEdges(nid))
			{
				if (edge->n1->id == edge->n2->id) continue;
				auto otherNode = edge->otherNode(nid);
				if (otherNode->property["propagated"].toBool()) continue;

				if (!curLevel.contains(otherNode->id)) curLevel << otherNode->id;

				constraints[otherNode->id].push_front(ProximityConstraint(graph->getNode(nid), edge));
			}
		}

		for (auto & nid : curLevel)	graph->getNode(nid)->property["propagated"].setValue(true);
		if (curLevel.isEmpty())	break;
		propagationLevel.push_back(curLevel);
	};

	// Fixed parts do not change
	propagationLevel.removeFirst();

	// Apply constraints
	for (int i = 0; i < propagationLevel.size(); i++)
	{
		for (auto & nid : propagationLevel[i])
		{
			auto n = graph->getNode(nid);

			// Experimental: allow sliding
			if (n->property["isMerged"].toBool())
				continue;

			QVector<ProximityConstraint> current_constraints;
			for (auto c : constraints[n->id])
			{
				// Experimental: allow sliding
				bool fromIsRotation = (graph->hasRelation(c.from->id) && graph->relationOf(c.from->id).type == Structure::Relation::ROTATIONAL);
				if (fromIsRotation && c.from->property["isSplit"].toBool()) 	continue;
				if (fromIsRotation && c.from->property["isMerged"].toBool())	continue;

				current_constraints << c;
			}

			if (current_constraints.isEmpty()) continue;

			if (current_constraints.size() == 1)
			{
				auto & c = current_constraints.front();

				if (c.link->type == Structure::POINT_EDGE)
					n->deformTo(c.coord(), c.start() + c.delta(), true);

				if (c.link->type == Structure::LINE_EDGE)
				{
					Vector3 p1 = c.start() + c.delta();
					Vector3 p2 = c.start2() + c.delta();
					n->deformTwoHandles(c.coords().front(), p1, c.coords().back(), p2);
				}
			}
			else
			{
				auto & c_list = current_constraints;

				if (c_list.size() == 2)
				{
					auto & ca = c_list.front();
					auto & cb = c_list.back();

					if (ca.from->property["isMerged"].toBool() && cb.from->property["isMerged"].toBool())
						continue;

					auto coord_a = ca.coord();
					auto coord_b = cb.coord();

					double dist = (coord_a - coord_b).norm();
					double threshold = 0.2;

					if (dist < threshold)
						n->deformTo(coord_a, ca.start() + ca.delta(), true);
					else
						n->deformTwoHandles(coord_a, ca.start() + ca.delta(), coord_b, cb.start() + cb.delta());
				}
				else
				{
					// Only consider outer most constraints
					QVector<ProximityConstraint> filtered_constraints;
					std::vector< std::array<double, 2> > coords;
					for (auto & c : c_list)
					{
						auto crds = c.coord();
						coords.push_back({ { crds[0], crds[1] } });
					}

					for (auto idx : chull2d::convex_hull_2d(coords)) filtered_constraints << c_list[idx];

					if (filtered_constraints.size() == 2 || n->type() == Structure::CURVE)
					{
						auto & ca = c_list.front();
						auto & cb = c_list.back();
						n->deformTwoHandles(ca.coord(), ca.start() + ca.delta(), cb.coord(), cb.start() + cb.delta());

						continue;
					}
					else
					{
						bool isFixedAround = true;
						for (auto & c : c_list) isFixedAround &= (fixedNodes.count(c.from->id) != 0);
						if (isFixedAround)
						{
							filtered_constraints = c_list;

							std::vector < Eigen::Vector3d > cage;
							for (auto & c : filtered_constraints) cage.push_back(c.link->position(n->id));

							// Rotate cage towards z-axis around center of constraints
							auto best_plane = best_plane_from_points(cage);
							double cage_plane_dot = best_plane.second.dot(Vector3::UnitZ());
							if (cage_plane_dot < 0) best_plane.second *= 1;
							Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(best_plane.second, Vector3::UnitZ());
							for (auto & p : cage) p = (q * (p - best_plane.first)) + best_plane.first;  // now a rotated cage

							QVector<ProximityConstraint> ordered_constraints;
							auto ordered_indices = chull2d::convex_hull_2d(cage);
							for (auto idx : ordered_indices) ordered_constraints << filtered_constraints[idx];
							filtered_constraints = ordered_constraints;
						}
					}

					// Build cage from convex hull of constraints
					std::vector < Eigen::Vector3d > cage;
					for (auto & c : filtered_constraints) cage.push_back(c.link->position(n->id));

					// Translation
					Vector3 oldCenter(0, 0, 0), newCenter(0, 0, 0);
					for (auto & p : cage) oldCenter += p;
					oldCenter /= cage.size();

					// Rotate cage towards z-axis around center of constraints
					auto best_plane = best_plane_from_points(cage);
					double cage_plane_dot = best_plane.second.dot(Vector3::UnitZ());
					if (cage_plane_dot < 0) best_plane.second *= 1;
					Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(best_plane.second, Vector3::UnitZ());
					for (auto & p : cage) p = (q * (p - best_plane.first)) + best_plane.first;  // now a rotated cage

					// DEBUG: cage
					if (false){
						auto ps = new starlab::PolygonSoup;
						QVector<starlab::QVector3> pts;
						for (auto p : cage) pts << p;
						ps->addPoly(pts);
						graph->debug << ps;
					}

					// Compute weights and 'heights' for control points
					auto cpts = n->controlPoints();
					std::vector< std::vector<double> > weights;
					std::vector<double> heights;

					for (auto & p : cpts)
					{
						// rotate control points towards z-axis using same above rotation
						p = (q * (p - best_plane.first)) + best_plane.first;

						// Record "height"
						heights.push_back(p.z());

						// Compute weights
						weights.push_back(MeanValueCoordinates::computeWeights(p[0], p[1], cage));
					}

					// Modify rotated cage to expected positions
					for (size_t i = 0; i < cage.size(); i++)
					{
						Vector3 pj = filtered_constraints[i].start() + filtered_constraints[i].delta();
						Vector3 p = (q * (pj - best_plane.first)) + best_plane.first;
						cage[i][0] = p[0];
						cage[i][1] = p[1];

						newCenter += pj;
					}

					newCenter /= cage.size();
					Vector3 translation = newCenter - oldCenter;

					// Compute new locations along z-plane:
					for (size_t i = 0; i < cpts.size(); i++){
						auto p = MeanValueCoordinates::interpolate2d(weights[i], cage);
						cpts[i][0] = p.first;
						cpts[i][1] = p.second;
						cpts[i][2] = heights[i];
					}

					// Rotate back
					auto qinv = q.inverse();
					for (auto & p : cpts) p = translation + ((qinv * (p - best_plane.first)) + best_plane.first);

					n->setControlPoints(cpts);
				}
			}
		}

		graph->saveKeyframe();
	}
}
