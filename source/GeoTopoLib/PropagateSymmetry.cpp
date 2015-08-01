#include "PropagateSymmetry.h"
#include "ShapeGraph.h"
#include "StructureAnalysis.h"

void PropagateSymmetry::propagate(const QSet<QString> &fixedNodes, Structure::ShapeGraph *graph)
{
	for (auto & relation : graph->relations)
	{
		if (relation.parts.isEmpty()) continue;

		// Experimental: allow sliding
		{
			auto rep = graph->getNode(relation.parts.front());
			if (rep->property["isSplit"].toBool() || rep->property["isMerged"].toBool()) 
				continue;
		}

		// If all nodes in relation fixed, it is a fixed relation
		{
			bool isFullyFixed = true;
			for (auto partID : relation.parts) if (!fixedNodes.contains(partID)) { isFullyFixed = false; break; }
			if (isFullyFixed) continue;
		}

		if (relation.type == Structure::Relation::REFLECTIONAL)
		{
			auto partA = graph->getNode(relation.parts.front()), partB = graph->getNode(relation.parts.back());
			if (partA == partB) continue;
			if (!fixedNodes.contains(partA->id)) std::swap(partA, partB);
			
			// Apply reflection to control points of other part
			Array1D_Vector3 cptsA = partA->controlPoints();
			Array1D_Vector3 cptsB;

			Eigen::Vector4d c(0.5, 0.5, 0, 0);
			Vector3 delta = partA->position(c) - partB->position(c);
			double d = delta.norm() * 0.5;
			double direction = delta.normalized().dot(relation.axis) > 0 ? -1 : 1;
			Vector3 planePos = partA->position(c) + (relation.axis * d * direction);

			for (auto p : cptsA) cptsB.push_back(StructureAnalysis::pointReflection(p, planePos, relation.axis));

			partB->setControlPoints(cptsB);
		}

		if (relation.type == Structure::Relation::ROTATIONAL)
		{
			int n_fold = relation.parts.size();
			if (n_fold < 2) continue;

			double theta = 2.0 * M_PI / n_fold;

			auto part = graph->getNode(relation.parts.front());
			for (auto partID : relation.parts) if (fixedNodes.contains(partID)){ part = graph->getNode(partID); break; }

			int idx = relation.parts.indexOf(part->id);
			auto start = part->position(Eigen::Vector4d(0.5, 0.5, 0, 0));
			Vector3 centroid = start + relation.deltas[idx];

			auto cpts = part->controlPoints();

			double angle = theta;

			for (size_t i = idx; i < idx + relation.parts.size(); i++)
			{
				if (idx == i) continue;
				size_t j = i % relation.parts.size();
				auto part_j = graph->getNode(relation.parts[j]);

				auto cur_cpts = cpts;
				Eigen::AngleAxisd rotation(angle, relation.axis);
				for (auto & p : cur_cpts) p = (rotation * (p-centroid)) + centroid;

				part_j->setControlPoints(cur_cpts);

				angle += theta;
			}
		}

		if (relation.type == Structure::Relation::TRANSLATIONAL)
		{
			if (relation.parts.size() < 2) continue;
			auto part = graph->getNode(relation.parts.front());
			for (auto partID : relation.parts) if (fixedNodes.contains(partID)){ part = graph->getNode(partID); break; }

			auto cpts_local = part->controlPoints();
			Vector3 centroid = part->position(Eigen::Vector4d(0.5,0.5,0,0));
			for (auto & p : cpts_local) p -= centroid;

			for (auto partID : relation.parts)
			{
				auto cur_part = graph->getNode(partID);
				Vector3 centroid = cur_part->position(Eigen::Vector4d(0.5, 0.5, 0, 0));
				auto cur_cpts = cpts_local;
				for (auto & p : cur_cpts) p += centroid;

				cur_part->setControlPoints(cur_cpts);
			}
		}
	}
}
