#include "StructureAnalysis.h"
#include "ShapeGraph.h"
#include "GenericGraph.h"
#include "helpers/PhysicsHelper.h"
#include "disjointset.h"

void StructureAnalysis::analyzeGroups(Structure::ShapeGraph * shape, bool isDebug)
{
	shape->relations.clear();

	QStringList nodesIDs;
	Structure::NodeGroups tmpA;
	for (auto n : shape->nodes) nodesIDs << n->id;
	for (auto g : shape->groups) for (auto nid : g) nodesIDs.removeAll(nid);
	for (auto nid : nodesIDs) tmpA.push_back(QVector<QString>() << nid);
	for (auto g : shape->groups) tmpA.push_back(g);
	shape->groups = tmpA;

	for (auto g : shape->groups)
	{
		Structure::Relation r;

		r.parts = QStringList::fromVector(g);

		if (g.size() == 1)
		{
			r.type = Structure::Relation::SELF;
			// TODO: figure out the plane, or select the most similar to global reflectional
		}

		// Since fitted input might not always be perfect, we equalize parts geometry (resample) here
		{
			auto firstPart = shape->getNode(r.parts.front());
			for (auto partID : r.parts)
				if (shape->getNode(partID)->numCtrlPnts() > firstPart->numCtrlPnts())
					firstPart = shape->getNode(partID);

			for (auto partID : r.parts)
			{
				auto smaller = firstPart;
				auto larger = shape->getNode(partID);
				if (smaller->numCtrlPnts() > larger->numCtrlPnts()) std::swap(smaller, larger);
				smaller->equalizeControlPoints(larger);
			}
		}

		if (g.size() == 2)
		{
			r.type = Structure::Relation::REFLECTIONAL;

			auto partA = shape->getNode(g.front()), partB = shape->getNode(g.back());

			auto plane = getReflectionalPlane(partA, partB);
			r.point = plane.first;
			r.axis = plane.second;
		}

		std::vector<Vector3> centers;			
		Vector3 centroid(0, 0, 0);

		if (g.size() > 2)
		{
			// Compute distances from group centroid to parts
			for (auto part : r.parts) centers.push_back( shape->getNode(part)->position(Eigen::Vector4d(0.5,0.5,0,0)) );

			for (auto c : centers) centroid += c;
			centroid /= centers.size();
			Array1D_Real cdists;
			for (auto c : centers) cdists.push_back((c - centroid).norm());

			// Compute closest pair distance 
			std::vector< QVector<double> > pair_dists(g.size());
			Array1D_Real dists(g.size());
			for (int i = 0; i < g.size(); i++){
				for (int j = 0; j < g.size(); j++){
					if (i==j) continue;
					double d = (centers.at(i) - centers.at(j)).norm();
					pair_dists[i] << d;
				}
				std::sort(pair_dists[i].begin(), pair_dists[i].end());
				dists[i] = (pair_dists[i][0] + pair_dists[i][1]) / 2.0;
			}
			double avg_dist_pair = 0;
			for (auto d : dists) avg_dist_pair += d;
			avg_dist_pair /= dists.size();

			if (*std::min_element(cdists.begin(), cdists.end()) < avg_dist_pair * 0.5)
				r.type = Structure::Relation::TRANSLATIONAL;
			else
				r.type = Structure::Relation::ROTATIONAL;

			/*
			// Median absolute deviation (MAD) 			
			double avg_dist = std::accumulate(dists.begin(), dists.end(), 0.0) / dists.size();
			QVector<double> absolute_deviations;
			for (auto dist : dists) absolute_deviations << abs(dist - avg_dist);
			std::sort(absolute_deviations.begin(), absolute_deviations.end());
			double mad = 0;
			if (absolute_deviations.size() % 2 == 1)
				mad = absolute_deviations[(absolute_deviations.size() - 1) / 2];
			else
			{
				auto a = absolute_deviations[(absolute_deviations.size() / 2) - 1];
				auto b = absolute_deviations[(absolute_deviations.size() / 2)];
				mad = (a + b) / 2;
			}

			// Rotational relations have consistent distance
			double threshold = avg_dist * 0.45;
			if (mad < threshold) 
				r.type = Structure::Relation::ROTATIONAL;
			else 
				r.type = Structure::Relation::TRANSLATIONAL;
			*/

			if (r.type == Structure::Relation::TRANSLATIONAL)
			{
				auto line = best_line_from_points(centers);
				r.point = line.first;
				r.axis = line.second;

				// Sort parts in relation
				{
					Vector3 point, direction;
					MatrixXd curveCenters(r.parts.size(), 3);
					for (int pi = 0; pi < (int)r.parts.size(); pi++) curveCenters.row(pi) = shape->getNode(r.parts[pi])->position(Eigen::Vector4d(0.5, 0.5, 0, 0));
					point = Vector3(curveCenters.colwise().mean());
					curveCenters = curveCenters.rowwise() - point.transpose();
					Eigen::JacobiSVD<Eigen::MatrixXd> svd(curveCenters, Eigen::ComputeThinU | Eigen::ComputeThinV);
					direction = Vector3(svd.matrixV().col(0)).normalized();
					std::vector <size_t> sorted;
					QMap<size_t, double> dists;
					for (size_t pi = 0; pi < (int)r.parts.size(); pi++) dists[pi] = curveCenters.row(pi).dot(direction);
					for (auto p : sortQMapByValue(dists)) sorted.push_back(p.second);

					QStringList sortedParts;
					for (auto idx : sorted) sortedParts << r.parts[idx];
					r.parts = sortedParts;
				}
			}

			if (r.type == Structure::Relation::ROTATIONAL)
			{
				auto plane = best_plane_from_points(centers);
				r.point = plane.first;
				r.axis = plane.second;

				// Sort parts by angle around axis
				QStringList sorted;
				QMap<QString, double> angles;
				for (size_t i = 0; i < r.parts.size(); i++)
				{
					double angle = signedAngle(centers[0], centers[i], r.axis);
					if (angle < 0) angle = (M_PI * 2) + angle;
					angles[r.parts[i]] = angle;
				}
				for (auto pair : sortQMapByValue(angles)) sorted << pair.second;
				r.parts = sorted;

				// Vector from part's head to centroid
				for (size_t i = 0; i < r.parts.size(); i++)
					r.deltas.push_back(centroid - shape->getNode(r.parts[i])->position(Eigen::Vector4d(0.5, 0.5, 0, 0)));
			}
		}

		// Record volume of relation
		{
			double relationVolume = 0.0;
			for (auto partID : r.parts){
				auto mesh = shape->getNode(partID)->property["mesh"].value< QSharedPointer<SurfaceMeshModel> >();
				if (mesh.isNull()) continue;
				relationVolume += PhysicsHelper(mesh.data()).volume();
			}
			if (relationVolume == 0.0) relationVolume = 1.0;
			r.property["volume"].setValue(relationVolume);
		}

		// Add relation to shape:
		shape->relations.push_back(r);

		for (auto partID : r.parts)
		{
			auto node = shape->getNode(partID);
			node->property["groupParts"].setValue(r.parts);
			if (r.type == Structure::Relation::ROTATIONAL) node->property["isRotational"].setValue(true);
		}

		// Visualize:
		if (isDebug)
		{
			shape->debug << starlab::PointSoup::drawPoint(r.point, 12, Qt::red);

			if (r.type != Structure::Relation::TRANSLATIONAL)
			{
				auto plane = new starlab::PlaneSoup(0.1, true, r.type == Structure::Relation::REFLECTIONAL ? Qt::red : Qt::green);
				plane->addPlane(r.point, r.axis);
				shape->debug << plane;
			}
			else
			{
				auto line = new starlab::LineSegments(3);
				line->addLine(r.point, Vector3(r.point + r.axis));
				shape->debug << line;
			}
		}
	}

	// Same height grouping
	{
		DisjointSet set(shape->nodes.size());

		double range_z = shape->robustBBox().sizes().z();
		double threshold = 0.05;

		QMap < Structure::Node*, int > idx_map;
		int idx = 0; for (auto n : shape->nodes) idx_map[n] = idx++;

		for (auto ni : shape->nodes){
			for (auto nj : shape->nodes){
				double zi = ni->center().z();
				double zj = nj->center().z();
				double dist = abs(zi - zj);
				if (dist < range_z * threshold) set.Union(idx_map[ni], idx_map[nj]);
			}
		}

		auto hight_groups  = set.Groups();

		for (auto hight_group : hight_groups){
			for (auto ni : hight_group){
				auto node_i = shape->nodes.at(ni);
				QStringList height_siblings;

				for (auto nj : hight_group){
					if (ni == nj) continue;
					auto node_j = shape->nodes.at(nj);
					height_siblings << node_j->id;
				}

				node_i->property.insert("height_siblings", height_siblings);
			}
		}
	}
}

std::pair<Vector3, Vector3> StructureAnalysis::getReflectionalPlane(Structure::Node * n1, Structure::Node * n2)
{
	Vector3 point(0, 0, 0), axis(0, 0, 0);

	int num_samples = 10;

	for (int i = 0; i < num_samples; i++)
	{
		double t = double(i) / (num_samples - 1);
		auto p = n1->position(Eigen::Vector4d(t, t, 0, 0));
		auto q = n2->position(Eigen::Vector4d(t, t, 0, 0));

		point += (p + q) * 0.5;
		axis += (p - q).normalized();
	}

	point /= num_samples;
	axis /= num_samples;
	axis.normalize();

	return std::make_pair(point, axis);
}

SurfaceMesh::Vector3 StructureAnalysis::pointReflection(const Vector3 & p, const Vector3 & planePoint, const Vector3 & planeNormal)
{
	double d = (p - planePoint).dot(planeNormal);
	Vector3 v = planeNormal * d;
	return p - (v * 2);
}

void StructureAnalysis::removeFromGroups(Structure::ShapeGraph * shape, Structure::Node * node)
{
	for (auto & r : shape->relations)
		r.parts.removeAll(node->id);
}

void StructureAnalysis::updateRelation(Structure::ShapeGraph * shape, Structure::Relation & r)
{
	std::vector<Vector3> centers;
	Vector3 centroid(0, 0, 0);

	for (auto part : r.parts) centers.push_back(shape->getNode(part)->position(Eigen::Vector4d(0.5, 0.5, 0, 0)));

	if (r.type == Structure::Relation::REFLECTIONAL)
	{

	}

	if (r.type == Structure::Relation::ROTATIONAL)
	{
		auto plane = best_plane_from_points(centers);
		r.point = plane.first;
		r.axis = plane.second;

		// Vector from part's head to centroid
		for (size_t i = 0; i < r.parts.size(); i++)
			r.deltas[i] = centroid - shape->getNode(r.parts[i])->position(Eigen::Vector4d(0.5, 0.5, 0, 0));
	}
}
