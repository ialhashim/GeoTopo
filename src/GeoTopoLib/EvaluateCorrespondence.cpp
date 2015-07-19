#include "EvaluateCorrespondence.h"

#include "hausdorff.h"

#include "helpers/PhysicsHelper.h"

int EvaluateCorrespondence::numSamples = 5;

Q_DECLARE_METATYPE(Array1D_Vector4d);

Array1D_Vector3 coupledNormals(Structure::Link * l){
	Array1D_Vector3 normals;
	Eigen::Vector4d c1(0,0,0,0), c2(1,1,0,0), midc(0.5,0.5,0,0);
	
	Vector3 v1 = (l->n1->position(c2) - l->n1->position(c1)).normalized();
	Vector3 v2 = (l->n2->position(midc) - l->n1->position(c1)).normalized();

	Vector3 v3 = (l->n2->position(c2) - l->n2->position(c1)).normalized();
	Vector3 v4 = (l->n1->position(midc) - l->n2->position(c1)).normalized();

	normals.push_back(v1.cross(v2).normalized());
	normals.push_back(v3.cross(v4).normalized());

	return normals;
}

Array1D_Vector3 EvaluateCorrespondence::spokesFromLink(Structure::ShapeGraph * shape, Structure::Link * l, bool isFindCoordClosest)
{
	auto samples1 = l->n1->property["samples_coords"].value<Array2D_Vector4d>();
	auto samples2 = l->n2->property["samples_coords"].value<Array2D_Vector4d>();

	if (samples1.empty()) samples1 = EvaluateCorrespondence::sampleNode(shape, l->n1, 0);
	if (samples2.empty()) samples1 = EvaluateCorrespondence::sampleNode(shape, l->n2, 0);

	double min_spoke_len = DBL_MAX;
	Array1D_Vector4d cur_coords(2, Eigen::Vector4d(0, 0, 0, 0));
	std::vector < Array1D_Vector4d > spoke_coords;

	Array1D_Vector3 spokes;
	for (auto rowi : samples1) for (auto ci : rowi)
	for (auto rowj : samples2) for (auto cj : rowj)
	{
		auto spoke = l->n1->position(ci) - l->n2->position(cj);
		spokes.push_back(spoke);

		if ( isFindCoordClosest )
		{
			cur_coords[0] = ci;
			cur_coords[1] = cj;
			spoke_coords.push_back(cur_coords);
			l->property["spoke_coords"].setValue(spoke_coords);
			
			auto sqrd = spoke.squaredNorm();
			if (sqrd < min_spoke_len){
				min_spoke_len = sqrd;
				l->property["spoke_closest_idx"].setValue(int(spoke_coords.size() - 1));
			}
		}
	}

	return spokes;
}

Array2D_Vector4d EvaluateCorrespondence::sampleNode(Structure::ShapeGraph * shape, Structure::Node * n, double resolution)
{
	Array2D_Vector4d samples_coords;

	auto regularSamples = [&](int num_samples, bool isCurve){
		Array2D_Vector4d samplesCoords;
		double step = 1.0 / num_samples;
		for (double u = 0; u <= 1.0; u += step){
			Array1D_Vector4d row;
			for (double v = 0; v <= 1.0; v += step)
				row.push_back(Eigen::Vector4d(v, u, 0, 0));
			samplesCoords.push_back(row);
			if (isCurve) break;
		}
		return samplesCoords;
	};

	samples_coords = regularSamples(numSamples, n->type() != Structure::SHEET);

	n->property["samples_coords"].setValue(samples_coords);

	// Unary node properties
	{
		n->property["orig_diagonal"].setValue(n->diagonal());
		n->property["orig_start"].setValue(n->startPoint());
		n->property["orig_length"].setValue(n->length());

		// Volume
		double volume = 0.0;
		auto mesh = n->property["mesh"].value< QSharedPointer<SurfaceMeshModel> >();

		if (!mesh.isNull())
		{
			if (mesh->property("volume").isValid())
				volume = mesh->property("volume").toDouble();
			else{
				volume = PhysicsHelper(mesh.data()).volume();
				mesh->setProperty("volume", volume);
			}
		}

		n->property["orig_volume"].setValue(volume);

		// Check if is a loop
		double loopDist = n->diagonal().norm();
		double threshold = n->length() * 0.01;
		if (loopDist < threshold) n->property["isLoop"].setValue(true);
	}

	return samples_coords;
}

void EvaluateCorrespondence::prepare(Structure::ShapeGraph * shape)
{
	double sum_length = 0;
	for (auto n : shape->nodes) sum_length += n->length();
	double avg_length = sum_length / shape->nodes.size();

	double resolution = avg_length / numSamples;

	shape->property["sampling_resolution"].setValue(resolution);

	for (auto n : shape->nodes)
		EvaluateCorrespondence::sampleNode(shape, n, resolution);

	// Sample curve/sheet
	for (auto l : shape->edges)
	{
		l->property["orig_spokes"].setValue(EvaluateCorrespondence::spokesFromLink(shape, l, true));
		l->property["orig_centroid_dir"].setValue(Vector3((l->n1->center() - l->n2->center()).normalized()));
	}

	for (auto r : shape->relations)
	{
		// Default solidity is 1.0
		for (auto partID : r.parts)
			shape->getNode(partID)->property["solidity"].setValue(1.0);

		// Compute solidity of non rotational groups
		if (r.type == Structure::Relation::ROTATIONAL)
		{
			for (auto partID : r.parts)
				shape->getNode(partID)->property["solidity"].setValue(1.0 / r.parts.size());
		}
		else if (r.parts.size() > 1 && r.type != Structure::Relation::ROTATIONAL)
		{
			std::vector<Vector3> centers;
			for (auto part : r.parts) centers.push_back(shape->getNode(part)->position(Eigen::Vector4d(0.5, 0.5, 0, 0)));

			auto line = best_line_from_points(centers);
			Vector3 centroid = line.first;
			Vector3 line_direction = line.second;
			Vector3 line_start = line.first - (line_direction * 1000.0);

			Eigen::AlignedBox3d groupBox;

			QVector <double> all_projections;
			double min_part_range = DBL_MAX;
			for (auto partID : r.parts)
			{
				auto n = shape->getNode(partID);
				auto mesh = n->property["mesh"].value< QSharedPointer<SurfaceMeshModel> >();
				if (mesh.isNull()) continue;

				mesh->updateBoundingBox();

				QVector <double> part_projections;
				for (auto v : mesh->vertices())
				{
					double t = (mesh->vertex_coordinates()[v] - line_start).dot(line_direction);
					part_projections << t;
					all_projections << t;
				}
				qSort(part_projections);

				part_projections.size();

				min_part_range = std::min(min_part_range, part_projections.back() - part_projections.front());
			}

			// When no meshes exist..
			if (!all_projections.size())
			{
				for (auto partID : r.parts) shape->getNode(partID)->property["solidity"].setValue(1);
				continue;
			}

			qSort(all_projections);
			double group_range = all_projections.back() - all_projections.front();

			if (min_part_range == DBL_MAX) min_part_range = 1.0;

			double filled = (min_part_range * r.parts.size());

			// Special case: intersection
			if (r.parts.size() == 2){
				double dot_val = shape->getNode(r.parts.front())->diagonal().normalized()
					.dot(shape->getNode(r.parts.back())->diagonal().normalized());
				if (abs(dot_val) < 0.2){
					auto mesh = shape->getNode(r.parts.front())->property["mesh"].value< QSharedPointer<SurfaceMeshModel> >();
					filled = mesh->bbox().sizes().minCoeff();
				}
			}

			double solidity = std::min(0.9, std::max(0.0, filled / group_range));

			for (auto partID : r.parts)
				shape->getNode(partID)->property["solidity"].setValue(solidity);
		}
	}
}

QMap<QString, NanoKdTree*> EvaluateCorrespondence::kdTreesNodes(Structure::ShapeGraph * shape)
{
	QMap < QString, NanoKdTree* > result;

	auto buildKdTree = [](Structure::Node * n, const Array2D_Vector4d & coords){
		auto t = new NanoKdTree;
		for (auto row : coords) for (auto coord : row) t->addPoint(n->position(coord));
		t->build();
		return t;
	};

	for (auto n : shape->nodes)
	{
		auto coords = n->property["samples_coords"].value<Array2D_Vector4d>();
		if (coords.empty()) coords = EvaluateCorrespondence::sampleNode(shape, n, 0);
		result[n->id] = buildKdTree(n, coords);
	}

	return result;
}

QMap<QString, QMap<QString, double> > EvaluateCorrespondence::hausdroffDistance(Structure::ShapeGraph * shapeA, Structure::ShapeGraph * shapeB)
{
	QMap<QString, QMap<QString, double> > result;

	// Initialize search acceleration
	auto trees_a = EvaluateCorrespondence::kdTreesNodes(shapeA);
	auto trees_b = EvaluateCorrespondence::kdTreesNodes(shapeB);

	for (auto nA : shapeA->nodes)
	{
		for (auto nB : shapeB->nodes)
		{
			double distance = hausdroff::distance(trees_a[nA->id], trees_b[nB->id]);
			result[nA->id][nB->id] = distance;
		}
	}

	// Clean up
	for (auto t : trees_a) delete t;
	for (auto t : trees_b) delete t;

	return result;
}

double EvaluateCorrespondence::RMSD(Structure::ShapeGraph * shapeA, Structure::ShapeGraph * shapeB)
{
	// Collect point sets X, Y
	auto sampleShape = [&](Structure::ShapeGraph * shape){
		std::vector<Vector3> samples;
		for (auto n : shape->nodes){
			auto coords = n->property["samples_coords"].value<Array2D_Vector4d>();
			if (coords.empty()) coords = EvaluateCorrespondence::sampleNode(shape, n, 0);
			for (auto row : coords) for (auto c : row) samples.push_back(n->position(c));
		}
		return samples;
	};

	auto X = sampleShape(shapeA);
	auto Y = sampleShape(shapeB);

	// Point query acceleration
	auto buildKdTree = [](std::vector<Vector3> samples){
		auto t = new NanoKdTree;
		for (auto p : samples) t->addPoint(p);
		t->build();
		return QSharedPointer<NanoKdTree>(t);
	};
	QSharedPointer<NanoKdTree> X_tree = buildKdTree(X);
	QSharedPointer<NanoKdTree> Y_tree = buildKdTree(Y);

	auto dist_x_Y = [](QSharedPointer<NanoKdTree> X_tree, QSharedPointer<NanoKdTree> Y_tree){
		auto & X = X_tree->cloud.pts, Y = Y_tree->cloud.pts;
		int n = X.size();
		double sum = 0;
		for (int i = 0; i < n; i++)
		{
			auto & p = X[i];
			KDResults matches;
			Y_tree->k_closest(p, 1, matches); // min_j
			sum += matches.front().second; // (Euclidean distance)^2
		}
		return sum;
	};

	int n = (int)X.size(); // or Y.size()??

	double a = (dist_x_Y(X_tree, Y_tree) + dist_x_Y(Y_tree, X_tree));
	double b = 2.0 * n;

	return sqrt(a / b);
}

double EvaluateCorrespondence::evaluate2(Energy::SearchNode * searchNode)
{
	auto shape = searchNode->shapeA.data();
	auto targetShape = searchNode->shapeB.data();

	QVector<double> feature_vector;

	feature_vector << 0;

	Eigen::Map<Eigen::VectorXd> v(&feature_vector[0], feature_vector.size());
	Eigen::VectorXd original_v = Eigen::VectorXd::Ones(v.size());
	double v_norm = v.norm(), original_norm = original_v.norm();
	double similarity = v_norm / original_norm;
	double cost = 1.0 - similarity;
	return cost;

	return 0;
}

// Value of 1 means perfect matching to original shape, anything lower is bad
double EvaluateCorrespondence::vectorSimilarity(QVector<double> & feature_vector)
{
	Eigen::Map<Eigen::VectorXd> v(&feature_vector[0], feature_vector.size());
	Eigen::VectorXd original_v = Eigen::VectorXd::Ones(v.size());

	double v_norm = v.norm(), original_norm = original_v.norm();

	double similarity = v_norm / original_norm;
	return similarity;
}

double EvaluateCorrespondence::evaluate(Energy::SearchNode * searchNode)
{
	auto shape = searchNode->shapeA.data();
	auto targetShape = searchNode->shapeB.data();

	// Logging
	QVariantMap costMap;

	QVector<double> distortion_vector, split_vector, connection_vector;

	// Unary properties:
	/// Splitting term:
	QMap<QString, double> split_weights;
	QMap<QString, bool> isSeen;
	for (auto n : shape->nodes)
	{
		double volumeRatio = 1.0;
		double before_ratio = 1.0, after_ratio = 1.0;

		// Skip split copies
		if (n->id.contains("@")) continue;

		bool isApplicable = searchNode->mapping.contains(n->id) && n->property.contains("solidity");

		if ( isApplicable /*&& (n->property["isSplit"].toBool() || n->property["isMerged"].toBool() || n->property["isManyMany"].toBool())*/ )
		{
			auto tn = targetShape->getNode(searchNode->mapping[n->id]);
			before_ratio = n->property["solidity"].toDouble();
			after_ratio = tn->property["solidity"].toDouble();

			// Experimental: for reflectional symmetry minimize split factor
			if (n->property["groupParts"].toStringList().size() == tn->property["groupParts"].toStringList().size() &&
				n->property["groupParts"].toStringList().size() == 2)
				before_ratio = after_ratio = 1.0;
		}

		volumeRatio = std::min(before_ratio, after_ratio) / std::max(before_ratio, after_ratio);

		// Experimental: disconnection of loop has fixed penalty, StructureGraphLib does not handle loops well
		if (searchNode->mapping.contains(n->id) && ( n->property["isLoop"].toBool() != 
			targetShape->getNode(searchNode->mapping[n->id])->property["isLoop"].toBool())){
			volumeRatio = 0.2;
		}

		// Experimental: extra penalty from sheet to single curve
		if (isApplicable && 
			n->type() == Structure::SHEET && 
			!n->property["isSplit"].toBool() &&
			targetShape->getNode(searchNode->mapping[n->id])->type() == Structure::CURVE){
			volumeRatio *= 0.8;
		}

		split_weights[n->id] = volumeRatio;

		costMap[QString("N:") + n->id].setValue(QString(" weight = %1 / before %2 / after %3 ")
			.arg(split_weights[n->id]).arg(before_ratio).arg(after_ratio));

		// Avoid redundancy
		if (isSeen[n->id]) continue;
		for (auto pj : n->property["groupParts"].toStringList()) isSeen[pj] = true;

		split_vector << volumeRatio;
	}

	// Binary properties:
	for (auto l : shape->edges)
	{
		auto original_spokes = l->property["orig_spokes"].value<Array1D_Vector3>();
		auto current_spokes = EvaluateCorrespondence::spokesFromLink(shape, l);

		// Sliding
		auto spoke_coords = l->property["spoke_coords"].value< std::vector < Array1D_Vector4d > >();
		auto spoke_closest_idx = l->property["spoke_closest_idx"].toInt();
		auto samples1 = l->n1->property["samples_coords"].value<Array2D_Vector4d>();
		auto samples2 = l->n2->property["samples_coords"].value<Array2D_Vector4d>();

		/// (a) Connection: difference in closeness distance
		double connection_weight = 1.0;
		{
			auto minMaxDist = [](Array1D_Vector3& vectors){
				double mn = DBL_MAX, mx = -DBL_MAX;
				for (auto& p : vectors) { double d = p.norm(); mn = std::min(mn, d); mx = std::max(mx, d); }
				return std::make_pair(mn, mx);
			};

			auto bounds_orig = minMaxDist(original_spokes);
			auto bounds_curr = minMaxDist(current_spokes);

			double connection_weight_near = 1.0, connection_weight_far = 1.0;

			// Near point got further
			if (bounds_curr.first > bounds_orig.first){
				double ratio = std::min(1.0, bounds_curr.first / std::max(1e-6, bounds_orig.second));
				connection_weight_near = 1.0 - ratio;
			}

			// Far point got closer
			//if (bounds_curr.second < bounds_orig.second * 1.25){
			//	connection_weight_far = bounds_curr.second / bounds_orig.second;
			//}

			connection_weight = std::min(connection_weight_near, connection_weight_far);

			// Sliding penalty
			if (false)
			{
				auto orig_closest1 = spoke_coords[spoke_closest_idx].front();
				auto orig_closest2 = spoke_coords[spoke_closest_idx].back();

				auto closestCoords = [&](const Vector3 & p, Structure::Node * n, Array2D_Vector4d coords){
					double min_dist = DBL_MAX;
					Eigen::Vector4d min_coord(0, 0, 0, 0);
					for (auto row : coords){
						for (auto c : row){
							double d = (p - n->position(c)).norm();
							if (d < min_dist){
								min_dist = d;
								min_coord = c;
							}
						}
					}
					return min_coord;
				};

				auto cur_closest1 = closestCoords(l->n1->position(orig_closest1), l->n2, samples2);
				auto cur_closest2 = closestCoords(l->n2->position(orig_closest2), l->n1, samples1);

				double psd1 = (cur_closest1 - orig_closest1).norm() / Eigen::Vector4d(1, 1, 0, 0).norm();
				double psd2 = (cur_closest2 - orig_closest2).norm() / Eigen::Vector4d(1, 1, 0, 0).norm();
				double parameter_space_dist = (psd1 + psd2) / 2.0;

				connection_weight *= (1.0 - parameter_space_dist);
			}

			// Semantically broken (two parts correspond to single target)
			if (true)
			{
				if (searchNode->mapping.contains(l->n1->id) && searchNode->mapping.contains(l->n2->id)
					&& searchNode->mapping[l->n1->id] == searchNode->mapping[l->n2->id])
					connection_weight = 0.5;
			}

			connection_vector << connection_weight;
		}

		QVector < double > link_vector;

		// Deformation of structural rods
		for (size_t i = 0; i < original_spokes.size(); i++)
		{
			double v = original_spokes[i].normalized().dot(current_spokes[i].normalized());

			// Bad values e.g. nan are broken feature values
			if (!std::isfinite(v)) v = 0;

			// Bound: the original edge is broken beyond a certain point
			v = std::max(0.0, v);

			link_vector << v;
		}

		// Should not happen..
		if (original_spokes.size() == 0) link_vector << 0;

		for (auto v : link_vector) distortion_vector << v;

		double sum_link_vector = 0; for (auto v : link_vector) sum_link_vector += v;
		double avg_link_vector = sum_link_vector / link_vector.size();

		// Logging:
		qSort(link_vector);
		costMap[QString("L:") + l->id].setValue(QString("[%1, %2] w_conn = %3")
			.arg(link_vector.front()).arg(link_vector.back()).arg(connection_weight));
	}

	double distortion_cost = 1.0 - vectorSimilarity(distortion_vector);
	double split_cost = 1.0 - vectorSimilarity(split_vector);
	double connection_cost = 1.0 - vectorSimilarity(connection_vector);

	// Weights
	double w_d = 1.1, w_s = 0.4, w_c = 0.6;

	// Normalize weights
	double total_weights = w_d + w_s + w_c;
	w_d /= total_weights; w_s /= total_weights; w_c /= total_weights;

	// Compute energy:
	double total_cost = (w_d * distortion_cost) + (w_s * split_cost) + (w_c * connection_cost);

	// Logging:
    costMap["zzShapeCost"].setValue(QString("Total (%1) = Distortion (%2) + (w_s) Split (%3) + (w_c) Connection(%4)")
		.arg(total_cost).arg(distortion_cost).arg(split_cost).arg(connection_cost));
 	shape->property["costs"].setValue(costMap);

	return total_cost;
}
