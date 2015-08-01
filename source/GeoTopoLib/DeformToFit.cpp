#include "DeformToFit.h"
#include "StructureGraph.h"

template<typename T, typename MapType>
static T linear_interpolate(const double &x, const MapType &table){
    assert(table.size() > 0);
    auto it = table.lower_bound(x);
    if (it == table.end())
        return table.rbegin()->second;
    else
    {
        if (it == table.begin())
            return it->second;
        else
        {
            double x2 = it->first;
            T y2 = it->second;
            --it;
            double x1 = it->first;
            T y1 = it->second;
            double p = (x - x1) / (x2 - x1);
            return (1 - p) * y1 + p * y2;
        }
    }
}

template<typename T, typename MapType>
static std::tuple<size_t, size_t, double> linear_interpolate_weight(const double &x, const MapType &table){
	assert(table.size() > 0);
	auto it = table.lower_bound(x);
	if (it == table.end())
		return std::make_tuple(table.size() - 2, table.size() - 1, 0.0);
	else
	{
		if (it == table.begin())
			return std::make_tuple(0, 1, 0);
		else
		{
			double x2 = it->first;
			--it;
			double x1 = it->first;
			double p = (x - x1) / (x2 - x1);
			auto idx = std::distance(table.begin(), it);
			return std::make_tuple(idx, idx + 1, p);
		}
	}
}

static inline Vector3 vec_lerp(const Vector3& a, const Vector3& b, double u){
	return (a * (1-u)) + (b * u);
}

static inline Vector3 quad_interpolate(const Vector3 a, const Vector3 b, const Vector3 c, const Vector3 d, double u, double v)
{
	Vector3 ab = vec_lerp(a, b, u);
	Vector3 cd = vec_lerp(c, d, u);
	return vec_lerp(ab, cd, v);
}

static inline Vector3 small_noise(double scale = 1.0){
	return Vector3::Random() * scale;
}

void DeformToFit::registerAndDeformNodes(Structure::Node * snode, Structure::Node * tnode)
{
    auto scenter = snode->position(Eigen::Vector4d(0.5, 0.5, 0, 0));
    auto tcenter = tnode->position(Eigen::Vector4d(0.5, 0.5, 0, 0));

    auto translation = Vector3(tcenter - scenter);

	if (snode->type() == tnode->type())
	{
		if (snode->type() == Structure::CURVE)
		{
			auto scpts = snode->controlPoints();
			auto tcpts = tnode->controlPoints();

			for (auto & p : tcpts) p -= translation;

			// Register
			Vector3 sfront = scpts.front();
			Vector3 tfront = tcpts.front();
			Vector3 tback = tcpts.back();
			bool isReverse = (sfront - tfront).norm() > (sfront - tback).norm() ? true : false;
			if (isReverse) std::reverse(tcpts.begin(), tcpts.end());

			// Encode target curve as deltas from center
			std::map < double, Vector3 > deltas;
			for (size_t i = 0; i < tcpts.size(); i++){
				double t = double(i) / (tcpts.size() - 1);
				deltas[t] = tcpts[i] - scenter;
			}

			// Deform source curve from deltas
			for (size_t i = 0; i < scpts.size(); i++){
				double t = double(i) / (scpts.size() - 1);
				scpts[i] = (scenter + translation) + linear_interpolate<Vector3>(t, deltas);
			}

			// Apply deformed control points
			snode->setControlPoints(scpts);
		}

		if (snode->type() == Structure::SHEET)
		{
			Structure::Sheet * ssheet = (Structure::Sheet*) snode;
			Structure::Sheet * tsheet = (Structure::Sheet*) tnode;

			auto ssurface = ssheet->surface;
			auto tsurface = tsheet->surface;

			// Remove translation
			tsurface.translate(-translation);
			
			// Minimize rotation
			Vector3 tpos, tu, tv, tnormal;
			tsurface.GetFrame(0.5, 0.5, tpos, tu, tv, tnormal);
			Vector3 spos, su, sv, snormal;
			ssurface.GetFrame(0.5, 0.5, spos, su, sv, snormal);

			if (snormal.dot(tnormal) < 0) 
				tnormal *= -1;

			Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(snormal, tnormal);
			for (auto & row : tsurface.mCtrlPoint) for (auto & p : row) p = (q.inverse() * (p - scenter)) + scenter;

			QMap < double, QVector<double> > dists;
			for (double u = 0; u <= 1.0; u += 1.0){
				for (double v = 0; v <= 1.0; v += 1.0){
					for (double i = 0; i <= 1.0; i += 1.0){
						for (double j = 0; j <= 1.0; j += 1.0){
							dists[(ssurface.P(u, v) - tsurface.P(i, j)).norm()] = (QVector<double>() << u << v << i << j);
						}
					}
				}
			}

			auto bestChoice = dists.values().front();
			bool isReverseU = bestChoice[0] != bestChoice[2], isReverseV = bestChoice[1] != bestChoice[3];

			// Reverse if needed
			if ( isReverseV ){
				for (int i = 0; i < (int)tsurface.mCtrlPoint.size(); i++){
					std::reverse(tsurface.mCtrlPoint[i].begin(), tsurface.mCtrlPoint[i].end());
					std::reverse(tsurface.mCtrlWeight[i].begin(), tsurface.mCtrlWeight[i].end());
				}
			}
			if( isReverseU ){
				std::reverse(tsurface.mCtrlPoint.begin(), tsurface.mCtrlPoint.end());
				std::reverse(tsurface.mCtrlWeight.begin(), tsurface.mCtrlWeight.end());
			}

			std::map < double, size_t > mapU, mapV;
			for (size_t i = 0; i < tsurface.mNumUCtrlPoints; i++) mapU[double(i) / (tsurface.mNumUCtrlPoints - 1)] = i;
			for (size_t j = 0; j < tsurface.mNumVCtrlPoints; j++) mapV[double(j) / (tsurface.mNumVCtrlPoints - 1)] = j;

			auto getQuad = [&](size_t u, size_t v, Array2D_Vector3& cpts){ 
				return QVector<Vector3>() << cpts[u][v] << cpts[u+1][v] << cpts[u][v+1] << cpts[u+1][v+1];
			};

			for (size_t i = 0; i < ssurface.mNumUCtrlPoints; i++){
				for (size_t j = 0; j < ssurface.mNumVCtrlPoints; j++){
					double u = double(i) / (ssurface.mNumUCtrlPoints-1);
					double v = double(j) / (ssurface.mNumVCtrlPoints-1);

					auto weight_u = linear_interpolate_weight<size_t>(u, mapU);
					auto weight_v = linear_interpolate_weight<size_t>(v, mapV);

					std::pair <size_t, size_t> quad_idx(std::get<0>(weight_u), std::get<0>(weight_v));
					std::pair <double, double> quad_uv(std::get<2>(weight_u), std::get<2>(weight_v));

					auto quad = getQuad(quad_idx.first, quad_idx.second, tsurface.mCtrlPoint);

					auto interp = quad_interpolate(quad[0], quad[1], quad[2], quad[3], quad_uv.first, quad_uv.second);

					ssurface.mCtrlPoint[i][j] = interp;
				}
			}

			// Apply rotation
			for (auto & row : ssurface.mCtrlPoint) for (auto & p : row) p = (q * (p - scenter)) + tcenter;

			ssheet->surface.mCtrlPoint = ssurface.mCtrlPoint;
			ssheet->surface.quads.clear();
        }
    }
    else
    {
		Structure::Curve curve((snode->type() == Structure::CURVE) ? (*(Structure::Curve*)snode) : (*(Structure::Curve*)tnode));
		Structure::Sheet sheet((snode->type() == Structure::SHEET) ? (*(Structure::Sheet*)snode) : (*(Structure::Sheet*)tnode));

		double minU = std::min((sheet.surface.P(0, 0) - sheet.surface.P(1, 0)).norm(), (sheet.surface.P(0, 1) - sheet.surface.P(1, 1)).norm());
		double minV = std::min((sheet.surface.P(0, 0) - sheet.surface.P(0, 1)).norm(), (sheet.surface.P(1, 0) - sheet.surface.P(1, 1)).norm());
		bool isProjectAlongU = minU < minV;

		// Roll up sheet
		int idx = (isProjectAlongU) ? (sheet.surface.mNumUCtrlPoints - 1) * 0.5 : (sheet.surface.mNumVCtrlPoints - 1) * 0.5;
		Array1D_Vector3 projection = (isProjectAlongU) ? sheet.surface.GetControlPointsV(idx) : sheet.surface.GetControlPointsU(idx);
		Array2D_Vector3 ctrlPnts(sheet.surface.mNumUCtrlPoints);
		if (isProjectAlongU){
			ctrlPnts = Array2D_Vector3(sheet.surface.mNumUCtrlPoints, projection);
		}else{
			for (size_t i = 0; i < sheet.surface.mNumUCtrlPoints; i++)
				ctrlPnts[i] = Array1D_Vector3(sheet.surface.mNumVCtrlPoints, projection[i]);
		}
		sheet.surface.mCtrlPoint = ctrlPnts;

		Structure::Curve curveFromSheet(NURBS::NURBSCurved::createCurveFromPoints(isProjectAlongU ?
			sheet.surface.GetControlPointsV(0) : sheet.surface.GetControlPointsU(0)), "temp");

		// Sheet to curve case:
		if (snode->type() == Structure::SHEET)
		{
			DeformToFit::registerAndDeformNodes(&curveFromSheet, &curve);

			if (isProjectAlongU)
				sheet.surface.mCtrlPoint = Array2D_Vector3(sheet.surface.mNumUCtrlPoints, curveFromSheet.curve.mCtrlPoint);
			else{
				for (size_t i = 0; i < sheet.surface.mNumUCtrlPoints; i++)
					sheet.surface.mCtrlPoint[i] = Array1D_Vector3(sheet.surface.mNumVCtrlPoints, curveFromSheet.curve.mCtrlPoint[i]);
			}

			((Structure::Sheet*)snode)->surface.mCtrlPoint = sheet.surface.mCtrlPoint;
		}

		// Curve to sheet case:
		if (snode->type() == Structure::CURVE)
		{
			DeformToFit::registerAndDeformNodes(snode, &curveFromSheet);
		}
    }
}
