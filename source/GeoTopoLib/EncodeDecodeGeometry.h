#pragma once
#include "ShapeGraph.h"
#include "Synthesizer.h"

#include "myglobals.h"

using namespace Eigen;

namespace ShapeGeometry
{
inline void encodeGeometry(Structure::ShapeGraph * g)
{
    if (g->property["isGeometryEncoded"].toBool()) return;

    for (auto n : g->nodes)
    {
        auto mesh = g->getMesh(n->id);
        if (!mesh) continue;
        std::vector < Vector3f > points, normals;
        auto mesh_points = mesh->vertex_coordinates(), mesh_normals = mesh->vertex_normals();
        for (auto v : mesh->vertices()){
            points.push_back(mesh_points[v].cast<float>());
            normals.push_back(mesh_normals[v].cast<float>());
        }

        QVector<ParameterCoord> encoding;
        if (n->type() == Structure::CURVE) encoding = Synthesizer::genPointCoordsCurve((Structure::Curve*)n, points, normals);
        if (n->type() == Structure::SHEET) encoding = Synthesizer::genPointCoordsSheet((Structure::Sheet*)n, points, normals);
        n->property["encoding"].setValue(encoding);
    }

    g->property["isGeometryEncoded"].setValue(true);
}

inline void decodeGeometry(Structure::ShapeGraph * g)
{
    for (auto n : g->nodes)
    {
        auto mesh = g->getMesh(n->id);
        if (!mesh) continue;
        auto mesh_points = mesh->vertex_coordinates();
        QVector<ParameterCoord> encoding = n->property["encoding"].value< QVector<ParameterCoord> >();

        if (encoding.empty()) continue;

        // Generate consistent frames along curve
        Array1D_Vector4 coords;
        RMF rmf;
        if (n->type() == Structure::CURVE) rmf = Synthesizer::consistentFrame((Structure::Curve*)n, coords);

        // Collapsed sheet
        if (n->type() == Structure::SHEET)
        {
            auto sheet = (Structure::Sheet*)n;
            Structure::Curve curve_u(NURBS::NURBSCurved::createCurveFromPoints(sheet->surface.GetControlPointsU(0)), "curveU");
            Structure::Curve curve_v(NURBS::NURBSCurved::createCurveFromPoints(sheet->surface.GetControlPointsV(0)), "curveV");
            if (curve_u.area() < 1e-6) rmf = Synthesizer::consistentFrame(&curve_v, coords);
            if (curve_v.area() < 1e-6) rmf = Synthesizer::consistentFrame(&curve_u, coords);
        }

        for (int i = 0; i < encoding.size(); i++){
            auto & sample = encoding[i];

            Vector3d startPoint;
            std::vector<Vector3d> frame;
            n->get(Eigen::Vector4d(sample.u, sample.v, 0, 0), startPoint, frame);
            Vector3f rayPos = Vector3f(startPoint[0], startPoint[1], startPoint[2]);

            Vector3d _X, _Y, _Z;
            if (!rmf.U.empty())
            {
                int idx = sample.u * (rmf.count() - 1);
                _X = rmf.U[idx].r; _Y = rmf.U[idx].s; _Z = rmf.U[idx].t;
            }
            else
            {
                _X = frame[0]; _Z = frame[2]; _Y = _Z.cross(_X);
            }

            // double float
            Vector3f X(_X[0], _X[1], _X[2]), Y(_Y[0], _Y[1], _Y[2]), Z(_Z[0], _Z[1], _Z[2]);
            Vector3f rayDir;
            localSphericalToGlobal(X, Y, Z, sample.theta, sample.psi, rayDir);

            // Reconstructed point
            Vector3f isect = rayPos + (rayDir * sample.origOffset);
            mesh_points[opengp::SurfaceMesh::SurfaceMeshModel::Vertex(i)] = isect.cast<double>();
        }

        mesh->update_face_normals();
        mesh->update_vertex_normals();
    }
}
}
