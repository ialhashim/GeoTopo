#pragma once
// Based on "Computation of Rotation Minimizing Frames" Wang et al. 2008

#include <vector>
typedef unsigned int uint;

#define ZERO_NORM 1e-7

#include "SurfaceMeshModel.h"
using namespace SurfaceMesh;

class RMF{
public:
    RMF(){}

    RMF(const std::vector<Vector3> & fromPoints, bool isCompute = true)
    {
        if(fromPoints.size() == 0) return;

        point = fromPoints;

        if(fromPoints.size() > 1 && (fromPoints[0] - fromPoints[1]).norm() > ZERO_NORM)
        {
            if(isCompute) compute();
        }
        else
        {
            U.push_back(Frame::fromRS(Vector3(1,0,0), Vector3(0,1,0)));
            U.back().center = fromPoints.front();
        }
    }

    void compute()
    {
        Vector3 firstT = (point[1] - point[0]).normalized();
        //Vector3 firstR = orthogonalVector( firstT );

        Vector3 T(1,  1, 1); T.normalize();
        Vector3 R(1, -1, 0); R.normalize();
        auto q = Eigen::Quaterniond::FromTwoVectors(T, firstT);
        Vector3 r = q * R;

        Vector3 firstR(r[0], r[1], r[2]);
        compute( firstR );
    }

    void compute( Vector3 firstR )
    {
        // Reset computation
        std::vector<Vector3> tangent;

        // Estimate tangents
        for(uint i = 0; i < point.size() - 1; i++){
            Vector3 t = point[i+1] - point[i];
            if(i > 0 && t.norm() < ZERO_NORM)
                tangent.push_back(tangent.back());
            else
                tangent.push_back((t).normalized());
        }
        tangent.push_back(tangent.back());

        Vector3 firstT = tangent.front().normalized();

        // Make sure firstR is perpendicular to plane of first tangent
        firstR = pointOnPlane(firstR, firstT);
        if(firstR.norm() > ZERO_NORM)
            firstR.normalize();
        else
            firstR = Frame::orthogonalVector(firstT);

        // First frame
        Frame firstFrame = Frame::fromTR( firstT, firstR.normalized() );

        U.clear();
        U.push_back( firstFrame );

        // Double reflection method: compute rotation minimizing frames
        for(uint i = 0; i < point.size() - 1; i++)
        {
            Vector3 ri = U.back().r, ti = U.back().t, tj = tangent[i+1];

            /*1 */ Vector3 v1 = point[i+1] - point[i]; if(v1.norm() < ZERO_NORM){ U.push_back(U.back()); continue; }
            /*2 */ double c1 = v1.dot(v1);
            /*3 */ Vector3 rLi = ri - (2.0 / c1) * v1.dot(ri) * v1;
            /*4 */ Vector3 tLi = ti - (2.0 / c1) * v1.dot(ti) * v1;
            /*5 */ Vector3 v2 = tj - tLi;
            /*6 */ double c2 = v2.dot(v2);
            /*7 */ Vector3 rj = rLi - (2.0 / c2) * v2.dot(rLi) * v2;
            /*8 */ Vector3 sj = tj.cross(rj);

            U.push_back(Frame::fromST(sj.normalized(), tj.normalized()));
        }

        // RMF Visualization
        for(int i = 0; i < (int)point.size(); i++)
            U[i].center = point[i];
    }

    void generate()
    {
        std::vector<Vector3> tangent;

        // Estimate tangents
        for(uint i = 0; i < point.size() - 1; i++){
            Vector3 t = point[i+1] - point[i];
            if(i > 0 && t.norm() < ZERO_NORM)
                tangent.push_back(tangent.back());
            else
                tangent.push_back((t).normalized());
        }
        tangent.push_back(tangent.back());

        // generate frames
        // by rotating a fixed reference frame (R, S, T) to match T with the curve tangent
        // the generated frames are not consistent the tangent is close to T
        // which will create twisting artifacts
        // That is why we picked up (1, 1, 1) as T with the assumption
        // that no curve has tangent along this direction
        Vector3 T(1,  1, 1); T.normalize();
        Vector3 R(1, -1, 0); R.normalize();
        for(uint i = 0; i < point.size(); i++)
        {
            Vector3 t = tangent[i];

            auto q = Eigen::Quaterniond::FromTwoVectors(T, t);
            Vector3 r = q * R;

            U.push_back(Frame::fromTR(t, r));
        }

        // RMF Visualization
        for(int i = 0; i < (int)point.size(); i++)
            U[i].center = point[i];
    }

    inline size_t count() { return U.size(); }

    class Frame{
    public:
        Vector3 r, s, t;
        Vector3 center; // optional

        Frame(){ r = Vector3(1,0,0); s = Vector3(0,1,0); t = Vector3(0,0,1); }
        Frame(const Vector3& R, const Vector3& S, const Vector3& T) { r = R; s = S; t = T; normalize(); }

        static Frame fromTR(const Vector3& T, const Vector3& R) { return Frame(R, T.cross(R), T); }
        static Frame fromRS(const Vector3& R, const Vector3& S) { return Frame(R, S, R.cross(S)); }
        static Frame fromST(const Vector3& S, const Vector3& T) { return Frame(S.cross(T), S, T); }
        static Frame fromT(const Vector3& T) { Vector3 R = orthogonalVector(T.normalized()).normalized(); return fromTR(T,R); }

        static Vector3 orthogonalVector(const Vector3& n) {
            if ((abs(n.y()) >= 0.9 * abs(n.x())) && abs(n.z()) >= 0.9 * abs(n.x())) return Vector3(0.0, -n.z(), n.y());
            else if ( abs(n.x()) >= 0.9 * abs(n.y()) && abs(n.z()) >= 0.9 * abs(n.y()) ) return Vector3(-n.z(), 0.0, n.x());
            else return Vector3(-n.y(), n.x(), 0.0);
        }

        void normalize() { r.normalize(); s.normalize(); t.normalize(); }
    };

    static inline Vector3 pointOnPlane(Vector3 p, Vector3 plane_normal, Scalar plane_d = 0)
    {
        Scalar t = plane_normal.dot(p) - plane_d;
        return p - (t * plane_normal);
    }

    inline Frame frameAt(double t){
        return U[ (qRanged(0.0, t, 1.0) * (U.size() - 1)) ];
    }

    std::vector<Vector3> point;
    std::vector<Frame> U;
};
