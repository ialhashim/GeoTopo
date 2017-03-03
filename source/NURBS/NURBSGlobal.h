#pragma once
#include <float.h>
#include <set>
#include <map>
#include <iostream>

#include <Eigen/Geometry>

#include "weld.h"

#define MAX_REAL std::numeric_limits<Real>::max()
#define REAL_ZERO_TOLERANCE 1e-6

#ifdef WIN32
namespace std{  static inline bool isnan(double x){ return _isnan(x); } }
namespace std{  static inline bool isfinite(double x){ return _finite(x); } }
#else
#include <cmath>
#endif

namespace NURBS{

typedef double Real;
typedef Eigen::Vector3d Vector3;
typedef Eigen::Vector4d Vector4;

#define Vector3_ZERO Vector3(0,0,0)

#define qRanged(_min, v, _max) ( (std::max(_min, (std::min(v, _max)))) )

// For visualization
struct SurfaceQuad{	Vector3 p[4]; Vector3 n[4]; };

inline double ClosestPointTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c, Vector3 & closest)
{
    // Check if P in vertex region outside A
    Vector3 ab = b - a;
    Vector3 ac = c - a;
    Vector3 ap = p - a;
    double d1 = ab.dot(ap);
    double d2 = ac.dot(ap);
    if (d1 <= 0 && d2 <= 0)
    {
        closest = a;
        return (p-closest).squaredNorm(); // barycentric coordinates (1,0,0)
    }
    // Check if P in vertex region outside B
    Vector3 bp = p - b;
    double d3 = ab.dot(bp);
    double d4 = ac.dot(bp);
    if (d3 >= 0 && d4 <= d3)
    {
        closest = b;
        return (p-closest).squaredNorm(); // barycentric coordinates (0,1,0)
    }
    // Check if P in edge region of AB, if so return projection of P onto AB
    double vc = d1*d4 - d3*d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        double v = d1 / (d1 - d3);
        closest = a + v * ab;
        return (p - closest).squaredNorm(); // barycentric coordinates (1-v,v,0)
    }
    // Check if P in vertex region outside C
    Vector3 cp = p - c;
    double d5 = ab.dot(cp);
    double d6 = ac.dot(cp);
    if (d6 >= 0 && d5 <= d6)
    {
        closest = c;
        return (p-closest).squaredNorm(); // barycentric coordinates (0,0,1)
    }
    // Check if P in edge region of AC, if so return projection of P onto AC
    double vb = d5*d2 - d1*d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        double w = d2 / (d2 - d6);
        closest = a + w * ac;
        return (p - closest).squaredNorm(); // barycentric coordinates (1-w,0,w)
    }
    // Check if P in edge region of BC, if so return projection of P onto BC
    double va = d3*d6 - d5*d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        closest = b + w * (c - b);
        return (p - closest).squaredNorm(); // barycentric coordinates (0,1-w,w)
    }
    // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
    double denom = 1.0 / (va + vb + vc);
    double v = vb * denom;
    double w = vc * denom;
    closest = a + ab * v + ac * w;
    return (p - closest).squaredNorm(); // = u*a + v*b + w*c, u = va * denom = 1 - v - w
}

inline double ClosestPointSegment(Vector3 p, Vector3 a, Vector3 b, double &t, Vector3 &d)
{
    Vector3 ab = b - a;
    // Project c onto ab, but deferring divide by Dot(ab, ab)
    t = Vector3(p - a).dot(ab);
    if (t <= 0.0) {
        // c projects outside the [a,b] interval, on the a side; clamp to a
        t = 0.0;
        d = a;
    } else {
        double denom = ab.dot(ab); // Always nonnegative since denom = ||ab||
        if (t >= denom) {
            // c projects outside the [a,b] interval, on the b side; clamp to b
            t = 1.0;
            d = b;
        } else {
            // c projects inside the [a,b] interval; must do deferred divide now
            t = t / denom;
            d = a + t * ab;
        }
    }
    return (p - d).squaredNorm();
}

// Clamp n to lie within the range [min, max]
inline double Clamp(double n, double min, double max) {
    if (n < min) return min;
    if (n > max) return max;
    return n;
}

// Computes closest points C1 and C2 of S1(s)=P1+s*(Q1-P1) and
// S2(t)=P2+t*(Q2-P2), returning s and t. Function result is squared
// distance between between S1(s) and S2(t)

#define SEGMENTS_EPSILON 1e-6

inline double ClosestPointSegments(Vector3 p1, Vector3 q1, Vector3 p2, Vector3 q2, double &s, double &t, Vector3 &c1, Vector3 &c2)
{
    #define dots(a,b) ((a).dot(b))

    Vector3 d1 = q1 - p1; // Direction vector of segment S1
    Vector3 d2 = q2 - p2; // Direction vector of segment S2
    Vector3 r = p1 - p2;
    double a = dots(d1, d1); // Squared length of segment S1, always nonnegative
    double e = dots(d2, d2); // Squared length of segment S2, always nonnegative
    double f = dots(d2, r);
    // Check if either or both segments degenerate into points
    if (a <= SEGMENTS_EPSILON && e <= SEGMENTS_EPSILON) {
        // Both segments degenerate into points
        s = t = 0.0;
        c1 = p1;
        c2 = p2;
        return dots(c1 - c2, c1 - c2);
    }
    if (a <= SEGMENTS_EPSILON) {
        // First segment degenerates into a point
        s = 0.0;
        t = f / e; // s = 0 => t = (b*s + f) / e = f / e
        t = Clamp(t, 0.0, 1.0);
    } else {
        double c = dots(d1, r);
        if (e <= SEGMENTS_EPSILON) {
            // Second segment degenerates into a point
            t = 0.0;
            s = Clamp(-c / a, 0.0, 1.0); // t = 0 => s = (b*t - c) / a = -c / a
        } else {
            // The general non-degenerate case starts here
            double b = dots(d1, d2);
            double denom = a*e-b*b; // Always nonnegative
            // If segments not parallel, compute closest point on L1 to L2 and
            // clamp to segment S1. Else pick arbitrary s (here 0)
            if (denom != 0.0) {
                s = Clamp((b*f - c*e) / denom, 0.0, 1.0);
            } else s = 0.0;
            // Compute point on L2 closest to S1(s) using
            // t = Dot((P1 + D1*s) - P2,D2) / Dot(D2,D2) = (b*s + f) / e
            t = (b*s + f) / e;
            // If t in [0,1] done. Else clamp t, recompute s for the new value
            // of t using s = Dot((P2 + D2*t) - P1,D1) / Dot(D1,D1)= (t*b - c) / a
            // and clamp s to [0, 1]
            if (t < 0.0) {
                t = 0.0;
                s = Clamp(-c / a, 0.0, 1.0);
            } else if (t > 1.0) {
                t = 1.0;
                s = Clamp((b - c) / a, 0.0, 1.0);
            }
        }
    }
    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
    return dots(c1 - c2, c1 - c2);
}

inline double ClosestSegmentTriangle(Vector3 p, Vector3 q, Vector3 a, Vector3 b, Vector3 c, Vector3 &projSegment, Vector3 &projTriangle)
{
    // Variables
    int numCases = 5;
    std::vector<double> dists(numCases, 0.0);
    std::vector<double> s(numCases, 0.0), t(numCases, 0.0);
    std::vector<Vector3> c1 (numCases, Vector3(0,0,0)), c2 (numCases, Vector3(0,0,0));

    // segment PQ and triangle edge AB,
    dists[0] = ClosestPointSegments(p,q, a,b, s[0], t[0], c1[0], c2[0]);

    // segment PQ and triangle edge BC,
    dists[1] = ClosestPointSegments(p,q, b,c, s[1], t[1], c1[1], c2[1]);

    // segment PQ and triangle edge CA,
    dists[2] = ClosestPointSegments(p,q, c,a, s[2], t[2], c1[2], c2[2]);

    // segment endpoint P and plane of triangle (when P projects inside ABC), and
    dists[3] = ClosestPointTriangle(p, a,b,c, c1[3]); c2[3] = p;

    // segment endpoint Q and plane of triangle (when Q projects inside ABC)
    dists[4] = ClosestPointTriangle(p, a,b,c, c1[4]); c2[3] = q;

    // Find minimum distance
    double minDist = DBL_MAX;

    for(int i = 0; i < numCases; i++)
    {
        double dist = (c1[i] - c2[i]).squaredNorm();

        if(dist < minDist)
        {
            minDist = dist;
            projSegment = c1[i];
            projTriangle = c2[i];
        }
    }

    return (projSegment - projTriangle).norm();
}

#include "TriTriIntersect.h"

// Intersection line between triangles 'abc' and 'ijk', line is 'pq'
inline Vector3 TriTriIntersect(	Vector3 a, Vector3 b, Vector3 c,
                                Vector3 i, Vector3 j, Vector3 k,
                                Vector3 & p, Vector3 & q)
{
    float V0[3] = {(float)a[0], (float)a[1], (float)a[2]};
    float V1[3] = {(float)b[0], (float)b[1], (float)b[2]};
    float V2[3] = {(float)c[0], (float)c[1], (float)c[2]};

    float U0[3] = {(float)i[0], (float)i[1], (float)i[2]};
    float U1[3] = {(float)j[0], (float)j[1], (float)j[2]};
    float U2[3] = {(float)k[0], (float)k[1], (float)k[2]};

    float P[3] = {0,0,0};
    float Q[3] = {0,0,0};

    int isCoplanar = 0;

    tri_tri_intersect_with_isectline(V0, V1, V2,
                                     U0, U1, U2,
                                     &isCoplanar,
                                     P,Q);
    p = Vector3((float)P[0], (float)P[1], (float)P[2]);
    q = Vector3((float)Q[0], (float)Q[1], (float)Q[2]);

    if( isCoplanar ) p = q = (a+b+c+i+j+k) / 6.0;

    return (p + q) / 2.0;
}

inline static bool sphereTest(Vector3 & p1, Vector3 & p2, double r1, double r2)
{
    double minDist = r1 + r2;
    Vector3 relPos = p1 - p2;
    double dist = relPos.x() * relPos.x() + relPos.y() * relPos.y() + relPos.z() * relPos.z();
    return dist <= minDist * minDist;
}

inline static bool intersectRayTri(const std::vector<Vector3> & tri, const Vector3 & rayOrigin,
    const Vector3 & rayDirection, Vector3 & intersectionPoint)
{
    double u, v, t;
    Vector3 edge1 = tri[1] - tri[0];
    Vector3 edge2 = tri[2] - tri[0];
    Vector3 pvec = rayDirection.cross(edge2);
    double det = edge1.dot(pvec);
    if (det == 0) return false;
    double invDet = 1 / det;
    Vector3 tvec = rayOrigin - tri[0];
    u = tvec.dot(pvec) * invDet;
    if (u < 0 || u > 1) return false;
    Vector3 qvec = tvec.cross(edge1);
    v = rayDirection.dot(qvec) * invDet;
    if (v < 0 || u + v > 1) return false;
    t = edge2.dot(qvec) * invDet;
    intersectionPoint = rayOrigin + (t * rayDirection);
    return true;
}

inline static bool TestSphereTriangle(Vector3 sphereCenter, double sphereRadius, Vector3 a, Vector3 b, Vector3 c, Vector3 &p)
{
	// Find point P on triangle ABC closest to sphere center
	ClosestPointTriangle(sphereCenter, a, b, c, p);

	// Sphere and triangle intersect if the (squared) distance from sphere
	// center to point p is less than the (squared) sphere radius
    Vector3 v = p - sphereCenter;
    return v.dot(v) <= sphereRadius * sphereRadius;
}

/// Utility functions
inline static std::vector< std::vector<Vector3> > inverseVectors3(std::vector< std::vector<Vector3> > vectors)
{
    std::vector< std::vector<Vector3> > result = vectors;

    for(int i = 0; i < (int)result.size(); i++)
        for(int j = 0; j < (int)result.front().size(); j++)
            result[i][j] *= -1;

    return result;
}

inline static std::vector<Vector3> inverseVectors3(std::vector<Vector3> vectors)
{
    std::vector<Vector3> result = vectors;

    for(int i = 0; i < (int)result.size(); i++)
            result[i] *= -1;

    return result;
}

// Memory helpers
template<typename T>
static inline std::vector<T> new1(int y, const T& defaultValue = T()){
    return std::vector<T>( y, defaultValue );
}

template<typename T>
static inline std::vector< std::vector<T> > new2(int y, int x, const T& defaultValue = T()){
    return std::vector< std::vector<T> >( y, std::vector<T>(x, defaultValue) );
}

struct NURBSException : public std::exception
{
   std::string s;
   NURBSException(std::string ss) : s(ss) {}
   ~NURBSException() throw () {} // Updated
   const char* what() const throw() { return s.c_str(); }
};

}

typedef std::vector< NURBS::Vector3 > Array1D_Vector3;
typedef std::vector< Array1D_Vector3 > Array2D_Vector3;
typedef std::vector< NURBS::Real > Array1D_Real;
typedef std::vector< Array1D_Real > Array2D_Real;
typedef std::vector< NURBS::Vector4, Eigen::aligned_allocator<NURBS::Vector4> > Array1D_Vector4;
typedef std::vector< Array1D_Vector4 > Array2D_Vector4;

/// Quick Assertion
#define assertion(condition, message) do { } while (false)

