#pragma once

// From: http://www.algorithmist.com/index.php/Monotone_Chain_Convex_Hull.cpp
namespace chull2d
{
	typedef double coord_t;         // coordinate type
	typedef double coord2_t;  // must be big enough to hold 2*max(|coordinate|)^2
	struct Point {
		coord_t x, y;
		bool operator <(const Point &p) const {
			return x < p.x || (x == p.x && y < p.y);
		}
	};

	// 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
	// Returns a positive value, if OAB makes a counter-clockwise turn,
	// negative for clockwise turn, and zero if the points are collinear.
	coord2_t cross(const Point &O, const Point &A, const Point &B)
	{
		return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
	}

	template <typename T>
	std::vector<size_t> ordered(std::vector<T> const& values) {
		std::vector<size_t> indices(values.size());
		std::iota(begin(indices), end(indices), static_cast<size_t>(0));
		std::sort( begin(indices), end(indices), [&](size_t a, size_t b) { return values[a] < values[b]; }
		);
		return indices;
	}

	// Returns a list of points on the convex hull in counter-clockwise order.
	// Note: the last point in the returned list is the same as the first one.
	std::vector<Point> convex_hull(std::vector<Point> P, std::vector<int>* indices = NULL)
	{
		int n = P.size(), k = 0;
		std::vector<Point> H(2 * n);
		indices->resize(2 * n);

		// Sort points lexicographically
		auto pmap = ordered(P);
		std::sort(P.begin(), P.end());

		// Build lower hull
		for (int i = 0; i < n; ++i) {
			while (k >= 2 && cross(H[k - 2], H[k - 1], P[i]) <= 0) k--;
			indices->at(k) = pmap[i];
			H[k++] = P[i];
		}

		// Build upper hull
		for (int i = n - 2, t = k + 1; i >= 0; i--) {
			while (k >= t && cross(H[k - 2], H[k - 1], P[i]) <= 0) k--;
			indices->at(k) = pmap[i];
			H[k++] = P[i];
		}

		H.resize(k - 1);
		indices->resize(k - 1);
		return H;
	}

	template<class Point2D>
	std::vector<int> convex_hull_2d(const std::vector<Point2D> & pts)
	{
		std::vector<Point> points;
		for (auto p : pts)
		{
			Point v; 
			v.x = p[0]; 
			v.y = p[1];
			points.push_back(v);
		}

		std::vector<int> result;
		auto ch = convex_hull(points, &result);
		return result;
	}
}
