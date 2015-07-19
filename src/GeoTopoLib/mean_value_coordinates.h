#pragma once
#include <vector>

// From: https://github.com/razh/coordinates
struct MeanValueCoordinates
{
	typedef std::vector<double> Weights;

	/**
	* modulo with Euclidean division.
	* Always the same sign as the divisor (d).
	* Useful for accessing arrays using negative indices.
	*
	* Example:
	*   modulo( -1, 3 ) => 2
	*/
	static inline int modulo(int n, int d) { return ((n % d) + d) % d; }

	/**
	* Converts (x, y) to a set of mean-value coordinate weights.
	*
	* Implementation of the interpolation function from K. Hormann and
	* M. S. Floater's Mean Value Coordinates for Arbitrary Planar Polygons.
	*/
	template<class Cage>
	static inline Weights computeWeights(double x, double y, const Cage & vertices) {
		int vertexCount = (int)vertices.size();

		// The edgeWeights array contains all zeros and is not used unless
		// (x, y) lies on a vertex or edge.
		Weights edgeWeights(vertexCount);
		Weights weights(vertexCount);

		// Assign all zeros.
		for (int i = 0; i < vertexCount; i++) {
			edgeWeights[i] = 0;
			weights[i] = 0;
		}

		/**
		* Determine if (x, y) lies on a vertex or an edge.
		* Otherwise, determine mean-value weights for (x, y).
		*
		* Subscripts:
		*   0 - Previous vertex.
		*   1 - Current vertex.
		*   2 - Next vertex.
		*/
		double x0, y0, x1, y1, x2, y2;
		double dx0, dy0, dx1, dy1, dx2, dy2;
		double r0, r1, r2;
		/**
		* Areas and dot products correspond to triangles:
		*
		* Subscripts:
		*  0 - Triangle formed by (x, y), (x0, y0), and (x1, y1).
		*  1 - Triangle formed by (x, y), (x1, y1), and (x2, y2).
		*/
		double area0, area1;
		double dot0, dot1;
		double sum = 0;
		for (int i = 0; i < vertexCount; i++) {
			// Current vertex.
			x1 = vertices[i][0];
			y1 = vertices[i][1];
			// Next vertex.
			x2 = vertices[(i + 1) % vertexCount][0];
			y2 = vertices[(i + 1) % vertexCount][1];

			dx1 = x1 - x;
			dy1 = y1 - y;

			dx2 = x2 - x;
			dy2 = y2 - y;

			// Radii from (x, y).
			r1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
			// (x, y) lies on (x1, y1).
			if (!r1) {
				edgeWeights[i] = 1;
				return edgeWeights;
			}

			area1 = 0.5 * (dx1 * dy2 - dx2 * dy1);
			dot1 = dx1 * dx2 + dy1 * dy2;

			// (x, y) lies on the edge (x1, y1) - (x2, y2).
			if (!area1 && dot1 < 0) {
				r2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
				edgeWeights[i] = r2 / (r1 + r2);
				edgeWeights[(i + 1) % vertexCount] = 1 - edgeWeights[i];
				return edgeWeights;
			}

			// Previous vertex.
			x0 = vertices[modulo(i - 1, vertexCount)][0];
			y0 = vertices[modulo(i - 1, vertexCount)][1];

			dx0 = x0 - x;
			dy0 = y0 - y;

			area0 = 0.5 * (dx0 * dy1 - dx1 * dy0);
			// Add contribution of first triangle.
			if (area0) {
				r0 = std::sqrt(dx0 * dx0 + dy0 * dy0);
				dot0 = dx0 * dx1 + dy0 * dy1;
				weights[i] += (r0 - dot0 / r1) / area0;
			}

			// And contribution of second triangle.
			if (area1) {
				r2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
				weights[i] += (r2 - dot1 / r1) / area1;
			}

			sum += weights[i];
		}

		// Normalize weights.
		double sumInverse = 1.0 / sum;
		for (int i = 0; i < vertexCount; i++) {
			weights[i] *= sumInverse;
		}

		return weights;
	}

	/**
	* Interpolates mean-value coordinate weights along vertices.
	*/
	template<class Cage>
	static inline std::pair<double,double> interpolate2d(const Weights & weights, const Cage & vertices) {
		int vertexCount = (int)vertices.size();
		double x = 0, y = 0;
		for (int i = 0; i < vertexCount; i++) {
			x += weights[i] * vertices[i][0];
			y += weights[i] * vertices[i][1];
		}
		return std::make_pair(x, y);
	}
};
