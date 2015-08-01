#include "NanoKdTree.h"

namespace hausdroff
{
	/* Compute Hausdorff distance between point clouds */
	template<typename PointsContainer>
	inline double distance(const PointsContainer & a, const PointsContainer & b)
	{
		// compare point cloud A to point cloud B
		auto max_dist = [](const PointsContainer & cloud_a, const PointsContainer & cloud_b){
			NanoKdTree tree_b;
			for(auto p : cloud_b) tree_b.addPoint(p);
			tree_b.build();
			
			double max_dist_a = -std::numeric_limits<double>::max ();
			for (size_t i = 0; i < cloud_a.size (); ++i)
			{
				KDResults results;
				tree_b.k_closest(cloud_a[i], 1, results);
				auto sqr_distance = results.front().second;

				if (sqr_distance > max_dist_a)
					max_dist_a = sqr_distance;
			}
			return max_dist_a;
		};

		double max_dist_a = std::sqrt (max_dist(a,b));
		double max_dist_b = std::sqrt(max_dist(b, a));

		double dist = std::max (max_dist_a, max_dist_b);
		return dist;
	}

	/* More efficient given pre-computed kd-trees */
	inline double distance(NanoKdTree * tree_a, NanoKdTree * tree_b)
	{
		auto max_dist = [](NanoKdTree * t_a, NanoKdTree * t_b){
			double max_dist_a = -std::numeric_limits<double>::max();
			for (size_t i = 0; i < t_a->cloud.pts.size(); ++i){
				KDResults results;
				t_b->k_closest(t_a->cloud.pts[i], 1, results);
				auto sqr_distance = results.front().second;
				if (sqr_distance > max_dist_a)
					max_dist_a = sqr_distance;
			}
			return max_dist_a;
		};
		double max_dist_a = std::sqrt(max_dist(tree_a, tree_b));
		double max_dist_b = std::sqrt(max_dist(tree_b, tree_a));
		double dist = std::max(max_dist_a, max_dist_b);
		return dist;
	}
}
