#pragma once

#include "EnergyGuidedDeformation.h"
#include "EvaluateCorrespondence.h"

#include "stlastar.h"

namespace AStar
{
	class PathSearchNode : public Energy::SearchNode{
	public:
		PathSearchNode(const Energy::SearchNode & n = Energy::SearchNode()) : Energy::SearchNode(n){}

		static int k_top;

		/* Heuristic */
		double GoalDistanceEstimate( PathSearchNode & )
		{
			bool isHausdorff = false;
			bool isRMSD = false;

			// Hausdorff distance
			if (isHausdorff)
			{
				auto allDists = EvaluateCorrespondence::hausdroffDistance(shapeA.data(), shapeB.data());
				double sum = 0;
				for (auto partID : allDists.keys()){
					auto dists = allDists[partID];
					double curSum = 0;
					for (auto d : dists) curSum += d;
					sum += curSum / dists.size();
				}
				double avg = sum / allDists.size();
				return avg;
			}

			//  Root Mean Squared Deviation (RMSD)
			if (isRMSD)
			{
				double rmsd = EvaluateCorrespondence::RMSD(shapeA.data(), shapeB.data());
				return rmsd;
			}

			return 0.25 * double(unassigned.size()) / shapeA->nodes.size();
		}

		/* Actual cost (arc cost) */
		double GetCost( PathSearchNode &successor )
		{
			Energy::GuidedDeformation::applyAssignment(&successor, false);
			return successor.cost;
		}

		bool GetSuccessors( AStarSearch<PathSearchNode> *astarsearch )
		{
			auto suggestions = Energy::GuidedDeformation::suggestChildren(*this, k_top);

			for (auto suggestion : suggestions)
				astarsearch->AddSuccessor(PathSearchNode(suggestion));

			return suggestions.size();
		}

		bool IsGoal()
		{
			return unassigned.empty();
		}

		bool IsSameState( PathSearchNode &rhs )
		{
			if (this->mapping.empty()) return false;
			return this->mapping == rhs.mapping;
		}
	};

	static inline std::vector< std::vector<Energy::SearchNode> > search(Energy::SearchNode & root, 
		int num_solutions = 100, int k_top = 4, unsigned int * steps = nullptr)
	{
		// Prepare shapes
		Energy::GuidedDeformation::preprocess(root.shapeA.data(), root.shapeB.data());

		Energy::SearchNode start;
		start.shapeA = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*root.shapeA.data()));
		start.shapeB = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*root.shapeB.data()));
		start.assignments = root.assignments;

		if(root.unassigned.empty()) 
		{
			start.unassigned = root.unassignedList();
		}
		else
		{
			start.unassigned = root.unassigned;
		}

		Energy::GuidedDeformation::applyAssignment(&start, true);

		auto startCopy = QSharedPointer<Energy::SearchNode>(new Energy::SearchNode(start));

		PathSearchNode::k_top = k_top;

		PathSearchNode startNode(*startCopy);

		std::vector< std::vector<Energy::SearchNode> > result;

		/// Perform search:
		{
            //int max_open_set = 10000;

			AStarSearch<PathSearchNode> astarsearch(num_solutions);

			astarsearch.SetStartAndGoalStates(startNode, startNode);

			unsigned int SearchSteps = 0;
			if (!steps) steps = &SearchSteps;
			unsigned int SearchState;

			do
			{
                SearchState = astarsearch.SearchStep(num_solutions /*, max_open_set*/);
				(*steps)++;
			} while (SearchState == AStarSearch<PathSearchNode>::SEARCH_STATE_SEARCHING);

			// Convert to original type
			for (auto & row : astarsearch.solutions){
				std::vector<Energy::SearchNode> r;
				for (auto & element : row) r.push_back(element);
				result.push_back(r);
			}
		}

		return result;
	}
}
