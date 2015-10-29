#pragma once
#undef SearchPath

#include "ShapeGraph.h"
#include "RenderObjectExt.h"

#include <QStack>
#include "tree.hh"

namespace Energy
{
	typedef QVector< QPair<QStringList, QStringList> > Assignments;
	static QString null_part = "NULL_PART";
	typedef QSet<QString> QSetString;

	struct SearchNode
	{
		double cost, energy;
		QSetString fixed, current, unassigned;
		Assignments assignments;
		QMap<QString, QString> mapping;
		QMap<QString, double> mappingCost;
		QSharedPointer<Structure::ShapeGraph> shapeA, shapeB;
		QMap<QString, QVariant> property;
		int num_children;

		SearchNode(QSharedPointer<Structure::ShapeGraph> shapeA = QSharedPointer<Structure::ShapeGraph>(), 
			QSharedPointer<Structure::ShapeGraph> shapeB = QSharedPointer<Structure::ShapeGraph>(),
			const QSetString & fixed = QSetString(), const Assignments & assignments = Assignments(),
			const QSetString & unassigned = QSetString(), const QMap<QString, QString> & mapping = QMap<QString, QString>(),
			double cost = std::numeric_limits<double>::max(), double energy = 0.0)
			: shapeA(shapeA), shapeB(shapeB), fixed(fixed), assignments(assignments), unassigned(unassigned), mapping(mapping), 
			cost(cost), energy(energy), num_children(0){}

		QSetString fixedOnTarget(){ QSetString result; for (auto a : assignments) for (auto p : a.second) result << p; return result; }
		QSetString unassignedList(){
			QSetString result;
			for (auto n : shapeA->nodes){
				bool isUnassigned = true;
				if (fixed.contains(n->id)){ isUnassigned = false; }
				else { for (auto a : assignments) if (a.first.contains(n->id)){ isUnassigned = false; break; } }
				if(isUnassigned) result << n->id;
			}
			return result; 
		}

		bool operator<(const SearchNode & path) const { return energy < path.energy; }
	};

	typedef tree<SearchNode> SearchTree;

	struct GuidedDeformation
	{
		QSharedPointer<Structure::ShapeGraph> origShapeA, origShapeB;
		QVector< SearchTree > searchTrees;
		PropertyMap property;

		static void preprocess(Structure::ShapeGraph * shapeA, Structure::ShapeGraph * shapeB);
        void searchAll(Structure::ShapeGraph * shapeA, Structure::ShapeGraph * shapeB, QVector<Energy::SearchNode> & roots, bool isSaveKeyframes, int k_top = 4);

		static void topologicalOpeartions(Structure::ShapeGraph *shapeA, Structure::ShapeGraph *shapeB, QStringList & la, QStringList & lb);
		static void applyDeformation(Structure::ShapeGraph *shapeA, Structure::ShapeGraph *shapeB, const QStringList & la, const QStringList & lb, const QSetString & fixed, bool isSaveKeyframes = false);
		static void postDeformation(Structure::ShapeGraph * shape, const QSet<QString> & fixed);

		static void applyAssignment(Energy::SearchNode * path, bool isSaveKeyframes);
		static QVector<Energy::SearchNode> suggestChildren(Energy::SearchNode & path, int k_top);
		QVector<Energy::SearchNode*> solutions();
		QVector<Energy::SearchNode*> parents();
		QVector<Energy::SearchNode*> childrenOf(Energy::SearchNode * path);
		QVector<Energy::SearchNode*> getEntirePath(Energy::SearchNode * path);

		void applySearchPath(QVector<Energy::SearchNode*> path);

		//////////////////////////////////////////////////////////////////////////

		GuidedDeformation() :isInitTest(false), isApplySYMH(false), K(20), K_2(1){}

		//SYMH
		bool isInitTest, isApplySYMH;
		QMap<QString, int> nidMapA, nidMapB;
		std::vector<int> symhA, symhB;
		int symhGroupSizeA, symhGroupSizeB;
		std::vector<int> symhSubgraphSizeA, symhSubgraphSizeB;

		//Dynamic programming
		int K, K_2;

		void symhPruning(Energy::SearchNode & path, QVector < QPair<Structure::Relation, Structure::Relation> > & pairings, std::vector<int>& pairAward);
		void propagateDP(Energy::SearchNode & path, Structure::Relation& frontParts, std::vector<Structure::Relation>& mirrors, std::vector<double>& costs, std::vector<Energy::SearchNode>& res);
		void searchDP(Structure::ShapeGraph * shapeA, Structure::ShapeGraph * shapeB, QVector<Energy::SearchNode> & roots);

		Energy::SearchNode partialSelectionGreedy(const Energy::SearchNode &initpath, const Energy::SearchNode &path, int bestK = 0);
		//////////////////////////////////////////////////////////////////////////

	};
}
