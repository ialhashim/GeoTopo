#pragma once

#include <vector>

/// A Union-Find/Disjoint-Set data structure.
class DisjointSet {
public:
	/// The number of elements in the universe.
	int Count;

	/// The parent of each element in the universe.
	std::vector<int> Parent;

	/// The rank of each element in the universe.
	std::vector<int> Rank;

	/// The size of each set.
	std::vector<int> SizeOfSet;

	/// The number of disjoint sets.
	int SetCount;

	/// Initializes a new Disjoint-Set data structure, with the specified amount of elements in the universe.
	/// The number of elements in the universe.
	DisjointSet(int count) {
		Count = count;
		SetCount = count;
		Parent.resize(Count, 0);
		Rank.resize(Count, 0);
		SizeOfSet.resize(Count, 0);

		for (int i = 0; i < Count; i++) {
			Parent[i] = i;
			Rank[i] = 0;
			SizeOfSet[i] = 1;
		}
	}

	/// Find the parent of the specified element.
	int Find(int i) {
		if (Parent[i] == i) {
			return i;
		} else {
			// Recursively find the real parent of i, and then cache it for later lookups.
			Parent[i] = Find(Parent[i]);
			return Parent[i];
		}
	}

	/// Unite the sets that the specified elements belong to.
	void Union(int i, int j) {

		// Find the representatives (or the root nodes) for the set that includes i
		int irep = Find(i),
			// And do the same for the set that includes j
			jrep = Find(j),
			// Get the rank of i's tree
			irank = Rank[irep],
			// Get the rank of j's tree
			jrank = Rank[jrep];

		// Elements are in the same set, no need to unite anything.
		if (irep == jrep)
			return;

		SetCount--;

		// If i's rank is less than j's rank
		if (irank < jrank) {

			// Then move i under j
			Parent[irep] = jrep;
			SizeOfSet[jrep] += SizeOfSet[irep];

		} // Else if j's rank is less than i's rank
		else if (jrank < irank) {

			// Then move j under i
			Parent[jrep] = irep;
			SizeOfSet[irep] += SizeOfSet[jrep];

		} // Else if their ranks are the same
		else {

			// Then move i under j (doesn't matter which one goes where)
			Parent[irep] = jrep;
			SizeOfSet[jrep] += SizeOfSet[irep];

			// And increment the the result tree's rank by 1
			Rank[jrep]++;
		}
	}

	std::vector< std::vector<size_t> > Groups()
	{
		std::map< size_t, std::vector<size_t> > groups;
		for(size_t i = 0; i < Parent.size(); i++)
			groups[ Parent[i] ].push_back(i);

		std::vector< std::vector<size_t> > groupsVector;
		for(auto g : groups) groupsVector.push_back(g.second);
		return groupsVector;
	}
};
