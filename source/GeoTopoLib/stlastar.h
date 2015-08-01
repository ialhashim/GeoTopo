/*
A* Algorithm Implementation using STL is
Copyright (C)2001-2005 Justin Heyes-Jones

Permission is given by the author to freely redistribute and 
include this code in any program as long as this credit is 
given where due.
 
  COVERED CODE IS PROVIDED UNDER THIS LICENSE ON AN "AS IS" BASIS, 
  WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, 
  INCLUDING, WITHOUT LIMITATION, WARRANTIES THAT THE COVERED CODE 
  IS FREE OF DEFECTS, MERCHANTABLE, FIT FOR A PARTICULAR PURPOSE
  OR NON-INFRINGING. THE ENTIRE RISK AS TO THE QUALITY AND 
  PERFORMANCE OF THE COVERED CODE IS WITH YOU. SHOULD ANY COVERED 
  CODE PROVE DEFECTIVE IN ANY RESPECT, YOU (NOT THE INITIAL 
  DEVELOPER OR ANY OTHER CONTRIBUTOR) ASSUME THE COST OF ANY 
  NECESSARY SERVICING, REPAIR OR CORRECTION. THIS DISCLAIMER OF 
  WARRANTY CONSTITUTES AN ESSENTIAL PART OF THIS LICENSE. NO USE 
  OF ANY COVERED CODE IS AUTHORIZED HEREUNDER EXCEPT UNDER
  THIS DISCLAIMER.
 
  Use at your own risk!

*/

#ifndef STLASTAR_H
#define STLASTAR_H
// used for text debugging
#include <iostream>
#include <stdio.h>
//#include <conio.h>
#include <assert.h>

// stl includes
#include <algorithm>
#include <set>
#include <vector>
#include <cfloat>

using namespace std;

// fast fixed size memory allocator, used for fast node memory management
//#include "fsa.h"

// Fixed size memory allocator can be disabled to compare performance
// Uses std new and delete instead if you turn it off
#define USE_FSA_MEMORY 0

// disable warning that debugging information has lines that are truncated
// occurs in stl headers
#if defined(WIN32) && defined(_WINDOWS)
#pragma warning( disable : 4786 )
#endif

// The AStar search class. UserState is the users state space type
template <class UserState> class AStarSearch
{

public: // data

	enum
	{
		SEARCH_STATE_NOT_INITIALISED,
		SEARCH_STATE_SEARCHING,
		SEARCH_STATE_SUCCEEDED,
		SEARCH_STATE_FAILED,
		SEARCH_STATE_OUT_OF_MEMORY,
		SEARCH_STATE_INVALID
	};


	// A node represents a possible state in the search
	// The user provided state type is included inside this type

	public:

	class Node
	{
		public:

			std::shared_ptr<Node> parent; // used during the search to record the parent of successor nodes
			std::shared_ptr<Node> child; // used after the search for the application to view the search in reverse
			
			float g; // cost of this node + it's predecessors
			float h; // heuristic estimate of distance to goal
			float f; // sum of cumulative cost of predecessors and self and heuristic

			Node() :
				parent( 0 ),
				child( 0 ),
				g( 0.0f ),
				h( 0.0f ),
				f( 0.0f )
			{			
			}

			UserState m_UserState;
	};


	// For sorting the heap the STL needs compare function that lets us compare
	// the f value of two nodes
	class HeapCompare_f 
	{
		public:

			bool operator() (const std::shared_ptr<Node>& x, const std::shared_ptr<Node>& y) const
			{
				return x->f > y->f;
			}
	};


public: // methods


	// constructor just initializes private data
	AStarSearch() :
                m_State( SEARCH_STATE_NOT_INITIALISED )
#if USE_FSA_MEMORY
                ,m_FixedSizeAllocator( 1000 )
#endif
                //,m_AllocateNodeCount(0)
	{
	}

	AStarSearch( int ) :
		m_State( SEARCH_STATE_NOT_INITIALISED )
#if USE_FSA_MEMORY
		m_FixedSizeAllocator( MaxNodes ),
#endif
	{
	}

	// Set Start and goal states
	void SetStartAndGoalStates( UserState &Start, UserState &Goal )
	{
		m_Start = AllocateNode();
		m_Goal = AllocateNode();

		assert((m_Start != NULL && m_Goal != NULL));
		
		m_Start->m_UserState = Start;
		m_Goal->m_UserState = Goal;

		m_State = SEARCH_STATE_SEARCHING;
		
		// Initialize the AStar specific parts of the Start Node
		// The user only needs fill out the state information

		m_Start->g = 0; 
		m_Start->h = m_Start->m_UserState.GoalDistanceEstimate( m_Goal->m_UserState );
		m_Start->f = m_Start->g + m_Start->h;
		m_Start->parent = 0;

		// Push the start node on the Open list

		m_OpenList.push_back( m_Start ); // heap now unsorted

		// Sort back element into heap
		push_heap( m_OpenList.begin(), m_OpenList.end(), HeapCompare_f() );

		// Initialize counter for search steps
		m_Steps = 0;
	}

	// Advances search one step 
        unsigned int SearchStep(int num_solutions/*, int max_open_set*/)
	{
		// Firstly break if the user has not initialized the search
		assert( (m_State > SEARCH_STATE_NOT_INITIALISED) && (m_State < SEARCH_STATE_INVALID) );

		// Next I want it to be safe to do a search step once the search has succeeded...
		if((m_State == SEARCH_STATE_SUCCEEDED) || (m_State == SEARCH_STATE_FAILED))	return m_State; 

		// Failure is defined as emptying the open list as there is nothing left to search...
		if( m_OpenList.empty() )
		{
			m_State = SEARCH_STATE_FAILED;
			return m_State;
		}
		
		// Increment step count
		m_Steps++;

		// Pop the best node (the one with the lowest f) 
		auto n = m_OpenList.front(); // get pointer to the node
		pop_heap( m_OpenList.begin(), m_OpenList.end(), HeapCompare_f() );
		m_OpenList.pop_back();

		// Check for the goal, once we pop that we're done
		if( n->m_UserState.IsGoal() )
		{
			if (solutions.size() >= num_solutions)
			{
				m_State = SEARCH_STATE_SUCCEEDED;

				return m_State;
			}
			else
			{
				m_ClosedList.push_back(n);

				std::vector<UserState> r;

				// back trace
				while (n)
				{
					auto data = n->m_UserState;
					r.push_back(data);
					n = n->parent;
				}

				std::reverse( r.begin(), r.end() );

				// Only record the minimal unique solutions
				bool isFoundSimilar = false;
				UserState curLastState = r.back();

				for (size_t si = 0; si < solutions.size(); si++)
				{
					auto & solution = solutions[si];

					UserState prevLastState = solution.back();

					if (prevLastState.IsSameState(curLastState))
					{
						double prevCost = prevLastState.cost;
						double curCost = curLastState.cost;

						if (curCost < prevCost)
							solutions[si] = r;

						isFoundSimilar = true;
						break;
					}
				}

				if (!isFoundSimilar) solutions.push_back( r );
			}
		}
		else // not goal
		{
			m_Successors.clear(); // empty vector of successor nodes to n

			n->m_UserState.GetSuccessors( this ); 

			// Now handle each successor to the current node ...
			for (typename vector< std::shared_ptr<Node> >::iterator successoritr = m_Successors.begin(); successoritr != m_Successors.end(); successoritr++)
			{
				auto successor = *successoritr;

				// 	The g value for this successor ...
				float newg = n->g + n->m_UserState.GetCost( successor->m_UserState );

				// Now we need to find whether the node is on the open or closed lists
				// If it is but the node that is already on them is better (lower g)
				// then we can forget about this successor

				// First linear search of open list to find node
				typename vector< std::shared_ptr<Node> >::iterator openlist_result;
				typename vector< std::shared_ptr<Node> >::iterator closedlist_result;

				for( openlist_result = m_OpenList.begin(); openlist_result != m_OpenList.end(); openlist_result ++ )
					if( (*openlist_result)->m_UserState.IsSameState( successor->m_UserState ) )
						break;

				if( openlist_result != m_OpenList.end() ){
					// we found this state on open
					if( (*openlist_result)->g <= newg )
					{
						// the one on Open is cheaper than this one
						continue;
					}
				}

				for( closedlist_result = m_ClosedList.begin(); closedlist_result != m_ClosedList.end(); closedlist_result ++ )
					if( (*closedlist_result)->m_UserState.IsSameState( successor->m_UserState ) )
						break;					

				if( closedlist_result != m_ClosedList.end() ){
					// we found this state on closed
					if( (*closedlist_result)->g <= newg )
					{
						// the one on Closed is cheaper than this one
						continue;
					}
				}

				// This node is the best node so far with this particular state
				// so lets keep it and set up its AStar specific data ...
				successor->parent = n;
				successor->g = newg;
				successor->h = successor->m_UserState.GoalDistanceEstimate( m_Goal->m_UserState );
				successor->f = successor->g + successor->h;

				// Remove successor from closed if it was on it
				if( closedlist_result != m_ClosedList.end() )
				{
					// remove it from Closed
					//FreeNode(  (*closedlist_result) ); 
					m_ClosedList.erase( closedlist_result );

					// Fix thanks to ...
					// Greg Douglas <gregdouglasmail@gmail.com>
					// who noticed that this code path was incorrect
					// Here we have found a new state which is already CLOSED
				}

				// Update old version of this node
				if( openlist_result != m_OpenList.end() )
				{	   
					//FreeNode( (*openlist_result) ); 
			   		m_OpenList.erase( openlist_result );

					// re-make the heap 
					// make_heap rather than sort_heap is an essential bug fix
					// thanks to Mike Ryynanen for pointing this out and then explaining
					// it in detail. sort_heap called on an invalid heap does not work
					make_heap( m_OpenList.begin(), m_OpenList.end(), HeapCompare_f() );
				}

				// heap now unsorted
				m_OpenList.push_back( successor );

				// sort back element into heap
				push_heap( m_OpenList.begin(), m_OpenList.end(), HeapCompare_f() );

			} // end successor

			// push n onto Closed, as we have expanded it now
			m_ClosedList.push_back(n);

		} // end else (not goal so expand)
 		return m_State; // Succeeded bool is false at this point. 
	}

	// User calls this to add a successor to a list of successors
	// when expanding the search frontier
	bool AddSuccessor( UserState &State )
	{
		auto node = AllocateNode();

		if( node ){
			node->m_UserState = State;
			m_Successors.push_back( node );
			return true;
		}

		return false;
	}

	int GetStepCount() { return m_Steps; }

private:
	// Node memory management
	std::shared_ptr<Node> AllocateNode()
	{
#if !USE_FSA_MEMORY
		return std::make_shared<Node>();
#else
		Node *address = m_FixedSizeAllocator.alloc();

		if( !address )
		{
			return NULL;
		}
                //m_AllocateNodeCount ++;
		Node *p = new (address) Node;
		return p;
#endif
	}

	void FreeNode(Node *node)
	{
                //m_AllocateNodeCount--;

#if !USE_FSA_MEMORY
		delete node;
#else
		node->~Node();
		m_FixedSizeAllocator.free( node );
#endif
	}

private: // data

	// Heap (simple vector but used as a heap, cf. Steve Rabin's game gems article)
	vector< std::shared_ptr<Node> > m_OpenList;

	// Closed list is a vector.
	vector< std::shared_ptr<Node> > m_ClosedList;

	// Successors is a vector filled out by the user each type successors to a node
	// are generated
	vector< std::shared_ptr<Node> > m_Successors;

	// State
	unsigned int m_State;

	// Counts steps
	int m_Steps;

	// Start and goal state pointers
	std::shared_ptr<Node> m_Start;
	std::shared_ptr<Node> m_Goal;

#if USE_FSA_MEMORY
	// Memory
 	FixedSizeAllocator<Node> m_FixedSizeAllocator;
#endif
	
public:
	std::vector< std::vector<UserState> > solutions;
};

template <class T> class AStarState
{
public:
	virtual ~AStarState() {}
	virtual float GoalDistanceEstimate( T &nodeGoal ) = 0; // Heuristic function which computes the estimated cost to the goal node
	virtual bool IsGoal() = 0; // Returns true if this node is the goal node
	virtual bool GetSuccessors( AStarSearch<T> *astarsearch ) = 0; // Retrieves all successors to this node and adds them via astarsearch.addSuccessor()
	virtual float GetCost( T &successor ) = 0; // Computes the cost of traveling from this node to the successor node
	virtual bool IsSameState( T &rhs ) = 0; // Returns true if this node is the same as the rhs node
};

#endif
