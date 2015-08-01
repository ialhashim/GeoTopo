/* Retrieved from: http://en.literateprograms.org/Dijkstra's_algorithm_(C_Plus_Plus)?oldid=13422 */
#pragma once

#include <vector>
#include <map>
#include <list>
#include <queue>
#include <set>
#include <unordered_set>
#include <functional>

#undef min
#undef max
#include <limits>

using namespace std;

#define Max(a,b) (((a) > (b)) ? (a) : (b))
#define Min(a,b) (((a) < (b)) ? (a) : (b))

static size_t GLOBAL_GRAPH_UID = 0;

// Variant class used for attributes, needs a better alternative
#include <QVariant>
#include <QMap>
typedef QMap<QString,QVariant> PropertyMap;

namespace GenericGraphs{

	template <typename VertexType = unsigned int, typename WeightType = double>
	class Graph
	{
	private:
		typedef VertexType vertex_t;
		typedef WeightType weight_t;

	public:
		struct Edge {
			vertex_t target;
			weight_t weight;
			unsigned int index;

			Edge(vertex_t arg_target, weight_t arg_weight, unsigned int edge_index = -1)
				: target(arg_target), weight(arg_weight), index(edge_index) { }

			bool operator == (const Edge & rhs) const{
				return (target == rhs.target && weight == rhs.weight && index == rhs.index);
			}

			bool has(vertex_t v){ return index == v || target == v; } // assuming edge index is for a vertex

			// Attributes
			PropertyMap property;
		};

		struct CompareEdge{
			bool operator()(const Edge &a, const Edge &b){
				if (a.target < b.target) return true;
				if (a.target > b.target) return false;
				return a.index < b.index;
			}
		};

		template<typename T>
		static inline std::size_t make_hash(const T& v){
			return std::hash<T>()(v);
		}
		static inline void hash_combine(std::size_t& h, const std::size_t& v){
			h ^= v + 0x9e3779b9 + (h << 6) + (h >> 2);
		}
		struct hashEdge{
			size_t operator()(const Edge& e) const{
				size_t h = make_hash(e.index);
				hash_combine(h, make_hash(e.target));
				return h;
			}
		};

		typedef std::unordered_set<Edge,hashEdge> EdgesSet;
		typedef std::set<vertex_t> vertices_set;

	private:
		typedef std::map<vertex_t, std::list<Edge> > adjacency_map_t;

		template <typename T1, typename T2>
		struct pair_first_less{
			bool operator()(std::pair<T1,T2> p1, std::pair<T1,T2> p2) const {
				if(p1.first == p2.first) {
					return p1.second < p2.second;
				}
				return p1.first < p2.first;
			}
		};

	public:
		void DijkstraComputePathsMany(const std::set<vertex_t> & sources)
		{
			// Add dummy start node
			auto newNode = AddVertex( vertex_t(vertices.size()) );

			// Add edges to dummy node
			for(auto & v : sources)	AddEdge(v, newNode, weight_t(0), -1);

			DijkstraComputePaths( newNode );

			// Remove edges
			for(auto & v : sources) removeEdge(v, newNode);

			// Remove dummy node
			vertices.erase( newNode );
		}

		void DijkstraComputePaths(vertex_t source)
		{
			if(source == lastStart)
				return;

			lastStart = source;
			previous.clear();

			min_distance.clear();
			min_distance.resize(vertices.size(), std::numeric_limits< WeightType >::infinity());

			for (typename adjacency_map_t::iterator vertex_iter = adjacency_map.begin();
				vertex_iter != adjacency_map.end(); vertex_iter++){
					vertex_t v = vertex_iter->first;
					previous[v] = -1;
			}

			min_distance[source] = 0;

			std::set< std::pair<weight_t, vertex_t>, pair_first_less<weight_t, vertex_t> > vertex_queue;
			vertex_queue.insert(std::make_pair(min_distance[source], source));

			while (!vertex_queue.empty()) {
				vertex_t u = vertex_queue.begin()->second;
				vertex_queue.erase(vertex_queue.begin());

				// Visit each edge exiting u
				for (typename std::list<Edge>::iterator edge_iter = adjacency_map[u].begin();
					edge_iter != adjacency_map[u].end();
					edge_iter++)
				{
					vertex_t v = edge_iter->target;
					weight_t weight = edge_iter->weight;
					weight_t distance_through_u = min_distance[u] + weight;
					if (distance_through_u < min_distance[v]) {
						vertex_queue.erase(std::pair<weight_t, vertex_t>(min_distance[v], v));

						min_distance[v] = distance_through_u;
						previous[v] = u;
						vertex_queue.insert(std::pair<weight_t, vertex_t>(min_distance[v], v));
					}
				}
			}
		}

		std::list<vertex_t> DijkstraGetShortestPathsTo(vertex_t target)
		{
			std::list<vertex_t> path;
			typename std::map<vertex_t, vertex_t>::iterator prev;
			vertex_t vertex = target;

			path.push_front(vertex);

			while((prev = previous.find(vertex)) != previous.end())
			{
				vertex = prev->second;
				path.push_front(vertex);
			}

			// Remove last 'previous' which is null
			if(path.size() > 1) path.erase(path.begin());

			return path;
		}

		std::list<vertex_t> DijkstraShortestPath(vertex_t start, vertex_t end)
		{
			this->DijkstraComputePaths(start);
			return this->DijkstraGetShortestPathsTo(end);
		}

		std::vector<Edge> DijkstraShortestPathEdges(vertex_t start, vertex_t end)
		{
			this->DijkstraComputePaths(start);
			auto path = this->DijkstraGetShortestPathsTo(end);

			std::vector<Edge> result;
			if(path.size() < 2) return result;

			vertex_t lastVert = start;
			for(auto v : path){
				result.push_back(Edge(lastVert, 1.0, v));
				lastVert = v;
			}
			result.erase(result.begin());
			return result;
		}

		int NodeDistance(vertex_t n1, vertex_t n2)
		{
			if(n1 == n2) return 0;
			if(!this->isConnected(n1, n2)) return -1;
			return DijkstraShortestPath(n1, n2).size();
		}

	public:
		// Graph Variables:
		vertices_set vertices;
		adjacency_map_t adjacency_map;
		vertex_t lastStart;

		// Attributes
		std::vector<PropertyMap> vertices_prop;
		void initAttributes(){ vertices_prop.resize(vertices.size()); }

		// Graph-wide
		PropertyMap property;

	public:

		std::vector<weight_t> min_distance;
		std::map<vertex_t, vertex_t> previous;

		Graph()
		{
			lastStart = std::numeric_limits<vertex_t>::max();
			sid = 0;
			pid = 0;
			uid = GLOBAL_GRAPH_UID++;
		}

		Graph(const Graph& from)
		{
			this->adjacency_map = from.adjacency_map;
			this->vertices = from.vertices;
			this->min_distance = from.min_distance;
			this->previous = from.previous;
			this->lastStart = from.lastStart;
			this->sid = from.sid;
			this->uid = from.uid;
			this->pid = from.pid;

			// Attributes
			this->vertices_prop = from.vertices_prop;
			this->property = from.property;
		}

		void AddEdge(vertex_t p1, vertex_t p2, weight_t weight, int index = -1)
		{
			if( IsEdgeExists(p1, p2) ) return;

			adjacency_map[AddVertex(p1)].push_back(Edge(AddVertex(p2), weight, index));
			adjacency_map[AddVertex(p2)].push_back(Edge(AddVertex(p1), weight, index));
		}

		vertex_t AddVertex(vertex_t p)
		{
			vertices.insert(p);

			return p;
		}

		vertex_t FirstVertex()
		{
			if(vertices.empty()) return -1;
			return *vertices.begin();
		}

		inline void removeDirectedEdge(vertex_t p1, vertex_t p2)
		{
			std::list<Edge> * adj = &adjacency_map[p1];

			for(typename std::list<Edge>::iterator i = adj->begin(); i != adj->end(); i++)
			{
				Edge * e = &(*i);

				if(e->target == p2)
				{
					adj->remove(*e);
					return;
				}
			}
		}

		inline void removeEdge(vertex_t p1, vertex_t p2)
		{
			removeDirectedEdge(p1, p2);
			removeDirectedEdge(p2, p1);
		}

		inline void SetDirectedEdgeWeight(vertex_t p1, vertex_t p2, weight_t newWeight)
		{
			std::list<Edge> * adj = &adjacency_map[p1];

			for(typename std::list<Edge>::iterator i = adj->begin(); i != adj->end(); i++)
			{
				Edge * e = &(*i);

				if(e->target == p2)
				{
					e->weight = newWeight;
					return;
				}
			}
		}

		inline void SetEdgeWeight(vertex_t p1, vertex_t p2, weight_t newWeight)
		{
			SetDirectedEdgeWeight(p1, p2, newWeight);
			SetDirectedEdgeWeight(p2, p1, newWeight);
		}

		inline void SetEdgePropertyDirected(vertex_t p1, vertex_t p2, const QString & propName, const QVariant & propValue){
			std::list<Edge> * adj = &adjacency_map[p1];
			for(typename std::list<Edge>::iterator i = adj->begin(); i != adj->end(); i++){
				Edge * e = &(*i);
				if(e->target == p2){
					e->property[propName].setValue(propValue);
					return;
				}
			}
		}

		inline void SetEdgeProperty(vertex_t p1, vertex_t p2, const QString & propName, const QVariant & propValue)
		{
			SetEdgePropertyDirected(p1, p2, propName, propValue);
			SetEdgePropertyDirected(p2, p1, propName, propValue);
		}

		vertex_t GetRandomNeighbour(vertex_t p)
		{
			return adjacency_map[p].front().target;
		}

		std::vector<vertex_t> GetNeighbours(vertex_t p)
		{
			std::vector<vertex_t> neighbours;

			std::list<Edge> * adj = &adjacency_map[p];

			for(typename std::list<Edge>::iterator i = adj->begin(); i != adj->end(); i++)
			{
				Edge * e = &(*i);

				neighbours.push_back(e->target);
			}

			return neighbours;
		}

		vertex_t GetOtherNeighbour(vertex_t p, vertex_t q)
		{
			vertex_t n = p;

			std::list<Edge> * adj = &adjacency_map[p];
			for(typename std::list<Edge>::iterator i = adj->begin(); i != adj->end(); i++)
			{
				Edge * e = &(*i);

				if(e->target != q)
				{
					n = e->target;
					break;
				}
			}

			return n;
		}

		bool isCircular(vertex_t p)
		{
			std::set<vertex_t> visited;

			visited.insert(p);

			bool hasMore = true;

			vertex_t curr = p;
			vertex_t prev = p;

			while(hasMore)
			{
				vertex_t next = GetOtherNeighbour(curr, prev);

				if(next == curr)
					return false;

				if(visited.find(next) != visited.end())
					return true;

				prev = curr;
				curr = next;
			}

			return false;
		}

		bool isConnected(vertex_t v1, vertex_t v2)
		{
			this->DijkstraComputePaths(v1);

			if(min_distance[v2] != std::numeric_limits< WeightType >::infinity())
				return true;

			return false;
		}

		vertices_set GetNodes()
		{
			return vertices;
		}

		bool IsEmpty()
		{
			return vertices.empty();
		}

		bool IsHasVertex( vertex_t v )
		{
			return vertices.find(v) != vertices.end();
		}

		inline bool IsEdgeExists(vertex_t v1, vertex_t v2)
		{
			std::list<Edge> * adj = &adjacency_map[v1];

			for(typename std::list<Edge>::iterator i = adj->begin(); i != adj->end(); i++){
				Edge * e = &(*i);
				if(e->target == v2)
					return true;
			}

			return false;
		}

		// the 'index' of the edge will be replaced with index of a vertex
		std::vector<Edge> GetEdges()
		{
			std::vector<Edge> result;

			for(typename adjacency_map_t::iterator it = adjacency_map.begin(); it != adjacency_map.end(); it++)
			{
				vertex_t v1 = it->first;
				std::list<Edge> adj = it->second;

				for(typename std::list<Edge>::iterator i = adj.begin(); i != adj.end(); i++)
				{
					Edge e = *i;

					auto ej = Edge(Min(e.target, v1), e.weight, Max(e.target, v1));
					ej.property = e.property;
					result.push_back(ej);
				}
			}

			return result;
		}

		EdgesSet GetEdgesSet() const
		{
			EdgesSet result;

			for(auto & edgeList : adjacency_map)
			{
				auto v1 = edgeList.first;
				for(auto & e : edgeList.second)
				{
					auto ej = Edge(Min(e.target, v1), e.weight, Max(e.target, v1));

					ej.property = e.property;
					result.insert(ej);
				}
			}

			return result;
		}

		size_t GetNumberEdges()
		{
			return GetEdgesSet().size();
		}

		std::vector<vertex_t> GetLeaves() const
		{
			std::vector<vertex_t> leaves;

			for(typename adjacency_map_t::const_iterator it = adjacency_map.begin(); it != adjacency_map.end(); it++)
			{
				if(it->second.size() < 2)
					leaves.push_back(it->first);
			}

			return leaves;
		}

		std::vector<vertex_t> BFS( vertex_t start, int limit = -1 )
		{
			std::vector<vertex_t> visitOrder;

			std::queue<vertex_t> toVisit;
			std::map<vertex_t,bool> explored;
			for(auto v : vertices) explored[v] = false;

			toVisit.push( start );
			explored[ start ] = true;

			while( !toVisit.empty() )
			{
				auto v = toVisit.front();
				toVisit.pop();
				visitOrder.push_back( v );

				for(auto & e : adjacency_map[v])
				{
					auto w = e.target;
					if(explored[w]) continue;

					explored[w] = true;
					toVisit.push(w);
				}

				if(limit > 0 && visitOrder.size() > limit)
					break;
			}

			return visitOrder;
		}

		void explore(vertex_t seed, std::set<vertex_t> & explored)
		{
			std::queue<vertex_t> q;

			q.push(seed);
			explored.insert(seed);

			while(!q.empty())
			{
				vertex_t i = q.front();
				q.pop();

				std::list<Edge> * adj = &adjacency_map[i];
				for(typename std::list<Edge>::const_iterator it = adj->begin(); it != adj->end(); it++)
				{
					vertex_t j = it->target;

					// Check: is not visited ?
					if(explored.find(j) == explored.end()) 
					{
						explored.insert(j);
						q.push(j);
					}
				}
			}
		}

		std::vector<std::set<vertex_t> > GetConnectedComponents()
		{
			std::vector< std::set<vertex_t> > connectedComponents;
			std::set<vertex_t> unvisited;

			// fill unvisited set
			for(typename vertices_set::const_iterator it = vertices.begin(); it != vertices.end(); it++)
				unvisited.insert(*it);

			while(unvisited.size() > 1)
			{
				// Take first unvisited node
				vertex_t firstNode = *(unvisited.begin());

				// Explore its tree
				std::set<vertex_t> currVisits;
				currVisits.insert(firstNode);
				explore(firstNode, currVisits);

				// Add as a connected component
				connectedComponents.push_back(currVisits);

				// Remove from unvisited set
				for(typename std::set<vertex_t>::iterator it = currVisits.begin(); it != currVisits.end(); it++)
					unvisited.erase(*it);
			}

			return connectedComponents;
		}

		std::set<vertex_t> GetLargestConnectedComponent()
		{
			std::vector<std::set<vertex_t> > connectedComponents = GetConnectedComponents();

			// Find set with maximum number of nodes
			int maxConnectSize = -1, max_i = 0;
			for(int i = 0; i < (int)connectedComponents.size(); i++)
			{
				int currSize = connectedComponents[i].size();

				if(currSize > maxConnectSize){
					maxConnectSize = currSize;
					max_i = i;
				}
			}

			// Return maximum set
			return connectedComponents[max_i];
		}

		template<typename Container>
		void subGraph(Graph & g, const Container & explored)
		{
			for(auto vi : explored)
			{
				std::list<Edge> adj = g.adjacency_map[vi];

				for(typename std::list<Edge>::iterator e = adj.begin(); e != adj.end(); e++)
					this->AddEdge(vi, e->target, e->weight);
			}

			// Isolated nodes
			for(auto vi : explored)
				this->AddVertex( vi );
		}

		std::vector< Graph <vertex_t,weight_t> > toConnectedParts()
		{
			std::vector< Graph <vertex_t,weight_t> > result;

			// Make a 'bitmap' of visited nodes
			std::map<vertex_t, bool> isVisited;
			for(typename std::set<vertex_t>::iterator it = vertices.begin(); it != vertices.end(); it++)
				isVisited[*it] = false;

			for(typename std::map<vertex_t,bool>::iterator i = isVisited.begin(); i != isVisited.end(); i++)
			{
				// Check if visited
				if(i->second)
					continue;

				vertex_t seed = i->first;

				std::set<vertex_t> explored;
				explore(seed, explored);

				// Add this new connected sub graph from exploration
				result.push_back(Graph<vertex_t, weight_t>());
				result.back().subGraph(*this, explored);

				// mark as visited the explored nodes
				for(typename std::set<vertex_t>::iterator vi = explored.begin(); vi != explored.end(); vi++)
					isVisited[*vi] = true;
			}

			return result;
		}

		std::list<vertex_t> GetLargestConnectedPath()
		{
			std::list<vertex_t> longestPath;

			std::set<vertex_t> seedSet = GetLargestConnectedComponent(); 
			std::vector<vertex_t> leaves = GetLeaves();

			if(!leaves.size())
			{
				leaves.push_back(*this->vertices.begin());
			}

			vertex_t seed = leaves.front();

			DijkstraComputePaths(seed);

			for(typename std::set<vertex_t>::iterator it = seedSet.begin(); it != seedSet.end(); it++)
			{
				std::list<vertex_t> curPath = DijkstraGetShortestPathsTo(*it);

				if(curPath.size() > longestPath.size())
					longestPath = curPath;
			}

			return longestPath;
		}

		bool CheckAdjacent(vertex_t v1, vertex_t v2)
		{
			if(v1 == v2) return true;

			for(typename std::list<Edge>::iterator e = adjacency_map[v1].begin(); e != adjacency_map[v1].end(); e++)
				if(e->target == v2) return true;

			return false;
		}

		std::vector<int> getEdges(int x)
		{
			std::vector<int> edges;
			for(typename std::list<Edge>::iterator e = adjacency_map[x].begin(); e != adjacency_map[x].end(); e++)
				edges.push_back(e->target);
			std::sort(edges.begin(),edges.end());
			return edges;
		}

		// Adapted from python code: http://stackoverflow.com/questions/12367801/finding-all-cycles-in-undirected-graphs/25072113
		std::vector< std::vector<vertex_t> > findAllCycles( int cycle_length_limit = -1 )
		{
			std::vector< std::vector<vertex_t> > cycles;

			auto edges = GetEdgesSet();

			std::function<void(std::vector<vertex_t>)> findNewCycles = [&]( std::vector<vertex_t> sub_path )
			{
				auto visisted = []( vertex_t v, const std::vector<vertex_t> & path ){
					return std::find(path.begin(),path.end(),v) != path.end();
				};

				auto rotate_to_smallest = []( std::vector<vertex_t> path ){
					std::rotate(path.begin(), std::min_element(path.begin(), path.end()), path.end());
					return path;
				};

				auto invert = [&]( std::vector<vertex_t> path ){
					std::reverse(path.begin(),path.end());
					return rotate_to_smallest(path);
				};

				auto isNew = [&cycles]( const std::vector<vertex_t> & path ){
					return std::find(cycles.begin(), cycles.end(), path) == cycles.end();
				};

				vertex_t start_node = sub_path[0];
				vertex_t next_node;

				// visit each edge and each node of each edge
				for(auto edge : edges)
				{
					if( edge.has(start_node) )
					{
						vertex_t node1 = edge.index, node2 = edge.target;

						if(node1 == start_node)
							next_node = node2;
						else
							next_node = node1;

						if( !visisted(next_node, sub_path) )
						{
							// neighbor node not on path yet
							std::vector<vertex_t> sub;
							sub.push_back(next_node);
							sub.insert(sub.end(), sub_path.begin(), sub_path.end());
							findNewCycles( sub );
						} 
						else if( sub_path.size() > 2 && next_node == sub_path.back() )
						{
							// cycle found
							auto p = rotate_to_smallest(sub_path);
							auto inv = invert(p);

							if( isNew(p) && isNew(inv) )
								cycles.push_back( p );
						}
					}
				}
			};

			for(auto edge : edges)
			{
				findNewCycles( std::vector<vertex_t>(1,edge.target ) );
				findNewCycles( std::vector<vertex_t>(1,edge.index) );
			}

			// Option to limit cycles to a specific length
			if(cycle_length_limit > 1){
				std::vector< std::vector<vertex_t> > short_cycles;
				for(auto & cycle : cycles)
					if(cycle.size() <= cycle_length_limit)
						short_cycles.push_back(cycle);
				cycles = short_cycles;
			}

			return cycles;
		}

		int findFixP(vector<int> cand) 
		{
			std::vector<int> connections;
			connections.resize(cand.size());

			// This is necessary for the set_intersection function
			std::sort(cand.begin(),cand.end());

			// Auxiliary lambda function
			auto intersection = [&](int x) -> int {
				vector<int> x_edges = getEdges(x);
				std::vector<int> intersection;

				set_intersection(x_edges.begin(), x_edges.end(),
					cand.begin(), cand.end(),
					back_inserter(intersection));
				return intersection.size();
			};

			// Create an auxiliary vector with the intersection sizes
			std::transform(cand.begin(),cand.end(),connections.begin(),intersection);

			// Find the maximum size and return the corresponding edge
			std::vector<int>::const_iterator it1, it2,itmax;
			int max = -1;
			itmax = cand.end();
			for(it1=cand.begin(),it2=connections.begin(); it1!=cand.end(); ++it1,++it2){
				if(max < *it2){
					max = *it2;
					itmax = it1;
				}
			}
			if(itmax == cand.end()) return -1;
			else return *itmax;
		}

		// Bron–Kerbosch algorithm
		std::vector< std::vector<int> > cliques()
		{
			std::vector< std::vector<int> > result;
			std::vector<int> compsub;
			std::vector<int> cand, cnot;
			for(auto v : vertices) cand.push_back(v);

			cliqueEnumerate(compsub, cand, cnot, result);

			return result;
		}

		// Adapted from : http://www.tonicebrian.com/2011/08/29/maximal-clique-enumeration-using-c0x-and-the-stl/
		void cliqueEnumerate(const std::vector<int>& compsub, 
			std::vector<int> cand, std::vector<int> cnot,
			std::vector< std::vector<int> >& result) 
		{
			// Function that answer whether the node is connected
			if(cand.empty()){
				if(cnot.empty()){
					// New clique found
					result.push_back(compsub);
				}
			} else {
				int fixp = findFixP(cand);
				int cur_v = fixp;

				while(cur_v != -1)
				{
					std::vector<int> new_not;
					std::vector<int> new_cand;
					std::vector<int> new_compsub;

					// Auxiliary lambda function
					auto isConected =[cur_v,this](int x) {
						const std::vector<int>& edges = this->getEdges(x);
						return find(edges.begin(), edges.end(), cur_v) != edges.end();
					};

					// Compose new vector
					// Avoid performance bottlenecks by reserving memory before hand
					new_compsub.reserve(compsub.size()+1);
					new_not.reserve(cnot.size());
					new_cand.reserve(cand.size());
					copy_if(cnot.begin(),cnot.end(),back_inserter(new_not),isConected);
					copy_if(cand.begin(),cand.end(),back_inserter(new_cand),isConected);
					copy(compsub.begin(), compsub.end(), back_inserter(new_compsub));
					new_compsub.push_back(cur_v);

					// Recursive call
					cliqueEnumerate(new_compsub, new_cand, new_not, result);

					// Generate new cnot and cand for the loop
					cnot.push_back(cur_v);
					cand.erase(find(cand.begin(),cand.end(),cur_v));

					// Last check
					auto v = find_if(cand.begin(),
						cand.end(),
						[fixp,this](int x) {
							const std::vector<int>& edges = this->getEdges(x);
							return find(edges.begin(), edges.end(), fixp) == edges.end();
					});

					// Obtain new cur_v value
					if(v != cand.end()) cur_v = *v;
					else break;
				}
			}
		}

		// Custom identifiers
		size_t uid, sid, pid;
	};
}
