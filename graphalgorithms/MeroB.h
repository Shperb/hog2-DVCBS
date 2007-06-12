/*
 *  MeroB.h
 *  hog2
 *
 *  Created by Nathan Sturtevant, modified by Zhifu Zhang
 *
 */

#ifndef MEROB_H
#define MEROB_H

#include <math.h>
#include "SearchEnvironment.h"
#include "GraphEnvironment.h"
#include <ext/hash_map>
#include "FPUtil.h"
#include "OpenClosedList.h"

#ifndef UINT32_MAX
#define UINT32_MAX        4294967295U
#endif


#define MB_A 1
#define MB_B 2
#define MB_BP 3

//typedef __gnu_cxx::hash_map<uint64_t, double> NodeHashTable;

namespace MeroBUtil
{
	class SearchNode {
	public:
		SearchNode(double _fCost=0, double _gCost=0, graphState curr=0, graphState prev=0)
		:fCost(_fCost), gCost(_gCost), currNode(curr), prevNode(prev) {}

		SearchNode(graphState curr)
		:fCost(0), gCost(0), currNode(curr), prevNode(0) {}

		void copy(double f, double g, graphState curr, graphState prev) {
			fCost = f;
			gCost = g;
			currNode = curr;
			prevNode = prev;
		}

		double fCost;
		double gCost;
		graphState currNode;
		graphState prevNode;
	};

	struct SearchNodeEqual {
		bool operator()(const SearchNode &i1, const SearchNode &i2)
		{ return (i1.currNode == i2.currNode); } 
	};

	struct SearchNodeCompare { // true means i2 is preferable over i1
		// prefering larger g, i.e. smaller h is also in favor of goal nodes
		bool operator()(const SearchNode &i1, const SearchNode &i2)
		{
			if (fequal(i1.fCost, i2.fCost))
			{
				return (fless(i1.gCost, i2.gCost));
			}
			return (fgreater(i1.fCost, i2.fCost));
		} 
	};

	struct GGreater {
		bool operator()(const SearchNode &i1, const SearchNode &i2) {
			if(fequal(i1.gCost,i2.gCost))
				return fgreater(i1.fCost,i2.fCost);

			return fgreater(i1.gCost,i2.gCost);
		}
	};

	struct SearchNodeHash {
		size_t operator()(const SearchNode &x) const
		{ return (size_t)(x.currNode); }
	};

	struct graphGenerator {
		static void SetLoc(node *n, double x, double y, double z)
		{
			n->SetLabelL(GraphSearchConstants::kXCoordinate, x);
			n->SetLabelL(GraphSearchConstants::kYCoordinate, y);
			n->SetLabelL(GraphSearchConstants::kZCoordinate, z);
		}
		
		static Graph* genFig1(int N)
	  {
			// h(0) = h(1) = 0; 
			// h(i) = 2^(i-1) + 2i - 3, for 1<i<=N
			// c(i,j) = 2^(i-2) + i - 2^(j-1) - j, for 1<=j<i<=N
			// c(1,0) = 2^(N-1) + N - 2
			assert(N >= 3);

			Graph *g = new Graph();

			// add nodes
			
			for(unsigned int nodeID = 0; nodeID <= N; nodeID++)
			{
				node *n = new node("");
				if (nodeID == 0 || nodeID == 1)
				{
					n->SetLabelF(GraphSearchConstants::kHCost, 0);
				}
				else {
					double h = pow(2,nodeID-1) + 2*nodeID - 3;
					n->SetLabelF(GraphSearchConstants::kHCost,h);
				}
				g->AddNode(n); // the real nodeID is assigned here
			}

			// add edges
			double c = pow(2,N-1) + N - 2;
			edge *e = new edge(1,0,c);
			g->AddEdge(e);
			for (int j = 1; j < N; j++)
			{
				for (int i = j+1; i <= N; i++)
				{
					c = pow(2,i-2) + i - pow(2,j-1) - j;
					e = new edge(i,j,c);
					g->AddEdge(e);
				}
			}

			return g;
		}

		static Graph* genFig2(int N)
	  {
			// h(0) = h(2N-1) = 0
			// h(i) = 2(N-1)^2 - N - i + 2, for i = 1,...,N-1
			// h(N+j) = 2(N-1)(N-2-j) + 1,  for j = 0,...,N-2
			// c(0,i) = 1,  for i = 1,...,N-1
			// c(i,N) = 2(N-1)(i-1) + 1,  for i = 1,...,N-1
			// c(i,i+1) = 2N - 2,  for i = N,...,2N-1 (should it be 2N-2 ?)
			assert(N >= 2);

			Graph* g = new Graph();

			// add nodes
			node* n = new node("");
			n->SetLabelF(GraphSearchConstants::kHCost,0);
			SetLoc(n, 1, 0.9, 0);
			g->AddNode(n);

			for (unsigned int i=1;i<=N-1;i++)
			{
				n = new node("");
				double h = 2*(N-1)*(N-1) - N - i + 2;
				n->SetLabelF(GraphSearchConstants::kHCost,h);
				g->AddNode(n);
				SetLoc(n, -1+2.0/((double)N-1.0), 0.0, 0);
			}

			for(unsigned int j=0;j<=N-2;j++) {
				n = new node("");
				double h = 2*(N-1)*(N-2-j) + 1;
				n->SetLabelF(GraphSearchConstants::kHCost,h);
				g->AddNode(n);
				SetLoc(n, 0-2.0/((double)N-1.0), -0.9+(j%2)?0.5:0.0, 0);
			}

			n = new node(""); // the last node (2N-1)
			n->SetLabelF(GraphSearchConstants::kHCost,0);
			g->AddNode(n);

			// add edges

			// type 1
			for (unsigned int i=1;i<=N-1;i++)
			{
				edge* e = new edge(0,i,1);
				g->AddEdge(e);
			}

			// type 2
			for (unsigned int i=1;i<=N-1;i++)
			{
				double c = 2*(N-1)*(i-1) + 1;
				edge* e = new edge(i,N,c);
				g->AddEdge(e);
			}

			// type 3
			for (unsigned int i=N;i<=2*N-2;i++)
			{
				double c = 2*N - 2;
				edge* e = new edge(i,i+1,c);
				g->AddEdge(e);
			}

			return g;
		}
	};

	typedef OpenClosedList<MeroBUtil::SearchNode, MeroBUtil::SearchNodeHash,
		MeroBUtil::SearchNodeEqual, MeroBUtil::SearchNodeCompare> PQueue;

	typedef __gnu_cxx::hash_map<graphState, MeroBUtil::SearchNode > NodeLookupTable;

	typedef OpenClosedList<MeroBUtil::SearchNode, MeroBUtil::SearchNodeHash,
		MeroBUtil::SearchNodeEqual, MeroBUtil::GGreater> GQueue;
}

class MeroB {
public:
	MeroB() { verID = MB_A;}
	MeroB(unsigned int v) { verID = v; }
	virtual ~MeroB() {}
	void GetPath(GraphEnvironment *env, graphState from, graphState to, std::vector<graphState> &thePath);
	
	long GetNodesExpanded() { return nodesExpanded; }
	long GetNodesTouched() { return nodesTouched; }

	bool InitializeSearch(GraphEnvironment *env, graphState from, graphState to, std::vector<graphState> &thePath);
	bool DoSingleSearchStep(std::vector<graphState> &thePath);
	bool DoSingleStepA(std::vector<graphState> &thePath);
	bool DoSingleStepB(std::vector<graphState> &thePath);
	bool DoSingleStepBP(std::vector<graphState> &thePath);
	void ExtractPathToStart(graphState goalNode, std::vector<graphState> &thePath);

private:
	unsigned int verID;
	double F;
	long nodesExpanded, nodesTouched;
	std::vector<graphState> neighbors;
	graphState goal, start;
	GraphEnvironment *env;
	MeroBUtil::PQueue openQueue;
	MeroBUtil::NodeLookupTable closedList; 
	MeroBUtil::GQueue FCache; // storing nodes with f < F, this is temporary cache

};	

#endif
