//
//  HeuristicError.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/24/18.
//  Copyright © 2018 University of Denver. All rights reserved.
//

#ifndef HeuristicError_h
#define HeuristicError_h

#include "Heuristic.h"
#include <functional>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <iostream>
#include <utility>
#include "AStarOpenClosed.h"


// This function counts the number of states that match the functional criteria on the heuristic when
// regressing the given distance and then doing a forward search the forward distance.
// For instance, HR2 could be tested by regressing 3, forward 2, and testing for 0 heuristics.
// Returns the percentage of states at the regression distance that have a child that meet the criteria


template <class environment, class state, typename func>
float MeasureHeuristicErrors(environment *e, state s, Heuristic<state> *h, int regressionDistance, int forwardDistance, func f)
{
  
//	std::cout << "Processing " << s << "\n";
	// Store states and their depth
	std::unordered_map<state, std::pair<int,double>> table;
	std::vector<state> succ;
	table[s] = std::make_pair(0,0);
	for (int x = 0; x < regressionDistance; x++)
	{
		for (auto iter = table.begin(); iter != table.end(); iter++)
		{
			// Expands states at current depth
			if (iter->second.first == x)
			{
				e->GetSuccessors(iter->first, succ);
				for (auto &itm : succ)
				{
					if (table.find(itm) == table.end())
					{
//						std::cout << "Adding (" << itm << ") at depth " << x+1 << "\n";
						table[itm] = std::make_pair(x+1,iter->second.second + e->GCost(iter->first,itm));
					}
				}
			}
		}
	}

	std::function<int (const state&, int,double)> DFS;
	DFS = [h,&s,&f,e,&DFS,&table](const state &i, int depth,double distance)->int {
    //printf("%d, %f\n",depth,distance);
    if (depth == 0){
        return ((fless(h->HCost(i, s),distance)) ? 1 : 0);
    }
		std::vector<state> neighbors;
		e->GetSuccessors(i, neighbors);
		int count = 0;
		for (const state &succ : neighbors)
		{
//			if (table.find(succ) != table.end())
//				continue;
			count += DFS(succ, depth-1,distance - e->GCost(i,succ));
		}
		return count;
	};

	float total = 0;
	float pass = 0;
	float children = 0;
	for (auto iter = table.begin(); iter != table.end(); iter++)
	{
		// Expands states at current depth
		if (iter->second.first == regressionDistance)
		{
			total++;
			int res = DFS(iter->first, forwardDistance,iter->second.second);
      //printf("%d\n", res);
			if (res > 0)
				pass++;
			children += res;
		}
	}
	printf("%d of %d match condition. Avg %1.2f per state\n", (int)pass, (int)total, children/total);
	return pass/total;
  
  return 0;
}

  template <class state>
  struct gCompareofElements{
    bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
    {
      return fless(i2.g, i1.g);
    }
  };
template <class environment, class state>  
void StateNeighborsUpToDistance(environment *e, state s,Heuristic<state> *h,double distance){
  
  SearchUpToDistance(e,s,h,distance,false);
}


template <class environment, class state>
double SearchUpToDistance(environment *e, state s,Heuristic<state> *h,double distance, bool isNodeLimit)
{
  

  int countInaccurate = 0;
  int countLow = 0;
  double maxg;
  std::vector<state> neighbors;
	std::vector<uint64_t> neighborID;
	std::vector<double> edgeCosts;
	std::vector<dataLocation> neighborLoc;
  AStarOpenClosed<state, gCompareofElements<state>> openClosedList;
  openClosedList.Reset(e->GetMaxHash());
  openClosedList.AddOpenNode(s, e->GetStateHash(s), 0, h->HCost(s,s));
  while(openClosedList.OpenSize() != 0){
      if (isNodeLimit && distance <= 0){
        break;
      }
      uint64_t nodeid = openClosedList.Close();
      if (isNodeLimit){
        distance--;
      }
      maxg = openClosedList.Lookup(nodeid).g;
      //printf("%f %f \n",openClosedList.Lookup(nodeid).g ,openClosedList.Lookup(nodeid).h);
      if (!isNodeLimit && openClosedList.Lookup(nodeid).g > distance){
        break;
      }
      
      if (openClosedList.Lookup(nodeid).h <= distance){
        countLow++;
        if (abs(openClosedList.Lookup(nodeid).h-openClosedList.Lookup(nodeid).g)>=1){
          countInaccurate++;
        }
        //std::cout << openClosedList.Lookup(nodeid).h << " " << openClosedList.Lookup(nodeid).g << std::endl;
      }
      neighbors.resize(0);
      edgeCosts.resize(0);
      neighborID.resize(0);
      neighborLoc.resize(0);
      e->GetSuccessors(openClosedList.Lookup(nodeid).data, neighbors);
      for (unsigned int x = 0; x < neighbors.size(); x++)
      {
        uint64_t theID;
        neighborLoc.push_back(openClosedList.Lookup(e->GetStateHash(neighbors[x]), theID));
        neighborID.push_back(theID);
        edgeCosts.push_back(e->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]));
      }
      for (int x = 0; x < neighbors.size(); x++)
      {
        
        switch (neighborLoc[x])
        {
          case kClosedList:
            break;
          case kOpenList:
            //printf("n %f %f \n",openClosedList.Lookup(neighborID[x]).g ,openClosedList.Lookup(neighborID[x]).h);
            if (fless(openClosedList.Lookup(nodeid).g+edgeCosts[x], openClosedList.Lookup(neighborID[x]).g))
            {
              openClosedList.Lookup(neighborID[x]).parentID = nodeid;
              openClosedList.Lookup(neighborID[x]).g = openClosedList.Lookup(nodeid).g+edgeCosts[x];
              openClosedList.Lookup(neighborID[x]).data = neighbors[x];
              openClosedList.KeyChanged(neighborID[x]);
            }
            break;
          case kNotFound:
            { 
              //printf("n %f %f \n",openClosedList.Lookup(nodeid).g+edgeCosts[x] ,h->HCost(neighbors[x], s));
              openClosedList.AddOpenNode(neighbors[x],
                             e->GetStateHash(neighbors[x]),
                             openClosedList.Lookup(nodeid).g+edgeCosts[x],
                             h->HCost(neighbors[x], s),
                             nodeid);
            }
        }
      }
  }
  if (!isNodeLimit){
    printf("NodesNum %d, NodesIaccurate %d\n",countLow,countInaccurate);
  }
  return maxg;

  // std::unordered_set<state> forwardSet;
  // std::priority_queue<std::pair<state,double>,std::vector<std::pair<state,double>>,gCompareofElements> queue;
  // queue.push(std::make_pair(s,0));
  // forwardSet.insert(s);
	// std::vector<state> succ;
  // int countInaccurate = 0;
  // while(!queue.empty()){
    // std::pair<state,double> cur = queue.top();
    // queue.pop();
    // if (fless(h->HCost(cur.first,s),cur.second)){
      // countInaccurate++;
    // }
    // e->GetSuccessors(cur.first, succ);
    // for (auto &itm : succ)
    // {
      // if (e->GCost(cur.first,itm) + cur.second <= distance){
        // auto inset = forwardSet.insert(itm);
        // if (inset.second){
          // queue.push(std::make_pair(itm,e->GCost(cur.first,itm) + cur.second));
        // }
        // else{
          // for (auto &ele : queue){
              // if (ele.first == itm){
                // if (ele.second > e->GCost(cur.first,itm) + cur.second){
                  // ele.second = e->GCost(cur.first,itm) + cur.second;
                // }
              // break;
            // }
          // }
        // }
      // }
    // }
  // }

	// printf("NodesNum %d, NodesIaccurate %d\n",forwardSet.size(),countInaccurate);
}

template <class environment, class state>
bool IsGoalInRadius(environment *e, state s,state goal,Heuristic<state> *h,double distance)
{
  int countInaccurate = 0;
  int countLow = 0;
  double maxg;
  std::vector<state> neighbors;
	std::vector<uint64_t> neighborID;
	std::vector<double> edgeCosts;
	std::vector<dataLocation> neighborLoc;
  AStarOpenClosed<state, gCompareofElements<state>> openClosedList;
  openClosedList.Reset(e->GetMaxHash());
  openClosedList.AddOpenNode(s, e->GetStateHash(s), 0, h->HCost(s,s));
  while(openClosedList.OpenSize() != 0){
      uint64_t nodeid = openClosedList.Close();
      if (e->GoalTest(openClosedList.Lookup(nodeid).data, goal)){
        return true;
      }
      //printf("%f %f \n",openClosedList.Lookup(nodeid).g ,openClosedList.Lookup(nodeid).h);
      if (openClosedList.Lookup(nodeid).g > distance){
        return false;
      }
      
      if (openClosedList.Lookup(nodeid).h <= distance){
        countLow++;
        if (abs(openClosedList.Lookup(nodeid).h-openClosedList.Lookup(nodeid).g)>=1){
          countInaccurate++;
        }
        //std::cout << openClosedList.Lookup(nodeid).h << " " << openClosedList.Lookup(nodeid).g << std::endl;
      }
      neighbors.resize(0);
      edgeCosts.resize(0);
      neighborID.resize(0);
      neighborLoc.resize(0);
      e->GetSuccessors(openClosedList.Lookup(nodeid).data, neighbors);
      for (unsigned int x = 0; x < neighbors.size(); x++)
      {
        uint64_t theID;
        neighborLoc.push_back(openClosedList.Lookup(e->GetStateHash(neighbors[x]), theID));
        neighborID.push_back(theID);
        edgeCosts.push_back(e->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]));
      }
      for (int x = 0; x < neighbors.size(); x++)
      {
        
        switch (neighborLoc[x])
        {
          case kClosedList:
            break;
          case kOpenList:
            //printf("n %f %f \n",openClosedList.Lookup(neighborID[x]).g ,openClosedList.Lookup(neighborID[x]).h);
            if (fless(openClosedList.Lookup(nodeid).g+edgeCosts[x], openClosedList.Lookup(neighborID[x]).g))
            {
              openClosedList.Lookup(neighborID[x]).parentID = nodeid;
              openClosedList.Lookup(neighborID[x]).g = openClosedList.Lookup(nodeid).g+edgeCosts[x];
              openClosedList.Lookup(neighborID[x]).data = neighbors[x];
              openClosedList.KeyChanged(neighborID[x]);
            }
            break;
          case kNotFound:
            { 
              //printf("n %f %f \n",openClosedList.Lookup(nodeid).g+edgeCosts[x] ,h->HCost(neighbors[x], s));
              openClosedList.AddOpenNode(neighbors[x],
                             e->GetStateHash(neighbors[x]),
                             openClosedList.Lookup(nodeid).g+edgeCosts[x],
                             h->HCost(neighbors[x], s),
                             nodeid);
            }
        }
      }
  }
  return false;

}

template <class environment, class state>
double dijkstraUptoLimit(environment *e, state s,int lim)
{
  return SearchUpToDistance(e,s,e,lim,true);

  
  // struct gCompareofPairs{
    // bool operator() (const std::pair<state,int> &i1,const std::pair<state,int> &i2) const
    // {
        // return fless(i1.second,i2.second);
    // }
    
  // };

  // std::unordered_set<state> forwardSet;
  // std::priority_queue<std::pair<state,double>,std::vector<std::pair<state,double>>,gCompareofPairs> queue;
  // queue.push(std::make_pair(s,0));
  // forwardSet.insert(s);
	// std::vector<state> succ;
  // double maxg = 0;
  // while(!queue.empty() && lim > 0){
    // std::pair<state,double> cur = queue.top();
    // queue.pop();
    // maxg = cur.second;
    
    // e->GetSuccessors(cur.first, succ);
    // for (auto &itm : succ)
    // {
      // if (forwardSet.insert(itm).second){
        // queue.push(std::make_pair(itm,e->GCost(cur.first,itm) + cur.second));
      // }
    // }
    // lim--;
  // }
  
	// return maxg;
}


template <class environment, class state, typename func>
void printNodesInDistance(environment *e, state s, Heuristic<state> *h, int regressionDistance, int forwardDistance, func f)
{
  
//	std::cout << "Processing " << s << "\n";
	// Store states and their depth
	std::unordered_map<state, int> table;
  std::unordered_set<state> forwardSet;
  std::unordered_set<state> forwardInaccurateSet;
	std::vector<state> succ;
	table[s] = 0;
	for (int x = 0; x < regressionDistance; x++)
	{
		for (auto iter = table.begin(); iter != table.end(); iter++)
		{
			// Expands states at current depth
			if (iter->second == x)
			{
				e->GetSuccessors(iter->first, succ);
				for (auto &itm : succ)
				{
					if (table.find(itm) == table.end())
					{
//						std::cout << "Adding (" << itm << ") at depth " << x+1 << "\n";
						table[itm] = x+1;
					}
				}
			}
		}
	}

	std::function<void (const state&, int)> DFS;
	DFS = [h,&s,&f,e,&DFS,&table,&forwardInaccurateSet,&forwardSet](const state &i, int depth)->void {
		if (depth == 0){
      if (f(h->HCost(i, s))){
        forwardInaccurateSet.insert(i);
      }
      forwardSet.insert(i);
      return;
    }
    //std::cout << i << std::endl;
		std::vector<state> neighbors;
		e->GetSuccessors(i, neighbors);
		int count = 0;
		for (const state &succ : neighbors)
		{
//			if (table.find(succ) != table.end())
//				continue;
			DFS(succ, depth-1);
		}
	};

	float total = 0;
	float pass = 0;
	float children = 0;
	for (auto iter = table.begin(); iter != table.end(); iter++)
	{
		// Expands states at current depth
		if (iter->second == regressionDistance)
		{
			DFS(iter->first, forwardDistance);
		}
	}
	printf("%d %d %d\n", table.size(), forwardSet.size(), forwardInaccurateSet.size());
}

#endif /* HeuristicError_h */
