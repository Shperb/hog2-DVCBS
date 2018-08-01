//
//	CBBS.h
//	This file derived from MM.h by Nathan Sturtevant
//	The following is the original claim
//
//  MM.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 10/27/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#ifndef CBBS_H
#define CBBS_H

#include "CBBSOpenClosed.h"
#include "FPUtil.h"
#include <unordered_map>
#include "CBBSQueue.h"
#include "NBSQueueGF.h"
#include <algorithm>

using std::cout;


template <class state, class action, class environment, class dataStructure = CBBSQueue<state>,
          class priorityQueue = CBBSOpenClosed<state> >
class CBBS {
public:
	CBBS(int tieBreaking, bool allSolutions = false)
	{
		forwardHeuristic = 0; backwardHeuristic = 0; env = 0; ResetNodeCount();
		tieBreakingPolicy = tieBreaking;
		isAllSolutions = allSolutions;
	}
	virtual ~CBBS() {}
	void GetPath(environment *env, const state& from, const state& to,
				 Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);
	bool InitializeSearch(environment *env, const state& from, const state& to,
						  Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);
	bool ExpandAVertexCover(std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	
	uint64_t getForwardUnnecessaryNodesInPath(){
		return forwardUnnecessaryNodesInPath;
	}
	
	uint64_t getBackwardUnnecessaryNodesInPath(){
		return backwardUnnecessaryNodesInPath;
	}

	uint64_t getForwardMeetingPoint(){
		return forwardMeetingPoint;
	}
	
	uint64_t getBackwardMeetingPoint(){
		return backwardMeetingPoint;
	}
	
	uint64_t getOptimalNumberOfExpantions(){
		//uint64_t theID; queue.forwardQueue.Lookup(env->GetStateHash(middleNode), theID);
		//double cost = queue.forwardQueue.Lookup(theID).g;
		//queue.backwardQueue.Lookup(env->GetStateHash(middleNode), theID);
		//double cost2 = queue.backwardQueue.Lookup(theID).g;
		printf("curr Cost: %d\n",currentCost);
		//printf("cost: %d\n",cost);
		//printf("cost2: %d\n",cost2);
		return queue.getMinimalVertexCover(currentCost);
	}
	
	
	virtual const char *GetName() { return "CBBS"; }
	
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; counts.clear(); }
	
	inline const int GetNumForwardItems() { return queue.forwardQueue.size(); }
	inline const CBBSOpenClosedData<state> &GetForwardItem(unsigned int which) { return queue.forwardQueue.Lookat(which); }
	inline const int GetNumBackwardItems() { return queue.backwardQueue.size(); }
	inline const CBBSOpenClosedData<state> &GetBackwardItem(unsigned int which) { return queue.backwardQueue.Lookat(which); }
	
	void SetForwardHeuristic(Heuristic<state> *h) { forwardHeuristic = h; }
	void SetBackwardHeuristic(Heuristic<state> *h) { backwardHeuristic = h; }
	stateLocation GetNodeForwardLocation(const state &s)
	{
		uint64_t childID;
		auto l = queue.forwardQueue.Lookup(env->GetStateHash(s), childID);
		return l;
	}
	stateLocation GetNodeBackwardLocation(const state &s)
	{
		uint64_t childID;
		return queue.backwardQueue.Lookup(env->GetStateHash(s), childID);
	}
	double GetNodeForwardG(const state& s)
	{
		
		uint64_t childID;
		auto l = queue.forwardQueue.Lookup(env->GetStateHash(s), childID);
		if (l != kUnseen)
			return queue.forwardQueue.Lookat(childID).g;
		return -1;
	}
	double GetNodeBackwardG(const state& s)
	{
		
		uint64_t childID;
		auto l = queue.backwardQueue.Lookup(env->GetStateHash(s), childID);
		if (l != kUnseen)
			return queue.backwardQueue.Lookat(childID).g;
		return -1;
	}
	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	uint64_t GetDoubleExpansions() const;
	uint64_t GetNecessaryExpansions() const {
		uint64_t necessary = 0;
		for (const auto &i : counts)
		{
			if (isAllSolutions){
				if (i.first <= currentCost)
					necessary+=i.second;
			}
			else{
				if (i.first < currentCost)
					necessary+=i.second;
			}
		}
		return necessary;
	}
	double GetSolutionCost() const { return currentCost; }
	double GetExpansionUntilFirstSolution() const { return expansionsUntilSolution; }

	
	void OpenGLDraw() const;
	
	//	void SetWeight(double w) {weight = w;}
private:
	void ExtractFromMiddle(std::vector<state> &thePath);
	double ExtractCostFromMiddle();
	void ExtractPathToGoal(state &node, std::vector<state> &thePath)
	{ uint64_t theID; queue.backwardQueue.Lookup(env->GetStateHash(node), theID); ExtractPathToGoalFromID(theID, thePath); }
	void ExtractPathToGoalFromID(uint64_t node, std::vector<state> &thePath)
	{
		do {
			thePath.push_back(queue.backwardQueue.Lookup(node).data);
			backwardMeetingPoint++;
			//printf("B: <%f,%f,%f> ",queue.backwardQueue.Lookup(node).g,queue.backwardQueue.Lookup(node).h,currentCost);
			if (queue.backwardQueue.Lookup(node).g+queue.backwardQueue.Lookup(node).h == currentCost){
				backwardUnnecessaryNodesInPath++;
			}
			node = queue.backwardQueue.Lookup(node).parentID;
		} while (queue.backwardQueue.Lookup(node).parentID != node);
		thePath.push_back(queue.backwardQueue.Lookup(node).data);
		//printf("B: <%f,%f,%f> ",queue.backwardQueue.Lookup(node).g,queue.backwardQueue.Lookup(node).h,currentCost);

	}
	
	double ExtractCostToGoal(state &node)
	{ uint64_t theID; queue.backwardQueue.Lookup(env->GetStateHash(node), theID); return ExtractCostToGoalFromID(theID); }
	double ExtractCostToGoalFromID(uint64_t node)
	{
		double cost = 0;
		do {
			cost += queue.backwardQueue.Lookup(node).g;
			node = queue.backwardQueue.Lookup(node).parentID;
		} while (queue.backwardQueue.Lookup(node).parentID != node);
		cost += queue.backwardQueue.Lookup(node).g;
		return cost;
	}
		
	void ExtractPathToStart(state &node, std::vector<state> &thePath)
	{ uint64_t theID; queue.forwardQueue.Lookup(env->GetStateHash(node), theID); ExtractPathToStartFromID(theID, thePath); }
	void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath)
	{
		do {
			thePath.push_back(queue.forwardQueue.Lookup(node).data);
			forwardMeetingPoint++;
			//printf("F: <%f,%f,%f> ",queue.forwardQueue.Lookup(node).g,queue.forwardQueue.Lookup(node).h,currentCost);
			if (queue.forwardQueue.Lookup(node).g+queue.forwardQueue.Lookup(node).h == currentCost){
				forwardUnnecessaryNodesInPath++;
			}
			node = queue.forwardQueue.Lookup(node).parentID;
		} while (queue.forwardQueue.Lookup(node).parentID != node);
		thePath.push_back(queue.forwardQueue.Lookup(node).data);
		//printf("F: <%f,%f,%f> ",queue.forwardQueue.Lookup(node).g,queue.forwardQueue.Lookup(node).h,currentCost);
	}
	
	double  ExtractCostToStart(state &node)
	{ uint64_t theID; queue.forwardQueue.Lookup(env->GetStateHash(node), theID); return ExtractCostToStartFromID(theID); }
	double ExtractCostToStartFromID(uint64_t node)
	{
		double cost = 0;
		do {
			cost += queue.forwardQueue.Lookup(node).g;
			printf("cost: %d ",cost);
			node = queue.forwardQueue.Lookup(node).parentID;
		} while (queue.forwardQueue.Lookup(node).parentID != node);
		cost += queue.forwardQueue.Lookup(node).g;
		return cost;
	}
	
	void OpenGLDraw(const priorityQueue &queue) const;
	
	void Expand(uint64_t nextID,
				priorityQueue &current,
				priorityQueue &opposite,
				Heuristic<state> *heuristic, const state &target);
	//direction ==0 forward; 1 backward
	//void Expand(int direction);
	uint64_t nodesTouched, nodesExpanded;
	state middleNode;
	double currentCost;
	double expansionsUntilSolution;
	double currentSolutionEstimate;
	std::vector<state> neighbors;
	environment *env;
	std::unordered_map<double, int> counts;
	bool isAllSolutions;
	
	dataStructure queue;
	//	priorityQueue queue.forwardQueue, queue.backwardQueue;
	//priorityQueue2 queue.forwardQueue, queue.backwardQueue;
	
	state goal, start;
	
	Heuristic<state> *forwardHeuristic;
	Heuristic<state> *backwardHeuristic;
	
	//keep track of whether we expand a node or put it back to open
	bool expand;
	
	double currentPr;
	
	int tieBreakingPolicy;
	
	uint64_t forwardUnnecessaryNodesInPath;
	uint64_t backwardUnnecessaryNodesInPath;
	uint64_t forwardMeetingPoint;
	uint64_t backwardMeetingPoint;	
	
};

template <class state, class action, class environment, class dataStructure, class priorityQueue>
void CBBS<state, action, environment, dataStructure, priorityQueue>::GetPath(environment *env, const state& from, const state& to,
																			 Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath)
{
	if (InitializeSearch(env, from, to, forward, backward, thePath) == false)
		return;
	
	while (!ExpandAVertexCover(thePath))
	{ }
}

template <class state, class action, class environment, class dataStructure, class priorityQueue>
bool CBBS<state, action, environment, dataStructure, priorityQueue>::InitializeSearch(environment *env, const state& from, const state& to,
																					  Heuristic<state> *forward, Heuristic<state> *backward,
																					  std::vector<state> &thePath)
{
	this->env = env;
	forwardHeuristic = forward;
	backwardHeuristic = backward;
	currentSolutionEstimate = 0;
	forwardUnnecessaryNodesInPath = 0;
	backwardUnnecessaryNodesInPath = 0;
	forwardMeetingPoint = 0;
	backwardMeetingPoint = 0;
	currentCost = DBL_MAX;
	expansionsUntilSolution = 0;
	queue.Reset();
//	queue.forwardQueue.Reset();
//	queue.backwardQueue.Reset();
	ResetNodeCount();
	thePath.resize(0);
	start = from;
	goal = to;
	if (start == goal)
		return false;
	
	queue.forwardQueue.AddOpenNode(start, env->GetStateHash(start), 0, forwardHeuristic->HCost(start, goal));
	queue.backwardQueue.AddOpenNode(goal, env->GetStateHash(goal), 0, backwardHeuristic->HCost(goal, start));
	
	return true;
}

template <class state, class action, class environment, class dataStructure, class priorityQueue>
bool CBBS<state, action, environment, dataStructure, priorityQueue>::ExpandAVertexCover(std::vector<state> &thePath)
{
	std::vector<uint64_t> nForward, nBackward;
	bool result = queue.getVertexCover(nForward, nBackward,tieBreakingPolicy);
	// if failed, see if we have optimal path (but return)
	if (result == false)
	{
		if (currentCost == DBL_MAX)
		{
			thePath.resize(0);
			//printf("here1");
			return true;
		}
		ExtractFromMiddle(thePath);
		//printf("here2");
		return true;
	}
	/*
	else if (nForward.size() > 0 && //check if correct
			 nBackward.size()> 0)
	{
		for (int i=0; i< nForward.size();i++){
			for(int j=0; j< nBackward.size();j++){
				if (queue.forwardQueue.Lookup(nForward[i]).data == queue.backwardQueue.Lookup(nBackward[j]).data){ // if success, see if nodes are the same (return path)
					ExtractFromMiddle(thePath);
					return true;
				}
			}
		}

	}
	*/
	
	else if (nForward.size() > 0 && //check if correct
			 nBackward.size()> 0)
	{
		std::unordered_map<state*,bool> mapData;
		for (int i =0; i< nForward.size(); i++){
			mapData[&(queue.forwardQueue.Lookup(nForward[i]).data)] = true;
		}
		for (int j =0; j< nBackward.size(); j++){
			if (mapData.find(&(queue.backwardQueue.Lookup(nBackward[j]).data)) != mapData.end()){
				ExtractFromMiddle(thePath);
				//printf("here3");
				return true;
			}
		}
		
	}
	struct compareBackward {
		compareBackward(dataStructure currQueue) : queue(currQueue) {}
		bool operator () (uint64_t i, uint64_t j) { return (queue.backwardQueue.Lookup(i).h<queue.backwardQueue.Lookup(j).h); }
		dataStructure queue;
	};
	struct compareForward {
		compareForward(dataStructure currQueue) : queue(currQueue) {}
		bool operator () (uint64_t i, uint64_t j) { return (queue.forwardQueue.Lookup(i).h<queue.forwardQueue.Lookup(j).h); }
		dataStructure queue;
	};
	double currentLowerBound = queue.GetLowerBound();
	
	if (nForward.size() == 0){
		/*
		uint64_t node = *(queue.forwardQueue.getNodesMapBegin(kOpenReady)->second.begin());
		if (queue.forwardQueue.Lookup(node).where != kClosed){
			counts[currentLowerBound]++;
			Expand(node, queue.forwardQueue, queue.backwardQueue, forwardHeuristic, goal);
		}
		if (!fless(queue.GetLowerBound(), currentCost)){
				ExtractFromMiddle(thePath);
				return true;
		}
		if (currentLowerBound != queue.GetLowerBound()){
			return false;
		}
		*/
		//std::sort (nBackward.begin(), nBackward.end(),compareBackward(queue));
		for (int j =0; j< ((int)nBackward.size());j++){
			double oldKey = queue.backwardQueue.getFirstKey(kOpenReady);
			if (queue.backwardQueue.Lookup(nBackward[j]).where != kClosed){
					counts[currentLowerBound]++;
					Expand(nBackward[j], queue.backwardQueue, queue.forwardQueue, backwardHeuristic, start);
			}
			if ((!isAllSolutions && !fless(queue.GetLowerBound(), currentCost)) || (isAllSolutions && !flesseq(queue.GetLowerBound(), currentCost))){
					ExtractFromMiddle(thePath);
					//printf("B, LB: %f,g:%f, h:%f\n",queue.GetLowerBound(),queue.backwardQueue.Lookup(nBackward[j]).g,queue.backwardQueue.Lookup(nBackward[j]).h);
					return true;
			}
			if (currentLowerBound != queue.GetLowerBound() || oldKey != queue.backwardQueue.getFirstKey(kOpenReady)){
				return false;
			}
		}
	}
	
	else if (nBackward.size() == 0){
		/*
		uint64_t node = *(queue.backwardQueue.getNodesMapBegin(kOpenReady)->second.begin());
		if (queue.backwardQueue.Lookup(node).where != kClosed){
			counts[currentLowerBound]++;
			Expand(node, queue.backwardQueue, queue.forwardQueue, backwardHeuristic, start);
		}
		if (!fless(queue.GetLowerBound(), currentCost)){
				ExtractFromMiddle(thePath);
				return true;
		}
		if (currentLowerBound != queue.GetLowerBound()){
			return false;
		}
		*/
		//std::sort (nForward.begin(), nForward.end(),compareForward(queue));
		for (int i =0; i< ((int)nForward.size());i++){
			double oldKey = queue.forwardQueue.getFirstKey(kOpenReady);
			if (queue.forwardQueue.Lookup(nForward[i]).where != kClosed){
				counts[currentLowerBound]++;
				Expand(nForward[i], queue.forwardQueue, queue.backwardQueue, forwardHeuristic, goal);
			}
			if ((!isAllSolutions && !fless(queue.GetLowerBound(), currentCost)) || (isAllSolutions && !flesseq(queue.GetLowerBound(), currentCost))){
					ExtractFromMiddle(thePath);
					//printf("F, LB: %f,g:%f, h:%f\n",queue.GetLowerBound(),queue.forwardQueue.Lookup(nForward[i]).g,queue.forwardQueue.Lookup(nForward[i]).h);
					return true;
			}
			if (currentLowerBound != queue.GetLowerBound() || oldKey != queue.forwardQueue.getFirstKey(kOpenReady)){
				return false;
			}
		}
	}
	else{
		//printf("here");
		int i = nForward.size()-1;
		int j = nBackward.size()-1;
		while (i >= 0 || j >=0 ){
			if ((!isAllSolutions && !fless(queue.GetLowerBound(), currentCost)) || (isAllSolutions && !flesseq(queue.GetLowerBound(), currentCost)))
			{
				ExtractFromMiddle(thePath);
				return true;
			}
			bool expandForward;
			if (i < 0){
				expandForward = false;
			}
			else if (j < 0){
				expandForward = true;
			}
			else {
				if (queue.forwardQueue.Lookup(nForward[i]).g >= queue.backwardQueue.Lookup(nBackward[j]).g){
					expandForward = true;
				}
				else{
					expandForward = false;
				}
			}
			if (expandForward){
				if (queue.forwardQueue.Lookup(nForward[i]).where != kClosed){
					counts[currentLowerBound]++;
					Expand(nForward[i], queue.forwardQueue, queue.backwardQueue, forwardHeuristic, goal);
				}
				i--;
			}
			else{
				if (queue.backwardQueue.Lookup(nBackward[j]).where != kClosed){
					counts[currentLowerBound]++;
					Expand(nBackward[j], queue.backwardQueue, queue.forwardQueue, backwardHeuristic, start);
				}
				j--;
			}
			if (currentLowerBound != queue.GetLowerBound()){
				return false;
			}
		}
	}
	/*
	uint64_t i = 0;
	uint64_t j = 0;
	while (i < nForward.size() || j < nBackward.size() ){
		if (!fless(queue.GetLowerBound(), currentCost))
		{
			ExtractFromMiddle(thePath);
			printf("here4,i:%d,j:%d,fs:%d,bs:%d\n",i,j,nForward.size(),nBackward.size());
			return true;
		}
		bool expandForward;
		if (i >= nForward.size()){
			expandForward = false;
		}
		else if (j >= nBackward.size()){
			expandForward = true;
		}
		else {
			if (queue.forwardQueue.Lookup(nForward[i]).g <= queue.backwardQueue.Lookup(nBackward[j]).g){
				expandForward = true;
			}
			else{
				expandForward = false;
			}
		}
		if (expandForward){
			if (queue.forwardQueue.Lookup(nForward[i]).where != kClosed){
				counts[currentLowerBound] = counts[currentLowerBound]+1;
				Expand(nForward[i], queue.forwardQueue, queue.backwardQueue, forwardHeuristic, goal);
			}
			i++;
		}
		else{
			if (queue.backwardQueue.Lookup(nBackward[j]).where != kClosed){
				counts[currentLowerBound] = counts[currentLowerBound]+1;
				Expand(nBackward[j], queue.backwardQueue, queue.forwardQueue, backwardHeuristic, start);
			}
			j++;
		}
		if (currentLowerBound != queue.GetLowerBound()){
			return false;
		}
		
	}
	*/
	/*
	for(int i = 0; i< nForward.size(); i++){
		counts[currentLowerBound]++;
		Expand(nForward[i], queue.forwardQueue, queue.backwardQueue, forwardHeuristic, goal);
		if (!fless(currentLowerBound, currentCost))
		{
			ExtractFromMiddle(thePath);
			return true;
		}
	}
	
	for(int i = 0; i< nBackward.size(); i++){
		counts[currentLowerBound]++;
		Expand(nBackward[i], queue.backwardQueue, queue.forwardQueue, backwardHeuristic, start);
		if (!fless(currentLowerBound, currentCost))
		{
			ExtractFromMiddle(thePath);
			return true;
		}
	}
	*/
	return false;
}
	/*
	else if (!fless(queue.GetLowerBound(), currentCost))
	{
		ExtractFromMiddle(thePath);
		return true;
	}
		
		for(int i = 0; i< nBackward.size(); i++){
		counts[currentLowerBound]++;
		Expand(nBackward[i], queue.backwardQueue, queue.forwardQueue, backwardHeuristic, start);
		if (!fless(queue.GetLowerBound(), currentCost))
		{
			ExtractFromMiddle(thePath);
			return true;
		}
	}
	return false;

	
	/*
	counts[queue.GetLowerBound()]+=nForward.size()+nBackward.size();
	for(int i = 0; i< nForward.size(); i++){
		Expand(nForward[i], queue.forwardQueue, queue.backwardQueue, forwardHeuristic, goal);
	}
	for(int i = 0; i< nBackward.size(); i++){
		Expand(nBackward[i], queue.backwardQueue, queue.forwardQueue, backwardHeuristic, start);
	}
	
	return false;
	*/


template <class state, class action, class environment, class dataStructure, class priorityQueue>
double CBBS<state, action, environment, dataStructure, priorityQueue>::ExtractCostFromMiddle()
{
	double cost = 0;
	printf("cost1: %d",cost);
	cost += ExtractCostToGoal(middleNode);
	printf("cost2: %d",cost);
	cost += ExtractCostToStart(middleNode);
	printf("cost3: %d",cost);
	return cost;
}

template <class state, class action, class environment, class dataStructure, class priorityQueue>
void CBBS<state, action, environment, dataStructure, priorityQueue>::ExtractFromMiddle(std::vector<state> &thePath)
{
	
	std::vector<state> pFor, pBack;
	ExtractPathToGoal(middleNode, pBack);
	ExtractPathToStart(middleNode, pFor);
	reverse(pFor.begin(), pFor.end());
	thePath = pFor;
	thePath.insert( thePath.end(), pBack.begin()+1, pBack.end() );
}

template <class state, class action, class environment, class dataStructure, class priorityQueue>
bool CBBS<state, action, environment, dataStructure, priorityQueue>::DoSingleSearchStep(std::vector<state> &thePath)
{
	return ExpandAVertexCover(thePath);
}


template <class state, class action, class environment, class dataStructure, class priorityQueue>
void CBBS<state, action, environment, dataStructure, priorityQueue>::Expand(uint64_t nextID,
																			priorityQueue &current,
																			priorityQueue &opposite,
																			Heuristic<state> *heuristic, const state &target)
{
	if (current.Lookup(nextID).where == kClosed){
		return;
	}

	uint64_t tmp = current.CloseAtIndex(nextID);
	assert(tmp == nextID);

	//this can happen when we expand a single node instead of a pair
	if ( (!isAllSolutions && fgreatereq(current.Lookup(nextID).g + current.Lookup(nextID).h, currentCost)) || (isAllSolutions && fgreater(current.Lookup(nextID).g + current.Lookup(nextID).h, currentCost))){
		return;
	}
		
	
	nodesExpanded++;
	env->GetSuccessors(current.Lookup(nextID).data, neighbors);
	for (auto &succ : neighbors)
	{
		nodesTouched++;
		uint64_t childID;
		auto loc = current.Lookup(env->GetStateHash(succ), childID);

		// screening
		double edgeCost = env->GCost(current.Lookup(nextID).data, succ);
		if ( (!isAllSolutions && fgreatereq(current.Lookup(nextID).g+edgeCost, currentCost)) || (isAllSolutions && fgreater(current.Lookup(nextID).g+edgeCost, currentCost))){
			continue;
		}

		switch (loc)
		{
			case kClosed: // ignore
				break;
			case kOpenReady: // update cost if needed
			case kOpenWaiting:
			{
				if (fless(current.Lookup(nextID).g+edgeCost, current.Lookup(childID).g))
				{
					double oldGCost = current.Lookup(childID).g;
					current.Lookup(childID).parentID = nextID;
					current.Lookup(childID).g = current.Lookup(nextID).g+edgeCost;
					if (loc == kOpenWaiting){
						current.KeyChanged(childID,oldGCost+current.Lookup(childID).h);
					}
					else{
						current.KeyChanged(childID,oldGCost);
					}

					// TODO: check if we improved the current solution?
					uint64_t reverseLoc;
					auto loc = opposite.Lookup(env->GetStateHash(succ), reverseLoc);
					if (loc == kOpenReady || loc == kOpenWaiting)
					{
						if (fless(current.Lookup(nextID).g+edgeCost + opposite.Lookup(reverseLoc).g, currentCost))
						{
							// TODO: store current solution
//							printf("NBS Potential updated solution found, cost: %1.2f + %1.2f = %1.2f (%llu nodes)\n",
//								   current.Lookup(nextID).g+edgeCost,
//								   opposite.Lookup(reverseLoc).g,
//								   current.Lookup(nextID).g+edgeCost+opposite.Lookup(reverseLoc).g,
//								nodesExpanded);
							currentCost = current.Lookup(nextID).g+edgeCost + opposite.Lookup(reverseLoc).g;
							expansionsUntilSolution = nodesExpanded;
							middleNode = succ;
						}
					}
					else if (loc == kClosed)
					{
						current.Remove(childID);
					}
				}
			}
				break;
			case kUnseen:
			{
				uint64_t reverseLoc;
				auto loc = opposite.Lookup(env->GetStateHash(succ), reverseLoc);
				if (loc == kClosed)// then
				{
					break;			//do nothing. do not put this node to open
				}
				else//loc == kUnseen
				{
					//double edgeCost = env->GCost(current.Lookup(nextID).data, succ);
					//if(fless(current.Lookup(nextID).g + edgeCost + heuristic->HCost(succ, target),currentPr))
					//	current.AddOpenNode(succ,
					//		env->GetStateHash(succ),
					//		current.Lookup(nextID).g + edgeCost,
					//		heuristic->HCost(succ, target),
					//		nextID,0);
					//else
					double newNodeF = current.Lookup(nextID).g + edgeCost + heuristic->HCost(succ, target);
					if ((!isAllSolutions && fless(newNodeF , currentCost)) || (isAllSolutions && flesseq(newNodeF , currentCost)))
					{
						if ((!isAllSolutions && fless(newNodeF , queue.GetLowerBound())) || (isAllSolutions && flesseq(newNodeF , queue.GetLowerBound())))
							current.AddOpenNode(succ,
												env->GetStateHash(succ),
												current.Lookup(nextID).g + edgeCost,
												heuristic->HCost(succ, target),
												nextID, kOpenReady);
						else
							current.AddOpenNode(succ,
												env->GetStateHash(succ),
												current.Lookup(nextID).g + edgeCost,
												heuristic->HCost(succ, target),
												nextID, kOpenWaiting);
					}
					if (loc == kOpenReady || loc == kOpenWaiting)
					{
						//double edgeCost = env->GCost(current.Lookup(nextID).data, succ);
						if (fless(current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g, currentCost))
						{
							// TODO: store current solution
//							printf("NBS Potential solution found, cost: %1.2f + %1.2f = %1.2f (%llu nodes)\n",
//								current.Lookup(nextID).g + edgeCost,
//								opposite.Lookup(reverseLoc).g,
//								current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g,
//								nodesExpanded);
							currentCost = current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g;
							expansionsUntilSolution = nodesExpanded;
							
							middleNode = succ;
						}
						
					}
				}
				
			}
				break;
		}
	}
}


template <class state, class action, class environment, class dataStructure, class priorityQueue>
uint64_t CBBS<state, action, environment, dataStructure, priorityQueue>::GetDoubleExpansions() const
{
	uint64_t doubles = 0;
	for (unsigned int x = 0; x < queue.forwardQueue.size(); x++)
	{
		uint64_t key;
		const auto &data = queue.forwardQueue.Lookat(x);
		if (data.where == kClosed)
		{
			auto loc = queue.backwardQueue.Lookup(env->GetStateHash(data.data), key);
			if (loc == kClosed)
				doubles++;
		}

	}
	return doubles;
}

template <class state, class action, class environment, class dataStructure, class priorityQueue>
void CBBS<state, action, environment, dataStructure, priorityQueue>::OpenGLDraw() const
{
	OpenGLDraw(queue.forwardQueue);
	OpenGLDraw(queue.backwardQueue);
}

template <class state, class action, class environment, class dataStructure, class priorityQueue>
void CBBS<state, action, environment, dataStructure, priorityQueue>::OpenGLDraw(const priorityQueue &queue) const
{
}

#endif /* CBBS_h */