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

#include "BDOpenClosed.h"
#include "FPUtil.h"
#include <unordered_map>
#include "NBSQueue.h"
#include "NBSQueueGF.h"

#define EPSILON 1

using std::cout;


template <class state, class action, class environment, class dataStructure = NBSQueue<state>,
          class priorityQueue = BDOpenClosed<state, NBSCompareOpenReady<state>, NBSCompareOpenWaiting<state>>>
class CBBS {
public:
	CBBS(int tieBreaking)
	{
		forwardHeuristic = 0; backwardHeuristic = 0; env = 0; ResetNodeCount();
		tieBreakingPolicy = tieBreaking;
	}
	virtual ~CBBS() {}
	void GetPath(environment *env, const state& from, const state& to,
				 Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);
	bool InitializeSearch(environment *env, const state& from, const state& to,
						  Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);
	bool ExpandAVertexCover(std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	
	
	
	
	virtual const char *GetName() { return "CBBS"; }
	
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; counts.clear(); }
	
	inline const int GetNumForwardItems() { return queue.forwardQueue.size(); }
	inline const BDOpenClosedData<state> &GetForwardItem(unsigned int which) { return queue.forwardQueue.Lookat(which); }
	inline const int GetNumBackwardItems() { return queue.backwardQueue.size(); }
	inline const BDOpenClosedData<state> &GetBackwardItem(unsigned int which) { return queue.backwardQueue.Lookat(which); }
	
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
			if (i.first < currentCost)
				necessary+=i.second;
		}
		return necessary;
	}
	double GetSolutionCost() const { return currentCost; }
	
	void OpenGLDraw() const;
	
	//	void SetWeight(double w) {weight = w;}
private:
	void ExtractFromMiddle(std::vector<state> &thePath);
	void ExtractPathToGoal(state &node, std::vector<state> &thePath)
	{ uint64_t theID; queue.backwardQueue.Lookup(env->GetStateHash(node), theID); ExtractPathToGoalFromID(theID, thePath); }
	void ExtractPathToGoalFromID(uint64_t node, std::vector<state> &thePath)
	{
		do {
			thePath.push_back(queue.backwardQueue.Lookup(node).data);
			node = queue.backwardQueue.Lookup(node).parentID;
		} while (queue.backwardQueue.Lookup(node).parentID != node);
		thePath.push_back(queue.backwardQueue.Lookup(node).data);
	}
	
	void ExtractPathToStart(state &node, std::vector<state> &thePath)
	{ uint64_t theID; queue.forwardQueue.Lookup(env->GetStateHash(node), theID); ExtractPathToStartFromID(theID, thePath); }
	void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath)
	{
		do {
			thePath.push_back(queue.forwardQueue.Lookup(node).data);
			node = queue.forwardQueue.Lookup(node).parentID;
		} while (queue.forwardQueue.Lookup(node).parentID != node);
		thePath.push_back(queue.forwardQueue.Lookup(node).data);
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
	double currentSolutionEstimate;
	std::vector<state> neighbors;
	environment *env;
	std::unordered_map<double, int> counts;
	
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
	currentCost = DBL_MAX;
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
			return true;
		}
		ExtractFromMiddle(thePath);
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
				return true;
			}
		}
		
	}
	double currentLowerBound = queue.GetLowerBound();
	int i = nForward.size()-1;
	int j = nBackward.size()-1;
	while (i >= 0 || j >=0 ){
		if (!fless(queue.GetLowerBound(), currentCost))
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
	/*
	double currentLowerBound = queue.GetLowerBound();
	uint64_t i = 0;
	uint64_t j = 0;
	while (i < nForward.size() || j < nBackward.size() ){
		if (!fless(queue.GetLowerBound(), currentCost))
		{
			ExtractFromMiddle(thePath);
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
				counts[currentLowerBound]++;
				Expand(nForward[i], queue.forwardQueue, queue.backwardQueue, forwardHeuristic, goal);
			}
			i++;
		}
		else{
			if (queue.backwardQueue.Lookup(nBackward[j]).where != kClosed){
				counts[currentLowerBound]++;
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
	//	uint64_t nextID = current.Peek(kOpenReady);
	//
	uint64_t tmp = current.CloseAtIndex(nextID);
	//uint64_t tmp = current.Close();
	assert(tmp == nextID);

	//this can happen when we expand a single node instead of a pair
	if (fgreatereq(current.Lookup(nextID).g + current.Lookup(nextID).h, currentCost))
		return;
	
	nodesExpanded++;
	env->GetSuccessors(current.Lookup(nextID).data, neighbors);
	for (auto &succ : neighbors)
	{
		nodesTouched++;
		uint64_t childID;
		auto loc = current.Lookup(env->GetStateHash(succ), childID);

		// screening
		double edgeCost = env->GCost(current.Lookup(nextID).data, succ);
		if (fgreatereq(current.Lookup(nextID).g+edgeCost, currentCost))
			continue;

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
					current.KeyChanged(childID);
					
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
					if (fless(newNodeF , currentCost))
					{
						if (fless(newNodeF, queue.GetLowerBound()))
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
	double transparency = 0.9;
	if (queue.size() == 0)
		return;
	uint64_t top = -1;
	//	double minf = 1e9, maxf = 0;
	if (queue.OpenReadySize() > 0)
	{
		top = queue.Peek(kOpenReady);
	}
	for (unsigned int x = 0; x < queue.size(); x++)
	{
		const auto &data = queue.Lookat(x);
		if (x == top)
		{
			env->SetColor(1.0, 1.0, 0.0, transparency);
			env->OpenGLDraw(data.data);
		}
		if (data.where == kOpenWaiting)
		{
			env->SetColor(0.0, 0.5, 0.5, transparency);
			env->OpenGLDraw(data.data);
		}
		else if (data.where == kOpenReady)
		{
			env->SetColor(0.0, 1.0, 0.0, transparency);
			env->OpenGLDraw(data.data);
		}
		else if (data.where == kClosed)
		{
			env->SetColor(1.0, 0.0, 0.0, transparency);
			env->OpenGLDraw(data.data);
		}
	}
}

#endif /* CBBS_h */
