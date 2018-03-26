//
//  NBSQueue.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/10/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#ifndef NBSQueue_h
#define NBSQueue_h

#include "BDOpenClosed.h"
#include <vector>
#include <climits>
#include <unordered_map>
#include <utility>
#include <algorithm>


//low g -> low f
template <class state>
struct NBSCompareOpenReady {
	bool operator()(const BDOpenClosedData<state> &i1, const BDOpenClosedData<state> &i2) const
	{
		double f1 = i1.g + i1.h;
		double f2 = i2.g + i2.h;
		
		if (fequal(i1.g, i2.g))
		{
			return (!fless(f1, f2));
		}
		return (fgreater(i1.g, i2.g)); // low g over high
	}
};

template <class state>
struct NBSCompareOpenWaiting {
	bool operator()(const BDOpenClosedData<state> &i1, const BDOpenClosedData<state> &i2) const
	{
		double f1 = i1.g + i1.h;
		double f2 = i2.g + i2.h;
		
		if (fequal(f1, f2))
		{
			return (!fgreater(i1.g, i2.g));
		}
		return (fgreater(f1, f2)); // low f over high
	}
};

//bool compareTwoKeysForward (uint64_t i,uint64_t j) { return (forwardQueue.Lookup(i).g<forwardQueue.Lookup(j).g); }

//bool compareTwoKeysBackward (uint64_t i,uint64_t j) { return (backwardQueue.Lookup(i).g<backwardQueue.Lookup(j).g); }

template <typename state, int epsilon = 1>
class NBSQueue {
public:
bool getVertexCover(std::vector<uint64_t> &nextForward, std::vector<uint64_t> &nextBackward,int TieBreakingPolicy)
	{

		while (true)
		{
			if (forwardQueue.OpenSize() == 0)
				return false;
			if (backwardQueue.OpenSize() == 0)
				return false;
			
				// move items with f<CLowerBound to ready
				
				while (forwardQueue.OpenWaitingSize() != 0 && (!fgreater(forwardQueue.PeekAt(kOpenWaiting).g+forwardQueue.PeekAt(kOpenWaiting).h, CLowerBound)))
				{
					forwardQueue.PutToReady();
				}
				while (backwardQueue.OpenWaitingSize() != 0 && (!fgreater(backwardQueue.PeekAt(kOpenWaiting).g+backwardQueue.PeekAt(kOpenWaiting).h, CLowerBound)))
				{
					backwardQueue.PutToReady();
				}
				
			if ((forwardQueue.OpenReadySize() != 0) && (backwardQueue.OpenReadySize() != 0) &&
				(!fgreater(forwardQueue.PeekAt(kOpenReady).g + backwardQueue.PeekAt(kOpenReady).g + epsilon, CLowerBound)))
			{
				std::vector<uint64_t> forwardCandidates;
				std::vector<uint64_t> backwardCandidates;
				//uint64_t i = 0;
				//while(forwardQueue.OpenReadySize() >= i+1 && !fgreater(forwardQueue.Lookup(forwardQueue.PeekAtIndex(kOpenReady,i)).g+backwardQueue.PeekAt(kOpenReady).g + epsilon, CLowerBound))
				//{
				//	forwardCandidates.push_back(forwardQueue.Peek(kOpenReady,i));
				//	
				//}
				//uint64_t originalSize = forwardQueue.OpenReadySize();
				
				std::unordered_map<double,std::vector<uint64_t> > forwardMap,backwardMap;
				forwardQueue.getNodesLeqValue(CLowerBound-backwardQueue.PeekAt(kOpenReady).g - epsilon+TOLERANCE,forwardMap);
				backwardQueue.getNodesLeqValue(CLowerBound-forwardQueue.PeekAt(kOpenReady).g - epsilon+TOLERANCE,backwardMap);
				std::vector<std::pair<uint64_t,uint64_t> > forwardCluster;
				std::vector<std::pair<uint64_t,uint64_t> > backwardCluster;
				
				struct compareValues {
					compareValues(BDOpenClosed<state, NBSCompareOpenReady<state>, NBSCompareOpenWaiting<state> >& currQueue) : queue(currQueue) {}
					bool operator () (std::pair<uint64_t,uint64_t> i, std::pair<uint64_t,uint64_t> j) { return (queue.Lookup(i.first).g<queue.Lookup(j.first).g); }

					BDOpenClosed<state, NBSCompareOpenReady<state>, NBSCompareOpenWaiting<state> >&  queue;
				};
				
				for (auto it : forwardMap){
					forwardCluster.push_back(std::make_pair(it.second[0],it.second.size()));
				}
				std::sort (forwardCluster.begin(), forwardCluster.end(),compareValues(forwardQueue));
				for (auto it : backwardMap){
					backwardCluster.push_back(std::make_pair(it.second[0],it.second.size()));
				}
				std::sort (backwardCluster.begin(), backwardCluster.end(),compareValues(backwardQueue));
				/*
				int minJ = INT_MAX;
				int minI = INT_MAX;
				int minValue = INT_MAX;
				uint64_t NumForwardInVC = 0;
				uint64_t NumBackwardInVC = 0;
				int count = 0;
				for (int i = -1; i < ((int)forwardCluster.size()); i++){
					if (i > 0){
						NumForwardInVC += forwardCluster[i].second;
					}
					else{
						NumForwardInVC = 0;
					}
					bool skip = false;
					for (int j = -1; j < ((int)backwardCluster.size()) && !skip; j++){
						if (j > 0){
							NumBackwardInVC += backwardCluster[j].second;
						}
						else{
							NumBackwardInVC = 0;
						}
						if (i == ((int)forwardCluster.size())-1){
							if (NumForwardInVC < minValue) 
							{
								minValue = NumForwardInVC;
								minJ = j;
								minI = i;
								count = 0;
							}
							else if (NumForwardInVC == minValue){
								count++;
							}
							skip = true;
						} 
						else if(j == ((int)backwardCluster.size())-1) {
							if (NumBackwardInVC < minValue) 
							{
								minValue = NumBackwardInVC;
								minJ = j;
								minI = i;
								count = 0;
							}
							else if (NumBackwardInVC == minValue){
								count++;
							}
							skip = true;
						}
						else if(fgreater(backwardQueue.Lookup(backwardCluster[j+1].first).g+forwardQueue.Lookup(forwardCluster[i+1].first).g + epsilon, CLowerBound)){
							if (NumBackwardInVC+NumForwardInVC < minValue ){
								minValue = NumBackwardInVC+NumForwardInVC;
								minJ = j;
								minI = i;
								count = 0;
							}
							else if (NumBackwardInVC+NumForwardInVC == minValue){
								count++;
							}
							skip = true;
						}
					}
				}
				if (count > 0){
					std::vector<uint64_t> v = forwardMap[forwardQueue.Lookup(forwardCluster[0].first).g];
					nextForward.insert( nextForward.end(), v.begin(), v.end() );
					std::vector<uint64_t> v1 = backwardMap[backwardQueue.Lookup(backwardCluster[0].first).g];
					nextBackward.insert( nextBackward.end(), v1.begin(), v1.end() );
				}
				else{
					if (minI > 0){
						std::vector<uint64_t> v = forwardMap[forwardQueue.Lookup(forwardCluster[0].first).g];
						nextForward.insert( nextForward.end(), v.begin(), v.end() );
					}
					else{
						std::vector<uint64_t> v1 = backwardMap[backwardQueue.Lookup(backwardCluster[0].first).g];
						nextBackward.insert( nextBackward.end(), v1.begin(), v1.end() );
					}
				}
				new */
				
				
				int minJ = INT_MAX;
				int minI = INT_MAX;
				int minValue = INT_MAX;
				uint64_t NumForwardInVC = 0;
				uint64_t NumBackwardInVC = 0;
				std::vector<std::pair<int,int> > minimalVertexCovers;
				for (int i = -1; i < ((int)forwardCluster.size()); i++){
					if (i > 0){
						NumForwardInVC += forwardCluster[i].second;
					}
					else{
						NumForwardInVC = 0;
					}
					bool skip = false;
					for (int j = -1; j < ((int)backwardCluster.size()) && !skip; j++){
						if (j > 0){
							NumBackwardInVC += backwardCluster[j].second;
						}
						else{
							NumBackwardInVC = 0;
						}
						if (i == ((int)forwardCluster.size())-1){
							if (NumForwardInVC < minValue){
								minimalVertexCovers.clear();
							}
							if(NumForwardInVC <= minValue){
								minimalVertexCovers.push_back(std::make_pair(i,j));
								minValue = NumForwardInVC;
							}
							skip = true;
						} 
						else if(j == ((int)backwardCluster.size())-1) {
							if (NumBackwardInVC < minValue){
								minimalVertexCovers.clear();
							}
							if(NumBackwardInVC <= minValue){
								minimalVertexCovers.push_back(std::make_pair(i,j));
								minValue = NumBackwardInVC;
							}
							skip = true;
						}
						else if(fgreater(backwardQueue.Lookup(backwardCluster[j+1].first).g+forwardQueue.Lookup(forwardCluster[i+1].first).g + epsilon, CLowerBound)){
							if (NumBackwardInVC+NumForwardInVC < minValue){
								minimalVertexCovers.clear();
							}
							if(NumBackwardInVC+NumForwardInVC <= minValue){
								minimalVertexCovers.push_back(std::make_pair(i,j));
								minValue = NumBackwardInVC+NumForwardInVC;
							}
							skip = true;
						}
					}
				}
				
				std::pair<int,int> chosenVC = computeTieBreaking(minimalVertexCovers,forwardCluster,backwardCluster,TieBreakingPolicy);
				
				for (int i = 0; i <= chosenVC.first;i++){
					std::vector<uint64_t> v = forwardMap[forwardQueue.Lookup(forwardCluster[i].first).g];
					nextForward.insert( nextForward.end(), v.begin(), v.end() );
				}
				for (int j = 0; j <= chosenVC.second;j++){
					std::vector<uint64_t> v = backwardMap[backwardQueue.Lookup(backwardCluster[j].first).g];
					nextBackward.insert( nextBackward.end(), v.begin(), v.end() );
				}
				
				/*

				if (minI >= 0){
					std::vector<uint64_t> v = forwardMap[forwardQueue.Lookup(forwardCluster[0].first).g];
					nextForward.insert( nextForward.end(), v.begin(), v.end() );
				}
				if (minJ >= 0){
					std::vector<uint64_t> v = backwardMap[backwardQueue.Lookup(backwardCluster[0].first).g];
					nextBackward.insert( nextBackward.end(), v.begin(), v.end() );
				}	
				*/
				
				//while(forwardQueue.OpenReadySize() > 0 && !fgreater(forwardQueue.PeekAt(kOpenReady).g+backwardQueue.PeekAt(kOpenReady).g + epsilon, CLowerBound))
				//{
				//	forwardCandidates.push_back(forwardQueue.Peek(kOpenReady));
				//	forwardQueue.removeFirstInReady();
				//}
				//for (int i = 0; i < forwardCandidates.size();i++){
				//	forwardQueue.addAgainToReady(forwardCandidates[i]);
				//}
				//assert(originalSize = forwardQueue.OpenReadySize());
				//originalSize = backwardQueue.OpenReadySize();
				//while(backwardQueue.OpenReadySize() > 0 && !fgreater(forwardQueue.PeekAt(kOpenReady).g+backwardQueue.PeekAt(kOpenReady).g + epsilon, CLowerBound))
				//{
				//	backwardCandidates.push_back(backwardQueue.Peek(kOpenReady));
				//	backwardQueue.removeFirstInReady();
				//}
				//for (int i = 0; i < backwardCandidates.size();i++){
				//	backwardQueue.addAgainToReady(backwardCandidates[i]);
				//}
				//assert(originalSize = backwardQueue.OpenReadySize());
				/*
				std::vector<uint64_t> forwardCluster;
				std::vector<uint64_t> backwardCluster;
				for (auto it : forwardMap){
					forwardCluster.push_back(it.second[0]);
				}
				
				struct compareValues {
					compareValues(BDOpenClosed<state, NBSCompareOpenReady<state>, NBSCompareOpenWaiting<state> >& currQueue) : queue(currQueue) {}
					bool operator () (uint64_t i, uint64_t j) { return (queue.Lookup(i).g<queue.Lookup(j).g); }

					BDOpenClosed<state, NBSCompareOpenReady<state>, NBSCompareOpenWaiting<state> >&  queue;
				};
				std::sort (forwardCluster.begin(), forwardCluster.end(),compareValues(forwardQueue));
				for (auto it : backwardMap){
					backwardCluster.push_back(it.second[0]);
				}
				
				std::sort (backwardCluster.begin(), backwardCluster.end(),compareValues(backwardQueue));
				*/
/* 				if (forwardCandidates.size()>0){
					forwardCluster.push_back(forwardCandidates[0]);
				}
				for (int i = 1; i< forwardCandidates.size();i++){
					if(forwardQueue.Lookup(forwardCandidates[i]).g != forwardQueue.Lookup(forwardCluster[forwardCluster.size()-1]).g){
						forwardCluster.push_back(forwardCandidates[i]);
					}
				}
				
				if (backwardCandidates.size()>0){
					backwardCluster.push_back(backwardCandidates[0]);
				}
				for (int i = 1; i< backwardCandidates.size();i++){
					if(backwardQueue.Lookup(backwardCandidates[i]).g != backwardQueue.Lookup(backwardCluster[backwardCluster.size()-1]).g){
						backwardCluster.push_back(backwardCandidates[i]);
					}
				} */
				//i = 0;
				//while(backwardQueue.OpenReadySize() >= i+1 && !fgreater(backwardQueue.Lookup(backwardQueue.PeekAtIndex(kOpenReady,i)).g+forwardQueue.PeekAt(kOpenReady).g + epsilon, CLowerBound))
				//{
				//	backwardCandidates.push_back(backwardQueue.PeekAtIndex(kOpenReady,i));
				//	i++;
				//}
				/*
				int minJ = INT_MAX;
				int minI = INT_MAX;
				int minValue = INT_MAX;
				for (int i = -1; i < ((int)forwardCluster.size()); i++){
					bool skip = false;
					for (int j = -1; j < ((int)backwardCluster.size()) && !skip; j++){
						if (i == ((int)forwardCluster.size())-1){
							if (forwardCluster.size() < minValue || (forwardCluster.size() == minValue && tieBreakCriteria())){
								minValue = forwardCluster.size();
								minJ = j;
								minI = i;
							}
							skip = true;
						} 
						else if(j == ((int)backwardCluster.size())-1) {
							if (backwardCluster.size() < minValue || (backwardCluster.size() == minValue && tieBreakCriteria())){
								minValue = backwardCluster.size();
								minJ = j;
								minI = i;
							}
							skip = true;
						}
						else if(fgreater(backwardQueue.Lookup(backwardCluster[j+1]).g+forwardQueue.Lookup(forwardCluster[i+1]).g + epsilon, CLowerBound)){
							if (i+j+2 < minValue || (i+j+2 == minValue && tieBreakCriteria())){
								minValue = i+j+2;
								minJ = j;
								minI = i;
							}
							skip = true;
						}
					}
				}
				
				for (int i = 0; i <= minI;i++){
					std::vector<uint64_t> & v = forwardMap[forwardQueue.Lookup(forwardCluster[i]).g];
					nextForward.insert( nextForward.end(), v.begin(), v.end() );
				}
				for (int j = 0; j <= minJ;j++){
					std::vector<uint64_t> & v = backwardMap[backwardQueue.Lookup(backwardCluster[j]).g];
					nextBackward.insert( nextBackward.end(), v.begin(), v.end() );
				}
				*/
				/*for (int i = 0; i < nextForward.size();i++){
					if (forwardQueue.Lookup(nextForward[i]).where == kClosed){
						int x = 5;
					}
				}
				for (int i = 0; i < nextBackward.size();i++){
					if (backwardQueue.Lookup(nextBackward[i]).where == kClosed){
						int x = 5;
					}
				}
				*/
/* 				if (minI >= 0){
					double maxForwardValue = forwardQueue.Lookup(forwardCluster[minI]).g;
					int i = 0;
					while (i < forwardCandidates.size() && forwardQueue.Lookup(forwardCandidates[i]).g <= maxForwardValue){
						nextForward.push_back(forwardCandidates[i]);
						i++;
					}
				}
				
				if (minJ >= 0){
					double maxBackwardValue = backwardQueue.Lookup(backwardCluster[minJ]).g;
					int j = 0;
					while (j < backwardCandidates.size() && backwardQueue.Lookup(backwardCandidates[i]).g <= maxBackwardValue){
						nextBackward.push_back(backwardCandidates[i]);
						j++;
					}
				} */
				//for (int i = 0; i<= minI; i++){
				//	nextForward.push_back(forwardCandidates[i]);
				//}
				//for (int j = 0; j<= minJ; j++){
				//	nextBackward.push_back(backwardCandidates[j]);
				//}
				return true;
			}
			else
			{
				CLowerBound = DBL_MAX;
				if (forwardQueue.OpenWaitingSize() != 0)
				{
					const auto i5 = forwardQueue.PeekAt(kOpenWaiting);
					CLowerBound = std::min(CLowerBound, i5.g+i5.h);
				}
				if (backwardQueue.OpenWaitingSize() != 0)
				{
					const auto i6 = backwardQueue.PeekAt(kOpenWaiting);
					CLowerBound = std::min(CLowerBound, i6.g+i6.h);
				}
				if ((forwardQueue.OpenReadySize() != 0) && (backwardQueue.OpenReadySize() != 0))
					CLowerBound = std::min(CLowerBound, forwardQueue.PeekAt(kOpenReady).g + backwardQueue.PeekAt(kOpenReady).g + epsilon);
			}
			

		}
		return false;
	}


	bool GetNextPair(uint64_t &nextForward, uint64_t &nextBackward)
	{
		// move items with f<CLowerBound to ready
		while (forwardQueue.OpenWaitingSize() != 0 && fless(forwardQueue.PeekAt(kOpenWaiting).g+forwardQueue.PeekAt(kOpenWaiting).h, CLowerBound))
		{
			forwardQueue.PutToReady();
		}
		while (backwardQueue.OpenWaitingSize() != 0 && fless(backwardQueue.PeekAt(kOpenWaiting).g+backwardQueue.PeekAt(kOpenWaiting).h, CLowerBound))
		{
			backwardQueue.PutToReady();
		}

		while (true)
		{
			if (forwardQueue.OpenSize() == 0)
				return false;
			if (backwardQueue.OpenSize() == 0)
				return false;
			if ((forwardQueue.OpenReadySize() != 0) && (backwardQueue.OpenReadySize() != 0) &&
				(!fgreater(forwardQueue.PeekAt(kOpenReady).g + backwardQueue.PeekAt(kOpenReady).g + epsilon, CLowerBound)))
			{
				nextForward = forwardQueue.Peek(kOpenReady);
				nextBackward = backwardQueue.Peek(kOpenReady);
				return true;
			}
			bool changed = false;

			if (backwardQueue.OpenWaitingSize() != 0)
			{
				const auto i4 = backwardQueue.PeekAt(kOpenWaiting);
				if (!fgreater(i4.g+i4.h, CLowerBound))
				{
					changed = true;
					backwardQueue.PutToReady();
				}
			}
			if (forwardQueue.OpenWaitingSize() != 0)
			{
				const auto i3 = forwardQueue.PeekAt(kOpenWaiting);
				if (!fgreater(i3.g+i3.h, CLowerBound))
				{
					changed = true;
					forwardQueue.PutToReady();
				}
			}
			if (!changed)
			{
				CLowerBound = DBL_MAX;
				if (forwardQueue.OpenWaitingSize() != 0)
				{
					const auto i5 = forwardQueue.PeekAt(kOpenWaiting);
					CLowerBound = std::min(CLowerBound, i5.g+i5.h);
				}
				if (backwardQueue.OpenWaitingSize() != 0)
				{
					const auto i6 = backwardQueue.PeekAt(kOpenWaiting);
					CLowerBound = std::min(CLowerBound, i6.g+i6.h);
				}
				if ((forwardQueue.OpenReadySize() != 0) && (backwardQueue.OpenReadySize() != 0))
					CLowerBound = std::min(CLowerBound, forwardQueue.PeekAt(kOpenReady).g + backwardQueue.PeekAt(kOpenReady).g + epsilon);
			}

		}
		return false;
	}
	
	uint64_t getMinimalVertexCover(double CStar)
	{

				std::vector<uint64_t> forwardCandidates;
				std::vector<uint64_t> backwardCandidates;				
				std::unordered_map<double,std::vector<uint64_t> > forwardMap,backwardMap;
				forwardQueue.getClosedNodesLeqCStart(CStar,forwardMap);
				backwardQueue.getClosedNodesLeqCStart(CStar,backwardMap);
				std::vector<std::pair<double,uint64_t> > forwardCluster;
				std::vector<std::pair<double,uint64_t> > backwardCluster;
				
				struct compareValues {
					compareValues(){}
					bool operator () (std::pair<double,uint64_t> i, std::pair<double,uint64_t> j) { return (i.first<j.first); }
				};
				
				for (auto it : forwardMap){
					forwardCluster.push_back(std::make_pair(it.first,it.second.size()));
				}
				std::sort (forwardCluster.begin(), forwardCluster.end(),compareValues());
				for (auto it : backwardMap){
					backwardCluster.push_back(std::make_pair(it.first,it.second.size()));
				}
				std::sort (backwardCluster.begin(), backwardCluster.end(),compareValues());

				
				int minJ = INT_MAX;
				int minI = INT_MAX;
				int minValue = INT_MAX;
				uint64_t NumForwardInVC = 0;
				uint64_t NumBackwardInVC = 0;
				std::vector<std::pair<int,int> > minimalVertexCovers;
				for (int i = -1; i < ((int)forwardCluster.size()); i++){
					if (i > 0){
						NumForwardInVC += forwardCluster[i].second;
					}
					else{
						NumForwardInVC = 0;
					}
					bool skip = false;
					for (int j = -1; j < ((int)backwardCluster.size()) && !skip; j++){
						if (j > 0){
							NumBackwardInVC += backwardCluster[j].second;
						}
						else{
							NumBackwardInVC = 0;
						}
						if (i == ((int)forwardCluster.size())-1){
							if (NumForwardInVC < minValue){
								minimalVertexCovers.clear();
							}
							if(NumForwardInVC <= minValue){
								minimalVertexCovers.push_back(std::make_pair(i,j));
								minValue = NumForwardInVC;
							}
							skip = true;
						} 
						else if(j == ((int)backwardCluster.size())-1) {
							if (NumBackwardInVC < minValue){
								minimalVertexCovers.clear();
							}
							if(NumBackwardInVC <= minValue){
								minimalVertexCovers.push_back(std::make_pair(i,j));
								minValue = NumBackwardInVC;
							}
							skip = true;
						}
						else if(backwardCluster[j+1].first+forwardCluster[i+1].first >= CStar){
							if (NumBackwardInVC+NumForwardInVC < minValue){
								minimalVertexCovers.clear();
							}
							if(NumBackwardInVC+NumForwardInVC <= minValue){
								minimalVertexCovers.push_back(std::make_pair(i,j));
								minValue = NumBackwardInVC+NumForwardInVC;
							}
							skip = true;
						}
					}
				}
				
				std::pair<int,int> chosenVC = minimalVertexCovers[0];
				uint64_t expansions = 0;
				for (int i = 0; i <= chosenVC.first;i++){
					std::vector<uint64_t> v = forwardMap[forwardCluster[i].first];
					expansions+= v.size();
				}
				for (int j = 0; j <= chosenVC.second;j++){
					std::vector<uint64_t> v = backwardMap[backwardCluster[j].first];
					expansions+= v.size();
				}
				return expansions;
	}
	
	void Reset()
	{
		CLowerBound = 0;
		forwardQueue.Reset();
		backwardQueue.Reset();
	}
	double GetLowerBound() { return CLowerBound; }
	BDOpenClosed<state, NBSCompareOpenReady<state>, NBSCompareOpenWaiting<state>> forwardQueue;
	BDOpenClosed<state, NBSCompareOpenReady<state>, NBSCompareOpenWaiting<state>> backwardQueue;
private:
	double CLowerBound;
	bool tieBreakCriteria(int i,int j,int minI,int minJ,std::vector<std::pair<uint64_t,uint64_t> > forwardCluster, std::vector<std::pair<uint64_t,uint64_t> > backwardCluster){
		int iValue = 0;
		int jValue = 0;
		int minIValue = 0;
		int minJValue = 0;
		if (i > 0){
			iValue += forwardQueue.Lookup(forwardCluster[i].first).g;
		}
		if (minI > 0){
			minIValue += forwardQueue.Lookup(forwardCluster[minI].first).g;
		}
		if (j > 0){
			jValue += backwardQueue.Lookup(backwardCluster[j].first).g;
		}
		if (minJ > 0){
			minJValue += backwardQueue.Lookup(backwardCluster[minJ].first).g;
		}
		return (std::max(iValue,jValue) > std::max(minIValue,minJValue));
	}
	bool tieBreakCriteria(double currentSum,double minSum){
		return (currentSum > minSum);
	}
	
		std::pair<int,int> computeTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<uint64_t,uint64_t> >& forwardCluster, std::vector<std::pair<uint64_t,uint64_t> >& backwardCluster, int TieBreakingPolicy){
		switch(TieBreakingPolicy) {
			case 1 : return computeFullMaxGTieBreaking(minimalVertexCovers,forwardCluster,backwardCluster);
			case 2 : return computeSingleClusterMaxGTieBreaking(minimalVertexCovers,forwardCluster,backwardCluster);
			case 3 : return computeSingleClusterMinGTieBreaking(minimalVertexCovers,forwardCluster,backwardCluster);
			case 4 : return computeSingleClusterMinNodesTieBreaking(minimalVertexCovers,forwardCluster,backwardCluster);
			case 5 : return computeSingleClusterMaxNodesTieBreaking(minimalVertexCovers,forwardCluster,backwardCluster);
			case 6 : return computeSingleClusterMinGTieBreakingWithSub(minimalVertexCovers,forwardCluster,backwardCluster);
			case 7 : return computeSingleClusterMaxGTieBreakingWithSub(minimalVertexCovers,forwardCluster,backwardCluster);
			case 8 : return computeFullMinGTieBreaking(minimalVertexCovers,forwardCluster,backwardCluster);
			case 9 : return computeMinGTieBreakingWithSub(minimalVertexCovers,forwardCluster,backwardCluster);
			case 10 : return computeMajorityMaxWithSubTieBreaking(minimalVertexCovers,forwardCluster,backwardCluster);
			case 11 : return computeMajorityMaxTieBreaking(minimalVertexCovers,forwardCluster,backwardCluster);
			case 12 : return computeMajorityMinTieBreaking(minimalVertexCovers,forwardCluster,backwardCluster);
			case 13 : return computeMajorityMinWithSubTieBreaking(minimalVertexCovers,forwardCluster,backwardCluster);
			case 14 : return computeFullMaxGTieBreakingOld(minimalVertexCovers,forwardCluster,backwardCluster);
			case 15 : return computeMajorityMinNodesTieBreaking(minimalVertexCovers,forwardCluster,backwardCluster);
			case 16: return computeSingleClusterMinNodesMaxGFTieBreaking(minimalVertexCovers,forwardCluster,backwardCluster);
			default: assert(false);
					 return std::make_pair(-1,-1);
		}	
		
	}
	
	std::pair<int,int> computeFullMaxGTieBreakingOld(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<uint64_t,uint64_t> >& forwardCluster, std::vector<std::pair<uint64_t,uint64_t> >& backwardCluster){
		
		int maxValue = -1;
		std::pair<int,int> maxPair;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			int maxF = 0;
			int maxB = 0;
			if (it->first > 0){
				maxF = forwardQueue.Lookup(forwardCluster[it->first].first).g;
			}
			if (it->second > 0){
				 maxB = backwardQueue.Lookup(backwardCluster[it->second].first).g;
			}
			if ((maxF > maxValue) || (maxB > maxValue)){
				maxPair = *it;
				maxValue = std::max(maxF,maxB);
			}
		}
		return maxPair;
	}
	
	std::pair<int,int> computeFullMaxGTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<uint64_t,uint64_t> >& forwardCluster, std::vector<std::pair<uint64_t,uint64_t> >& backwardCluster){
		
		int maxValue = -1;
		std::pair<int,int> maxPair;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			int maxF = -1;
			int maxB = -1;
			if (it->first >= 0){
				maxF = forwardQueue.Lookup(forwardCluster[it->first].first).g;
			}
			if (it->second >= 0){
				 maxB = backwardQueue.Lookup(backwardCluster[it->second].first).g;
			}
			if ((maxF > maxValue) || (maxB > maxValue)){
				maxPair = *it;
				maxValue = std::max(maxF,maxB);
			}
		}
		return maxPair;
	}

	std::pair<int,int> computeSingleClusterMaxGTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<uint64_t,uint64_t> >& forwardCluster, std::vector<std::pair<uint64_t,uint64_t> >& backwardCluster){
		int maxF = -1;
		int maxB = -1;
		std::pair<int,int> maxPair;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (maxF > 0 && maxB > 0){
				break;
			}
			if (it->first >= 0){
				maxF = forwardQueue.Lookup(forwardCluster[0].first).g;
			}
			if (it->second >= 0){
				 maxB = backwardQueue.Lookup(backwardCluster[0].first).g;
			}
		}
		if (maxF >= maxB){
			return std::make_pair(0,-1);
		}
		else{
			return std::make_pair(-1,0);
		}
		return maxPair;
	}

	std::pair<int,int> computeSingleClusterMinGTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<uint64_t,uint64_t> >& forwardCluster, std::vector<std::pair<uint64_t,uint64_t> >& backwardCluster){
		int maxF = INT_MAX;
		int maxB = INT_MAX;
		std::pair<int,int> maxPair;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (maxF < INT_MAX && maxB < INT_MAX){
				break;
			}
			if (it->first >= 0){
				maxF = forwardQueue.Lookup(forwardCluster[0].first).g;
			}
			if (it->second >= 0){
				 maxB = backwardQueue.Lookup(backwardCluster[0].first).g;
			}
		}
		if (maxF < maxB){
			return std::make_pair(0,-1);
		}
		else{
			return std::make_pair(-1,0);
		}
		return maxPair;
	}

	std::pair<int,int> computeSingleClusterMinNodesTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<uint64_t,uint64_t> >& forwardCluster, std::vector<std::pair<uint64_t,uint64_t> >& backwardCluster){
		int maxF = INT_MAX;
		int maxB = INT_MAX;
		std::pair<int,int> maxPair;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (maxF < INT_MAX && maxB < INT_MAX){
				break;
			}
			if (it->first >= 0){
				maxF = forwardCluster[0].second;
			}
			if (it->second >= 0){
				 maxB = backwardCluster[0].second;
			}
		}
		if (maxF < maxB){
			return std::make_pair(0,-1);
		}
		else{
			return std::make_pair(-1,0);
		}
		return maxPair;
	}

	std::pair<int,int> computeSingleClusterMinNodesMaxGFTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<uint64_t,uint64_t> >& forwardCluster, std::vector<std::pair<uint64_t,uint64_t> >& backwardCluster){
		int maxF = INT_MAX;
		int maxB = INT_MAX;
		std::pair<int,int> maxPair;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (maxF < INT_MAX && maxB < INT_MAX){
				break;
			}
			if (it->first >= 0){
				maxF = forwardCluster[0].second;
			}
			if (it->second >= 0){
				 maxB = backwardCluster[0].second;
			}
		}
		if (maxF < maxB){
			return std::make_pair(0,-1);
		}
		else if (maxF > maxB){
			return std::make_pair(-1,0);
		}
		else{
			if (forwardCluster[0].first >= backwardCluster[0].first){
				return std::make_pair(0,-1);
			}
			else{
				return std::make_pair(-1,0);
			}
		}
		return maxPair;
	}	
	
	std::pair<int,int> computeMajorityMinNodesTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<uint64_t,uint64_t> >& forwardCluster, std::vector<std::pair<uint64_t,uint64_t> >& backwardCluster){
		
		int forwardCount = 0;
		int backwardCount = 0;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (it->first >= 0){
				forwardCount++;
			}
			if (it->second >= 0){
				backwardCount++;
			}
		}
		if (forwardCount > backwardCount){
			return std::make_pair(0,-1);
		}
		else if (forwardCount < backwardCount){
			return std::make_pair(-1,0);
		}
		else{
			int maxF = forwardCluster[0].second;
			int maxB = backwardCluster[0].second;
			if (maxF < maxB){
				return std::make_pair(0,-1);
			}
			else{
				return std::make_pair(-1,0);
			}
		}
	}
	
	std::pair<int,int> computeSingleClusterMaxNodesTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<uint64_t,uint64_t> >& forwardCluster, std::vector<std::pair<uint64_t,uint64_t> >& backwardCluster){
		int maxF = -1;
		int maxB = -1;
		std::pair<int,int> maxPair;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (maxF > 0 && maxB > 0){
				break;
			}
			if (it->first >= 0){
				maxF = forwardCluster[0].second;
			}
			if (it->second >= 0){
				 maxB = backwardCluster[0].second;
			}
		}
		if (maxF >= maxB){
			return std::make_pair(0,-1);
		}
		else{
			return std::make_pair(-1,0);
		}
		return maxPair;
	}
	
		std::pair<int,int> computeSingleClusterMinGTieBreakingWithSub(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<uint64_t,uint64_t> >& forwardCluster, std::vector<std::pair<uint64_t,uint64_t> >& backwardCluster){
		int maxF = INT_MAX;
		int maxB = INT_MAX;
		int minOpenForward = 0;
		int minOpenBackward = 0;
		if (forwardCluster.size() > 0){
			minOpenForward = forwardQueue.Lookup(forwardCluster[0].first).g;
		}
		if (backwardCluster.size() > 0){
			minOpenBackward = backwardQueue.Lookup(backwardCluster[0].first).g;
		}
		std::pair<int,int> maxPair;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (maxF < INT_MAX && maxB < INT_MAX){
				break;
			}
			if (it->first >= 0){
				maxF = forwardQueue.Lookup(forwardCluster[0].first).g;
			}
			if (it->second >= 0){
				 maxB = backwardQueue.Lookup(backwardCluster[0].first).g;
			}
		}
		if (maxF - minOpenBackward < maxB - minOpenForward){
			return std::make_pair(0,-1);
		}
		else{
			return std::make_pair(-1,0);
		}
		return maxPair;
	}
	
	std::pair<int,int> computeSingleClusterMaxGTieBreakingWithSub(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<uint64_t,uint64_t> >& forwardCluster, std::vector<std::pair<uint64_t,uint64_t> >& backwardCluster){
		int maxF = -1;
		int maxB = -1;
		int minOpenForward = 0;
		int minOpenBackward = 0;
		if (forwardCluster.size() > 0){
			minOpenForward = forwardQueue.Lookup(forwardCluster[0].first).g;
		}
		if (backwardCluster.size() > 0){
			minOpenBackward = backwardQueue.Lookup(backwardCluster[0].first).g;
		}
		std::pair<int,int> maxPair;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (maxF > 0 && maxB > 0){
				break;
			}
			if (it->first >= 0){
				maxF = forwardQueue.Lookup(forwardCluster[0].first).g;
			}
			if (it->second >= 0){
				 maxB = backwardQueue.Lookup(backwardCluster[0].first).g;
			}
		}
		if (maxF - minOpenBackward >= maxB - minOpenForward){
			return std::make_pair(0,-1);
		}
		else{
			return std::make_pair(-1,0);
		}
		return maxPair;
	}
	
	std::pair<int,int> computeFullMinGTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<uint64_t,uint64_t> >& forwardCluster, std::vector<std::pair<uint64_t,uint64_t> >& backwardCluster){
		
		int minValue = INT_MAX;
		std::pair<int,int> minPair;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			int maxF = INT_MAX;
			int maxB = INT_MAX;
			if (it->first >= 0){
				maxF = forwardQueue.Lookup(forwardCluster[it->first].first).g;
			}
			if (it->second >= 0){
				 maxB = backwardQueue.Lookup(backwardCluster[it->second].first).g;
			}
			if ((maxF < minValue) || (maxB < minValue)){
				minPair = *it;
				minValue = std::min(maxF,maxB);
			}
		}
		return minPair;
	}
	
	
	std::pair<int,int> computeMinGTieBreakingWithSub(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<uint64_t,uint64_t> >& forwardCluster, std::vector<std::pair<uint64_t,uint64_t> >& backwardCluster){	
		int minValue = INT_MAX;
		int minOpenForward = 0;
		int minOpenBackward = 0;
		if (forwardCluster.size() > 0){
			minOpenForward = forwardQueue.Lookup(forwardCluster[0].first).g;
		}
		if (backwardCluster.size() > 0){
			minOpenBackward = backwardQueue.Lookup(backwardCluster[0].first).g;
		}
		std::pair<int,int> minPair;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			int maxF = INT_MAX;
			int maxB = INT_MAX;
			if (it->first >= 0){
				maxF = forwardQueue.Lookup(forwardCluster[it->first].first).g;
			}
			if (it->second >= 0){
				 maxB = backwardQueue.Lookup(backwardCluster[it->second].first).g;
			}
			if ((maxF - minOpenBackward < minValue) || (maxB - minOpenForward < minValue)){
				minPair = *it;
				minValue = std::min(maxF- minOpenBackward,maxB - minOpenForward);
			}
		}
		return minPair;
	}
	
	std::pair<int,int> computeMajorityMaxTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<uint64_t,uint64_t> >& forwardCluster, std::vector<std::pair<uint64_t,uint64_t> >& backwardCluster){
		
		int forwardCount = 0;
		int backwardCount = 0;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (it->first >= 0){
				forwardCount++;
			}
			if (it->second >= 0){
				backwardCount++;
			}
		}
		if (forwardCount > backwardCount){
			return std::make_pair(0,-1);
		}
		else if (forwardCount < backwardCount){
			return std::make_pair(-1,0);
		}
		else{
			
			int fVal = forwardQueue.Lookup(forwardCluster[0].first).g;
			int bVal = backwardQueue.Lookup(backwardCluster[0].first).g;
			if (fVal >= bVal){
				return std::make_pair(0,-1);
			}
			else{
				return std::make_pair(-1,0);
			}
		}
	}


	std::pair<int,int> computeMajorityMinTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<uint64_t,uint64_t> >& forwardCluster, std::vector<std::pair<uint64_t,uint64_t> >& backwardCluster){
		
		int forwardCount = 0;
		int backwardCount = 0;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (it->first >= 0){
				forwardCount++;
			}
			if (it->second >= 0){
				backwardCount++;
			}
		}
		if (forwardCount > backwardCount){
			return std::make_pair(0,-1);
		}
		else if (forwardCount < backwardCount){
			return std::make_pair(-1,0);
		}
		else{
			
			int fVal = forwardQueue.Lookup(forwardCluster[0].first).g;
			int bVal = backwardQueue.Lookup(backwardCluster[0].first).g;
			if (fVal < bVal){
				return std::make_pair(0,-1);
			}
			else{
				return std::make_pair(-1,0);
			}
		}
	}	
	
	std::pair<int,int> computeMajorityMaxWithSubTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<uint64_t,uint64_t> >& forwardCluster, std::vector<std::pair<uint64_t,uint64_t> >& backwardCluster){
		
		int forwardCount = 0;
		int backwardCount = 0;
		int minOpenForward = 0;
		int minOpenBackward = 0;
		if (forwardCluster.size() > 0){
			minOpenForward = forwardQueue.Lookup(forwardCluster[0].first).g;
		}
		if (backwardCluster.size() > 0){
			minOpenBackward = backwardQueue.Lookup(backwardCluster[0].first).g;
		}
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (it->first >= 0){
				forwardCount++;
			}
			if (it->second >= 0){
				backwardCount++;
			}
		}
		if (forwardCount > backwardCount){
			return std::make_pair(0,-1);
		}
		else if (forwardCount < backwardCount){
			return std::make_pair(-1,0);
		}
		else{
			
			int fVal = forwardQueue.Lookup(forwardCluster[0].first).g - minOpenBackward;
			int bVal = backwardQueue.Lookup(backwardCluster[0].first).g - minOpenForward;
			if (fVal >= bVal){
				return std::make_pair(0,-1);
			}
			else{
				return std::make_pair(-1,0);
			}
		}
	}
	
	std::pair<int,int> computeMajorityMinWithSubTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<uint64_t,uint64_t> >& forwardCluster, std::vector<std::pair<uint64_t,uint64_t> >& backwardCluster){
		
		int forwardCount = 0;
		int backwardCount = 0;
		int minOpenForward = 0;
		int minOpenBackward = 0;
		if (forwardCluster.size() > 0){
			minOpenForward = forwardQueue.Lookup(forwardCluster[0].first).g;
		}
		if (backwardCluster.size() > 0){
			minOpenBackward = backwardQueue.Lookup(backwardCluster[0].first).g;
		}
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (it->first >= 0){
				forwardCount++;
			}
			if (it->second >= 0){
				backwardCount++;
			}
		}
		if (forwardCount > backwardCount){
			return std::make_pair(0,-1);
		}
		else if (forwardCount < backwardCount){
			return std::make_pair(-1,0);
		}
		else{
			
			int fVal = forwardQueue.Lookup(forwardCluster[0].first).g - minOpenBackward;
			int bVal = backwardQueue.Lookup(backwardCluster[0].first).g - minOpenForward;
			if (fVal < bVal){
				return std::make_pair(0,-1);
			}
			else{
				return std::make_pair(-1,0);
			}
		}
	}
	/*
	std::pair<int,int> computeMajorityTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<uint64_t,uint64_t> >& forwardCluster, std::vector<std::pair<uint64_t,uint64_t> >& backwardCluster){
		
		int forwardCount = 0;
		int backwardCount = 0;
		int minI = INT_MAX;
		int minJ = INT_MAX;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (it->first >= 0){
				forwardCount++;
				minI = std::min(it->first,minI);
			}
			if (it->second >= 0){
				backwardCount++;
				minJ = std::min(it->second,minJ);
			}
		}
		if (forwardCount > backwardCount){
			return std::make_pair(minI,-1);
		}
		else if (forwardCount < backwardCount){
			return std::make_pair(-1,minJ);
		}
		else{
			int fVal = forwardQueue.Lookup(forwardCluster[minI].first).g;
			int bVal = backwardQueue.Lookup(backwardCluster[minJ].first).g;
			if (fVal >= bVal){
				return std::make_pair(minI,-1);
			}
			else{
				return std::make_pair(-1,minJ);
			}
		}
	*/


};

#endif /* NBSQueue_h */
