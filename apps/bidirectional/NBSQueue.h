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
bool getVertexCover(std::vector<uint64_t> &nextForward, std::vector<uint64_t> &nextBackward)
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
				int minSum = INT_MAX;
				uint64_t NumForwardInVC = 0;
				uint64_t NumBackwardInVC = 0;
				double currentSumI = 0;
				double currentSumJ = 0;
				for (int i = -1; i < ((int)forwardCluster.size()); i++){
					if (i > 0){
						NumForwardInVC += forwardCluster[i].second;
						currentSumI += forwardQueue.Lookup(forwardCluster[i].first).g;
					}
					else{
						NumForwardInVC = 0;
					}
					bool skip = false;
					for (int j = -1; j < ((int)backwardCluster.size()) && !skip; j++){
						if (j > 0){
							NumBackwardInVC += backwardCluster[j].second;
							currentSumJ += backwardQueue.Lookup(backwardCluster[j].first).g;
						}
						else{
							NumBackwardInVC = 0;
							currentSumJ = 0;
						}
						if (i == ((int)forwardCluster.size())-1){
							if (NumForwardInVC < minValue 
							|| (NumForwardInVC == minValue &&
							tieBreakCriteria(i,j,minI,minJ,forwardCluster,backwardCluster)
							//tieBreakCriteria(currentSumJ+currentSumI,minSum)
							)
							){
								minValue = NumForwardInVC;
								minSum = currentSumJ+currentSumI;
								minJ = j;
								minI = i;
							}
							skip = true;
						} 
						else if(j == ((int)backwardCluster.size())-1) {
							if (NumBackwardInVC < minValue 
							|| (NumBackwardInVC == minValue && 
							tieBreakCriteria(i,j,minI,minJ,forwardCluster,backwardCluster)
							//tieBreakCriteria(currentSumJ+currentSumI,minSum)
							)
							){
								minValue = NumBackwardInVC;
								minSum = currentSumJ+currentSumI;
								minJ = j;
								minI = i;
							}
							skip = true;
						}
						else if(fgreater(backwardQueue.Lookup(backwardCluster[j+1].first).g+forwardQueue.Lookup(forwardCluster[i+1].first).g + epsilon, CLowerBound)){
							if (NumBackwardInVC+NumForwardInVC < minValue 
							|| (NumBackwardInVC+NumForwardInVC == minValue && 
							tieBreakCriteria(i,j,minI,minJ,forwardCluster,backwardCluster)
							//tieBreakCriteria(currentSumJ+currentSumI,minSum)
							)
							){
								minValue = NumBackwardInVC+NumForwardInVC;
								minSum = currentSumJ+currentSumI;
								minJ = j;
								minI = i;
							}
							skip = true;
						}
					}
				}
				
				for (int i = 0; i <= minI;i++){
					std::vector<uint64_t> v = forwardMap[forwardQueue.Lookup(forwardCluster[i].first).g];
					nextForward.insert( nextForward.end(), v.begin(), v.end() );
				}
				for (int j = 0; j <= minJ;j++){
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

};

#endif /* NBSQueue_h */
