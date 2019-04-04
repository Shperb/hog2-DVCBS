/*
 *  FMMBDOpenClosed.h
 */

#ifndef FMMBDOpenClosed_H
#define FMMBDOpenClosed_H

#include <cassert>
#include <vector>
#include <ext/hash_map>
#include <stdint.h>
#include "AStarOpenClosed.h"
#include <unordered_map>

//struct AHash64 {
//	size_t operator()(const uint64_t &x) const
//	{ return (size_t)(x); }
//};
 
 
enum FMMStateLocation {
	flocOpen,//priority queue 0, low g -> low f
	flocClosed,
	flocUnseen
};

enum fmmOpenQueueType {
	fmmpriority = 0,//priority queue 0, low g -> low f
  gpriority = 1,
  fpriority = 2
};

//const uint64_t kTBDNoNode = 0xFFFFFFFFFFFFFFFFull;

template<typename state>
class FMMBDOpenClosedData {
public:
	FMMBDOpenClosedData() {}
	FMMBDOpenClosedData(const state &theData, double gCost, double hCost,uint64_t parent, uint64_t priorityLoc,uint64_t gpriorityLoc,uint64_t fpriorityLoc,uint64_t openLoc,std::unordered_map<double,uint64_t> map,  FMMStateLocation location, double fraction = 1) :data(theData), g(gCost), h(hCost),parentID(parent), priorityLocation(priorityLoc),gpriorityLocation(gpriorityLoc),fpriorityLocation(fpriorityLoc),openListLocation(openLoc),indexMap(map), frac(fraction),where(location) { reopened = false; }
	state data;
	double g;
	double h;
	uint64_t parentID;
	uint64_t priorityLocation;
	uint64_t gpriorityLocation;
	uint64_t fpriorityLocation;
  uint64_t openListLocation;
	double frac;
	bool reopened;
  std::unordered_map<double,uint64_t> indexMap;
	FMMStateLocation where;
};

template<typename state, typename CmpKey0, typename CmpKey1,typename CmpKey2,typename CmpKey3,class dataStructure = FMMBDOpenClosedData<state>>
class FMMBDOpenClosed {
public:
	FMMBDOpenClosed();
	~FMMBDOpenClosed();
	void Reset();
	uint64_t AddOpenNode(const state &val, uint64_t hash, double g, double h,uint64_t parent=kTBDNoNode, FMMStateLocation whichQueue = flocOpen);
	uint64_t AddClosedNode(state &val, uint64_t hash, double g, double h, uint64_t parent=kTBDNoNode);
	void KeyChanged(uint64_t objKey);
	void ReOpen(uint64_t objKey);
	void Remove(uint64_t objKey);
	//void IncreaseKey(uint64_t objKey);
	FMMStateLocation Lookup(uint64_t hashKey, uint64_t &objKey) const;
	inline dataStructure &Lookup(uint64_t objKey) { return elements[objKey]; }
	inline const dataStructure &Lookat(uint64_t objKey) const { return elements[objKey]; }
	uint64_t Peek(fmmOpenQueueType whichQueue) const;
	inline dataStructure &PeekAt(fmmOpenQueueType whichQueue);
  
  inline dataStructure &PeekAtG(double g);
  double GetPrioritySum(int epsilon);

	//if exist min pair, return true, else(one queue is empty) return false;
	//if it returns true, left and right should be set to the pair that is supposed to be returned.
	//bool ExtractMinPair(uint64_t& left, uint64_t& right) ;
	uint64_t Close(fmmOpenQueueType whichQueue);
	//void Reopen(uint64_t objKey);

	uint64_t GetOpenItem(unsigned int which, fmmOpenQueueType where){	return priorityQueues[where][which];}
  uint64_t GetOpenItem(unsigned int which){	return openList[which];}
	size_t OpenSize() const { return priorityQueues[fmmpriority].size(); }

	size_t ClosedSize() const { return size()-OpenSize(); }
	size_t size() const { return elements.size(); }
  std::unordered_map<double,std::vector<uint64_t>> priorityMap;

  
  
private:
	bool HeapifyUp(unsigned int index, fmmOpenQueueType whichQueue);
	void HeapifyDown(unsigned int index, fmmOpenQueueType whichQueue);
	bool HeapifyUp(unsigned int index,  std::vector<uint64_t> & queue,double g);
	void HeapifyDown(unsigned int index,  std::vector<uint64_t> & queue,double g);
	//2 queues:
	//priorityQueues[0] is openReady, priorityQueues[1] is openWaiting
	std::vector<std::vector<uint64_t>> priorityQueues;
	//std::vector<uint64_t> readyQueue;
	//std::vector<uint64_t> waitingQueue;

	// storing the element id; looking up with...hash?
	typedef __gnu_cxx::hash_map<uint64_t, uint64_t, AHash64> IndexTable;
	
	IndexTable table;
	//all the elements, open or closed
	std::vector<dataStructure> elements;
  std::vector<uint64_t> openList;
};


template<typename state, typename CmpKey0, typename CmpKey1,typename CmpKey2,typename CmpKey3,class dataStructure>
FMMBDOpenClosed<state, CmpKey0,CmpKey1,CmpKey2,CmpKey3,dataStructure>::FMMBDOpenClosed()
{
	std::vector<uint64_t> queue;
	queue.resize(0);
	priorityQueues.push_back(queue);
  priorityQueues.push_back(queue);
  priorityQueues.push_back(queue);
	//readyQueue.resize(0);
	//waitingQueue.resize(0);
}

template<typename state, typename CmpKey0, typename CmpKey1,typename CmpKey2,typename CmpKey3,class dataStructure>
FMMBDOpenClosed<state, CmpKey0,CmpKey1,CmpKey2,CmpKey3,dataStructure>::~FMMBDOpenClosed()
{
}

/**
 * Remove all objects from queue.
 */
template<typename state, typename CmpKey0, typename CmpKey1,typename CmpKey2,typename CmpKey3,class dataStructure>
void FMMBDOpenClosed<state, CmpKey0,CmpKey1,CmpKey2,CmpKey3,dataStructure>::Reset()
{
	table.clear();
	elements.clear();
	priorityQueues[fmmpriority].resize(0);
  priorityQueues[gpriority].resize(0);
  priorityQueues[fpriority].resize(0);
	//readyQueue.resize(0);
	//waitingQueue.resize(0);
}

/**
 * Add object into open list.
 */
template<typename state, typename CmpKey0, typename CmpKey1,typename CmpKey2,typename CmpKey3,class dataStructure>
uint64_t FMMBDOpenClosed<state, CmpKey0,CmpKey1,CmpKey2,CmpKey3,dataStructure>::AddOpenNode(const state &val, uint64_t hash, double g, double h, uint64_t parent, FMMStateLocation whichQueue)
{
	// should do lookup here...
	if (table.find(hash) != table.end())
	{
		//return -1; // TODO: find correct id and return
		assert(false);
	}
	if (whichQueue == flocOpen)
	{
    std::unordered_map<double,uint64_t> indexMap;
    for (auto it= priorityMap.begin(); it != priorityMap.end();++it){
      it->second.push_back(elements.size());
      indexMap.insert(std::make_pair(it->first,it->second.size()-1));
    }

    elements.push_back(dataStructure(val, g, h, parent,priorityQueues[fmmpriority].size(),priorityQueues[gpriority].size(),priorityQueues[fpriority].size(),openList.size(),indexMap,  flocOpen));
    for (auto it= priorityMap.begin(); it != priorityMap.end();++it){
      if (elements[it->second[it->second.size()-1]].where == flocClosed){
        printf("fuckkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk1111");
      } 
    }
    if (parent == kTBDNoNode)
      elements.back().parentID = elements.size()-1;
    table[hash] = elements.size()-1; // hashing to element list location

    priorityQueues[fmmpriority].push_back(elements.size() - 1);
    HeapifyUp(priorityQueues[fmmpriority].size() - 1,fmmpriority);
   
    priorityQueues[gpriority].push_back(elements.size() - 1);
    HeapifyUp(priorityQueues[gpriority].size() - 1,gpriority);

    priorityQueues[fpriority].push_back(elements.size() - 1);
    HeapifyUp(priorityQueues[fpriority].size() - 1,fpriority);
    
    openList.push_back(elements.size() - 1);
    
    for (auto it= priorityMap.begin(); it != priorityMap.end();++it){
      HeapifyUp(it->second.size() - 1,it->second,it->first);
    }
  }
  else{
    assert(false);
  }
	return elements.size()-1;
}

/**
 * Add object into closed list.
 */
template<typename state, typename CmpKey0, typename CmpKey1,typename CmpKey2,typename CmpKey3,class dataStructure>
uint64_t FMMBDOpenClosed<state, CmpKey0,CmpKey1,CmpKey2,CmpKey3,dataStructure>::AddClosedNode(state &val, uint64_t hash, double g, double h, uint64_t parent)
{
	// should do lookup here...
	assert(table.find(hash) == table.end());
  std::unordered_map<double,uint64_t> indexMap= std::unordered_map<double,uint64_t>();
	elements.push_back(dataStructure(val, g, h, parent, 0,0,indexMap,flocClosed));
	if (parent == kTBDNoNode)
		elements.back().parentID = elements.size()-1;
	table[hash] = elements.size()-1; // hashing to element list location
	return elements.size()-1;
}

/**
 * Indicate that the key for a particular object has changed.
 */
template<typename state, typename CmpKey0, typename CmpKey1,typename CmpKey2,typename CmpKey3,class dataStructure>
void FMMBDOpenClosed<state, CmpKey0,CmpKey1,CmpKey2,CmpKey3,dataStructure>::KeyChanged(uint64_t val)
{
//	EqKey eq;
//	assert(eq(waitingQueue[table[val]], val));
//	//HeapifyUp(val->key);
//	waitingQueue[table[val]] = val;
	if (elements[val].where == flocOpen)
	{
		if (!HeapifyUp(elements[val].priorityLocation, fmmpriority))
			HeapifyDown(elements[val].priorityLocation, fmmpriority);
    if (!HeapifyUp(elements[val].gpriorityLocation, gpriority))
			HeapifyDown(elements[val].gpriorityLocation, gpriority);
    if (!HeapifyUp(elements[val].fpriorityLocation, fpriority))
			HeapifyDown(elements[val].fpriorityLocation, fpriority);
    for (auto it= priorityMap.begin(); it != priorityMap.end();++it){
      if (!HeapifyUp(elements[val].indexMap[it->first],it->second,it->first))
        HeapifyDown(elements[val].indexMap[it->first],it->second,it->first);
    }
	}
}

template<typename state, typename CmpKey0, typename CmpKey1,typename CmpKey2,typename CmpKey3,class dataStructure>
void FMMBDOpenClosed<state, CmpKey0,CmpKey1,CmpKey2,CmpKey3,dataStructure>::ReOpen(uint64_t val)
{
//	EqKey eq;
//	assert(eq(waitingQueue[table[val]], val));
//	//HeapifyUp(val->key);
//	waitingQueue[table[val]] = val;

	if (elements[val].where == flocClosed){
		elements[val].where = flocOpen;	
		priorityQueues[fmmpriority].push_back(val);
		elements[val].priorityLocation = priorityQueues[fmmpriority].size() - 1;
		HeapifyUp(priorityQueues[fmmpriority].size() - 1,fmmpriority);

		priorityQueues[gpriority].push_back(val);
		elements[val].gpriorityLocation = priorityQueues[gpriority].size() - 1;
		HeapifyUp(priorityQueues[gpriority].size() - 1,gpriority);
    
		priorityQueues[fpriority].push_back(val);
		elements[val].fpriorityLocation = priorityQueues[fpriority].size() - 1;
		HeapifyUp(priorityQueues[fpriority].size() - 1,fpriority);
    
		openList.push_back(val);
		elements[val].openListLocation = openList.size() - 1;    
    
    elements[val].reopened = true;
    
    for (auto it= priorityMap.begin(); it != priorityMap.end();++it){
      it->second.push_back(val);
      if (elements[it->second[it->second.size()-1]].where == flocClosed){
        printf("fuckkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk2222");
      } 
      elements[val].indexMap[it->first] = it->second.size()-1;
      HeapifyUp(elements[val].indexMap[it->first],it->second,it->first);
    }
	}

}

template<typename state, typename CmpKey0, typename CmpKey1,typename CmpKey2,typename CmpKey3,class dataStructure>
void FMMBDOpenClosed<state, CmpKey0,CmpKey1,CmpKey2,CmpKey3,dataStructure>::Remove(uint64_t val)
{
	int pindex = elements[val].priorityLocation;
	int gindex = elements[val].gpriorityLocation;
  int findex = elements[val].fpriorityLocation;
	FMMStateLocation whichQueue = elements[val].where;
	if (whichQueue == flocOpen){
		elements[val].where = flocClosed;
		priorityQueues[fmmpriority][pindex] = priorityQueues[fmmpriority][priorityQueues[fmmpriority].size() - 1];
		elements[priorityQueues[fmmpriority][pindex]].priorityLocation = pindex;
		priorityQueues[fmmpriority].pop_back();
    if (pindex < priorityQueues[fmmpriority].size() && !HeapifyUp(pindex, fmmpriority))
			HeapifyDown(pindex, fmmpriority);
		priorityQueues[gpriority][gindex] = priorityQueues[gpriority][priorityQueues[gpriority].size() - 1];
		elements[priorityQueues[gpriority][gindex]].gpriorityLocation = gindex;
		priorityQueues[gpriority].pop_back();
    if (gindex < priorityQueues[gpriority].size() && !HeapifyUp(gindex, gpriority))
			HeapifyDown(gindex, gpriority);    

		priorityQueues[fpriority][findex] = priorityQueues[fpriority][priorityQueues[fpriority].size() - 1];
		elements[priorityQueues[fpriority][findex]].fpriorityLocation = findex;
		priorityQueues[fpriority].pop_back();
    if (findex < priorityQueues[fpriority].size() && !HeapifyUp(findex, fpriority))
			HeapifyDown(findex, fpriority);    
    uint64_t oindex = elements[val].openListLocation;
		openList[oindex] = openList[openList.size() - 1];
		elements[openList[oindex]].openListLocation = oindex;
		openList.pop_back();

    for (auto it= priorityMap.begin(); it != priorityMap.end();++it){
      if (elements[val].indexMap[it->first] != it->second.size() - 1 && elements[it->second[it->second.size() - 1]].where == flocClosed){
        printf("something went wrong");
      }
      uint64_t index = elements[val].indexMap[it->first];
      if (val != priorityMap[it->first][index]){
        printf("noooooooooooooooooooooo");        
      }
      it->second[index] = it->second[it->second.size() - 1];
      elements[it->second[index]].indexMap[it->first] = index;
      it->second.pop_back();
      if (index < it->second.size() && !HeapifyUp(index, it->second,it->first))
        HeapifyDown(index, it->second,it->first);
    }

	}


}

///**
// * Indicate that the key for a particular object has increased.
// */
//template<typename state, typename CmpKey0, typename CmpKey1,typename CmpKey2,typename CmpKey3,class dataStructure>
//void FMMBDOpenClosed<state, CmpKey0,CmpKey1,CmpKey2,CmpKey3,dataStructure>::IncreaseKey(uint64_t val)
//{
////	EqKey eq;
////	assert(eq(waitingQueue[table[val]], val));
////	waitingQueue[table[val]] = val;
//	HeapifyDown(val);
//}

/**
 * Returns location of object as well as object key.
 */

template<typename state, typename CmpKey0, typename CmpKey1,typename CmpKey2,typename CmpKey3,class dataStructure>
FMMStateLocation FMMBDOpenClosed<state, CmpKey0,CmpKey1,CmpKey2,CmpKey3,dataStructure>::Lookup(uint64_t hashKey, uint64_t &objKey) const
{
	typename IndexTable::const_iterator it;
	it = table.find(hashKey);
	if (it != table.end())
	{
		objKey = (*it).second;
		return elements[objKey].where;
	}
	return flocUnseen;
}


/**
 * Peek at the next item to be expanded.
 */
template<typename state, typename CmpKey0, typename CmpKey1,typename CmpKey2,typename CmpKey3,class dataStructure>
uint64_t FMMBDOpenClosed<state, CmpKey0,CmpKey1,CmpKey2,CmpKey3,dataStructure>::Peek(fmmOpenQueueType whichQueue) const
{
	assert(OpenSize() != 0);		
	return priorityQueues[whichQueue][0];
}

/**
 * Peek at the next item to be expanded.
 */
template<typename state, typename CmpKey0, typename CmpKey1,typename CmpKey2,typename CmpKey3,class dataStructure>
inline dataStructure &FMMBDOpenClosed<state, CmpKey0,CmpKey1,CmpKey2,CmpKey3,dataStructure>::PeekAt(fmmOpenQueueType whichQueue)
{
	assert(OpenSize() != 0);
	return elements[priorityQueues[whichQueue][0]];
}

template<typename state, typename CmpKey0, typename CmpKey1,typename CmpKey2,typename CmpKey3,class dataStructure>
inline dataStructure &FMMBDOpenClosed<state, CmpKey0,CmpKey1,CmpKey2,CmpKey3,dataStructure>::PeekAtG(double g){
	assert(OpenSize() != 0);
  if (priorityMap.find(g) == priorityMap.end()){
    std::vector<uint64_t> vec = std::vector<uint64_t>();
    priorityMap.insert(std::make_pair(g,vec));
    for (auto it = openList.begin(); it != openList.end(); ++it){
      if (elements[*it].where == flocClosed){
        printf("fuckkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk");
      }
      priorityMap[g].push_back(*it);
      elements[*it].indexMap[g] = priorityMap[g].size()-1;
      if(!HeapifyUp(priorityMap[g].size() - 1,priorityMap[g],g)){
        HeapifyDown(priorityMap[g].size() - 1,priorityMap[g],g);
      }
    }
  }
	return elements[priorityMap[g][0]];  
}
template<typename state, typename CmpKey0, typename CmpKey1,typename CmpKey2,typename CmpKey3,class dataStructure>
double FMMBDOpenClosed<state, CmpKey0,CmpKey1,CmpKey2,CmpKey3,dataStructure>::GetPrioritySum(int epsilon){
  double sum = 0;
  for (auto it= priorityMap.begin(); it != priorityMap.end();++it){
    if (it->second.size() > 0 ){
      dataStructure element = elements[it->second[0]];
      sum += std::max((element.g+element.h)-(it->first), element.g+epsilon);
    }
  }
  return sum;
}

/**
 * Move the best item to the closed list and return key.
 */
template<typename state, typename CmpKey0, typename CmpKey1,typename CmpKey2,typename CmpKey3,class dataStructure>
uint64_t FMMBDOpenClosed<state, CmpKey0,CmpKey1,CmpKey2,CmpKey3,dataStructure>::Close(fmmOpenQueueType whichQueue)
{
	assert(OpenSize() != 0);
	uint64_t openSize = OpenSize();
	uint64_t ans = priorityQueues[whichQueue][0];
	Remove(ans);
	/*
	elements[ans].where = flocClosed;
	
	priorityQueues[fpriority][elements[ans].fOpenLocation] = priorityQueues[fpriority][openSize-1];
	uint64_t oldLocationF = elements[ans].fOpenLocation;
	elements[priorityQueues[fpriority][elements[ans].fOpenLocation]].fOpenLocation = oldLocationF;
	elements[ans].fOpenLocation = -1;
	priorityQueues[fpriority].pop_back();
	
	HeapifyDown(oldLocationF,fpriority);
	
	priorityQueues[gpriority][elements[ans].gOpenLocation] = priorityQueues[gpriority][openSize-1];
	uint64_t oldLocationG = elements[ans].gOpenLocation;
	elements[priorityQueues[gpriority][elements[ans].gOpenLocation]].gOpenLocation = oldLocationG;
	elements[ans].gOpenLocation = -1;
	priorityQueues[gpriority].pop_back();
	
	HeapifyDown(oldLocationG,gpriority);
	
	priorityQueues[priorityfMax][elements[ans].fMaxOpenLocation] = priorityQueues[priorityfMax][openSize-1];
	uint64_t oldLocationFMax = elements[ans].fMaxOpenLocation;
	elements[priorityQueues[priorityfMax][elements[ans].fMaxOpenLocation]].fMaxOpenLocation = oldLocationFMax;
	elements[ans].fMaxOpenLocation = -1;
	priorityQueues[priorityfMax].pop_back();
	
	HeapifyDown(oldLocationFMax,priorityfMax);
	
	priorityQueues[prioritygMax][elements[ans].gMaxOpenLocation] = priorityQueues[prioritygMax][openSize-1];
	uint64_t oldLocationGMax = elements[ans].gMaxOpenLocation;
	elements[priorityQueues[prioritygMax][elements[ans].gMaxOpenLocation]].gMaxOpenLocation = oldLocationGMax;
	elements[ans].gMaxOpenLocation = -1;
	priorityQueues[prioritygMax].pop_back();
	
	HeapifyDown(oldLocationGMax,prioritygMax);
	*/
	return ans;
}


/**
 * Moves a node up the heap. Returns true if the node was moved, false otherwise.
 */
template<typename state, typename CmpKey0, typename CmpKey1,typename CmpKey2,typename CmpKey3,class dataStructure>
bool FMMBDOpenClosed<state, CmpKey0,CmpKey1,CmpKey2,CmpKey3,dataStructure>::HeapifyUp(unsigned int index, fmmOpenQueueType whichQueue)
{
	if (index == 0) return false;
	int parent = (index-1)/2;

	if (whichQueue == fmmpriority)
	{
		CmpKey0 compare;
		if (compare(elements[priorityQueues[whichQueue][parent]], elements[priorityQueues[whichQueue][index]]))
		{
			unsigned int tmp = priorityQueues[whichQueue][parent];
			priorityQueues[whichQueue][parent] = priorityQueues[whichQueue][index];
			priorityQueues[whichQueue][index] = tmp;
			elements[priorityQueues[whichQueue][parent]].priorityLocation = parent;
			elements[priorityQueues[whichQueue][index]].priorityLocation = index;
			HeapifyUp(parent, whichQueue);
			return true;
		}
	}
	else if (whichQueue == gpriority)
	{
		CmpKey1 compare;
		if (compare(elements[priorityQueues[whichQueue][parent]], elements[priorityQueues[whichQueue][index]]))
		{
			unsigned int tmp = priorityQueues[whichQueue][parent];
			priorityQueues[whichQueue][parent] = priorityQueues[whichQueue][index];
			priorityQueues[whichQueue][index] = tmp;
			elements[priorityQueues[whichQueue][parent]].gpriorityLocation = parent;
			elements[priorityQueues[whichQueue][index]].gpriorityLocation = index;
			HeapifyUp(parent, whichQueue);
			return true;
		}
	}
  
	else if (whichQueue == fpriority)
	{
		CmpKey2 compare;
		if (compare(elements[priorityQueues[whichQueue][parent]], elements[priorityQueues[whichQueue][index]]))
		{
			unsigned int tmp = priorityQueues[whichQueue][parent];
			priorityQueues[whichQueue][parent] = priorityQueues[whichQueue][index];
			priorityQueues[whichQueue][index] = tmp;
			elements[priorityQueues[whichQueue][parent]].fpriorityLocation = parent;
			elements[priorityQueues[whichQueue][index]].fpriorityLocation = index;
			HeapifyUp(parent, whichQueue);
			return true;
		}
	} 

 
	return false;
}

template<typename state, typename CmpKey0, typename CmpKey1,typename CmpKey2,typename CmpKey3,class dataStructure>
void FMMBDOpenClosed<state, CmpKey0,CmpKey1,CmpKey2,CmpKey3,dataStructure>::HeapifyDown(unsigned int index, fmmOpenQueueType whichQueue)
{
	
	unsigned int child1 = index*2+1;
	unsigned int child2 = index*2+2;



	if (whichQueue == fmmpriority)
	{
		CmpKey0 compare;
		int which;
		unsigned int count = priorityQueues[whichQueue].size();
		// find smallest child
		if (child1 >= count) // no children; done
			return;
		else if (child2 >= count) // one child - compare there
			which = child1;
		// find larger child to move up
		else if (!(compare(elements[priorityQueues[whichQueue][child1]], elements[priorityQueues[whichQueue][child2]])))
			which = child1;
		else
			which = child2;

		if (!(compare(elements[priorityQueues[whichQueue][which]], elements[priorityQueues[whichQueue][index]])))
		{
			unsigned int tmp = priorityQueues[whichQueue][which];
			priorityQueues[whichQueue][which] = priorityQueues[whichQueue][index];
			priorityQueues[whichQueue][index] = tmp;

			elements[priorityQueues[whichQueue][which]].priorityLocation = which;
			elements[priorityQueues[whichQueue][index]].priorityLocation = index;
			HeapifyDown(which, whichQueue);

		}
	}
  else if (whichQueue == gpriority)
	{
		CmpKey1 compare;
		int which;
		unsigned int count = priorityQueues[whichQueue].size();
		// find smallest child
		if (child1 >= count) // no children; done
			return;
		else if (child2 >= count) // one child - compare there
			which = child1;
		// find larger child to move up
		else if (!(compare(elements[priorityQueues[whichQueue][child1]], elements[priorityQueues[whichQueue][child2]])))
			which = child1;
		else
			which = child2;

		if (!(compare(elements[priorityQueues[whichQueue][which]], elements[priorityQueues[whichQueue][index]])))
		{
			unsigned int tmp = priorityQueues[whichQueue][which];
			priorityQueues[whichQueue][which] = priorityQueues[whichQueue][index];
			priorityQueues[whichQueue][index] = tmp;

			elements[priorityQueues[whichQueue][which]].gpriorityLocation = which;
			elements[priorityQueues[whichQueue][index]].gpriorityLocation = index;
			HeapifyDown(which, whichQueue);

		}
	}
  
else if (whichQueue == fpriority)
	{
		CmpKey2 compare;
		int which;
		unsigned int count = priorityQueues[whichQueue].size();
		// find smallest child
		if (child1 >= count) // no children; done
			return;
		else if (child2 >= count) // one child - compare there
			which = child1;
		// find larger child to move up
		else if (!(compare(elements[priorityQueues[whichQueue][child1]], elements[priorityQueues[whichQueue][child2]])))
			which = child1;
		else
			which = child2;

		if (!(compare(elements[priorityQueues[whichQueue][which]], elements[priorityQueues[whichQueue][index]])))
		{
			unsigned int tmp = priorityQueues[whichQueue][which];
			priorityQueues[whichQueue][which] = priorityQueues[whichQueue][index];
			priorityQueues[whichQueue][index] = tmp;

			elements[priorityQueues[whichQueue][which]].fpriorityLocation = which;
			elements[priorityQueues[whichQueue][index]].fpriorityLocation = index;
			HeapifyDown(which, whichQueue);

		}
	}
}
  
template<typename state, typename CmpKey0, typename CmpKey1,typename CmpKey2,typename CmpKey3,class dataStructure>
bool FMMBDOpenClosed<state, CmpKey0,CmpKey1,CmpKey2,CmpKey3,dataStructure>::HeapifyUp(unsigned int index, std::vector<uint64_t> & queue,double g)
{
	if (index == 0) return false;
  if (index > queue.size() -1){
    printf("wwitb1");
  }
	int parent = (index-1)/2;
  CmpKey3 compare = CmpKey3(g);
  if (compare(elements[queue[parent]], elements[queue[index]]))
  {
    unsigned int tmp = queue[parent];
    queue[parent] = queue[index];
    queue[index] = tmp;
    elements[queue[parent]].indexMap[g] = parent;
    elements[queue[index]].indexMap[g] = index;
    HeapifyUp(parent, queue,g);
    return true;
  }
	

	return false;
}

template<typename state, typename CmpKey0, typename CmpKey1,typename CmpKey2,typename CmpKey3,class dataStructure>
void FMMBDOpenClosed<state, CmpKey0,CmpKey1,CmpKey2,CmpKey3,dataStructure>::HeapifyDown(unsigned int index, std::vector<uint64_t> & queue,double g)
{
	
	unsigned int child1 = index*2+1;
	unsigned int child2 = index*2+2;

    if (index > queue.size() -1){
      printf("wwitb2");
    }
		CmpKey3 compare = CmpKey3(g);
		int which;
		unsigned int count = queue.size();
		// find smallest child
		if (child1 >= count) // no children; done
			return;
		else if (child2 >= count) // one child - compare there
			which = child1;
		// find larger child to move up
		else if (!(compare(elements[queue[child1]], elements[queue[child2]])))
			which = child1;
		else
			which = child2;

		if (!(compare(elements[queue[which]], elements[queue[index]])))
		{
			unsigned int tmp = queue[which];
			queue[which] = queue[index];
			queue[index] = tmp;

			elements[queue[which]].indexMap[g] = which;
			elements[queue[index]].indexMap[g] = index;
			HeapifyDown(which, queue,g);

		}
}


#endif