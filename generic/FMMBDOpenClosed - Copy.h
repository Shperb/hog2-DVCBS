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
	fpriority = 0,//priority queue 0, low g -> low f
	gpriority = 1,
	priorityfMax = 2,
	prioritygMax = 3,
	priority = 4
};

//const uint64_t kTBDNoNode = 0xFFFFFFFFFFFFFFFFull;

template<typename state>
class FMMBDOpenClosedData {
public:
	FMMBDOpenClosedData() {}
	FMMBDOpenClosedData(const state &theData, double gCost, double hCost, double fCost,uint64_t parent, uint64_t gopenLoc,uint64_t fopenLoc, uint64_t gMaxOpenLoc, uint64_t fMaxOpenLoc, uint64_t priorityLoc,  FMMStateLocation location, double fraction = 1)
	:data(theData), g(gCost), h(hCost),f(fCost),parentID(parent), gOpenLocation(gopenLoc),fOpenLocation(fopenLoc), gMaxOpenLocation(gMaxOpenLoc),fMaxOpenLocation(fMaxOpenLoc), priorityLocation(priorityLoc), frac(fraction),where(location) { reopened = false; }
	state data;
	double g;
	double h;
	double f;
	uint64_t parentID;
	uint64_t gOpenLocation;
	uint64_t fOpenLocation;
	uint64_t gMaxOpenLocation;
	uint64_t fMaxOpenLocation;
	uint64_t priorityLocation;
	double frac;
	bool reopened;
	FMMStateLocation where;
};

template<typename state, typename CmpKey0, typename CmpKey1, typename CmpKey2,typename CmpKey3, typename CmpKey4, class dataStructure = FMMBDOpenClosedData<state> >
class FMMBDOpenClosed {
public:
	FMMBDOpenClosed();
	~FMMBDOpenClosed();
	void Reset();
	uint64_t AddOpenNode(const state &val, uint64_t hash, double g, double h, double f,uint64_t parent=kTBDNoNode, FMMStateLocation whichQueue = flocOpen);
	uint64_t AddClosedNode(state &val, uint64_t hash, double g, double h, double f, uint64_t parent=kTBDNoNode);
	void KeyChanged(uint64_t objKey);
	void ReOpen(uint64_t objKey);
	void Remove(uint64_t objKey);
	//void IncreaseKey(uint64_t objKey);
	FMMStateLocation Lookup(uint64_t hashKey, uint64_t &objKey) const;
	inline dataStructure &Lookup(uint64_t objKey) { return elements[objKey]; }
	inline const dataStructure &Lookat(uint64_t objKey) const { return elements[objKey]; }
	uint64_t Peek(fmmOpenQueueType whichQueue) const;
	inline dataStructure &PeekAt(fmmOpenQueueType whichQueue);

	//if exist min pair, return true, else(one queue is empty) return false;
	//if it returns true, left and right should be set to the pair that is supposed to be returned.
	//bool ExtractMinPair(uint64_t& left, uint64_t& right) ;
	uint64_t Close(fmmOpenQueueType whichQueue);
	//void Reopen(uint64_t objKey);

	uint64_t GetOpenItem(unsigned int which, fmmOpenQueueType where){	return priorityQueues[where][which];}
	size_t OpenSize() const { return priorityQueues[fpriority].size(); }

	size_t ClosedSize() const { return size()-OpenSize(); }
	size_t size() const { return elements.size(); }

private:
	bool HeapifyUp(unsigned int index, fmmOpenQueueType whichQueue);
	void HeapifyDown(unsigned int index, fmmOpenQueueType whichQueue);

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
};


template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3, typename  CmpKey4, class dataStructure>
FMMBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,CmpKey4, dataStructure>::FMMBDOpenClosed()
{
	std::vector<uint64_t> queue;
	queue.resize(0);
	priorityQueues.push_back(queue);
	priorityQueues.push_back(queue);
	priorityQueues.push_back(queue);
	priorityQueues.push_back(queue);
	priorityQueues.push_back(queue);
	//readyQueue.resize(0);
	//waitingQueue.resize(0);
}

template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3, typename  CmpKey4, class dataStructure>
FMMBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,CmpKey4, dataStructure>::~FMMBDOpenClosed()
{
}

/**
 * Remove all objects from queue.
 */
template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3, typename CmpKey4,   class dataStructure>
void FMMBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,CmpKey4,   dataStructure>::Reset()
{
	table.clear();
	elements.clear();
	priorityQueues[fpriority].resize(0);
	priorityQueues[gpriority].resize(0);
	priorityQueues[priorityfMax].resize(0);
	priorityQueues[prioritygMax].resize(0);
	priorityQueues[priority].resize(0);
	//readyQueue.resize(0);
	//waitingQueue.resize(0);
}

/**
 * Add object into open list.
 */
template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3, typename CmpKey4,   class dataStructure>
uint64_t FMMBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,CmpKey4,   dataStructure>::AddOpenNode(const state &val, uint64_t hash, double g, double h,double f, uint64_t parent, FMMStateLocation whichQueue)
{
	// should do lookup here...
	if (table.find(hash) != table.end())
	{
		//return -1; // TODO: find correct id and return
		assert(false);
	}
	if (whichQueue == flocOpen)
	{
		elements.push_back(dataStructure(val, g, h,f, parent, priorityQueues[gpriority].size(),priorityQueues[fpriority].size() , priorityQueues[prioritygMax].size(),priorityQueues[priorityfMax].size(),priorityQueues[priority].size(),  flocOpen));
		
	}
	
	if (parent == kTBDNoNode)
		elements.back().parentID = elements.size()-1;
	table[hash] = elements.size()-1; // hashing to element list location

	priorityQueues[fpriority].push_back(elements.size() - 1);
	HeapifyUp(priorityQueues[fpriority].size() - 1,fpriority);
	
	priorityQueues[gpriority].push_back(elements.size() - 1);
	HeapifyUp(priorityQueues[gpriority].size() - 1,gpriority);
	
	priorityQueues[priorityfMax].push_back(elements.size() - 1);
	HeapifyUp(priorityQueues[priorityfMax].size() - 1,priorityfMax);
	
	priorityQueues[prioritygMax].push_back(elements.size() - 1);
	HeapifyUp(priorityQueues[prioritygMax].size() - 1,prioritygMax);
	
	priorityQueues[priority].push_back(elements.size() - 1);
	HeapifyUp(priorityQueues[priority].size() - 1,priority);

	return elements.size()-1;
}

/**
 * Add object into closed list.
 */
template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3, typename CmpKey4,   class dataStructure>
uint64_t FMMBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,CmpKey4,   dataStructure>::AddClosedNode(state &val, uint64_t hash, double g, double h, double f, uint64_t parent)
{
	// should do lookup here...
	assert(table.find(hash) == table.end());
	elements.push_back(dataStructure(val, g, h,f, parent, 0,0, flocClosed));
	if (parent == kTBDNoNode)
		elements.back().parentID = elements.size()-1;
	table[hash] = elements.size()-1; // hashing to element list location
	return elements.size()-1;
}

/**
 * Indicate that the key for a particular object has changed.
 */
template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3, typename CmpKey4,   class dataStructure>
void FMMBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,CmpKey4,   dataStructure>::KeyChanged(uint64_t val)
{
//	EqKey eq;
//	assert(eq(waitingQueue[table[val]], val));
//	//HeapifyUp(val->key);
//	waitingQueue[table[val]] = val;
	if (elements[val].where == flocOpen)
	{
		if (!HeapifyUp(elements[val].fOpenLocation, fpriority))
			HeapifyDown(elements[val].fOpenLocation, fpriority);
		
		if (!HeapifyUp(elements[val].gOpenLocation, gpriority))
			HeapifyDown(elements[val].gOpenLocation, gpriority);

		if (!HeapifyUp(elements[val].fMaxOpenLocation, priorityfMax))
			HeapifyDown(elements[val].fMaxOpenLocation, priorityfMax);
		
		if (!HeapifyUp(elements[val].gMaxOpenLocation, prioritygMax))
			HeapifyDown(elements[val].gMaxOpenLocation, prioritygMax);
		
		if (!HeapifyUp(elements[val].priorityLocation, priority))
			HeapifyDown(elements[val].priorityLocation, priority);
	}
}

template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3, typename CmpKey4,   class dataStructure>
void FMMBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,CmpKey4,   dataStructure>::ReOpen(uint64_t val)
{
//	EqKey eq;
//	assert(eq(waitingQueue[table[val]], val));
//	//HeapifyUp(val->key);
//	waitingQueue[table[val]] = val;

	if (elements[val].where == flocClosed){
		elements[val].where = flocOpen;
	
		priorityQueues[fpriority].push_back(val);
		elements[val].fOpenLocation = priorityQueues[fpriority].size() - 1;
		HeapifyUp(priorityQueues[fpriority].size() - 1,fpriority);
	
		priorityQueues[gpriority].push_back(val);
		elements[val].gOpenLocation = priorityQueues[gpriority].size() - 1;
		HeapifyUp(priorityQueues[gpriority].size() - 1,gpriority);
		
		priorityQueues[priorityfMax].push_back(val);
		elements[val].fMaxOpenLocation = priorityQueues[priorityfMax].size() - 1;
		HeapifyUp(priorityQueues[priorityfMax].size() - 1,priorityfMax);
	
		priorityQueues[prioritygMax].push_back(val);
		elements[val].gMaxOpenLocation = priorityQueues[prioritygMax].size() - 1;
		HeapifyUp(priorityQueues[prioritygMax].size() - 1,prioritygMax);
		
		priorityQueues[priority].push_back(val);
		elements[val].priorityLocation = priorityQueues[priority].size() - 1;
		HeapifyUp(priorityQueues[priority].size() - 1,priority);
	}

}

template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3, typename  CmpKey4, class dataStructure>
void FMMBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,CmpKey4, dataStructure>::Remove(uint64_t val)
{

	int findex = elements[val].fOpenLocation;
	int gindex = elements[val].gOpenLocation;
	int fMaxindex = elements[val].fMaxOpenLocation;
	int gMaxindex = elements[val].gMaxOpenLocation;
	int pindex = elements[val].priorityLocation;
	FMMStateLocation whichQueue = elements[val].where;
	if (whichQueue == flocOpen){
		elements[val].where = flocClosed;
		
		priorityQueues[fpriority][findex] = priorityQueues[fpriority][priorityQueues[fpriority].size() - 1];
		elements[priorityQueues[fpriority][findex]].fOpenLocation = findex;
		priorityQueues[fpriority].pop_back();

		if (!HeapifyUp(findex, fpriority))
			HeapifyDown(findex, fpriority);
		
		priorityQueues[gpriority][gindex] = priorityQueues[gpriority][priorityQueues[gpriority].size() - 1];
		elements[priorityQueues[gpriority][gindex]].gOpenLocation = gindex;
		priorityQueues[gpriority].pop_back();

		if (!HeapifyUp(gindex, gpriority))
			HeapifyDown(gindex, gpriority);
		
		priorityQueues[priorityfMax][fMaxindex] = priorityQueues[priorityfMax][priorityQueues[priorityfMax].size() - 1];
		elements[priorityQueues[priorityfMax][fMaxindex]].fMaxOpenLocation = fMaxindex;
		priorityQueues[priorityfMax].pop_back();

		if (!HeapifyUp(fMaxindex, priorityfMax))
			HeapifyDown(fMaxindex, priorityfMax);
		
		priorityQueues[prioritygMax][gMaxindex] = priorityQueues[prioritygMax][priorityQueues[prioritygMax].size() - 1];
		elements[priorityQueues[prioritygMax][gMaxindex]].gMaxOpenLocation = gMaxindex;
		priorityQueues[prioritygMax].pop_back();

		if (!HeapifyUp(gMaxindex, prioritygMax))
			HeapifyDown(gMaxindex, prioritygMax);
		
		priorityQueues[priority][pindex] = priorityQueues[priority][priorityQueues[priority].size() - 1];
		elements[priorityQueues[priority][pindex]].priorityLocation = pindex;
		priorityQueues[priority].pop_back();

		if (!HeapifyUp(pindex, priority))
			HeapifyDown(pindex, priority);
	}


}

///**
// * Indicate that the key for a particular object has increased.
// */
//template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3, typename CmpKey4,   class dataStructure>
//void FMMBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,CmpKey4,   dataStructure>::IncreaseKey(uint64_t val)
//{
////	EqKey eq;
////	assert(eq(waitingQueue[table[val]], val));
////	waitingQueue[table[val]] = val;
//	HeapifyDown(val);
//}

/**
 * Returns location of object as well as object key.
 */

template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3, typename CmpKey4,   class dataStructure>
FMMStateLocation FMMBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,CmpKey4,   dataStructure>::Lookup(uint64_t hashKey, uint64_t &objKey) const
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
template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3, typename CmpKey4,   class dataStructure>
uint64_t FMMBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,CmpKey4, dataStructure>::Peek(fmmOpenQueueType whichQueue) const
{
	assert(OpenSize() != 0);		
	return priorityQueues[whichQueue][0];
}

/**
 * Peek at the next item to be expanded.
 */
template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3, typename CmpKey4,   class dataStructure>
inline dataStructure &FMMBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,CmpKey4, dataStructure>::PeekAt(fmmOpenQueueType whichQueue)
{
	assert(OpenSize() != 0);
	return elements[priorityQueues[whichQueue][0]];
}



/**
 * Move the best item to the closed list and return key.
 */
template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3, typename CmpKey4,   class dataStructure>
uint64_t FMMBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,CmpKey4,   dataStructure>::Close(fmmOpenQueueType whichQueue)
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
template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3, typename CmpKey4,   class dataStructure>
bool FMMBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,CmpKey4,   dataStructure>::HeapifyUp(unsigned int index, fmmOpenQueueType whichQueue)
{
	if (index == 0) return false;
	int parent = (index-1)/2;

	if (whichQueue == fpriority)
	{
		CmpKey0 compare;
		if (compare(elements[priorityQueues[whichQueue][parent]], elements[priorityQueues[whichQueue][index]]))
		{
			unsigned int tmp = priorityQueues[whichQueue][parent];
			priorityQueues[whichQueue][parent] = priorityQueues[whichQueue][index];
			priorityQueues[whichQueue][index] = tmp;
			elements[priorityQueues[whichQueue][parent]].fOpenLocation = parent;
			elements[priorityQueues[whichQueue][index]].fOpenLocation = index;
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
			elements[priorityQueues[whichQueue][parent]].gOpenLocation = parent;
			elements[priorityQueues[whichQueue][index]].gOpenLocation = index;
			HeapifyUp(parent, whichQueue);
			return true;
		}
	}
	else if (whichQueue == priorityfMax)
	{
		CmpKey2 compare;
		if (compare(elements[priorityQueues[whichQueue][parent]], elements[priorityQueues[whichQueue][index]]))
		{
			unsigned int tmp = priorityQueues[whichQueue][parent];
			priorityQueues[whichQueue][parent] = priorityQueues[whichQueue][index];
			priorityQueues[whichQueue][index] = tmp;
			elements[priorityQueues[whichQueue][parent]].fMaxOpenLocation = parent;
			elements[priorityQueues[whichQueue][index]].fMaxOpenLocation = index;
			HeapifyUp(parent, whichQueue);
			return true;
		}
	}
	else if (whichQueue == prioritygMax)
	{
		CmpKey3 compare;
		if (compare(elements[priorityQueues[whichQueue][parent]], elements[priorityQueues[whichQueue][index]]))
		{
			unsigned int tmp = priorityQueues[whichQueue][parent];
			priorityQueues[whichQueue][parent] = priorityQueues[whichQueue][index];
			priorityQueues[whichQueue][index] = tmp;
			elements[priorityQueues[whichQueue][parent]].gMaxOpenLocation = parent;
			elements[priorityQueues[whichQueue][index]].gMaxOpenLocation = index;
			HeapifyUp(parent, whichQueue);
			return true;
		}
	}
	else if (whichQueue == priority)
	{
		CmpKey4 compare;
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

	return false;
}

template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3, typename CmpKey4,   class dataStructure>
void FMMBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,CmpKey4,   dataStructure>::HeapifyDown(unsigned int index, fmmOpenQueueType whichQueue)
{
	
	unsigned int child1 = index*2+1;
	unsigned int child2 = index*2+2;



	if (whichQueue == fpriority)
	{
		CmpKey0 compare;
		int which;
		unsigned int count = priorityQueues[whichQueue].size();
		// find smallest child
		if (child1 >= count)
			return;
		else if (child2 >= count)
			which = child1;
		else if (!(compare(elements[priorityQueues[whichQueue][child1]], elements[priorityQueues[whichQueue][child2]])))
			which = child1;
		else
			which = child2;

		//if (fless(waitingQueue[which]->GetKey(), waitingQueue[index]->GetKey()))
		if (!(compare(elements[priorityQueues[whichQueue][which]], elements[priorityQueues[whichQueue][index]])))
		{
			unsigned int tmp = priorityQueues[whichQueue][which];
			priorityQueues[whichQueue][which] = priorityQueues[whichQueue][index];
			//		table[waitingQueue[which]] = which;
			priorityQueues[whichQueue][index] = tmp;
			elements[priorityQueues[whichQueue][which]].fOpenLocation = which;
			elements[priorityQueues[whichQueue][index]].fOpenLocation = index;
			//		table[waitingQueue[index]] = index;
			//    waitingQueue[which]->key = which;
			//    waitingQueue[index]->key = index;
			HeapifyDown(which,whichQueue);
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

		//if (fless(waitingQueue[which]->GetKey(), waitingQueue[index]->GetKey()))
		if (!(compare(elements[priorityQueues[whichQueue][which]], elements[priorityQueues[whichQueue][index]])))
		{
			unsigned int tmp = priorityQueues[whichQueue][which];
			priorityQueues[whichQueue][which] = priorityQueues[whichQueue][index];
			priorityQueues[whichQueue][index] = tmp;

//			assert(elements[priorityQueues[whichQueue][which]].where == flocOpenWaiting);
//			assert(elements[priorityQueues[whichQueue][index]].where == flocOpenWaiting);
			elements[priorityQueues[whichQueue][which]].gOpenLocation = which;
			elements[priorityQueues[whichQueue][index]].gOpenLocation = index;
			HeapifyDown(which, whichQueue);
//			assert((compare(elements[priorityQueues[whichQueue][which]], elements[priorityQueues[whichQueue][index]])));
//			if (child1 < count)
//				assert((compare(elements[priorityQueues[whichQueue][child1]], elements[priorityQueues[whichQueue][index]])));
//			if (child2 < count)
//			{
//				printf("w:%d - c1:%d c2:%d\n", which, child1, child2);
//				assert((compare(elements[priorityQueues[whichQueue][child2]], elements[priorityQueues[whichQueue][index]])));
//			}
		}
	}
		
	else if (whichQueue == priorityfMax)
	{
		CmpKey2 compare;
		int which;
		unsigned int count = priorityQueues[whichQueue].size();
		// find smallest child
		if (child1 >= count)
			return;
		else if (child2 >= count)
			which = child1;
		else if (!(compare(elements[priorityQueues[whichQueue][child1]], elements[priorityQueues[whichQueue][child2]])))
			which = child1;
		else
			which = child2;

		//if (fless(waitingQueue[which]->GetKey(), waitingQueue[index]->GetKey()))
		if (!(compare(elements[priorityQueues[whichQueue][which]], elements[priorityQueues[whichQueue][index]])))
		{
			unsigned int tmp = priorityQueues[whichQueue][which];
			priorityQueues[whichQueue][which] = priorityQueues[whichQueue][index];
			//		table[waitingQueue[which]] = which;
			priorityQueues[whichQueue][index] = tmp;
			elements[priorityQueues[whichQueue][which]].fMaxOpenLocation = which;
			elements[priorityQueues[whichQueue][index]].fMaxOpenLocation = index;
			//		table[waitingQueue[index]] = index;
			//    waitingQueue[which]->key = which;
			//    waitingQueue[index]->key = index;
			HeapifyDown(which,whichQueue);
		}
	}
	else if (whichQueue == prioritygMax)
	{
		CmpKey3 compare;
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

		//if (fless(waitingQueue[which]->GetKey(), waitingQueue[index]->GetKey()))
		if (!(compare(elements[priorityQueues[whichQueue][which]], elements[priorityQueues[whichQueue][index]])))
		{
			unsigned int tmp = priorityQueues[whichQueue][which];
			priorityQueues[whichQueue][which] = priorityQueues[whichQueue][index];
			priorityQueues[whichQueue][index] = tmp;

//			assert(elements[priorityQueues[whichQueue][which]].where == flocOpenWaiting);
//			assert(elements[priorityQueues[whichQueue][index]].where == flocOpenWaiting);
			elements[priorityQueues[whichQueue][which]].gMaxOpenLocation = which;
			elements[priorityQueues[whichQueue][index]].gMaxOpenLocation = index;
			HeapifyDown(which, whichQueue);
//			assert((compare(elements[priorityQueues[whichQueue][which]], elements[priorityQueues[whichQueue][index]])));
//			if (child1 < count)
//				assert((compare(elements[priorityQueues[whichQueue][child1]], elements[priorityQueues[whichQueue][index]])));
//			if (child2 < count)
//			{
//				printf("w:%d - c1:%d c2:%d\n", which, child1, child2);
//				assert((compare(elements[priorityQueues[whichQueue][child2]], elements[priorityQueues[whichQueue][index]])));
//			}
		}
	}
	else if (whichQueue == priority)
	{
		CmpKey4 compare;
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

		//if (fless(waitingQueue[which]->GetKey(), waitingQueue[index]->GetKey()))
		if (!(compare(elements[priorityQueues[whichQueue][which]], elements[priorityQueues[whichQueue][index]])))
		{
			unsigned int tmp = priorityQueues[whichQueue][which];
			priorityQueues[whichQueue][which] = priorityQueues[whichQueue][index];
			priorityQueues[whichQueue][index] = tmp;

//			assert(elements[priorityQueues[whichQueue][which]].where == flocOpenWaiting);
//			assert(elements[priorityQueues[whichQueue][index]].where == flocOpenWaiting);
			elements[priorityQueues[whichQueue][which]].priorityLocation = which;
			elements[priorityQueues[whichQueue][index]].priorityLocation = index;
			HeapifyDown(which, whichQueue);
//			assert((compare(elements[priorityQueues[whichQueue][which]], elements[priorityQueues[whichQueue][index]])));
//			if (child1 < count)
//				assert((compare(elements[priorityQueues[whichQueue][child1]], elements[priorityQueues[whichQueue][index]])));
//			if (child2 < count)
//			{
//				printf("w:%d - c1:%d c2:%d\n", which, child1, child2);
//				assert((compare(elements[priorityQueues[whichQueue][child2]], elements[priorityQueues[whichQueue][index]])));
//			}
		}
	}

}

#endif