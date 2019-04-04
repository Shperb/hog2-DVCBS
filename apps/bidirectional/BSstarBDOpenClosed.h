/*
 *  BSstarBDOpenClosed.h
 */

#ifndef BSstarBDOpenClosed_H
#define BSstarBDOpenClosed_H

#include <cassert>
#include <vector>
#include <ext/hash_map>
#include <stdint.h>
#include "AStarOpenClosed.h"

//struct AHash64 {
//	size_t operator()(const uint64_t &x) const
//	{ return (size_t)(x); }
//};


enum BSstarStateLocation {
	locOpen,//priority queue 0, low g -> low f
	locClosed,
	locUnseen
};

enum openQueueType {
	kf = 0,//priority queue 0, low g -> low f
	kg = 1,
	kfMax = 2,
	kgMax = 3
};

//const uint64_t kTBDNoNode = 0xFFFFFFFFFFFFFFFFull;

template<typename state>
class BSstarBDOpenClosedData {
public:
	BSstarBDOpenClosedData() {}
	BSstarBDOpenClosedData(const state &theData, double gCost, double hCost, double fCost,uint64_t parent, uint64_t gopenLoc,uint64_t fopenLoc, uint64_t gMaxOpenLoc, uint64_t fMaxOpenLoc,BSstarStateLocation location)
	:data(theData), g(gCost), h(hCost),f(fCost),parentID(parent), gOpenLocation(gopenLoc),fOpenLocation(fopenLoc), gMaxOpenLocation(gMaxOpenLoc),fMaxOpenLocation(fMaxOpenLoc), where(location) { reopened = false; }
	state data;
	double g;
	double h;
	double f;
	uint64_t parentID;
	uint64_t gOpenLocation;
	uint64_t fOpenLocation;
	uint64_t gMaxOpenLocation;
	uint64_t fMaxOpenLocation;
	bool reopened;
	BSstarStateLocation where;
};

template<typename state, typename CmpKey0, typename CmpKey1, typename CmpKey2,typename CmpKey3, class dataStructure = BSstarBDOpenClosedData<state> >
class BSstarBDOpenClosed {
public:
	BSstarBDOpenClosed();
	~BSstarBDOpenClosed();
	void Reset();
	uint64_t AddOpenNode(const state &val, uint64_t hash, double g, double h, double f,uint64_t parent=kTBDNoNode, BSstarStateLocation whichQueue = locOpen);
	uint64_t AddClosedNode(state &val, uint64_t hash, double g, double h, double f, uint64_t parent=kTBDNoNode);
	void KeyChanged(uint64_t objKey);
	void ReOpen(uint64_t objKey);
	void Remove(uint64_t objKey);
	//void IncreaseKey(uint64_t objKey);
	BSstarStateLocation Lookup(uint64_t hashKey, uint64_t &objKey) const;
	inline dataStructure &Lookup(uint64_t objKey) { return elements[objKey]; }
	inline const dataStructure &Lookat(uint64_t objKey) const { return elements[objKey]; }
	uint64_t Peek(openQueueType whichQueue) const;
	inline dataStructure &PeekAt(openQueueType whichQueue);

	//if exist min pair, return true, else(one queue is empty) return false;
	//if it returns true, left and right should be set to the pair that is supposed to be returned.
	//bool ExtractMinPair(uint64_t& left, uint64_t& right) ;
	uint64_t Close(openQueueType whichQueue);
	//void Reopen(uint64_t objKey);

	uint64_t GetOpenItem(unsigned int which, openQueueType where){	return priorityQueues[where][which];}
	size_t OpenSize() const { return priorityQueues[kf].size(); }

	size_t ClosedSize() const { return size()-OpenSize(); }
	size_t size() const { return elements.size(); }

private:
	bool HeapifyUp(unsigned int index, openQueueType whichQueue);
	void HeapifyDown(unsigned int index, openQueueType whichQueue);

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


template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3, class dataStructure>
BSstarBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3, dataStructure>::BSstarBDOpenClosed()
{
	std::vector<uint64_t> queue;
	queue.resize(0);
	priorityQueues.push_back(queue);
	priorityQueues.push_back(queue);
	priorityQueues.push_back(queue);
	priorityQueues.push_back(queue);
	//readyQueue.resize(0);
	//waitingQueue.resize(0);
}

template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3, class dataStructure>
BSstarBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3, dataStructure>::~BSstarBDOpenClosed()
{
}

/**
 * Remove all objects from queue.
 */
template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3,   class dataStructure>
void BSstarBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,   dataStructure>::Reset()
{
	table.clear();
	elements.clear();
	priorityQueues[kf].resize(0);
	priorityQueues[kg].resize(0);
	priorityQueues[kfMax].resize(0);
	priorityQueues[kgMax].resize(0);
	//readyQueue.resize(0);
	//waitingQueue.resize(0);
}

/**
 * Add object into open list.
 */
template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3,   class dataStructure>
uint64_t BSstarBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,   dataStructure>::AddOpenNode(const state &val, uint64_t hash, double g, double h,double f, uint64_t parent, BSstarStateLocation whichQueue)
{
	// should do lookup here...
	if (table.find(hash) != table.end())
	{
		//return -1; // TODO: find correct id and return
		assert(false);
	}
	if (whichQueue == locOpen)
	{
		elements.push_back(dataStructure(val, g, h,f, parent, priorityQueues[kg].size(),priorityQueues[kf].size() , priorityQueues[kgMax].size(),priorityQueues[kfMax].size(),  locOpen));
		
	}
	
	if (parent == kTBDNoNode)
		elements.back().parentID = elements.size()-1;
	table[hash] = elements.size()-1; // hashing to element list location

	priorityQueues[kf].push_back(elements.size() - 1);
	HeapifyUp(priorityQueues[kf].size() - 1,kf);
	
	priorityQueues[kg].push_back(elements.size() - 1);
	HeapifyUp(priorityQueues[kg].size() - 1,kg);
	
	priorityQueues[kfMax].push_back(elements.size() - 1);
	HeapifyUp(priorityQueues[kfMax].size() - 1,kfMax);
	
	priorityQueues[kgMax].push_back(elements.size() - 1);
	HeapifyUp(priorityQueues[kgMax].size() - 1,kgMax);

	return elements.size()-1;
}

/**
 * Add object into closed list.
 */
template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3,   class dataStructure>
uint64_t BSstarBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,   dataStructure>::AddClosedNode(state &val, uint64_t hash, double g, double h, double f, uint64_t parent)
{
	// should do lookup here...
	assert(table.find(hash) == table.end());
	elements.push_back(dataStructure(val, g, h,f, parent, 0,0, locClosed));
	if (parent == kTBDNoNode)
		elements.back().parentID = elements.size()-1;
	table[hash] = elements.size()-1; // hashing to element list location
	return elements.size()-1;
}

/**
 * Indicate that the key for a particular object has changed.
 */
template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3,   class dataStructure>
void BSstarBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,   dataStructure>::KeyChanged(uint64_t val)
{
//	EqKey eq;
//	assert(eq(waitingQueue[table[val]], val));
//	//HeapifyUp(val->key);
//	waitingQueue[table[val]] = val;
	if (elements[val].where == locOpen)
	{
		if (!HeapifyUp(elements[val].fOpenLocation, kf))
			HeapifyDown(elements[val].fOpenLocation, kf);
		
		if (!HeapifyUp(elements[val].gOpenLocation, kg))
			HeapifyDown(elements[val].gOpenLocation, kg);

		if (!HeapifyUp(elements[val].fMaxOpenLocation, kfMax))
			HeapifyDown(elements[val].fMaxOpenLocation, kfMax);
		
		if (!HeapifyUp(elements[val].gMaxOpenLocation, kgMax))
			HeapifyDown(elements[val].gMaxOpenLocation, kgMax);
	}
}

template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3,   class dataStructure>
void BSstarBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,   dataStructure>::ReOpen(uint64_t val)
{
//	EqKey eq;
//	assert(eq(waitingQueue[table[val]], val));
//	//HeapifyUp(val->key);
//	waitingQueue[table[val]] = val;

	if (elements[val].where == locClosed){
		elements[val].where = locOpen;
	
		priorityQueues[kf].push_back(val);
		elements[val].fOpenLocation = priorityQueues[kf].size() - 1;
		HeapifyUp(priorityQueues[kf].size() - 1,kf);
	
		priorityQueues[kg].push_back(val);
		elements[val].gOpenLocation = priorityQueues[kg].size() - 1;
		HeapifyUp(priorityQueues[kg].size() - 1,kg);
		
		priorityQueues[kfMax].push_back(val);
		elements[val].fMaxOpenLocation = priorityQueues[kfMax].size() - 1;
		HeapifyUp(priorityQueues[kfMax].size() - 1,kfMax);
	
		priorityQueues[kgMax].push_back(val);
		elements[val].gMaxOpenLocation = priorityQueues[kgMax].size() - 1;
		HeapifyUp(priorityQueues[kgMax].size() - 1,kgMax);
	}

}

template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3, class dataStructure>
void BSstarBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3, dataStructure>::Remove(uint64_t val)
{

	int findex = elements[val].fOpenLocation;
	int gindex = elements[val].gOpenLocation;
	int fMaxindex = elements[val].fMaxOpenLocation;
	int gMaxindex = elements[val].gMaxOpenLocation;
	BSstarStateLocation whichQueue = elements[val].where;
	if (whichQueue == locOpen){
		elements[val].where = locClosed;
		
		priorityQueues[kf][findex] = priorityQueues[kf][priorityQueues[kf].size() - 1];
		elements[priorityQueues[kf][findex]].fOpenLocation = findex;
		priorityQueues[kf].pop_back();

		if (!HeapifyUp(findex, kf))
			HeapifyDown(findex, kf);
		
		priorityQueues[kg][gindex] = priorityQueues[kg][priorityQueues[kg].size() - 1];
		elements[priorityQueues[kg][gindex]].gOpenLocation = gindex;
		priorityQueues[kg].pop_back();

		if (!HeapifyUp(gindex, kg))
			HeapifyDown(gindex, kg);
		
		priorityQueues[kfMax][fMaxindex] = priorityQueues[kfMax][priorityQueues[kfMax].size() - 1];
		elements[priorityQueues[kfMax][fMaxindex]].fMaxOpenLocation = fMaxindex;
		priorityQueues[kfMax].pop_back();

		if (!HeapifyUp(fMaxindex, kfMax))
			HeapifyDown(fMaxindex, kfMax);
		
		priorityQueues[kgMax][gMaxindex] = priorityQueues[kgMax][priorityQueues[kgMax].size() - 1];
		elements[priorityQueues[kgMax][gMaxindex]].gMaxOpenLocation = gMaxindex;
		priorityQueues[kgMax].pop_back();

		if (!HeapifyUp(gMaxindex, kgMax))
			HeapifyDown(gMaxindex, kgMax);
	}


}

///**
// * Indicate that the key for a particular object has increased.
// */
//template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3,   class dataStructure>
//void BSstarBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,   dataStructure>::IncreaseKey(uint64_t val)
//{
////	EqKey eq;
////	assert(eq(waitingQueue[table[val]], val));
////	waitingQueue[table[val]] = val;
//	HeapifyDown(val);
//}

/**
 * Returns location of object as well as object key.
 */

template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3,   class dataStructure>
BSstarStateLocation BSstarBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,   dataStructure>::Lookup(uint64_t hashKey, uint64_t &objKey) const
{
	typename IndexTable::const_iterator it;
	it = table.find(hashKey);
	if (it != table.end())
	{
		objKey = (*it).second;
		return elements[objKey].where;
	}
	return locUnseen;
}


/**
 * Peek at the next item to be expanded.
 */
template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3,   class dataStructure>
uint64_t BSstarBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3, dataStructure>::Peek(openQueueType whichQueue) const
{
	assert(OpenSize() != 0);		
	return priorityQueues[whichQueue][0];
}

/**
 * Peek at the next item to be expanded.
 */
template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3,   class dataStructure>
inline dataStructure &BSstarBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3, dataStructure>::PeekAt(openQueueType whichQueue)
{
	assert(OpenSize() != 0);
	return elements[priorityQueues[whichQueue][0]];
}



/**
 * Move the best item to the closed list and return key.
 */
template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3,   class dataStructure>
uint64_t BSstarBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,   dataStructure>::Close(openQueueType whichQueue)
{
	assert(OpenSize() != 0);
	uint64_t openSize = OpenSize();
	uint64_t ans = priorityQueues[whichQueue][0];
	Remove(ans);
	/*
	elements[ans].where = locClosed;
	
	priorityQueues[kf][elements[ans].fOpenLocation] = priorityQueues[kf][openSize-1];
	uint64_t oldLocationF = elements[ans].fOpenLocation;
	elements[priorityQueues[kf][elements[ans].fOpenLocation]].fOpenLocation = oldLocationF;
	elements[ans].fOpenLocation = -1;
	priorityQueues[kf].pop_back();
	
	HeapifyDown(oldLocationF,kf);
	
	priorityQueues[kg][elements[ans].gOpenLocation] = priorityQueues[kg][openSize-1];
	uint64_t oldLocationG = elements[ans].gOpenLocation;
	elements[priorityQueues[kg][elements[ans].gOpenLocation]].gOpenLocation = oldLocationG;
	elements[ans].gOpenLocation = -1;
	priorityQueues[kg].pop_back();
	
	HeapifyDown(oldLocationG,kg);
	
	priorityQueues[kfMax][elements[ans].fMaxOpenLocation] = priorityQueues[kfMax][openSize-1];
	uint64_t oldLocationFMax = elements[ans].fMaxOpenLocation;
	elements[priorityQueues[kfMax][elements[ans].fMaxOpenLocation]].fMaxOpenLocation = oldLocationFMax;
	elements[ans].fMaxOpenLocation = -1;
	priorityQueues[kfMax].pop_back();
	
	HeapifyDown(oldLocationFMax,kfMax);
	
	priorityQueues[kgMax][elements[ans].gMaxOpenLocation] = priorityQueues[kgMax][openSize-1];
	uint64_t oldLocationGMax = elements[ans].gMaxOpenLocation;
	elements[priorityQueues[kgMax][elements[ans].gMaxOpenLocation]].gMaxOpenLocation = oldLocationGMax;
	elements[ans].gMaxOpenLocation = -1;
	priorityQueues[kgMax].pop_back();
	
	HeapifyDown(oldLocationGMax,kgMax);
	*/
	return ans;
}


/**
 * Moves a node up the heap. Returns true if the node was moved, false otherwise.
 */
template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3,   class dataStructure>
bool BSstarBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,   dataStructure>::HeapifyUp(unsigned int index, openQueueType whichQueue)
{
	if (index == 0) return false;
	int parent = (index-1)/2;

	if (whichQueue == kf)
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
	else if (whichQueue == kg)
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
	else if (whichQueue == kfMax)
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
	else if (whichQueue == kgMax)
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

	return false;
}

template<typename state, typename CmpKey0, typename CmpKey1, typename  CmpKey2,typename  CmpKey3,   class dataStructure>
void BSstarBDOpenClosed<state, CmpKey0, CmpKey1,CmpKey2,CmpKey3,   dataStructure>::HeapifyDown(unsigned int index, openQueueType whichQueue)
{
	
	unsigned int child1 = index*2+1;
	unsigned int child2 = index*2+2;



	if (whichQueue == kf)
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
	else if (whichQueue == kg)
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

//			assert(elements[priorityQueues[whichQueue][which]].where == locOpenWaiting);
//			assert(elements[priorityQueues[whichQueue][index]].where == locOpenWaiting);
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
		
	else if (whichQueue == kfMax)
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
	else if (whichQueue == kgMax)
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

//			assert(elements[priorityQueues[whichQueue][which]].where == locOpenWaiting);
//			assert(elements[priorityQueues[whichQueue][index]].where == locOpenWaiting);
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

}

#endif