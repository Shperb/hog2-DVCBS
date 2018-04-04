/*
 *  CBBSOpenClosed.h
 */

#ifndef CBBSOpenClosed_H
#define CBBSOpenClosed_H

#include <cassert>
#include <vector>
#include <ext/hash_map>
#include <stdint.h>
#include "AStarOpenClosed.h"
#include <unordered_map>
#include <map>
#include <set>
//struct AHash64 {
//	size_t operator()(const uint64_t &x) const
//	{ return (size_t)(x); }
//};

template<typename state>
class CBBSOpenClosedData {
public:
	CBBSOpenClosedData() {}
	CBBSOpenClosedData(const state &theData, double gCost, double hCost, uint64_t parent, stateLocation location)
	:data(theData), g(gCost), h(hCost), parentID(parent), where(location) { reopened = false; }
	state data;
	double g;
	double h;
	uint64_t parentID;
	//uint64_t openLocation;
	bool reopened;
	stateLocation where;
};

template<typename state, class dataStructure = CBBSOpenClosedData<state> >
class CBBSOpenClosed {
public:
	CBBSOpenClosed();
	~CBBSOpenClosed();
	void Reset();
	uint64_t AddOpenNode(const state &val, uint64_t hash, double g, double h, uint64_t parent=kTBDNoNode, stateLocation whichQueue = kOpenWaiting);
	uint64_t AddClosedNode(state &val, uint64_t hash, double g, double h, uint64_t parent=kTBDNoNode);
	void KeyChanged(uint64_t objKey,double oldVal);
	void eraseFromNodesMap(stateLocation whichQueue,uint64_t val);
	void eraseFromNodesMap(stateLocation whichQueue,uint64_t val,double oldKey);
	void eraseFirstClusterFromNodesMap(stateLocation whichQueue);
	void putInNodesMap(stateLocation whichQueue,uint64_t val);
	void validateSize(int func);
	void Remove(uint64_t objKey);
	void getNodeClusterLeqValue(std::vector<std::pair<double,uint64_t> >& cluster, double value);
	//void IncreaseKey(uint64_t objKey);
	stateLocation Lookup(uint64_t hashKey, uint64_t &objKey) const;
	inline dataStructure &Lookup(uint64_t objKey) { return elements[objKey]; }
	inline const dataStructure &Lookat(uint64_t objKey) const { return elements[objKey]; }
	double getFirstKey(stateLocation whichQueue) const { return nodesMap[whichQueue].begin()->first; }

	//if exist min pair, return true, else(one queue is empty) return false;
	//if it returns true, left and right should be set to the pair that is supposed to be returned.
	//bool ExtractMinPair(uint64_t& left, uint64_t& right) ;
	uint64_t Close();
	uint64_t CloseAtIndex(uint64_t index);
	void PutToReady();
	//void Reopen(uint64_t objKey);
	
	

	size_t OpenReadySize() const { return nodesMapSize[kOpenReady]; }
	size_t OpenWaitingSize() const { return nodesMapSize[kOpenWaiting]; }
	size_t OpenSize() const { return OpenReadySize()+OpenWaitingSize(); }
	
	struct compareByH {
		compareByH(CBBSOpenClosed<state, dataStructure>& parent):mParent(parent){}
		bool operator () (uint64_t i, uint64_t j) { return (mParent.Lookup(i).h<mParent.Lookup(j).h); }
		CBBSOpenClosed<state, dataStructure>& mParent;
	};

	//std::map<double,std::pair>std::set<uint64_t> >::iterator getNodesMapBegin(stateLocation whichQueue) {return nodesMap[whichQueue].begin();}
	//std::map<double,std::set<uint64_t> >::iterator getNodesMapEnd(stateLocation whichQueue) {return nodesMap[whichQueue].end();}
	std::set<uint64_t,compareByH>& getNodesMapElements(stateLocation whichQueue,double key) {return nodesMap[whichQueue].find(key)->second.second;}



	size_t ClosedSize() const { return size()-OpenReadySize()-OpenWaitingSize(); }
	size_t size() const { return elements.size(); }
	void verifyData();
private:


	//2 queues:
	//priorityQueues[0] is openReady, priorityQueues[1] is openWaiting
	std::vector<std::map<double,std::pair<std::map<uint64_t,std::set<uint64_t>::iterator>,std::set<uint64_t,compareByH> > > > nodesMap;
	std::vector<double> nodesMapSize;
	//std::vector<uint64_t> readyQueue;
	//std::vector<uint64_t> waitingQueue;

	// storing the element id; looking up with...hash?
	typedef __gnu_cxx::hash_map<uint64_t, uint64_t, AHash64> IndexTable;
	
	IndexTable table;
	//all the elements, open or closed
	std::vector<dataStructure> elements;
};


template<typename state, class dataStructure>
CBBSOpenClosed<state, dataStructure>::CBBSOpenClosed()
{
	std::map<double,std::pair<std::map<uint64_t,std::set<uint64_t>::iterator>,std::set<uint64_t,compareByH> > > map;
	nodesMap.push_back(map);
	nodesMap.push_back(map);
	nodesMapSize.push_back(0);
	nodesMapSize.push_back(0);
	//readyQueue.resize(0);
	//waitingQueue.resize(0);
}

template<typename state, class dataStructure>
CBBSOpenClosed<state, dataStructure>::~CBBSOpenClosed()
{
}

/**
 * Remove all objects from queue.
 */
template<typename state,   class dataStructure>
void CBBSOpenClosed<state,   dataStructure>::Reset()
{
	table.clear();
	elements.clear();
	nodesMap[0].clear();
	nodesMap[1].clear();
	nodesMapSize[0]=0;
	nodesMapSize[1]=0;
	//readyQueue.resize(0);
	//waitingQueue.resize(0);
}

/**
 * Add object into open list.
 */
template<typename state,   class dataStructure>
uint64_t CBBSOpenClosed<state,   dataStructure>::AddOpenNode(const state &val, uint64_t hash, double g, double h, uint64_t parent, stateLocation whichQueue)
{
	//validateSize(1);
	// should do lookup here...
	if (table.find(hash) != table.end())
	{
		//return -1; // TODO: find correct id and return
		assert(false);
	}

	elements.push_back(dataStructure(val, g, h, parent, whichQueue));

	if (parent == kTBDNoNode)
		elements.back().parentID = elements.size()-1;
	table[hash] = elements.size()-1; // hashing to element list location
	
	putInNodesMap(whichQueue,elements.size() - 1);
	//validateSize(2);
	return elements.size()-1;
}

/**
 * Add object into closed list.
 */
template<typename state,   class dataStructure>
uint64_t CBBSOpenClosed<state,   dataStructure>::AddClosedNode(state &val, uint64_t hash, double g, double h, uint64_t parent)
{
	//validateSize(3);
	// should do lookup here...
	assert(table.find(hash) == table.end());
	elements.push_back(dataStructure(val, g, h, parent,kClosed));
	if (parent == kTBDNoNode)
		elements.back().parentID = elements.size()-1;
	table[hash] = elements.size()-1; // hashing to element list location
	//validateSize(4);
	return elements.size()-1;
}

template<typename state,   class dataStructure>
void CBBSOpenClosed<state,   dataStructure>::getNodeClusterLeqValue(std::vector<std::pair<double,uint64_t> >& cluster, double value)
{
	for (auto it = nodesMap[kOpenReady].begin(); it != nodesMap[kOpenReady].end() && it->first <= value; it++){
		cluster.push_back(std::make_pair(it->first,it->second.second.size()));
	}
}

/**
 * Indicate that the key for a particular object has changed.
 */

template<typename state,   class dataStructure>
void CBBSOpenClosed<state,   dataStructure>::KeyChanged(uint64_t val,double oldKey)
{
	//validateSize(5);
	eraseFromNodesMap(elements[val].where,val,oldKey);
	putInNodesMap(elements[val].where,val);
	//validateSize(6);
}

template<typename state,   class dataStructure>
void CBBSOpenClosed<state,   dataStructure>::putInNodesMap(stateLocation whichQueue,uint64_t val)
{
	//validateSize(7);
	double key;
	if (whichQueue == kOpenReady){
		key = elements[val].g;
	}
	else if (whichQueue == kOpenWaiting){
		key = elements[val].g+elements[val].h;
	}
	else{
		return;
	}
	std::pair<std::set<uint64_t>::iterator,bool> ret;
	auto container = nodesMap[whichQueue].find(key);
	if (container == nodesMap[whichQueue].end()){
		std::pair<std::map<uint64_t,std::set<uint64_t>::iterator>,std::set<uint64_t,compareByH>> pair
			= std::make_pair(std::map<uint64_t,std::set<uint64_t>::iterator>(),std::set<uint64_t,compareByH>(compareByH(*this)));
		container = nodesMap[whichQueue].insert(std::make_pair(key,pair)).first;
		//container = .first;
	}
	ret = container->second.second.insert(val);
	if (ret.second){
		container->second.first[val]=ret.first;
		nodesMapSize[whichQueue] = nodesMapSize[whichQueue]+1;
	}
	//validateSize(8);
}

template<typename state,   class dataStructure>
void CBBSOpenClosed<state,   dataStructure>::eraseFromNodesMap(stateLocation whichQueue,uint64_t val)
{
	//validateSize(9);
	double key;
	if (whichQueue == kOpenReady){
		key = elements[val].g;
	}
	else if (whichQueue == kOpenWaiting){
		key = elements[val].g+elements[val].h;
	}
	else{
		return;
	}
	eraseFromNodesMap(whichQueue,val,key);
	//validateSize(10);
}
/*
template<typename state,   class dataStructure>
void CBBSOpenClosed<state,   dataStructure>::validateSize(int func)
{
	//printf("bla ");
	for (int queue = 0;queue <=1;queue++){
		uint64_t size = 0;
		auto containerIter = nodesMap[queue].begin();
		while (containerIter != nodesMap[queue].end()){
			size+= containerIter->second.size();
			containerIter++;
		}
		if (size != nodesMapSize[queue]){
			printf("found inconsistancy: %i,%i,%lld,%lld",func,queue,size,nodesMapSize[queue]);
		}
	}

}
*/
template<typename state,   class dataStructure>
void CBBSOpenClosed<state,   dataStructure>::eraseFromNodesMap(stateLocation whichQueue,uint64_t val,double oldKey)
{
	//validateSize(11);
	auto containerIter = nodesMap[whichQueue].find(oldKey);
	if (containerIter != nodesMap[whichQueue].end()){
		auto toDelete = containerIter->second.first.find(val);
		if (toDelete != containerIter->second.first.end()){
			containerIter->second.second.erase(toDelete->second);
			containerIter->second.first.erase(toDelete);
			nodesMapSize[whichQueue] = nodesMapSize[whichQueue]-1;
			if (containerIter->second.second.empty()){
				nodesMap[whichQueue].erase(containerIter);
			}
		}
	}
	//validateSize(12);
}

template<typename state,   class dataStructure>
void CBBSOpenClosed<state,   dataStructure>::eraseFirstClusterFromNodesMap(stateLocation whichQueue)
{
	//validateSize(13);
	auto containerIter = nodesMap[whichQueue].begin();
	if (containerIter != nodesMap[whichQueue].end()){
		nodesMapSize[whichQueue] = nodesMapSize[whichQueue]-containerIter->second.second.size();
		nodesMap[whichQueue].erase(containerIter);
	}
	//validateSize(14);
}

template<typename state, class dataStructure>
void CBBSOpenClosed<state, dataStructure>::Remove(uint64_t val)
{
	//validateSize(15);
	stateLocation whichQueue = elements[val].where;
	elements[val].where = kClosed;
	eraseFromNodesMap(whichQueue,val);
	//validateSize(16);
}

template<typename state,   class dataStructure>
stateLocation CBBSOpenClosed<state,   dataStructure>::Lookup(uint64_t hashKey, uint64_t &objKey) const
{
	typename IndexTable::const_iterator it;
	it = table.find(hashKey);
	if (it != table.end())
	{
		objKey = (*it).second;
		return elements[objKey].where;
	}
	return kUnseen;
}

template<typename state,   class dataStructure>
uint64_t CBBSOpenClosed<state,   dataStructure>::CloseAtIndex(uint64_t val)
{
	//validateSize(17);
	stateLocation whichQueue = elements[val].where;
	elements[val].where = kClosed;
	eraseFromNodesMap(whichQueue,val);
	//validateSize(18);
	return val;
}

template<typename state, class dataStructure>
void CBBSOpenClosed<state, dataStructure>::PutToReady()
{
	//validateSize(19);
	assert(OpenWaitingSize() != 0);
	auto container = nodesMap[kOpenWaiting].begin()->second.second;
	for (auto i = container.begin();i != container.end(); i++){
		putInNodesMap(kOpenReady,*i);
		elements[*i].where = kOpenReady;
	}
	eraseFirstClusterFromNodesMap(kOpenWaiting);
	//validateSize(20);
}


#endif
