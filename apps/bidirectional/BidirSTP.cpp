//
//  BidirSTP.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/30/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#include "BidirSTP.h"
#include "MNPuzzle.h"
#include "NBS.h"
#include "IDAStar.h"
#include "MM.h"
#include "BSStar.h"
#include "TemplateAStar.h"
#include "WeightedVertexGraph.h"
#include "STPInstances.h"
#include "LexPermutationPDB.h"
#include "MR1PermutationPDB.h"
#include <map>
#include <random>
#include <vector>



typedef MR1PermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> STPPDB;
void MakePDBs(MNPuzzleState<4, 4> g, Heuristic<MNPuzzleState<4, 4>> &h, MNPuzzle<4,4> &mnp)
{
	std::vector<int> p1 = {0,1,2,3,4,5};
	std::vector<int> p2 = {0,10,11,12,13,14,15};
	std::vector<int> p3 = {0,6,7,8,9};
//	std::vector<int> p4 = {0,12,13,14,15};
//	mnp.StoreGoal(g);
	STPPDB *pdb1 = new STPPDB(&mnp, g, p1);
	STPPDB *pdb2 = new STPPDB(&mnp, g, p2);
	STPPDB *pdb3 = new STPPDB(&mnp, g, p3);
//	STPPDB *pdb4 = new STPPDB(&mnp, g, p4);
	pdb1->BuildPDB(g, std::thread::hardware_concurrency());
	pdb2->BuildPDB(g, std::thread::hardware_concurrency());
	pdb3->BuildPDB(g, std::thread::hardware_concurrency());
//	pdb4->BuildPDB(g, std::thread::hardware_concurrency());
	h.lookups.resize(0);
	h.lookups.push_back({kMaxNode, 1, 4});
	h.lookups.push_back({kLeafNode, 0, 0});
	h.lookups.push_back({kLeafNode, 1, 1});
	h.lookups.push_back({kLeafNode, 2, 2});
	h.lookups.push_back({kLeafNode, 3, 3});
//	h.lookups.push_back({kLeafNode, 4, 4});
	h.heuristics.resize(0);
	h.heuristics.push_back(&mnp);
	h.heuristics.push_back(pdb1);
	h.heuristics.push_back(pdb2);
	h.heuristics.push_back(pdb3);
//	h.heuristics.push_back(pdb4);
}

std::vector<void*>* MakeMaxSmallPDBs(MNPuzzleState<4, 4> g, Heuristic<MNPuzzleState<4, 4>> &h)
{
  std::cout <<"starting PDB1" << std::endl;
  std::vector<void*>* toClean = new std::vector<void*>();
  //MNPuzzle<4,4>* md = new MNPuzzle<4,4>();
  //toClean->push_back(md);
  const int NUM_REP = 9;
  unsigned seed = 208694125;
	std::mt19937 gen(seed);
	std::uniform_int_distribution<std::mt19937::result_type> dist(100000000,999999999);
  h.lookups.resize(0);
  h.heuristics.resize(0);
	//h.heuristics.push_back(md);
	//h.lookups.push_back({kMaxNode, 1, NUM_REP+1});
  h.lookups.push_back({kMaxNode, 1, NUM_REP});
	//h.lookups.push_back({kLeafNode, 0, 0});
  std::vector<int> arr = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
  for (int i =1 ; i <= NUM_REP; i++){
    shuffle(arr.begin(), arr.end(), std::default_random_engine(dist(gen)));
    std::vector<int> p1(arr.begin(), arr.begin() + 6);
    p1.insert(p1.begin(), 0);
    std::vector<int> p2(arr.begin()+6, arr.begin() + 12);
    p2.insert(p2.begin(), 0);
    std::vector<int> p3(arr.begin()+12, arr.end());
    p3.insert(p3.begin(), 0);
    MNPuzzle<4,4>* mn1 = new MNPuzzle<4,4>();
    toClean->push_back(mn1);
    mn1->SetPattern(p1);
    mn1->StoreGoal(g);
    LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>* pdb1 = new LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>(mn1, g, p1);
    toClean->push_back(pdb1);
    pdb1->BuildAdditivePDB(g, std::thread::hardware_concurrency());
    //pdb1->DeltaCompress(mn1, g, true);
    //pdb1->DivCompress(17-p1.size(), true);
    
    
    MNPuzzle<4,4>* mn2 = new MNPuzzle<4,4>();
    toClean->push_back(mn2);
    mn2->SetPattern(p2);
    mn2->StoreGoal(g);
    LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>* pdb2 = new LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>(mn2, g, p2);
    toClean->push_back(pdb2);
    pdb2->BuildAdditivePDB(g, std::thread::hardware_concurrency());
    //pdb2->DeltaCompress(mn2, g, true);
    //pdb2->DivCompress(17-p2.size(), true);
    
    
    MNPuzzle<4,4>* mn3 = new MNPuzzle<4,4>();
    toClean->push_back(mn3);
    mn3->SetPattern(p3);
    mn3->StoreGoal(g);
    LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>* pdb3 = new LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>(mn3, g, p3);
    toClean->push_back(pdb3);
    pdb3->BuildAdditivePDB(g, std::thread::hardware_concurrency());    
    //pdb3->DeltaCompress(mn3, g, true);
    //pdb3->DivCompress(17-p3.size(), true);    
    
    // STPPDB *pdb1 = new STPPDB(&mnp, g, p1);
    // STPPDB *pdb2 = new STPPDB(&mnp, g, p2);
    // STPPDB *pdb3 = new STPPDB(&mnp, g, p3);
    // pdb1->BuildPDB(g, std::thread::hardware_concurrency());
    // pdb2->BuildPDB(g, std::thread::hardware_concurrency());
    // pdb3->BuildPDB(g, std::thread::hardware_concurrency());
    
    Heuristic<MNPuzzleState<4, 4>> *addative = new Heuristic<MNPuzzleState<4, 4>>();
    toClean->push_back(addative);
    addative->lookups.push_back({kAddNode, 1, 3});
    //addative->lookups.push_back({kMaxNode, 1, 3});
    addative->lookups.push_back({kLeafNode, 0, 0});
    addative->lookups.push_back({kLeafNode, 1, 1});
    addative->lookups.push_back({kLeafNode, 2, 2});
    addative->heuristics.push_back(pdb1);
    addative->heuristics.push_back(pdb2);
    addative->heuristics.push_back(pdb3);
    //h.lookups.push_back({kLeafNode, i, i});
    h.lookups.push_back({kLeafNode, i-1, i-1});
    h.heuristics.push_back(addative);    
  }
  return toClean;
 
}

std::vector<void*>* MakeBigPDB(MNPuzzleState<4, 4> g, Heuristic<MNPuzzleState<4, 4>> &h)
{
  std::cout <<"starting PDB2" << std::endl;
  std::vector<void*>* toClean = new std::vector<void*>();
  //std::vector<int> p12 = {0,9};
	//std::vector<int> p11 = {0,1};
	std::vector<int> p11 = {0,1,2,3,4,5,6,7,8};
	std::vector<int> p12 = {0,9,10,11,12,13,14};
	std::vector<int> p13 = {0,15};
  MNPuzzle<4,4>* mnp1 = new MNPuzzle<4,4>();
  toClean->push_back(mnp1);
  mnp1->SetPattern(p11);
	mnp1->StoreGoal(g);
	LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>* pdb11 = new LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>(mnp1, g, p11) ;
  toClean->push_back(pdb11);
	pdb11->BuildAdditivePDB(g, std::thread::hardware_concurrency());
  //pdb11->DeltaCompress(mnp1, g, true);
  //pdb11->DivCompress(17-p11.size(), true);
  
  MNPuzzle<4,4>* mnp2 = new MNPuzzle<4,4>();
  toClean->push_back(mnp2);  
  mnp2->SetPattern(p12);
	mnp2->StoreGoal(g);
	LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>* pdb12 = new LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>(mnp2, g, p12);
  toClean->push_back(pdb12);
	pdb12->BuildAdditivePDB(g, std::thread::hardware_concurrency());  
  //pdb12->DeltaCompress(mnp2, g, true);
  //pdb12->DivCompress(17-p12.size(), true);
  
  MNPuzzle<4,4>* mnp3 = new MNPuzzle<4,4>();
  toClean->push_back(mnp3);
  mnp3->SetPattern(p13);
	mnp3->StoreGoal(g);
	LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>* pdb13 = new LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>>(mnp3, g, p13);
  toClean->push_back(pdb13);
	pdb13->BuildAdditivePDB(g, std::thread::hardware_concurrency());  
  //pdb13->DeltaCompress(mnp3, g, true);
  //pdb13->DivCompress(17-p13.size(), true);


	// STPPDB *pdb11 = new STPPDB(&mnp, g, p11);
  // STPPDB *pdb12 = new STPPDB(&mnp, g, p12);
  // STPPDB *pdb13 = new STPPDB(&mnp, g, p13);

	// pdb11->BuildPDB(g, std::thread::hardware_concurrency());
	// pdb12->BuildPDB(g, std::thread::hardware_concurrency());
	// pdb13->BuildPDB(g, std::thread::hardware_concurrency());
  h.lookups.resize(0);
	h.heuristics.resize(0);
  
  //Heuristic<MNPuzzleState<4, 4>> *addative1 = new Heuristic<MNPuzzleState<4, 4>>();
  //toClean->push_back(addative1);
  //addative1->lookups.resize(0);
  h.lookups.push_back({kAddNode, 1, 3});
  //addative1->lookups.push_back({kMaxNode, 1, 3});
  h.lookups.push_back({kLeafNode, 0, 0});
  h.lookups.push_back({kLeafNode, 1, 1});
  h.lookups.push_back({kLeafNode, 2, 2});
  
  //addative1->heuristics.resize(0);
	h.heuristics.push_back(pdb11);
	h.heuristics.push_back(pdb12);
  h.heuristics.push_back(pdb13);
  
  //h.lookups.resize(0);
	// h.lookups.push_back({kMaxNode, 1, 2});
	// h.lookups.push_back({kLeafNode, 0, 0});
	// h.lookups.push_back({kLeafNode, 1, 1});
//	h.lookups.push_back({kLeafNode, 4, 4});
	//h.heuristics.resize(0);
  //MNPuzzle<4,4>* mnp = new MNPuzzle<4,4>();
  //toClean->push_back(mnp);
	//h.heuristics.push_back(mnp);
//	h.heuristics.push_back(addative1);
//	h.heuristics.push_back(pdb2);
//	h.heuristics.push_back(pdb3);
//	h.heuristics.push_back(pdb4);
  return toClean;
}

void TestSTP_PDB_overall()
{
  std::map<int,int> h1_overall_vals;
  std::map<int,int> h2_overall_vals;
  std::map<int,int> h1_real_vals;
  std::map<int,int> h2_real_vals;
  std::map<int,int> h1_real_bd_vals;
  std::map<int,int> h2_real_bd_vals;
  std::vector<MNPuzzleState<4,4>> nbsPath;
  std::vector<MNPuzzleState<4,4>> astarPath;
  MNPuzzleState<4, 4> goal;
  MNPuzzle<4,4> mnp;
  goal.Reset();
  Heuristic<MNPuzzleState<4, 4>> hf_1;
  Heuristic<MNPuzzleState<4, 4>> hf_2;
  Heuristic<MNPuzzleState<4, 4>> hb_1;
  Heuristic<MNPuzzleState<4, 4>> hb_2;
  std::vector<void*>* toClean_f1;
  std::vector<void*>* toClean_f2;
  toClean_f1 = MakeMaxSmallPDBs(goal, hf_1);
  toClean_f2 = MakeBigPDB(goal, hf_2);
  std::cout <<"starting sampling" << std::endl;
  Timer t1;
  
  for (int x = 0; x < 100; x++) // 547 to 540
	{
      MNPuzzleState<4, 4> s = STP::GetKorfInstance(x);
      std::vector<void*>* toClean_b1;
      std::vector<void*>* toClean_b2;
      toClean_b1 = MakeMaxSmallPDBs(s, hb_1);
      std::cout <<"starting solving" << std::endl;
      t1.StartTimer();
      TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>>* astar = new TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>>();
      astar->SetHeuristic(&hf_1);
      astar->GetPath(&mnp, s, goal, astarPath);
      t1.EndTimer();
      printf("A* h1 found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", mnp.GetPathLength(astarPath),
           astar->GetNodesExpanded(), astar->GetNecessaryExpansions(), astar->GetNodesTouched(), t1.GetElapsedTime());
      for (auto it = astar->openClosedList.elements.begin(); it != astar->openClosedList.elements.end(); it++){
        int h_value = (*it).h;
        if (h1_real_vals.find(h_value) == h1_real_vals.end()){
          h1_real_vals.insert(std::make_pair(h_value,1));
        }
        else{
          h1_real_vals[h_value]++;
        }
      }
      t1.StartTimer();
      astar->SetHeuristic(&hb_1);
      astar->GetPath(&mnp, goal, s, astarPath);
      t1.EndTimer();
      printf("r-A* h1 found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", mnp.GetPathLength(astarPath),
           astar->GetNodesExpanded(), astar->GetNecessaryExpansions(), astar->GetNodesTouched(), t1.GetElapsedTime());
      for (auto it = astar->openClosedList.elements.begin(); it != astar->openClosedList.elements.end(); it++){
        int h_value = (*it).h;
        if (h1_real_vals.find(h_value) == h1_real_vals.end()){
          h1_real_vals.insert(std::make_pair(h_value,1));
        }
        else{
          h1_real_vals[h_value]++;
        }
      }
      delete astar;
      NBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>>* nbs = new NBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>>();
      t1.StartTimer();
      nbs->GetPath(&mnp, s, goal, &hf_1, &hb_1, nbsPath);
      t1.EndTimer();
      printf("NBS h1 found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", mnp.GetPathLength(nbsPath),
           nbs->GetNodesExpanded(), nbs->GetNecessaryExpansions(), nbs->GetNodesTouched(), t1.GetElapsedTime());
      for (auto it = nbs->queue.forwardQueue.elements.begin(); it != nbs->queue.forwardQueue.elements.end(); it++){
        int h_value = (*it).h;
        if (h1_real_bd_vals.find(h_value) == h1_real_bd_vals.end()){
          h1_real_bd_vals.insert(std::make_pair(h_value,1));
        }
        else{
          h1_real_bd_vals[h_value]++;
        }
      }
      for (auto it = nbs->queue.backwardQueue.elements.begin(); it != nbs->queue.backwardQueue.elements.end(); it++){
        int h_value = (*it).h;
        if (h1_real_bd_vals.find(h_value) == h1_real_bd_vals.end()){
          h1_real_bd_vals.insert(std::make_pair(h_value,1));
        }
        else{
          h1_real_bd_vals[h_value]++;
        }
      }
      for (auto p : *toClean_b1)
        {
          delete p;
        } 
      delete toClean_b1;
      delete nbs;
      astar = new TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>>();
  t1.StartTimer();
      astar->SetHeuristic(&hf_2);
      astar->GetPath(&mnp, s, goal, astarPath);
      t1.EndTimer();
      printf("A* h2 found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", mnp.GetPathLength(astarPath),
           astar->GetNodesExpanded(), astar->GetNecessaryExpansions(), astar->GetNodesTouched(), t1.GetElapsedTime());
      for (auto it = astar->openClosedList.elements.begin(); it != astar->openClosedList.elements.end(); it++){
        int h_value = (*it).h;
        if (h2_real_vals.find(h_value) == h2_real_vals.end()){
          h2_real_vals.insert(std::make_pair(h_value,1));
        }
        else{
          h2_real_vals[h_value]++;
        }
      }
      toClean_b2 = MakeBigPDB(s, hb_2);
      t1.StartTimer();
      astar->SetHeuristic(&hb_2);
      astar->GetPath(&mnp, goal, s, astarPath);
      t1.EndTimer();
      printf("r-A* h2 found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", mnp.GetPathLength(astarPath),
           astar->GetNodesExpanded(), astar->GetNecessaryExpansions(), astar->GetNodesTouched(), t1.GetElapsedTime());
      for (auto it = astar->openClosedList.elements.begin(); it != astar->openClosedList.elements.end(); it++){
        int h_value = (*it).h;
        if (h2_real_vals.find(h_value) == h2_real_vals.end()){
          h2_real_vals.insert(std::make_pair(h_value,1));
        }
        else{
          h2_real_vals[h_value]++;
        }
      }
      delete astar;
      nbs = new NBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>>();
      t1.StartTimer();
      nbs->GetPath(&mnp, s, goal, &hf_2, &hb_2, nbsPath);
      t1.EndTimer();
      printf("NBS h2 found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", mnp.GetPathLength(nbsPath),
           nbs->GetNodesExpanded(), nbs->GetNecessaryExpansions(), nbs->GetNodesTouched(), t1.GetElapsedTime());
      for (auto it = nbs->queue.forwardQueue.elements.begin(); it != nbs->queue.forwardQueue.elements.end(); it++){
        int h_value = (*it).h;
        if (h2_real_bd_vals.find(h_value) == h2_real_bd_vals.end()){
          h2_real_bd_vals.insert(std::make_pair(h_value,1));
        }
        else{
          h2_real_bd_vals[h_value]++;
        }
      }
      for (auto it = nbs->queue.backwardQueue.elements.begin(); it != nbs->queue.backwardQueue.elements.end(); it++){
        int h_value = (*it).h;
        if (h2_real_bd_vals.find(h_value) == h2_real_bd_vals.end()){
          h2_real_bd_vals.insert(std::make_pair(h_value,1));
        }
        else{
          h2_real_bd_vals[h_value]++;
        }
      }
      delete nbs;
      for (auto p : *toClean_b2)
       {
         delete p;
       } 
      delete toClean_b2;
  }
  
  for (int i = 0; i< 10000000; i++){
    
    MNPuzzleState<4, 4> s;
    s.Reset();
    std::random_shuffle(std::begin(s.puzzle),std::end(s.puzzle));
    s.FinishUnranking();
    int h_1_val = (int) hf_1.HCost(s,goal);
    int h_2_val = (int) hf_2.HCost(s,goal);
    if (h1_overall_vals.find(h_1_val) == h1_overall_vals.end()){
      h1_overall_vals.insert(std::make_pair(h_1_val,1));
    }
    else{
      h1_overall_vals[h_1_val]++;
    }
    if (h2_overall_vals.find(h_2_val) == h2_overall_vals.end()){
      h2_overall_vals.insert(std::make_pair(h_2_val,1));
    }
    else{
      h2_overall_vals[h_2_val]++;
    }
  }
  cout << "h1_overall_vals" << std::endl;
	for(auto elem : h1_overall_vals)
  {
     std::cout << elem.first << " " << elem.second  << "\n";
  }
  cout << "h2_overall_vals" << std::endl;
	for(auto elem : h2_overall_vals)
  {
     std::cout << elem.first << " " << elem.second  << "\n";
  }	
  cout << "h1_real_vals" << std::endl;
	for(auto elem : h1_real_vals)
  {
     std::cout << elem.first << " " << elem.second  << "\n";
  }
  cout << "h1_real_bd_vals" << std::endl;
	for(auto elem : h1_real_bd_vals)
  {
     std::cout << elem.first << " " << elem.second  << "\n";
  }	
  cout << "h2_real_vals" << std::endl;
	for(auto elem : h2_real_vals)
  {
     std::cout << elem.first << " " << elem.second  << "\n";
  }
  cout << "h2_real_bd_vals" << std::endl;
	for(auto elem : h2_real_bd_vals)
  {
     std::cout << elem.first << " " << elem.second  << "\n";
  }	
  
  for (auto p : *toClean_f1)
   {
     delete p;
   } 
  delete toClean_f1;
  
  for (auto p : *toClean_f2)
   {
     delete p;
   } 
  delete toClean_f2;
}

void TestSTP(int algorithm)
{
	NBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>> nbs;
	MM<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>> mm;
	BSStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>> bs;
	TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>> astar;
	MNPuzzle<4,4> mnp;
	
	Heuristic<MNPuzzleState<4, 4>> h_f;
	MNPuzzleState<4, 4> start, goal;
	MakePDBs(goal, h_f, mnp);

	
	for (int x = 0; x < 100; x++) // 547 to 540
	{
		Heuristic<MNPuzzleState<4, 4>> h_b;
		printf("Problem %d of %d\n", x+1, 100);
		
		std::vector<MNPuzzleState<4,4>> nbsPath;
		std::vector<MNPuzzleState<4,4>> astarPath;
		Timer t1, t2;

		start = STP::GetKorfInstance(x);
		goal.Reset();
		MakePDBs(start, h_b, mnp);

		if (algorithm == -1 || algorithm == 10) // Optimal Analysis
		{
			start = STP::GetKorfInstance(x);
			goal.Reset();

			std::string t = "/Users/nathanst/bidir/stp/stp_";
			t += std::to_string(x+1);
			
			BidirectionalProblemAnalyzer<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> p(start, goal, &mnp, &h_f, &h_b);
//			p.drawFullGraph = true;
//			p.drawProblemInstance = false;
//			p.drawMinimumVC = true;
//			p.drawAllG = false;
//			p.drawStatistics = false;
//			p.SaveSVG((t+"-full.svg").c_str());
//			p.drawFullGraph = false;
//			p.drawProblemInstance = false;
//			p.drawAllG = true;
//			p.drawStatistics = false;
//			p.SaveSVG((t+"-min.svg").c_str());
//			printf("Forward: %d\n", p.GetForwardWork());
//			printf("Backward: %d\n", p.GetBackwardWork());
//			printf("Minimum: %d\n", p.GetMinWork());
//			int maxg = p.GetNumGCosts();
//			p.SaveSVG((t+"-shrunk.svg").c_str(), (maxg+11)/12);

		}

		if (algorithm == 0 || algorithm == 10) // A*
		{
			goal.Reset();
			start = STP::GetKorfInstance(x);
			t1.StartTimer();
			astar.SetHeuristic(&h_f);
			astar.GetPath(&mnp, start, goal, astarPath);
			t1.EndTimer();
			printf("A* found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", mnp.GetPathLength(astarPath),
				   astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), astar.GetNodesTouched(), t1.GetElapsedTime());
		}
		if (algorithm == 1) // BS*
		{
			goal.Reset();
			start = STP::GetKorfInstance(x);
			t2.StartTimer();
			bs.GetPath(&mnp, start, goal, &mnp, &mnp, nbsPath);
			t2.EndTimer();
			printf("BS* found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", mnp.GetPathLength(nbsPath),
				   bs.GetNodesExpanded(), bs.GetNecessaryExpansions(), bs.GetNodesTouched(), t2.GetElapsedTime());
		}
		if (algorithm == 2) // MM
		{
			goal.Reset();
			start = STP::GetKorfInstance(x);
			t2.StartTimer();
			mm.GetPath(&mnp, start, goal, &mnp, &mnp, nbsPath);
			t2.EndTimer();
			printf("MM found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", mnp.GetPathLength(nbsPath),
				   mm.GetNodesExpanded(), mm.GetNecessaryExpansions(), mm.GetNodesTouched(), t2.GetElapsedTime());
		}
		if (algorithm == 3||algorithm == 10) // NBS
		{
			goal.Reset();
			start = STP::GetKorfInstance(x);
			t2.StartTimer();
			nbs.GetPath(&mnp, start, goal, &h_f, &h_b, nbsPath);
			t2.EndTimer();
			printf("NBS found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", mnp.GetPathLength(nbsPath),
				   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), nbs.GetNodesTouched(), t2.GetElapsedTime());
		}
		if (algorithm == 4) // MM0
		{
			ZeroHeuristic<MNPuzzleState<4,4>> z;
			goal.Reset();
			start = STP::GetKorfInstance(x);
			t2.StartTimer();
			mm.GetPath(&mnp, start, goal, &z, &z, nbsPath);
			t2.EndTimer();
			printf("MM found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", mnp.GetPathLength(nbsPath),
				   mm.GetNodesExpanded(), mm.GetNecessaryExpansions(), mm.GetNodesTouched(), t2.GetElapsedTime());
		}
		

		delete h_b.heuristics[1];
		delete h_b.heuristics[2];
		delete h_b.heuristics[3];
//		delete h_b.heuristics[4];
//
//		std::cout << astar.GetNodesExpanded() << "\t" << nbs.GetNodesExpanded() << "\t";
//		std::cout << t1.GetElapsedTime() << "\t" <<  t2.GetElapsedTime() << "\n";
	}
	exit(0);
}

void TestSTPFull()
{
	NBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>> nbs;
	MM<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>> mm;
	MNPuzzle<4,4> mnp;
	IDAStar<MNPuzzleState<4,4>, slideDir> ida;
	TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>> astar;

	for (int x = 0; x < 100; x++) // 547 to 540
	{
		MNPuzzleState<4, 4> start, goal;
		printf("Problem %d of %d\n", x+1, 100);
		
		std::vector<slideDir> idaPath;
		std::vector<MNPuzzleState<4,4>> nbsPath;
		std::vector<MNPuzzleState<4,4>> astarPath;
		std::vector<MNPuzzleState<4,4>> mmPath;
		Timer t1, t2, t3, t4;
		
		goal.Reset();
		start = STP::GetKorfInstance(x);
		t1.StartTimer();
		ida.GetPath(&mnp, start, goal, idaPath);
		t1.EndTimer();
		printf("IDA* found path length %ld; %llu expanded; %1.2fs elapsed\n", idaPath.size(),  ida.GetNodesExpanded(), t1.GetElapsedTime());

		goal.Reset();
		start = STP::GetKorfInstance(x);
		t2.StartTimer();
		astar.GetPath(&mnp, start, goal, astarPath);
		t2.EndTimer();
		printf("A* found path length %ld; %llu expanded; %1.2fs elapsed\n", astarPath.size()-1,  astar.GetNodesExpanded(), t2.GetElapsedTime());

		goal.Reset();
		start = STP::GetKorfInstance(x);
		t3.StartTimer();
		nbs.GetPath(&mnp, start, goal, &mnp, &mnp, nbsPath);
		t3.EndTimer();
		printf("NBS found path length %ld; %llu expanded; %1.2fs elapsed\n", nbsPath.size()-1,  nbs.GetNodesExpanded(), t3.GetElapsedTime());

		goal.Reset();
		start = STP::GetKorfInstance(x);
		t4.StartTimer();
		mm.GetPath(&mnp, start, goal, &mnp, &mnp, mmPath);
		t4.EndTimer();
		printf("MM found path length %ld; %llu expanded; %1.2fs elapsed\n", mmPath.size()-1,  mm.GetNodesExpanded(), t3.GetElapsedTime());


		std::cout << ida.GetNodesExpanded() << "\t" <<  astar.GetNodesExpanded() << "\t" << nbs.GetNodesExpanded() << "\t";
		std::cout << t1.GetElapsedTime() << "\t" <<  t2.GetElapsedTime() << "\t" << t3.GetElapsedTime() << "\n";
		
		//if (!fequal)
		if (nbsPath.size() != idaPath.size()+1)
		{
			std::cout << "error solution cost:\t expected cost\n";
			std::cout << nbsPath.size() << "\t" << idaPath.size() << "\n";
//			double d;
//			for (auto x : correctPath)
//			{
//				astar.GetClosedListGCost(x, d);
//				auto t = nbs.GetNodeForwardLocation(x);
//				auto u = nbs.GetNodeBackwardLocation(x);
//				std::cout << x << " is on " << t << " and " << u << "\n";
//				std::cout << "True g: " << d;
//				if (t != kUnseen)
//					std::cout << " forward g: " << nbs.GetNodeForwardG(x);
//				if (u != kUnseen)
//					std::cout << " backward g: " << nbs.GetNodeBackwardG(x);
//				std::cout << "\n";
//			}
			exit(0);
		}
		
	}
	exit(0);
}
