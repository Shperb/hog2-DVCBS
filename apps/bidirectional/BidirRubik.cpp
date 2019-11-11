//
//  BidirRubik.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/15/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#include "BidirRubik.h"
#include "RubiksCube.h"
#include "RubiksInstances.h"
#include "TemplateAStar.h"
#include "NBS.h"
#include "MM.h"
#include "BSStar.h"
#include "WeightedVertexGraph.h"

enum heuristicType {
	kNone,
	k444,
	kSmall,
	k888,
	k1997,
	k839,
	k8210,
	kCustom,
  k7edges,
  k6max6edges,
  k4max6edges
};

const char *hprefix = "/Users/nathanst/Data/pdb/";

void BuildHeuristics(RubiksState goal, Heuristic<RubiksState> &result, heuristicType h)
{
	RubiksCube cube;
	std::vector<int> blank;
	
	switch (h)
	{
		case kNone:
		{
			ZeroHeuristic<RubiksState> *zero = new ZeroHeuristic<RubiksState>();
			result.lookups.push_back({kLeafNode, 0, 0});
			result.heuristics.push_back(zero);
			break;
		}
		case k444:
		{
			std::vector<int> edges1 = {1, 3, 8, 9}; // first 4
			std::vector<int> edges2 = {0, 2, 4, 5}; // first 4
			std::vector<int> corners = {0, 1, 2, 3}; // first 4
			RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
			RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
			RubikPDB *pdb3 = new RubikPDB(&cube, goal, blank, corners);
			if (!pdb1->Load(hprefix))
			{
				pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb1->Save(hprefix);
			}
			if (!pdb2->Load(hprefix))
			{
				pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb2->Save(hprefix);
			}
			if (!pdb3->Load(hprefix))
			{
				pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb3->Save(hprefix);
			}
			result.lookups.push_back({kMaxNode, 1, 3});
			result.lookups.push_back({kLeafNode, 0, 0});
			result.lookups.push_back({kLeafNode, 1, 0});
			result.lookups.push_back({kLeafNode, 2, 0});
			result.heuristics.push_back(pdb1);
			result.heuristics.push_back(pdb2);
			result.heuristics.push_back(pdb3);
			break;
		}
		case kSmall:
		{
			assert(!"PDB not being saved!");
			std::vector<int> edges1 = {0, 1, 2, 4, 6};
			std::vector<int> edges2 = {3, 5};
			std::vector<int> edges3 = {7, 8, 9, 10, 11};
			std::vector<int> corners1 = {0, 1, 2, 3, 4, 5};
			std::vector<int> corners2 = {2, 3, 4, 5, 6, 7};
			RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
			RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
			RubikPDB *pdb3 = new RubikPDB(&cube, goal, edges3, blank);
			RubikPDB *pdb4 = new RubikPDB(&cube, goal, blank, corners1);
			RubikPDB *pdb5 = new RubikPDB(&cube, goal, blank, corners2);
			pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
			pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
			pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
			pdb4->BuildPDB(goal, std::thread::hardware_concurrency());
			pdb5->BuildPDB(goal, std::thread::hardware_concurrency());
			result.lookups.push_back({kMaxNode, 1, 5});
			result.lookups.push_back({kLeafNode, 0, 0});
			result.lookups.push_back({kLeafNode, 1, 0});
			result.lookups.push_back({kLeafNode, 2, 0});
			result.lookups.push_back({kLeafNode, 3, 0});
			result.lookups.push_back({kLeafNode, 4, 0});
			result.heuristics.push_back(pdb1);
			result.heuristics.push_back(pdb2);
			result.heuristics.push_back(pdb3);
			result.heuristics.push_back(pdb4);
			result.heuristics.push_back(pdb5);
			break;
		}
		case k1997:
		{
			std::vector<int> edges1 = {1, 3, 8, 9, 10, 11};
			std::vector<int> edges2 = {0, 2, 4, 5, 6, 7};
			std::vector<int> corners = {0, 1, 2, 3, 4, 5, 6, 7};
			RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
			RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
			RubikPDB *pdb3 = new RubikPDB(&cube, goal, blank, corners);
			
			if (!pdb1->Load(hprefix))
			{
				pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb1->Save(hprefix);
			}
			else {
				printf("Loaded previous heuristic\n");
			}
			if (!pdb2->Load(hprefix))
			{
				pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb2->Save(hprefix);
			}
			else {
				printf("Loaded previous heuristic\n");
			}
			if (!pdb3->Load(hprefix))
			{
				pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb3->Save(hprefix);
			}
			else {
				printf("Loaded previous heuristic\n");
			}
			result.lookups.push_back({kMaxNode, 1, 3});
			result.lookups.push_back({kLeafNode, 0, 0});
			result.lookups.push_back({kLeafNode, 1, 0});
			result.lookups.push_back({kLeafNode, 2, 0});
			result.heuristics.push_back(pdb1);
			result.heuristics.push_back(pdb2);
			result.heuristics.push_back(pdb3);
			break;
		}
		case k888:
		{
			std::vector<int> edges1 = {0, 1, 2, 3, 4, 5, 6, 7};
			std::vector<int> edges2 = {1, 3, 5, 7, 8, 9, 10, 11};
			std::vector<int> corners = {0, 1, 2, 3, 4, 5, 6, 7}; // first 4
			RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
			RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
			RubikPDB *pdb3 = new RubikPDB(&cube, goal, blank, corners);
			if (!pdb1->Load(hprefix))
			{
				pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb1->Save(hprefix);
			}
			if (!pdb2->Load(hprefix))
			{
				pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb2->Save(hprefix);
			}
			if (!pdb3->Load(hprefix))
			{
				pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb3->Save(hprefix);
			}
			result.lookups.push_back({kMaxNode, 1, 3});
			result.lookups.push_back({kLeafNode, 0, 0});
			result.lookups.push_back({kLeafNode, 1, 0});
			result.lookups.push_back({kLeafNode, 2, 0});
			result.heuristics.push_back(pdb1);
			result.heuristics.push_back(pdb2);
			result.heuristics.push_back(pdb3);
			break;
		}
		case k839:
		{
			std::vector<int> edges1 = {0, 1, 2, 3, 4, 5, 6, 7, 8};
			std::vector<int> edges2 = {9, 10, 11};
			std::vector<int> corners = {0, 1, 2, 3, 4, 5, 6, 7};
			RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
			RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
			RubikPDB *pdb3 = new RubikPDB(&cube, goal, blank, corners);
			if (!pdb1->Load(hprefix))
			{
				pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb1->Save(hprefix);
			}
			if (!pdb2->Load(hprefix))
			{
				pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb2->Save(hprefix);
			}
			if (!pdb3->Load(hprefix))
			{
				pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb3->Save(hprefix);
			}
			result.lookups.push_back({kMaxNode, 1, 3});
			result.lookups.push_back({kLeafNode, 0, 0});
			result.lookups.push_back({kLeafNode, 1, 0});
			result.lookups.push_back({kLeafNode, 2, 0});
			result.heuristics.push_back(pdb1);
			result.heuristics.push_back(pdb2);
			result.heuristics.push_back(pdb3);
			break;
		}
		case k8210:
		{
			std::vector<int> edges1 = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
			std::vector<int> edges2 = {10, 11};
			std::vector<int> corners = {0, 1, 2, 3, 4, 5, 6, 7};
			RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
			RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
			RubikPDB *pdb3 = new RubikPDB(&cube, goal, blank, corners);
			if (!pdb1->Load(hprefix))
			{
				pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb1->Save(hprefix);
			}
			if (!pdb2->Load(hprefix))
			{
				pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb2->Save(hprefix);
			}
			if (!pdb3->Load(hprefix))
			{
				pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb3->Save(hprefix);
			}
			result.lookups.push_back({kMaxNode, 1, 3});
			result.lookups.push_back({kLeafNode, 0, 0});
			result.lookups.push_back({kLeafNode, 1, 0});
			result.lookups.push_back({kLeafNode, 2, 0});
			result.heuristics.push_back(pdb1);
			result.heuristics.push_back(pdb2);
			result.heuristics.push_back(pdb3);
			break;
		}
		case kCustom:
		{
			std::vector<int> edges1 = {0, 1, 2, 3};
			std::vector<int> edges2 = {4, 5, 6, 7};
			std::vector<int> edges3 = {8, 9};
			std::vector<int> edges4 = {10, 11};
			std::vector<int> corners = {0, 1, 2, 3, 4, 5, 6, 7};
			RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
			RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
			RubikPDB *pdb3 = new RubikPDB(&cube, goal, blank, corners);
			RubikPDB *pdb4 = new RubikPDB(&cube, goal, blank, corners);
			RubikPDB *pdb5 = new RubikPDB(&cube, goal, blank, corners);
			if (!pdb1->Load(hprefix))
			{
				pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb1->Save(hprefix);
			}
			if (!pdb2->Load(hprefix))
			{
				pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb2->Save(hprefix);
			}
			if (!pdb3->Load(hprefix))
			{
				pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb3->Save(hprefix);
			}
			if (!pdb4->Load(hprefix))
			{
				pdb4->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb4->Save(hprefix);
			}
			if (!pdb5->Load(hprefix))
			{
				pdb5->BuildPDB(goal, std::thread::hardware_concurrency());
				pdb5->Save(hprefix);
			}
			result.lookups.push_back({kMaxNode, 1, 5});
			result.lookups.push_back({kLeafNode, 0, 0});
			result.lookups.push_back({kLeafNode, 1, 0});
			result.lookups.push_back({kLeafNode, 2, 0});
			result.lookups.push_back({kLeafNode, 3, 0});
			result.lookups.push_back({kLeafNode, 4, 0});
			result.heuristics.push_back(pdb1);
			result.heuristics.push_back(pdb2);
			result.heuristics.push_back(pdb3);
			result.heuristics.push_back(pdb4);
			result.heuristics.push_back(pdb5);
			break;
		}
    case k7edges:
		{
      std::cout << "building 7edges pdb" << std::endl;
			std::vector<int> edges = {0,1,2,3,4,5,6};
			RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges, blank);
      pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
			//result.lookups.push_back({kMaxNode, 1, 3});
			result.lookups.push_back({kLeafNode, 0, 0});
			//result.lookups.push_back({kLeafNode, 1, 0});
			//result.lookups.push_back({kLeafNode, 2, 0});
			result.heuristics.push_back(pdb1);
			//result.heuristics.push_back(pdb2);
			//result.heuristics.push_back(pdb3);
			break;
		}
    case k6max6edges:
		{
      std::cout << "building max6edges pdb" << std::endl;
			std::vector<int> edges1 = {0,1,2,3,4,5}; 
      std::vector<int> edges2 = {6,7,8,9,10,11};
      std::vector<int> edges3 = {0,1,2,6,7,8}; 
      std::vector<int> edges4 = {3,4,5,9,10,11};
      std::vector<int> edges5 = {0,11,5,6,3,8}; 
      std::vector<int> edges6 = {1,4,2,9,10,7};
			RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
      pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
      RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
      pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
      RubikPDB *pdb3 = new RubikPDB(&cube, goal, edges3, blank);
      pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
      RubikPDB *pdb4 = new RubikPDB(&cube, goal, edges4, blank);
      pdb4->BuildPDB(goal, std::thread::hardware_concurrency());
      RubikPDB *pdb5 = new RubikPDB(&cube, goal, edges5, blank);
      pdb5->BuildPDB(goal, std::thread::hardware_concurrency());
      RubikPDB *pdb6 = new RubikPDB(&cube, goal, edges6, blank);
      pdb6->BuildPDB(goal, std::thread::hardware_concurrency());
			result.lookups.push_back({kMaxNode, 1, 6});
			result.lookups.push_back({kLeafNode, 0, 0});
			result.lookups.push_back({kLeafNode, 1, 0});
			result.lookups.push_back({kLeafNode, 2, 0});
			result.lookups.push_back({kLeafNode, 3, 0});
			result.lookups.push_back({kLeafNode, 4, 0});
			result.lookups.push_back({kLeafNode, 5, 0});
			result.heuristics.push_back(pdb1);
			result.heuristics.push_back(pdb2);
			result.heuristics.push_back(pdb3);
			result.heuristics.push_back(pdb4);
			result.heuristics.push_back(pdb5);
			result.heuristics.push_back(pdb6);
			break;
		}
    case k4max6edges:
		{
      std::cout << "building max6edges pdb" << std::endl;
			std::vector<int> edges1 = {0,1,2,3,4,5}; 
      std::vector<int> edges2 = {6,7,8,9,10,11};
      std::vector<int> edges3 = {0,1,2,6,7,8}; 
      std::vector<int> edges4 = {3,4,5,9,10,11};
			RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
      pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
      RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
      pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
      RubikPDB *pdb3 = new RubikPDB(&cube, goal, edges3, blank);
      pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
      RubikPDB *pdb4 = new RubikPDB(&cube, goal, edges4, blank);
      pdb4->BuildPDB(goal, std::thread::hardware_concurrency());
			result.lookups.push_back({kMaxNode, 1, 6});
			result.lookups.push_back({kLeafNode, 0, 0});
			result.lookups.push_back({kLeafNode, 1, 0});
			result.lookups.push_back({kLeafNode, 2, 0});
			result.lookups.push_back({kLeafNode, 3, 0});
			result.heuristics.push_back(pdb1);
			result.heuristics.push_back(pdb2);
			result.heuristics.push_back(pdb3);
			result.heuristics.push_back(pdb4);
			break;
		}
	}
}

void TestRubikHeuristicMaxsmallVSBig(){
  std::map<double,int> h1_overall_vals;
  std::map<double,int> h2_overall_vals;
  std::map<double,int> h1_astar_vals;
  std::map<double,int> h2_astar_vals;
  std::map<double,int> h1_rastar_vals;
  std::map<double,int> h2_rastar_vals;
  std::map<double,int> h1_real_bd_vals;
  std::map<double,int> h2_real_bd_vals;
  const int walkLength = 12;

	RubiksCube cube;
	RubiksState start, goal;
  
  std::vector<RubiksState> nbsPath;
  std::vector<RubiksState> astarPath;
	NBS<RubiksState, RubiksAction, RubiksCube> nbs;
	TemplateAStar<RubiksState, RubiksAction, RubiksCube> astar;
  goal.Reset();
  
  Heuristic<RubiksState> hf_7;
  Heuristic<RubiksState> hf_max6;
  Heuristic<RubiksState> hb_7;
  Heuristic<RubiksState> hb_max6;
  BuildHeuristics(goal, hf_7, k7edges);
  BuildHeuristics(goal, hf_max6, k4max6edges);
	hb_7 = hf_7;
	for (int x = 0; x < hb_7.heuristics.size(); x++)
	{
		hb_7.heuristics[x] = new RubikArbitraryGoalPDB((RubikPDB*)hb_7.heuristics[x]);
	}
	hb_max6 = hf_max6;
	for (int x = 0; x < hb_max6.heuristics.size(); x++)
	{
		hb_max6.heuristics[x] = new RubikArbitraryGoalPDB((RubikPDB*)hb_max6.heuristics[x]);
	}
  Timer t1;
  std::cout << "starting to solve" << std::endl;
  srandom(2017218);
  for (int x = 0; x < 50; x++)
  {    
    RubiksCubeInstances::GetRandomN(start, walkLength, x);
    goal.Reset();
    
    //BidirectionalProblemAnalyzer<RubiksState, RubiksAction, RubiksCube>::GetWeightedVertexGraph(start, goal, &cube, &hf_7, &hb_7,(std::to_string(x)+"_7edges.svg").c_str());
    //BidirectionalProblemAnalyzer<RubiksState, RubiksAction, RubiksCube>::GetWeightedVertexGraph(start, goal, &cube, &hf_max6, &hb_max6,(std::to_string(x)+"_max6edges.svg").c_str());
    if (1) //get MVC
    {
      BidirectionalProblemAnalyzer<RubiksState, RubiksAction, RubiksCube> p(start, goal, &cube, &hf_7, &hb_7);
      p.drawProblemInstance = false;
      p.drawStatistics = false;
      p.drawAllG = true;
      p.flipBackwardsGCost = true;
      p.SaveSVG((std::to_string(x)+"_7edges.svg").c_str());

      BidirectionalProblemAnalyzer<RubiksState, RubiksAction, RubiksCube> p2(start, goal, &cube, &hf_max6, &hb_max6);
      p2.drawProblemInstance = false;
      p2.drawStatistics = false;
      p2.drawAllG = true;
      p2.flipBackwardsGCost = true;
      p2.SaveSVG((std::to_string(x)+"_max6edges.svg").c_str());
    }
    if (0) //solve
    {
      t1.StartTimer();
      astar.SetHeuristic(&hf_7);
      astar.GetPath(&cube, start, goal, astarPath);
      t1.EndTimer();
      printf("A* 7edges found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", cube.GetPathLength(astarPath),
           astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), astar.GetNodesTouched(), t1.GetElapsedTime());
      for (int x = 0; x < astar.GetNumItems(); x++)
      {
        const auto &i = astar.GetItem(x);
        if (i.where != kClosedList)
          continue;
        double h_value = i.h;
        if (h1_astar_vals.find(h_value) == h1_astar_vals.end()){
          h1_astar_vals.insert(std::make_pair(h_value,1));
        }
        else{
          h1_astar_vals[h_value]++;
        }
      }
      t1.StartTimer();
      astar.SetHeuristic(&hb_7);
      astar.GetPath(&cube, goal, start, astarPath);
      t1.EndTimer();
      printf("r-A* GAP\\2 found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", cube.GetPathLength(astarPath),
           astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), astar.GetNodesTouched(), t1.GetElapsedTime());
      for (int x = 0; x < astar.GetNumItems(); x++)
      {
        const auto &i = astar.GetItem(x);
        if (i.where != kClosedList)
          continue;
        double h_value = i.h;
        if (h1_rastar_vals.find(h_value) == h1_rastar_vals.end()){
          h1_rastar_vals.insert(std::make_pair(h_value,1));
        }
        else{
          h1_rastar_vals[h_value]++;
        }
      }
      t1.StartTimer();
      nbs.GetPath(&cube, start, goal, &hf_7, &hb_7, nbsPath);
      t1.EndTimer();
      printf("NBS GAP\\2 found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", cube.GetPathLength(nbsPath),
           nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), nbs.GetNodesTouched(), t1.GetElapsedTime());
           
      for (int x = 0; x < nbs.GetNumForwardItems(); x++)
      {
        const auto &i = nbs.GetForwardItem(x);
        if (i.where != kClosed)
          continue;
        double h_value = i.h;
        if (h1_real_bd_vals.find(h_value) == h1_real_bd_vals.end()){
          h1_real_bd_vals.insert(std::make_pair(h_value,1));
        }
        else{
          h1_real_bd_vals[h_value]++;
        }
      }
      for (int x = 0; x < nbs.GetNumBackwardItems(); x++)
      {
        const auto &i = nbs.GetBackwardItem(x);
        if (i.where != kClosed)
          continue;
        double h_value = i.h;
        if (h1_real_bd_vals.find(h_value) == h1_real_bd_vals.end()){
          h1_real_bd_vals.insert(std::make_pair(h_value,1));
        }
        else{
          h1_real_bd_vals[h_value]++;
        }
      }
          
      t1.StartTimer();
      astar.SetHeuristic(&hf_max6);
      astar.GetPath(&cube, start, goal, astarPath);
      t1.EndTimer();
      printf("A* 6edges-MAX(6) found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", cube.GetPathLength(astarPath),
           astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), astar.GetNodesTouched(), t1.GetElapsedTime());
      for (int x = 0; x < astar.GetNumItems(); x++)
      {
        const auto &i = astar.GetItem(x);
        if (i.where != kClosedList)
          continue;
        double h_value = i.h;
        if (h2_astar_vals.find(h_value) == h2_astar_vals.end()){
          h2_astar_vals.insert(std::make_pair(h_value,1));
        }
        else{
          h2_astar_vals[h_value]++;
        }
      }
      t1.StartTimer();
      astar.SetHeuristic(&hb_max6);
      astar.GetPath(&cube, goal, start, astarPath);
      t1.EndTimer();
      printf("r-A* 6edges-MAX(6) found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", cube.GetPathLength(astarPath),
           astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), astar.GetNodesTouched(), t1.GetElapsedTime());
      for (int x = 0; x < astar.GetNumItems(); x++)
      {
        const auto &i = astar.GetItem(x);
        if (i.where != kClosedList)
          continue;
        double h_value = i.h;
        if (h2_rastar_vals.find(h_value) == h2_rastar_vals.end()){
          h2_rastar_vals.insert(std::make_pair(h_value,1));
        }
        else{
          h2_rastar_vals[h_value]++;
        }
      }
      t1.StartTimer();
      nbs.GetPath(&cube, start, goal, &hf_max6, &hb_max6, nbsPath);
      t1.EndTimer();
      printf("NBS 6edges-MAX(6) found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", cube.GetPathLength(nbsPath),
           nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), nbs.GetNodesTouched(), t1.GetElapsedTime());
           
      for (int x = 0; x < nbs.GetNumForwardItems(); x++)
      {
        const auto &i = nbs.GetForwardItem(x);
        if (i.where != kClosed)
          continue;
        double h_value = i.h;
        if (h2_real_bd_vals.find(h_value) == h2_real_bd_vals.end()){
          h2_real_bd_vals.insert(std::make_pair(h_value,1));
        }
        else{
          h2_real_bd_vals[h_value]++;
        }
      }
      for (int x = 0; x < nbs.GetNumBackwardItems(); x++)
      {
        const auto &i = nbs.GetBackwardItem(x);
        if (i.where != kClosed)
          continue;
        double h_value = i.h;
        if (h2_real_bd_vals.find(h_value) == h2_real_bd_vals.end()){
          h2_real_bd_vals.insert(std::make_pair(h_value,1));
        }
        else{
          h2_real_bd_vals[h_value]++;
        }
      }
    }
      
  }
  if (0){
    for (int i = 0; i< 10000000; i++){
      start.Reset();
      goal.Reset();
      RubiksCubeInstances::GetRandomN(start, walkLength, i);  

      double h_1_val = hf_7.HCost(start,goal);
      double h_2_val = hf_max6.HCost(start,goal);
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
  cout << "h1_astar_vals" << std::endl;
	for(auto elem : h1_astar_vals)
  {
     std::cout << elem.first << " " << elem.second  << "\n";
  }
  cout << "h1_rastar_vals" << std::endl;
	for(auto elem : h1_rastar_vals)
  {
     std::cout << elem.first << " " << elem.second  << "\n";
  }
  cout << "h1_real_bd_vals" << std::endl;
	for(auto elem : h1_real_bd_vals)
  {
     std::cout << elem.first << " " << elem.second  << "\n";
  }	
  cout << "h2_astar_vals" << std::endl;
	for(auto elem : h2_astar_vals)
  {
     std::cout << elem.first << " " << elem.second  << "\n";
  }
  cout << "h2_rastar_vals" << std::endl;
	for(auto elem : h2_rastar_vals)
  {
     std::cout << elem.first << " " << elem.second  << "\n";
  }
  cout << "h2_real_bd_vals" << std::endl;
	for(auto elem : h2_real_bd_vals)
  {
     std::cout << elem.first << " " << elem.second  << "\n";
  }
}

void TestRubik(int algorithm)
{
	const int walkLength = 14;
	NBS<RubiksState, RubiksAction, RubiksCube> nbs;
	MM<RubiksState, RubiksAction, RubiksCube> mm;
	BSStar<RubiksState, RubiksAction, RubiksCube> bs;
	TemplateAStar<RubiksState, RubiksAction, RubiksCube> astar;
	RubiksCube cube;
	RubiksState start, goal;
	
	Heuristic<RubiksState> h_f;
	Heuristic<RubiksState> h_b;

	BuildHeuristics(goal, h_f, kCustom);
	h_b = h_f;
	for (int x = 0; x < h_b.heuristics.size(); x++)
	{
		h_b.heuristics[x] = new RubikArbitraryGoalPDB((RubikPDB*)h_b.heuristics[x]);
	}

	
	
	for (int x = 0; x < 100; x++) // 547 to 540
	{
		printf("Problem %d of %d\n", x+1, 100);
	
		RubiksCubeInstances::GetRandomN(start, walkLength, x);
		goal.Reset();

		std::vector<RubiksState> nbsPath;
		std::vector<RubiksState> astarPath;
		Timer t1, t2;
		
		if (algorithm == -1 || algorithm == 10) // Optimal Analysis
		{
			std::string t = hprefix;
			t += "RC_w"+std::to_string(walkLength)+"_"+std::to_string(x+1)+".svg";
			
			BidirectionalProblemAnalyzer<RubiksState, RubiksAction, RubiksCube> p(start, goal, &cube, &h_f, &h_b);
			p.drawProblemInstance = false;
			p.drawAllG = true;
			p.flipBackwardsGCost = true;
			//			p.drawFullGraph = true;
			//			p.drawMinimumVC = true;
			//			p.drawStatistics = false;
			//			p.SaveSVG((t+"-full.svg").c_str());
			//			p.drawFullGraph = false;
			//			p.drawProblemInstance = false;
			//			p.drawAllG = true;
			// p.drawStatistics = false;
			//			p.SaveSVG((t+"-min.svg").c_str());
			//			printf("Forward: %d\n", p.GetForwardWork());
			//			printf("Backward: %d\n", p.GetBackwardWork());
			//			printf("Minimum: %d\n", p.GetMinWork());
			//			int maxg = p.GetNumGCosts();
			//p.SaveSVG((t+"-shrunk.svg").c_str(), (maxg+11)/12);
			p.SaveSVG(t.c_str());
			
		}
		
		if (algorithm == 0 || algorithm == 10) // A*
		{
			goal.Reset();
			RubiksCubeInstances::GetRandomN(start, walkLength, x);
			t1.StartTimer();
			astar.SetHeuristic(&h_f);
			astar.GetPath(&cube, start, goal, astarPath);
			t1.EndTimer();
			printf("A* found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", cube.GetPathLength(astarPath),
				   astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), astar.GetNodesTouched(), t1.GetElapsedTime());
		}
		// if (algorithm == 1) // BS*
		// {
			// goal.Reset();
			// RubiksCubeInstances::GetRandomN(start, walkLength, x);
			// t2.StartTimer();
			// bs.GetPath(&cube, start, goal, &cube, &cube, nbsPath);
			// t2.EndTimer();
			// printf("BS* found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", cube.GetPathLength(nbsPath),
				   // bs.GetNodesExpanded(), bs.GetNecessaryExpansions(), bs.GetNodesTouched(), t2.GetElapsedTime());
		// }
		if (algorithm == 2) // MM
		{
			goal.Reset();
			RubiksCubeInstances::GetRandomN(start, walkLength, x);
			t2.StartTimer();
			mm.GetPath(&cube, start, goal, &h_f, &h_b, nbsPath);
			t2.EndTimer();
			printf("MM found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", cube.GetPathLength(nbsPath),
				   mm.GetNodesExpanded(), mm.GetNecessaryExpansions(), mm.GetNodesTouched(), t2.GetElapsedTime());
		}
		if (algorithm == 3||algorithm == 10) // NBS
		{
			goal.Reset();
			RubiksCubeInstances::GetRandomN(start, walkLength, x);
			t2.StartTimer();
			nbs.GetPath(&cube, start, goal, &h_f, &h_b, nbsPath);
			t2.EndTimer();
			printf("NBS found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", cube.GetPathLength(nbsPath),
				   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), nbs.GetNodesTouched(), t2.GetElapsedTime());
		}
		if (algorithm == 4) // MM0
		{
			ZeroHeuristic<RubiksState> z;
			goal.Reset();
			RubiksCubeInstances::GetRandomN(start, walkLength, x);
			t2.StartTimer();
			mm.GetPath(&cube, start, goal, &z, &z, nbsPath);
			t2.EndTimer();
			printf("MM found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", cube.GetPathLength(nbsPath),
				   mm.GetNodesExpanded(), mm.GetNecessaryExpansions(), mm.GetNodesTouched(), t2.GetElapsedTime());
		}
		
	}
	exit(0);
}