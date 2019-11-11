//
//  BidirTOH.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/14/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#include "BidirTOH.h"
#include "TOH.h"
#include "TemplateAStar.h"
#include "NBS.h"
#include "DVCBS.h"
#include "MM.h"
#include "BSStar.h"
#include "WeightedVertexGraph.h"
#include <queue>
#include <unordered_set>
#include "HeuristicError.h"
#include "OnlineStats.h"


//// Builds two PDBs according to the sizes, plus 2 smaller pdbs that also cover the state space
//template <int numDisks, int pdb1Disks, int pdb2Disks = numDisks-pdb1Disks>
//Heuristic<TOHState<numDisks>> *BuildPDB(const TOHState<numDisks> &goal)
//{
//	TOH<numDisks> toh;
//	TOH<pdb1Disks> absToh1;
//	TOH<pdb2Disks> absToh2;
//	TOH<pdb2Disks-2> absToh3;
//	TOH<pdb1Disks+2> absToh4;
//	TOHState<pdb1Disks> absTohState1;
//	TOHState<pdb2Disks> absTohState2;
//
//
//	TOHPDB<pdb1Disks, numDisks, pdb2Disks> *pdb1 = new TOHPDB<pdb1Disks, numDisks, pdb2Disks>(&absToh1, goal); // top disks
//	TOHPDB<pdb2Disks, numDisks> *pdb2 = new TOHPDB<pdb2Disks, numDisks>(&absToh2, goal); // bottom disks
//	TOHPDB<pdb2Disks-2, numDisks, pdb1Disks+2> *pdb3 = new TOHPDB<pdb2Disks-2, numDisks, pdb1Disks+2>(&absToh3, goal); // top disks
//	TOHPDB<pdb1Disks+2, numDisks> *pdb4 = new TOHPDB<pdb1Disks+2, numDisks>(&absToh4, goal); // top disks
//	pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
//	pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
//	pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
//	pdb4->BuildPDB(goal, std::thread::hardware_concurrency());
//
//	Heuristic<TOHState<numDisks>> *h = new Heuristic<TOHState<numDisks>>;
//
//	h->lookups.resize(0);
//
//	h->lookups.push_back({kMaxNode, 1, 2});
//	h->lookups.push_back({kAddNode, 3, 2});
//	h->lookups.push_back({kAddNode, 5, 2});
//	h->lookups.push_back({kLeafNode, 0, 0});
//	h->lookups.push_back({kLeafNode, 1, 1});
//	h->lookups.push_back({kLeafNode, 2, 2});
//	h->lookups.push_back({kLeafNode, 3, 3});
//	h->heuristics.resize(0);
//	h->heuristics.push_back(pdb1);
//	h->heuristics.push_back(pdb2);
//	h->heuristics.push_back(pdb3);
//	h->heuristics.push_back(pdb4);
//
//	return h;
//}

//// Builds two PDBs that are symmetric for the standard goal - but not necessarily for random goals
//template <int numDisks, int pdb1Disks, int pdb2Disks = numDisks-pdb1Disks>
//Heuristic<TOHState<numDisks>> *BuildPDB(const TOHState<numDisks> &goal)
//{
//	TOH<numDisks> toh;
//	TOH<pdb1Disks> absToh1;
//	TOH<pdb2Disks> absToh2;
//	TOHState<pdb1Disks> absTohState1;
//	TOHState<pdb2Disks> absTohState2;
//
//
//	TOHPDB<pdb1Disks, numDisks, pdb2Disks> *pdb1 = new TOHPDB<pdb1Disks, numDisks, pdb2Disks>(&absToh1, goal); // top disks
//	TOHPDB<pdb2Disks, numDisks> *pdb2 = new TOHPDB<pdb2Disks, numDisks>(&absToh2, goal); // bottom disks
//	TOHPDB<pdb2Disks, numDisks, pdb1Disks> *pdb3 = new TOHPDB<pdb2Disks, numDisks, pdb1Disks>(&absToh2, goal); // top disks
//	TOHPDB<pdb1Disks, numDisks> *pdb4 = new TOHPDB<pdb1Disks, numDisks>(&absToh1, goal); // bottom disks
//	pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
//	pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
//	pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
//	pdb4->BuildPDB(goal, std::thread::hardware_concurrency());
//
//	Heuristic<TOHState<numDisks>> *h = new Heuristic<TOHState<numDisks>>;
//
//	h->lookups.resize(0);
//
//	h->lookups.push_back({kMaxNode, 1, 2});
//	h->lookups.push_back({kAddNode, 3, 2});
//	h->lookups.push_back({kAddNode, 5, 2});
//	h->lookups.push_back({kLeafNode, 0, 0});
//	h->lookups.push_back({kLeafNode, 1, 1});
//	h->lookups.push_back({kLeafNode, 2, 2});
//	h->lookups.push_back({kLeafNode, 3, 3});
//	h->heuristics.resize(0);
//	h->heuristics.push_back(pdb1);
//	h->heuristics.push_back(pdb2);
//	h->heuristics.push_back(pdb3);
//	h->heuristics.push_back(pdb4);
//
//	return h;
//}

// Builds two PDBs that are symmetric for the standard goal - but not necessarily for random goals
template <int numDisks, int pdb1Disks, int pdb2Disks = numDisks-pdb1Disks>
Heuristic<TOHState<numDisks>> *BuildPDB(const TOHState<numDisks> &goal)
{
	TOH<numDisks> toh;
	TOH<pdb1Disks> absToh1;
	TOH<pdb2Disks> absToh2;
	TOHState<pdb1Disks> absTohState1;
	TOHState<pdb2Disks> absTohState2;
	
	
	TOHPDB<pdb1Disks, numDisks, pdb2Disks> *pdb1 = new TOHPDB<pdb1Disks, numDisks, pdb2Disks>(&absToh1, goal); // top disks
	TOHPDB<pdb2Disks, numDisks> *pdb2 = new TOHPDB<pdb2Disks, numDisks>(&absToh2, goal); // bottom disks
	pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
	pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
	
	Heuristic<TOHState<numDisks>> *h = new Heuristic<TOHState<numDisks>>;
	
	h->lookups.resize(0);
	
	h->lookups.push_back({kAddNode, 1, 2});
	h->lookups.push_back({kLeafNode, 0, 0});
	h->lookups.push_back({kLeafNode, 1, 1});
	h->heuristics.resize(0);
	h->heuristics.push_back(pdb1);
	h->heuristics.push_back(pdb2);
	
	return h;
}


template <int N, int pdb1Disks>
void TestTOH(int first, int last)
{
	TemplateAStar<TOHState<N>, TOHMove, TOH<N>> astar;
	NBS<TOHState<N>, TOHMove, TOH<N>, NBSQueue<TOHState<N>, 0>> nbse0;
	NBS<TOHState<N>, TOHMove, TOH<N>, NBSQueue<TOHState<N>, 1>> nbse1;
	MM<TOHState<N>, TOHMove, TOH<N>> mm;
	BSStar<TOHState<N>, TOHMove, TOH<N>> bs;

	TOH<N> toh;
	TOHState<N> s;
	TOHState<N> g;
	std::vector<TOHState<N>> thePath;
	std::vector<TOHMove> actionPath;
	Heuristic<TOHState<N>> *f;
	Heuristic<TOHState<N>> *b;
	ZeroHeuristic<TOHState<N>> z;
//	g.Reset();
//	f = BuildPDB<N, pdb1Disks>(g);
	
	int table[] = {52058078,116173544,208694125,131936966,141559500,133800745,194246206,50028346,167007978,207116816,163867037,119897198,201847476,210859515,117688410,121633885};
	int table2[] = {145008714,165971878,154717942,218927374,182772845,5808407,19155194,137438954,13143598,124513215,132635260,39667704,2462244,41006424,214146208,54305743};
	for (int count = first; count < last; count++)
	{
		printf("Seed: %d\n", table[count&0xF]^table2[(count>>4)&0xF]);
		srandom(table[count&0xF]^table2[(count>>4)&0xF]);

		s.counts[0] = s.counts[1] = s.counts[2] = s.counts[3] = 0;
		for (int x = N; x > 0; x--)
		{
			int whichPeg = random()%4;
			s.disks[whichPeg][s.counts[whichPeg]] = x;
			s.counts[whichPeg]++;
		}
		b = BuildPDB<N, pdb1Disks>(s);

		g.counts[0] = g.counts[1] = g.counts[2] = g.counts[3] = 0;
		for (int x = N; x > 0; x--)
		{
			int whichPeg = random()%4;
			g.disks[whichPeg][g.counts[whichPeg]] = x;
			g.counts[whichPeg]++;
		}
		// Using canonical goal currently - comment to use random goal
		//g.Reset();
		f = BuildPDB<N, pdb1Disks>(g);

		Timer timer;
	
//		printf("Starting heuristics: %f %f\n", f->HCost(s, g), b->HCost(g, s));
		
		std::cout << s << "\n";
		std::cout << g << "\n";

		if (0)
		{
			printf("-=-=-NBSe0-=-=-\n");
			timer.StartTimer();
			nbse0.GetPath(&toh, s, g, f, b, thePath);
			timer.EndTimer();
			printf("I%d-%d-%d\t%d\t", N, pdb1Disks, count, (int)toh.GetPathLength(thePath));
			printf("%llu nodes\t%llu necessary\t", nbse0.GetNodesExpanded(), nbse0.GetNecessaryExpansions());
			printf("%1.2fs elapsed\n", timer.GetElapsedTime());
		}
		if (0)
		{
			printf("-=-=-NBSe1-=-=-\n");
			timer.StartTimer();
			nbse1.GetPath(&toh, s, g, f, b, thePath);
			timer.EndTimer();
			printf("I%d-%d-%d\t%d\t", N, pdb1Disks, count, (int)toh.GetPathLength(thePath));
			printf("%llu nodes\t%llu necessary\t", nbse1.GetNodesExpanded(), nbse1.GetNecessaryExpansions());
			printf("%1.2fs elapsed\n", timer.GetElapsedTime());
		}
		if (0)
		{
			printf("-=-=-NBS0e1-=-=-\n");
			timer.StartTimer();
			nbse1.GetPath(&toh, s, g, &z, &z, thePath);
			timer.EndTimer();
			printf("I%d-%d-%d\t%d\t", N, pdb1Disks, count, (int)toh.GetPathLength(thePath));
			printf("%llu nodes\t%llu necessary\t", nbse1.GetNodesExpanded(), nbse1.GetNecessaryExpansions());
			printf("%1.2fs elapsed\n", timer.GetElapsedTime());
		}
		if (1)
		{
			BidirectionalProblemAnalyzer<TOHState<N>, TOHMove, TOH<N>>::GetWeightedVertexGraph(s, g, &toh, f, b);
		}
//		if (1)
//		{
//			printf("-=-=-BS-=-=-\n");
//			timer.StartTimer();
//			bs.GetPath(&toh, s, g, f, b, thePath);
//			timer.EndTimer();
//			printf("I%d-%d-%d\t%d\t", N, pdb1Disks, count, (int)toh.GetPathLength(thePath));
//			printf("%llu nodes\t%llu necessary\t", bs.GetNodesExpanded(), bs.GetNecessaryExpansions());
//			printf("%1.2fs elapsed\n", timer.GetElapsedTime());
//		}
//		if (1)
//		{
//			printf("-=-=-MM-=-=-\n");
//			timer.StartTimer();
//			mm.GetPath(&toh, s, g, f, b, thePath);
//			timer.EndTimer();
//			printf("I%d-%d-%d\t%d\t", N, pdb1Disks, count, (int)toh.GetPathLength(thePath));
//			printf("%llu nodes\t%llu necessary\t", mm.GetNodesExpanded(), mm.GetNecessaryExpansions());
//			printf("%1.2fs elapsed\n", timer.GetElapsedTime());
//		}
//		if (1)
//		{
//			printf("-=-=-A*-=-=-\n");
//			astar.SetHeuristic(f);
//			timer.StartTimer();
//			astar.GetPath(&toh, s, g, thePath);
//			timer.EndTimer();
//			printf("I%d-%d-%d\t%d\t", N, pdb1Disks, count, (int)toh.GetPathLength(thePath));
//			printf("%llu nodes\t%llu necessary\t", astar.GetNodesExpanded(), astar.GetNecessaryExpansions());
//			printf("%1.2fs elapsed\n", timer.GetElapsedTime());
//		}
//		while (b->heuristics.size() > 0)
//		{
//			delete b->heuristics.back();
//			b->heuristics.pop_back();
//		}
//		delete b;
//		while (f->heuristics.size() > 0)
//		{
//			delete f->heuristics.back();
//			f->heuristics.pop_back();
//		}
//		delete f;
	}

}

template <int N, int pdb1Disks, int pdb2Disks = N-pdb1Disks>
void TestTOHHeuristic(int first, int last, int th)
{
	NBS<TOHState<N>, TOHMove, TOH<N>, NBSQueue<TOHState<N>, 1>> nbse1;
  NBS<TOHState<N>, TOHMove, TOH<N>, NBSQueue<TOHState<N>, 1>> innerNbse1;
  NBS<TOHState<N>, TOHMove, TOH<N>, NBSQueue<TOHState<N>, 1>> secinnerNbse1;  
  
  
  

	TOH<N> toh;
	TOHState<N> s;
	TOHState<N> g;
	TOHState<N> start;
	TOHState<N> goal;
	std::vector<TOHState<N>> thePath;
  std::vector<TOHState<N>> secondPath;
	std::vector<TOHMove> actionPath;
	Heuristic<TOHState<N>> *f;
	Heuristic<TOHState<N>> *b;
  Heuristic<TOHState<N>> *fNew;
	Heuristic<TOHState<N>> *bNew;
  Heuristic<TOHState<N>> *innerf;
	ZeroHeuristic<TOHState<N>> z;
  TemplateAStar<TOHState<N>, TOHMove, TOH<N>> astar;
  TemplateAStar<TOHState<N>, TOHMove, TOH<N>> rastar;
  std::vector<TOHState<N>> astarPath;
  NBS<TOHState<N>, TOHMove, TOH<N>, NBSQueue<TOHState<N>, 1, true>> nbs(true);
	std::vector<TOHState<N>> nbsPath;
	std::vector<TOHState<N>> dvcbsPath;
  Timer astarTimer;
  Timer rastarTimer;
  Timer nbsTimer;
  Timer dvcbsTimer;
//	g.Reset();
//	f = BuildPDB<N, pdb1Disks>(g);
	
	int table[] = {52058078,116173544,208694125,131936966,141559500,133800745,194246206,50028346,167007978,207116816,163867037,119897198,201847476,210859515,117688410,121633885};
	int table2[] = {145008714,165971878,154717942,218927374,182772845,5808407,19155194,137438954,13143598,124513215,132635260,39667704,2462244,41006424,214146208,54305743};
	for (int count = first; count < last; count++)
	{
		//printf("Seed: %d\n", table[count&0xF]^table2[(count>>4)&0xF]);
    //int count = 0;
		srandom(table[count&0xF]^table2[(count>>4)&0xF]);


      s.counts[0] = s.counts[1] = s.counts[2] = s.counts[3] = 0;
      for (int x = N; x > 0; x--)
      {
        int whichPeg = random()%4;
        s.disks[whichPeg][s.counts[whichPeg]] = x;
        s.counts[whichPeg]++;
      }
      b = BuildPDB<N, pdb1Disks>(s);
      // b = &z;

      g.counts[0] = g.counts[1] = g.counts[2] = g.counts[3] = 0;
      for (int x = N; x > 0; x--)
      {
        int whichPeg = random()%4;
        g.disks[whichPeg][g.counts[whichPeg]] = x;
        g.counts[whichPeg]++;
      }

      // Using canonical goal currently - comment to use random goal
      //g.Reset();
      
      f = BuildPDB<N, pdb1Disks>(g);
      // f = &z;
      
      
      //printNodesInDistance(&toh, g, f, 5, 5, [](float i){return i <1;});
      //printNodesInDistance(&toh, s, b, 5, 5, [](float i){return i <1;});
      
      //float fl;
      if(0){
        printf("goal ");
        StateNeighborsUpToDistance(&toh, g,f,5);
        printf("start ");
        StateNeighborsUpToDistance(&toh, s,b,5);
      }        

      // fl = MeasureHeuristicErrors(&toh, g, f, 5, 4, [](float i){return i <1;});
      // printf("Start Error percentage (5,4): %1.1f\n", fl*100);
      // fl = MeasureHeuristicErrors(&toh, s, b, 5, 4, [](float i){return i <1;});
      // printf("Goal Error percentage (5,4): %1.1f\n", fl*100);
      if(1){
        printf("regular ");
        BidirectionalProblemAnalyzer<TOHState<N>, TOHMove, TOH<N>>::GetWeightedVertexGraph(s, g, &toh, f, b,0,1);
        if (pdb1Disks == 2){
          printf("zero ");
          BidirectionalProblemAnalyzer<TOHState<N>, TOHMove, TOH<N>>::GetWeightedVertexGraph(s, g, &toh, &z, &z,0,1);
        }
      }
      if(0){
        if(0){
          astar.SetHeuristic(f);
          astarTimer.StartTimer();
          astar.GetPath(&toh, s, g, astarPath);
          astarTimer.EndTimer();
          printf("%d %d %d %d A* ", N, pdb1Disks, count, (int)toh.GetPathLength(astarPath));
          printf("%llu nodes\t%llu necessary\t", astar.GetNodesExpanded(), astar.GetNecessaryExpansions());
          printf("%1.2fs elapsed\n", astarTimer.GetElapsedTime());
        
          astar.SetHeuristic(&z);
          astarTimer.StartTimer();
          astar.GetPath(&toh, s, g, astarPath);
          astarTimer.EndTimer();
          printf("%d %d %d %d A* ", N, 0, count, (int)toh.GetPathLength(astarPath));
          printf("%llu nodes\t%llu necessary\t", astar.GetNodesExpanded(), astar.GetNecessaryExpansions());
          printf("%1.2fs elapsed\n", astarTimer.GetElapsedTime());
        }
        if (0){
          rastar.SetHeuristic(b);
          rastarTimer.StartTimer();
          rastar.GetPath(&toh, g, s, astarPath);
          rastarTimer.EndTimer();
          printf("%d %d %d %d r-A* ", N, pdb1Disks, count, (int)toh.GetPathLength(astarPath));
          printf("%llu nodes\t%llu necessary\t", rastar.GetNodesExpanded(), rastar.GetNecessaryExpansions());
          printf("%1.2fs elapsed\n", rastarTimer.GetElapsedTime());
        
          rastar.SetHeuristic(&z);
          rastarTimer.StartTimer();
          rastar.GetPath(&toh, g, s, astarPath);
          rastarTimer.EndTimer(); 
          printf("%d %d %d %d r-A* ", N, 0, count, (int)toh.GetPathLength(astarPath));
          printf("%llu nodes\t%llu necessary\t", rastar.GetNodesExpanded(), rastar.GetNecessaryExpansions());
          printf("%1.2fs elapsed\n", rastarTimer.GetElapsedTime());
        }
        if(0){
          nbsTimer.StartTimer();
          nbs.GetPath(&toh, s, g, f, b, nbsPath);
          nbsTimer.EndTimer();
          printf("%d %d %d %d NBS ", N, pdb1Disks, count, (int)toh.GetPathLength(nbsPath));
          printf("%llu nodes\t%llu necessary\t", nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions());
          printf("%1.2fs elapsed\n", nbsTimer.GetElapsedTime());
        
          nbsTimer.StartTimer();
          nbs.GetPath(&toh, s, g, &z, &z, nbsPath);
          nbsTimer.EndTimer();
          printf("%d %d %d %d NBS ", N, 0, count, (int)toh.GetPathLength(nbsPath));
          printf("%llu nodes\t%llu necessary\t", nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions());
          printf("%1.2fs elapsed\n", nbsTimer.GetElapsedTime());
        }
        
        if(0){
          DVCBS<TOHState<N>, TOHMove, TOH<N>,DVCBSQueue<TOHState<N>,1,true>> dvcbs(false,true);
          dvcbsTimer.StartTimer();
          dvcbs.GetPath(&toh, s, g, f, b, dvcbsPath);
          dvcbsTimer.EndTimer();
          printf("%d %d %d %d DVCBS ", N, pdb1Disks, count, (int)toh.GetPathLength(dvcbsPath));
          printf("%llu nodes\t%llu necessary\t", dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions());
          printf("%1.2fs elapsed\n", dvcbsTimer.GetElapsedTime());
        
          dvcbsTimer.StartTimer();
          dvcbs.GetPath(&toh, s, g, &z, &z, dvcbsPath);
          dvcbsTimer.EndTimer();
          printf("%d %d %d %d DVCBS ", N, 0, count, (int)toh.GetPathLength(dvcbsPath));
          printf("%llu nodes\t%llu necessary\t", dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions());
          printf("%1.2fs elapsed\n", dvcbsTimer.GetElapsedTime());
        }
        if(0){
          DVCBS<TOHState<N>, TOHMove, TOH<N>,DVCBSQueue<TOHState<N>,1,false>> dvcbs(false,false);
          dvcbsTimer.StartTimer();
          dvcbs.GetPath(&toh, s, g, f, b, dvcbsPath);
          dvcbsTimer.EndTimer();
          printf("%d %d %d %d DVCBS-L ", N, pdb1Disks, count, (int)toh.GetPathLength(dvcbsPath));
          printf("%llu nodes\t%llu necessary\t", dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions());
          printf("%1.2fs elapsed\n", dvcbsTimer.GetElapsedTime());
        
          dvcbsTimer.StartTimer();
          dvcbs.GetPath(&toh, s, g, &z, &z, dvcbsPath);
          dvcbsTimer.EndTimer();
          printf("%d %d %d %d DVCBS-L ", N, 0, count, (int)toh.GetPathLength(dvcbsPath));
          printf("%llu nodes\t%llu necessary\t", dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions());
          printf("%1.2fs elapsed\n", dvcbsTimer.GetElapsedTime());
        }
      }
      /*
      BidirectionalProblemAnalyzer<TOHState<N>, TOHMove, TOH<N>>::GetWeightedVertexGraph(s, g, &toh, f, b);
      astar.SetHeuristic(f);
			astarTimer.StartTimer();
			astar.GetPath(&toh, s, g, astarPath);
			astarTimer.EndTimer();
      
      rastar.SetHeuristic(b);
			rastarTimer.StartTimer();
			rastar.GetPath(&toh, g, s, astarPath);
			rastarTimer.EndTimer();
      
      if (astar.GetNecessaryExpansions() <= rastar.GetNecessaryExpansions()){
        start = s;
        goal = g;
        bNew = b;
        fNew = f;
      }
      else{
        start = g;
        goal = s;
        bNew = f;
        fNew = b;
      }
      
      
      Timer nbsTimer;
      nbsTimer.StartTimer();
      nbse1.GetPath(&toh, start, goal, fNew, bNew, thePath);
      nbsTimer.EndTimer();
    //g.Reset();
      //int pdb2Disks = N-pdb1Disks;
    	//TOH<N> toh;
      TOH<pdb1Disks> absToh1;
      TOH<pdb2Disks> absToh2;
      TOHState<N-pdb1Disks> absTohState1;
      TOHState<N-pdb1Disks> absTohState2;
     
      
      TOHPDB<pdb1Disks, N, pdb2Disks> *pdb1 = new TOHPDB<pdb1Disks, N, pdb2Disks>(&absToh1, goal); // top disks
      TOHPDB<pdb2Disks, N> *pdb2 = new TOHPDB<pdb2Disks, N>(&absToh2, goal); // bottom disks
      pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
      pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
      
      Heuristic<TOHState<N>> *h = new Heuristic<TOHState<N>>;
      
      h->lookups.resize(0);
      
      h->lookups.push_back({kAddNode, 1, 2});
      h->lookups.push_back({kLeafNode, 0, 0});
      h->lookups.push_back({kLeafNode, 1, 1});
      h->heuristics.resize(0);
      h->heuristics.push_back(pdb1);
      h->heuristics.push_back(pdb2);
      

			//printf("I%d-%d-%d\t%d\t", N, pdb1Disks, count, (int)toh.GetPathLength(thePath));
			//printf("%llu nodes\t%llu necessary\t", astar.GetNodesExpanded(), astar.GetNecessaryExpansions());
			//printf("%1.2fs elapsed\n", timer.GetElapsedTime());
      
      
    TOHState<pdb1Disks> g1;
    TOHState<pdb2Disks> g2;
    TOHState<pdb1Disks> u1;
    TOHState<pdb2Disks> u2;

    std::queue<TOHState<pdb1Disks>> q1;
    std::queue<TOHState<pdb2Disks>> q2;
    std::vector<TOHMove> acts1;
    std::vector<TOHMove> acts2;
    TOH<pdb1Disks> env1;
    TOH<pdb2Disks> env2;
    //g1.Reset();
    g1.counts[0] = g1.counts[1] = g1.counts[2] = g1.counts[3] = 0;
    for (int x = 0; x < 4; x++)
    {
      for (int y = 0; y < goal.GetDiskCountOnPeg(x); y++){
        if (goal.GetDiskOnPeg(x, y) <= pdb1Disks){
          g1.disks[x][g1.counts[x]] = goal.GetDiskOnPeg(x, y);
          g1.counts[x]++;
        }
      }
    }
    g2.counts[0] = g2.counts[1] = g2.counts[2] = g2.counts[3] = 0;
    for (int x = 0; x < 4; x++)
    {
      for (int y = 0; y < goal.GetDiskCountOnPeg(x); y++){
        if (goal.GetDiskOnPeg(x, y) > pdb1Disks){
          g2.disks[x][g2.counts[x]] = goal.GetDiskOnPeg(x, y) - pdb1Disks;
          g2.counts[x]++;
        }
      }
    }
    //cout << g1 << std::endl;
    //g2.Reset();

    
    // struct CustomCompare1
    // {
        // bool operator()(const TOHState<pdb1Disks>& lhs, const TOHState<pdb1Disks>& rhs)
        // {
          // return lhs.GetDiskCountOnPeg(4) < lhs.GetDiskCountOnPeg(4);
        // }
    // };
    
    // struct CustomCompare2
    // {
        // bool operator()(const TOHState<pdb2Disks>& lhs, const TOHState<pdb2Disks>& rhs)
        // {
          // return lhs.GetDiskCountOnPeg(4) < lhs.GetDiskCountOnPeg(4);
        // }
    // }; 
    
    struct hash1 {
      public:
        size_t operator()(const TOHState<pdb1Disks> & s) const {
          int res = 0;
          for (int x = 0; x < 4; x++)
            {
              for (int y = 0; y < s.GetDiskCountOnPeg(x); y++)
                res += s.disks[x][y] * y + (pdb1Disks*pdb1Disks*10)*x;
            }
          return std::hash<int>()(res);
        }
      };

      struct hash2 {
      public:
        size_t operator()(const TOHState<pdb2Disks> & s) const {
          int res = 0;
          for (int x = 0; x < 4; x++)
            {
              for (int y = 0; y < s.GetDiskCountOnPeg(x); y++)
                res += s.disks[x][y] * y + (pdb1Disks*pdb1Disks*10)*x;
            }
          return std::hash<int>()(res);;
        }
      };
      struct hash3 {
      public:
        size_t operator()(const TOHState<N> & s) const {
          int res = 0;
          for (int x = 0; x < 4; x++)
            {
              for (int y = 0; y < s.GetDiskCountOnPeg(x); y++)
                res += s.disks[x][y] * y + (pdb1Disks*pdb1Disks*10)*x;
            }
          return std::hash<int>()(res);;
        }
      };

    //std::unordered_set<TOHState<pdb1Disks>,hash1> lowh1;
    //std::unordered_set<TOHState<pdb2Disks>,hash2> lowh2;
    std::unordered_set<TOHState<N>,hash3> allNearStates;
    q1.push(g1);
       

    for (int th1 = 0; th1 <= th; th1++){
      int th2 = th - th1;
      std::unordered_set<TOHState<pdb1Disks>,hash1> tmp1;
      std::unordered_set<TOHState<pdb2Disks>,hash2> tmp2;
      q2.push(g2); 
      while (!q2.empty()){
          TOHState<pdb2Disks> s = q2.front();
          q2.pop();
          if (tmp2.insert(s).second){
            env2.GetActions(s, acts2);
            for (int y = 0; y < acts2.size(); y++)
            {
              env2.GetNextState(s, acts2[y], u2);
              //cout << u2 << " " << th1 << std::endl;
              //cout << pdb2->HCost(pdb2->GetStateFromAbstractState(u2),pdb2->GetStateFromAbstractState(g2)) << std::endl;
              assert(env2.InvertAction(acts2[y]) == true);
              if (pdb2->HCost(pdb2->GetStateFromAbstractState(u2),pdb2->GetStateFromAbstractState(g2)) <= th1)
              {
                q2.push(u2);
              }
            }
          }
      }
      //lowh2.insert(tmp2.begin(),tmp2.end());
      q1.push(g1); 
      //cout << lowh2.size() << " " << th << " " << th1 << " " << th2 << std::endl;
      while (!q1.empty()){
          TOHState<pdb1Disks> s = q1.front();
          q1.pop();
          if (tmp1.insert(s).second){
            env1.GetActions(s, acts1);
            for (int y = 0; y < acts1.size(); y++)
            {
              env1.GetNextState(s, acts1[y], u1);
              assert(env1.InvertAction(acts1[y]) == true);
              if (pdb1->HCost(pdb1->GetStateFromAbstractState(u1),pdb1->GetStateFromAbstractState(g1)) <= th2)
              {
                q1.push(u1);
              }
            }
          }
      }
      //lowh1.insert(tmp1.begin(),tmp1.end());
      //cout << lowh1.size() << std::endl;  
      for (typename  std::unordered_set<TOHState<pdb2Disks>,hash2>::iterator it2=tmp2.begin(); it2!=tmp2.end(); ++it2){
            for (typename  std::unordered_set<TOHState<pdb1Disks>,hash1>::iterator it1=tmp1.begin(); it1!=tmp1.end(); ++it1){
              TOHState<N> newState;
              for (int x = 0; x < 4; x++)
              {
                newState.counts[x] = (*it1).counts[x] + (*it2).counts[x];
                for (int i = 0; i < (*it2).GetDiskCountOnPeg(x); i++)
                {
                  newState.disks[x][i] = (*it2).disks[x][i] + pdb1Disks;
                }
                for (int i = 0; i < (*it1).GetDiskCountOnPeg(x); i++)
                {
                  newState.disks[x][i+(*it2).GetDiskCountOnPeg(x)] = (*it1).disks[x][i];
                }
              }
              // cout << goal << std::endl;
              // cout << (*it1) << std::endl;
              // cout << (*it2) << std::endl;
              // cout << g1 << std::endl;
              // cout << g2 << std::endl;
              // cout << newState << " " << th1 << th2 << std::endl;
              allNearStates.insert(newState);
            }
      }
    }
    for (typename  std::unordered_set<TOHState<N>,hash3>::iterator it=allNearStates.begin(); it!=allNearStates.end(); ++it){
            std::cout << (*it) << std::endl;
            Timer timer;
            timer.StartTimer();
            innerNbse1.GetPath(&toh, (*it), goal, h, bNew, thePath);
            timer.EndTimer();
            //innerf = BuildPDB<N, pdb1Disks>((*it));
            //secinnerNbse1.GetPath(&toh, start, (*it), innerf, bNew, secondPath);
            if ((int)toh.GetPathLength(thePath) != (int)h->HCost((*it),goal)){
              printf("top\t%d\t%d\t%d\t%d\t%1.2f\t%d\t%d\t%1.2f\t%d\t%d\t%1.2f\t%d\t%d\t%1.2f\t%d\t%d\t%1.2f\t%d\t%d\n",N, pdb1Disks, count, (int)toh.GetPathLength(thePath),h->HCost((*it),goal),innerNbse1.GetNodesExpanded(), innerNbse1.GetNecessaryExpansions(), timer.GetElapsedTime(),astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), astarTimer.GetElapsedTime(),rastar.GetNodesExpanded(), rastar.GetNecessaryExpansions(), rastarTimer.GetElapsedTime(),nbse1.GetNodesExpanded(), nbse1.GetNecessaryExpansions(), nbsTimer.GetElapsedTime(),(int)toh.GetPathLength(astarPath),allNearStates.size()); //,(int)toh.GetPathLength(secondPath)
            }
    }
    
          // cout << newState<< std::endl;
          // b = BuildPDB<N, pdb1Disks>(newState);
          // Timer timer;
          // timer.StartTimer();
          // nbse1.GetPath(&toh, newState, g, h, b, thePath);
          // timer.EndTimer();
          // printf("bottom\t%d\t%d\t%d\t%d\t%1.2f\t%d\t%d\t%1.2f\n",N, pdb1Disks, count, (int)toh.GetPathLength(thePath),h->HCost(newState,g),nbse1.GetNodesExpanded(), nbse1.GetNecessaryExpansions(), timer.GetElapsedTime());
          // BidirectionalProblemAnalyzer<TOHState<N>, TOHMove, TOH<N>>::GetWeightedVertexGraph(newState, g, &toh, h, b);
    // }
    // for (typename  std::unordered_set<TOHState<pdb2Disks>,hash2>::iterator it2=lowh2.begin(); it2!=lowh2.end(); ++it2){
          // for (typename  std::unordered_set<TOHState<pdb1Disks>,hash1>::iterator it1=lowh1.begin(); it1!=lowh1.end(); ++it1){
            // TOHState<N> newState;
            // for (int x = 0; x < 4; x++)
            // {
              // newState.counts[x] = (*it1).counts[x] + (*it2).counts[x];
              // for (int i = 0; i < (*it2).GetDiskCountOnPeg(x); i++)
              // {
                // newState.disks[x][i] = (*it2).disks[x][i] + pdb1Disks;
              // }
              // for (int i = 0; i < (*it1).GetDiskCountOnPeg(x); i++)
              // {
                // newState.disks[x][i+(*it2).GetDiskCountOnPeg(x)] = (*it1).disks[x][i];
              // }
            // }
            // Timer timer;
            // timer.StartTimer();
            // innerNbse1.GetPath(&toh, newState, goal, h, bNew, thePath);
            // timer.EndTimer();
            // //innerf = BuildPDB<N, pdb1Disks>(newState);
            // //secinnerNbse1.GetPath(&toh, start, newState, innerf, bNew, secondPath);
            // if ((int)toh.GetPathLength(thePath) != (int)h->HCost(newState,goal)){
              // printf("top\t%d\t%d\t%d\t%d\t%1.2f\t%d\t%d\t%1.2f\t%d\t%d\t%1.2f\t%d\t%d\t%1.2f\t%d\t%d\t%1.2f\t%d\n",N, pdb1Disks, count, (int)toh.GetPathLength(thePath),h->HCost(newState,goal),innerNbse1.GetNodesExpanded(), innerNbse1.GetNecessaryExpansions(), timer.GetElapsedTime(),astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), astarTimer.GetElapsedTime(),rastar.GetNodesExpanded(), rastar.GetNecessaryExpansions(), rastarTimer.GetElapsedTime(),nbse1.GetNodesExpanded(), nbse1.GetNecessaryExpansions(), nbsTimer.GetElapsedTime(),(int)toh.GetPathLength(astarPath)); //,(int)toh.GetPathLength(secondPath)
            // }
          // }
    // }
    //printf("top\t%d\t%d\t%d\t%d\n",N, pdb1Disks, count, lowh1.size() * lowh2.size());
    */
  }
  
  /* sample heuristic 
  
  int hcount,ihcount;
  hcount = ihcount = 0;
  int limit = 10000;
  srandom(3201561);
  
  for (int count = 0; count < limit; count++)
  {
    srandom(random());
    s.counts[0] = s.counts[1] = s.counts[2] = s.counts[3] = 0;
    for (int x = N; x > 0; x--)
    {
      int whichPeg = random()%4;
      s.disks[whichPeg][s.counts[whichPeg]] = x;
      s.counts[whichPeg]++;
    }
    b = BuildPDB<N, pdb1Disks>(s);

    g.counts[0] = g.counts[1] = g.counts[2] = g.counts[3] = 0;
    for (int x = N; x > 0; x--)
    {
      int whichPeg = random()%4;
      g.disks[whichPeg][g.counts[whichPeg]] = x;
      g.counts[whichPeg]++;
    }
    //f = BuildPDB<N, pdb1Disks>(g);

    //printf("Problem %d of %d\n", count+1, limit);
    // std::cout << original << "\n";
    if (f->HCost(s,g) <= th){
      hcount++;
      astar.SetHeuristic(f);
      astar.GetPath(&toh, s, g, astarPath);
      if (toh.GetPathLength(astarPath) != f->HCost(s,g)){
        ihcount++;
      }
    }
  }
  printf("%d %d\n",hcount,ihcount);    

  */

  // Using canonical goal currently - comment to use random goal
  //g.Reset();
  //f = BuildPDB<N, pdb1Disks>(g);
  // for (int i = 1; i < 100000000; i++){
        // s.counts[0] = s.counts[1] = s.counts[2] = s.counts[3] = 0;
        // for (int x = N; x > 0; x--)
        // {
          // int whichPeg = random()%4;
          // s.disks[whichPeg][s.counts[whichPeg]] = x;
          // s.counts[whichPeg]++;
        // }
        //printf("%1.2f-%1.2f\n", pdb2->HCost(s,g), h->HCost(s,g));
        // if (pdb2->HCost(s,g) == 0.0 && h->HCost(s,g) <= th){
          // b = BuildPDB<N, pdb1Disks>(s);
          // Timer timer;
            // timer.StartTimer();
            // nbse1.GetPath(&toh, s, g, h, b, thePath);
            // timer.EndTimer();
            // printf("bottom\t%d\t%d\t%d\t%d\t%d\t%1.2f\t%d\t%d\t%1.2f\n", i,N, pdb1Disks, count, (int)toh.GetPathLength(thePath),h->HCost(s,g),nbse1.GetNodesExpanded(), nbse1.GetNecessaryExpansions(), timer.GetElapsedTime());
        // }
        // if (pdb1->HCost(s,g) == 0.0 && h->HCost(s,g) <= th){
          // b = BuildPDB<N, pdb1Disks>(s);
          // Timer timer;
            // timer.StartTimer();
            // nbse1.GetPath(&toh, s, g, h, b, thePath);
            // timer.EndTimer();
            // printf("top\t%d\t%d\t%d\t%d\t%d\t%1.2f\t%d\t%d\t%1.2f\n", i,N, pdb1Disks, count, (int)toh.GetPathLength(thePath),h->HCost(s,g),nbse1.GetNodesExpanded(), nbse1.GetNecessaryExpansions(), timer.GetElapsedTime());
        // }
  
   // }
   
   





}
template <int N>
void TestTOHDensityVariance(int samplesAmount,int sampleDepth)
{
  TOH<N> toh;
	TOHState<N> s;
  OnlineStats stats;
  srandom(1923544);
	for (int count = 0; count < samplesAmount; count++)
	{
		srandom(random());
		s.counts[0] = s.counts[1] = s.counts[2] = s.counts[3] = 0;
		for (int x = N; x > 0; x--)
		{
			int whichPeg = random()%4;
			s.disks[whichPeg][s.counts[whichPeg]] = x;
			s.counts[whichPeg]++;
		}
    stats.Push(dijkstraUptoLimit(&toh,s,sampleDepth));
    
  }
  printf("variance %1.2f", stats.Variance());
  
}

void TOHTest()
{
//	TestTOH<14, 2>(0, 50);
//	TestTOH<14, 4>(0, 50);
//    TestTOHDensityVariance<8>(1000,100);
//    TestTOHDensityVariance<10>(1000,100);
//    TestTOHDensityVariance<12>(1000,100);
    
    TestTOHHeuristic<8, 2>(0, 50,5);
    TestTOHHeuristic<8, 4>(0, 50,5);
    TestTOHHeuristic<8, 6>(0, 50,5);
    TestTOHHeuristic<10, 2>(0, 50,5);
    TestTOHHeuristic<10, 4>(0, 50,5);
    TestTOHHeuristic<10, 6>(0, 50,5);
    TestTOHHeuristic<10, 8>(0, 50,5);
    TestTOHHeuristic<12, 2>(0, 50,5);
    TestTOHHeuristic<12, 4>(0, 50,5);
    TestTOHHeuristic<12, 6>(0, 50,5);
    TestTOHHeuristic<12, 8>(0, 50,5);
    TestTOHHeuristic<12, 10>(0, 50,5);
    
    // TestTOHHeuristic<14, 2>(0, 50,3);
    // TestTOHHeuristic<14, 4>(0, 50,3);
    // TestTOHHeuristic<14, 6>(0, 50,3);
    // TestTOHHeuristic<14, 8>(0, 50,3);
    // TestTOHHeuristic<14, 10>(0, 50,3);    
    // TestTOHHeuristic<14, 12>(0, 50,3);    
    // TestTOHHeuristic<16, 2>(0, 50,3);
    // TestTOHHeuristic<16, 4>(0, 50,3);
    // TestTOHHeuristic<16, 6>(0, 50,3);
    // TestTOHHeuristic<16, 8>(0, 50,3);
    // TestTOHHeuristic<16, 10>(0, 50,3);    
    // TestTOHHeuristic<16, 12>(0, 50,3);  
    // TestTOHHeuristic<16, 14>(0, 50,3);

//	TestTOH<14, 5>(0, 50);
//	TestTOH<14, 6>(0, 50);
//	TestTOH<14, 7>(0, 50);
//	TestTOH<14, 8>(0, 50);
//	TestTOH<14, 9>(0, 50);
//	TestTOH<14, 10>(0, 50);
//	const int numDisks = 16; // [disks - 2] (4^14 - 256 million)
//	const int pdb1Disks = 10;
//	const int pdb2Disks = 6;
//	TOH<numDisks> toh;
//	TOHState<numDisks> s, g;
//	
//	TOHState<numDisks> goal;
//	TOH<pdb1Disks> absToh1;
//	TOH<pdb2Disks> absToh2;
//	TOHState<pdb1Disks> absTohState1;
//	TOHState<pdb2Disks> absTohState2;
//
//	TOHPDB<pdb1Disks, numDisks, pdb2Disks> pdb1(&absToh1); // top disks
//	TOHPDB<pdb2Disks, numDisks> pdb2(&absToh2); // bottom disks
//
//	ZeroHeuristic<TOHState<numDisks>> z;
//	
//	goal.Reset();
//	pdb1.BuildPDB(goal, std::thread::hardware_concurrency());
//	pdb2.BuildPDB(goal, std::thread::hardware_concurrency());
//	
////	s.Reset();
////	goal.Reset();
////	for (int x = 0; x < 100; x++)
////	{
////		std::vector<TOHMove> actionPath;
////		for (int x = 0; x < 20000; x++)
////		{
////			toh.GetActions(s, actionPath);
////			toh.ApplyAction(s, actionPath[random()%actionPath.size()]);
////		}
////		std::cout << s << "\n";
////		std::cout << "H1: " << pdb1.HCost(s, goal) << "\n";
////		std::cout << "H2: " << pdb2.HCost(s, goal) << "\n";
////	}
////	exit(0);
//	goal.Reset();
//	Heuristic<TOHState<numDisks>> h;
//	
//	h.lookups.resize(0);
//	h.lookups.push_back({kAddNode, 1, 2});
//	h.lookups.push_back({kLeafNode, 0, 0});
//	h.lookups.push_back({kLeafNode, 1, 1});
//	h.heuristics.resize(0);
//	h.heuristics.push_back(&pdb1);
//	h.heuristics.push_back(&pdb2);
//	printf("-=-=-=-==-=-=-=-=-\n");
//	printf("With %d and %d\n", pdb1Disks, pdb2Disks);
//	TestTOH<numDisks>(&h, 0, 10);
//
//	printf("-=-=-=-==-=-=-=-=-\n");
//	h.heuristics[1] = &z;
//	printf("With just %d\n", pdb1Disks);
//	TestTOH<numDisks>(&h, 0, 10);
	exit(0);
}


