//
//  BidirPancake.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/7/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#include "BidirPancake.h"
#include "PancakePuzzle.h"
#include "PancakePuzzleGapVector.h"
#include "TemplateAStar.h"
#include "NBS.h"
#include "DVCBS.h"
#include "IDAStar.h"
#include "MM.h"
#include "BSStar.h"
#include "PancakeInstances.h"
#include "WeightedVertexGraph.h"
#include "HeuristicError.h"
#include "OnlineStats.h"

const int S = 10; // must be factor of sizes below

void TestPancakeTR();
void TestPancakeRandom();
void TestPancakeHard(int gap = 0);
void TestPancakeLowHeuristic(int rad);
void TestPancakeDensityVariance(int samplesAmount,int sampleDepth);
void TestPancakeHeuristic();
void TestRob();
void TestVariants();
void TestError();
void TestPancake_overall();

void TestPancake()
{
//	TestRob();
//	TestPancakeRandom();

//	TestPancakeHard(0); // GAP heuristic #
//	TestPancakeHard(1);
//	TestPancakeHard(2);
//    TestPancakeHeuristic();
    TestPancakeLowHeuristic(5);
  //TestPancake_overall();
	//TestError();
  //TestPancakeDensityVariance(1000,100);

//	TestVariants();
	exit(0);
}


void TestRob()
{
//	0 3 2 1
	PancakePuzzleState<4> start;
	PancakePuzzleState<4> goal;
	PancakePuzzle<4> cake(1);
	ZeroHeuristic<PancakePuzzleState<4>> z;
	std::vector<PancakePuzzleState<4>> path;
	start.puzzle[0] = 0;
	start.puzzle[1] = 3;
	start.puzzle[2] = 2;
	start.puzzle[3] = 1;
	goal.puzzle[0] = 1;
	goal.puzzle[1] = 3;
	goal.puzzle[2] = 2;
	goal.puzzle[3] = 0;
	NBS<PancakePuzzleState<4>, PancakePuzzleAction, PancakePuzzle<4>> nbs;
	MM<PancakePuzzleState<4>, PancakePuzzleAction, PancakePuzzle<4>> mm;
	mm.GetPath(&cake, start, goal, &cake, &cake, path);
	printf("MM: %lld expansions\n", mm.GetNodesExpanded());
	mm.GetPath(&cake, start, goal, &z, &z, path);
	printf("MM0: %lld expansions\n", mm.GetNodesExpanded());
	
	exit(0);
}

void TestPancakeTR()
{
	// multiples of 5
	int arrangement[] = {0,2,4,1,3,5,7,9,6,8,10,12,14,11,13,15,17,19,16,18,20,22,24,21,23,25,27,29,26,28,30,32,34,31,33,35,37,39,36,38,40,42,44,41,43,45,47,49,46,48,50,52,54,51,53,55,57,59,56,58,60,62,64,61,63,65,67,69,66,68,70,72,74,71,73,75,77,79,76,78,80,82,84,81,83,85,87,89,86,88,90,92,94,91,93,95,97,99,96,98,};
	// multiples of 9
//	const int arrangement[] = {0,4,7,2,5,8,3,6,1,9,13,16,11,14,17,12,15,10,18,22,25,20,23,26,21,24,19,27,31,34,29,32,35,30,33,28,36,40,43,38,41,44,39,42,37,45,49,52,47,50,53,48,51,46,54,58,61,56,59,62,57,60,55,63,67,70,65,68,71,66,69,64,72,76,79,74,77,80,75,78,73,81,85,88,83,86,89,84,87,82,90,94,97,92,95,98,93,96,91};

	for (int gap = 0; gap < 10; gap++)
	{
		
		PancakePuzzleState<S> start;
		PancakePuzzleState<S> goal;
		PancakePuzzle<S> pancake(gap);
		PancakePuzzle<S> pancake2(gap);
		
		NBS<PancakePuzzleState<S>, PancakePuzzleAction, PancakePuzzle<S>> nbs;
		MM<PancakePuzzleState<S>, PancakePuzzleAction, PancakePuzzle<S>> mm;
		TemplateAStar<PancakePuzzleState<S>, PancakePuzzleAction, PancakePuzzle<S>> astar;
		IDAStar<PancakePuzzleState<S>, PancakePuzzleAction, false> idastar;
		
		std::vector<PancakePuzzleState<S>> nbsPath;
		std::vector<PancakePuzzleState<S>> astarPath;
		std::vector<PancakePuzzleState<S>> mmPath;
		std::vector<PancakePuzzleAction> idaPath;
		Timer t1, t2, t3, t4;
		
		
		goal.Reset();
		for (int x = 0; x < S; x++)
			start.puzzle[x] = arrangement[x];
		t1.StartTimer();
		astar.GetPath(&pancake, start, goal, astarPath);
		t1.EndTimer();
		uint64_t necessary = 0;
		double solutionCost = pancake.GetPathLength(astarPath);
		for (unsigned int x = 0; x < astar.GetNumItems(); x++)
		{
			const auto &item = astar.GetItem(x);
			if ((item.where == kClosedList) && (item.g+item.h < solutionCost))
				necessary++;
		}
		printf("A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", pancake.GetPathLength(astarPath),
			   astar.GetNodesExpanded(), necessary, t1.GetElapsedTime());
		
		goal.Reset();
		for (int x = 0; x < S; x++)
			start.puzzle[x] = arrangement[x];
		t2.StartTimer();
		nbs.GetPath(&pancake, start, goal, &pancake, &pancake2, nbsPath);
		t2.EndTimer();
		printf("NBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", pancake.GetPathLength(nbsPath),
			   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t2.GetElapsedTime());
		
		goal.Reset();
		for (int x = 0; x < S; x++)
			start.puzzle[x] = arrangement[x];
		t3.StartTimer();
		idastar.GetPath(&pancake, start, goal, idaPath);
		t3.EndTimer();
		printf("IDA* found path length %ld; %llu expanded; %llu generated; %1.2fs elapsed\n", idaPath.size(),
			   idastar.GetNodesExpanded(), idastar.GetNodesTouched(), t3.GetElapsedTime());
		
		
		goal.Reset();
		for (int x = 0; x < S; x++)
			start.puzzle[x] = arrangement[x];
		t4.StartTimer();
		mm.GetPath(&pancake, start, goal, &pancake, &pancake2, mmPath);
		t4.EndTimer();
		printf("MM found path length %1.0f; %llu expanded; %1.2fs elapsed\n", pancake.GetPathLength(mmPath),
			   mm.GetNodesExpanded(), t4.GetElapsedTime());
		
		printf("Problem & IDA* & & A* & & MM & & NBS* & \\\\\n");
		printf("%d G-%d & %llu & %1.2fs & %llu & %1.2fs & %llu & %1.2fs & %llu & %1.2fs \\\\ \n", S, gap,
			   idastar.GetNodesExpanded(), t3.GetElapsedTime(),
			   astar.GetNodesExpanded(), t1.GetElapsedTime(),
			   mm.GetNodesExpanded(), t4.GetElapsedTime(),
			   nbs.GetNodesExpanded(), t2.GetElapsedTime());
	}
	exit(0);
}

const int N = 12;
void TestPancakeRandom()
{
	for (int gap = 0; gap < 1; gap++)
	{
		srandom(2017218);
		PancakePuzzleState<N> start;
		PancakePuzzleState<N> original;
		PancakePuzzleState<N> goal;
		PancakePuzzle<N> pancake(gap);
		PancakePuzzle<N> pancake2(gap);
		
		
		std::vector<PancakePuzzleState<N>> nbsPath;
		std::vector<PancakePuzzleState<N>> bsPath;
		std::vector<PancakePuzzleState<N>> astarPath;
		std::vector<PancakePuzzleState<N>> mmPath;
    std::vector<PancakePuzzleState<N>> dPath;
		std::vector<PancakePuzzleAction> idaPath;
		Timer t1, t2, t3, t4, t5;
		
		for (int count = 0; count < 50; count++)
		{
			srandom(random());
			
			goal.Reset();
			original.Reset();
			for (int x = 0; x < N; x++)
				std::swap(original.puzzle[x], original.puzzle[x+random()%(N-x)]);
			
			printf("Problem %d of %d\n", count+1, 50);
			std::cout << original << "\n";
			
			// A*
			if (0)
			{
				TemplateAStar<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> astar;
				start = original;
				t1.StartTimer();
				astar.GetPath(&pancake, start, goal, astarPath);
				t1.EndTimer();
				printf("GAP-%d A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap, pancake.GetPathLength(astarPath),
					   astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());
			}
			
			// NBS
			if (1)
			{
				NBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> nbs;
				goal.Reset();
				start = original;
				t2.StartTimer();
				nbs.GetPath(&pancake, start, goal, &pancake, &pancake2, nbsPath);
				t2.EndTimer();
				printf("GAP-%d NBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed; %f meeting\n", gap, pancake.GetPathLength(nbsPath),
					   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t2.GetElapsedTime(), nbs.GetMeetingPoint());
			}
			
			// BS*
			if (0)
			{
				BSStar<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> bs;
				goal.Reset();
				start = original;
				t2.StartTimer();
				bs.GetPath(&pancake, start, goal, &pancake, &pancake2, bsPath);
				t2.EndTimer();
				printf("GAP-%d BS* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap, pancake.GetPathLength(bsPath),
					   bs.GetNodesExpanded(), bs.GetNecessaryExpansions(), t2.GetElapsedTime());
			}
			
			// IDA*
			if (0)
			{
				IDAStar<PancakePuzzleState<N>, PancakePuzzleAction, false> idastar;
				goal.Reset();
				start = original;
				t3.StartTimer();
				idastar.GetPath(&pancake, start, goal, idaPath);
				t3.EndTimer();
				printf("GAP-%d IDA* found path length %ld; %llu expanded; %llu generated; %1.2fs elapsed\n", gap, idaPath.size(),
					   idastar.GetNodesExpanded(), idastar.GetNodesTouched(), t3.GetElapsedTime());
			}
			
			// MM
			if (0)
			{
				MM<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> mm;
				goal.Reset();
				start = original;
				t4.StartTimer();
				mm.GetPath(&pancake, start, goal, &pancake, &pancake2, mmPath);
				t4.EndTimer();
				printf("GAP-%d MM found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap, pancake.GetPathLength(mmPath),
					   mm.GetNodesExpanded(), mm.GetNecessaryExpansions(), t1.GetElapsedTime());
			}

			// MM0
			if (0 && gap == 3)
			{
				MM<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> mm;
				ZeroHeuristic<PancakePuzzleState<N>> z;
				goal.Reset();
				start = original;
				t4.StartTimer();
				mm.GetPath(&pancake, start, goal, &z, &z, mmPath);
				t4.EndTimer();
				printf("GAP-%d MM0 found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap, pancake.GetPathLength(mmPath),
					   mm.GetNodesExpanded(), mm.GetNecessaryExpansions(), t1.GetElapsedTime());
			}
		}
	}
}

const int CNT = 12;
void TestPancakeHard(int gap)
{
	srandom(2017218);
	PancakePuzzleState<CNT> start;
	PancakePuzzleState<CNT> original;
	PancakePuzzleState<CNT> goal;
	PancakePuzzle<CNT> pancake(gap);
	PancakePuzzle<CNT> pancake2(gap);
	
	
	std::vector<PancakePuzzleState<CNT>> nbsPath;
	std::vector<PancakePuzzleState<CNT>> bsPath;
	std::vector<PancakePuzzleState<CNT>> astarPath;
	std::vector<PancakePuzzleState<CNT>> mmPath;
	std::vector<PancakePuzzleAction> idaPath;
	Timer t1, t2, t3, t4, t5;
	
	for (int count = 0; count < 100; count++)
	{
		goal.Reset();
		original.Reset();
		GetPancakeInstance(original, count);
		
		printf("Problem %d of %d\n", count+1, 100);
		std::cout << original << "; Initial heuristic " << pancake.HCost(original,goal) << "\n";
		
		// A*
		if (0)
		{
			TemplateAStar<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>> astar;
			start = original;
			t1.StartTimer();
			astar.GetPath(&pancake, start, goal, astarPath);
			t1.EndTimer();
			printf("HARD-%d-G%d A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, gap, pancake.GetPathLength(astarPath),
				   astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());
//			std::unordered_map<int, bool> m;
//			for (int x = 0; x < astar.GetNumItems(); x++)
//			{
//				auto &i = astar.GetItem(x);
//				if (i.where != kClosedList)
//					continue;
//				int tmp = (((int)i.g)<<10)|(int)i.h;
//				if (m.find(tmp) == m.end())
//				{
//					m[tmp] = true;
//					printf("(%d, %d)\n", (int)i.g, (int)i.h);
//				}
//			}
		}

		// Reverse A*
		if (0)
		{
			TemplateAStar<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>> astar;
			start = original;
			t1.StartTimer();
			astar.GetPath(&pancake, goal, start, astarPath);
			t1.EndTimer();
			printf("HARD-%d-G%d ReverseA* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, gap, pancake.GetPathLength(astarPath),
				   astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());
//			std::unordered_map<int, bool> m;
//			for (int x = 0; x < astar.GetNumItems(); x++)
//			{
//				auto &i = astar.GetItem(x);
//				if (i.where != kClosedList)
//					continue;
//				int tmp = (((int)i.g)<<10)|(int)i.h;
//				if (m.find(tmp) == m.end())
//				{
//					m[tmp] = true;
//					printf("(%d, %d)\n", (int)i.g, (int)i.h);
//				}
//			}
		}
		
		// Find minimum
		if (0)
		{
			start = original;

			std::string t = "/Users/nathanst/bidir/pancake/pancake_";
			t += std::to_string(count);
			t += "_GAP";
			t += std::to_string(gap);

			BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>> p(start, goal, &pancake, &pancake, &pancake);
			p.drawFullGraph = true;
			p.drawProblemInstance = false;
			p.drawMinimumVC = true;
			p.drawAllG = false;
			p.drawStatistics = false;
//			p.SaveSVG((t+"-full.svg").c_str());
			p.drawFullGraph = false;
			p.drawProblemInstance = false;
			p.drawAllG = true;
			p.drawStatistics = false;
//			p.SaveSVG((t+"-min.svg").c_str());
		}

		
		// NBS e=1
		if (1)
		{
			NBS<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>, NBSQueue<PancakePuzzleState<CNT>, 1>> nbs;
			goal.Reset();
			start = original;
			t2.StartTimer();
			nbs.GetPath(&pancake, start, goal, &pancake, &pancake2, nbsPath);
			t2.EndTimer();
			printf("HARD-%d-G%d NBSe1 found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed; %f meeting\n", count, gap, pancake.GetPathLength(nbsPath),
				   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t2.GetElapsedTime(), nbs.GetMeetingPoint());
		}
		// NBS e=1
		if (1)
		{
			NBS<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>, NBSQueue<PancakePuzzleState<CNT>, 1, true>> nbs;
			goal.Reset();
			start = original;
			t2.StartTimer();
			nbs.GetPath(&pancake, start, goal, &pancake, &pancake2, nbsPath);
			t2.EndTimer();
			printf("HARD-%d-G%d NBSAe1 found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed; %f meeting\n", count, gap, pancake.GetPathLength(nbsPath),
				   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t2.GetElapsedTime(), nbs.GetMeetingPoint());
		}
		// NBS
		if (0)
		{
			NBS<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>, NBSQueue<PancakePuzzleState<CNT>, 0>> nbs;
			goal.Reset();
			start = original;
			t2.StartTimer();
			nbs.GetPath(&pancake, start, goal, &pancake, &pancake2, nbsPath);
			t2.EndTimer();
			printf("HARD-%d-G%d NBSe0 found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed; %f meeting\n", count, gap, pancake.GetPathLength(nbsPath),
				   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t2.GetElapsedTime(), nbs.GetMeetingPoint());
		}
		// NBS0
		if (0)
		{
			ZeroHeuristic<PancakePuzzleState<CNT>> z;
			NBS<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>, NBSQueue<PancakePuzzleState<CNT>, 0>> nbs;
			goal.Reset();
			start = original;
			t2.StartTimer();
			nbs.GetPath(&pancake, start, goal, &z, &z, nbsPath);
			t2.EndTimer();
			printf("HARD-%d-G%d NBS0 found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed; %f meeting\n", count, gap, pancake.GetPathLength(nbsPath),
				   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t2.GetElapsedTime(), nbs.GetMeetingPoint());
		}
		
		// BS*
		if (0)
		{
			BSStar<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>> bs;
			goal.Reset();
			start = original;
			t2.StartTimer();
			bs.GetPath(&pancake, start, goal, &pancake, &pancake2, bsPath);
			t2.EndTimer();
			printf("HARD-%d BS* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake.GetPathLength(bsPath),
				   bs.GetNodesExpanded(), bs.GetNecessaryExpansions(), t2.GetElapsedTime());
		}
		
		// IDA*
		if (0)
		{
			IDAStar<PancakePuzzleState<CNT>, PancakePuzzleAction, false> idastar;
			goal.Reset();
			start = original;
			t3.StartTimer();
			idastar.GetPath(&pancake, start, goal, idaPath);
			t3.EndTimer();
			printf("HARD-%d IDA* found path length %ld; %llu expanded; %llu generated; %1.2fs elapsed\n", count, idaPath.size(),
				   idastar.GetNodesExpanded(), idastar.GetNodesTouched(), t3.GetElapsedTime());
		}
		
		// MM
		if (0)
		{
			MM<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>> mm;
			goal.Reset();
			start = original;
			t4.StartTimer();
			mm.GetPath(&pancake, start, goal, &pancake, &pancake2, mmPath);
			t4.EndTimer();
			printf("HARD-%d MM found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake.GetPathLength(mmPath),
				   mm.GetNodesExpanded(), mm.GetNecessaryExpansions(), t1.GetElapsedTime());
		}
		
		// MM0
		if (0)
		{
			MM<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>> mm;
			ZeroHeuristic<PancakePuzzleState<CNT>> z;
			goal.Reset();
			start = original;
			t4.StartTimer();
			mm.GetPath(&pancake, start, goal, &z, &z, mmPath);
			t4.EndTimer();
			printf("HARD-%d MM0 found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake.GetPathLength(mmPath),
				   mm.GetNodesExpanded(), mm.GetNecessaryExpansions(), t1.GetElapsedTime());
		}
	}
}

void Solve(Heuristic<PancakePuzzleState<CNT>> *h, const char *name)
{
	PancakePuzzle<CNT> pancake;
	PancakePuzzleState<CNT> start;
	PancakePuzzleState<CNT> goal;
	GetPancakeInstance(start, 11);
	goal.Reset();

	if (1)
	{
		BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>> p(start, goal, &pancake, h, h);
		p.drawProblemInstance = false;
		p.drawStatistics = false;
		p.drawMinimumVC = true;
		p.drawSumOnEdge = true;
		p.drawShortenedEdges = false;
		{
			p.drawProblemInstance = true;
			std::string s(name);
			s += "-instance.svg";
			p.SaveSVG(s.c_str());
			p.drawProblemInstance = false;
		}

		p.drawFullGraph = true;
		p.drawAllG = false;
		p.flipBackwardsGCost = false;
		{
			std::string s(name);
			s += "-ey-gn-fn.svg";
			p.SaveSVG(s.c_str());
		}

		p.drawFullGraph = true;
		p.drawAllG = false;
		p.flipBackwardsGCost = true;
		p.drawSumOnEdge = false;
		{
			std::string s(name);
			s += "-ey-gn-fy.svg";
			p.SaveSVG(s.c_str());
		}

		p.drawAllG = true;
		{
			std::string s(name);
			s += "-ey-gy-fy.svg";
			p.SaveSVG(s.c_str());
		}

		p.drawFullGraph = false;
		p.drawAllG = true;
		p.flipBackwardsGCost = true;
		p.drawSumOnEdge = true;
		{
			std::string s(name);
			s += "-en-gy-fy.svg";
			p.SaveSVG(s.c_str());
		}

		p.drawFullGraph = false;
		p.drawAllG = true;
		p.flipBackwardsGCost = true;
		p.drawShortenedEdges = true;
		p.drawSumOnEdge = true;
		{
			std::string s(name);
			s += "-es-gy-fy.svg";
			p.SaveSVG(s.c_str());
		}

}
	
	if (0)
	{
		std::vector<PancakePuzzleState<CNT>> nbsPath;
		NBS<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>> nbs;
		nbs.GetPath(&pancake, start, goal, h, h, nbsPath);
		printf("NBS found path length %1.0f; %llu expanded; %llu necessary; %f meeting\n", pancake.GetPathLength(nbsPath),
			   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), nbs.GetMeetingPoint());
	}
}

void TestError()
{
	srandom(2017218);
	PancakePuzzleState<CNT> goal;
  PancakePuzzleState<CNT> original;
	PancakePuzzle<4> pancake(2);
	PancakePuzzleState<4> smallGoal;
	PancakePuzzle<CNT> pancake0(0);
	PancakePuzzle<CNT> pancake1(1);
	PancakePuzzle<CNT> pancake2(2);
  PancakePuzzle<CNT> pancake3(3);
	OffsetHeuristic<PancakePuzzleState<CNT>> o1(&pancake0, 1);
	OffsetHeuristic<PancakePuzzleState<CNT>> o2(&pancake0, 2);
	OffsetHeuristic<PancakePuzzleState<CNT>> o3(&pancake0, 3);
	WeightedHeuristic<PancakePuzzleState<CNT>> w9(&pancake0, 0.9);
	WeightedHeuristic<PancakePuzzleState<CNT>> w8(&pancake0, 0.8);
	WeightedHeuristic<PancakePuzzleState<CNT>> w7(&pancake0, 0.7);
  float f;
	goal.Reset();
  
 	f = MeasureHeuristicErrors(&pancake0, goal, &pancake0, 5, 4, [](float i){return i <1;});
	printf("GAP\\0 Error percentage (5,4): %1.1f\n", f*100);
	f = MeasureHeuristicErrors(&pancake1, goal, &pancake1, 5, 4, [](float i){return i <1;});
	printf("GAP\\1 Error percentage (5,4): %1.1f\n", f*100);
	f = MeasureHeuristicErrors(&pancake2, goal, &pancake2, 5, 4, [](float i){return i <1;});
	printf("GAP\\2 Error percentage (5,4): %1.1f\n", f*100);
	f = MeasureHeuristicErrors(&pancake2, goal, &pancake3, 5, 4, [](float i){return i <1;});
	printf("GAP\\3 Error percentage (5,4): %1.1f\n", f*100);
	f = MeasureHeuristicErrors(&pancake0, goal, &o1, 5, 4, [](float i){return i <1;});
	printf("GAP-1 Error percentage (5,4): %1.1f\n", f*100);
	f = MeasureHeuristicErrors(&pancake0, goal, &o2, 5, 4, [](float i){return i <1;});
	printf("GAP-2 Error percentage (5,4): %1.1f\n", f*100);
	f = MeasureHeuristicErrors(&pancake0, goal, &o3, 5, 4, [](float i){return i <1;});
	printf("GAP-3 Error percentage (5,4): %1.1f\n", f*100);
	f = MeasureHeuristicErrors(&pancake0, goal, &w9, 5, 4, [](float i){return i <1;});
	printf("GAPx.9 Error percentage (5,4): %1.1f\n", f*100);
	f = MeasureHeuristicErrors(&pancake0, goal, &w8, 5, 4, [](float i){return i <1;});
	printf("GAPx.8 Error percentage (5,4): %1.1f\n", f*100);
	f = MeasureHeuristicErrors(&pancake0, goal, &w7, 5, 4, [](float i){return i <1;});
	printf("GAPx.7 Error percentage (5,4): %1.1f\n", f*100);
	
  srandom(2017218);
  for (int count = 0; count < 50; count++)
  {
    srandom(random());
    
    goal.Reset();
    original.Reset();
    for (int x = 0; x < CNT; x++)
      std::swap(original.puzzle[x], original.puzzle[x+random()%(CNT-x)]);

  

//	f = MeasureHeuristicErrors(&pancake, smallGoal, &pancake, 3, 2, [](float i){return i < 1;});
//	printf("GAP\\2 Error percentage: %1.1f (4-pancake)\n", f*100);

    f = MeasureHeuristicErrors(&pancake0, original, &pancake0, 5, 4, [](float i){return i <1;});
    printf("GAP\\0 Error percentage (5,4): %1.1f\n", f*100);
    f = MeasureHeuristicErrors(&pancake1, original, &pancake1, 5, 4, [](float i){return i <1;});
    printf("GAP\\1 Error percentage (5,4): %1.1f\n", f*100);
    f = MeasureHeuristicErrors(&pancake2, original, &pancake2, 5, 4, [](float i){return i <1;});
    printf("GAP\\2 Error percentage (5,4): %1.1f\n", f*100);
    f = MeasureHeuristicErrors(&pancake2, original, &pancake3, 5, 4, [](float i){return i <1;});
    printf("GAP\\3 Error percentage (5,4): %1.1f\n", f*100);
    f = MeasureHeuristicErrors(&pancake0, original, &o1, 5, 4, [](float i){return i <1;});
    printf("GAP-1 Error percentage (5,4): %1.1f\n", f*100);
    f = MeasureHeuristicErrors(&pancake0, original, &o2, 5, 4, [](float i){return i <1;});
    printf("GAP-2 Error percentage (5,4): %1.1f\n", f*100);
    f = MeasureHeuristicErrors(&pancake0, original, &o3, 5, 4, [](float i){return i <1;});
    printf("GAP-3 Error percentage (5,4): %1.1f\n", f*100);
    f = MeasureHeuristicErrors(&pancake0, original, &w9, 5, 4, [](float i){return i <1;});
    printf("GAPx.9 Error percentage (5,4): %1.1f\n", f*100);
    f = MeasureHeuristicErrors(&pancake0, original, &w8, 5, 4, [](float i){return i <1;});
    printf("GAPx.8 Error percentage (5,4): %1.1f\n", f*100);
    f = MeasureHeuristicErrors(&pancake0, original, &w7, 5, 4, [](float i){return i <1;});
    printf("GAPx.7 Error percentage (5,4): %1.1f\n", f*100);
  }
	// printf("\n----\n\n");
	
	// f = MeasureHeuristicErrors(&pancake0, goal, &pancake0, 5, 3, [](float i){return i <2;});
	// printf("GAP\\0 Error percentage (5,3): %1.1f\n", f*100);
	// f = MeasureHeuristicErrors(&pancake1, goal, &pancake1, 5, 3, [](float i){return i <2;});
	// printf("GAP\\1 Error percentage (5,3): %1.1f\n", f*100);
	// f = MeasureHeuristicErrors(&pancake2, goal, &pancake2, 5, 3, [](float i){return i <2;});
	// printf("GAP\\2 Error percentage (5,3): %1.1f\n", f*100);
	// f = MeasureHeuristicErrors(&pancake2, goal, &pancake3, 5, 3, [](float i){return i <2;});
	// printf("GAP\\2 Error percentage (5,3): %1.1f\n", f*100);
	// f = MeasureHeuristicErrors(&pancake0, goal, &o1, 5, 3, [](float i){return i <2;});
	// printf("GAP-1 Error percentage (5,3): %1.1f\n", f*100);
	// f = MeasureHeuristicErrors(&pancake0, goal, &o2, 5, 3, [](float i){return i <2;});
	// printf("GAP-2 Error percentage (5,3): %1.1f\n", f*100);
	// f = MeasureHeuristicErrors(&pancake0, goal, &o3, 5, 3, [](float i){return i <2;});
	// printf("GAP-3 Error percentage (5,3): %1.1f\n", f*100);
	// f = MeasureHeuristicErrors(&pancake0, goal, &w9, 5, 3, [](float i){return i <2;});
	// printf("GAPx.9 Error percentage (5,3): %1.1f\n", f*100);
	// f = MeasureHeuristicErrors(&pancake0, goal, &w8, 5, 3, [](float i){return i <2;});
	// printf("GAPx.8 Error percentage (5,3): %1.1f\n", f*100);
	// f = MeasureHeuristicErrors(&pancake0, goal, &w7, 5, 3, [](float i){return i <2;});
	// printf("GAPx.7 Error percentage (5,3): %1.1f\n", f*100);

	
	// f = MeasureHeuristicErrors(&pancake0, goal, &w9, 3, 2, [](float i){return i>=1 && i <2;});
	// printf("GAPx.9 Error percentage (3,1): %1.1f\n", f*100);
	// f = MeasureHeuristicErrors(&pancake0, goal, &w8, 3, 2, [](float i){return i>=1 && i <2;});
	// printf("GAPx.8 Error percentage (3,1): %1.1f\n", f*100);
	// f = MeasureHeuristicErrors(&pancake0, goal, &w7, 3, 2, [](float i){return i>=1 && i <2;});
	// printf("GAPx.7 Error percentage (3,1): %1.1f\n", f*100);

	exit(0);
}


void TestVariants()
{
	srandom(2017218);
	PancakePuzzleState<CNT> start;
	PancakePuzzleState<CNT> original;
	PancakePuzzleState<CNT> goal;
	PancakePuzzle<CNT> pancake;
	PancakePuzzle<CNT> pancake0(0);
	PancakePuzzle<CNT> pancake1(1);
	PancakePuzzle<CNT> pancake2(2);
	OffsetHeuristic<PancakePuzzleState<CNT>> o1(&pancake0, 1);
	OffsetHeuristic<PancakePuzzleState<CNT>> o2(&pancake0, 2);
	OffsetHeuristic<PancakePuzzleState<CNT>> o3(&pancake0, 3);
	WeightedHeuristic<PancakePuzzleState<CNT>> w9(&pancake0, 0.9);
	WeightedHeuristic<PancakePuzzleState<CNT>> w8(&pancake0, 0.8);
	WeightedHeuristic<PancakePuzzleState<CNT>> w7(&pancake0, 0.7);

//	Solve(&pancake0, "/Users/nathanst/bidir/pancake/p11_G0");
//	Solve(&pancake1, "/Users/nathanst/bidir/pancake/p11_G1");
//	Solve(&pancake2, "/Users/nathanst/bidir/pancake/p11_G2");
	Solve(&pancake2, "/Users/nathanst/bidir/pancake/p11_G2-E");
//	Solve(&o1, "/Users/nathanst/bidir/pancake/p11_O1");
//	Solve(&o2, "/Users/nathanst/bidir/pancake/p11_O2");
//	Solve(&o3, "/Users/nathanst/bidir/pancake/p11_O3");
//	Solve(&w9, "/Users/nathanst/bidir/pancake/p11_W9");
//	Solve(&w8, "/Users/nathanst/bidir/pancake/p11_W8");
//	Solve(&w7, "/Users/nathanst/bidir/pancake/p11_W7");
}


void TestPancake_overall()
{
  std::map<double,int> h1_overall_vals;
  std::map<double,int> h2_overall_vals;
  std::map<double,int> h1_astar_vals;
  std::map<double,int> h2_astar_vals;
  std::map<double,int> h1_rastar_vals;
  std::map<double,int> h2_rastar_vals;
  std::map<double,int> h1_real_bd_vals;
  std::map<double,int> h2_real_bd_vals;
  std::vector<PancakePuzzleState<CNT>> nbsPath;
  std::vector<PancakePuzzleState<CNT>> astarPath;
  TemplateAStar<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>> astar;
  NBS<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>> nbs;
  
  PancakePuzzleState<CNT> goal;
  PancakePuzzleState<CNT> original;
  PancakePuzzle<CNT> pancake;
  
  PancakePuzzle<CNT> pancake0(0);
  PancakePuzzle<CNT> pancake2(2);
  PancakePuzzle<CNT> pancake3(3);
  std::vector<int> g1 = {0,1,2};
  std::vector<int> g2 = {3,4,5};
  std::vector<int> g3 = {6,7,8};
  std::vector<int> g4 = {9,10,11};
  PancakePuzzleGapVector<CNT> pancake31(g1);
  PancakePuzzleGapVector<CNT> pancake32(g2);
  PancakePuzzleGapVector<CNT> pancake33(g3);
  PancakePuzzleGapVector<CNT> pancake34(g4);
  
  Heuristic<PancakePuzzleState<CNT>> maxgap3_4;
  //Heuristic<PancakePuzzleState<CNT>> maxgap3_3;
  //Heuristic<PancakePuzzleState<CNT>> maxgap3_2;
  

  maxgap3_4.lookups.resize(0);
  maxgap3_4.lookups.push_back({kMaxNode, 1, 4});
  maxgap3_4.lookups.push_back({kLeafNode, 0, 0});
  maxgap3_4.lookups.push_back({kLeafNode, 1, 1});
  maxgap3_4.lookups.push_back({kLeafNode, 2, 2});
  maxgap3_4.lookups.push_back({kLeafNode, 3, 3});
//	h.lookups.push_back({kLeafNode, 4, 4});
  maxgap3_4.heuristics.resize(0);
  maxgap3_4.heuristics.push_back(&pancake31);
  maxgap3_4.heuristics.push_back(&pancake32);
  maxgap3_4.heuristics.push_back(&pancake33);
  maxgap3_4.heuristics.push_back(&pancake34);
  

  Timer t1;
  srandom(2017218);
  for (int count = 0; count < 50; count++)
  {
    srandom(random());
    
    goal.Reset();
    original.Reset();
    for (int x = 0; x < CNT; x++)
      std::swap(original.puzzle[x], original.puzzle[x+random()%(CNT-x)]);
      if (1){
        BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>> p(original, goal, &pancake0, &pancake2, &pancake2);
        p.drawProblemInstance = false;
        p.drawStatistics = false;
        p.drawAllG = true;
        p.flipBackwardsGCost = true;
        p.SaveSVG((std::to_string(count)+"_GAP2.svg").c_str());        
        BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>> p1(original, goal, &pancake0, &maxgap3_4, &maxgap3_4);
        p1.drawProblemInstance = false;
        p1.drawStatistics = false;
        p1.drawAllG = true;
        p1.flipBackwardsGCost = true;
        p1.SaveSVG((std::to_string(count)+"_GAP3MAX4.svg").c_str());        
      }
      if (0){
        t1.StartTimer();
        astar.SetHeuristic(&pancake2);
        astar.GetPath(&pancake0, original, goal, astarPath);
        t1.EndTimer();
        printf("A* GAP\\2 found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", pancake0.GetPathLength(astarPath),
             astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), astar.GetNodesTouched(), t1.GetElapsedTime());
        for (int x = 0; x < astar.GetNumItems(); x++)
        {
          const auto &i = astar.GetItem(x);
          if (i.where != kClosedList)
            continue;
          int h_value = i.h;
          if (h1_astar_vals.find(h_value) == h1_astar_vals.end()){
            h1_astar_vals.insert(std::make_pair(h_value,1));
          }
          else{
            h1_astar_vals[h_value]++;
          }
        }
        t1.StartTimer();
        astar.SetHeuristic(&pancake2);
        astar.GetPath(&pancake0, goal, original, astarPath);
        t1.EndTimer();
        printf("r-A* GAP\\2 found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", pancake0.GetPathLength(astarPath),
             astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), astar.GetNodesTouched(), t1.GetElapsedTime());
        for (int x = 0; x < astar.GetNumItems(); x++)
        {
          const auto &i = astar.GetItem(x);
          if (i.where != kClosedList)
            continue;
          int h_value = i.h;
          if (h1_rastar_vals.find(h_value) == h1_rastar_vals.end()){
            h1_rastar_vals.insert(std::make_pair(h_value,1));
          }
          else{
            h1_rastar_vals[h_value]++;
          }
        }
        t1.StartTimer();
        nbs.GetPath(&pancake0, original, goal, &pancake2, &pancake2, nbsPath);
        t1.EndTimer();
        printf("NBS GAP\\2 found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", pancake0.GetPathLength(nbsPath),
             nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), nbs.GetNodesTouched(), t1.GetElapsedTime());
             
        for (int x = 0; x < nbs.GetNumForwardItems(); x++)
        {
          const auto &i = nbs.GetForwardItem(x);
          if (i.where != kClosed)
            continue;
          int h_value = i.h;
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
          int h_value = i.h;
          if (h1_real_bd_vals.find(h_value) == h1_real_bd_vals.end()){
            h1_real_bd_vals.insert(std::make_pair(h_value,1));
          }
          else{
            h1_real_bd_vals[h_value]++;
          }
        }
        
        
        
        
        
        t1.StartTimer();
        astar.SetHeuristic(&maxgap3_4);
        astar.GetPath(&pancake0, original, goal, astarPath);
        t1.EndTimer();
        printf("A* GAP\\3-MAX(4) found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", pancake0.GetPathLength(astarPath),
             astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), astar.GetNodesTouched(), t1.GetElapsedTime());
        for (int x = 0; x < astar.GetNumItems(); x++)
        {
          const auto &i = astar.GetItem(x);
          if (i.where != kClosedList)
            continue;
          int h_value = i.h;
          if (h2_astar_vals.find(h_value) == h2_astar_vals.end()){
            h2_astar_vals.insert(std::make_pair(h_value,1));
          }
          else{
            h2_astar_vals[h_value]++;
          }
        }
        t1.StartTimer();
        astar.SetHeuristic(&maxgap3_4);
        astar.GetPath(&pancake0, goal, original, astarPath);
        t1.EndTimer();
        printf("r-A* GAP\\3-MAX(4) found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", pancake0.GetPathLength(astarPath),
             astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), astar.GetNodesTouched(), t1.GetElapsedTime());
        for (int x = 0; x < astar.GetNumItems(); x++)
        {
          const auto &i = astar.GetItem(x);
          if (i.where != kClosedList)
            continue;
          int h_value = i.h;
          if (h2_rastar_vals.find(h_value) == h2_rastar_vals.end()){
            h2_rastar_vals.insert(std::make_pair(h_value,1));
          }
          else{
            h2_rastar_vals[h_value]++;
          }
        }
        t1.StartTimer();
        nbs.GetPath(&pancake0, original, goal, &maxgap3_4, &maxgap3_4, nbsPath);
        t1.EndTimer();
        printf("NBS GAP\\3-MAX(4) found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", pancake0.GetPathLength(nbsPath),
             nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), nbs.GetNodesTouched(), t1.GetElapsedTime());
             
        for (int x = 0; x < nbs.GetNumForwardItems(); x++)
        {
          const auto &i = nbs.GetForwardItem(x);
          if (i.where != kClosed)
            continue;
          int h_value = i.h;
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
          int h_value = i.h;
          if (h2_real_bd_vals.find(h_value) == h2_real_bd_vals.end()){
            h2_real_bd_vals.insert(std::make_pair(h_value,1));
          }
          else{
            h2_real_bd_vals[h_value]++;
          }
        }
      }
  }
  if(0){
    for (int i = 0; i< 10000000; i++){
      original.Reset();
      for (int x = 0; x < CNT; x++)
        std::swap(original.puzzle[x], original.puzzle[x+random()%(CNT-x)]);

      double h_1_val = pancake2.HCost(original,goal);
      double h_2_val = maxgap3_4.HCost(original,goal);
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

void TestPancakeHeuristic()
{
  printf("TestPancakeHeuristic\n");
  srandom(2017218);
  PancakePuzzleState<CNT> start;
  PancakePuzzleState<CNT> original;
  PancakePuzzleState<CNT> goal;
  

  PancakePuzzle<CNT> pancake0(0);
  PancakePuzzle<CNT> pancake1(1);
  PancakePuzzle<CNT> pancake2(2);
  PancakePuzzle<CNT> pancake3(3);
  std::vector<int> g1 = {0,1,2};
  std::vector<int> g2 = {3,4,5};
  std::vector<int> g3 = {6,7,8};
  std::vector<int> g4 = {9,10,11};
  PancakePuzzleGapVector<CNT> pancake31(g1);
  PancakePuzzleGapVector<CNT> pancake32(g2);
  PancakePuzzleGapVector<CNT> pancake33(g3);
  PancakePuzzleGapVector<CNT> pancake34(g4);
  
  Heuristic<PancakePuzzleState<CNT>> maxgap3_4;
  //Heuristic<PancakePuzzleState<CNT>> maxgap3_3;
  //Heuristic<PancakePuzzleState<CNT>> maxgap3_2;
  

  maxgap3_4.lookups.resize(0);
  maxgap3_4.lookups.push_back({kMaxNode, 1, 4});
  maxgap3_4.lookups.push_back({kLeafNode, 0, 0});
  maxgap3_4.lookups.push_back({kLeafNode, 1, 1});
  maxgap3_4.lookups.push_back({kLeafNode, 2, 2});
  maxgap3_4.lookups.push_back({kLeafNode, 3, 3});
//	h.lookups.push_back({kLeafNode, 4, 4});
  maxgap3_4.heuristics.resize(0);
  maxgap3_4.heuristics.push_back(&pancake31);
  maxgap3_4.heuristics.push_back(&pancake32);
  maxgap3_4.heuristics.push_back(&pancake33);
  maxgap3_4.heuristics.push_back(&pancake34);
  
  // maxgap3_3.lookups.resize(0);
  // maxgap3_3.lookups.push_back({kMaxNode, 1, 3});
  // maxgap3_3.lookups.push_back({kLeafNode, 0, 0});
  // maxgap3_3.lookups.push_back({kLeafNode, 1, 1});
  // maxgap3_3.lookups.push_back({kLeafNode, 2, 2});
  // maxgap3_3.heuristics.resize(0);
  // maxgap3_3.heuristics.push_back(&pancake31);
  // maxgap3_3.heuristics.push_back(&pancake32);
  // maxgap3_3.heuristics.push_back(&pancake33);
  
    // maxgap3_2.lookups.resize(0);
  // maxgap3_2.lookups.push_back({kMaxNode, 1, 2});
  // maxgap3_2.lookups.push_back({kLeafNode, 0, 0});
  // maxgap3_2.lookups.push_back({kLeafNode, 1, 1});
  // maxgap3_2.heuristics.resize(0);
  // maxgap3_2.heuristics.push_back(&pancake31);
  // maxgap3_2.heuristics.push_back(&pancake32);

  // Heuristic<PancakePuzzleState<CNT>> maxgap2_6;
  // Heuristic<PancakePuzzleState<CNT>> maxgap2_5;
  // Heuristic<PancakePuzzleState<CNT>> maxgap2_4;
  // Heuristic<PancakePuzzleState<CNT>> maxgap2_3;
  // Heuristic<PancakePuzzleState<CNT>> maxgap2_2;

  // std::vector<int> g2_1 = {0,1};
  // std::vector<int> g2_2 = {2,3};
  // std::vector<int> g2_3 = {4,5};
  // std::vector<int> g2_4 = {6,7};
  // std::vector<int> g2_5 = {8,9};
  // std::vector<int> g2_6 = {10,11};
  
  // PancakePuzzleGapVector<CNT> pancake21(g2_1);
  // PancakePuzzleGapVector<CNT> pancake22(g2_2);
  // PancakePuzzleGapVector<CNT> pancake23(g2_3);
  // PancakePuzzleGapVector<CNT> pancake24(g2_4);
  // PancakePuzzleGapVector<CNT> pancake25(g2_5);
  // PancakePuzzleGapVector<CNT> pancake26(g2_6);
  
  // maxgap2_6.lookups.resize(0);
  // maxgap2_6.lookups.push_back({kMaxNode, 1, 6});
  // maxgap2_6.lookups.push_back({kLeafNode, 0, 0});
  // maxgap2_6.lookups.push_back({kLeafNode, 1, 1});
  // maxgap2_6.lookups.push_back({kLeafNode, 2, 2});
  // maxgap2_6.lookups.push_back({kLeafNode, 3, 3});
  // maxgap2_6.lookups.push_back({kLeafNode, 4, 4});
  // maxgap2_6.lookups.push_back({kLeafNode, 5, 5});
  // maxgap2_6.heuristics.resize(0);
  // maxgap2_6.heuristics.push_back(&pancake21);
  // maxgap2_6.heuristics.push_back(&pancake22);
  // maxgap2_6.heuristics.push_back(&pancake23);
  // maxgap2_6.heuristics.push_back(&pancake24);
  // maxgap2_6.heuristics.push_back(&pancake25);
  // maxgap2_6.heuristics.push_back(&pancake26);
  
  // maxgap2_5.lookups.resize(0);
  // maxgap2_5.lookups.push_back({kMaxNode, 1, 5});
  // maxgap2_5.lookups.push_back({kLeafNode, 0, 0});
  // maxgap2_5.lookups.push_back({kLeafNode, 1, 1});
  // maxgap2_5.lookups.push_back({kLeafNode, 2, 2});
  // maxgap2_5.lookups.push_back({kLeafNode, 3, 3});
  // maxgap2_5.lookups.push_back({kLeafNode, 4, 4});
  // maxgap2_5.heuristics.resize(0);
  // maxgap2_5.heuristics.push_back(&pancake21);
  // maxgap2_5.heuristics.push_back(&pancake22);
  // maxgap2_5.heuristics.push_back(&pancake23);
  // maxgap2_5.heuristics.push_back(&pancake24);
  // maxgap2_5.heuristics.push_back(&pancake25);
  
  // maxgap2_4.lookups.resize(0);
  // maxgap2_4.lookups.push_back({kMaxNode, 1, 4});
  // maxgap2_4.lookups.push_back({kLeafNode, 0, 0});
  // maxgap2_4.lookups.push_back({kLeafNode, 1, 1});
  // maxgap2_4.lookups.push_back({kLeafNode, 2, 2});
  // maxgap2_4.lookups.push_back({kLeafNode, 3, 3});
  // maxgap2_4.heuristics.resize(0);
  // maxgap2_4.heuristics.push_back(&pancake21);
  // maxgap2_4.heuristics.push_back(&pancake22);
  // maxgap2_4.heuristics.push_back(&pancake23);
  // maxgap2_4.heuristics.push_back(&pancake24);

  // maxgap2_3.lookups.resize(0);
  // maxgap2_3.lookups.push_back({kMaxNode, 1, 3});
  // maxgap2_3.lookups.push_back({kLeafNode, 0, 0});
  // maxgap2_3.lookups.push_back({kLeafNode, 1, 1});
  // maxgap2_3.lookups.push_back({kLeafNode, 2, 2});
  // maxgap2_3.heuristics.resize(0);
  // maxgap2_3.heuristics.push_back(&pancake21);
  // maxgap2_3.heuristics.push_back(&pancake22);
  // maxgap2_3.heuristics.push_back(&pancake23);

  // maxgap2_2.lookups.resize(0);
  // maxgap2_2.lookups.push_back({kMaxNode, 1, 2});
  // maxgap2_2.lookups.push_back({kLeafNode, 0, 0});
  // maxgap2_2.lookups.push_back({kLeafNode, 1, 1});
  // maxgap2_2.heuristics.resize(0);
  // maxgap2_2.heuristics.push_back(&pancake21);
  // maxgap2_2.heuristics.push_back(&pancake22);

  srandom(2017218);
  for (int count = 0; count < 50; count++)
  {
    srandom(random());
    
    goal.Reset();
    original.Reset();
    for (int x = 0; x < CNT; x++)
      std::swap(original.puzzle[x], original.puzzle[x+random()%(CNT-x)]);
    // printf("GAP" );
    // BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>>::GetWeightedVertexGraph(original, goal, &pancake0, &pancake0, &pancake0);
    // printf("GAP 1" );
    // BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>>::GetWeightedVertexGraph(original, goal, &pancake0, &pancake1, &pancake1);
    //printf("GAP 2" );
    //BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>>::GetWeightedVertexGraph(original, goal, &pancake0, &pancake2, &pancake2);
    // printf("GAP 2 MAX(2)" );
    // BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>>::GetWeightedVertexGraph(original, goal, &pancake0, &maxgap2_2, &maxgap2_2);   
    // printf("GAP 2 MAX(3)" );
    // BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>>::GetWeightedVertexGraph(original, goal, &pancake0, &maxgap2_3, &maxgap2_3);
    // printf("GAP 2 MAX(4)" );
    // BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>>::GetWeightedVertexGraph(original, goal, &pancake0, &maxgap2_4, &maxgap2_4);
    // printf("GAP 2 MAX(5)" );
    // BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>>::GetWeightedVertexGraph(original, goal, &pancake0, &maxgap2_5, &maxgap2_5);
    // printf("GAP 2 MAX(6)" );
    // BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>>::GetWeightedVertexGraph(original, goal, &pancake0, &maxgap2_6, &maxgap2_6);
    // printf("GAP 3" );
    // BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>>::GetWeightedVertexGraph(original, goal, &pancake0, &pancake3, &pancake3);  
    // printf("GAP 3 MAX(2)" );
    // BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>>::GetWeightedVertexGraph(original, goal, &pancake0, &maxgap3_2, &maxgap3_2);   
    // printf("GAP 3 MAX(3)" );
    // BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>>::GetWeightedVertexGraph(original, goal, &pancake0, &maxgap3_3, &maxgap3_3);
    //printf("GAP 3 MAX(4)" );
    //BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>>::GetWeightedVertexGraph(original, goal, &pancake0, &maxgap3_4, &maxgap3_4);
  }
}


void TestPancakeLowHeuristic(int rad)
{
  printf("TestPancakeLowHeuristic %d\n",rad);
  int count_gap0,count_gap1,count_gap2,count_gap3,count_o1,count_o2,count_o3,count_w9,count_w8,count_w7,icount_gap0,icount_gap1,icount_gap2,icount_gap3,icount_o1,icount_o2,icount_o3,icount_w9,icount_w8,icount_w7,expansions;
  count_gap0 = count_gap1 = count_gap2 = count_gap3 = count_o1 = count_o2 = count_o3 = count_w9 = count_w8 = count_w7 = icount_gap0 = icount_gap1 = icount_gap2 = icount_gap3 = icount_o1 = icount_o2 = icount_o3 = icount_w9 = icount_w8 = icount_w7 = expansions = 0;
  PancakePuzzle<CNT> pancake0(0);
  PancakePuzzle<CNT> pancake1(1);
  PancakePuzzle<CNT> pancake2(2);
  PancakePuzzle<CNT> pancake3(3);
  OffsetHeuristic<PancakePuzzleState<CNT>> o1(&pancake0, 1);
  OffsetHeuristic<PancakePuzzleState<CNT>> o2(&pancake0, 2);
  OffsetHeuristic<PancakePuzzleState<CNT>> o3(&pancake0, 3);
  WeightedHeuristic<PancakePuzzleState<CNT>> w9(&pancake0, 0.9);
  WeightedHeuristic<PancakePuzzleState<CNT>> w8(&pancake0, 0.8);
  WeightedHeuristic<PancakePuzzleState<CNT>> w7(&pancake0, 0.7);
  ZeroHeuristic<PancakePuzzleState<CNT>> z;
  
  TemplateAStar<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>> astar;
	std::vector<PancakePuzzleState<CNT>> astarPath;
  NBS<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>, NBSQueue<PancakePuzzleState<CNT>, 1, true>> nbs(true);
	std::vector<PancakePuzzleState<CNT>> nbsPath;
	std::vector<PancakePuzzleState<CNT>> dvcbsPath;
  Timer t1;
  PancakePuzzleState<CNT> original,start;
  PancakePuzzleState<CNT> goal;
  goal.Reset();
  original.Reset();
  if(1){
    int limit = 1000000;
    t1.StartTimer();
    srandom(3201561);
    for (int count = 0; count < limit; count++)
    {
      srandom(random());
      
      goal.Reset();
      original.Reset();
      for (int x = 0; x < CNT; x++)
        std::swap(original.puzzle[x], original.puzzle[x+random()%(CNT-x)]);
      if (min(pancake0.HCost(original,goal),min(pancake1.HCost(original,goal),min(pancake2.HCost(original,goal),min(pancake3.HCost(original,goal),min(o1.HCost(original,goal),min(o2.HCost(original,goal),min(o3.HCost(original,goal),min(w9.HCost(original,goal),min(w8.HCost(original,goal),w7.HCost(original,goal)))))))))) <= rad){
        
        astar.GetPath(&pancake0, original, goal, astarPath);
        expansions+= astar.GetNodesExpanded();
        //printf("Problem %d of %d\n", count+1, 50);
        //std::cout << original << "\n";
        if (pancake0.HCost(original,goal) <= rad){
          count_gap0++;
          if (pancake0.GetPathLength(astarPath) != pancake0.HCost(original,goal)){
            icount_gap0++;
          }
        }
        if (pancake1.HCost(original,goal) <= rad){
          count_gap1++;
          if (pancake0.GetPathLength(astarPath) != pancake1.HCost(original,goal)){
            icount_gap1++;
          }
        }
        if (pancake2.HCost(original,goal) <= rad){
          count_gap2++;
          if (pancake0.GetPathLength(astarPath) != pancake2.HCost(original,goal)){
            icount_gap2++;
          }
        }
        if (pancake3.HCost(original,goal) <= rad){
          count_gap3++;
          if (pancake0.GetPathLength(astarPath) != pancake3.HCost(original,goal)){
            icount_gap3++;
          }
        }
        if (o1.HCost(original,goal) <= rad){
          count_o1++;
          if (pancake0.GetPathLength(astarPath) != o1.HCost(original,goal)){
            icount_o1++;
          }
        }
        if (o2.HCost(original,goal) <= rad){
          count_o2++;
          if (pancake0.GetPathLength(astarPath) != o2.HCost(original,goal)){
            icount_o2++;
          }
        }
        if (o3.HCost(original,goal) <= rad){
          count_o3++;
          if (pancake0.GetPathLength(astarPath) != o3.HCost(original,goal)){
            icount_o3++;
          }
        }
        if (w9.HCost(original,goal) <= rad){
          count_w9++;
          if (pancake0.GetPathLength(astarPath) != w9.HCost(original,goal)){
            icount_w9++;
          }
        }
        if (w8.HCost(original,goal) <= rad){
          count_w8++;
          if (pancake0.GetPathLength(astarPath) != w8.HCost(original,goal)){
            icount_w8++;
          }
        }
        if (w7.HCost(original,goal) <= rad){
          count_w7++;
          if (pancake0.GetPathLength(astarPath) != w7.HCost(original,goal)){
            icount_w7++;
          }
        }
      }
    }
    t1.EndTimer();
    printf("%d %d %d %d %d %d %d %d %d %d %d\n",count_gap0,count_gap1,count_gap2,count_gap3,count_o1,count_o2,count_o3,count_w9,count_w8,count_w7,limit);
    printf("%d %d %d %d %d %d %d %d %d %d %d\n",icount_gap0,icount_gap1,icount_gap2,icount_gap3,icount_o1,icount_o2,icount_o3,icount_w9,icount_w8,icount_w7,limit);
    printf("%1.2f %d\n",t1.GetElapsedTime(),expansions); 
  }
  if(1){
  srandom(2017218);
    t1.StartTimer();
    printf("goal ");
    StateNeighborsUpToDistance(&pancake0, goal, &pancake0, rad);
    StateNeighborsUpToDistance(&pancake0, goal, &pancake1, rad);
    StateNeighborsUpToDistance(&pancake0, goal, &pancake2, rad);
    StateNeighborsUpToDistance(&pancake0, goal, &pancake3, rad);
    StateNeighborsUpToDistance(&pancake0, goal, &o1, rad);
    StateNeighborsUpToDistance(&pancake0, goal, &o2, rad);
    StateNeighborsUpToDistance(&pancake0, goal, &o3, rad);
    StateNeighborsUpToDistance(&pancake0, goal, &w9, rad);
    StateNeighborsUpToDistance(&pancake0, goal, &w8, rad);
    StateNeighborsUpToDistance(&pancake0, goal, &w7, rad);
    //StateNeighborsUpToDistance(&pancake0, goal, &z, rad);
    t1.EndTimer();
    printf("%1.2f\n",t1.GetElapsedTime()); 
    printf("start ");
  }
  for (int count = 0; count < 50; count++)
	{

    srandom(random());
    
    goal.Reset();
    original.Reset();
    for (int x = 0; x < CNT; x++)
      std::swap(original.puzzle[x], original.puzzle[x+random()%(CNT-x)]);
    start = original;
    // A*
    if(0){ //run algorithms
      if (1)
      {
        t1.StartTimer();
        astar.SetHeuristic(&pancake0);
        astar.GetPath(&pancake0, start, goal, astarPath);
        t1.EndTimer();
        printf("%d GAP A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(astarPath),
             astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());
             
        t1.StartTimer();
        astar.SetHeuristic(&pancake1);
        astar.GetPath(&pancake0, start, goal, astarPath);
        t1.EndTimer();
        printf("%d GAP\\1 A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(astarPath),
             astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());
             
        t1.StartTimer();
        astar.SetHeuristic(&pancake2);
        astar.GetPath(&pancake0, start, goal, astarPath);
        t1.EndTimer();
        printf("%d GAP\\2 A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(astarPath),
             astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());

        t1.StartTimer();
        astar.SetHeuristic(&pancake3);
        astar.GetPath(&pancake0, start, goal, astarPath);
        t1.EndTimer();
        printf("%d GAP\\3 A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(astarPath),
             astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());

        t1.StartTimer();
        astar.SetHeuristic(&o1);
        astar.GetPath(&pancake0, start, goal, astarPath);
        t1.EndTimer();
        printf("%d GAP-1 A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(astarPath),
             astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());

        t1.StartTimer();
        astar.SetHeuristic(&o2);
        astar.GetPath(&pancake0, start, goal, astarPath);
        t1.EndTimer();
        printf("%d GAP-2 A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(astarPath),
             astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());      

        t1.StartTimer();
        astar.SetHeuristic(&o3);
        astar.GetPath(&pancake0, start, goal, astarPath);
        t1.EndTimer();
        printf("%d GAP-3 A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(astarPath),
             astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());    

        t1.StartTimer();
        astar.SetHeuristic(&w9);
        astar.GetPath(&pancake0, start, goal, astarPath);
        t1.EndTimer();
        printf("%d GAP0.9 A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(astarPath),
             astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());

        t1.StartTimer();
        astar.SetHeuristic(&w8);
        astar.GetPath(&pancake0, start, goal, astarPath);
        t1.EndTimer();
        printf("%d GAP0.8 A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(astarPath),
             astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());      

        t1.StartTimer();
        astar.SetHeuristic(&w7);
        astar.GetPath(&pancake0, start, goal, astarPath);
        t1.EndTimer();
        printf("%d GAP0.7 A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(astarPath),
             astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());            
      }

      // Reverse A*
      if (0)
      {
        t1.StartTimer();
        astar.SetHeuristic(&pancake0);
        astar.GetPath(&pancake0, goal, start, astarPath);
        t1.EndTimer();
        printf("%d GAP r-A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(astarPath),
             astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());
             
        t1.StartTimer();
        astar.SetHeuristic(&pancake1);
        astar.GetPath(&pancake0, goal, start, astarPath);
        t1.EndTimer();
        printf("%d GAP\\1 r-A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(astarPath),
             astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());
             
        t1.StartTimer();
        astar.SetHeuristic(&pancake2);
        astar.GetPath(&pancake0, goal, start, astarPath);
        t1.EndTimer();
        printf("%d GAP\\2 r-A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(astarPath),
             astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());

        t1.StartTimer();
        astar.SetHeuristic(&pancake3);
        astar.GetPath(&pancake0, goal, start, astarPath);
        t1.EndTimer();
        printf("%d GAP\\3 r-A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(astarPath),
             astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());

        t1.StartTimer();
        astar.SetHeuristic(&o1);
        astar.GetPath(&pancake0, goal, start, astarPath);
        t1.EndTimer();
        printf("%d GAP-1 r-A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(astarPath),
             astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());

        t1.StartTimer();
        astar.SetHeuristic(&o2);
        astar.GetPath(&pancake0, goal, start, astarPath);
        t1.EndTimer();
        printf("%d GAP-2 r-A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(astarPath),
             astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());      

        t1.StartTimer();
        astar.SetHeuristic(&o3);
        astar.GetPath(&pancake0, goal, start, astarPath);
        t1.EndTimer();
        printf("%d GAP-3 r-A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(astarPath),
             astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());    

        t1.StartTimer();
        astar.SetHeuristic(&w9);
        astar.GetPath(&pancake0, goal, start, astarPath);
        t1.EndTimer();
        printf("%d GAP0.9 r-A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(astarPath),
             astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());

        t1.StartTimer();
        astar.SetHeuristic(&w8);
        astar.GetPath(&pancake0, goal, start, astarPath);
        t1.EndTimer();
        printf("%d GAP0.8 r-A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(astarPath),
             astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());      

        t1.StartTimer();
        astar.SetHeuristic(&w7);
        astar.GetPath(&pancake0, goal, start, astarPath);
        t1.EndTimer();
        printf("%d GAP0.7 r-A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(astarPath),
             astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime()); 
      }
      
      // NBS e=1
      if (0)
      {
        t1.StartTimer();
        nbs.GetPath(&pancake0, start, goal,&pancake0, &pancake0, nbsPath);
        t1.EndTimer();
        printf("%d GAP NBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(nbsPath),
             nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t1.GetElapsedTime());
             
        t1.StartTimer();
        nbs.GetPath(&pancake0, start, goal,&pancake1, &pancake1, nbsPath);
        t1.EndTimer();
        printf("%d GAP\\1 NBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(nbsPath),
             nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t1.GetElapsedTime());
             
        t1.StartTimer();
        nbs.GetPath(&pancake0, start, goal,&pancake2, &pancake2, nbsPath);
        t1.EndTimer();
        printf("%d GAP\\2 NBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(nbsPath),
             nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t1.GetElapsedTime());

        t1.StartTimer();
        nbs.GetPath(&pancake0, start, goal,&pancake3, &pancake3, nbsPath);
        t1.EndTimer();
        printf("%d GAP\\3 NBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(nbsPath),
             nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t1.GetElapsedTime());

        t1.StartTimer();
        nbs.GetPath(&pancake0, start, goal,&o1, &o1, nbsPath);
        t1.EndTimer();
        printf("%d GAP-1 NBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(nbsPath),
             nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t1.GetElapsedTime());

        t1.StartTimer();
        nbs.GetPath(&pancake0, start, goal,&o2, &o2, nbsPath);
        t1.EndTimer();
        printf("%d GAP-2 NBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(nbsPath),
             nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t1.GetElapsedTime());      

        t1.StartTimer();
        nbs.GetPath(&pancake0, start, goal,&o3, &o3, nbsPath);
        t1.EndTimer();
        printf("%d GAP-3 NBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(nbsPath),
             nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t1.GetElapsedTime());    

        t1.StartTimer();
        nbs.GetPath(&pancake0, start, goal,&w9, &w9, nbsPath);
        t1.EndTimer();
        printf("%d GAP0.9 NBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(nbsPath),
             nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t1.GetElapsedTime());

        t1.StartTimer();
        nbs.GetPath(&pancake0, start, goal,&w8, &w8, nbsPath);
        t1.EndTimer();
        printf("%d GAP0.8 NBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(nbsPath),
             nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t1.GetElapsedTime());      

        t1.StartTimer();
        nbs.GetPath(&pancake0, start, goal,&w7, &w7, nbsPath);
        t1.EndTimer();
        printf("%d GAP0.7 NBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(nbsPath),
             nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t1.GetElapsedTime());            
      }
      // DVCBS
      if (0)
      {
        DVCBS<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>,DVCBSQueue<PancakePuzzleState<CNT>,1,true>> dvcbs(false,true);
        t1.StartTimer();
        dvcbs.GetPath(&pancake0, start, goal,&pancake0, &pancake0, dvcbsPath);
        t1.EndTimer();
        printf("%d GAP DVCBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(dvcbsPath),
             dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());
             
        t1.StartTimer();
        dvcbs.GetPath(&pancake0, start, goal,&pancake1, &pancake1, dvcbsPath);
        t1.EndTimer();
        printf("%d GAP\\1 DVCBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(dvcbsPath),
             dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());
             
        t1.StartTimer();
        dvcbs.GetPath(&pancake0, start, goal,&pancake2, &pancake2, dvcbsPath);
        t1.EndTimer();
        printf("%d GAP\\2 DVCBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(dvcbsPath),
             dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());

        t1.StartTimer();
        dvcbs.GetPath(&pancake0, start, goal,&pancake3, &pancake3, dvcbsPath);
        t1.EndTimer();
        printf("%d GAP\\3 DVCBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(dvcbsPath),
             dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());

        t1.StartTimer();
        dvcbs.GetPath(&pancake0, start, goal,&o1, &o1, dvcbsPath);
        t1.EndTimer();
        printf("%d GAP-1 DVCBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(dvcbsPath),
             dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());

        t1.StartTimer();
        dvcbs.GetPath(&pancake0, start, goal,&o2, &o2, dvcbsPath);
        t1.EndTimer();
        printf("%d GAP-2 DVCBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(dvcbsPath),
             dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());      

        t1.StartTimer();
        dvcbs.GetPath(&pancake0, start, goal,&o3, &o3, dvcbsPath);
        t1.EndTimer();
        printf("%d GAP-3 DVCBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(dvcbsPath),
             dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());    

        t1.StartTimer();
        dvcbs.GetPath(&pancake0, start, goal,&w9, &w9, dvcbsPath);
        t1.EndTimer();
        printf("%d GAP0.9 DVCBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(dvcbsPath),
             dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());

        t1.StartTimer();
        dvcbs.GetPath(&pancake0, start, goal,&w8, &w8, dvcbsPath);
        t1.EndTimer();
        printf("%d GAP0.8 DVCBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(dvcbsPath),
             dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());      

        t1.StartTimer();
        dvcbs.GetPath(&pancake0, start, goal,&w7, &w7, dvcbsPath);
        t1.EndTimer();
        printf("%d GAP0.7 DVCBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(dvcbsPath),
             dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());
      }
      // DVCBS-L
      if (0)
      {
        DVCBS<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>,DVCBSQueue<PancakePuzzleState<CNT>,1,false>> dvcbs(false,false);
        t1.StartTimer();
        dvcbs.GetPath(&pancake0, start, goal,&pancake0, &pancake0, dvcbsPath);
        t1.EndTimer();
        printf("%d GAP DVCBS-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(dvcbsPath),
             dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());
             
        t1.StartTimer();
        dvcbs.GetPath(&pancake0, start, goal,&pancake1, &pancake1, dvcbsPath);
        t1.EndTimer();
        printf("%d GAP\\1 DVCBS-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(dvcbsPath),
             dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());
             
        t1.StartTimer();
        dvcbs.GetPath(&pancake0, start, goal,&pancake2, &pancake2, dvcbsPath);
        t1.EndTimer();
        printf("%d GAP\\2 DVCBS-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(dvcbsPath),
             dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());

        t1.StartTimer();
        dvcbs.GetPath(&pancake0, start, goal,&pancake3, &pancake3, dvcbsPath);
        t1.EndTimer();
        printf("%d GAP\\3 DVCBS-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(dvcbsPath),
             dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());

        t1.StartTimer();
        dvcbs.GetPath(&pancake0, start, goal,&o1, &o1, dvcbsPath);
        t1.EndTimer();
        printf("%d GAP-1 DVCBS-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(dvcbsPath),
             dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());

        t1.StartTimer();
        dvcbs.GetPath(&pancake0, start, goal,&o2, &o2, dvcbsPath);
        t1.EndTimer();
        printf("%d GAP-2 DVCBS-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(dvcbsPath),
             dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());      

        t1.StartTimer();
        dvcbs.GetPath(&pancake0, start, goal,&o3, &o3, dvcbsPath);
        t1.EndTimer();
        printf("%d GAP-3 DVCBS-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(dvcbsPath),
             dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());    

        t1.StartTimer();
        dvcbs.GetPath(&pancake0, start, goal,&w9, &w9, dvcbsPath);
        t1.EndTimer();
        printf("%d GAP0.9 DVCBS-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(dvcbsPath),
             dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());

        t1.StartTimer();
        dvcbs.GetPath(&pancake0, start, goal,&w8, &w8, dvcbsPath);
        t1.EndTimer();
        printf("%d GAP0.8 DVCBS-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(dvcbsPath),
             dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());      

        t1.StartTimer();
        dvcbs.GetPath(&pancake0, start, goal,&w7, &w7, dvcbsPath);
        t1.EndTimer();
        printf("%d GAP0.7 DVCBS-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake0.GetPathLength(dvcbsPath),
             dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());
      }
    }
    if(0){
      StateNeighborsUpToDistance(&pancake0, original, &pancake0, rad);
      StateNeighborsUpToDistance(&pancake0, original, &pancake1, rad);
      StateNeighborsUpToDistance(&pancake0, original, &pancake2, rad);
      StateNeighborsUpToDistance(&pancake0, original, &pancake3, rad);
      StateNeighborsUpToDistance(&pancake0, original, &o1, rad);
      StateNeighborsUpToDistance(&pancake0, original, &o2, rad);
      StateNeighborsUpToDistance(&pancake0, original, &o3, rad);
      StateNeighborsUpToDistance(&pancake0, original, &w9, rad);
      StateNeighborsUpToDistance(&pancake0, original, &w8, rad);
      StateNeighborsUpToDistance(&pancake0, original, &w7, rad);
      //StateNeighborsUpToDistance(&pancake0, original, &z, rad);
    }
    
    if(0){
      //BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>>::GetWeightedVertexGraph(start, goal, &pancake0, &pancake0, &pancake0,0,1);
      // BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>>::GetWeightedVertexGraph(start, goal, &pancake0, &pancake1, &pancake1,"tst.svg",1);
      // BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>>::GetWeightedVertexGraph(start, goal, &pancake0, &pancake2, &pancake2,0,1);
      // BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>>::GetWeightedVertexGraph(start, goal, &pancake0, &pancake3, &pancake3,0,1);
      // BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>>::GetWeightedVertexGraph(start, goal, &pancake0, &o1, &o1,0,1);
      // BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>>::GetWeightedVertexGraph(start, goal, &pancake0, &o2, &o2,0,1);
      // BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>>::GetWeightedVertexGraph(start, goal, &pancake0, &o3, &o3,0,1);
      //BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>>::GetWeightedVertexGraph(start, goal, &pancake0, &w9, &w9,0,1);
      // BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>>::GetWeightedVertexGraph(start, goal, &pancake0, &w8, &w8,0,1);
      // BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>>::GetWeightedVertexGraph(start, goal, &pancake0, &w7, &w7,0,1);
        BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>> p(start, goal, &pancake0, &w8, &w8);
        p.drawProblemInstance = false;
        p.drawStatistics = false;
        p.drawAllG = true;
        p.flipBackwardsGCost = true;
        p.SaveSVG((std::to_string(count)+"w8.svg").c_str());        
        BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>> p1(start, goal, &pancake0, &w9, &w9);
        p1.drawProblemInstance = false;
        p1.drawStatistics = false;
        p1.drawAllG = true;
        p1.flipBackwardsGCost = true;
        p1.SaveSVG((std::to_string(count)+"w9.svg").c_str());        
      
      //BidirectionalProblemAnalyzer<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>> p9(original, goal, &pancake0, &z, &z);
    }
  }
  
}

void TestPancakeDensityVariance(int samplesAmount,int sampleDepth)
{
  PancakePuzzle<CNT> pancake;

  OnlineStats stats;
  PancakePuzzleState<CNT> original;
  
  srandom(1923544);
  for (int count = 0;  count < samplesAmount; count++)
	{
    srandom(random());
    
    original.Reset();
    for (int x = 0; x < CNT; x++)
      std::swap(original.puzzle[x], original.puzzle[x+random()%(CNT-x)]);
    
    stats.Push(dijkstraUptoLimit(&pancake,original,sampleDepth));
    
  }
  printf("variance %1.2f", stats.Variance());
  
}