//
//  BidirPancake.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/7/17.
//  Copyright Â© 2017 University of Denver. All rights reserved.
//

#include "BidirPancake.h"
#include "PancakePuzzle.h"
#include "TemplateAStar.h"
#include "NBS.h"
#include "IDAStar.h"
#include "MM.h"
#include "BSStar.h"
#include "CBBS.h"
#include "PancakeInstances.h"
#include "CalculateWVC.h"
#include "fMM.h"
//#include "WeightedVertexGraph.h"

const int S = 10; // must be factor of sizes below

void TestPancakeTR();
void TestPancakeRandom();
void TestPancakeHard();
void TestRob();
void TestPancake()
{
//	TestRob();
	TestPancakeRandom();
//	TestPancakeHard();
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
		CBBS<PancakePuzzleState<S>, PancakePuzzleAction, PancakePuzzle<S>> cbbs(1);
		MM<PancakePuzzleState<S>, PancakePuzzleAction, PancakePuzzle<S>> mm;
		TemplateAStar<PancakePuzzleState<S>, PancakePuzzleAction, PancakePuzzle<S>> astar;
		IDAStar<PancakePuzzleState<S>, PancakePuzzleAction, false> idastar;
		
		std::vector<PancakePuzzleState<S>> nbsPath;
		std::vector<PancakePuzzleState<S>> cbbsPath;
		std::vector<PancakePuzzleState<S>> astarPath;
		std::vector<PancakePuzzleState<S>> mmPath;
		std::vector<PancakePuzzleAction> idaPath;
		Timer t1, t2, t3, t4,t5;
		
		
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
		t5.StartTimer();
		cbbs.GetPath(&pancake, start, goal, &pancake, &pancake2, cbbsPath);
		t5.EndTimer();
		printf("CBBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", pancake.GetPathLength(cbbsPath),
			   cbbs.GetNodesExpanded(), cbbs.GetNecessaryExpansions(), t5.GetElapsedTime());
		
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
		
		printf("Problem & IDA* & & A* & & MM & & NBS* & CBBS & \\\\\n");
		printf("%d G-%d & %llu & %1.2fs & %llu & %1.2fs & %llu & %1.2fs & %llu & %1.2fs & %llu & %1.2fs \\\\ \n", S, gap,
			   idastar.GetNodesExpanded(), t3.GetElapsedTime(),
			   astar.GetNodesExpanded(), t1.GetElapsedTime(),
			   mm.GetNodesExpanded(), t4.GetElapsedTime(),
			   nbs.GetNodesExpanded(), t2.GetElapsedTime(),cbbs.GetNodesExpanded(), t5.GetElapsedTime());
	}
	exit(0);
}

const int N = 14;
void TestPancakeRandom()
{
	for (int gap = 0; gap < 4; gap++)
	{
	srandom(2017218);
		PancakePuzzleState<N> start;
		PancakePuzzleState<N> original;
		PancakePuzzleState<N> goal;
		PancakePuzzle<N> pancake(gap);
		PancakePuzzle<N> pancake2(gap);
		
		
		std::vector<PancakePuzzleState<N>> nbsPath;
		std::vector<PancakePuzzleState<N>> cbbsPath;
		std::vector<PancakePuzzleState<N>> nbsEpsilonPath;
		std::vector<PancakePuzzleState<N>> cbbsEpsilonPath;
		std::vector<PancakePuzzleState<N>> bsPath;
		std::vector<PancakePuzzleState<N>> astarPath;
		std::vector<PancakePuzzleState<N>> rastarPath;
		std::vector<PancakePuzzleState<N>> mmPath;
		std::vector<PancakePuzzleState<N>> fmmPath;
		std::vector<PancakePuzzleAction> idaPath;
		Timer t1, t2, t3, t4, t5, t6,t7;
		
				// variables for WVC calaculator (optimal p for FMM)
		std::vector<AStarOpenClosedData<PancakePuzzleState<N>>> astarOpenClose;
		std::vector<AStarOpenClosedData<PancakePuzzleState<N>>> rastarOpenClose;
		double p;
		double pEpsilon;
		
		for (int count = 0; count < 50; count++)
		{
			srandom(random());
			
			goal.Reset();
			original.Reset();
			for (int x = 0; x < N; x++)
				std::swap(original.puzzle[x], original.puzzle[x+random()%(N-x)]);
			
			
			if (false/*gap !=3 || count+1 != 5*/){
				continue;
			}
			
			printf("Problem %d of %d\n", count+1, 50);
			std::cout << original << "\n";
						
			// A*
			
			/*
			
			if(gap != 3)
			{
				TemplateAStar<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> astar;
				start = original;
				t1.StartTimer();
				astar.GetPath(&pancake, start, goal, astarPath);
				t1.EndTimer();
				printf("GAP-%d A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap, pancake.GetPathLength(astarPath),
					   astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());
				astarOpenClose = astar.openClosedList.elements;
			}
			
			// R-A*
			
			if(gap != 3)
			{
				TemplateAStar<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> rastar;
				goal.Reset();
				start = original;
				t1.StartTimer();
				rastar.GetPath(&pancake, goal, start, rastarPath);
				t1.EndTimer();
				printf("GAP-%d R-A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap, pancake.GetPathLength(rastarPath),
					   rastar.GetNodesExpanded(), rastar.GetNecessaryExpansions(), t1.GetElapsedTime());   
				rastarOpenClose = rastar.openClosedList.elements;
			}
			*/
			//Single Solution
			if(1)
			{
			// NBS
			
			
				{
					NBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, NBSQueue<PancakePuzzleState<N>,0,false>> nbs(false);
					goal.Reset();
					start = original;
					t2.StartTimer();
					nbs.GetPath(&pancake, start, goal, &pancake, &pancake2, nbsPath);
					t2.EndTimer();
					printf("GAP-%d NBS-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n", gap, pancake.GetPathLength(nbsPath),
						   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t2.GetElapsedTime(),nbs.getForwardMeetingPoint(),nbs.getBackwardMeetingPoint(),nbs.getForwardUnnecessaryNodesInPath(),nbs.getBackwardUnnecessaryNodesInPath(),nbs.GetExpansionUntilFirstSolution());
					
					NBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, NBSQueue<PancakePuzzleState<N>,0,true>> nbsLEQ(false,true);
					goal.Reset();
					start = original;
					t2.StartTimer();
					nbsLEQ.GetPath(&pancake, start, goal, &pancake, &pancake2, nbsPath);
					t2.EndTimer();
					printf("GAP-%d NBS-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n", gap, pancake.GetPathLength(nbsPath),
						   nbsLEQ.GetNodesExpanded(), nbsLEQ.GetNecessaryExpansions(), t2.GetElapsedTime(),nbsLEQ.getForwardMeetingPoint(),nbsLEQ.getBackwardMeetingPoint(),nbsLEQ.getForwardUnnecessaryNodesInPath(),nbsLEQ.getBackwardUnnecessaryNodesInPath(),nbsLEQ.GetExpansionUntilFirstSolution());
				}
				{
					NBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>,NBSQueue<PancakePuzzleState<N>,1,false>> nbsEpsilon(false);
					goal.Reset();
					start = original;
					t2.StartTimer();
					nbsEpsilon.GetPath(&pancake, start, goal, &pancake, &pancake2, nbsEpsilonPath);
					t2.EndTimer();
					printf("GAP-%d NBS-E-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n", gap, pancake.GetPathLength(nbsEpsilonPath),
						   nbsEpsilon.GetNodesExpanded(), nbsEpsilon.GetNecessaryExpansions(), t2.GetElapsedTime(),nbsEpsilon.getForwardMeetingPoint(),nbsEpsilon.getBackwardMeetingPoint(),nbsEpsilon.getForwardUnnecessaryNodesInPath(),nbsEpsilon.getBackwardUnnecessaryNodesInPath(),nbsEpsilon.GetExpansionUntilFirstSolution());
						   
					NBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>,NBSQueue<PancakePuzzleState<N>,1,true>> nbsEpsilonLeq(false,true);
					goal.Reset();
					start = original;
					t2.StartTimer();
					nbsEpsilonLeq.GetPath(&pancake, start, goal, &pancake, &pancake2, nbsEpsilonPath);
					t2.EndTimer();
					printf("GAP-%d NBS-E-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n", gap, pancake.GetPathLength(nbsEpsilonPath),
						   nbsEpsilonLeq.GetNodesExpanded(), nbsEpsilonLeq.GetNecessaryExpansions(), t2.GetElapsedTime(),nbsEpsilonLeq.getForwardMeetingPoint(),nbsEpsilonLeq.getBackwardMeetingPoint(),nbsEpsilonLeq.getForwardUnnecessaryNodesInPath(),nbsEpsilonLeq.getBackwardUnnecessaryNodesInPath(),nbsEpsilonLeq.GetExpansionUntilFirstSolution());
				}
				
				
							// CBBS
				{
					for (int i = 1; i<=15;i++){
						if (i !=4){
							continue;
						}
						CBBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>,CBBSQueue<PancakePuzzleState<N>,0,false>> cbbs(i,false);
						goal.Reset();
						start = original;
						t6.StartTimer();
						cbbs.GetPath(&pancake, start, goal, &pancake, &pancake2, cbbsPath);
						t6.EndTimer();
						printf("GAP-%d CBBS-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n", gap, pancake.GetPathLength(cbbsPath),
							   cbbs.GetNodesExpanded(), cbbs.GetNecessaryExpansions(), t6.GetElapsedTime(),cbbs.getForwardMeetingPoint(),cbbs.getBackwardMeetingPoint(),cbbs.getForwardUnnecessaryNodesInPath(),cbbs.getBackwardUnnecessaryNodesInPath(),cbbs.GetExpansionUntilFirstSolution());
					}
					
					for (int i = 1; i<=15;i++){
						if (i !=4){
							continue;
						}
						CBBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>,CBBSQueue<PancakePuzzleState<N>,0,true>> cbbs(i,false,true);
						goal.Reset();
						start = original;
						t6.StartTimer();
						cbbs.GetPath(&pancake, start, goal, &pancake, &pancake2, cbbsPath);
						t6.EndTimer();
						printf("GAP-%d CBBS-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n", gap, pancake.GetPathLength(cbbsPath),
							   cbbs.GetNodesExpanded(), cbbs.GetNecessaryExpansions(), t6.GetElapsedTime(),cbbs.getForwardMeetingPoint(),cbbs.getBackwardMeetingPoint(),cbbs.getForwardUnnecessaryNodesInPath(),cbbs.getBackwardUnnecessaryNodesInPath(),cbbs.GetExpansionUntilFirstSolution());
					}
				}
										// CBBS
				{
					for (int i = 1; i<=15;i++){
						if (i !=4){
							continue;
						}
						CBBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>,CBBSQueue<PancakePuzzleState<N>,1,false>> cbbsEpsilon(i,false);
						goal.Reset();
						start = original;
						t6.StartTimer();
						cbbsEpsilon.GetPath(&pancake, start, goal, &pancake, &pancake2, cbbsEpsilonPath);
						t6.EndTimer();
						printf("GAP-%d CBBS-E-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n", gap, pancake.GetPathLength(cbbsEpsilonPath),
							   cbbsEpsilon.GetNodesExpanded(), cbbsEpsilon.GetNecessaryExpansions(), t6.GetElapsedTime(),cbbsEpsilon.getForwardMeetingPoint(),cbbsEpsilon.getBackwardMeetingPoint(),cbbsEpsilon.getForwardUnnecessaryNodesInPath(),cbbsEpsilon.getBackwardUnnecessaryNodesInPath(),cbbsEpsilon.GetExpansionUntilFirstSolution());
					}
					for (int i = 1; i<=15;i++){
						if (i !=4){
							continue;
						}
						CBBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>,CBBSQueue<PancakePuzzleState<N>,1,true>> cbbsEpsilon(i,false,true);
						goal.Reset();
						start = original;
						t6.StartTimer();
						cbbsEpsilon.GetPath(&pancake, start, goal, &pancake, &pancake2, cbbsEpsilonPath);
						t6.EndTimer();
						printf("GAP-%d CBBS-E-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n", gap, pancake.GetPathLength(cbbsEpsilonPath),
							   cbbsEpsilon.GetNodesExpanded(), cbbsEpsilon.GetNecessaryExpansions(), t6.GetElapsedTime(),cbbsEpsilon.getForwardMeetingPoint(),cbbsEpsilon.getBackwardMeetingPoint(),cbbsEpsilon.getForwardUnnecessaryNodesInPath(),cbbsEpsilon.getBackwardUnnecessaryNodesInPath(),cbbsEpsilon.GetExpansionUntilFirstSolution());
					}

				}
			}
			
			//ALL Solution
			if(1)
			{
			// NBS
			
			
				{
					NBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, NBSQueue<PancakePuzzleState<N>,0,false>> nbs(true);
					goal.Reset();
					start = original;
					t2.StartTimer();
					nbs.GetPath(&pancake, start, goal, &pancake, &pancake2, nbsPath);
					t2.EndTimer();
					printf("GAP-%d NBS-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n", gap, pancake.GetPathLength(nbsPath),
						   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t2.GetElapsedTime(),nbs.getForwardMeetingPoint(),nbs.getBackwardMeetingPoint(),nbs.getForwardUnnecessaryNodesInPath(),nbs.getBackwardUnnecessaryNodesInPath(),nbs.GetExpansionUntilFirstSolution());
					
					NBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, NBSQueue<PancakePuzzleState<N>,0,true>> nbsLEQ(true,true);
					goal.Reset();
					start = original;
					t2.StartTimer();
					nbsLEQ.GetPath(&pancake, start, goal, &pancake, &pancake2, nbsPath);
					t2.EndTimer();
					printf("GAP-%d NBS-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n", gap, pancake.GetPathLength(nbsPath),
						   nbsLEQ.GetNodesExpanded(), nbsLEQ.GetNecessaryExpansions(), t2.GetElapsedTime(),nbsLEQ.getForwardMeetingPoint(),nbsLEQ.getBackwardMeetingPoint(),nbsLEQ.getForwardUnnecessaryNodesInPath(),nbsLEQ.getBackwardUnnecessaryNodesInPath(),nbsLEQ.GetExpansionUntilFirstSolution());
				}
				{
					NBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>,NBSQueue<PancakePuzzleState<N>,1,false>> nbsEpsilon(true);
					goal.Reset();
					start = original;
					t2.StartTimer();
					nbsEpsilon.GetPath(&pancake, start, goal, &pancake, &pancake2, nbsEpsilonPath);
					t2.EndTimer();
					printf("GAP-%d NBS-E-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n", gap, pancake.GetPathLength(nbsEpsilonPath),
						   nbsEpsilon.GetNodesExpanded(), nbsEpsilon.GetNecessaryExpansions(), t2.GetElapsedTime(),nbsEpsilon.getForwardMeetingPoint(),nbsEpsilon.getBackwardMeetingPoint(),nbsEpsilon.getForwardUnnecessaryNodesInPath(),nbsEpsilon.getBackwardUnnecessaryNodesInPath(),nbsEpsilon.GetExpansionUntilFirstSolution());
						   
					NBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>,NBSQueue<PancakePuzzleState<N>,1,true>> nbsEpsilonLeq(true,true);
					goal.Reset();
					start = original;
					t2.StartTimer();
					nbsEpsilonLeq.GetPath(&pancake, start, goal, &pancake, &pancake2, nbsEpsilonPath);
					t2.EndTimer();
					printf("GAP-%d NBS-E-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n", gap, pancake.GetPathLength(nbsEpsilonPath),
						   nbsEpsilonLeq.GetNodesExpanded(), nbsEpsilonLeq.GetNecessaryExpansions(), t2.GetElapsedTime(),nbsEpsilonLeq.getForwardMeetingPoint(),nbsEpsilonLeq.getBackwardMeetingPoint(),nbsEpsilonLeq.getForwardUnnecessaryNodesInPath(),nbsEpsilonLeq.getBackwardUnnecessaryNodesInPath(),nbsEpsilonLeq.GetExpansionUntilFirstSolution());
				}
				
				
							// CBBS
				{
					for (int i = 1; i<=15;i++){
						if (i !=4){
							continue;
						}
						CBBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>,CBBSQueue<PancakePuzzleState<N>,0,false>> cbbs(i,true);
						goal.Reset();
						start = original;
						t6.StartTimer();
						cbbs.GetPath(&pancake, start, goal, &pancake, &pancake2, cbbsPath);
						t6.EndTimer();
						printf("GAP-%d CBBS-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n", gap, pancake.GetPathLength(cbbsPath),
							   cbbs.GetNodesExpanded(), cbbs.GetNecessaryExpansions(), t6.GetElapsedTime(),cbbs.getForwardMeetingPoint(),cbbs.getBackwardMeetingPoint(),cbbs.getForwardUnnecessaryNodesInPath(),cbbs.getBackwardUnnecessaryNodesInPath(),cbbs.GetExpansionUntilFirstSolution());
					}
					
					for (int i = 1; i<=15;i++){
						if (i !=4){
							continue;
						}
						CBBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>,CBBSQueue<PancakePuzzleState<N>,0,true>> cbbs(i,true,true);
						goal.Reset();
						start = original;
						t6.StartTimer();
						cbbs.GetPath(&pancake, start, goal, &pancake, &pancake2, cbbsPath);
						t6.EndTimer();
						printf("GAP-%d CBBS-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n", gap, pancake.GetPathLength(cbbsPath),
							   cbbs.GetNodesExpanded(), cbbs.GetNecessaryExpansions(), t6.GetElapsedTime(),cbbs.getForwardMeetingPoint(),cbbs.getBackwardMeetingPoint(),cbbs.getForwardUnnecessaryNodesInPath(),cbbs.getBackwardUnnecessaryNodesInPath(),cbbs.GetExpansionUntilFirstSolution());
					}
				}
										// CBBS
				{
					for (int i = 1; i<=15;i++){
						if (i !=4){
							continue;
						}
						CBBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>,CBBSQueue<PancakePuzzleState<N>,1,false>> cbbsEpsilon(i,true);
						goal.Reset();
						start = original;
						t6.StartTimer();
						cbbsEpsilon.GetPath(&pancake, start, goal, &pancake, &pancake2, cbbsEpsilonPath);
						t6.EndTimer();
						printf("GAP-%d CBBS-E-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n", gap, pancake.GetPathLength(cbbsEpsilonPath),
							   cbbsEpsilon.GetNodesExpanded(), cbbsEpsilon.GetNecessaryExpansions(), t6.GetElapsedTime(),cbbsEpsilon.getForwardMeetingPoint(),cbbsEpsilon.getBackwardMeetingPoint(),cbbsEpsilon.getForwardUnnecessaryNodesInPath(),cbbsEpsilon.getBackwardUnnecessaryNodesInPath(),cbbsEpsilon.GetExpansionUntilFirstSolution());
					}
					for (int i = 1; i<=15;i++){
						if (i !=4){
							continue;
						}
						CBBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>,CBBSQueue<PancakePuzzleState<N>,1,true>> cbbsEpsilon(i,true,true);
						goal.Reset();
						start = original;
						t6.StartTimer();
						cbbsEpsilon.GetPath(&pancake, start, goal, &pancake, &pancake2, cbbsEpsilonPath);
						t6.EndTimer();
						printf("GAP-%d CBBS-E-LEQ found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed %llu forwardMeeting %llu backwardMeeting %llu forwardDistance %llu backwardDistance %f ExpansionUntilSolution\n", gap, pancake.GetPathLength(cbbsEpsilonPath),
							   cbbsEpsilon.GetNodesExpanded(), cbbsEpsilon.GetNecessaryExpansions(), t6.GetElapsedTime(),cbbsEpsilon.getForwardMeetingPoint(),cbbsEpsilon.getBackwardMeetingPoint(),cbbsEpsilon.getForwardUnnecessaryNodesInPath(),cbbsEpsilon.getBackwardUnnecessaryNodesInPath(),cbbsEpsilon.GetExpansionUntilFirstSolution());
					}

				}
			}
			
			/*
			// BS*
			
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
			*/
			// IDA*
			/*
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
			*/
			
			/*
			if(gap != 3)
			{
				int C = pancake.GetPathLength(astarPath);
				
				CalculateWVC<PancakePuzzleState<N>> calculateWVCEpsilon;
				double minWVCEpsilon = calculateWVCEpsilon.CalcWVC(astarOpenClose, rastarOpenClose, C, 1);
				pEpsilon = calculateWVCEpsilon.getOptimalP();
				
				CalculateWVC<PancakePuzzleState<N>> calculateWVC;
				double minWVC = calculateWVC.CalcWVC(astarOpenClose, rastarOpenClose, C, 0);
				p = calculateWVC.getOptimalP();
				//printf("minWVC is: %f\n", minWVC);
				//printf("Optimal P is: %f\n", p);
			
				fMM<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, AStarOpenClosed<PancakePuzzleState<N>, fMMCompare<PancakePuzzleState<N>,1>, FMMOpenClosedData<PancakePuzzleState<N>>>> fmm(p,0);
				//fmm.SetFraction(p);
				goal.Reset();
				start = original;
				t7.StartTimer();
				fmm.GetPath(&pancake, start, goal, &pancake, &pancake2, fmmPath);
				t7.EndTimer();
				printf("GAP-%d fMM found path length %1.0f; %llu expanded; %f necessary; %1.2fs elapsed\n", gap, pancake.GetPathLength(fmmPath),
					   fmm.GetNodesExpanded(), minWVC, t7.GetElapsedTime());
			
			
				fMM<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, AStarOpenClosed<PancakePuzzleState<N>, fMMCompare<PancakePuzzleState<N>,1>, FMMOpenClosedData<PancakePuzzleState<N>>>> fmmEpsilon(pEpsilon,1);
				//fmm.SetFraction(p);
				goal.Reset();
				start = original;
				t7.StartTimer();
				fmmEpsilon.GetPath(&pancake, start, goal, &pancake, &pancake2, fmmPath);
				t7.EndTimer();
				printf("GAP-%d fMM-E found path length %1.0f; %llu expanded; %f necessary; %1.2fs elapsed\n", gap, pancake.GetPathLength(fmmPath),
					   fmmEpsilon.GetNodesExpanded(), minWVCEpsilon, t7.GetElapsedTime());
					   
				
			}
			
			// fMM
			/*
			{
				fMM<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> fmm;
				fmm.SetFraction(0.5);
				goal.Reset();
				start = original;
				t4.StartTimer();
				fmm.GetPath(&pancake, start, goal, &pancake, &pancake2, mmPath);
				t4.EndTimer();
				printf("GAP-%d fMM found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap, pancake.GetPathLength(fmmPath),
					   fmm.GetNodesExpanded(), fmm.GetNecessaryExpansions(), t4.GetElapsedTime());
			}
	
			// fMM
			
			{
				fMM<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> fmm;
				fmm.SetFraction(0);
				goal.Reset();
				start = original;
				t4.StartTimer();
				fmm.GetPath(&pancake, start, goal, &pancake, &pancake2, mmPath);
				t4.EndTimer();
				printf("GAP-%d fMM found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap, pancake.GetPathLength(fmmPath),
					   fmm.GetNodesExpanded(), fmm.GetNecessaryExpansions(), t4.GetElapsedTime());
			}
			
			// fMM
			
			{
				fMM<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> fmm;
				fmm.SetFraction(1);
				goal.Reset();
				start = original;
				t4.StartTimer();
				fmm.GetPath(&pancake, start, goal, &pancake, &pancake2, mmPath);
				t4.EndTimer();
				printf("GAP-%d fMM found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap, pancake.GetPathLength(fmmPath),
					   fmm.GetNodesExpanded(), fmm.GetNecessaryExpansions(), t4.GetElapsedTime());
			}
			*/
			// MM
			/*
			{
				MM<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> mm;
				goal.Reset();
				start = original;
				t4.StartTimer();
				mm.GetPath(&pancake, start, goal, &pancake, &pancake2, mmPath);
				t4.EndTimer();
				printf("GAP-%d MM found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", gap, pancake.GetPathLength(mmPath),
					   mm.GetNodesExpanded(), mm.GetNecessaryExpansions(), t4.GetElapsedTime());
			}
			*/
			// MM0
			/*
			if (gap == 3)
			{
				MM<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> mm;
				ZeroHeuristic<PancakePuzzleState<N>> z;
				goal.Reset();
				start = original;
				t4.StartTimer();
				mm.GetPath(&pancake, start, goal, &z, &z, mmPath);
				t4.EndTimer();
				printf("GAP-%d MM0 found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake.GetPathLength(mmPath),
					   mm.GetNodesExpanded(), mm.GetNecessaryExpansions(), t4.GetElapsedTime());
			}
			*/
		}
	}
}

const int CNT = 28;
void TestPancakeHard()
{
	srandom(2017218);
	PancakePuzzleState<CNT> start;
	PancakePuzzleState<CNT> original;
	PancakePuzzleState<CNT> goal;
	PancakePuzzle<CNT> pancake;
	PancakePuzzle<CNT> pancake2;
	
	
	std::vector<PancakePuzzleState<CNT>> nbsPath;
	std::vector<PancakePuzzleState<CNT>> cbbsPath;
	std::vector<PancakePuzzleState<CNT>> bsPath;
	std::vector<PancakePuzzleState<CNT>> astarPath;
	std::vector<PancakePuzzleState<CNT>> mmPath;
	std::vector<PancakePuzzleAction> idaPath;
	Timer t1, t2, t3, t4, t5;
	
	for (int count = 2; count < 100; count++)
	{
		goal.Reset();
		original.Reset();
		GetPancakeInstance(original, count);
		
		printf("Problem %d of %d\n", count+1, 100);
		std::cout << original << "; Initial heuristic " << pancake.HCost(original, goal) << "\n";
		
		// A*
		if (0)
		{
			TemplateAStar<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>> astar;
			start = original;
			t1.StartTimer();
			astar.GetPath(&pancake, start, goal, astarPath);
			t1.EndTimer();
			printf("HARD-%d A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake.GetPathLength(astarPath),
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
			goal.Reset();
			start = original;
			t1.StartTimer();
			astar.GetPath(&pancake, goal, start, astarPath);
			t1.EndTimer();
			printf("HARD-%d ReverseA* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", count, pancake.GetPathLength(astarPath),
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
		/*
		if (0)
		{
			std::string t = "/Users/nathanst/pancake_";
			t += std::to_string(count);
			t += ".svg";
			
			start = original;
			GetWeightedVertexGraph<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>>(start, goal, &pancake, &pancake, &pancake);
		}
		*/
		// CBBS
		if (1)
		{
			for (int i = 1; i<=15;i++){
				if (i != 4){
					continue;
				}
				CBBS<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>> cbbs(i);
				goal.Reset();
				start = original;
				t2.StartTimer();
				cbbs.GetPath(&pancake, start, goal, &pancake, &pancake2, cbbsPath);
				t2.EndTimer();
				printf("HARD-%d CBBS %d found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed; \n", count, i, pancake.GetPathLength(cbbsPath),
					cbbs.GetNodesExpanded(), cbbs.GetNecessaryExpansions(), t2.GetElapsedTime());
			}
			
		}
		// NBS
		if (1)
		{
			NBS<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>> nbs;
			goal.Reset();
			start = original;
			t2.StartTimer();
			nbs.GetPath(&pancake, start, goal, &pancake, &pancake2, nbsPath);
			t2.EndTimer();
			printf("HARD-%d NBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed; \n", count, pancake.GetPathLength(nbsPath),
				   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t2.GetElapsedTime());
		}

		// NBS0
		if (0)
		{
			ZeroHeuristic<PancakePuzzleState<CNT>> z;
			NBS<PancakePuzzleState<CNT>, PancakePuzzleAction, PancakePuzzle<CNT>> nbs;
			goal.Reset();
			start = original;
			t2.StartTimer();
			nbs.GetPath(&pancake, start, goal, &z, &z, nbsPath);
			t2.EndTimer();
			printf("HARD-%d NBS0 found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed;\n", count, pancake.GetPathLength(nbsPath),
				   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t2.GetElapsedTime());
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

