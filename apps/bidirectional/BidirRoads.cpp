
#include "BidirRoads.h"
#include <unordered_map>

  RoadMap *ge;
  std::string roadType;
  //void TestRoadsRandom();
  void SetupRoad(std::string graphFile, std::string coordinatesFile);
  void TestRoadsLowHeuristic(int rad);
  void TestRoadsDensityVariance(int samplesAmount,int sampleDepth);
  //void TestRoadseuristic();
  //void TestError();

void TestRoads(std::string graphFile, std::string coordinatesFile)
{
  SetupRoad(graphFile,coordinatesFile);
	//TestRoadsRandom();
  //TestRoadsHeuristic();
  //TestRoadsLowHeuristic(35000);
	//TestError();
  TestRoadsDensityVariance(1000,100);

//	TestVariants();
	exit(0);
}

void SetupRoad(std::string graphFile, std::string coordinatesFile)
{
  bool timeGraph = true;
  roadType = "t";
	if (strstr(graphFile.c_str(), "road-t") == NULL){
    timeGraph = false;
    roadType = "d";
  }
		
  
  ge = new RoadMap(graphFile.c_str(), coordinatesFile.c_str(), timeGraph);
}


void TestRoadsLowHeuristic(int rad)
{
  float f;
  std::vector<graphState> thePath;
  ZeroHeuristic<graphState> z;
  TemplateAStar<graphState, graphMove, RoadMap> astar;
  NBS<graphState, graphMove,RoadMap, NBSQueue<graphState, 1, true>> nbs(true);
 
  Timer t1;
  
  srandom(2017218);
  for (int x = 0; x < 100; x++)
  {
    node *start = ge->GetGraph()->GetRandomNode();
    node *goal = ge->GetGraph()->GetRandomNode();
    if(0){ //run algorithms
        if(1){
          t1.StartTimer();
          astar.SetHeuristic(ge);
          astar.GetPath(ge, start->GetNum(), goal->GetNum(), thePath);
          t1.EndTimer();
          printf("%d GE A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", x, ge->GetPathLength(thePath),
               astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());
               
          t1.StartTimer();
          astar.SetHeuristic(&z);
          astar.GetPath(ge, start->GetNum(), goal->GetNum(), thePath);
          t1.EndTimer();
          printf("%d Zero A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", x, ge->GetPathLength(thePath),
               astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());
        }
        if(1){
          t1.StartTimer();
          astar.SetHeuristic(ge);
          astar.GetPath(ge, goal->GetNum(),start->GetNum(), thePath);
          t1.EndTimer();
          printf("%d GE r-A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", x, ge->GetPathLength(thePath),
               astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());
               
          t1.StartTimer();
          astar.SetHeuristic(&z);
          astar.GetPath(ge, goal->GetNum(),start->GetNum(), thePath);
          t1.EndTimer();
          printf("%d Zero r-A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", x, ge->GetPathLength(thePath),
               astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());
        }
        if(1){
          t1.StartTimer();
          nbs.GetPath(ge, start->GetNum(), goal->GetNum(),ge,ge, thePath);
          t1.EndTimer();
          printf("%d GE NBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", x, ge->GetPathLength(thePath),
               nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t1.GetElapsedTime());
               
          t1.StartTimer();
          nbs.GetPath(ge, start->GetNum(), goal->GetNum(),&z,&z, thePath);
          t1.EndTimer();
          printf("%d Zero NBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", x, ge->GetPathLength(thePath),
               nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t1.GetElapsedTime());
        }
        if(1){
          DVCBS<graphState, graphMove, RoadMap,DVCBSQueue<graphState,1,true>> dvcbs(false,true);
          t1.StartTimer();
          dvcbs.GetPath(ge, start->GetNum(), goal->GetNum(),ge,ge, thePath);
          t1.EndTimer();
          printf("%d GE DVCBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", x, ge->GetPathLength(thePath),
               dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());
               
          t1.StartTimer();
          dvcbs.GetPath(ge, start->GetNum(), goal->GetNum(),&z,&z, thePath);
          t1.EndTimer();
          printf("%d Zero DVCBS found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", x, ge->GetPathLength(thePath),
               dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());
        }      
        if(1){
          DVCBS<graphState, graphMove, RoadMap,DVCBSQueue<graphState,1,false>> dvcbs(false,false);
          t1.StartTimer();
          dvcbs.GetPath(ge, start->GetNum(), goal->GetNum(),ge,ge, thePath);
          t1.EndTimer();
          printf("%d GE DVCBS-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", x, ge->GetPathLength(thePath),
               dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());
               
          t1.StartTimer();
          dvcbs.GetPath(ge, start->GetNum(), goal->GetNum(),&z,&z, thePath);
          t1.EndTimer();
          printf("%d Zero DVCBS-L found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", x, ge->GetPathLength(thePath),
               dvcbs.GetNodesExpanded(), dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());
        }         
    }
    
    if(0){ //Analyze VC
      BidirectionalProblemAnalyzer<graphState, graphMove, RoadMap>::GetWeightedVertexGraph(start->GetNum(), goal->GetNum(), ge, ge, ge);
      BidirectionalProblemAnalyzer<graphState, graphMove, RoadMap>::GetWeightedVertexGraph(start->GetNum(), goal->GetNum(), ge, &z, &z);
    }
    // f = MeasureHeuristicErrors(ge, (graphState)goal->GetNum(), ge, 5, 4, [](float i){return i <1;});
    // printf("Roads-%s Forward Error percentage (5,4): %1.1f\n", roadType.c_str(),f*100);
    // f = MeasureHeuristicErrors(ge, (graphState)start->GetNum(), ge, 5, 4, [](float i){return i <1;});
    // printf("Roads-%s Backward Error percentage (5,4): %1.1f\n", roadType.c_str(),f*100);
    
    if(1){ //measure distance from goal
      printf("goal ");
      StateNeighborsUpToDistance(ge, (graphState)goal->GetNum(),ge,rad);
      printf("goal ");
      StateNeighborsUpToDistance(ge, (graphState)goal->GetNum(),&z,rad);
      printf("start ");
      StateNeighborsUpToDistance(ge, (graphState)start->GetNum(),ge,rad);  
      printf("start ");
      StateNeighborsUpToDistance(ge, (graphState)start->GetNum(),&z,rad);
    }

  }
  if (0){ //sample lowh
    int hcount,ihcount;
    hcount = ihcount = 0;

    
    TemplateAStar<graphState, graphMove, RoadMap> astar;
    std::vector<graphState> astarPath;

    int limit = 1000000;
    srandom(3201561);
    
    for (int count = 0; count < limit; count++)
    {
      srandom(random());
      node *start = ge->GetGraph()->GetRandomNode();
      node *goal = ge->GetGraph()->GetRandomNode();

      if (ge->HCost(start->GetNum(),goal->GetNum()) <= rad){
        hcount++;
        astar.GetPath(ge, start->GetNum(), goal->GetNum(), astarPath);
        if (ge->GetPathLength(astarPath) != ge->HCost(start->GetNum(),goal->GetNum())){
          ihcount++;
        }
      }
    }
    printf("%d %d\n",hcount,ihcount);
  }
}

void TestRoadsDensityVariance(int samplesAmount,int sampleDepth)
{
  OnlineStats stats;
  std::unordered_map<double,int> countStats;
  srandom(1923544);
  for (int count = 0;  count < samplesAmount; count++)
	{
    srandom(random());
    node *state = ge->GetGraph()->GetRandomNode();
    //stats.Push(dijkstraUptoLimit(ge,(graphState)state->GetNum(),sampleDepth));
    //printf("variance %1.2f\n", stats.Variance());
      
    double gmax = dijkstraUptoLimit(ge,(graphState)state->GetNum(),sampleDepth);
    stats.Push(gmax);
    auto it = countStats.find(gmax);
    if (it == countStats.end()){
      countStats.insert(std::make_pair(gmax,1));
    }
    else{
      countStats[gmax]++;
    }
      
    
    
  }
  printf("variance %1.2f", stats.Variance());
  int currentMax = 0;
  int totalNum = 0;
  for(auto it = countStats.begin(); it != countStats.end(); ++it ){
      if (it ->second > currentMax) {
          currentMax = it->second;
      }
      totalNum+= it->second;;
  }
  printf("variance-2 %d %d %1.2f\n", currentMax, totalNum, currentMax/totalNum);
  
}