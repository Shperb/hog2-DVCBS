#ifndef PANCAKEGAP_H
#define PANCAKEGAP_H

#include <stdint.h>
#include <iostream>
#include <sstream>
#include <algorithm>
#include "PancakePuzzle.h"

template <int N>
class PancakePuzzleGapVector : public PancakePuzzle<N> {
public:
	PancakePuzzleGapVector(std::vector<int> gap = std::vector<int> ());
	PancakePuzzleGapVector(const std::vector<unsigned> op_order); // used to set action order
  double DefaultH(const PancakePuzzleState<N> &state, const std::vector<int> &goal_locs) const;

	~PancakePuzzleGapVector();
private:
  std::vector<int> gapVec;
};

template <int N>
PancakePuzzleGapVector<N>::PancakePuzzleGapVector(std::vector<int> gap)
:PancakePuzzle<N>(),gapVec(gap)
{
}

template <int N>
PancakePuzzleGapVector<N>::PancakePuzzleGapVector(const std::vector<PancakePuzzleAction> op_order)
:PancakePuzzle<N>(),gapVec(std::vector<int>())
{
}

template <int N>
PancakePuzzleGapVector<N>::~PancakePuzzleGapVector()
{
}



template <int N>
double PancakePuzzleGapVector<N>::DefaultH(const PancakePuzzleState<N> &state, const std::vector<int> &goal_locs) const
{
//	if (state.size() != N)
//	{
//		fprintf(stderr, "ERROR: HCost called with state with wrong size.\n");
//		exit(1);
//	}
	
	double h_count = 0.0;
	unsigned i = 0;
	for (; i < N - 1; i++)
	{
		if ((std::find(gapVec.begin(),gapVec.end(),goal_locs[state.puzzle[i]]) != gapVec.end()) || (std::find(gapVec.begin(),gapVec.end(),goal_locs[state.puzzle[i+1]]) != gapVec.end()))
			continue;
		int diff = goal_locs[state.puzzle[i]] - goal_locs[state.puzzle[i+1]];
		if (diff > 1 || diff < -1)
			h_count++;
	}
	if ((unsigned) goal_locs[state.puzzle[i]]!= N -1)
		h_count++;
	
	return h_count;
}


#endif
