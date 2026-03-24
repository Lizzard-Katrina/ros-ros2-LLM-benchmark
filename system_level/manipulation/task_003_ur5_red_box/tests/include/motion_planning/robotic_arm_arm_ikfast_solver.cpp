#pragma once

#include <vector>

typedef double IkReal;

class IkSolutionList
{
public:
  std::vector<std::vector<IkReal>> solutions;

  size_t GetNumSolutions() const
  {
    return solutions.size();
  }
};
