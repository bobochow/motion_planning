#ifndef _LAZYTSTAR_SEARCHER_H_
#define _LAZYTSTAR_SEARCHER_H_

# include "Astar_searcher.h"
#include <iostream>


class LazyTstarPathFinder : public AstarPathFinder
{
private:
    /* data */
    bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
	bool isOccupied(const Eigen::Vector3i & index) const;
	bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;
	bool isFree(const Eigen::Vector3i & index) const;
    
public:

    void LThetastarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);
    bool LineOfSight(Eigen::Vector3i start, Eigen::Vector3i end);
    void ValidateParent(GridNodePtr currentPtr , GridNodePtr goal);
    void LThetastarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);


};




#endif