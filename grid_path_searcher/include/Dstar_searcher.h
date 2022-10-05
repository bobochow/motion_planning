# ifndef _DSTAR_SEARCHER_H
# define _DSTAR_SEARCHER_H

#include "Astar_searcher.h"
#include <iostream>
class DstarPathFinder : public AstarPathFinder
{

protected:

    bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
	bool isOccupied(const Eigen::Vector3i & index) const;
	bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;
	bool isFree(const Eigen::Vector3i & index) const;

    double k_min,k_old;

public:


    void insert(GridNodePtr n,double & h_new);
    double process_state();
    void modify_cost(GridNodePtr n);
    void DstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);
    void DstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);


};

# endif