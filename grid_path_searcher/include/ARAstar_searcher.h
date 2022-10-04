#ifndef _ARA_SEARCHER_H
#define _ARA_SEARCHER_H


#include "Astar_searcher.h"
#include "iostream"


class ARAPathFinder : public AstarPathFinder
{

protected:

    std::multimap<double, GridNodePtr> INCONS_set;
        
    std::vector<GridNodePtr> CLOSED_set;
        

    bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
    bool isOccupied(const Eigen::Vector3i & index) const;
    bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;
    bool isFree(const Eigen::Vector3i & index) const;


public:


    double weight;//w' = min( w, g(s_goal) / min(g(s_all_incons)+h(s_all_incons)));
        
    void AdjustWeight();

    double fvalue(GridNodePtr node1,GridNodePtr node2);

    void ImprovePath(GridNodePtr startPtr,GridNodePtr endPtr);

    void ARAGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);




};


#endif