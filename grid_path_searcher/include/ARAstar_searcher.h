#ifndef _ARA_SEARCHER_H
#define _ARA_SEARCHER_H


#include "Astar_searcher.h"
#include "iostream"


class ARAPathFinder : public AstarPathFinder
{

protected:

    std::multimap<double, GridNodePtr> INCONS_set;
        
    std::vector<GridNodePtr> CLOSED_set;
        
    std::vector<std::vector<Eigen::Vector3d>> pathlist;
    
    int search_count=0;

    bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
    bool isOccupied(const Eigen::Vector3i & index) const;
    bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;
    bool isFree(const Eigen::Vector3i & index) const;

    void save_current_path(GridNodePtr terminatePtr);

    void UpdateOpenset();



public:


    double weight=5.0;//f=g+w*h,w = min( w, g(s_goal) / min(g(s_all_incons)+h(s_all_incons)));

    double Weight_;

    void AdjustWeight();

    double fvalue(GridNodePtr node1,GridNodePtr node2);

    void ImprovePath(GridNodePtr startPtr,GridNodePtr endPtr,bool& flag);

    void ARAGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

    std::vector<Eigen::Vector3d> getaraPath();


};


#endif