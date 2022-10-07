#include "Dstar_searcher.h"

using namespace std;
using namespace Eigen;

inline bool DstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool DstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool DstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool DstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}



void DstarPathFinder::insert(GridNodePtr n,double h_new){
    
    if(n->id==0){
        n->fScore=h_new;
        n->gScore=h_new;
        n->id=1;
        n->nodeMapIt = openSet.insert(make_pair(n->fScore,n));
    }
    else if(n->id==1){
        
        n->fScore=std::min(n->fScore,h_new);
        n->gScore=h_new;
        openSet.erase(n->nodeMapIt);
        n->nodeMapIt = openSet.insert(make_pair(n->fScore,n));
    }
    else if(n->id==-1){
        
        n->fScore=min(n->gScore,h_new);
        n->gScore=h_new;
        n->id=1;
        n->nodeMapIt = openSet.insert(make_pair(n->fScore,n));
    }

}

void DstarPathFinder::modify_cost(GridNodePtr n){
    
    if(n->id==-1){
        insert(n,n->cameFrom->gScore+ 100000.0);
    }
}

double DstarPathFinder::process_state(){
    
    if(openSet.empty()){
        return -1;
    }

    GridNodePtr current = NULL;
    current=openSet.begin()->second;
    current->id=-1;
    k_old=current->fScore;

    openSet.erase(openSet.begin());
    
    GridNodePtr neighborPtr = NULL;
    // 预先定义拓展队列和cost队列
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    // -----------------------主要条件过程--------------------------------------
    // rasie状态，该状态表明，由于修改了状态点的可通行状态，即：h（不可通行h很大），
    // 导致该点曾经的优代价k_old已经不在等于h（曾经最优k_old=h），而是k_old小于h
    // 这种点不仅仅是，直接修改的点(faster)，还包括faster扩张后的点，
    // 这些点由于曾经指向faster,导致其h值在faster扩张过程中被变得很大很大。
    // 对于这样的raise点，首先希望他能够从周围点中获取更短的代价路径，
    // 一旦有这样的点，就指向他。然而，一般不存在，因为，曾经的路线本来就是最优的，周围点的代价都是大于k_old的
    // 当然，由于扩展的进行，状态改的越来越多，说不定就有这样的情况发生


    if(k_old < current->gScore){
        AstarGetSucc(current, neighborPtrSets, edgeCostSets);  
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            neighborPtr = neighborPtrSets[i];
            double cost = edgeCostSets[i];
            if(neighborPtr->gScore <= k_old && current->gScore > neighborPtr->gScore + cost){
                current->cameFrom=neighborPtr;
                current->gScore=neighborPtr->gScore + cost;
            }
        }
    }
    if(k_old==current->gScore){
        AstarGetSucc(current, neighborPtrSets, edgeCostSets);  
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            neighborPtr = neighborPtrSets[i];
            double cost = edgeCostSets[i] + current->gScore;
            if(neighborPtr->id==0||(neighborPtr->cameFrom==current && neighborPtr->gScore != cost)||(neighborPtr->cameFrom!=current && neighborPtr->gScore > cost)){
                neighborPtr->cameFrom=current;
                insert(neighborPtr,cost);
            }
        }
    }
    else{
        AstarGetSucc(current, neighborPtrSets, edgeCostSets);  
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            neighborPtr = neighborPtrSets[i];
            double cost = edgeCostSets[i] + current->gScore;
            if(neighborPtr->id==0||(neighborPtr->cameFrom == current && neighborPtr->gScore != cost)){
                neighborPtr->cameFrom=current;
                insert(neighborPtr,cost);
            }
            else{
                if(neighborPtr->cameFrom!=current && neighborPtr->gScore > cost){
                    insert(current,current->gScore);
                }
                else{
                    if(neighborPtr->cameFrom!=current && neighborPtr->gScore > cost && neighborPtr->id==-1 &&  neighborPtr->gScore > k_old){
                        insert(neighborPtr,neighborPtr->gScore);
                    }
                }
            }
        }
        
    }

    return openSet.begin()->first;

}


void DstarPathFinder::DstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
{
    // 记录路径搜索需要的时间
    ros::Time time_1 = ros::Time::now();    

    // 记录起点和终点对应的栅格坐标
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);

    // 目标索引
    goalIdx = end_idx;

    // 初始化起点和终点节点(因为前面已经初始化过，所以不需要再New)
    GridNodePtr startPtr = GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr   = GridNodeMap[end_idx(0)][end_idx(1)][end_idx(2)];

    // 待弹出点集
    openSet.clear();

    // 定义要弹出的节点
    GridNodePtr currentPtr  = startPtr;
    
    //push goal state into openset
    endPtr->id=1;
    endPtr->gScore=0;//gscore -> h    fscore-> k
    endPtr->fScore=0;
    endPtr->nodeMapIt=openSet.insert(make_pair(endPtr->fScore,endPtr));

    do
    {
        k_min=process_state();
    } while (k_min !=-1 && startPtr->id!=-1 );
    //if search fails
    if(k_min==-1){
        
        ros::Time time_2 = ros::Time::now();
        if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Dstar path finding is %f", (time_2 - time_1).toSec() );
    }
    else{
        do
        {
            do
            {
                currentPtr=currentPtr->cameFrom;
                DstarPath.push_back(currentPtr->coord);

            } while (currentPtr->index!=goalIdx && !isOccupied(currentPtr->cameFrom->index));
            if(currentPtr->index==goalIdx){
                ros::Time time_2 = ros::Time::now();
                terminatePtr = currentPtr;
                ROS_WARN("[D*]{sucess}  Time in D*  is %f ms", (time_2 - time_1).toSec() * 1000.0);    
                return;
            }
            else{
                modify_cost(currentPtr);
                do
                {
                    k_min=process_state();
                } while (k_min !=-1 && k_min <currentPtr->gScore);//k_min>=h(Y)时表示已经找到了最优路径，不可能有更优
                if(k_min==-1){
                    ros::Time time_2 = ros::Time::now();
                    if((time_2 - time_1).toSec() > 0.1)
                        ROS_WARN("Time consume in Dstar path finding is %f", (time_2 - time_1).toSec() );
                }
            }
        } while (1);
    }
}