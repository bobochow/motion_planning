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

void DstarPathFinder::insert(GridNodePtr n,double & h_new){
    double kx;
    if(n->id==0){
        kx=h_new;
        n->nodeMapIt = openSet.insert(make_pair(n->fScore,n));
        n->gScore=h_new;
        n->id=1;
    }
    if(n->id==1){
        kx=n->fScore < n->gScore ? n->fScore : n->gScore;
        n->nodeMapIt = openSet.insert(make_pair(n->fScore,n));
        n->gScore=h_new;
        
    }
    if(n->id==-1){
        kx= n->gScore < h_new ? n->gScore : h_new;
        openSet.erase(n->nodeMapIt);
        n->nodeMapIt = openSet.insert(make_pair(n->fScore,n));
        n->gScore=h_new;
        n->id=1;
    }

}

void DstarPathFinder::modify_cost(GridNodePtr n){
    GridNodePtr temp;
    temp =n->cameFrom;
    if(temp->id==-1){
        double t = temp->gScore+ 10000;
        insert(n,t);
    }
}

double DstarPathFinder::process_state(){
    GridNodePtr x=NULL;
    x=openSet.begin()->second;
    x->id=-1;
    k_old=x->fScore;
    openSet.erase(openSet.begin());
    
    GridNodePtr neighborPtr = NULL;
    // 预先定义拓展队列和cost队列
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;
    if(k_old<x->gScore){
        AstarGetSucc(x, neighborPtrSets, edgeCostSets);  
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            neighborPtr = neighborPtrSets[i];
            double cost = getHeu(x,neighborPtr);
            if(neighborPtr->gScore <= k_old && x->fScore > neighborPtr->gScore + cost){
                x->cameFrom=neighborPtr;
                x->gScore=neighborPtr->gScore + cost;
            }
        }
    }
    if(k_old==x->gScore){
        AstarGetSucc(x, neighborPtrSets, edgeCostSets);  
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            neighborPtr = neighborPtrSets[i];
            double cost = getHeu(x,neighborPtr) + x->gScore;
            if(neighborPtr->id==0||(neighborPtr->cameFrom==x && neighborPtr->gScore != cost)||(neighborPtr->cameFrom!=x && neighborPtr->gScore > cost)){
                neighborPtr->cameFrom=x;
                insert(neighborPtr,cost);
            }
        }
    }
    else{
        AstarGetSucc(x, neighborPtrSets, edgeCostSets);  
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            neighborPtr = neighborPtrSets[i];
            double cost = getHeu(x,neighborPtr) + x->gScore;
            if(neighborPtr->id==0||(neighborPtr->cameFrom == x && neighborPtr->gScore != cost)){
                neighborPtr->cameFrom=x;
                insert(neighborPtr,cost);
            }
            else{
                if(neighborPtr->cameFrom!=x && neighborPtr->gScore > cost){
                    insert(x,x->gScore);
                }
                else{
                    if(neighborPtr->cameFrom!=x && neighborPtr->gScore > cost && neighborPtr->id==-1 &&  neighborPtr->gScore > k_old){
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
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
    }
    else{
        do
        {
            do
            {
                currentPtr=currentPtr->cameFrom;

            } while (currentPtr->index!=goalIdx && !isOccupied(currentPtr->cameFrom->index));
            if(currentPtr->index==goalIdx){
                ros::Time time_2 = ros::Time::now();
                terminatePtr = currentPtr;
                ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );    
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
                        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
                }
            }
        } while (1);
    }
}