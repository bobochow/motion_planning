#include "ARAstar_searcher.h"

using namespace std;
using namespace Eigen;

inline bool ARAPathFinder::isOccupied(const Eigen::Vector3i &index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool ARAPathFinder::isFree(const Eigen::Vector3i &index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool ARAPathFinder::isOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool ARAPathFinder::isFree(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

double ARAPathFinder::fvalue(GridNodePtr node1,GridNodePtr node2)
{
    node1->hScore=getHeu(node1,node2);
    return node1->gScore + weight * node1->hScore;

}

void ARAPathFinder::AdjustWeight()
{
    double min_f=inf;
    for(auto iter = openSet.begin();iter!=openSet.end();++iter)
    {
        double f= iter->second->gScore + iter->second->hScore;
        if(f<min_f){
            min_f = f ;
        }
    }
    weight=min(weight,terminatePtr->gScore / min_f);
    return ;
}



void ARAPathFinder::ImprovePath(GridNodePtr startPtr,GridNodePtr endPtr)
{
    
    GridNodePtr currentPtr = NULL;
    GridNodePtr neighborPtr = NULL;
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;
    
    while (endPtr->fScore > openSet.begin()->first )
    {
        // 弹出最大fvalue的节点
        currentPtr = openSet.begin()->second;
        currentPtr->id = -1; // 标记为闭集

        CLOSED_set.emplace(CLOSED_set.end(),currentPtr);
        

        // 从开集中移除
        openSet.erase(openSet.begin());

        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);

        for(int i = 0; i < (int)neighborPtrSets.size(); i++)
        {
            
            neighborPtr = neighborPtrSets[i];
            double gh = currentPtr->gScore + edgeCostSets[i];
            double fh = fvalue(neighborPtr,endPtr);
            // 如果为自由节点
            if(neighborPtr -> id == 0){ 
                
                // 计算相应的g和f，并加入opensets
                neighborPtr->gScore = gh;
                neighborPtr->fScore = fh;
                neighborPtr->cameFrom = currentPtr;

                // 此处注意一定要先计算和赋值完f再加入

                // 判断是否为目标节点 改到此处为了提高代码效率 不用将所有节点加入后等弹出时发现目标再退出
                if(neighborPtr->index == goalIdx){
                    ros::Time time_2 = ros::Time::now();
                    endPtr->fScore=fh;
                    terminatePtr = neighborPtr;
                        
                    return;
                }
                else{
                    // 标记为open list
                    neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore,neighborPtr));
                    neighborPtr->id = 1;
                    continue;

                }
            }
            else if(neighborPtr ->id == 1){ 
                // 如果已经在openlist里面
                if(neighborPtr->gScore > gh)
                {
                    // 更新对应的f值

                    neighborPtr->gScore = gh;
                    neighborPtr->fScore = fh;
                    neighborPtr->cameFrom = currentPtr;
                    openSet.erase(neighborPtr->nodeMapIt);
                    neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore,neighborPtr));


                }
            }
            else if(neighborPtr ->id == -1){
                // 如果是closelist里面,insert into INCONS
                if(neighborPtr->gScore > gh)
                {
                    // 更新对应的f值

                    neighborPtr->gScore = gh;
                    neighborPtr->fScore = fh;
                    neighborPtr->cameFrom = currentPtr;
                    openSet.erase(neighborPtr->nodeMapIt);
                    neighborPtr->nodeMapIt = INCONS_set.insert(make_pair(neighborPtr->fScore,neighborPtr));

                    neighborPtr->id = 2;//ICONS_SET

                }
            }
        }
    }
    
}



void ARAPathFinder::ARAGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
{

    // 记录路径搜索需要的时间
    ros::Time time_1 = ros::Time::now();

    // 记录起点和终点对应的栅格坐标
    Eigen::Vector3i start_idx = coord2gridIndex(start_pt);
    Eigen::Vector3i end_idx = coord2gridIndex(end_pt);

    // 目标索引
    goalIdx = end_idx;

    // 起点和终点的位置(应该可以省略)
    start_pt = gridIndex2coord(start_idx);
    end_pt = gridIndex2coord(end_idx);

    // 初始化起点和终点节点(因为前面已经初始化过，所以不需要再New)
    GridNodePtr startPtr = GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap[end_idx(0)][end_idx(1)][end_idx(2)];

    // 待弹出点集
    openSet.clear();
    CLOSED_set.clear();
    INCONS_set.clear();



    //计算启发函数
    startPtr->gScore = 0;
    startPtr->fScore = fvalue(startPtr, endPtr);

    // 将起点加入开集
    // Unvisited-->0 ,closed--> -1, open--> 1 , ICONS-->2
    startPtr->id = 1;
    startPtr->nodeMapIt = openSet.insert(make_pair(startPtr->fScore, startPtr));

    ImprovePath(startPtr,endPtr);

    AdjustWeight();

    while (weight > 1)
    {
        weight-=0.3;

        for(auto iter =INCONS_set.begin();iter!=INCONS_set.end();iter++ )
        {
            GridNodePtr temp =NULL;
            temp=iter->second;
            temp->id=1;
            INCONS_set.erase(temp->nodeMapIt);
            temp->nodeMapIt=openSet.insert(make_pair(temp->fScore,temp));
            
        }

        CLOSED_set.clear();
        endPtr->gScore=inf;
        ImprovePath(startPtr,endPtr);
        AdjustWeight();

        

    }
    
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );


}