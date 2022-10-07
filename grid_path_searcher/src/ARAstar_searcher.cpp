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

    //ROS_INFO("gscore is %f",terminatePtr->gScore );
    //ROS_INFO("min_f is %f",min_f );
    Weight_=min(weight,terminatePtr->gScore / min_f);
    ROS_INFO("weight is %f",weight );
    ROS_INFO("Weight_ is %f",Weight_ );
    return ;
}

void ARAPathFinder::save_current_path(GridNodePtr terminate){

    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;
    
    GridNodePtr tmp_ptr = terminate;
    
    while(tmp_ptr->cameFrom != NULL )
    {
        gridPath.push_back(tmp_ptr);
        tmp_ptr = tmp_ptr->cameFrom;
    }

    for (auto ptr: gridPath)
        path.push_back(ptr->coord);

    reverse(path.begin(),path.end());
    pathlist.push_back(path);
    ROS_WARN("pathsize:%d",gridPath.size());
    ROS_WARN("pathlistsize:%d",pathlist.size());
}

void ARAPathFinder::UpdateOpenset(){
    
    std::multimap<double, GridNodePtr> temp_queue;
    std::swap(temp_queue,openSet);

    while(!temp_queue.empty()){
        GridNodePtr temp =temp_queue.begin()->second;
            
        temp->fScore=temp->gScore+weight*temp->hScore;
            
        temp->nodeMapIt=openSet.insert(make_pair(temp->fScore,temp));

        temp_queue.erase(temp_queue.begin());
    }
}


void ARAPathFinder::ImprovePath(GridNodePtr startPtr,GridNodePtr endPtr)
{
    
    CLOSED_set.clear();// closeset，初始化为空集
    INCONS_set.clear();//非一致列表，初始化为空集

    GridNodePtr currentPtr = NULL;
    GridNodePtr neighborPtr = NULL;
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;
    
    

    while (endPtr->fScore > openSet.begin()->first )
    {
        // 弹出最大fvalue的节点
        currentPtr = openSet.begin()->second;
        currentPtr->id = -1; 
        CLOSED_set.emplace(CLOSED_set.end(),currentPtr);
        

        // 从开集中移除
        openSet.erase(openSet.begin());

        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);

        for(int i = 0; i < (int)neighborPtrSets.size(); i++)
        {
            
            neighborPtr = neighborPtrSets[i];
            double gh = currentPtr->gScore + edgeCostSets[i];
            
            // 如果为自由节点
            if(neighborPtr -> id == 0){ 
                
                // 计算相应的g和f，并加入opensets
                
                neighborPtr->gScore = gh;
                neighborPtr->fScore = fvalue(neighborPtr,endPtr);
                neighborPtr->cameFrom = currentPtr;

                
                if(neighborPtr->index == goalIdx){
                    
                    endPtr->fScore=fvalue(neighborPtr,endPtr);
                    terminatePtr = neighborPtr;
                    save_current_path(neighborPtr);
                    search_count++;
                    
                    ROS_INFO("Search successfully");
                    // ROS_INFO("END Gscore is %f",terminatePtr->gScore);
                    // ROS_INFO("END Hscore is %f",terminatePtr->hScore);
                    // ROS_INFO("END Fscore is %f",terminatePtr->fScore);
                    // ROS_INFO("END id is %d",terminatePtr->id);
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
                    neighborPtr->fScore = fvalue(neighborPtr,endPtr);
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
                    neighborPtr->fScore = fvalue(neighborPtr,endPtr);
                    neighborPtr->cameFrom = currentPtr;
                    //openSet.erase(neighborPtr->nodeMapIt);
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


    weight=5.0;
    Weight_=5.0;
    pathlist.clear();
    INCONS_set.clear();
    CLOSED_set.clear();
    search_count=0;

    // 记录起点和终点对应的栅格坐标
    Eigen::Vector3i start_idx = coord2gridIndex(start_pt);
    Eigen::Vector3i end_idx = coord2gridIndex(end_pt);

    // 目标索引
    goalIdx = end_idx;

    // 初始化起点和终点节点(因为前面已经初始化过，所以不需要再New)
    GridNodePtr startPtr = GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap[end_idx(0)][end_idx(1)][end_idx(2)];

    // 待弹出点集
    openSet.clear();
    
    //计算启发函数
    startPtr->gScore = 0;
    startPtr->fScore = fvalue(startPtr, endPtr);

    // 将起点加入开集
    // Unvisited-->0 ,closed--> -1, open--> 1 , ICONS-->2
    startPtr->id = 1;
    startPtr->nodeMapIt = openSet.insert(make_pair(startPtr->fScore, startPtr));
    
    ImprovePath(startPtr,endPtr);
    
    
    while (Weight_ - 1.0> eps )
    {
        
        //ROS_INFO("search num is %d",search_count);
        //decrease w
        weight-=0.5;
        ROS_INFO("incons size is %d",INCONS_set.size());
        for(auto iter =INCONS_set.begin();iter!=INCONS_set.end();iter++ )
        {
            GridNodePtr temp =NULL;
            temp=iter->second;
            temp->id=1;
            temp->nodeMapIt=openSet.insert(make_pair(temp->fScore,temp));
            INCONS_set.erase(temp->nodeMapIt);
        }
        ROS_INFO("expanded node nums is %d",CLOSED_set.size());
        //ROS_INFO("union");
        AdjustWeight();
        //ROS_INFO("adjust");
        UpdateOpenset();
        //ROS_INFO("update");
        //endPtr->gScore=inf;
        ImprovePath(startPtr,endPtr);
        //ROS_INFO("improve");

    }
    
    ros::Time time_2 = ros::Time::now();
    ROS_WARN("[ARA*]{sucess}  Time in ARA*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, endPtr->gScore * resolution );
}


vector<Vector3d> ARAPathFinder::getaraPath(){

    return *pathlist.rbegin();
}