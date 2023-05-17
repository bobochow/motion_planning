#include "LazyTstar_searcher.h"

using namespace std;
using namespace Eigen;

bool LazyTstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

bool LazyTstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

bool LazyTstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

bool LazyTstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

void LazyTstarPathFinder::LThetastarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets){
    neighborPtrSets.clear();
    edgeCostSets.clear();
    

    Eigen::Vector3i current_index = currentPtr->index;
    int current_x = current_index[0];
    int current_y = current_index[1];
    int current_z = current_index[2];

    int n_x,n_y,n_z;
    GridNodePtr tmp_ptr = NULL;
    Eigen::Vector3i tmp_index;
    GridNodePtr parent =currentPtr->cameFrom;
    
    for(int i=-1;i <=1 ;++i){
        for(int j=-1;j<=1;++j){
            for(int k = -1;k<=1;++k){

                if( i==0 && j ==0 && k==0){
                    continue;
                }
                
                n_x = current_x + i;
                n_y = current_y + j;
                n_z = current_z + k;

                if( (n_x < 0 )|| (n_y < 0) || (n_z <0) || (n_x > GLX_SIZE -1) ||  (n_y > GLY_SIZE -1) || (n_z > GLZ_SIZE -1)){
                    continue;
                }

                if(isOccupied(n_x,n_y,n_z)){
                    continue;
                }

                tmp_ptr = GridNodeMap[n_x][n_y][n_z];

                double dist = getHeu(parent,tmp_ptr);//difference

                neighborPtrSets.push_back(tmp_ptr);
                edgeCostSets.push_back(dist);


            }
        }
    }
}


bool LazyTstarPathFinder::LineOfSight(Eigen::Vector3i start, Eigen::Vector3i end){

    if(start==end){
        return true;
    }
    int x1=start(0);
    int x2=end(0);
    int y1=start(1);
    int y2=end(1);
    int z1=start(2);
    int z2=end(2);

    int dx=abs(x1-x2);
    int dy=abs(y1-y2);
    int dz=abs(z1-z2);

    int xs,ys,zs;
    if(x1<x2){
        xs=1;
    }
    else{
        xs=-1;
    }
    if(y1<y2){
        ys=1;
    }
    else{
        ys=-1;
    }
    if(z1<z2){
        zs=1;
    }
    else{
        zs=-1;
    }

    //Driving axis is X-axis
    if(dx >= dy && dx >= dz ){
        int p1=2*dy-dx;
        int p2=2*dz-dx;
        while (x1!=x2)
        {
            x1+=xs;
            if(p1>=0){
                y1+=ys;
                p1-=2*dx;
            }
            if(p2>=0){
                z1+=zs;
                p2-=2*dx;
            }
            p1+=2*dy;
            p2+=2*dz;
            if(isOccupied(x1,y1,z1)){
                return false;
            }
        }
    }
    else if (dy >= dx && dy >= dz)
    {
        int p1 = 2 * dx - dy;
        int p2 = 2 * dz - dy;
        while (y1 != y2)
        {
            y1+=ys;
            if(p1>=0){
                x1+=xs;
                p1-=2*dy;
            }
            if(p2>=0){
                z1+=zs;
                p2-=2*dy;
            }
            p1+=2*dx;
            p2+=2*dz;
            if(isOccupied(x1,y1,z1)){
                return false;
            }
        }
        
    }
    else{
        int p1=2*dy-dz;
        int p2=2*dx-dz;
        while (z1!=z2)
        {
            z1+=zs;
            if(p1>=0){
                y1+=ys;
                p1-=2*dz;
            }
            if(p2>=0){
                x1+=xs;
                p2-=2*dz;
            }
            p1+=2*dx;
            p2+=2*dy;
            if(isOccupied(x1,y1,z1)){
                return false;
            }
        }
        
    }
    
    return true;


}

void LazyTstarPathFinder::ValidateParent(GridNodePtr currentPtr , GridNodePtr goal){

    if (!LineOfSight(currentPtr->cameFrom->index, currentPtr->index)) {

        vector<GridNodePtr> neighborPtrSets;
        vector<double> edgeCostSets;
        GridNodePtr neighborPtr = NULL;

        AstarPathFinder::AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);
        
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            neighborPtr = neighborPtrSets[i];
            double gh = currentPtr->cameFrom->gScore + edgeCostSets[i];
            double fh = gh + getHeu(neighborPtr,goal);

            // 如果为自由节点
            if(neighborPtr -> id == 0){ 
                
                // 计算相应的g和f，并加入opensets
                neighborPtr->gScore = gh;
                neighborPtr->fScore = fh;
                neighborPtr->cameFrom = currentPtr;

                neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore,neighborPtr));
                // 标记为open list
                neighborPtr->id = 1;
                continue;

                
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
        }   
    }
}


void LazyTstarPathFinder::LThetastarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt){
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
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    //计算启发函数
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);

    // 将起点加入开集
    // 自由点为0 闭集为-1 开集为1
    startPtr -> id = 1; 
    startPtr->nodeMapIt = openSet.insert( make_pair(startPtr -> fScore, startPtr) );
    startPtr->cameFrom= startPtr;
    // 预先定义拓展队列和cost队列
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    // this is the main loop
    while ( !openSet.empty()){

        // 弹出最大f的节点
        currentPtr = openSet.begin()->second;
        currentPtr->id = -1; // 标记为闭集

        // 从开集中移除
        openSet.erase(openSet.begin());
        //Eigen::Vector3i current_idx = currentPtr->index;

        // First, check if we have LOS from the parent and update the parent and g-value of curr if necessary.
        ValidateParent(currentPtr,endPtr);

        // 获取拓展集合
        LThetastarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);     

        // 遍历拓展集合        
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            

            neighborPtr = neighborPtrSets[i];
            double gh = currentPtr->cameFrom->gScore + edgeCostSets[i];
            double fh = gh + getHeu(neighborPtr,endPtr);

            // 如果为自由节点
            if(neighborPtr -> id == 0){ 
                
                // 计算相应的g和f，并加入opensets
                neighborPtr->gScore = gh;
                neighborPtr->fScore = fh;
                neighborPtr->cameFrom = currentPtr->cameFrom;

                neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore,neighborPtr));// 此处注意一定要先计算和赋值完f再加入

                // 判断是否为目标节点 改到此处为了提高代码效率 不用将所有节点加入后等弹出时发现目标再退出
                if(neighborPtr->index == goalIdx){
                    ros::Time time_2 = ros::Time::now();
                    terminatePtr = neighborPtr;
                    startPtr->cameFrom= NULL;
                    ROS_WARN("[theta*]{sucess}  Time in theta*  is %f ms, path cost is %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );    
                    return;
                }
                else{
                    // 标记为open list
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
                    neighborPtr->cameFrom = currentPtr->cameFrom;
                    openSet.erase(neighborPtr->nodeMapIt);
                    neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore,neighborPtr));


                }
            }
            else{
                // 如果是closelist里面的则不做处理
                continue;
            }
        } 
    }
        
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1){
    ROS_WARN("Time consume in theta_star path finding is %f", (time_2 - time_1).toSec() );
    }
        
}

