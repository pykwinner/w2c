#include "upros_navigation/AreaPath.h"

// TODO: 
// 优化区域范围的变量接口，不使用全局变量

namespace area_path
{
    // public:
    AreaPath::AreaPath(nav_msgs::OccupancyGrid map)
    {
        ros::NodeHandle private_nh("~");
        m_pathPub = private_nh.advertise<nav_msgs::Path>("area_path", 1);

        ros::NodeHandle params_nh("~/area_path_params");
        params_nh.param<int>("row_size", ROW_SIZE, 8);
        params_nh.param<int>("obstacle_radius", OBSTACLE_RADIUS, 8);
        params_nh.param<double>("min_length", MIN_LENGTH, 0.5);

        m_map = map;
        m_mapWidth = m_map.info.width; //获取地图尺寸
        m_mapHeight = m_map.info.height;
        m_resolution = m_map.info.resolution; //分辨率
        if (m_map.info.origin.orientation.w != 1)
        {
            //本功能包无法处理地图原点旋转的情况
            ROS_ERROR("The map's origin include rotation! area_path_node can't handle rotated map!");
            ros::shutdown();
        }
        m_mapOffsetX = m_map.info.origin.position.x;
        m_mapOffsetY = m_map.info.origin.position.y;

        cout << "The size of map is " << m_mapWidth << "  " << m_mapHeight << endl;
        cout << "The resolution of map is " << m_resolution << endl;

        ObstacleExpand();

        m_sortedSize = 0;
        m_isCalculated = false;
    }

    void AreaPath::AddAreaInPixels(unsigned int pointAx, unsigned int pointAy, unsigned int pointBx, unsigned int pointBy)
    {
        m_isCalculated = false;

        if (pointAx == 0 && pointAy == 0 && pointBx == 0 && pointBy == 0) //全图模式
        {
            m_pointAx = 0;
            m_pointAy = 0;
            m_pointBx = m_mapWidth - 1;
            m_pointBy = m_mapHeight - 1;
        }
        else
        {
            m_pointAx = MAX(MIN(pointAx, pointBx), 0);
            m_pointAy = MAX(MIN(pointAy, pointBy), 0);
            m_pointBx = MIN(MAX(pointAx, pointBx), m_mapWidth - 1);
            m_pointBy = MIN(MAX(pointAy, pointBy), m_mapHeight - 1);
        }

        AppendPathInArea();
    }

    void AreaPath::AddAreaInMeters(double pointAxMeter, double pointAyMeter, double pointBxMeter, double pointByMeter)
    {
        unsigned int pointAx = (pointAxMeter - m_mapOffsetX) / m_resolution;
        unsigned int pointAy = (pointAyMeter - m_mapOffsetY) / m_resolution;
        unsigned int pointBx = (pointBxMeter - m_mapOffsetX) / m_resolution;
        unsigned int pointBy = (pointByMeter - m_mapOffsetY) / m_resolution;

        AddAreaInPixels(pointAx, pointAy, pointBx, pointBy);
    }

    bool AreaPath::CalculatePath()
    {
        SortPoints();
        m_path = PoseStamped2Path(m_sortedPoseStampedVector);
        m_isCalculated = true;
        return true;
    }

    nav_msgs::Path AreaPath::GetPath()
    {
        return m_path;
    }

    bool AreaPath::ClearPath()
    {
        m_poseStampedVector.clear();
        m_sortedPoseStampedVector.clear();
        m_sortedSize = 0;
        m_isCalculated = false;
        m_path = PoseStamped2Path(m_sortedPoseStampedVector);
        return true;
    }

    void AreaPath::PublishAreaPath()
    {
        if (!m_isCalculated)
        {
            ROS_ERROR(
                "This path has not been calculated yet, but it is being used, please call CalculatePath() before use");
            return;
        }
        m_pathPub.publish(m_path);
    }

    // private:
    void AreaPath::PrintMap()
    {
        for (int i = m_mapHeight - 1; i >= 0; i--)
        {
            for (int j = 0; j < m_mapWidth; j++)
            {
                if (i == 0 && j == 0)
                {
                    cout << "X"; //原点
                    continue;
                }
                int a = (unsigned int)m_map.data[i * m_mapWidth + j];
                switch (a)
                {
                case SIGN_NULL: //灰区
                    cout << " ";
                    break;
                case SIGN_EMPTY: //白区
                    cout << "O";
                    break;
                case SIGN_OBSTACLE: //膨胀区
                    cout << "/";
                    break;
                case SIGN_WALL: //黑区
                    cout << "#";
                    break;
                }
            }
            cout << endl;
        }
    }

    void AreaPath::ObstacleExpand()
    {
        int beginX, beginY, endX, endY;

        for (int i = 0; i < m_mapHeight; i++)
        {
            for (int j = 0; j < m_mapWidth; j++)
            {
                bool expand_flag = false;
                switch (m_map.data[i * m_mapWidth + j])
                {
                case SIGN_NULL: //灰区
                    break;
                case SIGN_EMPTY: //白区
                    //进行膨胀
                    beginX = MAX(0, j - OBSTACLE_RADIUS);
                    beginY = MAX(0, i - OBSTACLE_RADIUS);
                    endX = MIN(m_mapWidth - 1, j + OBSTACLE_RADIUS);
                    endY = MIN(m_mapHeight - 1, i + OBSTACLE_RADIUS);

                    for (int ii = beginY; ii < endY; ii++)
                    {
                        for (int jj = beginX; jj < endX; jj++)
                        {
                            if (m_map.data[ii * m_mapWidth + jj] == SIGN_NULL || m_map.data[ii * m_mapWidth + jj] == SIGN_WALL) //若是灰区或黑区
                            {
                                if ((pow(ii - i, 2) + pow(jj - j, 2)) < pow(OBSTACLE_RADIUS, 2)) //若在膨胀半径内
                                {
                                    m_map.data[i * m_mapWidth + j] = SIGN_OBSTACLE; //设为膨胀区
                                    expand_flag = true;
                                    break;
                                }
                            }
                        }
                        if (expand_flag)
                        {
                            expand_flag = false;
                            break;
                        }
                    }
                    break;
                case SIGN_OBSTACLE: //膨胀区
                    break;
                case SIGN_WALL: //黑区
                    break;
                }
            }
        }
    }

    void AreaPath::AppendPathInArea()
    {
        std::vector<geometry_msgs::PoseStamped> PSVIA_Fitted, PSVIA_X, PSVIA_Y;

        PSVIA_X = GetPointsInArea(DIRECTION_X); // X方向扫描得到的点列
        PSVIA_Y = GetPointsInArea(DIRECTION_Y); // Y方向扫描得到的点列

        //最适方向设置为点最少的方向
        if (PSVIA_X.size() <= PSVIA_Y.size())
        {
            PSVIA_Fitted = PSVIA_X;
        }
        else
        {
            PSVIA_Fitted = PSVIA_Y;
        }

        //将该区域点列并入总点列
        m_poseStampedVector.insert(m_poseStampedVector.end(), PSVIA_Fitted.begin(), PSVIA_Fitted.end());
    }

    std::vector<geometry_msgs::PoseStamped> AreaPath::GetPointsInArea(int MainDirection)
    {
        std::vector<geometry_msgs::PoseStamped> PoseStampedVectorInArea;

        int direction; //行内方向，1往右（递增），2往左（递减），3往下（递增），4往上（递减）
        int height_i, width_j;
        bool isInWhite = false;     //上一点在白区标志
        bool getFirstPoint = false; //是否找到可用起始点

        //区域的起始点
        height_i = m_pointAy;
        width_j = m_pointAx;

        //设置初始方向
        if (MainDirection == DIRECTION_X)
        {
            direction = 1;
        }
        else if (MainDirection == DIRECTION_Y)
        {
            direction = 3;
        }

        //起始点处理
        if (m_map.data[height_i * m_mapWidth + width_j] == SIGN_EMPTY)
        {
            isInWhite = true;
            getFirstPoint = true;
            AddPoint(width_j, height_i, direction, PoseStampedVectorInArea);
        }
        else
        {
            isInWhite = false;
            getFirstPoint = false;
        }

        //区域内其他点处理
        if (MainDirection == DIRECTION_X) // x主方向路径
        {
            while (true)
            {
                //下一点寻找
                if (direction == 1)
                {
                    if (width_j >= m_pointBx) //搜索到区域右边缘
                    {
                        //换行
                        if (getFirstPoint) //如果找到可用起始点
                        {
                            height_i += ROW_SIZE; //一次换多行
                        }
                        else //没找到可用起始点
                        {
                            height_i++; //逐行搜索直到找到起始点
                        }
                        if (height_i > m_pointBy) //行超过区域下方
                        {
                            break; //不再继续搜索，跳出循环结束路径
                        }
                        //换向
                        direction = 2;
                        isInWhite = false;
                    }
                    else //正常情况
                    {
                        width_j++;
                    }
                }
                else if (direction == 2)
                {
                    if (width_j <= m_pointAx) //搜索到区域左边缘
                    {
                        //换行
                        if (getFirstPoint) //如果找到可用起始点
                        {
                            height_i += ROW_SIZE; //一次换多行
                        }
                        else //没找到可用起始点
                        {
                            height_i++; //逐行搜索直到找到起始点
                        }
                        if (height_i > m_pointBy) //行超过区域下方
                        {
                            break; //不再继续搜索，跳出循环结束路径
                        }
                        //换向
                        direction = 1;
                        isInWhite = false;
                    }
                    else //正常情况
                    {
                        width_j--;
                    }
                }

                //当前点处理
                if (m_map.data[height_i * m_mapWidth + width_j] == SIGN_EMPTY) //如果当前点是白区点
                {

                    if (width_j == m_pointAx || width_j == m_pointBx) //是区域边缘的白区点
                    {
                        if ((direction == 1 && width_j == m_pointBx) || (direction == 2 && width_j == m_pointAx)) //向右到边缘终点or向左到边缘终点
                        {
                            if (isInWhite) //现在和之前均在白区
                            {
                                if (!getFirstPoint)
                                {
                                    getFirstPoint = true;
                                }
                                AddPoint(width_j, height_i, direction, PoseStampedVectorInArea);
                            }
                            else //上一个是非白区
                            {
                                // do nothing
                                //是边缘1格长度的线段，不能添加该边缘点
                            }
                        }
                        else //是边缘起点，直接添加点
                        {
                            if (!getFirstPoint)
                            {
                                getFirstPoint = true;
                            }
                            AddPoint(width_j, height_i, direction, PoseStampedVectorInArea);
                        }
                    }
                    else //非区域边缘白区点
                    {
                        if (isInWhite) //现在和之前均在白区，无需处理
                        {
                            // do noting
                        }
                        else //上一个是非白区
                        {
                            if (!getFirstPoint)
                            {
                                getFirstPoint = true;
                            }
                            AddPoint(width_j, height_i, direction, PoseStampedVectorInArea);
                        }
                    }
                    isInWhite = true;
                }
                else //如果当前点是非白区点
                {
                    if (isInWhite) //之前是白区
                    {
                        if (!getFirstPoint)
                        {
                            getFirstPoint = true;
                        }
                        AddPoint(width_j, height_i, direction, PoseStampedVectorInArea);
                    }
                    else //现在和之前均在非白区，无需处理
                    {
                        continue;
                    }

                    isInWhite = false;
                }
            }
        }
        else if (MainDirection == DIRECTION_Y) // y主方向路径
        {
            while (true)
            {
                //下一点寻找
                if (direction == 3)
                {
                    if (height_i >= m_pointBy) //搜索到区域下边缘
                    {
                        //换行
                        if (getFirstPoint) //如果找到可用起始点
                        {
                            width_j += ROW_SIZE; //一次换多列
                        }
                        else //没找到可用起始点
                        {
                            width_j++; //逐列搜索直到找到起始点
                        }
                        if (width_j > m_pointBx) //列超过区域右方
                        {
                            break; //不再继续搜索，跳出循环结束路径
                        }
                        //换向
                        direction = 4;
                        isInWhite = false;
                    }
                    else //正常情况
                    {
                        height_i++;
                    }
                }
                else if (direction == 4)
                {
                    if (height_i <= m_pointAy) //搜索到区域左边缘
                    {
                        //换行
                        if (getFirstPoint) //如果找到可用起始点
                        {
                            width_j += ROW_SIZE; //一次换多行
                        }
                        else //没找到可用起始点
                        {
                            width_j++; //逐行搜索直到找到起始点
                        }
                        if (width_j > m_pointBx) //行超过区域下方
                        {
                            break; //不再继续搜索，跳出循环结束路径
                        }
                        //换向
                        direction = 3;
                        isInWhite = false;
                    }
                    else //正常情况
                    {
                        height_i--;
                    }
                }

                //当前点处理
                if (m_map.data[height_i * m_mapWidth + width_j] == SIGN_EMPTY) //如果当前点是白区点
                {

                    if (height_i == m_pointAy || height_i == m_pointBy) //是区域边缘的白区点，添加该点
                    {
                        if ((direction == 3 && height_i == m_pointBy) || (direction == 4 && height_i == m_pointAy)) //向右到边缘终点or向左到边缘终点
                        {
                            if (isInWhite) //现在和之前均在白区
                            {
                                if (!getFirstPoint)
                                {
                                    getFirstPoint = true;
                                }
                                AddPoint(width_j, height_i, direction, PoseStampedVectorInArea);
                            }
                            else //上一个是非白区
                            {
                                // do nothing
                                //是边缘1格长度的线段，不能添加该边缘点
                            }
                        }
                        else //是边缘起点，直接添加点
                        {
                            if (!getFirstPoint)
                            {
                                getFirstPoint = true;
                            }
                            AddPoint(width_j, height_i, direction, PoseStampedVectorInArea);
                        }
                    }
                    else //非区域边缘白区点
                    {
                        if (isInWhite) //现在和之前均在白区，无需处理
                        {
                            // do noting
                        }
                        else //上一个是非白区
                        {
                            if (!getFirstPoint)
                            {
                                getFirstPoint = true;
                            }
                            AddPoint(width_j, height_i, direction, PoseStampedVectorInArea);
                        }
                    }
                    isInWhite = true;
                }
                else //如果当前点是非白区点
                {

                    if (isInWhite) //之前是白区
                    {
                        if (!getFirstPoint)
                        {
                            getFirstPoint = true;
                        }
                        AddPoint(width_j, height_i, direction, PoseStampedVectorInArea);
                    }
                    else //现在和之前均在非白区，无需处理
                    {
                        continue;
                    }

                    isInWhite = false;
                }
            }
        }

        return PoseStampedVectorInArea;
    }

    void AreaPath::SortPoints()
    {
        if (m_poseStampedVector.size() == 0)
        {
            ROS_WARN("No point can be sorted in this area");
            return;
        }
        else
        {
            ROS_INFO("Sorting in this area");
        }

        // DEBUG:两种排法还可优化
        //  // 起终点确定排法
        //  m_sortedPoseStampedVector.resize(m_poseStampedVector.size());
        //  //第一根线段直接放入
        //  m_sortedPoseStampedVector[0] = m_poseStampedVector[0];
        //  m_sortedPoseStampedVector[1] = m_poseStampedVector[1];
        //  //偶数号为线段起点，奇数号为线段终点
        //  for (int i = 2; i < m_sortedPoseStampedVector.size();) //i++操作在循环内部完成
        //  {
        //      double minDistance2 = DBL_MAX;
        //      int minDistanceIndex = -1;
        //      double distance2;
        //      double deltaX;
        //      double deltaY;

        //     //遍历每个线段，寻找与现在终点最近的那个起点
        //     for (int j = 0; j < m_poseStampedVector.size() / 2; j++)
        //     {
        //         deltaX = m_sortedPoseStampedVector[i - 1].pose.position.x - m_poseStampedVector[j * 2].pose.position.x;
        //         deltaY = m_sortedPoseStampedVector[i - 1].pose.position.y - m_poseStampedVector[j * 2].pose.position.y;
        //         distance2 = deltaX * deltaX + deltaY * deltaY;

        //         if(distance2<minDistance2)
        //         {
        //             minDistance2 = distance2;
        //             minDistanceIndex = j;
        //         }
        //     }

        //     //如果没找到，报错
        //     if (minDistance2 == DBL_MAX || minDistanceIndex == -1)
        //     {
        //         ROS_ERROR("Can not find minDistance!");
        //         return;
        //     }
        //     else //找到了最近的起点
        //     {
        //         //复制
        //         m_sortedPoseStampedVector[i] = m_poseStampedVector[minDistanceIndex * 2];
        //         i++;
        //         m_sortedPoseStampedVector[i] = m_poseStampedVector[minDistanceIndex * 2 + 1];
        //         i++;
        //         //删除
        //         m_poseStampedVector.erase(m_poseStampedVector.begin() + (minDistanceIndex * 2), m_poseStampedVector.begin() + (minDistanceIndex * 2 + 2));
        //     }
        // }

        /////////////////////////////

        // DEBUG:可以试试在sort的时候更改点的方向？

        //最近点排法 起终点可排
        long unsigned int totalSize = m_poseStampedVector.size();
        m_sortedPoseStampedVector.resize(m_sortedSize + totalSize);
        //第一根线段直接放入
        m_sortedPoseStampedVector[m_sortedSize + 0] = m_poseStampedVector[0];
        m_sortedPoseStampedVector[m_sortedSize + 1] = m_poseStampedVector[1];
        m_poseStampedVector.erase(m_poseStampedVector.begin() + 0, m_poseStampedVector.begin() + 2);
        //偶数号为线段起点，奇数号为线段终点

        for (long unsigned int i = 2; i < totalSize;) // i++操作在循环内部完成
        {
            double minDistance2 = DBL_MAX;
            int minDistanceIndex = -1;
            double distance2;
            double deltaX;
            double deltaY;

            //遍历现在剩下的每个点，寻找与现在终点最近的那个点
            for (long unsigned int j = 0; j < m_poseStampedVector.size(); j++)
            {
                deltaX = m_sortedPoseStampedVector[m_sortedSize + i - 1].pose.position.x - m_poseStampedVector[j].pose.position.x;
                deltaY = m_sortedPoseStampedVector[m_sortedSize + i - 1].pose.position.y - m_poseStampedVector[j].pose.position.y;
                distance2 = deltaX * deltaX + deltaY * deltaY;

                if (distance2 < minDistance2)
                {
                    minDistance2 = distance2;
                    minDistanceIndex = j;
                }
            }

            //如果没找到，报错
            if (minDistance2 == DBL_MAX || minDistanceIndex == -1)
            {
                ROS_ERROR("Can not find minDistance!");
                return;
            }
            else //找到了最近的点
            {
                if (minDistanceIndex % 2 == 0) //是偶数，原来是起点
                {
                    //复制
                    m_sortedPoseStampedVector[m_sortedSize + i] = m_poseStampedVector[minDistanceIndex];
                    i++;
                    m_sortedPoseStampedVector[m_sortedSize + i] = m_poseStampedVector[minDistanceIndex + 1];
                    i++;
                    //删除
                    m_poseStampedVector.erase(m_poseStampedVector.begin() + (minDistanceIndex), m_poseStampedVector.begin() + (minDistanceIndex + 2));
                }
                else //是奇数，原来是终点
                {
                    //复制
                    m_sortedPoseStampedVector[m_sortedSize + i] = m_poseStampedVector[minDistanceIndex];
                    i++;
                    m_sortedPoseStampedVector[m_sortedSize + i] = m_poseStampedVector[minDistanceIndex - 1];
                    i++;
                    //删除
                    m_poseStampedVector.erase(m_poseStampedVector.begin() + (minDistanceIndex - 1), m_poseStampedVector.begin() + (minDistanceIndex + 1));
                }
            }
        }
        m_sortedSize = m_sortedPoseStampedVector.size(); //记录已排序点数
    }

    void AreaPath::AddPoint(unsigned int pointX, unsigned int pointY, int direction, std::vector<geometry_msgs::PoseStamped> &PoseStampedVector)
    {
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.pose.orientation.x = 0;
        poseStamped.pose.orientation.y = 0;
        switch (direction)
        {
        case 1:
            poseStamped.pose.orientation.z = 0;
            poseStamped.pose.orientation.w = 1;
            break;
        case 2:
            poseStamped.pose.orientation.z = 1;
            poseStamped.pose.orientation.w = 0;
            break;
        case 3:
            poseStamped.pose.orientation.z = 0.707;
            poseStamped.pose.orientation.w = 0.707;
            break;
        case 4:
            poseStamped.pose.orientation.z = -0.707;
            poseStamped.pose.orientation.w = 0.707;
            break;
        }

        poseStamped.header.stamp = ros::Time::now();
        poseStamped.header.frame_id = "map";

        //需要加地图原点的偏置
        poseStamped.pose.position.x = pointX * m_resolution + m_mapOffsetX;
        poseStamped.pose.position.y = pointY * m_resolution + m_mapOffsetY;
        poseStamped.pose.position.z = 0;

        PoseStampedVector.push_back(poseStamped);

        if (PoseStampedVector.size() % 2 == 0 && PoseStampedVector.size() != 0) //如果刚建完一个线段
        {
            int size = PoseStampedVector.size();
            double distance;
            double deltaX;
            double deltaY;

            deltaX = PoseStampedVector[size - 1].pose.position.x - PoseStampedVector[size - 2].pose.position.x;
            deltaY = PoseStampedVector[size - 1].pose.position.y - PoseStampedVector[size - 2].pose.position.y;
            distance = sqrt(deltaX * deltaX + deltaY * deltaY);

            if (distance < MIN_LENGTH) //如果线段短于阈值
            {
                //去除最后两个点
                PoseStampedVector.pop_back();
                PoseStampedVector.pop_back();
            }
        }
    }

    nav_msgs::Path AreaPath::PoseStamped2Path(const std::vector<geometry_msgs::PoseStamped> &poseStampedVector)
    {
        nav_msgs::Path path;
        path.poses.resize(poseStampedVector.size());
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();

        for (unsigned int i = 0; i < poseStampedVector.size(); i++)
        {
            path.poses[i] = poseStampedVector[i];
        }

        return path;
    }

} // namespace area_path
