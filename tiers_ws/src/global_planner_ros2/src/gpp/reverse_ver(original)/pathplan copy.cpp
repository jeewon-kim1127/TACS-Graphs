#include "pathplan.h"

#define _USE_MATH_DEFINES

using namespace saslam;
// using namespace unavlib;

// int VISUALIZATION_SHIFT_X = 20;
// int VISUALIZATION_SHIFT_Y = 20;

std::mutex mutex_map;

pathplan::pathplan()
{
    // n = rclcpp::Node::make_shared("n_node");
    m_flag_stop = false;

}

pathplan::~pathplan()
{

}


void pathplan::callback_initialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
//    std::cout<<"[PATHPLAN] INIT POSE SUB"<<std::endl;
    mutex_map.lock();
    if(m_occumap.data.size()==0)
    {
           std::cout << "[PATHPLAN] Warning : no map " << std::endl;
        mutex_map.unlock();
        return;
    }
    mutex_map.unlock();
    // if(m_waypts.empty())
    // {
    //    std::cout << "[PATHPLAN] Warning : no waypoint " << std::endl;
    //     return;
    // }
    m_flag_stop = false;
    mutex_map.lock();
    procMap();
    cvMapForPlan = m_occumapCv.cvMapForPlan.clone();

    m_curPose = geoPose2eigen(msg->pose.pose);
    curPoseOnGrid = m_occumapCv.m_T_global2gridmap * m_curPose;
    std::cout << "curPoseOnGrid: " << curPoseOnGrid << std::endl;

    m_flag_init = false;
    //To make empty space ON robot. (when robot is on the obstacle)
    std::cout << "curPose: " << m_curPose << std::endl;
    curPoseOnGridCV = cv::Point(curPoseOnGrid(0,3),curPoseOnGrid(1,3));
    std::cout << "curPoseOnGridCV: " << curPoseOnGridCV << std::endl;
    cv::circle(cvMapForPlan,curPoseOnGridCV,m_robotParam.robotSize / m_occumapCv.resolution ,255,-1); //Clear robot pose (prevent block)
    mutex_map.unlock();

}

void pathplan::callback_goalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    clearWaypt();
    // std::cout<< "Goal Pose Sub" << std::endl;
    subPath subWaypts;
    mutex_map.lock();
    m_waypts.push_back(std::make_pair(msg->pose.position,subWaypts));
    mutex_map.unlock();

    if(m_waypts.empty()){
        std::cout << "Goal Not Added" << std::endl;
    }
    else{
        std::cout << "Goal Added" << std::endl;
        // mutex_map.lock();
        m_flag_init = false;
        sleep(0.5);
        procMap();
        sleep(0.5);
        m_pub_base_grid->publish(m_occumap);
        mgs04_base_msgs::msg::Map custom_map;
        custom_map.sec = uint8_t(m_occumap.header.stamp.sec);
        custom_map.nanosec = uint8_t(m_occumap.header.stamp.nanosec);
        custom_map.info.map_load_time = m_occumap.info.map_load_time;
        custom_map.info.resolution = m_occumap.info.resolution;
        custom_map.info.width = m_occumap.info.width;
        custom_map.info.height = m_occumap.info.height;
        custom_map.info.origin = m_occumap.info.origin;
        custom_map.grid_map = m_occumap.data;
        m_pub_customMap->publish(custom_map);
        mutex_map.unlock();
    }
    bool flag_rePlaning = false;

    std::cout << "m_flag_init: " << m_flag_init << std::endl;
    
    if(m_waypts.front().second.splinedPath.empty()) flag_rePlaning = true; //When no subpath exist
    else //Subpath exist
    {
        mutex_map.lock();
        m_flag_init = true;
        float distance = distancePoint(curPoseOnGrid,m_waypts.front().second.splinedPath.front()); //Calc distance in pixel
        distance = distance * m_occumapCv.resolution; //Convert to [m] unit
        if(distance < m_successDistance) //If robot is at waypoint
        {
            m_waypts.front().second.splinedPath.erase(m_waypts.front().second.splinedPath.begin());
            if(m_waypts.front().second.splinedPath.empty())
            {
                m_waypts.erase(m_waypts.begin());
                mutex_map.unlock();
                return;
            }
            else{
                flag_rePlaning = true;
            }
        }
        mutex_map.unlock();
    }

    if(flag_rePlaning) // NOT YET PATH PLANNED
    {
        // std::cout << "[callback_crtstate] if(flag_rePlanning)" << std::endl;
        mutex_map.lock();
        m_flag_init = true;
        Eigen::MatrixXf goalOnGrid = m_occumapCv.m_T_global2gridmap * geoPoint2eigen(m_waypts.front().first);
        mutex_map.unlock();
        cv::Point goalOnGridCV = cv::Point(goalOnGrid(0,0),goalOnGrid(1,0));
        m_waypts.front().second = pathplanning(cvMapForPlan,curPoseOnGridCV,goalOnGridCV); // Generate Path
        std::cout << "[PATHPLAN] Generated path length : "<<m_waypts.front().second.splinedPath.size() << std::endl;
        for(int i=0;i<m_waypts.front().second.splinedPath.size();i++)
        {
            std::cout<<m_waypts.front().second.splinedPath.at(i)(0)<<","
                    <<m_waypts.front().second.splinedPath.at(i)(1)<<std::endl;
        }
        if(m_waypts.front().second.splinedPath.size()==0)
        {
            std::cout << "[PATHPLAN] Warning : Cannot found path. Erase current path" << std::endl;
            m_flag_init = false;
            m_waypts.erase(m_waypts.begin());
            return;
        }
    }

    if(m_flag_init){
        // std::cout << "SPINTIMER" << std::endl;
        bool flag_visualizeTraj = true;
        if(m_waypts.empty()){
            std::cout << "[SpinTimer] Empty Goal Pose" << std::endl;
            flag_visualizeTraj = false;
        }
        // else if(m_waypts.front().second.splinedPath.empty()){
        //     std::cout << "[SpinTimer] Empty WayPoints" << std::endl;
        //     flag_visualizeTraj = false;
        // }
        geometry_msgs::msg::Point targetMsg;
        nav_msgs::msg::Path pathMsg;
        // std::cout << flag_visualizeTraj << std::endl;
        // if(!flag_visualizeTraj || m_flag_stop) //Publish target point (next waypoint) For stop
        // {
        //     targetMsg.x = 0;targetMsg.y = 0;targetMsg.z = 0; // if all 0, target point is itself.
        //     m_pub_nextWaypt->publish(targetMsg);
        //     m_pub_splinedWayPt->publish(pathMsg);
        //     std::cout << "[SpinTimer] target point is itself: return" << std::endl;
        //     return;
        // }
        int cnt_seq = 0;
        pathMsg.header.frame_id = "world";
        geometry_msgs::msg::PoseStamped tmp_pose;
        tmp_pose.pose.orientation.w = 1;

        //Add current pose for first segment of line
        tmp_pose.pose.position.x = m_curPose(0,3);
        tmp_pose.pose.position.y = m_curPose(1,3);
        // tmp_pose.header.seq = cnt_seq++;
        pathMsg.poses.push_back(tmp_pose);

        //Add sub trajectories on line.
        mutex_map.lock();
        for(int i=0;i<m_waypts.front().second.splinedPath.size();i++)
        {
            geometry_msgs::msg::PoseStamped tmp_pose;
            Eigen::Vector4f pathOnGlobal;
            pathOnGlobal<<m_waypts.front().second.splinedPath.at(i)(0),m_waypts.front().second.splinedPath.at(i)(1),0,1; // Pixel location of path
            pathOnGlobal = m_occumapCv.m_T_global2gridmap.inverse()*pathOnGlobal; // Convert to global location of path
            if(i==0) // Publish pathfollow signal
            {
                Eigen::Vector4f relPathOnGlobal = m_curPose.inverse() * pathOnGlobal;
                targetMsg.x = relPathOnGlobal(0);
                targetMsg.y = relPathOnGlobal(1);
                m_pub_nextWaypt->publish(targetMsg);
            }

            tmp_pose.pose.position.x = pathOnGlobal(0);
            tmp_pose.pose.position.y = pathOnGlobal(1);
            // tmp_pose.header.seq = cnt_seq++;
            pathMsg.poses.push_back(tmp_pose);
        }
        mutex_map.unlock();
        m_pub_splinedWayPt->publish(pathMsg);

        cnt_seq = 0;
        pcl::PointCloud<pcl::PointXYZRGB> wayptPtcl;
        for(int i=0;i<m_waypts.size();i++)
        {
            cv::Vec3b color = heightcolor(i/(float)m_waypts.size());
            pcl::PointXYZRGB oneWaypt = pcl::PointXYZRGB(color(0), color(1), color(2));
            Eigen::MatrixXf oneWayptOnMap = geoPoint2eigen(m_waypts.at(i).first);
            oneWaypt.x = oneWayptOnMap(0);
            oneWaypt.y = oneWayptOnMap(1);
            oneWaypt.z = oneWayptOnMap(2);
            wayptPtcl.push_back(oneWaypt);
        }
        m_pub_wayPt->publish(cloud2msg(wayptPtcl,"world"));

        // std::cout << "SPINTIMER FINISH" << std::endl;
    }
}


void pathplan::spinTimer(std::shared_ptr<rclcpp::Node> n)
{

    bool flag_visualizeTraj = true;
    if(m_waypts.empty()) flag_visualizeTraj = false;
    else if(m_waypts.front().second.splinedPath.empty()) flag_visualizeTraj = false;
    geometry_msgs::msg::Point targetMsg;
    nav_msgs::msg::Path pathMsg;
    if(!flag_visualizeTraj || m_flag_stop) //Publish target point (next waypoint) For stop
    {
        targetMsg.x = 0;targetMsg.y = 0;targetMsg.z = 0; // if all 0, target point is itself.
        m_pub_nextWaypt->publish(targetMsg);
        m_pub_splinedWayPt->publish(pathMsg);
        return;
    }
    int cnt_seq = 0;
    pathMsg.header.frame_id = "world";
    geometry_msgs::msg::PoseStamped tmp_pose;
    tmp_pose.pose.orientation.w = 1;

    //Add current pose for first segment of line
    tmp_pose.pose.position.x = m_curPose(0,3);
    tmp_pose.pose.position.y = m_curPose(1,3);
    // tmp_pose.header.seq = cnt_seq++;
    pathMsg.poses.push_back(tmp_pose);

    //Add sub trajectories on line.
    for(int i=0;i<m_waypts.front().second.splinedPath.size();i++)
    {
        geometry_msgs::msg::PoseStamped tmp_pose;
        Eigen::Vector4f pathOnGlobal;
        pathOnGlobal<<m_waypts.front().second.splinedPath.at(i)(0),m_waypts.front().second.splinedPath.at(i)(1),0,1; // Pixel location of path
        pathOnGlobal = m_occumapCv.m_T_global2gridmap.inverse()*pathOnGlobal; // Convert to global location of path
        if(i==0) // Publish pathfollow signal
        {
            Eigen::Vector4f relPathOnGlobal = m_curPose.inverse() * pathOnGlobal;
            targetMsg.x = relPathOnGlobal(0);
            targetMsg.y = relPathOnGlobal(1);
            m_pub_nextWaypt->publish(targetMsg);
        }

        tmp_pose.pose.position.x = pathOnGlobal(0);
        tmp_pose.pose.position.y = pathOnGlobal(1);
        // tmp_pose.header.seq = cnt_seq++;
        pathMsg.poses.push_back(tmp_pose);
    }
    m_pub_splinedWayPt->publish(pathMsg);

    cnt_seq = 0;
    pcl::PointCloud<pcl::PointXYZRGB> wayptPtcl;
    for(int i=0;i<m_waypts.size();i++)
    {
        cv::Vec3b color = heightcolor(i/(float)m_waypts.size());
        pcl::PointXYZRGB oneWaypt = pcl::PointXYZRGB(color(0), color(1), color(2));
        Eigen::MatrixXf oneWayptOnMap = geoPoint2eigen(m_waypts.at(i).first);
        oneWaypt.x = oneWayptOnMap(0);
        oneWaypt.y = oneWayptOnMap(1);
        oneWaypt.z = oneWayptOnMap(2);
        wayptPtcl.push_back(oneWaypt);
    }
    m_pub_wayPt->publish(cloud2msg(wayptPtcl,"map"));
}

pathplan::subPath pathplan::pathplanning(cv::Mat map, cv::Point src, cv::Point dst)
{
    subPath resultPP;
    CShortestPP cshortpp;
    iPoint srcPos(src.x,src.y);
    iPoint dstPos(dst.x,dst.y);
    iPointArray arrPath(1000);
    iPointArray pathArrayReal(1000);
    cshortpp.FindPathPyr(&map, map.cols, map.cols, map.rows, srcPos, dstPos, arrPath, pathArrayReal);
    if(arrPath.size()>1)
    {
        for(int i = 1; i < arrPath.size(); i++)
        {
            resultPP.splinedPath.push_back(Eigen::Vector2f(arrPath[i].x,arrPath[i].y));
        }

        for(int j = 1; j < pathArrayReal.size(); j++)
        {
            resultPP.rawPath.push_back(Eigen::Vector2f(pathArrayReal[j].x,pathArrayReal[j].y));
        }
    }

    cshortpp.~CShortestPP();
    return resultPP;
}

void pathplan::callback_crtState(nav_msgs::msg::Odometry::SharedPtr msg)//EDITED_MICROSWARM
{
//    std::cout<<"[PATHPLAN] INIT POSE SUB"<<std::endl;
    mutex_map.lock();
    if(m_occumap.data.size()==0)
    {
           std::cout << "[PATHPLAN] Warning : no map " << std::endl;
        mutex_map.unlock();
        return;
    }
    mutex_map.unlock();
    if(m_waypts.empty())
    {
       std::cout << "[PATHPLAN] Warning : no waypoint " << std::endl;
        return;
    }
    m_flag_stop = false;
    mutex_map.lock();
    cv::Mat cvMapForPlan = m_occumapCv.cvMapForPlan.clone();

    Eigen::MatrixXf curPose = geoPose2eigen(msg->pose.pose);
    Eigen::MatrixXf curPoseOnGrid = m_occumapCv.m_T_global2gridmap * curPose;
    m_curPose = curPose;
    //To make empty space ON robot. (when robot is on the obstacle)
    // std::cout << "curPose: " << curPose << std::endl;
    cv::Point curPoseOnGridCV = cv::Point(curPoseOnGrid(0,3),curPoseOnGrid(1,3));
    // std::cout << "curPoseOnGrid: " << curPoseOnGridCV << std::endl;
    cv::circle(cvMapForPlan,curPoseOnGridCV,m_robotParam.robotSize / m_occumapCv.resolution ,255,-1); //Clear robot pose (prevent block)
    mutex_map.unlock();

    bool flag_rePlaning = false;

    // std::cout << "m_flag_init: " << m_flag_init << std::endl;

    // spinTimer Added EDITED_MICROSWARM
    if(m_flag_init){
        // std::cout << "SPINTIMER" << std::endl;
    bool flag_visualizeTraj = true;
    if(m_waypts.empty()){
        std::cout << "[SpinTimer] Empty Goal Pose" << std::endl;
        flag_visualizeTraj = false;
    }
    else if(m_waypts.front().second.splinedPath.empty()){
        std::cout << "[SpinTimer] Empty WayPoints" << std::endl;
        flag_visualizeTraj = false;
    }
    geometry_msgs::msg::Point targetMsg;
    nav_msgs::msg::Path pathMsg;
    // std::cout << flag_visualizeTraj << std::endl;
    if(!flag_visualizeTraj || m_flag_stop) //Publish target point (next waypoint) For stop
    {
        targetMsg.x = 0;targetMsg.y = 0;targetMsg.z = 0; // if all 0, target point is itself.
        m_pub_nextWaypt->publish(targetMsg);
        m_pub_splinedWayPt->publish(pathMsg);
        std::cout << "[SpinTimer] target point is itself: return" << std::endl;
        return;
    }
    int cnt_seq = 0;
    pathMsg.header.frame_id = "world";
    geometry_msgs::msg::PoseStamped tmp_pose;
    tmp_pose.pose.orientation.w = 1;

    //Add current pose for first segment of line
    tmp_pose.pose.position.x = m_curPose(0,3);
    tmp_pose.pose.position.y = m_curPose(1,3);
    // tmp_pose.header.seq = cnt_seq++;
    pathMsg.poses.push_back(tmp_pose);

    //Add sub trajectories on line.
    mutex_map.lock();
    for(int i=0;i<m_waypts.front().second.splinedPath.size();i++)
    {
        geometry_msgs::msg::PoseStamped tmp_pose;
        Eigen::Vector4f pathOnGlobal;
        pathOnGlobal<<m_waypts.front().second.splinedPath.at(i)(0),m_waypts.front().second.splinedPath.at(i)(1),0,1; // Pixel location of path
        pathOnGlobal = m_occumapCv.m_T_global2gridmap.inverse()*pathOnGlobal; // Convert to global location of path
        if(i==0) // Publish pathfollow signal
        {
            Eigen::Vector4f relPathOnGlobal = m_curPose.inverse() * pathOnGlobal;
            targetMsg.x = relPathOnGlobal(0);
            targetMsg.y = relPathOnGlobal(1);
            m_pub_nextWaypt->publish(targetMsg);
        }

        tmp_pose.pose.position.x = pathOnGlobal(0);
        tmp_pose.pose.position.y = pathOnGlobal(1);
        // tmp_pose.header.seq = cnt_seq++;
        pathMsg.poses.push_back(tmp_pose);
    }
    mutex_map.unlock();
    m_pub_splinedWayPt->publish(pathMsg);

    cnt_seq = 0;
    pcl::PointCloud<pcl::PointXYZRGB> wayptPtcl;
    for(int i=0;i<m_waypts.size();i++)
    {
        cv::Vec3b color = heightcolor(i/(float)m_waypts.size());
        pcl::PointXYZRGB oneWaypt = pcl::PointXYZRGB(color(0), color(1), color(2));
        Eigen::MatrixXf oneWayptOnMap = geoPoint2eigen(m_waypts.at(i).first);
        oneWaypt.x = oneWayptOnMap(0);
        oneWaypt.y = oneWayptOnMap(1);
        oneWaypt.z = oneWayptOnMap(2);
        wayptPtcl.push_back(oneWaypt);
    }
    m_pub_wayPt->publish(cloud2msg(wayptPtcl,"world"));

    // std::cout << "SPINTIMER FINISH" << std::endl;
    }
    
    if(m_waypts.front().second.splinedPath.empty()) flag_rePlaning = true; //When no subpath exist
    else //Subpath exist
    {
        mutex_map.lock();
        m_flag_init = true;
        float distance = distancePoint(curPoseOnGrid,m_waypts.front().second.splinedPath.front()); //Calc distance in pixel
        distance = distance * m_occumapCv.resolution; //Convert to [m] unit
        if(distance < m_successDistance) //If robot is at waypoint
        {
            m_waypts.front().second.splinedPath.erase(m_waypts.front().second.splinedPath.begin());
            if(m_waypts.front().second.splinedPath.empty())
            {
                m_waypts.erase(m_waypts.begin());
                mutex_map.unlock();
                return;
            }
            else{
                flag_rePlaning = true;
            }
        }
        mutex_map.unlock();
    }
    if(flag_rePlaning) // NOT YET PATH PLANNED
    {
        // std::cout << "[callback_crtstate] if(flag_rePlanning)" << std::endl;
        mutex_map.lock();
        m_flag_init = true;
        Eigen::MatrixXf goalOnGrid = m_occumapCv.m_T_global2gridmap * geoPoint2eigen(m_waypts.front().first);
        mutex_map.unlock();
        cv::Point goalOnGridCV = cv::Point(goalOnGrid(0,0),goalOnGrid(1,0));
        m_waypts.front().second = pathplanning(cvMapForPlan,curPoseOnGridCV,goalOnGridCV); // Generate Path
        std::cout << "[PATHPLAN] Generated path length : "<<m_waypts.front().second.splinedPath.size() << std::endl;
        for(int i=0;i<m_waypts.front().second.splinedPath.size();i++)
        {
            std::cout<<m_waypts.front().second.splinedPath.at(i)(0)<<","
                    <<m_waypts.front().second.splinedPath.at(i)(1)<<std::endl;
        }
        if(m_waypts.front().second.splinedPath.size()==0)
        {
            std::cout << "[PATHPLAN] Warning : Cannot found path. Erase current path" << std::endl;
            m_flag_init = false;
            m_waypts.erase(m_waypts.begin());
            return;
        }
    }
}

float pathplan::distancePoint(Eigen::MatrixXf pose,Eigen::Vector2f wayPt)
{
    return sqrt(pow(pose(0,3)-wayPt(0),2)+pow(pose(1,3)-wayPt(1),2));
}

void pathplan::procMap()
{
    // std::cout << "PROCMAP IN" << std::endl;
    m_occumapCv.resolution = m_occumap.info.resolution;
    //m_occumapCv.m_T_global2gridmap = Eigen::Matrix4f::Identity();
    //m_occumapCv.m_T_global2gridmap(0,0) = 1./m_occumap.info.resolution;
    //m_occumapCv.m_T_global2gridmap(1,1) = 1./m_occumap.info.resolution;
    //m_occumapCv.m_T_global2gridmap(0,3) = -m_occumap.info.origin.position.x / m_occumap.info.resolution;
    //m_occumapCv.m_T_global2gridmap(1,3) = -m_occumap.info.origin.position.y / m_occumap.info.resolution;

    m_occumapCv.m_T_global2gridmap = geoPose2eigen(m_occumap.info.origin).inverse();
    m_occumapCv.m_T_global2gridmap.topRows(3) /= m_occumap.info.resolution;

    // std::cout<< "[Procmap] occumapCv Declared" << std::endl;

    cv::Mat mapIn = occumap2cvimg(m_occumap);

    cv::Mat element_ero = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                             cv::Size( (m_robotParam.robotSize - m_robotParam.robotMargin) / m_occumap.info.resolution + 1, (m_robotParam.robotSize - m_robotParam.robotMargin) / m_occumap.info.resolution + 1 ),
                                             cv::Point( (m_robotParam.robotSize - m_robotParam.robotMargin) / m_occumap.info.resolution, (m_robotParam.robotSize - m_robotParam.robotMargin) / m_occumap.info.resolution ) );
    cv::erode( mapIn, m_occumapCv.cvMapForCheck, element_ero );
    element_ero = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                             cv::Size( m_robotParam.robotSize / m_occumap.info.resolution + 1, m_robotParam.robotSize / m_occumap.info.resolution + 1 ),
                                            cv::Point( m_robotParam.robotSize / m_occumap.info.resolution, m_robotParam.robotSize / m_occumap.info.resolution ) );
    cv::erode( mapIn, m_occumapCv.cvMapForPlan, element_ero );

    if(m_useObject) {
    cv::erode( mapIn, mapIn, element_ero );
    cv::Mat image(mapIn.rows,mapIn.cols,CV_8UC1,cv::Scalar(255));

    // std::cout << "[Procmap] Erosion FINISHED" << std::endl;
    

    for(size_t i=0;i<m_groups.size();i++)
    {
        std::cout << "[Procmap] Group? IN " << m_groups[i].size()<<" "<<m_groups_class[i]<<std::endl;
        if(m_groups[i].size()==1 && m_groups_class[i]==0) //IF ONE-LEG CHAIR
        {
            Eigen::MatrixXf objOnGrid = m_occumapCv.m_T_global2gridmap * geoPoint2eigen(m_groups_pose[i]);
            cv::circle(image,cv::Point(objOnGrid(0,0),objOnGrid(1,0)),0.5/m_occumap.info.resolution,0,-1);
            continue;
        }
        else if(m_groups_class[i]==3) //IF PERSON
        {
            continue;
        }
        std::vector<cv::Point> objPoints;
        for(size_t j=0;j<m_groups[i].size();j++)
        {
            Eigen::MatrixXf objOnGrid = m_occumapCv.m_T_global2gridmap * geoPoint2eigen(m_objects[m_groups[i][j]].location);
            objPoints.push_back(cv::Point(objOnGrid(0,0),objOnGrid(1,0)));
        }
        std::vector<cv::Point> hullPoints;
        cv::convexHull(objPoints,hullPoints);
        cv::fillConvexPoly(image,hullPoints,cv::Scalar(0));
    }
    
    cv::erode( image, image, element_ero );
    cv::addWeighted(mapIn,0.5,image,0.5,0.0,m_occumapCv.cvMapForPlan);
    threshold(m_occumapCv.cvMapForPlan,m_occumapCv.cvMapForPlan, 200, 255, cv::THRESH_BINARY);
    // std::cout << "[Procmap] Final Erosion and Weighting Finished" << std::endl;
    }

    // else m_occumapCv.cvMapForPlan = mapIn.clone();

    // std::cout << "[Procmap] Procmap Finished" << std::endl;
}

void pathplan::callback_goal(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    clearWaypt();
    // std::cout<< "Goal Pose Sub" << std::endl;
    subPath subWaypts;
    mutex_map.lock();
    m_waypts.push_back(std::make_pair(msg->pose.position,subWaypts));
    mutex_map.unlock();

    if(m_waypts.empty()){
        std::cout << "Goal Not Added" << std::endl;
    }
    else{
        std::cout << "Goal Added" << std::endl;
        mutex_map.lock();
        m_flag_init = false;
        sleep(0.5);
        procMap();
        sleep(0.5);
        m_pub_base_grid->publish(m_occumap);
        mgs04_base_msgs::msg::Map custom_map;
        custom_map.sec = uint8_t(m_occumap.header.stamp.sec);
        custom_map.nanosec = uint8_t(m_occumap.header.stamp.nanosec);
        custom_map.info.map_load_time = m_occumap.info.map_load_time;
        custom_map.info.resolution = m_occumap.info.resolution;
        custom_map.info.width = m_occumap.info.width;
        custom_map.info.height = m_occumap.info.height;
        custom_map.info.origin = m_occumap.info.origin;
        custom_map.grid_map = m_occumap.data;
        m_pub_customMap->publish(custom_map);
        mutex_map.unlock();
    }
    
}

void pathplan::callback_waypt(geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    subPath subWaypts;
    m_waypts.push_back(std::make_pair(msg->point,subWaypts));
}

void pathplan::callback_2dmap(nav_msgs::msg::OccupancyGrid::SharedPtr msg)//EDITED_MICROSWARM
{
    if(msg->data.size()==0){
        std::cout << "callback_2dmap: No Map" << std::endl;
        return;
    }

    // std::cout << "2dmap callback" << std::endl;
    mutex_map.lock();
    m_occumap.header = msg->header;
    m_occumap.info = msg->info;
    m_occumap.data = msg->data;
    
    // int VISUALIZATION_SHIFT_X = 0.5*m_occumap.info.width/m_occumap.info.resolution;
    // int VISUALIZATION_SHIFT_Y = 0.5*m_occumap.info.height/m_occumap.info.resolution;
    // m_occumap.info.origin.position.x -= VISUALIZATION_SHIFT_X;
    // m_occumap.info.origin.position.y -= VISUALIZATION_SHIFT_Y;
    m_occumap.info.origin = msg->info.origin;
    // m_occumap.info.origin.orientation.x = 0;
    // m_occumap.info.origin.orientation.y = 0;
    // m_occumap.info.origin.orientation.z = 0; //-0.7071;
    // m_occumap.info.origin.orientation.w = 1; //0.7071;

    // std::cout << "[callback_2dmap] map origin: x " << m_occumap.info.origin.position.x  << " y " << m_occumap.info.origin.position.y << std::endl;
    mutex_map.unlock();
    if(map_callback_count == 10){
        std::cout << "[callback_2dmap] Procmap IN" << std::endl;
        mutex_map.lock();
        sleep(0.5);
        map_callback_count = 0;
        procMap();
        sleep(0.5);
        m_pub_base_grid->publish(m_occumap);
        mgs04_base_msgs::msg::Map custom_map;
        custom_map.sec = uint8_t(m_occumap.header.stamp.sec);
        custom_map.nanosec = uint8_t(m_occumap.header.stamp.nanosec);
        custom_map.info.map_load_time = m_occumap.info.map_load_time;
        custom_map.info.resolution = m_occumap.info.resolution;
        custom_map.info.width = m_occumap.info.width;
        custom_map.info.height = m_occumap.info.height;
        custom_map.info.origin = m_occumap.info.origin;
        custom_map.grid_map = m_occumap.data;
        m_pub_customMap->publish(custom_map);
        mutex_map.unlock();
        // std::cout << "[callback_2dmap] Procmap OUT" << std::endl;
    }
    else{
        map_callback_count++;
    }
    // std::cout << "[PATHPLAN] PROCESS MAP DONE" << std::endl;
}

void pathplan::clearWaypt()
{
    m_waypts.clear();
}


void pathplan::getParam(rclcpp::Node::SharedPtr n)
{
    n->get_parameter("movingControl/robot_size",m_robotParam.robotSize);
    n->get_parameter("movingControl/robot_margin",m_robotParam.robotMargin);
    n->get_parameter("movingControl/success_distance",m_successDistance);
    n->get_parameter("movingControl/safearea_side",m_safeAreaSide);
    n->get_parameter("movingControl/safearea_front",m_safeAreaFront);
    n->get_parameter("movingControl/safearea_frontmin",m_safeAreaFrontMin);

    n->get_parameter<std::string>("movingControl/objectmapPath",m_param_mapdir);

    n->get_parameter("movingControl/useObject",m_useObject);

    n->get_parameter("neuralNet/object/name",m_classes);
    m_classes.push_back(("none"));
    std::vector<std::string> nameColor;
    n->get_parameter("neuralNet/object/color",nameColor);
    for(size_t i=0;i<nameColor.size();i++)
    {
        Eigen::Vector3i colorVal;
        std::stringstream colorStr(nameColor[i]);
        colorStr>>colorVal[0];
        colorStr>>colorVal[1];
        colorStr>>colorVal[2];
        m_colors.push_back(colorVal);
    }
    { //for non-class
        Eigen::Vector3i colorVal;
        colorVal<<255,255,255;
        m_colors.push_back(colorVal);
    }
}

void pathplan::pubBase(std::shared_ptr<rclcpp::Node> n)
{
//    m_occumap.header.frame_id="map";

    m_pub_base_grid->publish(m_occumap);

    visualization_msgs::msg::Marker markerPub;
    markerPub.scale.x = 0.1;
    markerPub.scale.y = 0.1;
    markerPub.scale.z = 0.1;
    markerPub.pose.orientation.w = 1;
    markerPub.header.frame_id = "world";

    // zinuok: this may cause error, because Foxy uses incomplete time method
    markerPub.header.stamp = n->now();

    //REMOVE ALL MARKERS
    markerPub.action = visualization_msgs::msg::Marker::DELETEALL;
    m_pub_base_marker->publish(markerPub);
    //OBJECT MARKER ( SPHERE )
    markerPub.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    markerPub.action = visualization_msgs::msg::Marker::ADD;
    markerPub.id = 0;
    visualization_msgs::msg::Marker markerPubTxt = markerPub;
    markerPubTxt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    markerPubTxt.scale.z = 0.5;
    for(int i=0;i<m_objects.size();i++)
    {
        geometry_msgs::msg::Point tmpPt;
        tmpPt = m_objects[i].location;

        std_msgs::msg::ColorRGBA tmpCl;
        tmpCl.a = 1;
        tmpCl.b = m_colors[m_objects[i].classIdx](2)/255.0;
        tmpCl.g = m_colors[m_objects[i].classIdx](1)/255.0;
        tmpCl.r = m_colors[m_objects[i].classIdx](0)/255.0;
        markerPub.points.push_back(tmpPt);
        markerPub.colors.push_back(tmpCl);
    }
    for(int i=0;i<m_groups_pose.size();i++)
    {
        geometry_msgs::msg::Point tmpPt;
        tmpPt = m_groups_pose[i];
        tmpPt.z += 0.1;

        std::ostringstream strI;
        strI<<i;
        markerPubTxt.text = strI.str();

        markerPubTxt.id = i+1;
        markerPubTxt.pose.position = tmpPt;

        std_msgs::msg::ColorRGBA tmpCl;
        tmpCl.a = 1;
        tmpCl.b = m_colors[m_groups_class[i]](2)/255.0;
        tmpCl.g = m_colors[m_groups_class[i]](1)/255.0;
        tmpCl.r = m_colors[m_groups_class[i]](0)/255.0;
        markerPubTxt.color = tmpCl;
        m_pub_base_marker->publish(markerPubTxt);
    }
    m_pub_base_marker->publish(markerPub);
    markerPub.points.clear();
    markerPub.colors.clear();
}
