# 修改内容： 传递第一个LLA信息
## 修改GPS阅读器节点
<img src="images\00.png" alt="Terminator" width="100%"> 

在上图位置添加代码：
```cpp
navfixMsg.position_covariance[0] = -1; //frist latitude  
navfixMsg.position_covariance[1] = -1; //first longitude  
navfixMsg.position_covariance[2] = -1; //frist altitude
```
<img src="images\01.png" alt="Terminator" width="100%"> 

在上图位置添加代码：
```cpp
        // record first LLA
        if (navfixMsg.position_covariance[0] == -1 &&
            navfixMsg.position_covariance[1] == -1 &&
            navfixMsg.position_covariance[2] == -1)
        {
            navfixMsg.position_covariance[0] = gps_latitude;  //frist latitude
            navfixMsg.position_covariance[1] = gps_longitude; //first longitude
            navfixMsg.position_covariance[2] = gps_altitude;  //frist altitude
            std::cout << "-------------first LLA--------------------" << std::endl;
            std::cout << "frist latitude:" << navfixMsg.position_covariance[0] << std::endl;
            std::cout << "first longitud:" << navfixMsg.position_covariance[1] << std::endl;
            std::cout << "frist altitude:" << navfixMsg.position_covariance[2] << std::endl;
            std::cout << "-------------first LLA--------------------" << std::endl;
        }
```
## 修改GPS转当地直角坐标系节点
<img src="images\02.png" alt="Terminator" width="100%"> 

如上图替换整个gpsCallBack:
代码为：\\
``` cpp
void gpsCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
    static GeographicLib::LocalCartesian local_cartesian(msg->position_covariance[0],  //frist latitude
                                                         msg->position_covariance[1],  //first longitude
                                                         msg->position_covariance[2]); //frist altitude

    deque_gps.push_back(*msg);

    while (deque_imu.size() > 0 &&
           deque_gps.size() > 0 &&
           deque_gps.front().header.stamp <= deque_imu.back().header.stamp &&
           deque_gps.front().header.stamp >= deque_imu.front().header.stamp)
    {
        //
        double xx, yy, zz;
        local_cartesian.Forward(deque_gps.front().latitude,
                                deque_gps.front().longitude,
                                deque_gps.front().altitude,
                                xx, yy, zz);

        //
        Eigen::Quaterniond q_ENU_roslidar;
        int index_closest = 0;
        for (int i = 0; i < deque_imu.size(); i++)
        {
            if (fabs((deque_imu.at(i).header.stamp - deque_gps.front().header.stamp).toSec()) <
                fabs((deque_imu.at(index_closest).header.stamp - deque_gps.front().header.stamp).toSec()))
            {
                index_closest = i;
            }
        }
        q_ENU_roslidar.w() = deque_imu.at(index_closest).orientation.w;
        q_ENU_roslidar.x() = deque_imu.at(index_closest).orientation.x;
        q_ENU_roslidar.y() = deque_imu.at(index_closest).orientation.y;
        q_ENU_roslidar.z() = deque_imu.at(index_closest).orientation.z;

        //
        Eigen::Isometry3d T_ENU_gps;
        T_ENU_gps.setIdentity();
        T_ENU_gps.matrix().block<3, 3>(0, 0) = q_ENU_roslidar *
                                               T_rosbody_roslidar.rotation().transpose() *
                                               T_rosbody_gps.rotation();
        T_ENU_gps.matrix().block<3, 1>(0, 3) = Eigen::Vector3d(xx,
                                                               yy,
                                                               zz);

        //
        Eigen::Isometry3d T_ENU_roslidar;
        T_ENU_roslidar.setIdentity();
        T_ENU_roslidar = T_ENU_gps * T_rosbody_gps.inverse() * T_rosbody_roslidar;

        //
        nav_msgs::Odometry odomMsg_temp;
        odomMsg_temp.header.stamp = deque_gps.front().header.stamp;
        odomMsg_temp.header.frame_id = "map"; //ENU
        odomMsg_temp.child_frame_id = "base_link";
        odomMsg_temp.pose.pose.position.x = T_ENU_roslidar.translation().x();
        odomMsg_temp.pose.pose.position.y = T_ENU_roslidar.translation().y();
        odomMsg_temp.pose.pose.position.z = T_ENU_roslidar.translation().z();
        odomMsg_temp.pose.pose.orientation.w = q_ENU_roslidar.w();
        odomMsg_temp.pose.pose.orientation.x = q_ENU_roslidar.x();
        odomMsg_temp.pose.pose.orientation.y = q_ENU_roslidar.y();
        odomMsg_temp.pose.pose.orientation.z = q_ENU_roslidar.z();

        if (deque_gps.front().latitude == -1 ||
            deque_gps.front().longitude == -1 ||
            deque_gps.front().altitude == -1) // GPS signal is shielded
        {
            odomMsg_temp.pose.covariance.fill(100000);
        }
        else //   covariance is unknown
        {
            odomMsg_temp.pose.covariance.fill(1);
        }

        publisher_localCartesian.publish(odomMsg_temp);

        // discard uesed message
        deque_gps.pop_front();
    }

    if (deque_imu.size() > 0 &&
        deque_gps.size() > 0 &&
        deque_gps.front().header.stamp < deque_imu.front().header.stamp)
    {
        deque_gps.pop_front();
    }
}
```
## 修改回答节点
<img src="images\03.png" alt="Terminator" width="100%">

替换整个gpsCallback函数：
``` cpp
void gpsCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
    if (isInit_local_cartesian == false)
    {
        local_cartesian.Reset(msg->position_covariance[0],
                              msg->position_covariance[1],
                              msg->position_covariance[2]);
        isInit_local_cartesian = true;
        //
        std::cout << "-------------local_cartesian initialization--------------------" << std::endl;
        std::cout << " local_cartesian.LatitudeOrigin():" << local_cartesian.LatitudeOrigin() << std::endl;
        std::cout << "local_cartesian.LongitudeOrigin():" << local_cartesian.LongitudeOrigin() << std::endl;
        std::cout << "   local_cartesian.HeightOrigin():" << local_cartesian.HeightOrigin() << std::endl;
        std::cout << "-------------local_cartesian initialization--------------------" << std::endl;
    }
}
```


