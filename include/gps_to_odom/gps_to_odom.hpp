#ifndef GPS_TO_ODOM_GPS_TO_ODOM_HPP
#define GPS_TO_ODOM_GPS_TO_ODOM_HPP

#include "ros/ros.h"
#include "ros/time.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TransformStamped.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <geodesy/utm.h>
#include <nav_msgs/Odometry.h>
#include <deque>

#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>



#include <Eigen/Dense>
#include <cmath>

using namespace std;

struct Piont
{
    Eigen::Vector3d pos;//位置
    Eigen::Matrix3d orien;//姿态 旋转矩阵表示
    Eigen::Vector3d w;//角速度
    Eigen::Vector3d v;//线速度
};

//imu处理
class GpsOdom
{
private:
    ros::NodeHandle nhg;
    ros::Subscriber navsatsub;
    ros::Publisher odompub;
    nav_msgs::Odometry odom;
    nav_msgs::Odometry odomlast;
    nav_msgs::Odometry odomcurr;
    Piont point;
    boost::optional<Eigen::Vector3d> zero_utm;
    
    std::vector<nav_msgs::Odometry> gpsvector;
    std::deque<nav_msgs::Odometry> gpsdeque;
    geometry_msgs::PoseStamped gpscov; 
    ros::Publisher gpscovpub;
    //ros::WallTimer gps_juge_timer;

    ros::Subscriber pathsub;
    ros::Publisher TransformStampedpub;
    
public:
    //! Constructor.
    GpsOdom(ros::NodeHandle& nh);
    //! Destructor.
    ~GpsOdom();

    void NavsatCallback(const sensor_msgs::NavSatFixConstPtr& navsat_msg);
    void GpsCallback(const geographic_msgs::GeoPointStampedPtr& gps_msg);
    void PathCallback(const nav_msgs::Path::ConstPtr& path_msg);
    //void gps_juge_callback(const ros::WallTimerEvent& event);
};

GpsOdom::GpsOdom(ros::NodeHandle& nh):nhg(nh) {
  //参数初始化

  pathsub = nhg.subscribe("/trajectory_viz/trajectory", 32, &GpsOdom::PathCallback, this);
  TransformStampedpub = nh.advertise<tf2_msgs::TFMessage>("/tfs", 50);

  navsatsub = nhg.subscribe("/gps/fix", 32, &GpsOdom::NavsatCallback, this); //gps/fix
  odompub = nhg.advertise<nav_msgs::Odometry>("gps_odom", 32);
  gpscovpub = nhg.advertise<geometry_msgs::PoseStamped>("gps_cov", 32);
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  
  Eigen::Vector3d zero(0, 0, 0);
  point.pos = zero;
  point.orien = Eigen::Matrix3d::Identity();
  point.v = zero;
  point.w = zero;
  //gps_juge_timer = nh.createWallTimer(ros::WallDuration(1), &GpsOdom::gps_juge_callback, this);
}

void GpsOdom::PathCallback(const nav_msgs::Path::ConstPtr& path_msg) {
    for (int i = 0; i < path_msg->poses.size(); i++)
  {
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header = path_msg->header;
    transform_stamped.child_frame_id = "child_frame"; // 设置子坐标系
    transform_stamped.transform.translation.x = path_msg->poses[i].pose.position.x;
    transform_stamped.transform.translation.y = path_msg->poses[i].pose.position.y;
    transform_stamped.transform.translation.z = path_msg->poses[i].pose.position.z;
    transform_stamped.transform.rotation = path_msg->poses[i].pose.orientation;
    // Do something with the transform_stamped message here
    tf2_msgs::TFMessage tf_msg;
    tf_msg.transforms.push_back(transform_stamped);
    TransformStampedpub.publish(tf_msg);
  }
}

void GpsOdom::NavsatCallback(const sensor_msgs::NavSatFixConstPtr& navsat_msg) {
    geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
    gps_msg->header = navsat_msg->header;
    gps_msg->position.latitude = navsat_msg->latitude;
    gps_msg->position.longitude = navsat_msg->longitude;
    gps_msg->position.altitude = navsat_msg->altitude;
    GpsCallback(gps_msg);
}

void GpsOdom::GpsCallback(const geographic_msgs::GeoPointStampedPtr& gps_msg) {
    
    gps_msg->header.stamp += ros::Duration(0.0);
    geographic_msgs::GeoPoint gpspoint;
    gpspoint.altitude = gps_msg->position.altitude;
    gpspoint.latitude = gps_msg->position.latitude;
    gpspoint.longitude = gps_msg->position.longitude;
    geodesy::UTMPoint utm;//创建一个utm坐标
    geodesy::fromMsg(gpspoint, utm);//把一帧gps消息转化为utm格式
    Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);//创建一个xyz坐标向量

    //有局部坐标经纬度的时候启用
    // geographic_msgs::GeoPoint gpspoint0;
    // gpspoint0.altitude = 局部原点高度;
    // gpspoint0.latitude = xxx;
    // gpspoint0.longitude = xxx;
    // geodesy::UTMPoint utm0;
    // geodesy::fromMsg(gpspoint0, utm0);
    // Eigen::Vector3d xyz0(utm0.easting, utm0.northing, utm0.altitude);

      if(!zero_utm) {//如果第一帧
        zero_utm = xyz;//把第一帧设置为原点
        //zero_utm = xyz0；//有局部坐标经纬度的时候启用
      }
      xyz -= (*zero_utm);

    odom.header.seq = gps_msg->header.seq;
    odom.header.stamp = gps_msg->header.stamp;
    odom.pose.pose.position.x = xyz(0);
    odom.pose.pose.position.y = xyz(1);
    odom.pose.pose.position.z = xyz(2);

    //离散程度判断画图
    gpsdeque.push_back(odom);

    if(gpsdeque.size() >= 2){
    
      odomlast = gpsdeque.front();
      odomcurr = gpsdeque.back();

      gpscov.header.seq = gps_msg->header.seq;
      gpscov.header.stamp = gps_msg->header.stamp;
      gpscov.pose.position.x = odomcurr.pose.pose.position.x - odomlast.pose.pose.position.x;
      gpscov.pose.position.y = odomcurr.pose.pose.position.y - odomlast.pose.pose.position.y;
      gpscov.pose.position.z = odomcurr.pose.pose.position.z - odomlast.pose.pose.position.z;
      //odom.pose.covariance(1,1) = odomcurr.pose.pose.position.x - odomlast.pose.pose.position.x;
      gpsdeque.pop_front();
    }

    odompub.publish(odom);
    gpscovpub.publish(gpscov);
}

// void GpsOdom::gps_juge_callback(const ros::WallTimerEvent& event) {
//    //离散程度判断画图
//     gpsQueue.push_back(odom);


// }


#endif
