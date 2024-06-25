/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include<opencv2/core/core.hpp>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud.h>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM3::System* mpSLAM;

};

void pubTF(const Sophus::SE3f& Tcw, const std_msgs::Header &header);
void pub_pose(const Sophus::SE3f& Twc, const std_msgs::Header &header);
void pub_pointcloud(const Sophus::SE3f& Twc, const void* depth, const std_msgs::Header &header);

ros::Publisher pub_odometry;
// sensor_msgs::PointCloud point_cloud;
sensor_msgs::PointCloud point_cloud_dense;
ros::Publisher pub_point_cloud2;
ros::Publisher pub_point_cloud_dense;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 100);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));
    pub_odometry = nh.advertise<nav_msgs::Odometry>("/ORB_SLAM3/odometry", 1000);
    pub_point_cloud2  = nh.advertise<sensor_msgs::PointCloud2>("/ORB_SLAM3/points_filtered2", 1000);
    pub_point_cloud_dense = nh.advertise<sensor_msgs::PointCloud>("/ORB_SLAM3/point_cloud_dense", 1000);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    Sophus::SE3f Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    Sophus::SE3f Twc = Tcw.inverse();
    pubTF(Twc, msgRGB->header);
    pub_pose(Twc, msgRGB->header);
    pub_pointcloud(Tcw.inverse(), msgD->data.data(), msgRGB->header);
}

void pubTF(const Sophus::SE3f& Twc, const std_msgs::Header &header)
{
	static tf::TransformBroadcaster br;
    tf::Transform transform;
	tf::Quaternion q;

	Eigen::Quaternionf q_eigen(Twc.rotationMatrix()); // Rwc
    Eigen::Vector3f twc = Twc.translation();
    transform.setOrigin(tf::Vector3(twc(0), twc(1), twc(2))); // twc
	q.setW(q_eigen.w());
    q.setX(q_eigen.x());
    q.setY(q_eigen.y());
    q.setZ(q_eigen.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "slam", "cam"));
      
	// printf("trans : %1.4f, %1.4f, %1.4f; rot : %1.4f, %1.4f, %1.4f, %1.4f\n", pos(0), pos(1), pos(2), q_eigen.w(), q_eigen.x(), q_eigen.y(), q_eigen.z());
    tf::Transform transform_world;
	tf::Quaternion q_world(-0.707f, 0.0, 0.0, 0.707f);
    transform_world.setOrigin(tf::Vector3(0,0,0));
    transform_world.setRotation(q_world);
    br.sendTransform(tf::StampedTransform(transform_world, header.stamp, "world", "slam"));

    tf::Transform transform_body;
	tf::Quaternion q_body(0,0,0,1.0f);
    transform_body.setOrigin(tf::Vector3(0,0,-0.2));
    transform_body.setRotation(q_body);
    br.sendTransform(tf::StampedTransform(transform_body, header.stamp, "cam", "body"));

}

void pub_pose(const Sophus::SE3f& Twc, const std_msgs::Header &header)
{
    nav_msgs::Odometry odometry;
    odometry.header.stamp = header.stamp;
    odometry.header.frame_id = "slam";
    odometry.child_frame_id = "slam";

    // Tcw.rotationMatrix();
    auto qwc = Twc.unit_quaternion();

    Eigen::Vector3d pos(Twc.translation()(0), Twc.translation()(1), Twc.translation()(2));  // tcw

    // Eigen::Quaterniond tmp_Q;
    // tmp_Q = Eigen::Quaterniond(R);
    odometry.pose.pose.position.x = pos.x();
    odometry.pose.pose.position.y = pos.y();
    odometry.pose.pose.position.z = pos.z();
    odometry.pose.pose.orientation.x = qwc.x();
    odometry.pose.pose.orientation.y = qwc.y();
    odometry.pose.pose.orientation.z = qwc.z();
    odometry.pose.pose.orientation.w = qwc.w();
    // odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
    // odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
    // odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
    pub_odometry.publish(odometry);

	// static tf::TransformBroadcaster br;
    // tf::Transform transform;
    // tf::Quaternion q;

    // transform.setOrigin(tf::Vector3(pos(0),
    //                                 pos(1),
    //                                 pos(2)));
	// q.setW(qwc.w());
    // q.setX(qwc.x());
    // q.setY(qwc.y());
    // q.setZ(qwc.z());
    // transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, header.stamp, "slam", "body"));

	// // printf("trans : %1.4f, %1.4f, %1.4f; rot : %1.4f, %1.4f, %1.4f, %1.4f\n", pos(0), pos(1), pos(2), q_eigen.w(), q_eigen.x(), q_eigen.y(), q_eigen.z());
    // tf::Transform transform_world;
	// tf::Quaternion q_world(-0.707f, 0.0, 0.0, 0.707f);
    // transform_world.setOrigin(tf::Vector3(0,0,0));
    // transform_world.setRotation(q_world);
    // br.sendTransform(tf::StampedTransform(transform_world, header.stamp, "world", "slam"));
}

void pub_pointcloud(const Sophus::SE3f& Twc, const void* depth, const std_msgs::Header &header)
{
    auto qwc = Twc.unit_quaternion();
    Eigen::Vector3d twc(Twc.translation()(0), Twc.translation()(1), Twc.translation()(2));  // tcw
	// Eigen::Quaterniond q(R.transpose());
    Eigen::Matrix3f Rwcf = Twc.rotationMatrix().matrix();
    Eigen::Matrix3d Rwc = Rwcf.cast<double>();
    // Eigen::Matrix3d Rcw = Rwc.transpose();

#define USE_DATASET 0
#if USE_DATASET
		float fx = 535.4f; // fx
		float cx = 320.1f; // Cx
		float fy = 539.2f; // fy
		float cy = 247.6f; // Cy
        float scale = 5000;
#else
		float fx = 325.0188f; // fx
		float cx = 316.7723f; // Cx
		float fy = 325.0188f; // fy
		float cy = 180.9843f; // Cy
        float scale = 1000;
#endif
    int height = 360, width = 640;
    int numpoints = height*width;
    point_cloud_dense.points.clear();
    point_cloud_dense.points.reserve(height*width);

    sensor_msgs::PointCloud2 point_cloud2;
    sensor_msgs::PointCloud2Modifier modifier(point_cloud2);
    modifier.setPointCloud2FieldsByString(1,"xyz");
    modifier.resize(numpoints);
    point_cloud2.width  = numpoints;
    point_cloud2.height = 1;
    point_cloud2.is_bigendian = false;
    point_cloud2.is_dense = false; // there may be invalid points

    //iterators
    sensor_msgs::PointCloud2Iterator<float> out_x(point_cloud2, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(point_cloud2, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(point_cloud2, "z");

    // cloud.points.resize(height*width);
    for (int y = 0; y < height; y ++ ) {
        for (int x = 0; x < width; x ++ ) {
            geometry_msgs::Point32 p;

            uint16_t depth_t = ((uint16_t*)depth)[y * width + x];
            
            float Zr = depth_t / scale;
            float Xr = Zr * (x - cx) / fx;
			float Yr = Zr * (y - cy) / fy;

            Eigen::Vector3d Pc(Xr, Yr, Zr);
            Eigen::Vector3d Pw = Rwc * Pc + twc;
            // Eigen::Vector3d Pw = Rcw * Pc + twc;
            
            p.x = Pw.x();
            p.y = Pw.y();
            p.z = Pw.z();
            point_cloud_dense.points.push_back(p);

            *out_x = Pc.x();
            *out_y = Pc.y();
            *out_z = Pc.z();
            ++out_x;
            ++out_y;
            ++out_z;
        }
    }
    point_cloud_dense.header = header;
    point_cloud_dense.header.frame_id = "slam";
    point_cloud2.header = header;
    point_cloud2.header.frame_id = "body";
    pub_point_cloud_dense.publish(point_cloud_dense);
    pub_point_cloud2.publish(point_cloud2);
}
