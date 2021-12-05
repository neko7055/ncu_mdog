/*
 Copyright 2018 NCU MATH.
 Developer: Kuo-Shih Tseng (kuoshih@math.ncu.edu.tw)
 Description: This code activate a node "main." 
 This node subscribes three topics -- imu_data, odom, and scan. 
 You can access data from three Callback functions.
 $Revision: 1.0 $,  2018.07.24 
 $Revision: 1.1 $,  2018.12.20, add a maker for users 
 
 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at
     http://www.apache.org/licenses/LICENSE-2.0
 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
*/

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/Marker.h>
#define RAD2DEG(x) ((x)*180./M_PI)

void marker_callback(const ros::TimerEvent&);
void init_marker(void);

visualization_msgs::Marker marker;
uint32_t shape = visualization_msgs::Marker::CYLINDER;
ros::Publisher marker_pub;
int counter=0;

void marker_callback(const ros::TimerEvent&)
{// update maker location and publish it. 
    float x=1*cos(0.174*counter);
    float y=1*sin(0.174*counter);
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    //ROS_INFO("x=%f,y=%f\n",x,y);
    counter++;

    marker_pub.publish(marker);
}

void timer_callback1(const ros::TimerEvent&)
{
    printf( "Pass %f second\n\n", 1.0f);
}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    printf("[YDLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
    
    printf("[YDLIDAR INFO]: angle_range : [%f, %f]\n\n", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
    /*
    for(int i = 0; i < count; i++) 
    {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
	    if(degree > -5 && degree< 5)
        {printf("[YDLIDAR INFO]: angle-distance : [%f, %f, %i]\n", degree, scan->ranges[i], i);}
    }
    */
    
}

void imuCallback( const sensor_msgs::Imu::ConstPtr &imu_msg ) {
    ROS_INFO( "Sucess subscribe to imu_publisher" );
    ROS_INFO( "Imu Orientation : x[%f], y: [%f], z: [%f], w: [%f]", imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w );
    ROS_INFO( "IMU Angilar velocity : x[%f], y[%f], z[%f]", imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z );
    ROS_INFO( "IMU Linear acceleration : x[%f], y[%f], z[%f]\n", imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z );
}

void magCallback( const sensor_msgs::MagneticField::ConstPtr &mag_msg ) {
    ROS_INFO( "Sucess subscribe to mag_publisher" );
    ROS_INFO( "MagneticField: x[%f], y[%f], z[%f]\n", mag_msg->magnetic_field.x, mag_msg->magnetic_field.y, mag_msg->magnetic_field.z );
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");


  ros::NodeHandle n;
  //ros::Rate r(1); 

  ros::Subscriber sub_lidar = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
  ros::Subscriber sub_imu = n.subscribe<sensor_msgs::Imu>("/imu", 1000, imuCallback);
  ros::Subscriber sub_mag = n.subscribe<sensor_msgs::MagneticField>("/mag", 1000, magCallback);
  // create a timer callback
  ros::Timer timer1 = n.createTimer(ros::Duration(1), timer_callback1);
  // create a topic "visualization_marker"
  ros::Timer timer_marker = n.createTimer(ros::Duration(0.1), marker_callback);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  init_marker();

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */

  ros::spin();


  return 0;
}

void init_marker(void)
{
    // Initialize maker's setting.
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/target";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    // Tag(ACTION)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    //Tag(POSE)
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    //Tag(LIFETIME)
    marker.lifetime = ros::Duration();

}
