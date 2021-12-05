#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

void ImuCallback( const sensor_msgs::Imu::ConstPtr &imu_msg ) {
    ROS_INFO( "Sucess subscribe to imu_publisher" );
    ROS_INFO( "Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w );
    ROS_INFO( "IMU Angilar velocity x: [%f], y[%f], z[%f]", imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z );
    ROS_INFO( "IMU Linear acceleration x: [%f], y[%f], z[%f]\n", imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z );
}

int main( int argc, char **argv ) {

    ros::init( argc, argv, "imu_listener" );
    ros::NodeHandle n;
    ROS_INFO("Imu Subscriber turn on sucess");
    ros::Subscriber sub = n.subscribe( "/imu", 1000, ImuCallback );
    ros::spin();

    return 0;
}
