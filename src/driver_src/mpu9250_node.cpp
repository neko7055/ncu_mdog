#include <RTIMULib.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

int main( int argc, char **argv ) {
    ros::init( argc, argv, "mpu9250_node" );
    ROS_INFO( "Imu driver is now running" );
    ros::NodeHandle n;
    ros::NodeHandle private_n( "~" );
    float mag_cov, ori_cov, vel_cov, acc_cov;
    private_n.setParam("mag_cov", 0.5);
    private_n.getParam("mag_cov", mag_cov);
    private_n.setParam("ori_cov", 0.0025);
    private_n.getParam("ori_cov", ori_cov);
    private_n.setParam("vel_cov", 0.02);
    private_n.getParam("vel_cov", vel_cov);
    private_n.setParam("acc_cov", 0.04);
    private_n.getParam("acc_cov", acc_cov);
    private_n.setParam("imu_i2c", 0x68);
    private_n.setParam("imu_link", "imu_link");
    private_n.setParam("mag_link", "mag_link");
    

    std::string imu_topic_name;
    if ( !private_n.getParam( "imu_topic_name", imu_topic_name ) ) {
        ROS_WARN( "No topic_name provided - default: imu/data" );
        imu_topic_name = "imu/data";
    }
    else {
        ROS_INFO( "Success Get imu_topic_name!" );
    }

    std::string mag_topic_name;
    if ( !private_n.getParam( "mag_topic_name", mag_topic_name ) ) {
        ROS_WARN( "No topic_name provided - default: mag/data" );
        imu_topic_name = "mag/data";
    }
    else {
        ROS_INFO( "Success Get mag_topic_name!" );
    }

    std::string calibration_file_path;
    if ( !private_n.getParam( "calibration_file_path", calibration_file_path ) ) {
        ROS_ERROR( "The calibration_file_path parameter must be set to use a calibration file." );
    }
    else {
        ROS_INFO( "Success Get calibration_file_path!" );
    }

    std::string calibration_file_name;
    if ( !private_n.getParam( "calibration_file_name", calibration_file_name ) ) {
        ROS_WARN( "No calibration_file_name provided - default: RTIMULib.ini" );
        calibration_file_name = "RTIMULib";
    }
    else {
        ROS_INFO( "Success Get calibration_file_name!" );
    }

    std::string imu_link;
    if ( !private_n.getParam( "imu_link", imu_link ) ) {
        ROS_WARN( "No imu_link provided - default: imu_link" );
        imu_link = "imu_link";
    }
    else {
        ROS_INFO( "Success Get imu_frame_id!" );
    }

    std::string mag_link;
    if ( !private_n.getParam( "mag_link", mag_link ) ) {
        ROS_WARN( "No imu_link provided - default: imu_link" );
        mag_link = "mag_link";
    }
    else {
        ROS_INFO( "Success Get mag_frame_id!" );
    }

    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>( imu_topic_name.c_str(), 1000 );
    ros::Publisher mag_pub = n.advertise<sensor_msgs::MagneticField>( mag_topic_name.c_str(), 1000 );

    // Load the RTIMULib.ini config file
    RTIMUSettings *settings = new RTIMUSettings( calibration_file_path.c_str(), calibration_file_name.c_str() );

    RTIMU *imu = RTIMU::createIMU( settings );

    if ( ( imu == NULL ) || ( imu->IMUType() == RTIMU_TYPE_NULL ) ) {
        ROS_ERROR( "No Imu found" );
        return -1;
    }
    else {
        ROS_INFO( "Found Imu!" );
    }

    // Initialise the imu object
    imu->IMUInit();

    // Set the Fusion coefficient
    imu->setSlerpPower( 0.02 );
    // Enable the sensors
    imu->setGyroEnable( true );
    imu->setAccelEnable( true );
    imu->setCompassEnable( true );

    ros::Rate loop_rate( 1000 / imu->IMUGetPollInterval() );
    private_n.setParam("pub_freq", 1000 / imu->IMUGetPollInterval());
    ROS_INFO( "Get in while loop" );
    while ( ros::ok() ) {
        sensor_msgs::Imu imu_msg;
	sensor_msgs::MagneticField mag_msg;

        if ( imu->IMURead() ) {
            RTIMU_DATA imu_data = imu->getIMUData();
	    mag_msg.header.stamp = ros::Time::now();
	    mag_msg.header.frame_id = mag_link;
	    mag_msg.magnetic_field.z = imu_data.compass.x();
	    mag_msg.magnetic_field.y = imu_data.compass.y();
	    mag_msg.magnetic_field.x = imu_data.compass.z();
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = imu_link;
            imu_msg.orientation.x = imu_data.fusionQPose.x();
            imu_msg.orientation.y = imu_data.fusionQPose.y();
            imu_msg.orientation.z = imu_data.fusionQPose.z();
            imu_msg.orientation.w = imu_data.fusionQPose.scalar();
            imu_msg.angular_velocity.x = imu_data.gyro.x();
            imu_msg.angular_velocity.y = imu_data.gyro.y();
            imu_msg.angular_velocity.z = imu_data.gyro.z();
            imu_msg.linear_acceleration.x = imu_data.accel.x() * 9.80665;
            imu_msg.linear_acceleration.y = imu_data.accel.y() * 9.80665;
            imu_msg.linear_acceleration.z = imu_data.accel.z() * 9.80665;

            imu_pub.publish( imu_msg );
            mag_pub.publish( mag_msg );
        }

        ros::spinOnce();
    }
    return 0;
}
