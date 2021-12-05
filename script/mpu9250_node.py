#!/usr/bin/python

import rospy
import time
import sys
import math
from mpu9250 import MPU9250
from sensor_msgs.msg import Imu, MagneticField
acc_cov = [1.51602902e-04,-3.14548114e-06,3.29917718e-06,-3.14548114e-06,1.16652327e-04,5.75872312e-08,3.29917718e-06,5.75872312e-08,2.87442763e-04]
gyro_cov = [2.47028279e-03, 3.40442925e-05, 4.78160553e-06,3.40442925e-05, 3.17217356e-03, 1.73806438e-06,4.78160553e-06, 1.73806438e-06, 2.61970877e-03]

class IMUNode:
    def __init__(self):
        # ROS Parameters:
	self.mag_cov = float(rospy.get_param('~mag_cov', '0.5') ) # MagField covariance
        self.ori_cov = float(rospy.get_param('~ori_cov', '0.0025') ) # Orientation covariance
        self.vel_cov = float(rospy.get_param('~vel_cov', '0.02') ) # Angular velocity covariance
        self.acc_cov = float(rospy.get_param('~acc_cov', '0.04') ) # Linear acceleration covariance
        self.imu_i2c = rospy.get_param('~imu_i2c', '0x68') # I2C device No
        self.imu_link = rospy.get_param('~imu_link', 'imu_link') # imu_link name
	self.mag_link = rospy.get_param('~mag_link', 'mag_link') # imu_link name
        self.pub_freq = float( rospy.get_param('~pub_freq', '50') ) # hz of imu pub
                
        # I2C Communication:
        try:
	    self.sensor = MPU9250(0x68,0)
	    rospy.loginfo("Flusing first 50 data readings ...")
	    for x in range(0, 50):
	        gyro_data = self.sensor.readGyro()
	        time.sleep(0.01)

        except:
            rospy.logerr("Can not receive data from the I2C device: "+ self.imu_i2c + ". Did you specify the correct No. ?")
            sys.exit(0) 
        rospy.loginfo("Communication success !")

        # ROS handler        
        self.pub_imu = rospy.Publisher('imu_data', Imu, queue_size=1)
	self.pub_mag = rospy.Publisher('mag_data', MagneticField, queue_size=1)     
        self.timer_pub = rospy.Timer(rospy.Duration(1.0/self.pub_freq), self.timerCB) 

    def timerCB(self, event):    
        #I2C Serial read & publish 
        try:           
            gyro_data = self.sensor.readGyro()
            accel_data = self.sensor.readAccel()
	    mag_data = self.sensor.readMagnet()
	    TIME_NOW = rospy.Time.now()
            # Mag raw data
	    magMsg = MagneticField()
	    magMsg.header.stamp= TIME_NOW
            magMsg.header.frame_id = self.mag_link
            magMsg.magnetic_field_covariance[0] = self.mag_cov
            magMsg.magnetic_field_covariance[4] = self.mag_cov
            magMsg.magnetic_field_covariance[8] = self.mag_cov
	    magMsg.magnetic_field.x = float(mag_data['x'])
            magMsg.magnetic_field.y = float(mag_data['y'])
            magMsg.magnetic_field.z = float(mag_data['z'])
 	    # Imu raw data
            imuMsg = Imu()
            imuMsg.header.stamp= TIME_NOW
            imuMsg.header.frame_id = self.imu_link
            imuMsg.orientation_covariance[0] = self.ori_cov
            imuMsg.orientation_covariance[4] = self.ori_cov
            imuMsg.orientation_covariance[8] = self.ori_cov
            imuMsg.angular_velocity_covariance = gyro_cov
            imuMsg.linear_acceleration_covariance = acc_cov 
            imuMsg.angular_velocity.x = float(gyro_data['x'])
            imuMsg.angular_velocity.y = float(gyro_data['y'])
            imuMsg.angular_velocity.z = float(gyro_data['z'])
            imuMsg.linear_acceleration.x = float(accel_data['x'])
            imuMsg.linear_acceleration.y = float(accel_data['y'])
            imuMsg.linear_acceleration.z = float(accel_data['z'])
            # Publish
	    self.pub_mag.publish(magMsg)
            self.pub_imu.publish(imuMsg)
        except: 
            rospy.loginfo("Error in sensor value !") 	          
            pass            
        
if __name__ == "__main__":
    try:    
        # ROS Init    
        rospy.init_node('mpu9250_node', anonymous=True)

        # Constract IMUNode Obj
        rospy.loginfo("Start Reading ...")
        imu = IMUNode()
        rospy.spin()
    except KeyboardInterrupt:           
        print("Shutting down")
