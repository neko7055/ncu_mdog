#!/usr/bin/python

import rospy
import time
import sys
import math
import smbus
import os
from imusensor.MPU9250 import MPU9250
from sensor_msgs.msg import Imu, MagneticField

deg2rad = lambda x: x * math.pi / 180




def get_quaternion_from_euler(roll, pitch, yaw):
  qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
  qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
  qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
  qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
 
  return [qx, qy, qz, qw]

acc_cov = [1.51602902e-04,-3.14548114e-06,3.29917718e-06,-3.14548114e-06,1.16652327e-04,5.75872312e-08,3.29917718e-06,5.75872312e-08,2.87442763e-04]
gyro_cov = [2.47028279e-03, 3.40442925e-05, 4.78160553e-06,3.40442925e-05, 3.17217356e-03, 1.73806438e-06,4.78160553e-06, 1.73806438e-06, 2.61970877e-03]
address = 0x68
bus = smbus.SMBus(0)
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
	    self.sensor = MPU9250.MPU9250(bus, address)
	    self.gyroRoll = 0
	    self.gyroPitch = 0
	    self.gyroYaw = 0

	    self.roll = 0
	    self.pitch = 0
	    self.yaw = 0
	    self.sensor.setGyroRange("GyroRangeSelect500DPS")
	    self.sensor.setLowPassFilterFrequency("AccelLowPassFilter20")
	    self.sensor.setAccelRange("AccelRangeSelect16G")
	    self.sensor.begin()

	    self.newTime = time.time()
	    self.currTime = time.time()


	    rospy.loginfo("Flusing first 50 data readings ...")
            
	    for x in range(0, 50):
	        self.sensor.readSensor()

	        time.sleep(0.01)

        except:
            rospy.logerr("Can not receive data from the I2C device: "+ self.imu_i2c + ". Did you specify the correct No. ?")
            sys.exit(0) 
        rospy.loginfo("Communication success !")

        # ROS handler        
        self.pub_imu = rospy.Publisher('imu_data', Imu, queue_size=1)
	self.pub_mag = rospy.Publisher('mag_data', MagneticField, queue_size=1)     
        self.timer_pub = rospy.Timer(rospy.Duration(1.0/self.pub_freq), self.timerCB) 

    def compFilter(self,ax,ay,az,gx,gy,gz,dt):
	ax,ay,az = ax/9.80665,ay/9.80665,az/9.80665
        accPitch = math.degrees(math.atan2(ay, az))
        accRoll = math.degrees(math.atan2(ax, az))

        # Gyro integration angle
        self.gyroRoll -= gy * dt
        self.gyroPitch += gx * dt
        self.gyroYaw += gz * dt
        self.yaw = self.gyroYaw

        # Comp filter
        self.roll = (0.5)*(self.roll - gy*dt) + (0.5)*(accRoll)
        self.pitch = (0.5)*(self.pitch + gx*dt) + (0.5)*(accPitch)

    def timerCB(self, event):    
        #I2C Serial read & publish 
	global deg2rad
        try:           
            self.sensor.readSensor()
    	    self.newTime = time.time()
	    dt = self.newTime - self.currTime
	    self.currTime = self.newTime
	    self.compFilter(self.sensor.AccelVals[0],self.sensor.AccelVals[1],self.sensor.AccelVals[2],self.sensor.GyroVals[0],self.sensor.GyroVals[1],self.sensor.GyroVals[2],dt)

	    q = get_quaternion_from_euler(deg2rad(self.roll), (self.pitch), (self.yaw))
	    TIME_NOW = rospy.Time.now()
            # Mag raw data
	    magMsg = MagneticField()
	    magMsg.header.stamp= TIME_NOW
            magMsg.header.frame_id = self.mag_link
            magMsg.magnetic_field_covariance[0] = self.mag_cov
            magMsg.magnetic_field_covariance[4] = self.mag_cov
            magMsg.magnetic_field_covariance[8] = self.mag_cov
	    magMsg.magnetic_field.x = self.sensor.MagVals[0]
            magMsg.magnetic_field.y = self.sensor.MagVals[1]
            magMsg.magnetic_field.z = self.sensor.MagVals[2]
 	    # Imu raw data
            imuMsg = Imu()
            imuMsg.header.stamp= TIME_NOW
            imuMsg.header.frame_id = self.imu_link
	    imuMsg.orientation.x = q[0]
	    imuMsg.orientation.y = q[1]
	    imuMsg.orientation.z = q[2]
	    imuMsg.orientation.w = q[3]
            imuMsg.orientation_covariance[0] = self.ori_cov
            imuMsg.orientation_covariance[4] = self.ori_cov
            imuMsg.orientation_covariance[8] = self.ori_cov
            imuMsg.angular_velocity_covariance = gyro_cov
            imuMsg.linear_acceleration_covariance = acc_cov 
            imuMsg.angular_velocity.x = self.sensor.GyroVals[0]
            imuMsg.angular_velocity.y = self.sensor.GyroVals[1]
            imuMsg.angular_velocity.z = self.sensor.GyroVals[2]
            imuMsg.linear_acceleration.x = self.sensor.AccelVals[0]
            imuMsg.linear_acceleration.y = self.sensor.AccelVals[1]
            imuMsg.linear_acceleration.z = self.sensor.AccelVals[2]
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
