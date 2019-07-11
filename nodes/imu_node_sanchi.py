#!/usr/bin/env python

import rospy
import serial
import string
import math
import sys

#from time import time
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from razor_imu_9dof.cfg import imuConfig
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

def hexShow(data):  
    result = ''  
    hLen = len(data)  
    for i in xrange(hLen):  
        hvol = ord(data[i])  
        hhex = '%02x'%hvol  
        result += hhex+' '
    print 'hexShow:',result

def H_(data):
    re = ord(data) * 256
    if re > 32767:
        return re - 65536
    else:
        return re
def L_(data):
    re = ord(data)
    return re

degrees2rad = math.pi/180.0
imu_yaw_calibration = 0.0

rospy.init_node("razor_node_sanchi")
#We only care about the most recent measurement, i.e. queue_size=1
pub = rospy.Publisher('imu', Imu, queue_size=1)
diag_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=1)
diag_pub_time = rospy.get_time();

imuMsg = Imu()

# Orientation covariance estimation:
# Observed orientation noise: 0.3 degrees in x, y, 0.6 degrees in z
# Magnetometer linearity: 0.1% of full scale (+/- 2 gauss) => 4 milligauss
# Earth's magnetic field strength is ~0.5 gauss, so magnetometer nonlinearity could
# cause ~0.8% yaw error (4mgauss/0.5 gauss = 0.008) => 2.8 degrees, or 0.050 radians
# i.e. variance in yaw: 0.0025
# Accelerometer non-linearity: 0.2% of 4G => 0.008G. This could cause
# static roll/pitch error of 0.8%, owing to gravity orientation sensing
# error => 2.8 degrees, or 0.05 radians. i.e. variance in roll/pitch: 0.0025
# so set all covariances the same.
imuMsg.orientation_covariance = [
0.0025 , 0 , 0,
0, 0.0025, 0,
0, 0, 0.0025
]

# Angular velocity covariance estimation:
# Observed gyro noise: 4 counts => 0.28 degrees/sec
# nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
# Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
imuMsg.angular_velocity_covariance = [ #original 0.02
0.02, 0 , 0,
0 , 0.02, 0,
0 , 0 , 0.02
]

# linear acceleration covariance estimation:
# observed acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
# nonliniarity spec: 0.5% of full scale => 0.2m/s^2
# Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
imuMsg.linear_acceleration_covariance = [ #original 0.04
0.04 , 0 , 0,
0 , 0.04, 0,
0 , 0 , 0.04
]

default_port='/dev/ttyUSB0'

#read calibration parameters
port = rospy.get_param('~port', default_port)
frame_id = rospy.get_param('~frame_id','imu_link')

#accelerometer
accel_x_min = rospy.get_param('~accel_x_min', -250.0)
accel_x_max = rospy.get_param('~accel_x_max', 250.0)
accel_y_min = rospy.get_param('~accel_y_min', -250.0)
accel_y_max = rospy.get_param('~accel_y_max', 250.0)
accel_z_min = rospy.get_param('~accel_z_min', -250.0)
accel_z_max = rospy.get_param('~accel_z_max', 250.0)

# magnetometer
magn_x_min = rospy.get_param('~magn_x_min', -600.0)
magn_x_max = rospy.get_param('~magn_x_max', 600.0)
magn_y_min = rospy.get_param('~magn_y_min', -600.0)
magn_y_max = rospy.get_param('~magn_y_max', 600.0)
magn_z_min = rospy.get_param('~magn_z_min', -600.0)
magn_z_max = rospy.get_param('~magn_z_max', 600.0)
calibration_magn_use_extended = rospy.get_param('~calibration_magn_use_extended', False)
magn_ellipsoid_center = rospy.get_param('~magn_ellipsoid_center', [0, 0, 0])
magn_ellipsoid_transform = rospy.get_param('~magn_ellipsoid_transform', [[0, 0, 0], [0, 0, 0], [0, 0, 0]])
imu_yaw_calibration = rospy.get_param('~imu_yaw_calibration', 0.0)

# gyroscope
gyro_average_offset_x = rospy.get_param('~gyro_average_offset_x', 0.0)
gyro_average_offset_y = rospy.get_param('~gyro_average_offset_y', 0.0)
gyro_average_offset_z = rospy.get_param('~gyro_average_offset_z', 0.0)
accel_average_offset_x = rospy.get_param('~accel_average_offset_x', 0.0)
accel_average_offset_y = rospy.get_param('~accel_average_offset_y', 0.0)
accel_average_offset_z = rospy.get_param('~accel_average_offset_z', 0.0)

#rospy.loginfo("%f %f %f %f %f %f", accel_x_min, accel_x_max, accel_y_min, accel_y_max, accel_z_min, accel_z_max)
#rospy.loginfo("%f %f %f %f %f %f", magn_x_min, magn_x_max, magn_y_min, magn_y_max, magn_z_min, magn_z_max)
#rospy.loginfo("%s %s %s", str(calibration_magn_use_extended), str(magn_ellipsoid_center), str(magn_ellipsoid_transform[0][0]))
#rospy.loginfo("%f %f %f", gyro_average_offset_x, gyro_average_offset_y, gyro_average_offset_z)

# Check your COM port and baud rate
rospy.loginfo("Opening %s...", port)
try:
    ser = serial.Serial(port=port, baudrate=115200, timeout=3)
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port "+port + ". Did you specify the correct port in the launch file?")
    #exit
    sys.exit(0)

roll=0
pitch=0
yaw=0
seq=0
accel_factor = 9.806 / 256.0    # sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.

rospy.logwarn("[Imu_node_sanchi]: Sanchi IMU starting")

tmp = ser.write('\xA5\x5A\x04\x01\x05\xAA' + chr(13))

data_length = 40

while not rospy.is_shutdown():
    #rospy.logwarn("check1")
    data = ser.read(data_length)
    #rospy.logwarn("check2")
    if len(data) < 40:
        print "check 1"
        hexShow(data)
        continue
    if '\xa5' == data[0] and '\x5a' == data[1] and '\xaa' == data[data_length-1]:
        #print "Horay!!"
        #hexShow(data)
        yaw_deg = ( H_(data[3]) + L_(data[4]) )/10.0
        yaw_deg += imu_yaw_calibration
        if yaw_deg > 180.0:
            yaw_deg = yaw_deg - 360.0
        if yaw_deg < -180.0:
            yaw_deg = yaw_deg + 360.0

        yaw = degrees2rad * yaw_deg
        pitch = degrees2rad * ( H_(data[7]) + L_(data[8]) )/10.0
        roll = degrees2rad * ( H_(data[5]) + L_(data[6]) )/10.0

        imuMsg.linear_acceleration.x = 9.806 * ( H_(data[9]) + L_(data[10]) ) / 16384.0 + accel_average_offset_x
        imuMsg.linear_acceleration.y = 9.806 * ( H_(data[11]) + L_(data[12]) ) / 16384.0 + accel_average_offset_y
        imuMsg.linear_acceleration.z = 9.806 * ( H_(data[13]) + L_(data[14]) ) / 16384.0 + accel_average_offset_z

        imuMsg.angular_velocity.x = degrees2rad * ( H_(data[15]) + L_(data[16]) ) / 32.8
        imuMsg.angular_velocity.y = degrees2rad * ( H_(data[17]) + L_(data[18]) ) / 32.8
        imuMsg.angular_velocity.z = degrees2rad * ( H_(data[19]) + L_(data[20]) ) / 32.8

        q = quaternion_from_euler(roll,pitch,yaw)
        imuMsg.orientation.x = q[0]
        imuMsg.orientation.y = q[1]
        imuMsg.orientation.z = q[2]
        imuMsg.orientation.w = q[3]

        imuMsg.header.stamp= rospy.Time.now()
        imuMsg.header.frame_id = frame_id
        imuMsg.header.seq = seq
        seq = seq + 1
        pub.publish(imuMsg)

	#if seq < 1000:
        	#print roll, pitch, yaw

    else:
        print "check 2"
        ind = data.find('\xa5')
        if ind >= 0:
            if ind == 0:
                data = ser.read(data_length)
            #elif ind == 1:
            #    tmp = ser.write('\xA5\x5A\x04\x01\x05\xAA' + chr(13))
            else:
                data = ser.read(ind)
        else:
            print "check 3"
            hexShow(data)
        print ind
        continue

ser.write('\xA5\x5A\x04\x02\x06\xAA' + chr(13))    
ser.close
#f.close
