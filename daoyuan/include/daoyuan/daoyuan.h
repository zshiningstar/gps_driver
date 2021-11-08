#ifndef DAOYUAN_H_
#define DAOYUAN_H_
#include<iostream>
#include<iostream>
#include<geodesy/utm.h>
#include<geodesy/wgs84.h>
#include<geographic_msgs/GeoPoint.h>
#include<Eigen/Dense>
#include<ros/ros.h>
#include<cmath>
#include<serial/serial.h>
#include<boost/thread.hpp>
#include<boost/bind.hpp>
#include"gps_msgs/Daoyuan.h"
#include"gps_msgs/Rotation.h"
#include<nav_msgs/Odometry.h>
#include"gps_msgs/Satellite.h"
#include"gps_msgs/Satellites.h"
#include <tf2_ros/transform_broadcaster.h>

#define NATURE        2.718281
#define K1            360/32768.0
#define K2            300/32768.0
#define K3            12/32768.0
#define K4            0.0000001
#define K5            0.001
#define K6            100/32768.0
#define K7            0.25
#define K8            200/32768.0

#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))


PACK(
struct pkgRS422_t
{
	uint8_t header0;
	uint8_t header1;
	uint8_t header2;
	
	int16_t roll;
	int16_t pitch;
	int16_t azimuth;
	
	int16_t gyroscope_velocity_x;
	int16_t gyroscope_velocity_y;
	int16_t gyroscope_velocity_z;
	
	int16_t accelerator_x;
	int16_t accelerator_y;
	int16_t accelerator_z;
	
	int32_t latitude;
	int32_t longitude;
	int32_t height;
	
	int16_t north_velocity;
	int16_t east_velocity;
	int16_t down_velocity;
	
	uint8_t gps_state;
	
	int16_t wheel_data1;
	int16_t wheel_data2;	
	int16_t wheel_data3;

	uint32_t gps_time; 
	uint8_t rotation_type;
	uint8_t xorcheck_value1;
	uint32_t gps_week;
	uint8_t xorcheck_value2;
});

double deg2rad(const double& deg)
{
	return deg*M_PI/180.0;
}

class Daoyuan
{
public:
	Daoyuan();
	~Daoyuan();
	Daoyuan(const Daoyuan& obj) = delete;
	Daoyuan& operator=(const Daoyuan& obj) = delete;
	bool init();
	void startReading();
	void stopReading();

private:
	bool openSerial(const std::string& port,int baudrate);
	void closeSerial();
	void readSerialThread();
	void parseIncomingData(uint8_t* buffer,size_t len);
	void parse(const uint8_t* buffer);
	
	uint8_t XOR(const uint8_t* buf,size_t len)
	{
		uint8_t sum = buf[0];
		for(size_t i=1; i<len; ++i)
		{
			sum = sum ^ buf[i];
		}
		return sum;
	}

private:
	ros::Publisher m_pub_rs422;
	ros::Publisher m_pub_ll2utm;
	
	ros::Publisher m_pub_wheel;
	
	serial::Serial *m_serial_port;
	//! shared pointer to Boost thread for listening for data from novatel
	boost::shared_ptr<boost::thread> m_read_thread_ptr;
	bool m_reading_status;  //!< True if the read thread is running, false otherwise.
	uint8_t * const m_pkg_buffer;
	
	gps_msgs::Daoyuan m_gps;
	gps_msgs::Rotation m_wheel;
	bool m_is_pub_ll2utm;
	bool m_is_pub_tf;
	bool m_is_pub_wheel;
	int  m_satelliteNum;
	int m_locationState;
	
	tf2_ros::TransformBroadcaster m_tf_br;
	std::string m_child_frame_id, m_parent_frame_id;
};

#endif
