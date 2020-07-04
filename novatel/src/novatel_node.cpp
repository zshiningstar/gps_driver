/*
 * The MIT License (MIT)
 * Copyright (c) 2012 William Woodall <wjwwood@gmail.com>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a 
 * copy of this software and associated documentation files (the "Software"), 
 * to deal in the Software without restriction, including without limitation 
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, 
 * and/or sell copies of the Software, and to permit persons to whom the 
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <cfloat>
#include <algorithm>
#include <string>

#include <ros/ros.h>
#include <tf/tf.h>
 

#ifdef WIN32
 #ifdef DELETE
 // ach, windows.h polluting everything again,
 // clashes with autogenerated visualization_msgs/Marker.h
 #undef DELETE
 #endif
#endif
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "gps_msgs/Ephemeris.h"
#include "gps_msgs/L1L2Range.h"
#include "gps_msgs/Inspvax.h"   //add by wendao
#include "sensor_msgs/Imu.h"

#include <boost/tokenizer.hpp>
#include <boost/thread/thread.hpp>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPoint.h>
#include <Eigen/Dense>

#include "novatel/novatel.h"
using namespace novatel;

// Logging system message handlers
void handleInfoMessages(const std::string &msg) {ROS_INFO("%s",msg.c_str());}
void handleWarningMessages(const std::string &msg) {ROS_WARN("%s",msg.c_str());}
void handleErrorMessages(const std::string &msg) {ROS_ERROR("%s",msg.c_str());}
void handleDebugMessages(const std::string &msg) {ROS_DEBUG("%s",msg.c_str());}

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static double radians_to_degrees = 180.0 / M_PI;
static double degrees_to_radians = M_PI / 180.0;
static double degrees_square_to_radians_square = degrees_to_radians*degrees_to_radians;

static double sigma_v = 0.05; // velocity std dev in m/s

// ROS Node class
class NovatelNode {
public:
  NovatelNode() : nh_("~"){

    // set up logging handlers
    gps_.setLogInfoCallback(handleInfoMessages);
    gps_.setLogWarningCallback(handleWarningMessages);
    gps_.setLogErrorCallback(handleErrorMessages);
    gps_.setLogDebugCallback(handleDebugMessages);

    gps_.set_best_utm_position_callback(boost::bind(&NovatelNode::BestUtmHandler, this, _1, _2));
    gps_.set_best_velocity_callback(boost::bind(&NovatelNode::BestVelocityHandler, this, _1, _2));
    gps_.set_best_position_ecef_callback(boost::bind(&NovatelNode::BestPositionEcefHandler, this, _1, _2));
    //gps_.set_ins_position_velocity_attitude_short_callback(boost::bind(&NovatelNode::InsPvaHandler, this, _1, _2));
    //gps_.set_ins_covariance_short_callback(boost::bind(&NovatelNode::InsCovHandler, this, _1, _2));
    gps_.set_ins_position_velocity_attitude_callback(boost::bind(&NovatelNode::InsPvaHandler, this, _1, _2));
    gps_.set_ins_covariance_callback(boost::bind(&NovatelNode::InsCovHandler, this, _1, _2));    
    gps_.set_raw_imu_short_callback(boost::bind(&NovatelNode::RawImuShortHandler, this, _1, _2));
    gps_.set_receiver_hardware_status_callback(boost::bind(&NovatelNode::HardwareStatusHandler, this, _1, _2));
    gps_.set_gps_ephemeris_callback(boost::bind(&NovatelNode::EphemerisHandler, this, _1, _2));
    gps_.set_compressed_range_measurements_callback(boost::bind(&NovatelNode::CompressedRangeHandler, this, _1, _2));
    // gps_.set_range_measurements_callback(boost::bind(&NovatelNode::RangeHandler, this, _1, _2));
    // gps_.set_raw_msg_callback(boost::bind(&NovatelNode::RawMsgHandler, this, _1));
    gps_.set_best_pseudorange_position_callback(boost::bind(&NovatelNode::PsrPosHandler, this, _1, _2));
    
    gps_.set_inspvax_callback(boost::bind(&NovatelNode::InspvaxHandler,this,_1,_2)); //add by wendao
    gps_.set_bestgnss_callback(boost::bind(&NovatelNode::BestGnssHandler,this,_1,_2)); //add by wendao
    gps_.set_corrImu_short_callback(boost::bind(&NovatelNode::CorrImuShortHandler,this,_1,_2)); //add by wendao
  }

  ~NovatelNode() {
    this->disconnect();
  }


  inline double psi2theta(double psi) {return M_PI/2-psi;}  
  inline double theta2psi(double theta) {return M_PI/2-theta;}


  void BestUtmHandler(UtmPosition &pos, double &timestamp) {
    //ROS_DEBUG("Received BestUtm");
    cur_utm_bestpos_ = pos;
    ros::Time stamp = ros::Time::now();
    
    if(nav_sat_fix_publisher_.getNumSubscribers())
    {
		sensor_msgs::NavSatFix sat_fix;
		sat_fix.header.stamp = stamp;
		sat_fix.header.frame_id = "/odom";
		sat_fix.status.status= pos.position_type;
//		std::cout << int(pos.position_type) << std::endl;
//		if (pos.position_type == NONE)
//		  sat_fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX; 
//		else if ((pos.position_type == WAAS) || 
//		         (pos.position_type == OMNISTAR) ||   
//		         (pos.position_type == OMNISTAR_HP) || 
//		         (pos.position_type == OMNISTAR_XP) || 
//		         (pos.position_type == CDGPS))
//		  sat_fix.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;//星基增强
//		else if ((pos.position_type == PSRDIFF) || 
//		         (pos.position_type == NARROW_FLOAT) ||   
//		         (pos.position_type == WIDE_INT) ||     
//		         (pos.position_type == WIDE_INT) ||     
//		         (pos.position_type == NARROW_INT) ||     
//		         (pos.position_type == RTK_DIRECT_INS) ||     
//		         (pos.position_type == INS_PSRDIFF))
//		  sat_fix.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;//地基增强
//		 else if((pos.position_type == INS_RTKFLOAT) ||
//		         (pos.position_type == INS_RTKFIXED))
//		  sat_fix.status.status = 10; //自定义状态,RTK
//		 else 
//		  sat_fix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;

		if (pos.signals_used_mask & 0x30)
		  sat_fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GLONASS;
		else
		  sat_fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

		// TODO: convert positon to lat, long, alt to export (this is available from PSRPOS)

		// TODO: add covariance
		// covariance is east,north,up in row major form

		sat_fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
		nav_sat_fix_publisher_.publish(sat_fix);
	}
	
	if(bestutm_publisher_.getNumSubscribers())
	{
		nav_msgs::Odometry cur_odom_;
		cur_odom_.header.stamp = stamp;
		cur_odom_.header.frame_id = "/odom";
		cur_odom_.pose.pose.position.x = pos.easting;
		cur_odom_.pose.pose.position.y = pos.northing;
		cur_odom_.pose.pose.position.z = pos.height;
		// covariance representation given in REP 103
		//http://www.ros.org/reps/rep-0103.html#covariance-representation
		// (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
		// row major
		cur_odom_.pose.covariance[0] = pos.easting_standard_deviation * pos.easting_standard_deviation;
		cur_odom_.pose.covariance[7] = pos.northing_standard_deviation * pos.northing_standard_deviation;
		cur_odom_.pose.covariance[14] = pos.height_standard_deviation * pos.height_standard_deviation;
		// have no way of knowing roll and pitch with just GPS
		cur_odom_.pose.covariance[21] = DBL_MAX;
		cur_odom_.pose.covariance[28] = DBL_MAX;

		// see if there is a recent velocity message
		if ((cur_velocity_.header.gps_week==pos.header.gps_week) 
		     && (cur_velocity_.header.gps_millisecs==pos.header.gps_millisecs)) 
		{
		  cur_odom_.twist.twist.linear.x=cur_velocity_.horizontal_speed*cos(cur_velocity_.track_over_ground*degrees_to_radians);
		  cur_odom_.twist.twist.linear.y=cur_velocity_.horizontal_speed*sin(cur_velocity_.track_over_ground*degrees_to_radians);
		  cur_odom_.twist.twist.linear.z=cur_velocity_.vertical_speed;

		  cur_odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(
		      psi2theta(cur_velocity_.track_over_ground*degrees_to_radians));

		  // if i have a fix, velocity std, dev is constant
		  if (cur_velocity_.position_type>NONE) {
		    // yaw covariance
		    double heading_std_dev=sigma_v/cur_velocity_.horizontal_speed;
		    cur_odom_.pose.covariance[35] = heading_std_dev * heading_std_dev;
		    // x and y velocity covariance
		    cur_odom_.twist.covariance[0] = sigma_v*sigma_v;
		    cur_odom_.twist.covariance[7] = sigma_v*sigma_v;
		  } else {
		    cur_odom_.pose.covariance[35] = DBL_MAX;
		    cur_odom_.twist.covariance[0] = DBL_MAX;
		    cur_odom_.twist.covariance[7] = DBL_MAX;
		  }
		}

		bestutm_publisher_.publish(cur_odom_);
	}
  }


  void BestVelocityHandler(Velocity &vel, double &timestamp) {
    ROS_DEBUG("Received BestVel");
    cur_velocity_ = vel;
  }


  void InsPvaHandler(InsPositionVelocityAttitude &ins_pva, double &timestamp) {
    //ROS_INFO("Received inspva.");

    // convert pva position to UTM
    double northing, easting;
    int zoneNum;
    bool north;

    gps_.ConvertLLaUTM(ins_pva.latitude, ins_pva.longitude, &northing, &easting, &zoneNum, &north);

    sensor_msgs::NavSatFix sat_fix;
    sat_fix.header.stamp = ros::Time::now();
    sat_fix.header.frame_id = "/odom";

    if (ins_pva.status == INS_SOLUTION_GOOD)
      sat_fix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    else 
      sat_fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;

    sat_fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

    sat_fix.latitude = ins_pva.latitude;
    sat_fix.longitude = ins_pva.longitude;
    sat_fix.altitude = ins_pva.height;

    sat_fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    nav_sat_fix_publisher_.publish(sat_fix);

    nav_msgs::Odometry cur_odom_;
    cur_odom_.header.stamp = sat_fix.header.stamp;
    cur_odom_.header.frame_id = "/odom";
    cur_odom_.pose.pose.position.x = easting;
    cur_odom_.pose.pose.position.y = northing;
    cur_odom_.pose.pose.position.z = ins_pva.height;
    cur_odom_.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(ins_pva.roll*degrees_to_radians,
          ins_pva.pitch*degrees_to_radians,
          psi2theta(ins_pva.azimuth*degrees_to_radians));

    cur_odom_.twist.twist.linear.x=ins_pva.east_velocity;
    cur_odom_.twist.twist.linear.y=ins_pva.north_velocity;
    cur_odom_.twist.twist.linear.z=ins_pva.up_velocity;
      // TODO: add covariance

    // see if there is a matching ins covariance message
    if ((cur_ins_cov_.gps_week==ins_pva.gps_week) 
         && (cur_ins_cov_.gps_millisecs==ins_pva.gps_millisecs)) {

      cur_odom_.pose.covariance[0] = cur_ins_cov_.position_covariance[0];
      cur_odom_.pose.covariance[1] = cur_ins_cov_.position_covariance[1];
      cur_odom_.pose.covariance[2] = cur_ins_cov_.position_covariance[2];
      cur_odom_.pose.covariance[6] = cur_ins_cov_.position_covariance[3];
      cur_odom_.pose.covariance[7] = cur_ins_cov_.position_covariance[4];
      cur_odom_.pose.covariance[8] = cur_ins_cov_.position_covariance[5];
      cur_odom_.pose.covariance[12] = cur_ins_cov_.position_covariance[6];
      cur_odom_.pose.covariance[13] = cur_ins_cov_.position_covariance[7];
      cur_odom_.pose.covariance[14] = cur_ins_cov_.position_covariance[8];

      cur_odom_.pose.covariance[21] = cur_ins_cov_.attitude_covariance[0]*degrees_square_to_radians_square;
      cur_odom_.pose.covariance[22] = cur_ins_cov_.attitude_covariance[1]*degrees_square_to_radians_square;
      cur_odom_.pose.covariance[23] = cur_ins_cov_.attitude_covariance[2]*degrees_square_to_radians_square;
      cur_odom_.pose.covariance[27] = cur_ins_cov_.attitude_covariance[3]*degrees_square_to_radians_square;
      cur_odom_.pose.covariance[28] = cur_ins_cov_.attitude_covariance[4]*degrees_square_to_radians_square;
      cur_odom_.pose.covariance[29] = cur_ins_cov_.attitude_covariance[5]*degrees_square_to_radians_square;
      cur_odom_.pose.covariance[33] = cur_ins_cov_.attitude_covariance[6]*degrees_square_to_radians_square;
      cur_odom_.pose.covariance[34] = cur_ins_cov_.attitude_covariance[7]*degrees_square_to_radians_square;
      cur_odom_.pose.covariance[35] = cur_ins_cov_.attitude_covariance[8]*degrees_square_to_radians_square;

      cur_odom_.twist.covariance[0] = cur_ins_cov_.velocity_covariance[0];
      cur_odom_.twist.covariance[1] = cur_ins_cov_.velocity_covariance[1];
      cur_odom_.twist.covariance[2] = cur_ins_cov_.velocity_covariance[2];
      cur_odom_.twist.covariance[6] = cur_ins_cov_.velocity_covariance[3];
      cur_odom_.twist.covariance[7] = cur_ins_cov_.velocity_covariance[4];
      cur_odom_.twist.covariance[8] = cur_ins_cov_.velocity_covariance[5];
      cur_odom_.twist.covariance[12] = cur_ins_cov_.velocity_covariance[6];
      cur_odom_.twist.covariance[13] = cur_ins_cov_.velocity_covariance[7];
      cur_odom_.twist.covariance[14] = cur_ins_cov_.velocity_covariance[8];

    }

    bestutm_publisher_.publish(cur_odom_);
  }

  void InsCovHandler(InsCovariance &cov, double &timestamp) {
     cur_ins_cov_ = cov;
  }


  void HardwareStatusHandler(ReceiverHardwareStatus &status, double &timestamp) {}


  void EphemerisHandler(GpsEphemeris &ephem, double &timestamp) {
    cur_ephem_.header.stamp = ros::Time::now();
    cur_ephem_.gps_time = ephem.header.gps_millisecs*1000;
    cur_ephem_.obs = 1;
    uint8_t n = ephem.prn; // how drtk expects it
    cur_ephem_.prn[n] = ephem.prn;
    cur_ephem_.health[n] = ephem.health;
    cur_ephem_.semimajor_axis[n] = ephem.semi_major_axis; // this value is A, not the sqrt of A
    cur_ephem_.mean_anomaly[n] = ephem.anomoly_reference_time;
    cur_ephem_.eccentricity[n] = ephem.eccentricity;
    cur_ephem_.perigee_arg[n] = ephem.omega;
    cur_ephem_.cos_latitude[n] = ephem.latitude_cosine;
    cur_ephem_.sin_latitude[n] = ephem.latitude_sine;
    cur_ephem_.cos_orbit_radius[n] = ephem.orbit_radius_cosine;
    cur_ephem_.sin_orbit_radius[n] = ephem.orbit_radius_sine;
    cur_ephem_.cos_inclination[n] = ephem.inclination_cosine;
    cur_ephem_.sin_inclination[n] = ephem.inclination_sine;
    cur_ephem_.inclination_angle[n] = ephem.inclination_angle;
    cur_ephem_.right_ascension[n] = ephem.right_ascension;
    cur_ephem_.mean_motion_diff[n] = ephem.mean_motion_difference;
    cur_ephem_.inclination_rate[n] = ephem.inclination_angle_rate;
    cur_ephem_.ascension_rate[n] = ephem.right_ascension_rate;
    cur_ephem_.time_of_week[n] = ephem.time_of_week;
    cur_ephem_.reference_time[n] = ephem.time_of_ephemeris;
    cur_ephem_.clock_correction[n] = ephem.sv_clock_correction;
    cur_ephem_.group_delay[n] = ephem.group_delay_difference;
    cur_ephem_.clock_aging_1[n] = ephem.clock_aligning_param_0;
    cur_ephem_.clock_aging_2[n] = ephem.clock_aligning_param_1;
    cur_ephem_.clock_aging_3[n] = ephem.clock_aligning_param_2;

    ephemeris_publisher_.publish(cur_ephem_);
  }


  void CompressedRangeHandler(CompressedRangeMeasurements &range, double &timestamp) {
    gps_msgs::L1L2Range cur_range_;
    cur_range_.header.stamp = ros::Time::now();
    cur_range_.gps_time = range.header.gps_millisecs;
    uint8_t L1_obs = 0, L2_obs = 0;
    cur_range_.num_obs = range.number_of_observations;

    for (int n=0; n<range.number_of_observations; n++) { //! FIXME how far should this iterate?
      // printf("%i ",n);
      // make sure something on this index & it is a GPS constellation SV
      if (
        (!range.range_data[n].range_record.satellite_prn) // empty field
        || (range.range_data[n].channel_status.satellite_sys != 0) // critical
        || (range.range_data[n].range_record.satellite_prn > 33) // critical
        )
      {
        continue;
      }

      uint8_t prn_idx = range.range_data[n].range_record.satellite_prn;
      switch (range.range_data[n].channel_status.signal_type) {
        case 0: // L1 C/A
          cur_range_.L1.prn[prn_idx] = prn_idx;
          cur_range_.L1.psr[prn_idx] = range.range_data[n].range_record.pseudorange/128.;
          cur_range_.L1.psr_std[prn_idx] = range.range_data[n].range_record.pseudorange_standard_deviation; // FIXME scale factor?
          cur_range_.L1.carrier.doppler[prn_idx] = range.range_data[n].range_record.doppler/256.;
          cur_range_.L1.carrier.noise[prn_idx] = range.range_data[n].range_record.carrier_to_noise + 20.;
          cur_range_.L1.carrier.phase[prn_idx] = -range.range_data[n].range_record.accumulated_doppler/256.; // negative sign is critical!!!
          cur_range_.L1.carrier.phase_std[prn_idx] = range.range_data[n].range_record.accumulated_doppler_std_deviation; // FIXME scale factor?
          L1_obs++;
          break;
        case 5: // L2 P
        case 9: // L2 P codeless
        case 17: // L2 C
          cur_range_.L2.prn[prn_idx] = prn_idx;
          cur_range_.L2.psr[prn_idx] = range.range_data[n].range_record.pseudorange/128.;
          cur_range_.L2.psr_std[prn_idx] = range.range_data[n].range_record.pseudorange_standard_deviation; // FIXME scale factor?
          cur_range_.L2.carrier.doppler[prn_idx] = range.range_data[n].range_record.doppler/256.;
          cur_range_.L2.carrier.noise[prn_idx] = range.range_data[n].range_record.carrier_to_noise + 20.;
          cur_range_.L2.carrier.phase[prn_idx] = -range.range_data[n].range_record.accumulated_doppler/256.; // negative sign is critical!!!
          cur_range_.L2.carrier.phase_std[prn_idx] = range.range_data[n].range_record.accumulated_doppler_std_deviation; // FIXME scale factor?
          L2_obs++;
          break;
        default:
          ROS_INFO_STREAM(name_ << ": L1L2RangeHandler: Unhandled signal type " << range.range_data[n].channel_status.signal_type);
          break;
      }
    }
        
    cur_range_.L1.obs = L1_obs;
    cur_range_.L2.obs = L2_obs;
    // change this to be populated by bestpos
    cur_range_.lat = cur_psrpos_.latitude;
    cur_range_.lon = cur_psrpos_.longitude;
    cur_range_.alt = cur_psrpos_.height;
    // !FIXME - convert these codes to actual values and populate fields
    // cur_range_.lat_cov = pow(cur_psr_lla_std_[0], 2);
    // cur_range_.lon_cov = pow(cur_psr_lla_std_[1], 2);
    // cur_range_.alt_cov = pow(cur_psr_lla_std_[2], 2);

    dual_band_range_publisher_.publish(cur_range_);
  }


  void PsrPosHandler(Position &pos, double timestamp) {///////////////////////////////////////////////////////////////////////
    cur_psrpos_ = pos;

    sensor_msgs::NavSatFix sat_fix;
    
    if (pos.position_type == NONE)
      sat_fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    else if ((pos.position_type == WAAS) || 
             (pos.position_type == OMNISTAR) ||   
             (pos.position_type == OMNISTAR_HP) || 
             (pos.position_type == OMNISTAR_XP) || 
             (pos.position_type == CDGPS))
      sat_fix.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
    else if ((pos.position_type == PSRDIFF) || 
             (pos.position_type == NARROW_FLOAT) ||   
             (pos.position_type == WIDE_INT) ||     
             (pos.position_type == WIDE_INT) ||     
             (pos.position_type == NARROW_INT) ||     
             (pos.position_type == RTK_DIRECT_INS) ||     
             (pos.position_type == INS_PSRDIFF) ||    
             (pos.position_type == INS_RTKFLOAT) ||   
             (pos.position_type == INS_RTKFIXED))
      sat_fix.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
     else 
      sat_fix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;

    if (pos.signals_used_mask & 0x30)
      sat_fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GLONASS;
    else
      sat_fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

    sat_fix.header.stamp = ros::Time::now();
    sat_fix.header.frame_id = "/lla";
    sat_fix.latitude = pos.latitude;
    sat_fix.longitude = pos.longitude;
    sat_fix.altitude = pos.height;
    sat_fix.position_covariance[0] = pow(cur_utm_bestpos_.easting_standard_deviation, 2);
    sat_fix.position_covariance[4] = pow(cur_utm_bestpos_.northing_standard_deviation, 2);
    sat_fix.position_covariance[8] = pow(cur_utm_bestpos_.height_standard_deviation, 2);



    psrpos_publisher_.publish(sat_fix);

  }

  void BestPositionEcefHandler(PositionEcef &best_xyz, double timestamp) {
    nav_msgs::Odometry ecef_pos;

    ecef_pos.header.stamp = ros::Time::now();
    ecef_pos.header.frame_id = "/ecef";
    ecef_pos.pose.pose.position.x = best_xyz.x_position;
    ecef_pos.pose.pose.position.y = best_xyz.y_position;
    ecef_pos.pose.pose.position.z = best_xyz.z_position;

    ecef_pos.pose.covariance[1] = pow(best_xyz.x_standard_deviation,2);
    ecef_pos.pose.covariance[8] = pow(best_xyz.y_standard_deviation,2);
    ecef_pos.pose.covariance[15] = pow(best_xyz.z_standard_deviation,2);

    ecef_pos.twist.twist.linear.x = best_xyz.x_velocity;
    ecef_pos.twist.twist.linear.y = best_xyz.y_velocity;
    ecef_pos.twist.twist.linear.z = best_xyz.z_velocity;

    ecef_pos.twist.covariance[1] = pow(best_xyz.x_velocity_standard_deviation,2);
    ecef_pos.twist.covariance[8] = pow(best_xyz.y_velocity_standard_deviation,2);
    ecef_pos.twist.covariance[15] = pow(best_xyz.z_velocity_standard_deviation,2);

    ecefpos_publisher_.publish(ecef_pos);

  }

  void RawMsgHandler(unsigned char *msg) {
    // ROS_INFO_STREAM("RAW RANGE MSG\n\tsizeof: " << sizeof(msg));
  }

  inline double deg2rad(const double& deg)
  {
     return deg*M_PI/180.0;
  }
  
  void InspvaxHandler(Inspvax &inspvax,double timestamp)
  {
	ros::Time stamp = ros::Time::now();
	if(inspvax_publisher_.getNumSubscribers())
	{
		gps_msgs::Inspvax inspvax_msg;
		inspvax_msg.header.stamp = stamp;
		inspvax_msg.header.frame_id = "gps";
		inspvax_msg.latitude = inspvax.latitude;
		inspvax_msg.longitude = inspvax.longitude;
		inspvax_msg.height = inspvax.height;
		inspvax_msg.undulation = inspvax.undulation;
		inspvax_msg.north_velocity = inspvax.north_velocity ;
		inspvax_msg.east_velocity = inspvax.east_velocity ;
		inspvax_msg.up_velocity = inspvax.up_velocity;
		inspvax_msg.roll = inspvax.roll;
		inspvax_msg.pitch = inspvax.pitch;
		inspvax_msg.azimuth = inspvax.azimuth;
		inspvax_msg.latitude_standard_deviation = inspvax.latitude_standard_deviation;
		inspvax_msg.longitude_standard_deviation = inspvax.longitude_standard_deviation;
		inspvax_msg.height_standard_deviation =  inspvax.height_standard_deviation;
		inspvax_msg.northing_standard_deviation = inspvax.northing_standard_deviation;
		inspvax_msg.easting_standard_deviation = inspvax.easting_standard_deviation;
		inspvax_msg.uping_standard_deviation = inspvax.uping_standard_deviation;
		inspvax_msg.roll_standard_deviation = inspvax.roll_standard_deviation;
		inspvax_msg.pitch_standard_deviation =inspvax.pitch_standard_deviation ;
		inspvax_msg.azimuth_standard_deviation = inspvax.azimuth_standard_deviation;
		inspvax_publisher_.publish(inspvax_msg);
	}

	if(ll2utm_publisher_.getNumSubscribers())
	{
		nav_msgs::Odometry ll2utm_msg;
		ll2utm_msg.header.stamp = stamp;
		ll2utm_msg.header.frame_id = "world";
		
		geographic_msgs::GeoPoint point;
		point.latitude = inspvax.latitude;
		point.longitude = inspvax.longitude;
		point.altitude = inspvax.height;
		
		geodesy::UTMPoint utm;
		geodesy::fromMsg(point, utm);
		
		ll2utm_msg.pose.pose.position.x = utm.easting;
		ll2utm_msg.pose.pose.position.y = utm.northing;
		ll2utm_msg.pose.pose.position.z = utm.altitude;
		
		double yaw = -deg2rad(inspvax.azimuth-90.0);
		double roll = deg2rad(inspvax.roll);
		double pitch = deg2rad(inspvax.pitch);
		
		Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
		Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
		Eigen::Quaterniond q = yawAngle * rollAngle* pitchAngle;
		
		ll2utm_msg.pose.pose.orientation.x = q.x();
		ll2utm_msg.pose.pose.orientation.y = q.y();
		ll2utm_msg.pose.pose.orientation.z = q.z();
		ll2utm_msg.pose.pose.orientation.w = q.w();
		ll2utm_msg.pose.covariance[0] = deg2rad(inspvax.azimuth);
		ll2utm_msg.pose.covariance[1] = inspvax.longitude;
		ll2utm_msg.pose.covariance[2] = inspvax.latitude;
		
		ll2utm_msg.pose.covariance[3] = yaw;
		ll2utm_msg.pose.covariance[4] = roll;
		ll2utm_msg.pose.covariance[5] = pitch;
		
		ll2utm_msg.pose.covariance[6] = inspvax.north_velocity;
		ll2utm_msg.pose.covariance[7] = inspvax.east_velocity;
		
		//float x_speed = inspvax.north_velocity * cos(inspvax.azimuth) + inspvax.east_velocity * sin(inspvax.azimuth) *3.6;
		//float y_speed = inspvax.north_velocity * cos(inspvax.azimuth) + inspvax.east_velocity * sin(inspvax.azimuth) *3.6;
		

		ll2utm_publisher_.publish(ll2utm_msg);
	}
	
  }
  
  void BestGnssHandler(BestGnss &bestgnss,double timestamp)
  {
	gps_msgs::Inspvax gnss_msg;
	gnss_msg.header.stamp = ros::Time::now();
	gnss_msg.header.frame_id = "gps";
	gnss_msg.latitude = bestgnss.latitude;
	gnss_msg.longitude = bestgnss.longitude;
	gnss_msg.height = bestgnss.height;
	
	bestgnss_publisher_.publish(gnss_msg);
 }
	
  
  void CorrImuShortHandler(CorrImuShort &corr_imu,double &timestamp)
  {
  	static int log_frequency = nh_.param<int>("log_corrimu_frequency",0);
  	
  	sensor_msgs::Imu corr_imu_msg;
  	corr_imu_msg.header.stamp = ros::Time::now();
  	corr_imu_msg.header.frame_id = "imu";
  	corr_imu_msg.angular_velocity.x = corr_imu.pitch_rate *log_frequency;
  	corr_imu_msg.angular_velocity.y = corr_imu.roll_rate *log_frequency;
  	corr_imu_msg.angular_velocity.z = corr_imu.yaw_rate *log_frequency;
  	
  	corr_imu_msg.linear_acceleration.x = corr_imu.lateral_acc *log_frequency;
  	corr_imu_msg.linear_acceleration.y = corr_imu.longitudinal_acc *log_frequency;
  	corr_imu_msg.linear_acceleration.z = corr_imu.verticle_acc *log_frequency;
  	corrImus_publisher_.publish(corr_imu_msg);
  }
  
  void RawImuShortHandler(RawImuShort &raw_imu, double &timestamp) 
  {
	sensor_msgs::Imu raw_imu_msg;
  	raw_imu_msg.header.stamp = ros::Time::now();
  	raw_imu_msg.header.frame_id = "imu";
  	raw_imu_msg.angular_velocity.x = raw_imu.x_gyro_rate* 0.008/65536;
  	raw_imu_msg.angular_velocity.y = -raw_imu.y_gyro_rate_neg* 0.008/65536;
  	raw_imu_msg.angular_velocity.z = raw_imu.z_gyro_rate* 0.008/65536;
  	
  	raw_imu_msg.linear_acceleration.x = raw_imu.x_acceleration *0.0002/65536 * 9.80665 ;
  	raw_imu_msg.linear_acceleration.y = -raw_imu.y_acceleration_neg *0.0002/65536 * 9.80665;
  	raw_imu_msg.linear_acceleration.z = raw_imu.z_acceleration *0.0002/65536 * 9.80665;
  	
  	//ROS_INFO("%d\t%d\t%d",raw_imu.x_gyro_rate,raw_imu.y_gyro_rate_neg ,raw_imu.z_gyro_rate);
  	//ROS_INFO("%lf\t%lf\t%lf",raw_imu_msg.angular_velocity.x,raw_imu_msg.angular_velocity.y,raw_imu_msg.angular_velocity.z);
  	
  	rawImu_publisher_.publish(raw_imu_msg);
  }
  
  
  void run() {

    if (!this->getParameters())
      return;
	
	this->ll2utm_publisher_ =  nh_.advertise<nav_msgs::Odometry>(ll2utm_topic_,0);
    this->bestutm_publisher_ = nh_.advertise<nav_msgs::Odometry>(bestutm_topic_,0);
    this->nav_sat_fix_publisher_ = nh_.advertise<sensor_msgs::NavSatFix>(nav_sat_fix_topic_,0);
    // ! FIXME - only advertise ephem/range if going to publish it.
    this->ephemeris_publisher_ = nh_.advertise<gps_msgs::Ephemeris>(ephemeris_topic_,0);
    this->dual_band_range_publisher_ = nh_.advertise<gps_msgs::L1L2Range>(dual_band_range_topic_,0);
    this->psrpos_publisher_ = nh_.advertise<sensor_msgs::NavSatFix>(psrpos_topic_,0);
    this->ecefpos_publisher_ = nh_.advertise<nav_msgs::Odometry>(ecefpos_topic_,0);
    this->inspvax_publisher_ = nh_.advertise<gps_msgs::Inspvax>(inspvax_topic_,0); //add by wendao
    this->bestgnss_publisher_ = nh_.advertise<gps_msgs::Inspvax>(bestgnss_topic_,0); //add by wendao
    this->corrImus_publisher_ = nh_.advertise<sensor_msgs::Imu>(corr_imu_topic_,0); //add by wendao
    this->rawImu_publisher_ = nh_.advertise<sensor_msgs::Imu>(raw_imu_topic_,0);

    //em_.setDataCallback(boost::bind(&EM61Node::HandleEmData, this, _1));
    
    
    gps_.Connect(port_,baudrate_);

    // configure default log sets
    if (gps_default_logs_period_>0) {
      // request default set of gps logs at given rate
      // convert rate to string
      ROS_INFO("Requesting default GPS messages: BESTUTMB, BESTVELB");
      std::stringstream default_logs;
      default_logs.precision(2);
      default_logs << "BESTUTMB ONTIME " << std::fixed << gps_default_logs_period_ << ";";
      default_logs << "BESTVELB ONTIME " << std::fixed << gps_default_logs_period_;
      gps_.ConfigureLogs(default_logs.str());
    }

    if (span_default_logs_period_>0) {
      ROS_INFO("Requesting default SPAN messages: INSPVAB, INSCOVB");
      // request default set of gps logs at given rate
      // convert rate to string
      std::stringstream default_logs;
      default_logs.precision(2);
      default_logs << "INSPVAB ONTIME " << std::fixed << span_default_logs_period_ << ";";
      default_logs << "INSCOVB ONTIME " << std::fixed << span_default_logs_period_;
      ROS_INFO_STREAM("default logs: " << default_logs);  
    gps_.ConfigureLogs(default_logs.str());
    }

    if (!ephem_log_.empty()) {
      std::stringstream default_logs;
      default_logs << "GPSEPHEMB " << std::fixed << ephem_log_;
      gps_.ConfigureLogs(default_logs.str());
    }

    if (range_default_logs_period_>0) {
      std::stringstream default_logs;
      default_logs.precision(2);
      default_logs << "RANGECMPB ONTIME " << std::fixed << range_default_logs_period_ << ";";
      gps_.ConfigureLogs(default_logs.str());
    }

    if (psrpos_default_logs_period_>0) {
      std::stringstream default_logs;
      default_logs.precision(2);
      default_logs << "PSRPOSB ONTIME " << std::fixed << psrpos_default_logs_period_ << ";";
      gps_.ConfigureLogs(default_logs.str());
    }

    // configure additional logs
    gps_.ConfigureLogs(log_commands_);

    // configure serial port (for rtk generally)
    if (configure_port_!="") {
      // string should contain com_port,baud_rate,rx_mode,tx_mode
      // parse message body by tokening on ","
      typedef boost::tokenizer<boost::char_separator<char> >
        tokenizer;
      boost::char_separator<char> sep(",");
      tokenizer tokens(configure_port_, sep);
      // set up iterator to go through token list
      tokenizer::iterator current_token=tokens.begin();
      std::string num_comps_string=*(current_token);
      int number_components=atoi(num_comps_string.c_str());
      // make sure the correct number of tokens were found
      int token_count=0;
      for(current_token=tokens.begin(); current_token!=tokens.end();++current_token)
      {
        token_count++;
      }

      if (token_count!=4) {
        ROS_ERROR_STREAM("Incorrect number of tokens in configure port parameter: " << configure_port_);
      } else {
        current_token=tokens.begin();
        std::string com_port = *(current_token++);
        int baudrate = atoi((current_token++)->c_str());
        std::string rx_mode = *(current_token++);
        std::string tx_mode = *(current_token++);

        ROS_INFO_STREAM("Configure com port baud rate and interface mode for " << com_port << ".");
        gps_.ConfigureInterfaceMode(com_port,rx_mode,tx_mode);
        gps_.ConfigureBaudRate(com_port,baudrate);
      }

    }

    ros::spin();

  } // function

protected:

  void disconnect() {
    //em_.stopReading();
    //em_.disconnect();
    gps_.SendCommand("UNLOGALL THISPORT");
  }

  bool getParameters() {

    name_ = ros::this_node::getName();

    nh_.param("bestutm_topic", bestutm_topic_, std::string("/best_utm"));
    ROS_INFO_STREAM(name_ << ": Odom Topic: " << bestutm_topic_);

    nh_.param("nav_sat_fix_topic", nav_sat_fix_topic_, std::string("/gps_fix"));
    ROS_INFO_STREAM(name_ << ": NavSatFix Topic: " << nav_sat_fix_topic_);

    nh_.param("ephemeris_topic", ephemeris_topic_, std::string("/ephemeris"));
    ROS_INFO_STREAM(name_ << ": Ephemeris Topic: " << ephemeris_topic_);

    nh_.param("dual_band_range_topic", dual_band_range_topic_, std::string("/range"));
    ROS_INFO_STREAM(name_ << ": L1L2Range Topic: " << dual_band_range_topic_);

    nh_.param("port", port_, std::string("/dev/ttyUSB0"));
    ROS_INFO_STREAM(name_ << ": Port: " << port_);

    nh_.param("baudrate", baudrate_, 115200);
    ROS_INFO_STREAM(name_ << ": Baudrate: " << baudrate_);

    //nh_.param("log_commands", log_commands_, std::string("BESTUTMB ONTIME 1.0"));
    nh_.param("log_commands", log_commands_, std::string(""));
    ROS_INFO_STREAM(name_ << ": Log Commands: " << log_commands_);

    nh_.param("configure_port", configure_port_, std::string(""));
    ROS_INFO_STREAM(name_ << ": Configure port: " << configure_port_);

    nh_.param("gps_default_logs_period", gps_default_logs_period_, 0.0);
    ROS_INFO_STREAM(name_ << ": Default GPS logs period: " << gps_default_logs_period_);

    nh_.param("span_default_logs_period", span_default_logs_period_, 0.0);
    ROS_INFO_STREAM(name_ << ": Default SPAN logs period: " << span_default_logs_period_);

    nh_.param("ephem_log", ephem_log_, std::string(""));
    if (!ephem_log_.empty())
      ROS_INFO_STREAM(name_ << ": Ephemeris logging enabled: " << ephem_log_);

    nh_.param("range_default_logs_period", range_default_logs_period_, 0.00);
    ROS_INFO_STREAM(name_ << ": Default Range logs period: " << range_default_logs_period_);

    nh_.param("psrpos_default_logs_period", psrpos_default_logs_period_, 0.0);
    ROS_INFO_STREAM(name_ << ": Default Pseudorange Position logs period: " << psrpos_default_logs_period_);

    nh_.param("psrpos_topic", psrpos_topic_, std::string("gps_fix_psr"));
    ROS_INFO_STREAM(name_ << ": Pseudorange Position Topic: " << psrpos_topic_);


    nh_.param("ecefpos_topic", ecefpos_topic_, std::string("gps_fix_ecef"));
    ROS_INFO_STREAM(name_ << ": ECEF Position Topic: " << ecefpos_topic_);

// add by wendao     
    nh_.param("inspvax_topic", inspvax_topic_, std::string("/gps"));
    ROS_INFO_STREAM(name_ << ": GPS Inspvax Topic: " << inspvax_topic_);
    
    nh_.param("bestgnss_topic", bestgnss_topic_, std::string("best_gnss"));
    ROS_INFO_STREAM(name_ << ": GPS BestGnss Topic: " << bestgnss_topic_);

	nh_.param("corr_imu_topic",corr_imu_topic_,std::string("/corr_imu"));
	ROS_INFO_STREAM(name_ << ": GPS corrImu Topic: " << corr_imu_topic_);
	
	nh_.param("raw_imu_topic",raw_imu_topic_,std::string("/raw_imu"));
	ROS_INFO_STREAM(name_ << ": GPS rawImu Topic: " << raw_imu_topic_);
	
	is_ll2utm_ = nh_.param<bool>("is_lltoutm",false);
	ll2utm_topic_ = nh_.param<std::string>("ll2utm_topic","ll2utm_odom");
	
    return true;
  }

  ////////////////////////////////////////////////////////////////
  // ROSNODE Members
  ////////////////////////////////////////////////////////////////
  ros::NodeHandle nh_;
  std::string name_;
  ros::Publisher ll2utm_publisher_;
  ros::Publisher bestutm_publisher_;
  ros::Publisher nav_sat_fix_publisher_;
  ros::Publisher ephemeris_publisher_;
  ros::Publisher dual_band_range_publisher_;
  ros::Publisher psrpos_publisher_;
  ros::Publisher ecefpos_publisher_;
  ros::Publisher inspvax_publisher_;  //add by wendao
   ros::Publisher bestgnss_publisher_; //add by wendao
  ros::Publisher corrImus_publisher_;  //add by wendao
  ros::Publisher rawImu_publisher_;

  Novatel gps_; //

  // topics - why are we not using remap arguments?
  std::string bestutm_topic_;
  std::string ll2utm_topic_;
  std::string nav_sat_fix_topic_;
  std::string ephemeris_topic_;
  std::string dual_band_range_topic_;
  std::string psrpos_topic_;
  std::string ecefpos_topic_;
  std::string inspvax_topic_; //add by wendao 
  std::string bestgnss_topic_;
  std::string corr_imu_topic_; //add by wendao
  std::string raw_imu_topic_; 

  std::string port_;
  std::string log_commands_;
  std::string configure_port_;
  double gps_default_logs_period_;
  double span_default_logs_period_;
  double range_default_logs_period_;
  double psrpos_default_logs_period_;
  std::string ephem_log_;
  int baudrate_;
  double poll_rate_;

  Velocity cur_velocity_;
  // InsCovarianceShort cur_ins_cov_;
  gps_msgs::Ephemeris cur_ephem_;
  gps_msgs::L1L2Range cur_range_;
  InsCovariance cur_ins_cov_;
  Position cur_psrpos_;
  UtmPosition cur_utm_bestpos_;
  
  bool is_ll2utm_;

  // holders for data common to multiple messages
  // double cur_psr_lla_[3];
  // double cur_psr_lla_std_[3];
  // double cur_utm_[3];
  // double cur_utm_std_[3];

}; // end class NovatelNode

int main(int argc, char **argv) {
  ros::init(argc, argv, "novatel_node");
  
  NovatelNode node;
  
  node.run();
  
  return 0;
}
