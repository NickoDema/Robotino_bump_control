/* 
 * bump_control_lidar.h
 *
 *  Created on: 02.03.2016
 *      Author: Nicko_Dema@protonmail.com
 *		ITMO University
 *		Robotics Engineering Department
 */

#ifndef BUMPCONTROLLIDAR_H_
#define BUMPCONTROLLIDAR_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#define MIN_SAF_DIS 0.7		    //0.27 for robotino

class BumpControl
{
  public:
          BumpControl();
	  ~BumpControl();

	  void spin();

  private:
          ros::NodeHandle nh;

	  ros::Publisher  cmd_vel_pub;
	  ros::Subscriber cmd_vel_sub;
	  ros::Subscriber laser_scan_sub;

	  void cmdCallback( const geometry_msgs::TwistConstPtr& msg );
	  void lidarCallback( const sensor_msgs::LaserScanConstPtr& msg );

	  void sects_init();

	  geometry_msgs::Twist cmd_vel_msg;

	  int num_points;						    //number of points (msg)
	  int sec_points;						    //number of points in the sector
	  struct sector
	  {
	      double min_dis;						    //min distance in the sector
	      bool less;
	      double x_linear_scale;
	      double y_linear_scale;
	      double angular_scale;
	      double x_linear_intgr;		//integrators
	      double y_linear_intgr;
	      double angular_intgr;
	  };
	  sector arr_sects[17];
};

#endif /* BUMPCONTROLLIDAR_H_ */
