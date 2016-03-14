/* 
 * bump_control_lidar.cpp
 *
 *  Created on: 02.03.2016
 *      Author: Nicko_Dema@protonmail.com
 *		ITMO University
 *		Robotics Engineering Department
 */

#include <bump_control_lidar.h>

BumpControl::BumpControl():
    nh("~")
{
    laser_scan_sub = nh.subscribe( "/scan", 5, &BumpControl::lidarCallback, this );
    cmd_vel_sub =    nh.subscribe( "/bc_cmd_vel", 1, &BumpControl::cmdCallback, this );
    cmd_vel_pub =    nh.advertise<geometry_msgs::Twist>( "/cmd_vel", 1, true );

    sects_init();
}

void BumpControl::spin()
{
    ros::Rate rate(40);
    while( nh.ok() )
    {
	ros::spinOnce();
	rate.sleep();
    }
}

void BumpControl::lidarCallback( const sensor_msgs::LaserScanConstPtr& lidar_msg )
{   
   /**
    * In this part of callback the incoming array is divided into 18 sectors.
    * For each sector is calculated minimum value.
    */
    sensor_msgs::LaserScan msg = *lidar_msg; 
    int correct = 0;
    int boundary = 0;
    sec_points = num_points / 18;
    for( int i=0; i<=17; i++ )
    {
	float min_dis = 5.8;
	if( i == 0 || i == 17 )
	    boundary = num_points % 18 / 2;
	else
	{
	    correct = num_points % 18 / 2;
	    boundary = 0;
	}

	for( int j=0; j<=sec_points+boundary; j++ )
	{
	    float point = msg.ranges[i*sec_points+j+correct];
	    if( ( min_dis > point )  && ( point > msg.range_min ) && ( point < msg.range_max ) )
	    {
	    	min_dis = point;
	    }
	}
	arr_sects[i].min_dis = (double) min_dis;
    }
    
    /**
     * This implements a PI controller of velocity. 
     * You may change the proportional coefficients 
     * depending on the control parameters of your system.
     */
    for( int i=0; i<=17; i++ )
    {
	if( arr_sects[i].min_dis <= MIN_SAF_DIS )
	{
	    arr_sects[i].x_linear_intgr = arr_sects[i].x_linear_intgr + (-1)*arr_sects[i].x_linear_scale / arr_sects[i].min_dis/350;
	    if( i<=8 )
	    {
		arr_sects[i].y_linear_intgr = arr_sects[i].y_linear_intgr + arr_sects[i].y_linear_scale / arr_sects[i].min_dis/310;
		arr_sects[i].angular_intgr  = arr_sects[i].angular_intgr + arr_sects[i].angular_scale / arr_sects[i].min_dis/270;
	    }
	    else
	    {
		arr_sects[i].y_linear_intgr = arr_sects[i].y_linear_intgr + (-1)*arr_sects[i].y_linear_scale / arr_sects[i].min_dis/310;
		arr_sects[i].angular_intgr  = arr_sects[i].angular_intgr + (-1)*arr_sects[i].angular_scale / arr_sects[i].min_dis/270;
	    }
	}
	else
	{
	    arr_sects[i].x_linear_intgr = 0;
	    arr_sects[i].y_linear_intgr = 0;
	    arr_sects[i].angular_intgr = 0;
	}
    }
    
    for( int i=0; i<=17;i++ )
    {
	cmd_vel_msg.linear.x = cmd_vel_msg.linear.x + arr_sects[i].x_linear_intgr;
	cmd_vel_msg.linear.y  = cmd_vel_msg.linear.y + arr_sects[i].y_linear_intgr;
	cmd_vel_msg.angular.z = cmd_vel_msg.angular.z + arr_sects[i].angular_intgr;
    }

}

void BumpControl::cmdCallback( const geometry_msgs::TwistConstPtr& msg )
{
    cmd_vel_msg.linear.x =  msg->linear.x  + cmd_vel_msg.linear.x;
    cmd_vel_msg.linear.y =  msg->linear.y  + cmd_vel_msg.linear.y;
    cmd_vel_msg.angular.z = msg->angular.z + cmd_vel_msg.angular.z; 
    cmd_vel_pub.publish( cmd_vel_msg );
    cmd_vel_msg.linear.x  = 0.0;
    cmd_vel_msg.linear.y  = 0.0;
    cmd_vel_msg.angular.z = 0.0;
}

void BumpControl::sects_init()
{
    sensor_msgs::LaserScanConstPtr once_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>( "/scan" );
    num_points = once_msg->ranges.size();
    for( int i=0; i <= 17; i++ )
    {
	arr_sects[i].min_dis = 6;
	arr_sects[i].less = false;
	switch( i )
	{
	    case 0: 
		    arr_sects[i].x_linear_scale = 0.0;
		    arr_sects[i].y_linear_scale = 2.0;
		    arr_sects[i].angular_scale  = 0.0;
		    break;
	    case 1: 
		    arr_sects[i].x_linear_scale = 0.0;
		    arr_sects[i].y_linear_scale = 1.0;
		    arr_sects[i].angular_scale  = 0.2;
		    break;
	    case 2: 
		    arr_sects[i].x_linear_scale = 0.0;
		    arr_sects[i].y_linear_scale = 0.5;
		    arr_sects[i].angular_scale  = 0.5;
		    break;
	    case 3: 
		    arr_sects[i].x_linear_scale = 0.0;
		    arr_sects[i].y_linear_scale = 0.0;
		    arr_sects[i].angular_scale  = 1.0;
		    break;
	    case 4: 
		    arr_sects[i].x_linear_scale = 0.0;
		    arr_sects[i].y_linear_scale = 0.0;
		    arr_sects[i].angular_scale  = 1.0;
		    break;
	    case 5: 
		    arr_sects[i].x_linear_scale = 0.0;
		    arr_sects[i].y_linear_scale = 0.0;
		    arr_sects[i].angular_scale  = 1.0;
		    break;
	    case 6: 
		    arr_sects[i].x_linear_scale = 0.5;
		    arr_sects[i].y_linear_scale = 0.0;
		    arr_sects[i].angular_scale  = 2.0;
		    break;
	    case 7: 
		    arr_sects[i].x_linear_scale = 1.0;
		    arr_sects[i].y_linear_scale = 0.0;
		    arr_sects[i].angular_scale  = 2.0;
		    break;
	    case 8: 
		    arr_sects[i].x_linear_scale = 2.0;
		    arr_sects[i].y_linear_scale = 0.0;
		    arr_sects[i].angular_scale  = 2.0;
		    break;
	    case 9: 
		    arr_sects[i].x_linear_scale = 2.0;
		    arr_sects[i].y_linear_scale = 0.0;
		    arr_sects[i].angular_scale  = 2.0;
		    break;
	    case 10: 
		    arr_sects[i].x_linear_scale = 1.0;
		    arr_sects[i].y_linear_scale = 0.0;
		    arr_sects[i].angular_scale  = 2.0;
		    break;
	    case 11: 
		    arr_sects[i].x_linear_scale = 0.5;
		    arr_sects[i].y_linear_scale = 0.0;
		    arr_sects[i].angular_scale  = 2.0;
		    break;
	    case 12: 
		    arr_sects[i].x_linear_scale = 0.0;
		    arr_sects[i].y_linear_scale = 0.0;
		    arr_sects[i].angular_scale  = 1.0;
		    break;
	    case 13: 
		    arr_sects[i].x_linear_scale = 0.0;
		    arr_sects[i].y_linear_scale = 0.0;
		    arr_sects[i].angular_scale  = 1.0;
		    break;
	    case 14: 
		    arr_sects[i].x_linear_scale = 0.0;
		    arr_sects[i].y_linear_scale = 0.0;
		    arr_sects[i].angular_scale  = 1.0;
		    break;
	    case 15: 
		    arr_sects[i].x_linear_scale = 0.0;
		    arr_sects[i].y_linear_scale = 0.5;
		    arr_sects[i].angular_scale  = 0.5;
		    break;
	    case 16: 
		    arr_sects[i].x_linear_scale = 0.0;
		    arr_sects[i].y_linear_scale = 1.0;
		    arr_sects[i].angular_scale  = 0.2;
		    break;
	    case 17: 
		    arr_sects[i].x_linear_scale = 0.0;
		    arr_sects[i].y_linear_scale = 2.0;
		    arr_sects[i].angular_scale  = 0.0;
		    break;
	}
    }
}

BumpControl::~BumpControl()
{
    cmd_vel_pub.shutdown();
    cmd_vel_sub.shutdown();
    laser_scan_sub.shutdown();
}
