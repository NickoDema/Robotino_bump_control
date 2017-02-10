/* bump_control_lidar_node.cpp
 *
 *  Created on: 02.03.2016
 *      Author: Nicko_Dema@protonmail.com
 *		ITMO University
 *		Robotics Engineering Department
 */

#include "bump_control_lidar.h"

int main( int argc, char **argv)
{
    ros::init( argc, argv, "bump_control" );
    BumpControl robotino;
    robotino.spin();

    return 0;
}
