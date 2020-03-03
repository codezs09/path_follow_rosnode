#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"

constexpr int LOOP_RATE = 100; //processing frequency

class OdomRelay
{
private:
	geometry_msgs::TwistStamped twist;

public:
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
	{
		twist.header = msg->header;
		twist.twist = msg->twist.twist;
	}

	geometry_msgs::TwistStamped outputTwist()
	{
		return twist;
	}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "odom_relay");
	ros::NodeHandle nh; 

	OdomRelay odm1; 

	ros::Subscriber subOdom = nh.subscribe("odom",10, &OdomRelay::odomCallback, &odm1);
	ros::Publisher pubTwist = nh.advertise<geometry_msgs::TwistStamped>("current_velocity", 10);

	ros::Rate loop_rate(LOOP_RATE);
	while (ros::ok())
	{
		ros::spinOnce();
		pubTwist.publish(odm1.outputTwist());

		loop_rate.sleep();
	}
}


