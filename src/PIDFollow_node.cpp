#include "PIDFollow.h"

constexpr int LOOP_RATE = 30; //processing frequency

int main(int argc, char **argv)
{
  // set up ros
  ros::init(argc, argv, "path_follow");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  bool curve_fit, adaptive_lookahead, configure_velocity;
  double velocity, kp, ki, k_ld, lh;
  private_nh.param("curve_fit", curve_fit, bool(false));
  private_nh.param("adaptive_lookahead", adaptive_lookahead, bool(false));
  private_nh.param("configure_velocity", configure_velocity, bool(false));
  private_nh.param("velocity", velocity, double(10.0));
  private_nh.param("kp", kp, double(0.6));
  private_nh.param("ki", ki, double(0.0));
  private_nh.param("k_ld", k_ld, double(0.0));
  private_nh.param("lh", lh, double(4.0));
  //ROS_INFO_STREAM("linear_interpolate_mode : " << linear_interpolate_mode);

  PIDFollow pf(curve_fit, adaptive_lookahead, configure_velocity, velocity, kp, ki, k_ld , lh);

  ROS_INFO("set publisher...");
  // publish topic
  ros::Publisher cmd_velocity_publisher = nh.advertise<autoware_msgs::VehicleCmd>("vehicle_cmd", 10);
  ros::Publisher pub_steering = nh.advertise<std_msgs::Float64>("front_steering", 10);
  ros::Publisher pub_deltaphi = nh.advertise<std_msgs::Float64>("angle_error", 10);
  ros::Publisher pub_lateralerror = nh.advertise<std_msgs::Float64>("lateral_error", 10);

  /****************************************************************/
  /* debug publishers */
  ros::Publisher pub_debug_yaw = nh.advertise<std_msgs::Float64>("yaw", 10);
  ros::Publisher pub_debug_phir = nh.advertise<std_msgs::Float64>("phir", 10);
  ros::Publisher pub_debug_phi1 = nh.advertise<std_msgs::Float64>("phi1", 10);
  ros::Publisher pub_debug_phi2 = nh.advertise<std_msgs::Float64>("phi2", 10);
  ros::Publisher pub_debug_ratio = nh.advertise<std_msgs::Float64>("ratio", 10);
  /****************************************************************/


  ROS_INFO("set subscriber...");
  // subscribe topic
  ros::Subscriber waypoint_subscriber =
      nh.subscribe("final_waypoints", 1, &PIDFollow::CurrentPathCallback, &pf);
  ros::Subscriber ndt_subscriber =
      nh.subscribe("gnss_pose", 1, &PIDFollow::CurrentPoseCallback, &pf);
  ros::Subscriber est_twist_subscriber =
      nh.subscribe("current_velocity", 1, &PIDFollow::CurrentTwistCallback, &pf);

  ROS_INFO("PID path follow start");
  ros::Rate loop_rate(LOOP_RATE);
  while (ros::ok())
  {
    ros::spinOnce();
    cmd_velocity_publisher.publish(pf.run());
    pub_steering.publish(pf.outputSteering());
    pub_deltaphi.publish(pf.outputHeadingErr());
    pub_lateralerror.publish(pf.outputLateralErr());

    pub_debug_yaw.publish(pf.debugYaw());
    pub_debug_phir.publish(pf.debugPhir());
    pub_debug_phi1.publish(pf.debugPhi1());
    pub_debug_phi2.publish(pf.debugPhi2());
    pub_debug_ratio.publish(pf.debugRatio());

    loop_rate.sleep();
  }

  return 0;
}
