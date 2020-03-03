#ifndef PIDFOLLOW_H
#define PIDFOLLOW_H

//c++ header
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <cmath>
#include <algorithm>


// ROS includes
//#include <tf2_ros/transform_listener.h>
//#include <tf2_ros/transform_broadcaster.h>
#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "autoware_msgs/VehicleCmd.h"
// C++ includes
#include <memory>
#include "libwaypoint_follower.h"


//pid path following algorithm
//#include <nav_msgs/Odometry.h>

/*

*/

class PIDFollow
{
private:
    double lookahead_distance, constant_lookahead_distance; //lookahead_distance_ls
    double err_phi=0;     // heading error, [rad]
    double err_lat=0;     // lateral error, [m]
    double err_total=0;   // err_total = err_lat+ls*err_phi
    double delta_f=0;     // front steering angle, [rad]
    double delta_f_norm=0;    // contain delta_f within [-1, 1] for LGSVL simulator to use
    double phi_r;       // reference heading
    double yaw;         // current vehicle heading 
    double phi1;
    double phi2;
    double ratio;

    const double DELTA_F_MAX = 36.4*M_PI/180.0;    // maximum front steering angle could achieved, [rad]

    double integral_error;
    double v_max, VELOCITY;
    double max_fw_steering;
    double delta;    
    double steering_ratio;
    double displacement_threshold_;
    double relative_angle_threshold_;

    int num_of_next_waypoint_;

    bool goal_reached, waypoint_set_, currentpose_set_, velocity_set_, lookahead_distance_set;
 
    bool curve_fit, adaptive_lookahead, velocity_configured_;

    double Kp, Ki, K_ld;


    autoware_msgs::VehicleCmd vehiclecmd;
    geometry_msgs::TwistStamped current_twist_;
    geometry_msgs::PoseStamped current_pose_;

    WayPoints current_waypoints_;

    //ros infrastructure
    ros::NodeHandle nh_, nh_private_;
    ros::Subscriber sub_currentpose, sub_currentvel, sub_finalwaypoint;
    
    // tf2_ros::Buffer tf_buffer_;
    // tf2_ros::TransformListener tf_listener_;
    // tf2_ros::TransformBroadcaster tf_broadcaster_;

    //geometry_msgs::TransformStamped lookahead_;
    geometry_msgs::Point position_of_next_target_;

    std::string map_frame_id_, robot_frame_id_;

    
    void ComputeLHDistance(bool set_lookahead);

    bool calcDif(WayPoints wp, geometry_msgs::PoseStamped current_pose);

    double ComputeSteering(); 

    //double getCmdVelocity() const;

    autoware_msgs::VehicleCmd outputZero() const;

    //double calcCurvature(geometry_msgs::Point target) const;

    //double calcRadius(geometry_msgs::Point target) const;


    bool interpolateNextTarget(int next_waypoint, geometry_msgs::Point *next_target) const;

    double getCmdVelocity(int waypoint) ;

    bool verifyFollowing() const;

    void getNextWaypoint();

    autoware_msgs::VehicleCmd setVehicleCommand(double velocity, double steering_angle) const;



 public:

    //constructor
    PIDFollow(bool curve_fit, bool adaptive_lookahead, bool configure_velocity, double velocity, double kp, double ki, double k_ld,double lh) //get param from main
    : v_max(10.0)
    , VELOCITY(velocity)
    , err_phi(0.0)
    , Kp(kp)
    , Ki(ki)
    , K_ld(0.0)
    , displacement_threshold_(0.2)
    , relative_angle_threshold_(5.0)
    , max_fw_steering(470)
    , steering_ratio(14.8)
    , constant_lookahead_distance(lh)
    , num_of_next_waypoint_(-1)
    , goal_reached(false)
    , lookahead_distance_set(false)
    , waypoint_set_(false)
    , currentpose_set_(false)
    , velocity_set_(false)
    , curve_fit(curve_fit)
    , adaptive_lookahead(adaptive_lookahead)
    , velocity_configured_(configure_velocity)
    {
    }
  //destructor
  ~PIDFollow()
  {
    
  }



    void CurrentPathCallback(const styx_msgs::LaneConstPtr &msg);
    
    void CurrentPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    //TwistStampedConstPtr
    void CurrentTwistCallback(const geometry_msgs::TwistStampedConstPtr &msg);
    //TODO add a function for closet waypoint

    std_msgs::Float64 outputHeadingErr() const;
    std_msgs::Float64 outputLateralErr() const;
    std_msgs::Float64 outputSteering() const;
    void publishDebugFloat(const std::string &, double);

    geometry_msgs::Point getPoseOfNextWaypoint() const
    {
    return current_waypoints_.getWaypointPosition(num_of_next_waypoint_);
    }
    geometry_msgs::Point getPoseOfNextTarget() const
    {
    return position_of_next_target_;
    }
    geometry_msgs::Pose getCurrentPose() const
    {
    return current_pose_.pose;
    }
    autoware_msgs::VehicleCmd outputzero() const;
    /*double getLookaheadDistance() const
    {
    return lookahead_distance_;
    }*/
    // processing for ROS
    autoware_msgs::VehicleCmd run();

    /****************************************************************/
    /**************************DEBUG INFORMAITON************************/
    /****************************************************************/
    std_msgs::Float64 debugYaw() const;
    std_msgs::Float64 debugPhir() const;
    std_msgs::Float64 debugPhi1() const;
    std_msgs::Float64 debugPhi2() const;
    std_msgs::Float64 debugRatio() const;
    /****************************************************************/
    /****************************************************************/  

};


template<class T>
const T& clamp( const T& v, const T& lo, const T& hi )
{
    assert( !(hi < lo) );
    return (v < lo) ? lo : (hi < v) ? hi : v;
}

template<class T>
T dotProduct( const T (*a), const T (*b) )
{
  return (*a)*(*b) + (*(a+1)) * (*(b+1));
}

template<class T> 
T distance( const T (*a), const T (*b) )
{
  return sqrt( pow( *a-*b, 2) + pow( *(a+1)-*(b+1) , 2) );
}

// angleMinus(ang1, ang2) return the angular difference of (ang1-ang2) within [-pi, pi]
template<class T>
T angleMinus( const T& ang1, const T& ang2 )
{
    T err = ang1 - ang2;
    return err - 2*M_PI*floor( (err+M_PI) / (2*M_PI) ); // limit within [-PI, PI]
}


double quaternionMsgToYaw( geometry_msgs::Quaternion );


#endif // PIDFOLLOW_H
