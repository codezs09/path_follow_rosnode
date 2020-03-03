#include "ros/ros.h"
#include "PIDFollow.h"


void PIDFollow::CurrentPathCallback(const styx_msgs::LaneConstPtr &msg)
{
    current_waypoints_.setPath(*msg);
    waypoint_set_ =true;
}

void PIDFollow::CurrentPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    current_pose_.header = msg -> header;
    current_pose_.pose = msg -> pose;
    currentpose_set_ = true;
}

void PIDFollow::CurrentTwistCallback(const geometry_msgs::TwistStampedConstPtr &msg)
{
    current_twist_ = *msg;
    velocity_set_ = true;
}

void PIDFollow::getNextWaypoint()
{
  int path_size = static_cast<int>(current_waypoints_.getSize());

  // if waypoints are not given, do nothing.
  if (path_size == 0)
  {
    num_of_next_waypoint_ = -1;
    return;
  }
   // look for the next waypoint.
  for (int i = 0; i < path_size; i++)
  {
    // if search waypoint is the last
    if (i == (path_size - 1))
    {
      ROS_INFO("search waypoint is the last");
      num_of_next_waypoint_ = i;
      return;
    }

    // if there exists an effective waypoint
    if (getPlaneDistance(current_waypoints_.getWaypointPosition(i), current_pose_.pose.position) > lookahead_distance)
    {
      num_of_next_waypoint_ = i;
      //ROS_ERROR_STREAM("wp = " << i << " dist = " << getPlaneDistance(current_waypoints_.getWaypointPosition(i), current_pose_.pose.position) );
      return;
    }
  }

  // if this program reaches here , it means we lost the waypoint!
  num_of_next_waypoint_ = -1;
  return;
}

void PIDFollow::ComputeLHDistance(bool set_lookahead)
{
    double velocity = current_twist_.twist.linear.x;
    if (!set_lookahead)
        {
            lookahead_distance = constant_lookahead_distance;
        }

     else
        {
            lookahead_distance = K_ld*velocity;
        }
        return;

}
double PIDFollow::ComputeSteering() 
{
    //TODO later change the lookahead distance and proportional gain adaptive to speed
    double delta_f_norm;
    double dt = 0.03;
    integral_error += err_total*dt;
    delta_f = -Kp*err_total - Ki*integral_error;    

    delta_f_norm = clamp( delta_f/DELTA_F_MAX, -1.0, 1.0 );  // normalize to within [-1, 1]
    return delta_f_norm;
}


double PIDFollow::getCmdVelocity(int waypoint) 
{
  double desired_velocity;
  if (current_waypoints_.isEmpty())
  {
    ROS_INFO_STREAM("waypoint : not loaded path");
    return 0;
  }
  ROS_INFO_STREAM("velocity_configured = " << velocity_configured_);
  if (!velocity_configured_)
  {
       desired_velocity = current_waypoints_.getWaypointVelocityMPS(waypoint);
  }
  else 
       desired_velocity = 2.0;
  
  // ROS_INFO_STREAM("waypoint : " << mps2kmph(velocity) << " km/h ( " << velocity << "m/s )");
  return desired_velocity;
}



bool PIDFollow::calcDif(WayPoints wp, geometry_msgs::PoseStamped current_pose) 
{
    // int closest_waypoint_index = getClosestWaypoint(wp.getCurrentWaypoints(), current_pose.pose);
    int closest_waypoint_index = 0;
    //ROS_INFO_STREAM("index = " << closest_waypoint_index);
    //ROS_INFO_STREAM("current pose in calcDif is " << current_pose);
    if (closest_waypoint_index == -1)
        {
          return false;
        }

    //int current_waypoint = 0;
    int num_of_proceed = 1;
    //TODO decide num of proceed based on waypoint record frequency;
    geometry_msgs::Point  start =  current_waypoints_.getWaypointPosition(closest_waypoint_index) ;
    geometry_msgs::Point  end =  current_waypoints_.getWaypointPosition(closest_waypoint_index + num_of_proceed);
    geometry_msgs::Quaternion orient_start = current_waypoints_.getWaypointOrientation(closest_waypoint_index);
    geometry_msgs::Quaternion orient_end = current_waypoints_.getWaypointOrientation(closest_waypoint_index + num_of_proceed);
    geometry_msgs::Point  current_position = current_pose.pose.position;
    double a = 0;
    double b = 0;
    double c = 0;

    bool get_linear_flag = getLinearEquation(start, end, &a, &b, &c);
    if (!get_linear_flag)
        return false;
    
    double error =  getDistanceBetweenLineAndPoint(current_position, a, b, c);
    double d = (current_position.x - start.x)*(end.y - start.y)-(current_position.y - start.y)*(end.x - start.x);
    if (d <= 0)
    {
      err_lat = error;
    }
    else
    {
      err_lat = -error;
    }
    
    //get the heading angle phi_r, reference heading from the waypoints
    // double angle_path_y = end.y - start.y;
    // double angle_path_x = end.x - start.x;
    // double phi_r = atan2 (angle_path_y,angle_path_x);

    // first calculate the reflect length of vehicle current position on the segment
    double p1[2] = {start.x, start.y};
    double p2[2] = {end.x, end.y};
    double p_cur[2] = { current_position.x, current_position.y};
    phi1 = quaternionMsgToYaw( orient_start );
    phi2 = quaternionMsgToYaw( orient_end );
    yaw = quaternionMsgToYaw( current_pose.pose.orientation ); 

    double vec_a[2] = { p2[0]-p1[0], p2[1]-p1[1] };
    double vec_b[2] = { p_cur[0]-p1[0], p_cur[1]-p1[1] };

    ratio = dotProduct( vec_a, vec_b )/pow( distance(p1, p2), 2 );
    ratio = clamp(ratio, 0.0, 1.0);
    phi_r = phi1 + ratio*angleMinus(phi2, phi1);

    err_phi = angleMinus( yaw, phi_r );
    // err_phi= fmod((yaw - phi_r + M_PI),(2*M_PI)) - M_PI; // not working (rounded to zero)

    err_total = err_lat + lookahead_distance*sin(err_phi); 

    ROS_INFO("\n");
    ROS_INFO_STREAM("err_lat [m]:\t" << err_lat);
    ROS_INFO_STREAM("err_phi [deg]:\t" << err_phi*180.0/M_PI);
    ROS_INFO_STREAM("err_total [m]:\t" << err_total);
    ROS_INFO_STREAM("ratio:\t\t" << ratio);
    ROS_INFO_STREAM("phi2-phi1 = [deg]:\t" << angleMinus(phi2, phi1)*180.0/M_PI );

    return true;
}


bool PIDFollow::verifyFollowing() const
{
  double a = 0;
  double b = 0;
  double c = 0;
  getLinearEquation(current_waypoints_.getWaypointPosition(1), current_waypoints_.getWaypointPosition(2), &a, &b, &c);
  double displacement = getDistanceBetweenLineAndPoint(current_pose_.pose.position, a, b, c);
  double relative_angle = getRelativeAngle(current_waypoints_.getWaypointPose(1), current_pose_.pose);
  //ROS_ERROR("side diff : %lf , angle diff : %lf",displacement,relative_angle);
  if (displacement < displacement_threshold_ && relative_angle < relative_angle_threshold_)
  {
    // ROS_INFO("Following : True");
    return true;
  }
  else
  {
    // ROS_INFO("Following : False");
    return false;
  }
}

autoware_msgs::VehicleCmd PIDFollow::setVehicleCommand(double velocity, double steering_angle) const
{

  autoware_msgs::VehicleCmd vehicle_cmd;
  double steering;
  double linear_velocity;

  linear_velocity = velocity < v_max ? velocity : v_max;
  steering = clamp(steering_angle, -1.0, 1.0);   

  vehicle_cmd.header.stamp = ros::Time::now();
  vehicle_cmd.ctrl_cmd.linear_velocity = linear_velocity;
  // the acc should be within the range of comfort passengers
  if (current_twist_.twist.linear.x <= linear_velocity)
  {
    vehicle_cmd.ctrl_cmd.linear_acceleration = 0.5;
  }
  vehicle_cmd.ctrl_cmd.linear_acceleration = 0.0;
  vehicle_cmd.ctrl_cmd.steering_angle = -1*steering;
  vehicle_cmd.gear = 64;
  return vehicle_cmd;
}

autoware_msgs::VehicleCmd PIDFollow::outputZero() const
{
  autoware_msgs::VehicleCmd vmd;
  vmd.ctrl_cmd.linear_velocity = 0.0;
  vmd.ctrl_cmd.steering_angle = 0.0;
  vmd.header.stamp = ros::Time::now();
  return vmd;
}

std_msgs::Float64 PIDFollow::outputHeadingErr() const
{
   std_msgs::Float64 msg;
   msg.data = err_phi*180.0/M_PI;
   return msg;
}

std_msgs::Float64 PIDFollow::outputLateralErr() const
{
  std_msgs::Float64 msg;
   msg.data = err_lat;
   return msg;
}

std_msgs::Float64 PIDFollow::outputSteering() const
{
  std_msgs::Float64 msg;
  msg.data = delta_f*180.0/M_PI;
  return msg;
}

autoware_msgs::VehicleCmd PIDFollow::run()
{
     if(!currentpose_set_ || !waypoint_set_ || !velocity_set_){
    if(!currentpose_set_) {
       ROS_WARN("position is missing");
     }
     if(!waypoint_set_) {
       ROS_WARN("waypoint is missing");
     }
     if(!velocity_set_) {
       ROS_WARN("velocity is missing");
    }
    return outputZero();
  }
   
   //output of steering, err_phi and lateral error
  ComputeLHDistance(lookahead_distance_set);
  
  // search next waypoint
  getNextWaypoint();
  if (num_of_next_waypoint_ == -1)
  {
    
    ROS_WARN("lost next waypoint");
    return outputZero();
  }
  bool compute_flag = calcDif(current_waypoints_, current_pose_);
  ROS_INFO_STREAM("compute_flag = " << compute_flag);
  if(!compute_flag)
  { 
      return outputZero();
  }
  
  //ROS_ERROR_STREAM("next waypoint = " <<  num_of_next_waypoint_);

  // if g_linear_interpolate_mode is false or next waypoint is first or last
  /*if (!linear_interpolate_ || num_of_next_waypoint_ == 0 ||
      num_of_next_waypoint_ == (static_cast<int>(current_waypoints_.getSize() - 1)))
  {
    position_of_next_target_ = current_waypoints_.getWaypointPosition(num_of_next_waypoint_);
    return outputTwist(calcTwist(calcCurvature(position_of_next_target_), getCmdVelocity(0)));
  }

  // linear interpolation and calculate angular velocity
  interpolate_flag = interpolateNextTarget(num_of_next_waypoint_, &position_of_next_target_);

  if (!interpolate_flag)
  {
    ROS_ERROR_STREAM("lost target! ");
    return outputZero();
  }*/

  // ROS_INFO("next_target : ( %lf , %lf , %lf)", next_target.x, next_target.y,next_target.z);
  /*if(!verifyFollowing())
  {
      ROS_INFO_STREAM("not following");
      
      return outputZero();
  }*/
  
  return setVehicleCommand(getCmdVelocity(0), ComputeSteering());
  
  

// ROS_INFO("linear : %lf, angular : %lf",twist.twist.linear.x,twist.twist.angular.z);

#ifdef LOG
  std::ofstream ofs("/tmp/pure_pursuit.log", std::ios::app);
  ofs << _current_waypoints.getWaypointPosition(next_waypoint).x << " "
      << _current_waypoints.getWaypointPosition(next_waypoint).y << " " << next_target.x << " " << next_target.y
      << std::endl;
#endif
}

double quaternionMsgToYaw( geometry_msgs::Quaternion orientation )
{
    tf::Quaternion q;
    tf::quaternionMsgToTF( orientation, q );

    double roll, pitch, yaw;
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    return yaw;
}



/****************************************************************/
/**************************DEBUG INFORMAITON************************/
/****************************************************************/
std_msgs::Float64 PIDFollow::debugYaw() const
{
  std_msgs::Float64 msg;
  msg.data = ( yaw>=0?yaw:(yaw+2*M_PI) )*180.0/M_PI;
  return msg;
}

std_msgs::Float64 PIDFollow::debugPhir() const
{
  std_msgs::Float64 msg;
  msg.data = ( phi_r>=0?phi_r:(phi_r+2*M_PI) )*180.0/M_PI;
  return msg;
}

std_msgs::Float64 PIDFollow::debugPhi1() const
{
  std_msgs::Float64 msg;
  msg.data = ( phi1>=0?phi1:(phi1+2*M_PI) )*180.0/M_PI;
  return msg;
}

std_msgs::Float64 PIDFollow::debugPhi2() const
{
  std_msgs::Float64 msg;
  msg.data = ( phi2>=0?phi2:(phi2+2*M_PI) )*180.0/M_PI;
  return msg;
}

std_msgs::Float64 PIDFollow::debugRatio() const
{
  std_msgs::Float64 msg;
  msg.data = ratio;
  return msg;
}
/****************************************************************/
/****************************************************************/
/****************************************************************/