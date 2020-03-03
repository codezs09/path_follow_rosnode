#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import tf
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TwistStamped, Vector3
from nav_msgs.msg import Odometry

import csv


#Pose_x = []
#Pose_y = []
ndt_pose_x = []
ndt_pose_y = []
ndt_pose_z = []
ndt_pose_heading = []

# def utm_pose_callback(data):
#    global Pose_x
#    Pose_x.append(data.pose.pose.position.x)
#    # print Pose_x
#    global Pose_y
#    Pose_y.append(data.pose.pose.position.y)

def get_rotation(quaternion):

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)

    return yaw

# callback for pose
def ndt_pose_callback(data):
    global ndt_pose_x
    ndt_pose_x.append(data.pose.position.x)
    global ndt_pose_y
    ndt_pose_y.append(data.pose.position.y)
    global ndt_pose_z
    ndt_pose_z.append(data.pose.position.z)
    global ndt_pose_heading
    temp_heading = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
    yaw = get_rotation(temp_heading)
    ndt_pose_heading.append(yaw)
    


#def twist_callback(data):

#    global ndt_pose_heading
 #   #temp_heading = [data.twist.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
 #   yaw = data.twist.angular.z
#    ndt_pose_heading.append(yaw)





def listener():
    rospy.init_node('convert', anonymous = True)
    rospy.Subscriber("/ndt_pose",PoseStamped,ndt_pose_callback)
   # rospy.Subscriber("/current_velocity",TwistStamped,twist_callback)
#    rospy.Subscriber("/gps/odom",Odometry,utm_pose_callback)
    rospy.spin()

if __name__ =='__main__':
    listener()
#   coordinate_in_utm = zip(Pose_x,Pose_y)
    coordinate_in_ndt = zip(ndt_pose_x,ndt_pose_y, ndt_pose_z, ndt_pose_heading)

with open('pose_recording_2.csv',mode ='wb') as CF:
     writer = csv.writer(CF)
     writer.writerows(coordinate_in_ndt)
CF.close()

#with open('utm_pose_wimu_1218.csv',mode ='wb') as CF2:
#     writer = csv.writer(CF2)
#     writer.writerows(coordinate_in_utm)
#CF2.close()
