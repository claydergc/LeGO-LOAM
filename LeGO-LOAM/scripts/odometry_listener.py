#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped

legoLoamPoses = open("/home/claydergc/legoloam_poses_ovalo_upao_10022022.txt","w")
zedPoses = open("/home/claydergc/zed_poses_ovalo_upao_10022022.txt","w")

time_stamp = 0.0
x_zed = 0.0
y_zed = 0.0
th_zed = 0.0

def callback(data):
    global time_stamp
    #rospy.loginfo("%f,%f", data.latitude, data.longitude)
	#time_stamp = data.header.stamp.secs + data.header.stamp.nsecs * 10**(-9)
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    legoLoamPoses.write(str(time_stamp) + " " + str(data.pose.pose.position.z) + " " + str(data.pose.pose.position.x) + " " + str(pitch) + "\n")
    zedPoses.write(str(time_stamp) + " " + str(x_zed) + " " + str(y_zed) + " " + str(th_zed) + "\n")


def callback2(data):
    #rospy.loginfo("%f,%f", data.latitude, data.longitude)
    global time_stamp
    global x_zed
    global y_zed
    global th_zed
    time_stamp = data.header.stamp.secs + data.header.stamp.nsecs * 10**(-9)
    x_zed = data.pose.pose.position.x
    y_zed = data.pose.pose.position.y
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    th_zed = yaw
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('odometry_listener', anonymous=True)

    rospy.Subscriber("/integrated_to_init", Odometry, callback)
    rospy.Subscriber("/zed2/zed_node/pose_with_covariance", PoseWithCovarianceStamped, callback2)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    legoLoamPoses.close()
    zedPoses.close()

if __name__ == '__main__':
    listener()
