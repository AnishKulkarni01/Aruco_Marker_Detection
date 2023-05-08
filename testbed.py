#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Vector3
import cv2 as cv
import numpy as np
from cv2 import aruco   
from tf.transformations import quaternion_from_matrix
from tf.transformations import euler_from_matrix, quaternion_from_euler


#providing path to collected data from camera caliberation
calib_data_path = "MultiMatrix.npz"
try:
    calib_data = np.load(calib_data_path)
except FileNotFoundError:
    rospy.logerr("file not found")
    exit()
    
# print(calib_data.files)

# cam_mat= np.array([[504.0454674647588, 0, 311.2319809574285], [0, 501.8161118138039, 229.5929172462516], [0, 0, 1]])
# dist_coef= np.array([0.1003289975570445, -0.2070707062703581, -0.003269738523739203, -0.007686547973916399, 0])
# r_vectors = calib_data["rVector"]
# t_vectors = calib_data["tVector"]
cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

MARKER_SIZE = 19 #in cm

# id,marker pairs having 4x4 aruco markers 
marker_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
#parameters for detection
param_markers =  cv.aruco.DetectorParameters_create()


#creating camera object and index==0 as only one camera connected
cap = cv.VideoCapture(0)



def convert_rvec_tvec_to_odometry(rvec, tvec,i):
    # Convert rotation vector to rotation matrix
    R, _ = cv.Rodrigues(rvec)
    
    # Create Odometry message instance and set header and child frame IDs
    odom_msg = Odometry()
    
    # TODO
    odom_msg.header.frame_id = "odom"+str(i)
    odom_msg.child_frame_id = "base_link"+str(i)
    #print(R)

    # Convert rotation matrix to quaternion
    #q = Quaternion(*quaternion_from_matrix(R))
    t = euler_from_matrix(R)

    s = []
    for i in t:
        s.append(i)


    q=quaternion_from_euler(s[0],s[1],s[2])

    # Set pose and orientation in Odometry message
    odom_msg.pose.pose.position = Point(*tvec.flatten())
    odom_msg.pose.pose.orientation.x = q[0]
    odom_msg.pose.pose.orientation.y = q[1]
    odom_msg.pose.pose.orientation.z = q[2]
    odom_msg.pose.pose.orientation.w = q[3]

    # odom_msg.pose.covariance = [0.1,   0,   0,   0,   0, 0,
    #                     0, 0.1,   0,   0,   0, 0,
    #                     0,   0, 1e6,   0,   0, 0,
    #                     0,   0,   0, 1e6,   0, 0,
    #                     0,   0,   0,   0, 1e6, 0,
    #                     0,   0,   0,   0,   0, 0.2]

    # Set linear and angular velocities to zero
    odom_msg.twist.twist.linear = Vector3(0, 0, 0)
    odom_msg.twist.twist.angular = Vector3(0, 0, 0)


    return odom_msg


if __name__ == '__main__':
    # Initialize ROS node and publisher
    rospy.init_node('odom_publisher')
    odom_pub=[]

   

    # Publish Odometry message
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Failed to read from camera")
            break
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)  #converting to gray scale
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, marker_dict, parameters=param_markers
        )
        if marker_corners:
            #getting rotational and translational vectors for all markers
            rVec,tVec,_= aruco.estimatePoseSingleMarkers(marker_corners,MARKER_SIZE,cam_mat,dist_coef)
            total_markers=range(0,marker_IDs.size)
            for ids, corners ,i in zip(marker_IDs, marker_corners,total_markers):
                odom_pub.append(rospy.Publisher('/odom'+str(i), Odometry, queue_size=10))
            # Convert rvec and tvec to Odometry message
            for ids, corners ,i in zip(marker_IDs, marker_corners,total_markers):
                #odom_pub = rospy.Publisher('/odom'+str(i), Odometry, queue_size=10)
                tVec[i][0][0]/=100
                tVec[i][0][1]/=100
                tVec[i][0][2]/=100
                tVec[i][0][2]-=2.72338839249
                odom_msg = convert_rvec_tvec_to_odometry(rVec[i], tVec[i],i)
                
                #print(odom_msg)

                # odom_pub.publish(odom_msg)
                odom_pub[i].publish(odom_msg)
            
        cv.imshow("frame", frame)
        #quit window
        key = cv.waitKey(1)
        if key == ord("q"):
            break
        rospy.sleep(0.1)
    cap.release()
    cv.destroyAllWindows()
    

#rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 odom0 map