#!/usr/bin/env python3

# python libs
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation

# kiss_icp libs
from kiss_icp.config import KISSConfig, load_config
from kiss_icp.odometry import Odometry

# ros libs
import rospy
import ros_numpy
import rospkg

# ros msgs libs
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import tf

# get package path
rospack = rospkg.RosPack()
pkg_path = Path(rospack.get_path('kiss_icp'))


# Main class
class KissIcpOdometry:
    def __init__(self, config: str = None):
        
        # read config
        if config is None:
            config = Path(pkg_path, "config", "default.yaml") # default config
        else:
            config = Path(config) # passed in object
        config = rospy.get_param("~config", config) # passed as ros parameter (highest priority)
        self.config: KISSConfig = load_config(config)

        # define default value for parameters, added to config manually (originally wasn't there)
        deskew = False
        if hasattr(self.config, "deskew"):
            deskew = self.config.deskew
        self.publish_odom_tf = True
        if hasattr(self.config, "publish_odom_tf"):
            self.publish_odom_tf = self.config.publish_odom_tf
        
        # define used objects
        self.odometry = Odometry(config=self.config, deskew=deskew)
        self.times = []
        self.poses = self.odometry.poses

        # define frames
        self.frame_id_sensor_estimation = "velodyne_estimate"
        self.frame_id_sensor = "velodyne"
        self.frame_id_global = "world"
        self.frame_id_estimation = "base_link"
        if hasattr(self.config, "frames") and hasattr(self.config.frames, "sensor_frame"):
            self.frame_id_sensor = self.config.frames.sensor_frame
        else:
            rospy.loginfo(f"[KISS-ICP] Using default sensor frame {self.frame_id_sensor}")
        if hasattr(self.config, "frames") and hasattr(self.config.frames, "global_frame"):
            self.frame_id_global = self.config.frames.global_frame
        else:
            rospy.loginfo(f"[KISS-ICP] Using default global frame {self.frame_id_global}")
        if hasattr(self.config, "frames") and hasattr(self.config.frames, "estimation_frame"):
            self.frame_id_estimation = self.config.frames.estimation_frame
        else:
            rospy.loginfo(f"[KISS-ICP] Using default estimation frame {self.frame_id_estimation}")

        # get tf from sensor to estimated frame
        self.get_sensor_tf()

        # define pubs, subs
        self.pub = rospy.Publisher('~estimated_pose', PoseStamped, queue_size=1)
        self.points_pub = rospy.Publisher( '~velodyne_pcl', PointCloud2, queue_size=1)
        self.points_sub = rospy.Subscriber('/points_in', PointCloud2, self.points_callback, queue_size=10)
        
        self.br = tf.TransformBroadcaster()

    def points_callback(self, msg: PointCloud2):
        frame = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
        timestamps = np.zeros(frame.shape[0])
        start_time = time.perf_counter_ns()
        in_frame, source = self.odometry.register_frame(frame, timestamps)
        self.times.append(time.perf_counter_ns() - start_time)
        self.publish_pose(self.poses[-1])
        msg.header.frame_id = self.frame_id_sensor
        msg.header.stamp = rospy.Time.now()
        self.points_pub.publish(msg)

    def publish_pose(self, pose):
        # publish sensor pose
        pose = pose@self.T_sensor
        R_array = pose[:3, :3]
        R = Rotation.from_matrix(R_array)
        R_q = R.as_quat()
        t = pose[:3, 3]
        p = PoseStamped()
        p.header.frame_id = self.frame_id_global
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = t[0]
        p.pose.position.y = t[1]
        p.pose.position.z = t[2]
        p.pose.orientation.x = R_q[0]
        p.pose.orientation.y = R_q[1]
        p.pose.orientation.z = R_q[2]
        p.pose.orientation.w = R_q[3]
        self.pub.publish(p)

        # publish transformation to sensor estimated position
        self.br.sendTransform((t[0], t[1], t[2]),
                        R_q,
                        rospy.Time.now(),
                        self.frame_id_sensor_estimation,
                        self.frame_id_global)

        # publish transformation to estimated frame
        if self.publish_odom_tf:
            T_to_base = pose @  np.linalg.inv(self.T_sensor)
            R_array = T_to_base[:3, :3]
            R = Rotation.from_matrix(R_array)
            R_q = R.as_quat()
            t = T_to_base[:3, 3]
            self.br.sendTransform((t[0], t[1], t[2]),
                            R_q,
                            rospy.Time.now(),
                            self.frame_id_estimation,
                            self.frame_id_global)
    
    # function that get the transformation from estimated frame to the sensor frame
    def get_sensor_tf(self):
        listener = tf.TransformListener()
        c = 0
        # try to read the translation from the rplidar to the base_footprint
        while not rospy.is_shutdown() and c<=10:
           try:
               c +=1
               rospy.loginfo(f"[KISS-ICP] Trying to get transformation from {self.frame_id_estimation} to {self.frame_id_sensor} ..")
               trans, rot = listener.lookupTransform(self.frame_id_estimation, self.frame_id_sensor, rospy.Time(0))
               break
           except:
               time.sleep(1.0)
               continue
        # if not possible, using these values previously stored: (as when using bag files)
        if c >= 10:
            rot = [0.0, 0.0, 0.0, 1.0]
            trans = [0.0, 0.0, 0.138]
            rospy.loginfo(f"[KISS-ICP] Failed to get the transformation from {self.frame_id_estimation} to {self.frame_id_sensor}")
            rospy.loginfo(f"[KISS-ICP] Using default transformation: t = {trans}, R(quat) = {rot}")
        else:
            rospy.loginfo(f"[KISS-ICP] Got the transformation from {self.frame_id_estimation} to {self.frame_id_sensor}!")
        
        R = Rotation.from_quat(rot).as_matrix()
        self.T_sensor = np.eye(4)
        self.T_sensor[:3, :3] = R
        self.T_sensor [:3, 3] = trans
 
if __name__ == '__main__':
    rospy.init_node('kiss_icp')
    odometry = KissIcpOdometry()
    rospy.spin()