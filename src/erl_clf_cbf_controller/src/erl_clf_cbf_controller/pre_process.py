#!/usr/bin/env python3

import rospy
import numpy as np
import ros_numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_ros
from tf.transformations import euler_from_quaternion
from erl_msgs.msg import MovObsInfoArray
from geometry_msgs.msg import PoseWithCovarianceStamped

# Conditional import for erl_sdf_mapping
try:
    from erl_sdf_mapping.srv import PredictSdf
except ImportError:
    rospy.logwarn("erl_sdf_mapping package not found. Skipping import.")


# debug packages
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
from sensor_msgs import point_cloud2


class ClfCbfPreprocess:
    """
    Topic subscribe and service call
    """

    def __init__(self):

        # params from launch file
        _odom_topic = rospy.get_param("~odom_topic")
        _goal_topic = rospy.get_param("~goal_topic")
        _scan_topic = rospy.get_param("~scan_topic", "/front/scan")
        self.mov_obs = rospy.get_param("~mov_obs", False)
        self.use_sdf = rospy.get_param("~use_sdf", False)
        self._debug_on = rospy.get_param("~debug_mode", True)

        # subscribers
        self._odom_sub = rospy.Subscriber(_odom_topic, Odometry, self.odom_callback, queue_size=1)
        self._goal_sub = rospy.Subscriber(_goal_topic, Pose2D, self.goal_callback, queue_size=1)
        self._scan_sub = rospy.Subscriber(_scan_topic, LaserScan, self.scan_callback, queue_size=1)

        # Moving obstacle subscriber
        if self.mov_obs:
            self._movobs_sub = rospy.Subscriber(
                "/mov_obs_info_array", MovObsInfoArray, self.movobs_callback, queue_size=1
            )

        # debug publisher
        if self._debug_on:
            self.pcl2_pub = rospy.Publisher("~surfaces", PointCloud2, queue_size=1)
            self.pcl2_debug_msg = None

        # services
        if self.use_sdf:
            self.sdf_client = rospy.ServiceProxy("/erl_sdf_mapping_node/predict_sdf", PredictSdf, persistent=True)
            self.sdf_pub = rospy.Publisher("~sdf", PoseWithCovarianceStamped, queue_size=1)
            self.msg_sdf = PoseWithCovarianceStamped()
            self.msg_sdf.header.frame_id = "map"
            self.msg_sdf.header.seq = -1
            self.msg_sdf.pose.pose.position.z = 0.0
            self.msg_sdf.pose.pose.orientation.x = 0.0
            self.msg_sdf.pose.pose.orientation.y = 0.0
            self.msg_sdf.pose.pose.orientation.z = 0.0
            self.msg_sdf.pose.pose.orientation.w = 1.0
            self.msg_sdf.pose.covariance = [0.00001] * 36
            self._dist_service_ready = False

        # upstream status variables
        self._upstream_connection = 0
        self._upstream_connection_ready = False
        self._upstream_data_ready = False
        self._upstream_init_finish = False
        self._goal_received = False
        self.localization_fail = False
        self.scan_buffer_ready = False
        self.mov_obs_ready = False

        # ------------------- upstream data numpy container --------------------
        # containers for converted message in numpy
        self._np_dist = None
        self._np_z = None
        self._np_z_dsr = None
        self._prev_rbt_loc = None
        self._scan_frame = None
        self.scan_buffer = []
        self.mo_list = None

        self.path_received_time = rospy.Time.now()
        self.gvn_restart_received_time = rospy.Time.now()
        self.tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tf_buffer)
        self.laser_projection = lg.LaserProjection()

        # ------------------- Init Debug --------------------
        if self._debug_on:
            self.pcl2_debug_fields = [
                PointField("x", 0, PointField.FLOAT32, 1),
                PointField("y", 4, PointField.FLOAT32, 1),
                PointField("z", 8, PointField.FLOAT32, 1),
            ]

            self.pcl2_debug_header = Header()
            self.pcl2_debug_header.frame_id = "map"
            self.pcl2_debug_header.stamp = rospy.Time.now()

        # ------------------- Init Upstream --------------------
        self.init_upstream()
        rospy.loginfo("[CLF-CBF Preprocessor Created!]  \n")

    def _check_upstream_connections(self, upstream_connection=2):
        """check whether subscribers' uplink connections are established"""

        self._upstream_connection = self._odom_sub.get_num_connections() + self._scan_sub.get_num_connections()

        if self._upstream_connection < upstream_connection:
            # we need to wait path, states and lidar all ready
            rospy.logwarn_throttle(
                1.0,
                "[clf_cbf_pre] waiting upstream connections [%d / %d]:",
                self._upstream_connection,
                upstream_connection,
            )
            # odom
            if self._odom_sub.get_num_connections() < 1:
                rospy.loginfo_throttle(1.0, "[clf_cbf_pre] waiting odom...")
            # scan
            if self._scan_sub.get_num_connections() < 1:
                rospy.loginfo_throttle(1.0, "[clf_cbf_pre] waiting scan...")

        else:
            self._upstream_connection_ready = True
            rospy.loginfo("\n[clf_cbf_pre] %d upstream connections established !\n", upstream_connection)

    def odom_callback(self, odom_msg):
        rospy.logwarn_once("[clf_cbf_pre] Received odometry!")
        pose = odom_msg.pose.pose
        quaternion_sxyz = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        (_, _, yaw) = euler_from_quaternion(quaternion_sxyz)
        self._np_z = np.array([pose.position.x, pose.position.y, yaw])

        # set goal to current pose if no goal received
        if not self._goal_received:
            self._np_z_dsr = self._np_z

        # check for de-localization
        self.check_de_localization()
        return

    def goal_callback(self, goal_msg):
        rospy.logwarn_once("[clf_cbf_pre] Received goal!")
        self._np_z_dsr = np.array([goal_msg.x, goal_msg.y, goal_msg.theta])

        # set flag to indicate true goal has been received
        self._goal_received = True
        return

    def scan_callback(self, scan_msg):
        rospy.logwarn_once("[clf_cbf_pre] Received scan!")
        if not self._upstream_data_ready:
            self._scan_frame = scan_msg.header.frame_id

        if self._upstream_data_ready:
            try:
                transform_lidar2map = self.tf_buffer.lookup_transform("map", self._scan_frame, rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn_throttle(0.5, f"[clf_cbf_pre] Failed to get transform from {self._scan_frame} to map")
                return

            pcl2_msg = self.laser_projection.projectLaser(scan_msg)
            pcl2_transformed = do_transform_cloud(pcl2_msg, transform_lidar2map)
            pcl_data = ros_numpy.numpify(pcl2_transformed)
            points_x = pcl_data["x"]
            points_y = pcl_data["y"]
            surface_points = np.vstack((points_x, points_y))
            self.scan_buffer.insert(0, surface_points)

            if len(self.scan_buffer) > 5:
                # keep scan buffer at a certain length
                self.scan_buffer.pop()

                if not self.scan_buffer_ready:
                    # set scan buffer ready flag to true if not
                    self.scan_buffer_ready = True

            if self._debug_on and self.scan_buffer_ready and self._upstream_init_finish:
                surface_buffer_xy = np.row_stack(
                    (
                        self.scan_buffer[0].T,
                        self.scan_buffer[1].T,
                        self.scan_buffer[2].T,
                        self.scan_buffer[3].T,
                        self.scan_buffer[4].T,
                    )
                )
                surface_points_xyz = np.column_stack((surface_buffer_xy, np.zeros(np.shape(surface_buffer_xy)[0])))
                self.pcl2_debug_header.stamp = rospy.Time.now()
                self.pcl2_debug_msg = point_cloud2.create_cloud(
                    self.pcl2_debug_header, self.pcl2_debug_fields, surface_points_xyz
                )

                self.pcl2_pub.publish(self.pcl2_debug_msg)

    def movobs_callback(self, movobs_msg):
        rospy.logwarn_once("[ref_gvn_pre] received movobs_info")
        mo_list = np.empty((0, 5))
        mo_array = movobs_msg.movobsinfos
        mo_n = len(mo_array)

        for mi in range(mo_n):
            moi_msg = mo_array[mi]
            moi_info = np.array([moi_msg.p_mo.x, moi_msg.p_mo.y, moi_msg.v_mo.x, moi_msg.v_mo.y, moi_msg.r_mo])
            mo_list = np.vstack((mo_list, moi_info))
        self.mo_list = mo_list

        if not self.mov_obs_ready:
            self.mov_obs_ready = True

    def _check_upstream_data(self):
        """check whether upstream data container are loaded/initialized correctly"""
        # navigation path r
        status = True

        # robot state z
        if self._np_z is None:
            status = False
            rospy.loginfo_throttle(1.0, "[clf_cbf_pre] waiting zvec init...")

        # goal z_dsr
        if self._np_z_dsr is None:
            status = False
            rospy.loginfo_throttle(1.0, "[clf_cbf_pre] waiting goal init...")

        # scan
        if self._scan_frame is None:
            status = False
            rospy.loginfo_throttle(1.0, "[clf_cbf_pre] waiting scan init...")

        if status:
            self._upstream_data_ready = True
            rospy.loginfo_once("\n[clf_cbf_pre] all %d upstream data initialized !\n" % self._upstream_connection)

    def init_upstream(self):
        """
        Check upstream connection
        """
        while (not self._upstream_connection_ready) and (not rospy.is_shutdown()):
            self._check_upstream_connections()
            rospy.sleep(0.1)
            rospy.logdebug_throttle(1.0, "waiting upstream connections...")
        rospy.loginfo("[clf_cbf_pre] upstream [connection] is ready, check upstream [data]...")

        while (not self._upstream_data_ready) and (not rospy.is_shutdown()):
            self._check_upstream_data()
            rospy.sleep(0.1)
            rospy.logdebug_throttle(1.0, "waiting upstream data...")
        rospy.loginfo("[clf_cbf_pre] upstream [data] is ready, check upstream [service]...")

        if self.use_sdf:
            while (not self._dist_service_ready) and (not rospy.is_shutdown()):
                rospy.wait_for_service("/erl_sdf_mapping_node/predict_sdf")
                self._dist_service_ready = True
                rospy.sleep(0.1)

        while not self.scan_buffer_ready:
            rospy.sleep(0.1)
            rospy.logdebug_throttle(1.0, "waiting scan buffer...")

        rospy.loginfo_throttle(1.0, "[clf_cbf_pre] upstream [service] is ready!")

        self._upstream_init_finish = True
        rospy.loginfo("[clf_cbf_pre] Upstream Init Done!")

    def check_de_localization(self):
        """
        This function checks whether de-localization occurs.
        If the robot location different too much between in the direction perpendicular to the robot heading,
        then a de-localization is considered to be occurred.
        """
        current_rbt_loc = self._np_z[0:2]

        if self._prev_rbt_loc is None:
            self._prev_rbt_loc = self._np_z

        previous_rbt_loc = self._prev_rbt_loc[0:2]
        rbt_heading = self._np_z[2]
        dist_perp = np.abs(
            (current_rbt_loc - previous_rbt_loc) @ np.array([[-np.sin(rbt_heading)], [np.cos(rbt_heading)]])
        )
        if dist_perp <= 0.1 and not self.localization_fail:
            self._prev_rbt_loc = self._np_z
        else:
            self.localization_fail = True
            self._np_z = self._prev_rbt_loc
