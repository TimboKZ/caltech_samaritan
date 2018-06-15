import rospy
import tf2_ros
import ros_numpy
import numpy as np
from sensor_msgs.msg import Range
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from tf.transformations import euler_from_quaternion


class DataStore:

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.sonar_height = None

        self.last_pose = None

        self.last_map_resolution = None
        self.last_map_raw = None
        self.last_map = None
        self.last_map_extents = None

        self.last_global_costmap_raw = None
        self.last_global_costmap = None
        self.last_global_costmap_extents = None

        self.last_updated_costmap = None
        self.last_updated_costmap_extents = None

        rospy.Subscriber('/sonar_height', Range, self.sonar_height_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.global_cost_map_callback)
        rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate,
                         self.global_cost_map_update_callback)

    def step(self):
        self.update_last_pose()

    def wait_for_data(self):
        while True:
            self.step()

            missing_data = []
            if self.last_pose is None:
                missing_data.append('last_pose')
            if self.last_map is None:
                missing_data.append('last_map')
            if self.last_global_costmap is None:
                missing_data.append('last_global_costmap')

            if len(missing_data) == 0:
                break

            rospy.loginfo('Waiting for the following data: {}.'.format(', '.join(missing_data)))
            rospy.sleep(2)

    def get_pose(self):
        return self.last_pose

    def update_last_pose(self):
        try:
            tfmsg = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time())
            trans = tfmsg.transform.translation
            orient = tfmsg.transform.rotation
            yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])[2]
            self.last_pose = (trans.x, trans.y, yaw)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return False
        return True

    def get_sonar_height(self):
        return self.sonar_height

    def sonar_height_callback(self, height_data):
        self.sonar_height = height_data.range

    def get_map(self):
        return self.last_map

    def get_map_raw(self):
        return self.last_map_raw

    def map_callback(self, occupancy_grid_data):
        self.last_map_raw = occupancy_grid_data
        self.last_map = ros_numpy.occupancy_grid.occupancygrid_to_numpy(occupancy_grid_data).data
        resolution = occupancy_grid_data.info.resolution
        left = occupancy_grid_data.info.origin.position.x
        right = left + occupancy_grid_data.info.width * resolution
        bottom = occupancy_grid_data.info.origin.position.y
        top = bottom + occupancy_grid_data.info.height * resolution
        self.last_map_extents = (left, right, bottom, top)

    def get_global_costmap(self):
        return self.last_global_costmap

    def global_cost_map_callback(self, occupancy_grid_data):
        self.last_global_costmap_raw = occupancy_grid_data

        self.last_global_costmap = ros_numpy.occupancy_grid.occupancygrid_to_numpy(occupancy_grid_data)

        resolution = occupancy_grid_data.info.resolution
        left = occupancy_grid_data.info.origin.position.x
        right = left + occupancy_grid_data.info.width * resolution
        bottom = occupancy_grid_data.info.origin.position.y
        top = bottom + occupancy_grid_data.info.height * resolution
        self.last_global_costmap_extents = (left, right, bottom, top)

    def global_cost_map_update_callback(self, msg):
        if self.last_map_raw is None or self.last_global_costmap is None:
            return
        self.last_updated_costmap = self.occupancy_grid_update_to_numpy(msg)
        self.last_global_costmap[msg.y:msg.y + msg.height,
        msg.x:msg.x + msg.width] = self.last_updated_costmap
        info = self.last_map_raw.info
        resolution = info.resolution
        left = self.last_global_costmap_extents[0] + resolution * msg.x
        right = left + msg.width * resolution
        bottom = self.last_global_costmap_extents[2] + resolution * msg.y
        top = bottom + msg.height * resolution
        self.last_updated_costmap_extents = (left, right, bottom, top)

    @staticmethod
    def occupancy_grid_update_to_numpy(msg):
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.height, msg.width)
        return np.ma.array(data, mask=data == -1, fill_value=-1)
