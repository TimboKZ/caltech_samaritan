import math
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler


class Planner:

    def __init__(self, core, datastore, movement_handler):
        self.core = core
        self.store = datastore
        self.movement = movement_handler

        rospy.Subscriber('/planned_cmd_vel', Twist, self.planned_cmd_vel_callback)

        rospy.loginfo('Initialising a SimpleActionClient for move_base and waiting for connection...')
        self.sa_client = SimpleActionClient('move_base', MoveBaseAction)
        self.sa_client.wait_for_server()
        rospy.loginfo('Connected to move_base action server!')

        # Aux variables
        self.last_planned_cmd_vel_msg = None

    def planned_cmd_vel_callback(self, twist_msg):
        self.last_planned_cmd_vel_msg = twist_msg

    def find_goal(self):
        frontiers_as_indices = self.find_frontiers_as_indices()

        if len(frontiers_as_indices) < 1:
            return None

        ia, ib = frontiers_as_indices.pop()
        goal_ia, goal_ib = self.find_safe_place_to_observe(ia, ib)
        x, y = self.indices_to_meters(goal_ia, goal_ib)
        pos_x, pos_y, yaw = self.store.get_pose()
        angle_to_goal = math.atan2(y - pos_y, x - pos_x)

        return x, y, angle_to_goal

    def find_safe_place_to_observe(self, ia, ib):
        # TODO: Improve this
        return ia, ib

    def find_frontiers_as_indices(self):
        costmap = self.store.get_global_costmap()
        if costmap is None:
            rospy.logwarn('No costmap is available - cannot find frontiers as indices, doing nothing.')
            return []

        m, n = costmap.shape
        safe_space_indices = np.where(costmap == 0)
        frontier_indices = []

        candidates = [
            (1, -1),
            (1, 0),
            (1, 1),
            (0, 1),
            (-1, 1),
            (-1, 0),
            (-1, -1),
            (-1, -1),
            (-1, 0),
        ]

        # We define a frontier as a cell that has at at least three free neighbours and and at least three unknown
        # neighbours. Remaining neighbours should either be free on unknown.

        for (ia, ib) in zip(*safe_space_indices):

            # Sanity check - skip masked inputs if they are accidentally included
            if costmap.mask[ia, ib]:
                continue

            free = 0
            known_occupied = 0
            for off_ia, off_ib in candidates:
                ia_new, ib_new = ia + off_ia, ib + off_ib

                if 0 <= ia_new < m and 0 <= ib_new < n:
                    value = costmap[ia_new, ib_new]
                    masked = costmap.mask[ia_new, ib_new]
                    if value == 0:
                        free += 1
                    elif not masked:
                        known_occupied += 1
                        break
                else:
                    # We're add the edge of the costmap, can consider this as an unknown
                    pass

            if known_occupied == 0 and 3 <= free <= 5:
                frontier_indices.append((ia, ib))

        return frontier_indices

    def travel_to_goal(self):
        goal_status = self.sa_client.get_state()

        if goal_status == GoalStatus.PENDING:
            return False

        if goal_status == GoalStatus.ACTIVE:
            if self.last_planned_cmd_vel_msg is None:
                return False

            msg = self.last_planned_cmd_vel_msg
            self.movement.request_velocity(
                lin_x=msg.linear.x,
                lin_y=msg.linear.y,
                lin_z=msg.linear.z,
                ang_x=msg.angular.x,
                ang_y=msg.angular.y,
                ang_z=msg.angular.z
            )
            return False

        if goal_status in [GoalStatus.RECALLED, GoalStatus.REJECTED, GoalStatus.PREEMPTED, GoalStatus.ABORTED]:
            rospy.logwarn('Goal was recalled, rejected, preempted or aborted.')
            return True

        if goal_status == GoalStatus.SUCCEEDED:
            rospy.loginfo('Goal succeeded.')
            return True

        if goal_status == GoalStatus.LOST:
            rospy.loginfo('Goal was lost!')
            return True

    def publish_goal(self, x_coord, y_coord, yaw):
        rospy.loginfo(" requesting to move to {}".format((x_coord, y_coord, yaw)))

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x_coord
        goal.target_pose.pose.position.y = y_coord
        q = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.sa_client.send_goal(goal)

    def cancel_all_goals(self):
        self.sa_client.cancel_all_goals()

    def indices_to_meters(self, ia, ib):
        info = self.store.get_map_raw().info
        resolution = info.resolution
        x = info.origin.position.x + ib * resolution + resolution/2
        y = info.origin.position.y + ia * resolution + resolution/2
        return x, y

    def meters_to_indices(self, x, y):
        info = self.store.get_map_raw().info
        resolution = info.resolution
        ib = int(round((x - info.origin.position.x) / resolution))
        ia = int(round((y - info.origin.position.y) / resolution))
        return ia, ib
