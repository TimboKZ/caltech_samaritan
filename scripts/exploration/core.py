import rospy
from enum import Enum
from actions import Actions
from datastore import DataStore
from planner import Planner
from movement import MovementHandler


class State(Enum):
    Init = 0
    WaitingForData = 1
    Takeoff = 2
    ExploreStart = 3
    FullRadialScan = 4
    FindGoal = 5
    TravelToGoal = 6

    ExplorationFinish = 777
    Land = 888
    Done = 999


class Core:

    def __init__(self):
        self.state = State.Init
        self.config = {
            'hover_height': 1.0,

            # States for which movement.fix_hover() will NOT be called (to make sure the drone is at `hover_height`)
            'exclude_from_fix_hover': [
                State.Init,
                State.Takeoff,
                State.Land,
                State.Done,
            ],

            # Radius in meters around a blacklisted goal that the robot will ignore
            'blacklisted_goal_radius': 2.0
        }

        self.store = DataStore()
        self.movement = MovementHandler(core=self, datastore=self.store)
        self.actions = Actions(core=self, datastore=self.store, movement_handler=self.movement)
        self.planner = Planner(core=self, datastore=self.store, movement_handler=self.movement)

        # Aux files
        self.temp_data = {}
        self.last_goal = None

    def handle_state(self, state):
        """
        :param state: Current state
        :return: Returns either next state or None (i.e. stay in the same state)
        """

        if state == State.Init:

            # Make sure there are no lingering goals
            self.planner.cancel_all_goals()

            return State.WaitingForData

        elif state == State.WaitingForData:
            self.store.wait_for_data()
            return State.Takeoff

        elif state == State.Takeoff:
            return State.ExploreStart if self.actions.takeoff() else None

        elif state == State.ExploreStart:
            return State.FullRadialScan

        elif state == State.FullRadialScan:
            fr_scan_start = self.temp_data.get('fr_scan_start')
            if fr_scan_start is None:
                fr_scan_start = rospy.Time.now()
                self.temp_data['fr_scan_start'] = fr_scan_start

            done = self.actions.full_radial_scan(start_time=fr_scan_start)
            if done:
                self.temp_data['fr_scan_start'] = None
                return State.FindGoal
            else:
                return None

        elif state == State.FindGoal:
            goal = self.planner.find_goal()
            if goal is None:
                rospy.loginfo('Cannot find any more goals, finishing exploration.')
                return State.ExplorationFinish
            if goal == self.last_goal:
                rospy.loginfo('The new goal is the same as the old goal, finishing exploration.')
                return State.ExplorationFinish

            self.last_goal = goal
            x_coord, y_coord, yaw = goal
            self.planner.publish_goal(x_coord, y_coord, yaw)
            return State.TravelToGoal

        elif state == State.TravelToGoal:
            if self.planner.travel_to_goal():
                self.planner.cancel_all_goals()
                return State.FullRadialScan
            else:
                return None

        elif state == State.ExplorationFinish:
            self.planner.cancel_all_goals()
            return State.Land

        elif state == State.Land:
            return State.Done if self.actions.land() else None

        rospy.logwarn('Unrecognised state `{}` detected - ignoring.'.format(State(state).name))
        return None

    def step(self):

        self.store.step()

        current_state = self.state
        self.movement.start_step(current_state)
        next_state = self.handle_state(current_state)

        if next_state is not None:
            if next_state != current_state:
                rospy.loginfo('[?] Switching state from {} to {}'.format(State(current_state).name, State(next_state).name))
            self.state = next_state

        self.movement.finish_step()
        return next_state
