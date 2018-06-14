import rospy
from enum import Enum
from datastore import DataStore
from movement import MovementHandler
from actions import Actions


class State(Enum):
    Init = 1
    Takeoff = 2
    ExploreStart = 3
    FullRadialScan = 4
    FindGoal = 5

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
            ],
        }

        self.store = DataStore()
        self.movement = MovementHandler(core=self, datastore=self.store)
        self.actions = Actions(core=self, datastore=self.store, movement_handler=self.movement)

        self.temp_data = {}

    def handle_state(self, state):

        if state == State.Init:
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
            return None

        print('Unrecognised state `{}` detected - ignoring.'.format(State(state).name))
        return None

    def step(self):
        current_state = self.state
        self.movement.start_step(current_state)
        next_state = self.handle_state(current_state)

        if next_state is not None:
            if next_state != current_state:
                print ('[?] Switching state from {} to {}'.format(State(current_state).name, State(next_state).name))
            self.state = next_state

        self.movement.finish_step()
        return next_state
