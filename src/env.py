import os
import sys

if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    sys.exit("Please declare the environment variable 'SUMO_HOME'")
import traci
import sumolib
import numpy as np
import gym
from gym import spaces
from sumolib import checkBinary
from src.sumo_utils import get_state, take_action , get_total_waiting_time
from src.gen_sim import gen_sim


class SumoEnv(gym.Env):
    """Custom Environment that follows gym interface"""

    def __init__(self):
        super(SumoEnv, self).__init__()
        self.action_space = spaces.Discrete(4)
        self.low = np.zeros(9,)
        self.high = np.array([3] + 8*[2000])
        self.observation_space = spaces.Box(low=self.low, high=self.high, shape=(9,), dtype=np.int32)  
        self.conn = None
        self.max_step = 2000
        self.current_step = 0
 
    def step(self, action):
        cur_waiting_time, elapsed, emissions = take_action(self.conn, self.current_state, action)
        self.current_step += elapsed
        reward = (-1 * cur_waiting_time) / self.vehicles
        next_state = get_state(self.conn)
        self.current_state = next_state
        done = not (
            self.conn.simulation.getMinExpectedNumber() > 0
            and self.current_step <= self.max_step
        )
        if done:
            self.print_stats()
            close(self.conn)

        self.previous_action = action

        return np.array(next_state).reshape(9,), reward, done, {}

    def reset(self):
        self.reset_everyting()
        WEST_EAST , EAST_WEST , NORTH_SOUTH , SOUTH_NORTH = get_probs()
        self.vehicles = gen_sim(
            "",
            round=2,
            p_west_east=WEST_EAST,
            p_east_west=EAST_WEST,
            p_north_south=NORTH_SOUTH,
            p_south_north=SOUTH_NORTH,
        )
        self.conn = start()
        self.conn.trafficlight.setPhase("0", 2)
        state = get_state(self.conn)
        self.current_state = np.array(state)
        return self.current_state

    def render(self, mode="human"):
        raise NotImplementedError

    def print_stats(self):
        print("######### Finished Episode #############")


    def reset_everyting(self):
        self.current_step = 0
def start():
    sumoBinary = checkBinary("sumo")
    traci.start(
            [
                sumoBinary,
                "-c",
                "data/cross.sumocfg",
                "--time-to-teleport",
                "-1",
                "--tripinfo-output",
                "tripinfo.xml",
                "--start",
                "-Q",
            ],
            label="contestant",
    )
    conn = traci.getConnection("contestant")
    return conn

def close(conn):
    conn.close()
    traci.switch("contestant")
    traci.close()

def get_vehicle_ids(conn):
    vehicle_ids = (
        conn.lane.getLastStepVehicleIDs("1i_0")
        + conn.lane.getLastStepVehicleIDs("2i_0")
        + conn.lane.getLastStepVehicleIDs("4i_0")
        + conn.lane.getLastStepVehicleIDs("3i_0")
    )
    return vehicle_ids

def get_probs():
    WEST_EAST = np.random.rand()
    EAST_WEST = np.random.rand()
    NORTH_SOUTH = np.random.rand()
    SOUTH_NORTH = np.random.rand()
    return WEST_EAST , EAST_WEST , NORTH_SOUTH , SOUTH_NORTH