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
from sumo_utils import get_state, take_action , get_total_waiting_time
from gen_sim import gen_sim


class SumoEnv(gym.Env):
    """Custom Environment that follows gym interface"""

    def __init__(self, steps_per_episode=100000):
        super(SumoEnv, self).__init__()
        self.action_space = spaces.Discrete(2)
        self.low = np.zeros(9,)
        self.high = np.array([1] + 8*[1000])
        self.observation_space = spaces.Box(low=self.low, high=self.high, shape=(9,), dtype=np.int16)  
        self.steps_per_episode = steps_per_episode
        self.conn = None
        self.current_step = 0
        self.total_waiting_time = 0
        self.total_emissions = 0
        self.current_state = None
        self.vehicles = None
        self.current_reward = 0
 
    def step(self, action):
        cur_waiting_time, elapsed, emissions = take_action(self.conn, self.current_state, action)
        vehicle_ids = get_vehicle_ids(self.conn)
        self.total_emissions += emissions
        self.total_waiting_time += cur_waiting_time
        reward = emissions * -1
        self.current_step += elapsed
        next_state = get_state(self.conn)
        self.current_state = next_state
        done = not (
            self.conn.simulation.getMinExpectedNumber() > 0
            and self.current_step <= self.steps_per_episode
        )
        if done:
            self.print_stats()
            close(self.conn)

        return np.array(next_state), reward, done, {}

    def reset(self):
        self.reset_everyting()
        WEST_EAST = np.random.rand()
        EAST_WEST = np.random.rand()
        NORTH_SOUTH = np.random.rand()
        SOUTH_NORTH = np.random.rand()
        self.vehicles = gen_sim(
            "",
            round=1,
            p_west_east=0.3,
            p_east_west=0.2,
            p_north_south=0.2,
            p_south_north=0.1,
        )
        self.conn = start()
        self.conn.trafficlight.setPhase("0", 2)
        state = get_state(self.conn)
        self.current_state = np.array(state)
        return self.current_state

    def render(self, mode="human"):
        raise NotImplementedError


    def print_stats(self):
        avg_waiting_time = self.total_waiting_time / self.vehicles
        avg_emissions = self.total_emissions / (1000 *self.vehicles)
        print(avg_waiting_time,avg_emissions)

    def reset_everyting(self):
        self.current_step = 0
        self.total_waiting_time = 0
        self.total_emissions = 0


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