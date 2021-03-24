YELLOW_TIME = 6
HOLD_TIME = 6
MAX_STEP_COUNT = 1000
CENSOR_PROBABILITY = 0.1

import random


def get_curr_open_dir(conn):
    """
    :param conn: The simulation connection environment
    :return: curr_open_dir
    - curr_open_dir for COMPETITION_ROUND 1:
            (0 for vertical, 1 for horizontal) --> possible actions (0, 1)
    - curr_open_dir for COMPETITION_ROUND 2:
            (0 down, 1 left, 2 up, 3 right)    --> possible actions (0, 1, 2, 3)
    """
    return [conn.trafficlight.getPhase("0") // 2]


def get_waiting_count(conn, vehicle_ids):
    """
    :param conn: The simulation connection environment
    :param vehicle_ids: Vehicle ids in the simulation
    :return: the number of waiting vehicles.
    We define a waiting vehicle by passing its ID to the getWaitingTime SUMO function
    incrementing the waiting vehicles count by 1 whenever a car’s waiting time is more than 0.
    It uses the function getWaitingTime(self, vehID)
    which returns the waiting time of a vehicle, defined as the time (in seconds) spent with
    a speed below 0.1m/s since the last time it was faster than 0.1m/s.
    (basically, the waiting time of a vehicle is reset to 0 every time it moves).
    A vehicle that is stopping intentionally with a <stop> does not accumulate waiting time.
    """
    res = 0
    for vehicle_id in vehicle_ids:
        if conn.vehicle.getWaitingTime(vehicle_id) > 0:
            res += 1
    return res


def get_total_waiting_time(conn, vehicle_ids):
    """
    :param conn: The simulation connection environment
    :param vehicle_ids: Vehicle ids in the simulation
    :return:    a number representing the total waiting times of
                all vehicles whose ids are in the vehicle_ids list.
     It uses the function getWaitingTime described earlier
     to accumulate all waiting times of vehicles.
    """
    res = 0
    for vehicle_id in vehicle_ids:
        if conn.vehicle.getWaitingTime(vehicle_id) > 0:
            res += conn.vehicle.getWaitingTime(vehicle_id)
    return res


def get_total_co2(conn, vehicle_ids):
    """
    :param conn: The simulation connection environment
    :param vehicle_ids: Vehicle ids in the simulation
    :return:    a number representing the total CO2 emissions (in the last step)
                from all vehicles whose ids are in the vehicle_ids list.
                units: mg
    """
    res = 0
    for vehicle_id in vehicle_ids:
        res += conn.vehicle.getCOEmission(vehicle_id)
    return res


def get_total_accumulated_waiting_time(conn, vehicle_ids):
    """
    :param conn: The simulation connection environment
    :param vehicle_ids: Vehicle ids in the simulation
    :return: the sum of the accumulated waiting times of
            vehicles since they entered the simulation.
    (Note: this is different from getWaitingTime since the latter
    resets the waiting time of a car to 0 if it moves with a speed
    faster than 0.1m/s)
    """
    res = 0
    for vehicle_id in vehicle_ids:
        if conn.vehicle.getAccumulatedWaitingTime(vehicle_id) > 0:
            res += conn.vehicle.getAccumulatedWaitingTime(vehicle_id)
    return res


def get_total_speed(conn, vehicle_ids):
    """
    :param conn: The simulation connection environment
    :param vehicle_ids: Vehicle ids in the simulation
    :return: the sum of the current speeds of all cars in the simulation
    This function makes use the getSpeed(vehicle_id) which returns the speed
    in m/s of a single vehicle in the simulation within the last step (second).
    """
    res = 0
    for vehicle_id in vehicle_ids:
        res += conn.vehicle.getSpeed(vehicle_id)
    return res


# how many cars are moving (whose speed is higher than threshold making waiting time = 0)
def get_moving_count(conn, vehicle_ids):
    """
    :param conn: The simulation connection environment
    :param vehicle_ids: Vehicle ids in the simulation
    :return: the number of cars that are not stationary
            (i.e. moving with a speed larger than 0.1 m/s)
    It uses of the getWaitingTime(vehicle_id) function defined earlier.
    """
    res = 0
    for vehicle_id in vehicle_ids:
        if conn.vehicle.getWaitingTime(vehicle_id) == 0:
            res += 1
    return res


def get_state(conn, competition_round=1):
    """
    :param competition_round: current round of the competition
    :param conn: The simulation connection environment
    :return: the current state of the simulation as defined previously.
    The current state is defined as:
    state = [curr_open_dir, 8*detector(waiting times)]
    Where:
    - detector[i]: Waiting time for the vehicle on detector[i]
                    since it was last moving with speed > 0.1 m/s.
    - detector[i] for i in [0-3] is near traffic light
    - detector[i] for i in [4-7] is far from traffic light
    - For illustration of detector positions and numbering (check attached sensor_data.png)
    - curr_open_dir for COMPETITION_ROUND 1:
            (0 for vertical, 1 for horizontal) --> possible actions (0, 1)
    - curr_open_dir for COMPETITION_ROUND 2:
            (0 down, 1 left, 2 up, 3 right)    --> possible actions (0, 1, 2, 3)
    """

    result = get_curr_open_dir(conn)  # current direction
    # 8: Number of sensors
    for i in range(8):
        # get if there is a vehicle in front of the detector i
        if len(conn.inductionloop.getVehicleData(str(i))) > 0:
            # get vehicle id
            vehicle_id = conn.inductionloop.getVehicleData(str(i))[0][0]
            # get waiting time of a vehicle and append in result
            result.append(conn.vehicle.getWaitingTime(vehicle_id))
        else:
            # No vehicle --> 0 waiting time
            result.append(0)

    if competition_round == 2:
        for i in range(1, len(result)):
            if random.uniform(0, 1) < CENSOR_PROBABILITY:
                result[i] = -1

    return result


def take_action(conn, state, action, competition_round=2):
    """
    :param conn: The simulation connection environment
    :param state: state of the simulation as defined previously
    :param action: integer denoting the action taken by the agent.
            - actions for COMPETITION_ROUND 1: (0 vertical, 1 horizontal)
            - actions for COMPETITION_ROUND 2: (0 down, 1 left, 2 up, 3 right)
    :param competition_round: the current competition round (1 or 2)
    :return: (waiting, elapsed) where:
            waiting: total waiting time of all vehicles during this action
            elapsed: the time steps elapsed since this action (YELLOW_TIME+HOLD_TIME)
    """

    curr_action = state[0]
    # HOLD_TIME = the minimum time between traffic color switches.
    # A green light cannot switch to yellow unless HOLD_TIME has passed.
    # By default, HOLD_TIME = 6 seconds
    elapsed = HOLD_TIME

    # waiting: total waiting time of all vehicles during this action
    waiting = 0
    emissions = 0
    # vehicles in the simulation during this action
    vehicle_ids = conn.lane.getLastStepVehicleIDs("1i_0") \
                  + conn.lane.getLastStepVehicleIDs("2i_0") \
                  + conn.lane.getLastStepVehicleIDs("4i_0") \
                  + conn.lane.getLastStepVehicleIDs("3i_0")
    # if chosen direction is different from the current direction, switch
    if int(action) != int(curr_action):

        currentPhase = int(conn.trafficlight.getPhase("0"))
        """
        currentPhase = trafficlight phase from the connection (including the yellow)
        - currentPhase for COMPETITION_ROUND 1: (0 for vertical, 2 for horizontal)
        - currentPhase for COMPETITION_ROUND 2: (0 down, 2 left, 4 up, 6 right)
        odd currentPhase numbers are for the yellow phase of the traffic light.
        The simulation only updates the user on the open directions (even numbers)
        Possible actions (curr_open_dir) = currentPhase//2
        """
        # Switch to yellow
        nxt = (currentPhase + 1) % 8 if competition_round == 2 else (currentPhase + 1) % 4
        conn.trafficlight.setPhase("0", nxt)

        # Add switching time to the elapsed time
        elapsed += YELLOW_TIME

        # commit to switching time keeping track of waiting time
        for i in range(YELLOW_TIME):
            conn.simulationStep()  # increment the time in simulation
            waiting += get_waiting_count(conn, vehicle_ids)
            emissions += get_total_co2(conn, vehicle_ids)
        if competition_round == 2:
            nxt = action * 2
            conn.trafficlight.setPhase("0", nxt)
        # commit to hold time

    # Hold for HOLD_TIME, keeping track of waiting time
    for i in range(HOLD_TIME):
        conn.simulationStep()
        waiting += get_waiting_count(conn, vehicle_ids)
        emissions += get_total_co2(conn, vehicle_ids)
    return waiting, elapsed, emissions


# client testing framework
def run_episode(conn, agent, competition_round, train=True):
    """
    :param conn: The simulation connection environment
    :param agent: The action-taking agent object
    :param competition_round: the current competition round (1 or 2)
    :param train: train or test flag
            train is True:
                Agent's select_action method has access to 'vehicle_ids'
                vehicle_ids: a list of vehicles still in the simulation currently
            train is False:
                Agent's select_action simulates the actual testing environment
    :return: (total_waiting_time, waiting_times)
            total_waiting_time: the sum of the waiting times of all vehicles that ever existed
                                in the simulation episode
            waiting_times: a list of total_waiting_times per action
                            len(waiting_times) = # of actions taken
                            by the agent in the current episode
    """
    step = 0
    # we start with phase 2 where EW has green
    conn.trafficlight.setPhase("0", 2)
    total_waiting_time = 0
    total_emissions = 0
    state = get_state(conn, competition_round)
    waiting_times = []
    # Start simulation
    while conn.simulation.getMinExpectedNumber() > 0 and step <= MAX_STEP_COUNT:
        vehicle_ids = conn.lane.getLastStepVehicleIDs("1i_0") \
                      + conn.lane.getLastStepVehicleIDs("2i_0") \
                      + conn.lane.getLastStepVehicleIDs("4i_0") \
                      + conn.lane.getLastStepVehicleIDs("3i_0")
        if agent is not None:
            if train:
                action ,_  = agent.predict(state, conn, vehicle_ids)
            else:
                action,_ = agent.predict(state)
            if (competition_round == 2 and action not in range(0, 4)) or \
                    (competition_round == 1 and action not in range(0, 2)):
                print("Agent returned an invalid action")

            cur_waiting_time, elapsed, emissions = take_action(conn, state, action, competition_round)
            next_state = get_state(conn, competition_round)
            state = next_state
        else:
            cur_waiting_time = get_waiting_count(conn, vehicle_ids)
            emissions = get_total_co2(conn, vehicle_ids)
            elapsed = 1
            conn.simulationStep()
            next_state = get_state(conn, competition_round)
            state = next_state
        total_waiting_time += cur_waiting_time
        total_emissions += emissions
        waiting_times.append(cur_waiting_time)
        step += elapsed
    conn.close()
    return total_waiting_time, waiting_times, total_emissions