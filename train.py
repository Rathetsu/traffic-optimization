import yaml
import argparse
from src.training import training
import os
import sys
import traci
import sumolib

if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    sys.exit("Please declare the environment variable 'SUMO_HOME'")


parser = argparse.ArgumentParser(description='Start training')
parser.add_argument("-c", "--config", type=str, help="Path to the config file")

if __name__ == '__main__':
    args = vars(parser.parse_args())

    with open(args['config'], 'r') as ymfile:
        cfg = yaml.full_load(ymfile)

    print(cfg)
    training(cfg)