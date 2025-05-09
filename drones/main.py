"""Script demonstrating the joint use of simulation and control.

The simulation is run by a `CtrlAviary` environment.
The control is given by the PID implementation in `DSLPIDControl`.

Example
-------
In a terminal, run as:

    $ python pid.py

Notes
-----
The drones move, at different altitudes, along cicular trajectories
in the X-Y plane, around point (0, -.3).

"""

import argparse
import math
import os
import pdb
import random
import time
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import pybullet as p

from .control.DSLPIDControl import DSLPIDControl
from .envs.CtrlAviary import CtrlAviary
from .utils.enums import DroneModel, Physics
from .utils.Logger import Logger
from .utils.utils import str2bool, sync

DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = True
DEFAULT_RECORD_VISION = False
DEFAULT_PLOT = True
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_DURATION_SEC = 12
DEFAULT_OUTPUT_FOLDER = "results"


def run(
    drone=DEFAULT_DRONES,
    physics=DEFAULT_PHYSICS,
    gui=DEFAULT_GUI,
    record_video=DEFAULT_RECORD_VISION,
    plot=DEFAULT_PLOT,
    user_debug_gui=DEFAULT_USER_DEBUG_GUI,
    simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
    control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
    duration_sec=DEFAULT_DURATION_SEC,
    output_folder=DEFAULT_OUTPUT_FOLDER,
):
    #### Initialize the simulation #############################
    H = 0.1
    H_STEP = 0.05
    R = 0.3
    INIT_XYZS = np.array(
        [[3 * np.random.random(), 3 * np.random.random(), 2 + 2 * np.random.random()]]
    )
    INIT_RPYS = np.array([[0, 0, np.random.random() * (np.pi / 2)]])

    #### Create the environment ################################
    env = CtrlAviary(
        drone_model=drone,
        num_drones=1,
        initial_xyzs=INIT_XYZS,
        initial_rpys=INIT_RPYS,
        physics=physics,
        neighbourhood_radius=10,
        pyb_freq=simulation_freq_hz,
        ctrl_freq=control_freq_hz,
        gui=gui,
        record=record_video,
        user_debug_gui=user_debug_gui,
    )

    #### Obtain the PyBullet Client ID from the environment ####
    PYB_CLIENT = env.getPyBulletClient()

    #### Initialize the logger #################################
    logger = Logger(
        logging_freq_hz=control_freq_hz,
        num_drones=1,
        output_folder=output_folder,
    )

    #### Initialize the controller ############################
    ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)

    #### Run the simulation ####################################
    action = np.zeros((1, 4))
    START = time.time()
    for i in range(0, int(duration_sec * env.CTRL_FREQ)):
        #### Make it rain rubber ducks #############################
        # if i/env.SIM_FREQ>5 and i%10==0 and i/env.SIM_FREQ<10: p.loadURDF("duck_vhacd.urdf", [0+random.gauss(0, 0.3),-0.5+random.gauss(0, 0.3),3], p.getQuaternionFromEuler([random.randint(0,360),random.randint(0,360),random.randint(0,360)]), physicsClientId=PYB_CLIENT)

        #### Step the simulation ###################################
        obs, reward, terminated, truncated, info = env.step(action)

        #### Compute control for the current way point #############
        # TODO decide your target control values
        action[:], _, _ = ctrl.computeControlFromState(
            control_timestep=env.CTRL_TIMESTEP,
            state=obs[0],
            target_pos=np.array([0, 0, 2.1], dtype=np.single),
            target_rpy=np.array([0.0, 0.0, 0.0], dtype=np.single),
        )

        #### Log the simulation ####################################
        logger.log(
            drone=0,
            timestamp=i / env.CTRL_FREQ,
            state=obs[0],
            # TODO Decide what you want to log
            control=np.hstack(
                [
                    np.zeros(12),
                ]
            ),
        )

        #### Printout ##############################################
        env.render()

        #### Sync the simulation ###################################
        if gui:
            sync(i, START, env.CTRL_TIMESTEP)

    #### Close the environment #################################
    env.close()

    #### Save the simulation results ###########################
    logger.save()
    logger.save_as_csv("pid")  # Optional CSV save

    #### Plot the simulation results ###########################
    if plot:
        logger.plot()


if __name__ == "__main__":
    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(
        description="Helix flight script using CtrlAviary and DSLPIDControl"
    )
    parser.add_argument(
        "--drone",
        default=DEFAULT_DRONES,
        type=DroneModel,
        help="Drone model (default: CF2X)",
        metavar="",
        choices=DroneModel,
    )
    parser.add_argument(
        "--physics",
        default=DEFAULT_PHYSICS,
        type=Physics,
        help="Physics updates (default: PYB)",
        metavar="",
        choices=Physics,
    )
    parser.add_argument(
        "--gui",
        default=DEFAULT_GUI,
        type=str2bool,
        help="Whether to use PyBullet GUI (default: True)",
        metavar="",
    )
    parser.add_argument(
        "--record_video",
        default=DEFAULT_RECORD_VISION,
        type=str2bool,
        help="Whether to record a video (default: False)",
        metavar="",
    )
    parser.add_argument(
        "--plot",
        default=DEFAULT_PLOT,
        type=str2bool,
        help="Whether to plot the simulation results (default: True)",
        metavar="",
    )
    parser.add_argument(
        "--user_debug_gui",
        default=DEFAULT_USER_DEBUG_GUI,
        type=str2bool,
        help="Whether to add debug lines and parameters to the GUI (default: False)",
        metavar="",
    )
    parser.add_argument(
        "--simulation_freq_hz",
        default=DEFAULT_SIMULATION_FREQ_HZ,
        type=int,
        help="Simulation frequency in Hz (default: 240)",
        metavar="",
    )
    parser.add_argument(
        "--control_freq_hz",
        default=DEFAULT_CONTROL_FREQ_HZ,
        type=int,
        help="Control frequency in Hz (default: 48)",
        metavar="",
    )
    parser.add_argument(
        "--duration_sec",
        default=DEFAULT_DURATION_SEC,
        type=int,
        help="Duration of the simulation in seconds (default: 5)",
        metavar="",
    )
    parser.add_argument(
        "--output_folder",
        default=DEFAULT_OUTPUT_FOLDER,
        type=str,
        help='Folder where to save logs (default: "results")',
        metavar="",
    )
    ARGS = parser.parse_args()

    run(**vars(ARGS))
