# PHIA.py  2022-05-15
# MIT LICENSE 2020 Ewerton R. Vieira

import sys
import time
import os

from util.Connected_Comp import *
import util.PH_planning as PH_planning
import util.Stick_Simulation as Stick_Simulation
import numpy as np
from PHIA import main as main


if __name__ == '__main__':

    name_plan = os.path.splitext(os.path.basename(__file__))[0]
    main(name_plan, nu=0, h=-0.1)
    """ The choice of parameteres (nu=0 and h=-0.1) are equivalent to compute
    the distance of all obstacles to the gripper and select
    the closest obstacle to be pushed"""
