#public libraries
from cvxpy import *
import numpy as np
from scipy import sparse
import matplotlib.pyplot as plt

#our libraries
from components.controller import Controller


class MPC(Controller):
    def __init__(self):
        #TODO: Init using params from simulation
        pass

    def compute_next_command(self, state):
        #TODO: Run solver to return command for each onramp
        pass