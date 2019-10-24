#public libraries
from cvxpy import *
import numpy as np
from scipy import sparse
import matplotlib.pyplot as plt

#our libraries
from components.controller import Controller


class MPC(Controller):
    def __init__(self, h, w_list, x_jam_list, v_list, beta_list, onramp_flow_list, modeling_horizon=11):
        # Init using params from simulation
        # discretization
        self.h = h

        # supply
        self.w_list = w_list
        self.x_jam_list = x_jam_list

        # demand
        self.v_list = v_list

        # beta
        self.beta_list = beta_list

        # onramp max flows
        self.onramp_flow_list = onramp_flow_list

        # setup matrices for calculation
        self.num_cells = len(v_list)

        # top left of A - cell flow relationships
        A_top_left = []
        for i in range(0, self.num_cells):
            row = []
            for j in range(0, self.num_cells):
                if j == i:
                    row.append(-1)
                elif j == i-1:
                    row.append(self.beta_list[i-1])
                else:
                    row.append(0)
            A_top_left.append(row)
        A_top_left = np.array(A_top_left)

        # top right of A - onramp to cell flows
        self.num_onramps = 0
        A_top_right = []
        for o_id, o_flow in enumerate(self.onramp_flow_list):
            if o_flow > 0:
                self.num_onramps += 1
                row = [0 for i in range(self.num_cells)]
                row[o_id] = 1
                A_top_right.append(row)
        A_top_right = np.array(A_top_right).transpose()

        # bottom left of A - zero padding
        A_bottom_left = np.zeros((self.num_onramps, self.num_cells))

        # bottom right of A - offramp outflow
        A_bottom_right = -1 * np.eye(self.num_onramps)

        # start combining matrices
        A_top = np.hstack((A_top_left, A_top_right))
        A_bottom = np.hstack((A_bottom_left, A_bottom_right))

        self.A = np.vstack((A_top, A_bottom))

        self.A = sparse.csc_matrix(self.A)

        self.w_matrix = sparse.diags(self.w_list[1:])
        self.supply_b_list = np.array([[-1 * self.w_list[i] * self.x_jam_list[i]] for i in range(0, len(self.x_jam_list))])
        self.v_matrix = sparse.diags(self.v_list)

        self.N = modeling_horizon

        self.A[:-self.num_onramps, :-self.num_onramps] = self.A[:-self.num_onramps, :-self.num_onramps] * self.h

    def compute_next_command(self, timestep, state):
        #TODO: Run solver to return command for each onramp
        pass