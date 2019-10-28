#public libraries
from cvxpy import *
import numpy as np
from scipy import sparse
import matplotlib.pyplot as plt

#our libraries
from components.controller import Controller


class SMPC(Controller):
    def __init__(self, h, x_upper_list, x_lower_list, w_list, x_jam_list, v_list, beta_list, onramp_flow_list, input_array, modeling_horizon=11):
        # Init using params from simulation
        error = False #TODO: Put in checks for inputs

        # discretization
        self.h = h

        # x_upper and x_lower
        self.x_upper_list = x_upper_list
        self.x_lower_list = x_lower_list
        self.buffer = 5 # misc param for what actual sim will aim for instead of x_upper
        # TODO: make this more formal

        # supply
        self.w_list = w_list
        self.x_jam_list = x_jam_list

        # demand
        self.v_list = v_list

        # beta
        self.beta_list = beta_list

        # onramp max flows
        self.onramp_flow_list = onramp_flow_list
        self.onramp_flows = []
        for f in self.onramp_flow_list:
            if f > 0:
                self.onramp_flows.append(f)
        self.onramp_flows = sparse.diags(self.onramp_flows)

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
        self.cells_with_onramps = []
        A_top_right = []
        for o_id, o_flow in enumerate(self.onramp_flow_list):
            if o_flow > 0:
                self.num_onramps += 1
                row = [0 for i in range(self.num_cells)]
                row[o_id] = 1
                self.cells_with_onramps.append(o_id)
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

        self.N = modeling_horizon

        self.A[:-self.num_onramps, :-self.num_onramps] = self.A[:-self.num_onramps, :-self.num_onramps] * self.h

        self.num_flows = self.num_cells + self.num_onramps

        #handle input array
        if input_array.shape[0] != self.num_onramps:
            if input_array.shape[0] != self.num_cells:
                print("ERROR: input_matrix must have one row of values per cell OR one row of values per existing onramp")
                error = True
            else:
                self.input_array = input_array[self.cells_with_onramps, :]
        else:
            self.input_array = input_array

        if error:
            return None

        self.num_inputs_provided = self.input_array.shape[1]



    def compute_next_command(self, timestep, state, debug=False):
        onramp_state = state[0:self.num_cells]
        onramp_state = [onramp_state[c] for c in self.cells_with_onramps]
        cell_state = state[self.num_cells:2*self.num_cells]
        congestion_state = state[2*self.num_cells:]

        # set x0 based on input state
        x0 = np.zeros(self.num_flows)
        x0[:self.num_cells] = cell_state
        x0[self.num_cells:] = onramp_state

        # create supply/demand matrices based on congestion state
        # demand stays the same
        self.v_matrix = sparse.diags(self.v_list)

        # supply either overapproximates or underapproximates based on congestion state
        new_w_list = []
        for c in range(self.num_cells):
            if congestion_state[c] == 1:
                new_x_critical = self.x_lower_list[c]
                # if congested, underapproximate supply using x_lower as new critical point
            else:
                new_x_critical = self.x_upper_list[c] - self.buffer
                # if not, overapproximate supply using x_upper as new critical point (with a buffer to avoid cycling around congestion)

            w = (self.v_list[c] * new_x_critical) / (new_x_critical - float(self.x_jam_list[c]))
            new_w_list.append(w)

        self.w_matrix = sparse.diags(new_w_list[1:])
        self.supply_b_list = np.array(
            [[-1 * new_w_list[i] * self.x_jam_list[i]] for i in range(0, len(self.x_jam_list))])


        # Define problem
        f = Variable((self.num_flows, self.N))  # f represents the flow rate exiting each cell/onramp
        x = Variable((self.num_flows, self.N + 1))  # x represents the density of each cell/onramp
        u = Variable((self.num_onramps, self.N))  # u represents ratio of onramp max flow to let through

        # set up constraints
        x_init = Parameter(self.num_flows)
        x_init.value = x0

        objective = 0
        constraints = [x[:, 0] == x_init]

        # iterate over time steps
        for k in range(self.N):
            # objective: minimize the sum of all cell/onramp densities over time
            objective += sum(x[:, k])

            # calculating density at each time step using values from previous timestep
            # calculate inputs at timestep
            B = np.zeros(self.num_flows)
            if timestep + k < self.num_inputs_provided:
                B[-self.num_onramps:] = self.input_array[:, timestep + k]

            constraints += [x[:, k + 1] == x[:, k] + self.A @ f[:, k] + B]

            # constraints for demand & supply
            # for onramps
            constraints += [f[-self.num_onramps:, k] == self.onramp_flows @ u[:, k]] # onramp flow is controlled by u
            constraints += [f[-self.num_onramps:, k] <= x[-self.num_onramps:, k]]  # must also be lower than current density of onramp

            # for cells
            constraints += [f[:-self.num_onramps, k] <= x[:-self.num_onramps, k] @ self.v_matrix,
                            f[:-(self.num_onramps + 1), k] <= x[1:-self.num_onramps, k] @ self.w_matrix + np.squeeze(self.supply_b_list[1:])]

        # impose non-negative constraint on x and flow, as a check
        constraints += [x >= 0, f >= 0, u >= 0]
        # onramp cannot output more cars than maximum flow rate
        constraints += [u <= 1]

        prob = Problem(Minimize(objective), constraints)
        prob.solve(verbose=True, solver=GUROBI)

        if debug:
            self.plot_results(x, f, u)

        return np.expand_dims(u.value[:, 0], axis=0).transpose() # return commands for all onramps

    def plot_results(self, x, f, u):
        # FOR DEBUGGING
        desired_cell = 0
        for desired_cell in range(5):
            supplys = []
            demands = []
            flows = []
            times = []
            for t in range(self.N):
                times.append(t)

                demand = x.value[desired_cell, t] * self.v_list[desired_cell]
                demands.append(demand * self.h)

                if desired_cell < self.num_cells - 1:
                    supply = x.value[desired_cell + 1, t] * self.w_list[desired_cell + 1] + self.supply_b_list[desired_cell + 1]
                else:
                    supply = demand
                supplys.append(supply * self.h)

                flows.append(f.value[desired_cell, t] * self.h)

            fig, ax = plt.subplots()
            ax.plot(times, supplys, linestyle="dotted")
            ax.plot(times, demands, linestyle="dashed")
            ax.plot(times, flows)
            plt.xlabel("Time Step")
            plt.ylabel("Value (# of cars)")
            plt.title("Cell {} Flow vs. Supply/Demand".format(desired_cell + 1))
            ax.legend(['next cell supply', 'demand', 'outflow'])

        # plot onramp control per time step
        fig, ax = plt.subplots()
        for i in range(self.num_onramps):
            u_list = []
            for t in times:
                u_list.append(u.value[i, t])
            ax.plot(times, u_list)
        plt.xlabel("Time Step")
        plt.ylabel("Onramp Control Signal")
        plt.title("Onramp Control")
        ax.legend([str(c+1) for c in self.cells_with_onramps])

        # plot cell density per time step
        fig, ax = plt.subplots()
        cax = ax.matshow(np.flipud(x.value[:-self.num_onramps, :]), aspect="auto")
        fig.colorbar(cax)
        plt.xlabel("Time Step")
        plt.ylabel("Cell")
        plt.title("Cell Density")

        cell_ticks = ['']
        [cell_ticks.append(str(self.num_flows - i - self.num_onramps)) for i in range(0, self.num_cells)]
        ax.set_yticklabels(cell_ticks)
        ax.tick_params(axis='x', bottom=True, top=False, labelbottom=True, labeltop=False)

        fig, ax = plt.subplots()
        cax = ax.matshow(x.value[-self.num_onramps:, :], aspect="auto")
        fig.colorbar(cax)
        plt.xlabel("Time Step")
        plt.ylabel("On Ramp")
        plt.title("On Ramp Density")

        cell_ticks = ['']
        [cell_ticks.append(str(c+1)) for c in self.cells_with_onramps]
        ax.set_yticklabels(cell_ticks)
        ax.tick_params(axis='x', bottom=True, top=False, labelbottom=True, labeltop=False)
        plt.show()