#public libraries
from cvxpy import *
import numpy as np
from scipy import sparse
import matplotlib.pyplot as plt

#our libraries
from components.controller import Controller


class CHMPC(Controller):
    def __init__(self, h, x_upper_list, x_lower_list, w_list, x_jam_list, v_list, beta_list, onramp_flow_list, input_array, modeling_horizon=11):
        # Init using params from simulation
        error = False #TODO: Put in checks for inputs

        # discretization
        self.h = h

        # x_upper and x_lower
        self.x_upper_list = x_upper_list
        self.x_lower_list = x_lower_list
        # misc params for what actual sim will aim for
        self.up_buffer = 0
        # 6 seems to work well for upper
        self.low_buffer = 0
        # 5 seems to work well for lower
        # setting both to 10 seems safe
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
        congestion_state = np.array(state[2*self.num_cells:], dtype=bool)

        # set x0 based on input state
        x0 = np.zeros(self.num_flows)
        x0[:self.num_cells] = cell_state
        x0[self.num_cells:] = onramp_state

        c0 = np.zeros(self.num_cells, dtype=bool)
        c0[:] = congestion_state

        # create supply/demand matrices based on congestion state
        # demand stays the same
        self.v_matrix = sparse.diags(self.v_list)

        # supply drastically overapproximates if uncongested, otherwise matches behavior, thus in order to dynamically change this it must be set as a variable
        self.w_matrix = sparse.diags(self.w_list[1:])
        self.supply_b_list = np.array(
             [[-1 * self.w_list[i] * self.x_jam_list[i]] for i in range(0, len(self.x_jam_list))])


        # Define problem
        f = Variable((self.num_flows, self.N))  # f represents the flow rate exiting each cell/onramp
        x = Variable((self.num_flows, self.N + 1))  # x represents the density of each cell/onramp
        u = Variable((self.num_onramps, self.N))  # u represents ratio of onramp max flow to let through

        # congestion variables
        above_x_upper = Variable((self.num_flows, self.N))
        below_x_lower = Variable((self.num_flows, self.N))
        congested = Variable((self.num_cells, self.N+1)) #initial congestion state is used as "previous" calculation for first time step

        # set up constraints
        x_init = Parameter(self.num_flows)
        x_init.value = x0

        # set initial congestion state to input congestion state
        congest_init = Parameter(self.num_cells, boolean=True)
        congest_init.value = c0

        objective = 0
        constraints = [x[:, 0] == x_init]
        constraints += [congested[:, 0] == congest_init]


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

            # for cells - now utilizing on-the-fly calculation for supply
            constraints += [f[:-self.num_onramps, k] <= x[:-self.num_onramps, k] @ self.v_matrix]


            # calculate congestion state for each cell
            for c in range(self.num_cells):
                # theoretically should equal 0 when not above x_upper and 1 when it is
                constraints += [above_x_upper[c, k] == (x[c, k] - self.x_lower_list[c])/(self.x_upper_list[c] - self.x_lower_list[c])]
                # and vice versa
                constraints += [below_x_lower[c, k] == (self.x_upper_list[c] - x[c, k])/(self.x_upper_list[c] - self.x_lower_list[c])]
                # congestion state = previous state + above_x_upper - below_c_lower, bound by 0 and 1
                constraints += [congested[c, k+1] == congested[c, k] + above_x_upper[c, k] - below_x_lower[c, k]]

                if c > 0:
                    # apply new supply constraint to previous cell
                    constraints += [f[c-1, k] <= x[c, k] * self.w_list[c] + self.supply_b_list[c] + congested[c, k+1]*10000]


        # impose non-negative constraint on x and flow, as a check
        constraints += [x >= 0, f >= 0, u >= 0]

        # onramp cannot output more cars than maximum flow rate
        constraints += [u <= 1]

        #congestion state must stay between 0 and 1
        constraints += [congested >= 0, congested <= 1]
        constraints += [above_x_upper >= -100, above_x_upper <= 100]
        constraints += [below_x_lower >= -100, below_x_lower <= 100]

        prob = Problem(Minimize(objective), constraints)
        prob.solve(verbose=True, solver=GUROBI)

        if debug:
            self.plot_results(x, f, u)

        return np.expand_dims(u.value[:, 0], axis=0).transpose() # return commands for all onramps

    def plot_results(self, x, f, u):
        # FOR DEBUGGING
        for desired_cell in range(self.num_cells):
            supplys = []
            demands = []
            flows = []
            times = []
            densities = []
            x_upper = []
            x_lower = []
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

                densities.append(x.value[desired_cell, t])
                x_upper.append(self.x_upper_list[desired_cell])
                x_lower.append(self.x_lower_list[desired_cell])

            fig, ax = plt.subplots()
            ax.plot(times, supplys, linestyle="dotted")
            ax.plot(times, demands, linestyle="dashed")
            ax.plot(times, flows)
            plt.xlabel("Time Step")
            plt.ylabel("Value (# of cars)")
            plt.title("Cell {} Flow vs. Supply/Demand".format(desired_cell + 1))
            ax.legend(['next cell supply (perceived)', 'demand', 'outflow'])

            fig, ax = plt.subplots()
            ax.plot(times, densities)
            ax.plot(times, x_upper, linestyle="dashed")
            ax.plot(times, x_lower, linestyle="dashed")
            plt.xlabel("Time Step")
            plt.ylabel("Value (# of cars)")
            plt.title("Cell {} Density vs. X Upper/Lower".format(desired_cell + 1))
            ax.legend(['density', 'x_upper', 'x_lower'])

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