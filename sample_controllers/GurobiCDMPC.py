#public libraries
from gurobipy import *
import numpy as np
from scipy import sparse
import matplotlib.pyplot as plt

#our libraries
from components.controller import Controller


class GCDMPC(Controller):
    def __init__(self, h, x_upper_list, x_lower_list, w_list, x_jam_list, v_list, beta_list, onramp_flow_list, input_array, modeling_horizon=11):
        # Init using params from simulation
        error = False #TODO: Put in checks

        # discretization
        self.h = h

        # supply
        self.w_list = w_list
        self.x_jam_list = x_jam_list

        # demand
        self.v_list = v_list

        # beta
        self.beta_list = beta_list

        # congestion
        self.x_upper_list = x_upper_list
        self.x_lower_list = x_lower_list

        # onramp max flows
        self.onramp_flow_list = onramp_flow_list
        self.onramp_flows = []
        for f in self.onramp_flow_list:
            if f > 0:
                self.onramp_flows.append(f)
        #self.onramp_flows = sparse.diags(self.onramp_flows)

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

        self.w_matrix = sparse.diags(self.w_list[1:])
        self.supply_b_list = np.array([[-1 * self.w_list[i] * self.x_jam_list[i]] for i in range(0, len(self.x_jam_list))])
        self.v_matrix = sparse.diags(self.v_list)

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
        congestion_state = state[2 * self.num_cells:]

        # set x0 based on input state
        x0 = np.zeros(self.num_flows)
        x0[:self.num_cells] = cell_state
        x0[self.num_cells:] = onramp_state

        m = Model("GCDMPC")

        # Define problem
        f = m.addVars(self.num_flows, self.N, name="F")  # f represents the flow rate exiting each cell/onramp
        x = m.addVars(self.num_flows, self.N + 1, name="X")  # x represents the density of each cell/onramp
        u = m.addVars(self.num_onramps, self.N, vtype=GRB.BINARY, name="U")  # u represents ratio of onramp max flow to let through

        # congestion variables
        above_x_upper = m.addVars(self.num_cells, self.N, name="A")
        below_x_lower = m.addVars(self.num_cells, self.N, name="B")
        above_indicator = m.addVars(self.num_cells, self.N, name="AI")
        below_indicator = m.addVars(self.num_cells, self.N, name="BI")
        above_indicator_end = m.addVars(self.num_cells, self.N, name="AIE", vtype=GRB.BINARY)
        below_indicator_end = m.addVars(self.num_cells, self.N, name="BIE", vtype=GRB.BINARY)
        congested = m.addVars(self.num_cells, self.N + 1, vtype=GRB.BINARY, name="C") #congested is a binary variable stating whether the cell is congested

        one = m.addVar(name="ONE")
        zero = m.addVar(name="ZERO")

        # objective: minimize the sum of all cell/onramp densities over time
        m.setObjective(x.sum(), GRB.MINIMIZE)
        [m.addConstr(x[f, 0] == x0[f]) for f in range(self.num_flows)]
        [m.addConstr(congested[c, 0] == congestion_state[c]) for c in range(len(congestion_state))]
        m.addConstr(one == 1)
        m.addConstr(zero == 0)

        # iterate over time steps
        for k in range(self.N):
            # calculating density at each time step using values from previous timestep
            # calculate inputs at timestep
            B = np.zeros(self.num_flows)
            if timestep + k < self.num_inputs_provided:
                B[-self.num_onramps:] = self.input_array[:, timestep + k]

            #[m.addConstr(x[c, k + 1] == x[c, k] + sum([self.A[c, b] * f[b, k] for b in range(self.num_flows)]) + B) for c in range(self.num_flows)]

            # constraints for flow, demand & supply
            # for onramps
            for o in range(self.num_onramps):
                row_id = self.num_cells + o
                m.addConstr(f[row_id, k] == self.onramp_flows[o] * u[o, k]) # onramp flow is controlled by u
                m.addConstr(f[row_id, k] <= x[row_id, k])  # must also be lower than current density of onramp

                m.addConstr(x[row_id, k+1] == x[row_id, k] - f[row_id, k] + B[row_id])

                m.addConstr(x[row_id, k+1] >= 0)
                m.addConstr(f[row_id, k] >= 0)


            # for cells
            for c in range(self.num_cells):
                m.addConstr(x[c, k + 1] >= 0)
                m.addConstr(f[c, k] >= 0)

                # constraints for flow
                if c in self.cells_with_onramps:
                    onramp_id = self.cells_with_onramps.index(c)
                    onramp_flow = f[self.num_cells + onramp_id, k]
                else:
                    onramp_flow = 0

                if c > 0:
                    prev_flow = self.beta_list[c-1] * f[c-1, k] * self.h
                else:
                    prev_flow = 0

                m.addConstr(x[c, k+1] == x[c, k] - self.h * f[c, k] + onramp_flow + prev_flow)

                # constraints for congestion
                m.addConstr(above_x_upper[c, k] == x[c, k] - self.x_upper_list[c])
                m.addConstr(below_x_lower[c, k] == self.x_lower_list[c] - x[c, k])

                m.addConstr(above_indicator[c, k] == min_([above_x_upper[c, k], one]))
                m.addConstr(below_indicator[c, k] == min_([below_x_lower[c, k], one]))

                m.addConstr(above_indicator_end[c, k] == max_([above_indicator[c, k], zero]))
                m.addConstr(below_indicator_end[c, k] == max_([below_indicator[c, k], zero]))

                m.addGenConstrIndicator(above_indicator_end[c, k], True, congested[c, k + 1] == 1)
                m.addGenConstrIndicator(below_indicator_end[c, k], True, congested[c, k + 1] == 0)
                m.addGenConstrIndicator(above_indicator_end[c, k] or below_indicator_end[c, k], False, congested[c, k + 1] == congested[c, k])

                # constrain by supply and demand if next cell congested and not last cell
                # otherwise constrain by demand only
                m.addConstr(f[c, k] <= x[c, k] * self.v_list[c])
                if c < self.num_cells - 1:
                    m.addGenConstrIndicator(congested[c, k + 1], True, f[c, k] <= x[c+1, k] * self.w_list[c+1] + self.supply_b_list[c+1])





        # impose non-negative constraint on x and flow, as a check
        m.write("full_model.mps")
        m.computeIIS()
        m.write("model.ilp")
        m.optimize()

        if debug:
            self.plot_results(x, f, u)
        print(u[0, 0].x)

        return np.expand_dims(u.value[:, 0], axis=0).transpose() # return commands for all onramps

    def plot_results(self, x, f, u):
        # FOR DEBUGGING
        desired_cell = 0
        for desired_cell in range(self.num_cells):
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