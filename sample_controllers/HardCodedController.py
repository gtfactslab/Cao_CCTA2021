import numpy as np
from scipy import sparse

#our libraries
from components.controller import Controller

class HCC(Controller):
    def __init__(self, h, x_upper_list, x_lower_list, w_list, x_jam_list, v_list, beta_list, onramp_flow_list, input_array):
        # Init using params from simulation
        error = False #TODO: Put in checks for inputs

        # discretization
        self.h = h

        # x_upper and x_lower
        self.x_upper_list = x_upper_list
        self.x_lower_list = x_lower_list
        # misc params for what actual sim will aim for
        self.up_buffer = 1
        # 6 seems to work well for upper
        self.low_buffer = 10
        # 5 seems to work well for lower
        # setting both to 10 seems safe
        # TODO: make this more formal

        # supply
        self.w_list = w_list
        self.x_jam_list = x_jam_list
        self.supply_b_list = np.array(
            [[-1 * self.w_list[i] * self.x_jam_list[i]] for i in range(0, len(self.x_jam_list))])

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

        self.num_cells = len(v_list)

        # top right of A - onramp to cell flows
        self.num_onramps = 0
        self.cells_with_onramps = []
        for o_id, o_flow in enumerate(self.onramp_flow_list):
            if o_flow > 0:
                self.num_onramps += 1
                self.cells_with_onramps.append(o_id)


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
        cell_state = state[self.num_cells:2*self.num_cells]
        congestion_state = state[2*self.num_cells:]

        # Two step algorithm
        # 1) calculate cell states in next timestep without addition of cars from onramp
        prev_inflow = 0
        next_cell_state = cell_state.copy()
        for c in range(self.num_cells):
            cell_demand = self.v_list[c] * cell_state[c]
            if c == self.num_cells - 1 or congestion_state[c+1] == 0:
                next_supply = cell_demand
            else:
                next_supply = self.w_list[c+1] * cell_state[c+1] + self.supply_b_list[c+1]

            cell_outflow = min(cell_demand, next_supply) * self.h

            next_cell_state[c] = cell_state[c] + prev_inflow - cell_outflow

            prev_inflow = self.beta_list[c] * cell_outflow

        #print(next_cell_state)

        # 2) calculate goal densities based on congestion states
        goal_densities = []
        downstream_congested = True  # end cell should be x_upper
        for c in range(self.num_cells - 1, -1, -1):
            if downstream_congested:
                # if downstream would be congested at goal density, then get current cell at highest allowed density without becoming congested (flow is already limited by supply)
                goal_x = self.x_upper_list[c] - self.up_buffer
            else:
                # if downstream wouldn't be congested, then calculate what the ideal density for this cell would be to keep downstream at that density
                goal_x = (self.v_list[c + 1] / (self.beta_list[c] * self.v_list[c])) * goal_densities[0]

            goal_densities.insert(0, goal_x)

            downstream_congested = goal_x >= self.x_upper_list[c]

        print(goal_densities)


        # 3) calculate number of cars to insert from onramps to get cell states to desired density
        u = []
        for i, c in enumerate(self.cells_with_onramps):
            # UNCOMMENT BELOW TO SET GOALS BASED ON END CELL
            # if i < len(self.cells_with_onramps) - 1:
            #     end_cell = self.cells_with_onramps[i + 1] - 1
            # else:
            #     end_cell = self.num_cells - 1
            #
            # congested = congestion_state[end_cell]
            # if congested:
            #     goal_x = self.x_lower_list[end_cell] - self.low_buffer
            # else:
            #     goal_x = self.x_upper_list[end_cell] - self.up_buffer
            # END

            # UNCOMMENT BELOW TO SET GOAL TO GOAL DENSITY
            goal_x = goal_densities[c]
            # END

            car_diff = max(goal_x - next_cell_state[c], 0)

            command = min(car_diff / self.onramp_flow_list[c], 1.0)

            if type(command) is np.ndarray:
                command = command[0]

            u.append(command)

        u = np.array(u)[:, np.newaxis]
        print(u)
        return u



