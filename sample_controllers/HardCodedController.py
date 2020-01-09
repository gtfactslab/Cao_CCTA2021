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
        self.low_buffer = 1
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
            # supply constraint must account for beta term as not all cars progress to next cell, this does not change the actual supply function
            cell_outflow = min(cell_demand, next_supply/self.beta_list[c]) * self.h

            next_cell_state[c] = cell_state[c] + prev_inflow - cell_outflow

            prev_inflow = self.beta_list[c] * cell_outflow

        #print(next_cell_state)

        # 2) calculate ideal goal densities
        ideal_densities = []
        cell_types = []
        cell_type = 'A'
        next_cell_type = None
        for c in range(self.num_cells - 1, -1, -1):
            cell_types.insert(0, cell_type)
            if cell_type == 'A':
                ideal_x = self.x_upper_list[c] - self.up_buffer
                next_cell_type = 'B'
            elif cell_type == 'B':
                ideal_x = (self.v_list[c + 1] / (self.beta_list[c] * self.v_list[c])) * ideal_densities[0]
                if ideal_x >= self.x_upper_list[c]:
                    next_cell_type = 'C'
                else:
                    next_cell_type = 'B'
            elif cell_type == 'C':
                ideal_x = self.x_upper_list[c] - self.up_buffer
                next_cell_type = 'D'
            elif cell_type == 'D':
                ideal_x = (self.w_list[c + 2] * (ideal_densities[1] - self.x_jam_list[c + 2]))/(self.beta_list[c] * self.v_list[c])
                if ideal_x >= self.x_upper_list[c]:
                    next_cell_type = 'C'
                else:
                    next_cell_type = 'B'


            ideal_densities.insert(0, ideal_x)

            cell_type = next_cell_type

        print(ideal_densities)

        # 3) calculate goal densities based on congestion state
        goal_densities = ideal_densities
        congested_cell = None
        congested_care = []
        for c in range(self.num_cells - 1, 0, -1): # we don't care if the first cell is congested, there's no inflow limitation
            ideal = ideal_densities[c]
            if congestion_state[c] == 1:
                # if congested, check to see if we care
                if ideal <= self.x_upper_list[c]:
                    # if this cell isn't meant to be congested, we might care
                    # check to see if this cell being congested affects previous cell
                    w = self.w_list[c]
                    xj = self.x_jam_list[c]
                    v_prev = self.v_list[c-1]

                    x_crossover = (-1 * w * xj) / (v_prev - w)
                    print(x_crossover)
                    if ideal_densities[c-1] >= x_crossover:
                        # if the previous cell is affected, now we care and calculate new densities for all preceding cells
                        # these densities can be overwritten if there is another congested cell further up the chain
                        congested_cell = c
                        congested_care.append(c)
                        goal_densities[congested_cell] = self.x_lower_list[congested_cell] - self.low_buffer
                        for nc in range(congested_cell - 1, -1, -1):
                            goal_densities[nc] = (self.v_list[nc + 1] * goal_densities[nc + 1]) / (
                                        self.beta_list[nc] * self.v_list[nc])



        # 4) calculate number of cars to insert from onramps to get cell states to desired density
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

            car_diff = goal_x - next_cell_state[c]
            # print(cell_types)
            # if cell is feeding into a cell whose outflow is limited by supply, that cell can be fed up to its x_upper
            if cell_types[c] == 'D' and c not in congested_care and c+1 not in self.cells_with_onramps:
                gap = max(goal_densities[c + 1] - next_cell_state[c + 1], 0)
                car_diff = min(car_diff + (gap/self.beta_list[c]), self.x_upper_list[c] - self.up_buffer - next_cell_state[c])

            command = min(max(car_diff, 0) / self.onramp_flow_list[c], 1.0)

            if type(command) is np.ndarray:
                command = command[0]

            u.append(command)

        u = np.array(u)[:, np.newaxis]
        print(u)
        return u



