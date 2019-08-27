#public libraries
import numpy as np

#our libraries
from components.cell import Cell
from components.onramp import OnRamp


# Object for defining and storing structure/parameters of simulation
class Simulator():
    def __init__(self,
                 total_time,
                 time_step,
                 n,
                 capacity_list,
                 w_list,
                 x_jam_list,
                 v_list,
                 onramp_flow_list,
                 start_list=None,
                 onramp_start_list=None,
                 input_array=None):

        # checks
        starts_provided = bool(start_list)
        onramp_starts_provided = bool(onramp_start_list)
        inputs_provided = isinstance(input_array, np.ndarray)

        error = False
        if len(capacity_list) != n:
            print("ERROR: capacity_list must have one value per cell")
            error = True
        if len(w_list) != n:
            print("ERROR: w_list must have one value per cell")
            error = True
        if len(x_jam_list) != n:
            print("ERROR: x_jam_list must have one value per cell")
            error = True
        if len(v_list) != n:
            print("ERROR: v_list must have one value per cell")
            error = True
        if len(onramp_flow_list) != n:
            print("ERROR: onramp_flow_list must have one value per cell (if no onramp attached, set value to 0)")
            error = True
        if starts_provided and len(start_list) != n:
            print("ERROR: start_list must have one value per cell")
            error = True
        if onramp_starts_provided and len(onramp_start_list) != n:
            print("ERROR: onramp_start_list must have one value per cell (if no onramp attached, set value to 0)")
            error = True
        if inputs_provided and input_array.shape[0] != n:
            print("ERROR: input_matrix must have one row of values per cell")
            error = True

        if error:
            return None

        # if setup passes, create simulator object
        # time
        self.total_time = total_time
        self.time_step = time_step

        # cells
        self.num_cells = n
        self.cell_dict = {}

        # populate cell dict
        for i in range(0, self.num_cells):
            # we'll force cells to index by 1 to match existing convention
            self.cell_dict[i + 1] = Cell(
                starting_density=start_list[i] if starts_provided else 0,
                jam_density=capacity_list[i],
                w=w_list[i],
                x_jam=x_jam_list[i],
                v=v_list,
                attached_onramp=OnRamp(
                    starting_density=onramp_start_list[i] if onramp_starts_provided else 0,
                    max_flow_rate=onramp_flow_list[i] # if no onramp attached, set to 0 for equivalent behavior
                )
            )

        # if more than one cell, link cells together
        if n > 1:
            self.cell_dict[1].downstream = self.cell_dict[2]
            self.cell_dict[n].upstream = self.cell_dict[n - 1]
            for i in range(2, n):
                self.cell_dict[i].downstream = self.cell_dict[i + 1]
                self.cell_dict[i].upstream = self.cell_dict[i - 1]


        #state tracking, populate with initial state
        self.state = []
        self.update_state()

        #set input if provided
        self.u = None
        if inputs_provided:
            self.u = input_array



    # returns dict representation of simulation setup
    def to_dict(self):
        return

    # updates current state vector with values from cells/onramps
    def update_state(self):
        onramp_state = []
        cell_state = []
        congestion_state = []
        for i in self.cell_dict:
            cell_obj = self.cell_dict[i]
            cell_state.append(cell_obj.get_current_density())
            onramp_state.append(cell_obj.get_attached_onramp().get_current_density())
            congestion_state.append(1 if cell_obj.is_congested() else 0)
        self.state = []
        self.state.extend(onramp_state)
        self.state.extend(cell_state)
        self.state.extend(congestion_state)
        return self.state


    # runs simulation based on current parameters
    def run(self, u=None):
        if bool(u):
            if u.shape[0] != self.n:
                print("ERROR: input_matrix must have one row of values per cell (if no on")
                return 0
            else:
                self.u = u

        if not isinstance(self.u, np.ndarray):
            print("ERROR: no input matrix provided or stored")
            return 0

        for t in range(0, len(self.u[0])):
            for c in self.cell_dict:
                self.cell_dict[c].update(self.u[c - 1][t])
            self.update_state()
            print(self.state)
