#public libraries
import numpy as np
import matplotlib.pyplot as plt
from time import time

#our libraries
from components.cell import Cell
from components.onramp import OnRamp



# Object for defining and storing structure/parameters of simulation
class Simulator():
    def __init__(self,
                 total_time,
                 time_step,
                 n,
                 h,
                 x_upper_list,
                 x_lower_list,
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

        self.num_onramps = np.count_nonzero(onramp_flow_list)
        self.onramp_supply_mode = "ALL"

        error = False
        if len(x_upper_list) != n:
            print("ERROR: x_upper_list must have one value per cell")
            error = True
        if len(x_lower_list) != n:
            print("ERROR: x_lower_list must have one value per cell")
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
            if input_array.shape[0] != self.num_onramps:
                print("ERROR: input_matrix must have one row of values per cell OR one row of values per existing onramp")
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
        self.h = h

        # populate cell dict
        for i in range(0, self.num_cells):
            # we'll force cells to index by 1 to match existing convention
            self.cell_dict[i + 1] = Cell(
                starting_density=start_list[i] if starts_provided else 0,
                x_upper=x_upper_list[i],
                x_lower=x_lower_list[i],
                w=w_list[i],
                x_jam=x_jam_list[i],
                v=v_list[i],
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
            self.u = self.pad_and_match_inputs(input_array)

        print("Simulator Object successfully instantiated.")

    def pad_and_match_inputs(self, inputs):
        # process inputs if fewer rows than cells provided
        if inputs.shape[0] != self.num_cells:
            padded = np.empty((0, inputs.shape[1]), float)
            o = 0
            for c in self.cell_dict:
                if self.cell_dict[c].get_attached_onramp().get_max_flow_rate() != 0:
                    padded = np.vstack((padded, inputs[o, :]))
                    o = o + 1
                else:
                    padded = np.vstack((padded, np.zeros((1, inputs.shape[1]))))
            return np.array(padded)
        else:
        # if rows and cells match, then no processing needed
            return inputs


    # returns dict representation of simulation setup
    def to_dict(self):
        output_dict = {
            "num_cells": self.num_cells,
            "total_time": self.total_time,
            "time_step": self. time_step,
            "cells":{}
        }
        for c in self.cell_dict:
            output_dict["cells"][c] = self.cell_dict[c].to_dict()
        return output_dict

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
                if u.shape[0] != self.num_onramps:
                    print("ERROR: input_matrix must have one row of values per cell OR one row of values per existing onramp")
                    return 0
            self.u = self.pad_and_match_inputs(u)

        if not isinstance(self.u, np.ndarray):
            print("ERROR: no valid input matrix provided or stored")
            return 0

        start = time()

        results = []
        for t in range(0, len(self.u[0])):
            # each time step of the simulation is run in two stages
            # first, cycle through each cell and calculate densities for next time step
            o = 0
            for c in self.cell_dict:
                onramp_in = self.u[c - 1][t]
                self.cell_dict[c].calculate_next_step(onramp_in, self.h)

            # then, update densities for each cell
            for c in self.cell_dict:
                self.cell_dict[c].update()
            self.update_state()
            # this is done so that all the calculations are guaranteed to happen at the same time step
            # i.e. a cell's new value doesn't accidentally get factored into another cell's calculations

            # Save state
            results.append(self.state)

        print("Simulation complete in {} seconds.".format(time()-start))

        results = np.matrix(results)
        self.plot_results(results)
        return results

    def plot_results(self, results):
        print("Displaying Results.")
        # plot cell density per time step
        fig, ax = plt.subplots()
        cax = ax.matshow(np.flipud(results[:, self.num_cells:2*self.num_cells].transpose()), aspect="auto")
        fig.colorbar(cax)
        plt.xlabel("Time Step")
        plt.ylabel("Cell")
        plt.title("Cell Density")

        cell_ticks = ['']
        [cell_ticks.append(str(self.num_cells - i)) for i in range(0, self.num_cells)]
        ax.set_yticklabels(cell_ticks)
        ax.tick_params(axis='x', bottom=True, top=False, labelbottom=True, labeltop=False)

        # plot onramp density per time step
        fig, ax = plt.subplots()
        cax = ax.matshow(np.flipud(results[:, :self.num_cells].transpose()), aspect="auto")
        fig.colorbar(cax)
        plt.xlabel("Time Step")
        plt.ylabel("On Ramp")
        plt.title("On Ramp Density")

        cell_ticks = ['']
        [cell_ticks.append(str(self.num_cells - i)) for i in range(0, self.num_cells)]
        ax.set_yticklabels(cell_ticks)
        ax.tick_params(axis='x', bottom=True, top=False, labelbottom=True, labeltop=False)

        # plot congestion state per time step
        fig, ax = plt.subplots()
        cax = ax.matshow(np.flipud(results[:, 2 * self.num_cells:].transpose()), aspect="auto")
        fig.colorbar(cax)
        plt.xlabel("Time Step")
        plt.ylabel("Cell")
        plt.title("Congestion State")

        cell_ticks = ['']
        [cell_ticks.append(str(self.num_cells - i)) for i in range(0, self.num_cells)]
        ax.set_yticklabels(cell_ticks)
        ax.tick_params(axis='x', bottom=True, top=False, labelbottom=True, labeltop=False)

        fig, ax = plt.subplots()
        cax = ax.matshow(np.flipud(self.u), aspect="auto")
        fig.colorbar(cax)
        plt.xlabel("Time Step")
        plt.ylabel("On Ramp")
        plt.title("Input to On Ramp")

        cell_ticks = ['']
        [cell_ticks.append(str(self.num_cells - i)) for i in range(0, self.num_cells)]
        ax.set_yticklabels(cell_ticks)
        ax.tick_params(axis='x', bottom=True, top=False, labelbottom=True, labeltop=False)

        plt.show()