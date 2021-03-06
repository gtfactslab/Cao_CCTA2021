#public libraries
import numpy as np
import matplotlib.pyplot as plt
from time import time
import csv

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
                 beta_list,
                 onramp_flow_list,
                 start_list=None,
                 onramp_start_list=None,
                 input_array=None,
                 upstream_inflow=None):

        # checks
        starts_provided = bool(start_list)
        onramp_starts_provided = bool(onramp_start_list)
        inputs_provided = isinstance(input_array, np.ndarray)

        self.num_onramps = np.count_nonzero(onramp_flow_list)

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
        if len(beta_list) != n:
            print("ERROR: beta_list must have one value per cell")
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
        if upstream_inflow is not None:
            if not isinstance(upstream_inflow, (int, float)) and len(upstream_inflow) != input_array.shape[1]:
                print("ERROR: upstream_inflow must either be a constant or list of values with the same length as the number of input timesteps")

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
                beta=beta_list[i],
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

        #instantiate upstream inflow if provided
        self.upstream_inflow = upstream_inflow
        if self.upstream_inflow is not None:
            if isinstance(upstream_inflow, (int, float)):
                self.upstream_inflow = [upstream_inflow for i in range(0, input_array.shape[1])]


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
    def run(self, u=None, controller=None, plot_results=False, debug=False, output_csv=None):
        if bool(u):
            if u.shape[0] != self.n:
                if u.shape[0] != self.num_onramps:
                    print("ERROR: input_matrix must have one row of values per cell OR one row of values per existing onramp")
                    return 0
            self.u = self.pad_and_match_inputs(u)

        if not isinstance(self.u, np.ndarray):
            print("ERROR: no valid input matrix provided or stored")
            return 0

        cars_exited_network = 0
        self.cars_exited_per_timestep = []

        start = time()

        results = []
        results.append(self.state) # append initial state
        for t in range(0, len(self.u[0])):
            print("Calculating Time Step {}".format(t))
            prev_cars = cars_exited_network
            if debug:
                print(self.state)

            # each time step of the simulation is run in two (or three) stages
            # if a controller is provided, call the appropriate method and calculate control inputs
            if controller is not None:
                control_commands = controller.compute_next_command(t, self.state, debug)
                control_commands = self.pad_and_match_inputs(control_commands)
                if debug:
                    print(control_commands)
            # first, cycle through each cell and calculate densities for next time step

            for c in self.cell_dict:
                onramp_in = self.u[c - 1][t]
                self.cell_dict[c].calculate_next_step(onramp_in,
                                                      self.h,
                                                      float(control_commands[c - 1]) if controller is not None else None,
                                                      self.upstream_inflow[t] if c == 1 and self.upstream_inflow is not None else None)
                cars_exited_network += self.cell_dict[c].cars_leaving_network(self.h)

            diff = cars_exited_network - prev_cars

            # then, update densities for each cell
            for c in self.cell_dict:
                self.cell_dict[c].update()
            self.update_state()
            # this is done so that all the calculations are guaranteed to happen at the same time step
            # i.e. a cell's new value doesn't accidentally get factored into another cell's calculations

            # Save state
            results.append(self.state)

            # Record stats
            self.cars_exited_per_timestep.append(diff)

        print("Simulation complete in {} seconds.".format(time()-start))
        print("{} cars were able to exit the network during the simulation.".format(cars_exited_network))

        results = np.array(results)
        if plot_results:
            #print(results)
            self.plot_cars_exited_per_timestep()
            self.plot_results(results, line=True)

        if output_csv is not None:
            self.write_results_to_csv(results, output_csv)

        return results

    def write_results_to_csv(self, results, file_name):
        header = ["timestep"] \
                 + ["o{}_density".format(i) for i in range(1, self.num_cells + 1)] \
                 + ["c{}_density".format(i) for i in range(1, self.num_cells + 1)] \
                 + ["c{}_congested".format(i) for i in range(1, self.num_cells + 1)] \
                 + ["cars_exited"]
        #print(header)
        with open(file_name, 'w') as results_csv:
            results_writer = csv.writer(results_csv, delimiter=',')
            results_writer.writerow(header)
            for i, row in enumerate(results):
                if i == 0:
                    exited = 0
                else:
                    exited = self.cars_exited_per_timestep[i - 1]
                csv_row = row.tolist()
                csv_row.insert(0, i)
                csv_row.append(exited)
                results_writer.writerow(csv_row)
        print("wrote results to {}".format(file_name))

    def plot_results(self, results, line=False):
        print("Plotting Results...")
        if not line:
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
        else:
            all_densities = results[:, self.num_cells:2*self.num_cells].transpose()
            for cell in range(0, self.num_cells):
                densities = []
                times = []
                x_ups = []
                x_lows = []
                for t in range(len(self.u[0])):
                    densities.append(all_densities[cell, t])
                    times.append(t)
                    x_ups.append(self.cell_dict[cell + 1].x_upper)
                    x_lows.append(self.cell_dict[cell + 1].x_lower)

                fig, ax = plt.subplots()
                ax.plot(times, x_ups, linestyle="dashed")
                ax.plot(times, x_lows, linestyle="dashed")
                ax.scatter(times, densities)
                plt.xlabel("Time Step")
                plt.ylabel("Value (# of cars)")
                plt.title("Cell {} Density Over Time".format(cell + 1))
                ax.legend(['x_upper', 'x_lower', 'density'])



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

    def plot_cars_exited_per_timestep(self):
        times = [t for t in range(0, len(self.u[0]))]

        fig, ax = plt.subplots()
        ax.plot(times, self.cars_exited_per_timestep)
        plt.xlabel("Time Step")
        plt.ylabel("Number of Cars Exited")
        plt.title("Number of Cars Exiting Network Per Time Step")

        plt.show()

    def get_cars_exited_per_timestep(self):
        return self.cars_exited_per_timestep