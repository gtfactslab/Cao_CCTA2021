# public libraries
import numpy as np

# our libraries
from components.simulator import Simulator

# time parameters (not used in calculations)
total_time = 3600
time_step = 1

# simulator parameters
# number of cells
n = 5

# discretization factor
h = 1/100

# congestion density values per cell
x_upper_list = [400, 300, 200, 400, 400]
x_lower_list = [300, 200, 100, 300, 300]

# supply/demand parameters per cell
w_list = [-50, -100, -200, -200, -200]
x_jam_list = [500, 1000, 800, 900, 1000]
v_list = [50, 40, 70, 60, 100]  # equivalent to free flow speed @ certain density

# onramp parameters
# max flow per onramp
# if no onramp attached to cell, set flow to 0
onramp_flow_list = [100, 0, 200, 0, 200]

# start parameters (optional)
start_list = None #[2, 4, 6]
onramp_start_list = None # [1, 2, 3]

# inputs
u = np.array([[100, 300, 600, 600, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [100, 200, 300, 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [600, 600, 600, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])



sim_obj = Simulator(total_time=total_time,
                    time_step=time_step,
                    n=n,
                    h=h,
                    x_upper_list=x_upper_list,
                    x_lower_list=x_lower_list,
                    w_list=w_list,
                    x_jam_list=x_jam_list,
                    v_list=v_list,
                    onramp_flow_list=onramp_flow_list,
                    start_list=start_list,
                    onramp_start_list=onramp_start_list,
                    input_array=u)

sim_obj.run()