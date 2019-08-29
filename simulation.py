# public libraries
import numpy as np

# our libraries
from components.simulator import Simulator

# time parameters
total_time = 3600
time_step = 1

# simulator parameters
# number of cells
n = 5

# discretization factor
h = 1/100

# jam density values per cell
x_upper_list = [400, 300, 200, 400, 500]
x_lower_list = [300, 200, 100, 300, 400]

# supply/demand parameters
w_list = [-50, -100, -200, -200, -200]
x_jam_list = [500, 1000, 800, 900, 1000] #TODO: is this the same as x_upper_list?
v_list = [50, 100, 200, 200, 100] #equivalent to free flow speed @ certain density
# onramp parameters
# max flow per onramp
onramp_flow_list = [100, 0, 200, 0, 200]

# start parameters
start_list = None #[2, 4, 6]
onramp_start_list = None # [1, 2, 3]

# inputs
u = np.array([[100, 300, 600, 600, 0],
              [0, 0, 0, 0, 0],
              [100, 200, 300, 200, 0],
              [0, 0, 0, 0, 0],
              [600, 600, 600, 0, 0]])



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