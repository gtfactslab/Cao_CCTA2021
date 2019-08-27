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
# max capacity per cell
jam_density_list = [1, 3, 2, 4, 5]
# supply/demand parameters
w_list = [-0.5, -1, -2, -2, -2]
x_jam_list = [5, 10, 8, 9, 10] #TODO: isn't this the same as jam_density_list?
v_list = [0.5, 1, 2, 2, 1]
# onramp parameters
# max flow per onramp
onramp_flow_list = [1, 0, 2, 0, 2]

# start parameters
start_list = None #[2, 4, 6]
onramp_start_list = None # [1, 2, 3]

# inputs
u = np.array([[1, 3, 6],
              [0, 0, 0],
              [1, 2, 3],
              [0, 0, 0],
              [6, 6, 6]])

sim_obj = Simulator(total_time=total_time,
                    time_step=time_step,
                    n=n,
                    jam_density_list=jam_density_list,
                    w_list=w_list,
                    x_jam_list=x_jam_list,
                    v_list=v_list,
                    onramp_flow_list=onramp_flow_list,
                    start_list=start_list,
                    onramp_start_list=onramp_start_list,
                    input_array=u)

print(sim_obj.to_dict())
sim_obj.run()