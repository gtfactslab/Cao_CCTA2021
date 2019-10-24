# public libraries
import numpy as np

# our libraries
from components.simulator import Simulator
from sample_controllers.ModelPredictiveController import MPC

# time parameters (not used in calculations)
total_time = 3600
time_step = 1

# simulator parameters
# number of cells
n = 5

# discretization factor
h = 1/120

# congestion density values per cell
x_upper_list = [150, 150, 150, 150, 150]
x_lower_list = [100, 100, 100, 100, 100]

# supply/demand parameters per cell
w_list = [-20, -20, -20, -20, -20]
x_jam_list = [320, 320, 320, 320, 320]
v_list = [60, 60, 60, 60, 5]  # equivalent to free flow speed @ certain density

# beta for each cell
beta_list = [0.75, 0.8, 0.85, 0.9, 1]

# onramp parameters
# max flow per onramp
# if no onramp attached to cell, set flow to 0
onramp_flow_list = [30, 0, 0, 30, 0]

# start parameters (optional)
start_list = None #[2, 4, 6]
onramp_start_list = None # [1, 2, 3]

# inputs
# can either provide one row per cell (populate rows corresponding to cells with no onramp attached with zeros)
# or can provide one row per attached onramp (experimental)
u = np.array([[40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40],
              [40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40]])



sim_obj = Simulator(total_time=total_time,
                    time_step=time_step,
                    n=n,
                    h=h,
                    x_upper_list=x_upper_list,
                    x_lower_list=x_lower_list,
                    w_list=w_list,
                    x_jam_list=x_jam_list,
                    v_list=v_list,
                    beta_list=beta_list,
                    onramp_flow_list=onramp_flow_list,
                    start_list=start_list,
                    onramp_start_list=onramp_start_list,
                    input_array=u)

mpcontroller = MPC(h=h,
                   w_list=w_list,
                   x_jam_list=x_jam_list,
                   v_list=v_list,
                   beta_list=beta_list,
                   onramp_flow_list=onramp_flow_list,
                   modeling_horizon=51)
sim_obj.run()