# public libraries
import numpy as np
import os, sys

# our libraries
from components.simulator import Simulator
from sample_controllers.ModelPredictiveController import MPC
from sample_controllers.HardCodedController import HCC
from sample_controllers.GurobiCDMPC import GCDMPC

total_time = 3600
time_step = 1

# simulator parameters
# number of cells
n = 8

# discretization factor
h = 1/120

# congestion density values per cell
x_upper_list = [110, 110, 110, 110, 110, 110, 110, 110]
x_lower_list = [70, 70, 70, 70, 70, 70, 70, 70]

# supply/demand parameters per cell
w_list = [-20, -20, -20, -20, -20, -20, -20, -20]
x_jam_list = [320, 320, 320, 320, 320, 320, 320, 320]
v_list =  [60, 60, 60, 60, 60, 60, 60, 60]  # equivalent to free flow speed @ certain density

# beta for each cell
beta_list = [0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 1]

# onramp parameters
# max flow per onramp
# if no onramp attached to cell, set flow to 0
onramp_flow_list = [60, 0, 60, 0, 60, 0, 60, 0]

# start parameters (optional)
start_list = None #[2, 4, 6]
start_list = [150, 150, 150, 150, 150, 150, 150, 150] #[2, 4, 6]
onramp_start_list = None # [1, 2, 3]

# inputs
# can either provide one row per cell (populate rows corresponding to cells with no onramp attached with zeros)
# or can provide one row per attached onramp (experimental)
expected_u = np.array([[80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80],
                     [80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80],
                     [80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80],
                     [80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80]
                     ])

expected_u = np.hstack((expected_u, expected_u))

mpcontroller = MPC(h=h,
                   w_list=w_list,
                   x_jam_list=x_jam_list,
                   v_list=v_list,
                   beta_list=beta_list,
                   onramp_flow_list=onramp_flow_list,
                   input_array=expected_u,
                   modeling_horizon=51)
hcc = HCC(h=h,
                     x_upper_list=x_upper_list,
                     x_lower_list=x_lower_list,
                     w_list=w_list,
                     x_jam_list=x_jam_list,
                     v_list=v_list,
                     beta_list=beta_list,
                     onramp_flow_list=onramp_flow_list,
                     input_array=expected_u)

gcdmpcontroller = GCDMPC(h=h,
                     x_upper_list=x_upper_list,
                     x_lower_list=x_lower_list,
                     w_list=w_list,
                     x_jam_list=x_jam_list,
                     v_list=v_list,
                     beta_list=beta_list,
                     onramp_flow_list=onramp_flow_list,
                     input_array=expected_u,
                     modeling_horizon=11,
                     time_limit=3600,
                     control_memory=10) # horizon 16 if starting empty, 41 + cutoff early for congested

controllers = [("None", None),
               ("MPC", mpcontroller),
               ("HCC", hcc),
               ("GCDMPC",gcdmpcontroller)
               ]

times = [t for t in range(0, len(expected_u[0]))]

cars_exiting = []
controllers_run = []
results_dir = sys.argv[1]
os.makedirs("results/{}".format(results_dir))
for (name, c) in controllers:
    controllers_run.append(name)
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
                        input_array=expected_u)
    sim_obj.run(controller=c, plot_results=False, debug=False, output_csv="results/{}/{}_results.csv".format(results_dir, name))
    cars_exiting.append(sim_obj.get_cars_exited_per_timestep())

print("SUMMARY")
[print("{}:\t{}".format(controllers_run[i], sum(cars_exiting[i]))) for i in range(len(controllers_run))]

