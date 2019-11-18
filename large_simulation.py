# public libraries
import numpy as np
import matplotlib.pyplot as plt

# our libraries
from components.simulator import Simulator
from sample_controllers.ModelPredictiveController import MPC
from sample_controllers.SwitchingModelPredictiveController import SMPC
from sample_controllers.SwitchingMPC2 import SMPC2
from sample_controllers.HardCodedController import HCC

# time parameters (not used in calculations)
total_time = 3600
time_step = 1

# simulator parameters
# number of cells
n = 10

# discretization factor
h = 1/120

# congestion density values per cell
x_upper_list = [100, 100, 100, 100, 100, 100, 100, 100, 100, 120]
x_lower_list = [60, 60, 60, 60, 60, 60, 60, 60, 60, 80]

# supply/demand parameters per cell
w_list = [-20, -20, -20, -20, -20, -20, -20, -20, -20, -20]
x_jam_list = [320, 320, 320, 320, 320, 320, 320, 320, 320, 320]
v_list = [60, 60, 60, 60, 60, 60, 60, 60, 60, 45]  # equivalent to free flow speed @ certain density

# beta for each cell
beta_list = [0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 1]

# onramp parameters
# max flow per onramp
# if no onramp attached to cell, set flow to 0
onramp_flow_list = [60, 0, 0, 60, 0, 0, 60, 0, 0, 0]

# start parameters (optional)
start_list = None #[2, 4, 6]
onramp_start_list = None # [1, 2, 3]

# inputs
# can either provide one row per cell (populate rows corresponding to cells with no onramp attached with zeros)
# or can provide one row per attached onramp (experimental)
expected_u = np.array([[40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40],
                       [40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40],
                       [40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40]])

actual_u = np.array([[80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80],
                     [80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80],
                     [80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80]])

actual_u = np.hstack((actual_u, actual_u))

mpcontroller = MPC(h=h,
                   w_list=w_list,
                   x_jam_list=x_jam_list,
                   v_list=v_list,
                   beta_list=beta_list,
                   onramp_flow_list=onramp_flow_list,
                   input_array=actual_u,
                   modeling_horizon=51)


smpcontroller = SMPC(h=h,
                     x_upper_list=x_upper_list,
                     x_lower_list=x_lower_list,
                     w_list=w_list,
                     x_jam_list=x_jam_list,
                     v_list=v_list,
                     beta_list=beta_list,
                     onramp_flow_list=onramp_flow_list,
                     input_array=actual_u,
                     modeling_horizon=51)

smpcontroller2 = SMPC2(h=h,
                     x_upper_list=x_upper_list,
                     x_lower_list=x_lower_list,
                     w_list=w_list,
                     x_jam_list=x_jam_list,
                     v_list=v_list,
                     beta_list=beta_list,
                     onramp_flow_list=onramp_flow_list,
                     input_array=actual_u,
                     modeling_horizon=51)

hcc = HCC(h=h,
                     x_upper_list=x_upper_list,
                     x_lower_list=x_lower_list,
                     w_list=w_list,
                     x_jam_list=x_jam_list,
                     v_list=v_list,
                     beta_list=beta_list,
                     onramp_flow_list=onramp_flow_list,
                     input_array=actual_u)

controllers = [("None", None),
               ("MPC", mpcontroller),
               ("SMPC", smpcontroller),
               ("hcc", hcc)]
controllers = [("hcc", hcc),
               ("MPC", mpcontroller),
               ("SMPC", smpcontroller)]


times = [t for t in range(0, len(actual_u[0]))]

cars_exiting = []
controllers_run = []
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
                        input_array=actual_u)
    sim_obj.run(controller=c, plot_results=True, debug=False)
    cars_exiting.append(sim_obj.get_cars_exited_per_timestep())


fig, ax = plt.subplots()
plt.xlabel("Time Step")
plt.ylabel("Number of Cars Exited")
plt.title("Number of Cars Exiting Network Per Timestep")
[ax.plot(times, c) for c in cars_exiting]
ax.legend(controllers_run)

print("SUMMARY")
[print("{}:\t{}".format(controllers_run[i], sum(cars_exiting[i]))) for i in range(len(controllers_run))]

plt.show()





