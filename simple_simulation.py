# public libraries
import numpy as np
import matplotlib.pyplot as plt

# our libraries
from components.simulator import Simulator
from sample_controllers.ModelPredictiveController import MPC
from sample_controllers.SwitchingModelPredictiveController import SMPC
from sample_controllers.ToyMPC import TMPC
from sample_controllers.NaiveHighMPC import NHMPC

# time parameters (not used in calculations)
total_time = 3600
time_step = 1

# simulator parameters
# number of cells
n = 3

# discretization factor
h = 1/120

# congestion density values per cell
x_upper_list = [100, 100, 100]
x_lower_list = [60, 60, 60]

# supply/demand parameters per cell
w_list = [-20, -20, -20]
x_jam_list = [320, 320, 320]
v_list = [60, 60, 44]  # equivalent to free flow speed @ certain density

# beta for each cell
beta_list = [0.75, 0.75, 1]

# onramp parameters
# max flow per onramp
# if no onramp attached to cell, set flow to 0
onramp_flow_list = [40, 40, 0]
#onramp_flow_list = [80, 40, 0] increase flow to ramp 1 to act as supply

# start parameters (optional)
#start_list = [0, 150, 150] # start congested
start_list = [0, 0, 0] # start empty

onramp_start_list = None # [1, 2, 3]

# inputs
# can either provide one row per cell (populate rows corresponding to cells with no onramp attached with zeros)
# or can provide one row per attached onramp (experimental)
expected_u = np.array([[80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80],
                       [80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80]])

#expected_u = np.hstack((expected_u, expected_u)) # extend time


mpcontroller = MPC(h=h,
                   w_list=w_list,
                   x_jam_list=x_jam_list,
                   v_list=v_list,
                   beta_list=beta_list,
                   onramp_flow_list=onramp_flow_list,
                   input_array=expected_u,
                   modeling_horizon=51)


smpcontroller = SMPC(h=h,
                     x_upper_list=x_upper_list,
                     x_lower_list=x_lower_list,
                     w_list=w_list,
                     x_jam_list=x_jam_list,
                     v_list=v_list,
                     beta_list=beta_list,
                     onramp_flow_list=onramp_flow_list,
                     input_array=expected_u,
                     modeling_horizon=51)

tmpcontroller = TMPC(h=h,
                     x_upper_list=x_upper_list,
                     x_lower_list=x_lower_list,
                     w_list=w_list,
                     x_jam_list=x_jam_list,
                     v_list=v_list,
                     beta_list=beta_list,
                     onramp_flow_list=onramp_flow_list,
                     input_array=expected_u,
                     modeling_horizon=51)

nhmpcontroller = NHMPC(h=h,
                     x_upper_list=x_upper_list,
                     x_lower_list=x_lower_list,
                     w_list=w_list,
                     x_jam_list=x_jam_list,
                     v_list=v_list,
                     beta_list=beta_list,
                     onramp_flow_list=onramp_flow_list,
                     input_array=expected_u,
                     modeling_horizon=51)

#controllers = [None, mpcontroller, smpcontroller]
controllers = [nhmpcontroller]

times = [t for t in range(0, len(expected_u[0]))]

cars_exiting = []
for c in controllers:
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
    sim_obj.run(controller=c, plot_results=True, debug=False)
    cars_exiting.append(sim_obj.get_cars_exited_per_timestep())


fig, ax = plt.subplots()
plt.xlabel("Time Step")
plt.ylabel("Number of Cars Exited")
plt.title("Number of Cars Exiting Network Per Timestep")
[ax.plot(times, c) for c in cars_exiting]
ax.legend(['no control', 'mpc', 'smpc'])

print("SUMMARY")
print("NONE: {}".format(sum(cars_exiting[0])))
print("MPC:  {}".format(sum(cars_exiting[1])))
print("SMPC: {}".format(sum(cars_exiting[2])))
plt.show()
