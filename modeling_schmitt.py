from cvxpy import *
import numpy as np
import scipy as sp
from scipy import sparse
import matplotlib.pyplot as plt
import gurobipy

# convex relaxation setup

# constants
delta_t = 1/300
time_steps = 40
num_cells = 5 # five freeway cells, each with an attached onramp (if no onramp actually attached, set r_bar and q_bar to 0)

beta = [1, 1, 1, 1, 1] # turning rate from cell onto next cell

# reference values for
# supply
w_list = [-100, -100, -100, -100, -100]
x_jam_list = [300, 300, 300, 300, 300]
supply_b_list = [-1 * w_list[i] * x_jam_list[i] for i in range(0, len(x_jam_list))]
# demand
v_list = [100, 100, 100, 100, 10]

# variables
rho = Variable((num_cells, time_steps+1), integer=True) # represents the density in the cell
length = [1, 1, 1, 1, 1] # represents length of each cell

phi = Variable((num_cells, time_steps)) # represents outflow from each cell

q = Variable((num_cells, time_steps + 1), integer=True) # represents queue length in each onramp
r = Variable((num_cells, time_steps), integer=True) # represents outflow from each onramp

q_bar = [1500, 0, 0, 0, 0] # queue length limit (currently unused as it makes the problem unsolvable)
r_bar = [100, 0, 0, 0, 0] # onramp flow limit

omega = np.zeros((num_cells, time_steps)) # represents outside pressure to onramps
for i in range(time_steps):
    omega[0, i] = 300


# initial states
rho_0 = np.zeros(num_cells)
rho_init = Parameter(num_cells)
rho_init.value = rho_0
print(rho_0)

q_0 = np.zeros(num_cells)
q_init = Parameter(num_cells)
q_init.value = q_0

# objective function
objective = 0

# constraints
constraints = [rho[:, 0] == rho_init]
constraints += [q[:, 0] == q_init]

# iterate over time steps
for t in range(time_steps):
    for k in range(num_cells):
        # objective function
        objective += delta_t * (length[k] * rho[k, t] + q[k, t])

        # constraints

        # flow constraints between cells
        constraints += [phi[k, t] <= rho[k, t] * v_list[k]] # demand
        if k < num_cells-1:
            constraints += [beta[k] * phi[k, t] <= rho[k+1, t] * w_list[k+1] + supply_b_list[k+1]] # supply
            # last cell has access to unlimited supply

        # cell density update
        if k == 0:
            prev_cell_flow = 0 # represents flow entering first cell
        else:
            prev_cell_flow = beta[k-1] * phi[k-1, t]


        constraints += [rho[k, t + 1] == rho[k, t] + (delta_t / length[k]) *
                        (prev_cell_flow + r[k, t] - phi[k, t])]

        # onramp queue update
        # if onramp actually attached
        if q_bar[k] > 0:
            if t == 0:
                omega_term = 0
            else:
                omega_term = omega[k, t - 1]
            constraints += [q[k, t+1] == q[k, t] + delta_t * (omega_term - r[k, t])]

            # density and flow constraints for onramps
            #constraints += [q[k, t] <= q_bar[k]]
            constraints += [r[k, t] <= r_bar[k]]

            #supply and demand constraints for onramps
            constraints += [r[k, t] <= (1.0/delta_t)*q[k, t] + omega_term] # demand
            constraints += [r[k, t] <= rho[k, t] * w_list[k] + supply_b_list[k]] # supply

        # else, it remains 0
        else:
            constraints += [q[k, t + 1] == 0]
            constraints += [r[k, t] == 0]

# add non-negative constraints on applicable cells as a check
constraints += [rho >= 0]
constraints += [q >= 0]
constraints += [r >= 0] # we only apply nonnegative to onramp flow as they are controllable

prob = Problem(Minimize(objective), constraints)
prob.solve(verbose=True)
#print(prob.value)
#print(phi.value)
#print(rho.value)
#print(r.value)
desired_cell = 0
for t in range(time_steps):
    print(t)
    supply = rho.value[desired_cell+1, t] * w_list[desired_cell+1] + supply_b_list[desired_cell+1]
    print("SUPPLY: {} <= {}".format(phi.value[desired_cell, t], supply))
    demand = rho.value[desired_cell, t] * v_list[desired_cell]
    print("DEMAND: {} <= {}".format(phi.value[desired_cell, t], demand))
    print("DENSITY: {} = {} + {}/{}*({} + {} - {})".format(rho.value[desired_cell, t+1], rho.value[desired_cell, t], delta_t, length[desired_cell], 0, r.value[desired_cell, t], phi.value[desired_cell, t]))


# plot cell density per time step
fig, ax = plt.subplots()
cax = ax.matshow(np.flipud(rho.value[:, :-1]), aspect="auto")
fig.colorbar(cax)
plt.xlabel("Time Step")
plt.ylabel("Cell")
plt.title("Cell Density")

cell_ticks = ['']
[cell_ticks.append(str(num_cells - i)) for i in range(0, num_cells)]
ax.set_yticklabels(cell_ticks)
ax.tick_params(axis='x', bottom=True, top=False, labelbottom=True, labeltop=False)

fig, ax = plt.subplots()
cax = ax.matshow(np.flipud(phi.value[:, :-1]), aspect="auto")
fig.colorbar(cax)
plt.xlabel("Time Step")
plt.ylabel("Cell")
plt.title("Cell Flow")

cell_ticks = ['']
[cell_ticks.append(str(num_cells - i)) for i in range(0, num_cells)]
ax.set_yticklabels(cell_ticks)
ax.tick_params(axis='x', bottom=True, top=False, labelbottom=True, labeltop=False)

fig, ax = plt.subplots()
cax = ax.matshow(np.flipud(q.value[:, :-1]), aspect="auto")
fig.colorbar(cax)
plt.xlabel("Time Step")
plt.ylabel("On Ramp")
plt.title("On Ramp Density")

cell_ticks = ['']
[cell_ticks.append(str(num_cells - i)) for i in range(0, num_cells)]
ax.set_yticklabels(cell_ticks)
ax.tick_params(axis='x', bottom=True, top=False, labelbottom=True, labeltop=False)


fig, ax = plt.subplots()
cax = ax.matshow(np.flipud(r.value[:, :-1]), aspect="auto")
fig.colorbar(cax)
plt.xlabel("Time Step")
plt.ylabel("On Ramp")
plt.title("On Ramp Flow")

cell_ticks = ['']
[cell_ticks.append(str(num_cells - i)) for i in range(0, num_cells)]
ax.set_yticklabels(cell_ticks)
ax.tick_params(axis='x', bottom=True, top=False, labelbottom=True, labeltop=False)

plt.show()