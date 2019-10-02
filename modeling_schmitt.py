from cvxpy import *
import numpy as np
import scipy as sp
from scipy import sparse
import matplotlib.pyplot as plt
import gurobipy

# convex relaxation setup

# constants
delta_t = 0.5
time_steps = 40
num_cells = 6 # five freeway cells and one onramp to cell 1

beta = np.zeros((num_cells, num_cells))
for i in range(0, num_cells-1):
    beta[i+1][i] = 1
beta[0][5] = 1 # turning rate of traffic onto cell 0 from cell 5

# reference values for
# supply
w_list = [-100, -100, -100, -100, -100, -10]
x_jam_list = [600, 600, 600, 600, 600, 200]
supply_b_list = [-1 * w_list[i] * x_jam_list[i] for i in range(0, len(x_jam_list))]
# demand
v_list = [100, 100, 100, 100, 100, 100]

# variables
rho = Variable((num_cells, time_steps+1)) # represents the density in the cell
length = [1, 1, 1, 1, 1, 1] # represents length of each cell

phi = Variable((num_cells, time_steps)) # represents outflow from each cell

omega = np.zeros((num_cells, time_steps)) # represents outside pressure
for i in range(time_steps):
    omega[5, i] = 1500

# initial state
rho_0 = np.zeros(num_cells)
rho_init = Parameter(num_cells)
rho_init.value = rho_0

# objective function
objective = 0

# constraints
constraints = [rho[:, 0] == rho_init]

# iterate over time steps
for t in range(time_steps):
    for e in range(num_cells):
        # objective function
        objective += delta_t * length[e] * rho[e, t]

        # constraints
        if t == 0:
            omega_term = 0
        else:
            omega_term = omega[e, t - 1]
        constraints += [rho[e, t+1] == rho[e, t] + (delta_t/length[e]) *
                        (sum([beta[e][i] * phi[i, t] for i in range(num_cells)]) - phi[e, t] + omega_term)]
        constraints += [phi[e, t] <= rho[e, t] * v_list[e]] # demand
        if e < 5:
            constraints += [beta[e+1][e] * phi[e, t] <= rho[e, t]*w_list[e] + supply_b_list[e]] # supply
        if e == 5:
            constraints += [phi[e, t] >= 0]

# add non-negative constraints on rho as a check
constraints += [rho >= 0]

prob = Problem(Minimize(objective), constraints)
prob.solve(verbose=True)
print(prob.value)
print(phi.value)

# plot cell density per time step
fig, ax = plt.subplots()
cax = ax.matshow(np.flipud(rho.value[:-1, :-1]), aspect="auto")
fig.colorbar(cax)
plt.xlabel("Time Step")
plt.ylabel("Cell")
plt.title("Cell Density")

cell_ticks = ['']
[cell_ticks.append(str(num_cells - i - 1)) for i in range(0, num_cells-1)]
ax.set_yticklabels(cell_ticks)
ax.tick_params(axis='x', bottom=True, top=False, labelbottom=True, labeltop=False)

fig, ax = plt.subplots()
cax = ax.matshow(np.flipud(rho.value[-1:, :-1]), aspect="auto")
fig.colorbar(cax)
plt.xlabel("Time Step")
plt.ylabel("On Ramp")
plt.title("On Ramp Density")

cell_ticks = ['']
cell_ticks.append('1')
ax.set_yticklabels(cell_ticks)
ax.tick_params(axis='x', bottom=True, top=False, labelbottom=True, labeltop=False)
plt.show()