from cvxpy import *
import numpy as np
import scipy as sp
from scipy import sparse
import matplotlib.pyplot as plt
import gurobipy

# overall problem formulation is x[k+1] = x[k] + A * f[k] + B
# where f[k] is a vector [f_1, f_2, ... f_n, f_1r, ..., f_nr]' which represent the outflow from each cell/onramp respectively
A = sparse.csc_matrix([
  [-1.,      0.,     0., 0., 0., 1],
  [1.,      -1.,     0., 0., 0., 0],
  [0.,      1.,     -1., 0., 0., 0],
  [0.,       0.,    1., -1., 0., 0],
  [0.,      0,      0., 1., -1., 0],
  [0.,      0,      0., 0., 0., -1]])

# represents inflow that doesn't come from other cells (aka flow from outside the system)
B_1 = np.array([
  [0.],
  [0.],
  [0.],
  [0.],
  [0.],
  [150.]]) # in this case, every timestep adds 150 vehicles to the onramp

B_2 = np.array([
  [0.],
  [0.],
  [0.],
  [0.],
  [0.],
  [0.]]) # after a certain point, we want much fewer vehicles to enter
[r, c] = A.shape

#represents ratio of cars that actually progress to next cell
betas = sparse.diags([0.5, 0.5, 0.5, 0.5, 0.5, 1])

# reference values for
# supply
w_list = [-200, -200, -200, -200, -200]
w_matrix = sparse.diags(w_list[1:])
x_jam_list = [600, 600, 600, 600, 600]
supply_b_list = np.array([[-1 * w_list[i] * x_jam_list[i]] for i in range(0, len(x_jam_list))])
# demand
v_list = [100, 100, 100, 100, 10]
v_matrix = sparse.diags(v_list)

# modeling horizon
N = 41

# discretization factor
h = 1/100
A[:-1, :-1] = A[:-1, :-1] * h

# initial state
x0 = np.zeros(c)

# Define problem
f = Variable((c, N), integer=True) # f represents the flow rate exiting each cell/onramp
x = Variable((c, N+1), integer=True) # x represents the density of each cell/onramp

x_init = Parameter(c)
x_init.value = x0

objective = 0
constraints = [x[:,0] == x_init]
# iterate over time steps
for k in range(N):
    # objective: minimize the sum of all cell/onramp densities over time
    objective += sum(x[:-1,k])

    # calculating density at each time step using values from previous timestep
    if k < 41:
        constraints += [x[:,k+1] == x[:, k] + betas @ A @ f[:, k] + np.squeeze(B_1)]
    else:
        constraints += [x[:, k + 1] == x[:, k] + betas @ A @ f[:, k] + np.squeeze(B_2)]
    # constraints for demand & supply
    constraints += [f[-1, k] == 100] # last flow, f_1r in this case, has consistent outflow (aka no control)
    constraints += [f[:-1,k] <= x[:-1,k] @ v_matrix,
                    f[:-2,k] <= x[1:-1,k] @ w_matrix + np.squeeze(supply_b_list[1:])]
    # for i in range(r - 1):
    #     #constraints += [f[i, k] <= x[i, k] * v_list[i]]
    #     if i < r - 2:
    #     #    constraints += [f[i, k] <= x[i+1, k] * w_list[i+1] + supply_b_list[i+1]]
    #         constraints += [f[i, k] == cvxpy.minimum(x[i, k] * v_list[i], x[i+1, k] * w_list[i+1] + supply_b_list[i+1])]
    #     else:
    #         constraints += [f[i, k] == x[i, k] * v_list[i]]
# impose non-negative constraint on x and flow, as a check
constraints += [x >= 0, f >= 0]

prob = Problem(Minimize(objective), constraints)
#prob = Problem(Maximize(objective), constraints)#EXPERIMENT: see what happens if we try to maximize flow instead of minimize density

prob.solve(verbose=True, solver=GUROBI)
print(prob.value)
print(f.value)
#print(x.value)

#FOR DEBUGGING
desired_cell = 0
for desired_cell in range(5):
    supplys = []
    demands = []
    flows = []
    times = []
    for t in range(N):
        times.append(t)


        demand = x.value[desired_cell, t] * v_list[desired_cell]
        demands.append(demand)

        if desired_cell < r - 2:
            supply = x.value[desired_cell+1, t] * w_list[desired_cell+1] + supply_b_list[desired_cell+1]
        else:
            supply = demand
        supplys.append(supply)

        flows.append(f.value[desired_cell, t])


    fig, ax = plt.subplots()
    ax.plot(times, supplys, linestyle="dotted")
    ax.plot(times, demands, linestyle="dashed")
    ax.plot(times, flows)
    plt.xlabel("Time Step")
    plt.ylabel("Value")
    plt.title("Cell {} Flow vs. Supply/Demand".format(desired_cell + 1))
    ax.legend(['next cell supply', 'demand', 'outflow'])


# plot cell density per time step
fig, ax = plt.subplots()
cax = ax.matshow(np.flipud(x.value[:-1, :]), aspect="auto")
fig.colorbar(cax)
plt.xlabel("Time Step")
plt.ylabel("Cell")
plt.title("Cell Density")

cell_ticks = ['']
[cell_ticks.append(str(c - i - 1)) for i in range(0, c-1)]
ax.set_yticklabels(cell_ticks)
ax.tick_params(axis='x', bottom=True, top=False, labelbottom=True, labeltop=False)

fig, ax = plt.subplots()
cax = ax.matshow(np.flipud(x.value[-1:, :]), aspect="auto")
fig.colorbar(cax)
plt.xlabel("Time Step")
plt.ylabel("On Ramp")
plt.title("On Ramp Density")

cell_ticks = ['']
cell_ticks.append('1')
ax.set_yticklabels(cell_ticks)
ax.tick_params(axis='x', bottom=True, top=False, labelbottom=True, labeltop=False)
plt.show()