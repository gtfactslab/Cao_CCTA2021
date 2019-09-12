from cvxpy import *
import numpy as np
import scipy as sp
from scipy import sparse

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
B = np.array([
  [0.],
  [0.],
  [0.],
  [0.],
  [0.],
  [150.]]) # in this case, every timestep adds 150 vehicles to the onramp
[r, c] = A.shape

# reference values for
# supply
w_list = [-100, -100, -100, -100, -100]
w_matrix = sparse.diags(w_list[1:])
x_jam_list = np.array([[600], [600], [600], [600], [600]])
# demand
v_list = sparse.diags([100, 100, 100, 100, 0])

# modeling horizon
N = 10

# initial state
x0 = np.zeros(c)

# Define problem
f = Variable((c, N)) # f represents the flow rate exiting each cell/onramp
x = Variable((c, N+1)) # x represents the density of each cell/onramp

x_init = Parameter(c)
f_init = Parameter(c)

objective = 0
constraints = [x[:,0] == x_init, f[:,0] == f_init]
# iterate over time steps
for k in range(N):
    # objective: minimize the sum of all cell/onramp densities over time
    objective += sum(x[:,k])
    # calculating density at each time step using values from previous timestep
    constraints += [x[:,k+1] == x[:, k] + A @ f[:, k] + np.squeeze(B)]
    # constraints for demand & supply
    constraints += [f[-1, k] == 100] # last flow, f_1r in this case, has consistent outflow
    constraints += [f[:-1,k] <= x[:-1,k] @ v_list,
                    f[:-2,k] <= x[1:-1,k] @ w_matrix + np.squeeze(x_jam_list[1:])]
prob = Problem(Minimize(objective), constraints)

# Simulate
prob.solve()
print(prob.value)
print(f.value)
print(x.value)